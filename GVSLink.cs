using System;
using System.Diagnostics;
using System.Threading;
using UnityEngine;
using Debug = UnityEngine.Debug;

/// <summary>
/// The GVS streaming engine, shared by GVSController (the game) and Demo (the test bench),
/// so the bench exercises exactly the same sending code as the game.
///
/// How it sends:
///   * START handshake (with retries), then a dedicated thread sends a DATA packet every tick
///     at SendRateHz (heartbeat), whether or not the value changed.
///   * The main thread calls SetTarget() every frame. If the target is older than
///     StaleTimeoutMs (pause, hitch, disabled object) the output ramps to zero.
///   * Output is slew-limited on the SIGNED value, so polarity changes pass through zero.
///   * Hard amplitude cap applied right before the packet is built.
///   * Consecutive failures mark the link lost; on recovery START is re-sent.
///   * Stop() ends streaming and sends STOP.
///
/// Put this file in the same folder as BLE.cs so it is compiled wherever BLE is.
/// </summary>
public sealed class GVSLink
{
    // Absolute ceiling on commanded current (uA). Deliberately a constant, not a setting.
    public const int HardMaxAmplitudeMicroAmps = 3000;

    public const byte CMD_STOP = 0;
    public const byte CMD_START = 1;
    public const byte CMD_DATA = 2;

    /// <summary>10-byte packet, little-endian, matches a #pragma pack(1) struct on the ESP32.</summary>
    public struct GVSPacket
    {
        public const int Size = 10;

        public byte command;     // 0 = STOP, 1 = START, 2 = DATA
        public byte direction;   // 1 / 0 (polarity)
        public int frequency;    // unit defined by firmware
        public int amplitude;    // uA

        public bool SameAs(GVSPacket o)
        {
            return command == o.command && direction == o.direction &&
                   frequency == o.frequency && amplitude == o.amplitude;
        }

        public void WriteToBuffer(byte[] buf)
        {
            buf[0] = command;
            buf[1] = direction;
            buf[2] = (byte)(frequency & 0xFF);
            buf[3] = (byte)((frequency >> 8) & 0xFF);
            buf[4] = (byte)((frequency >> 16) & 0xFF);
            buf[5] = (byte)((frequency >> 24) & 0xFF);
            buf[6] = (byte)(amplitude & 0xFF);
            buf[7] = (byte)((amplitude >> 8) & 0xFF);
            buf[8] = (byte)((amplitude >> 16) & 0xFF);
            buf[9] = (byte)((amplitude >> 24) & 0xFF);
        }
    }

    // ---------------- Settings: change any time from the main thread; read live by the sender ----------------
    public volatile float SendRateHz = 100f;
    public volatile int StaleTimeoutMs = 100;
    public volatile float SlewRateMicroAmpsPerSec = 10000f;
    public volatile int MaxAmplitudeMicroAmps = HardMaxAmplitudeMicroAmps;
    public volatile int Frequency = 0;
    public volatile int LinkLossFailureThreshold = 10;
    public volatile int WriteStallTimeoutMs = 10000;
    /// <summary>When true the output ramps to zero (packets keep flowing as a heartbeat).</summary>
    public volatile bool Muted = false;
    /// <summary>Pop incoming notifications from the DLL queue (~10 Hz). Turn off if something else reads them.</summary>
    public volatile bool DrainNotifications = true;

    // ---------------- Telemetry (written by the sender thread) ----------------
    public volatile float MeasuredSendRateHz;
    public volatile int CurrentOutputAmplitude;
    public volatile int CurrentOutputDirection;
    public volatile int ConsecutiveWriteFailures;
    public volatile int NotificationsReceived;
    public bool IsStreaming { get { return Volatile.Read(ref state) == StateStreaming; } }
    public bool LinkLost { get { return linkLost; } }

    private const int StateIdle = 0, StateStarting = 1, StateStreaming = 2;
    private int state = StateIdle;

    private readonly string log;
    private volatile string deviceId, serviceUuid, characteristicUuid;
    private volatile bool linkLost;
    private readonly ManualResetEventSlim stopEvent = new ManualResetEventSlim(false);
    private volatile Thread senderThread;
    private int generation;
    private long writeInFlightSince;  // Stopwatch ticks; 0 = no write in progress
    private int abandonedSenders;
    private const int MaxAbandonedSenders = 3;

    private readonly object targetLock = new object();
    private int targetSignedUa;
    private long targetStamp;         // Stopwatch ticks; 0 = never set

    public GVSLink(string logPrefix = "[GVS]")
    {
        log = logPrefix + " ";
    }

    /// <summary>
    /// Signed target current in uA (positive = direction 1). Call every frame while streaming;
    /// a target not refreshed within StaleTimeoutMs is treated as zero.
    /// </summary>
    public void SetTarget(int signedMicroAmps)
    {
        lock (targetLock)
        {
            targetSignedUa = signedMicroAmps;
            targetStamp = Stopwatch.GetTimestamp();
        }
    }

    /// <summary>
    /// Sends START and begins streaming. BLOCKING (native write): call from a worker thread.
    /// Returns true if streaming is running.
    /// </summary>
    public bool Start(string devId, string svcUuid, string charUuid)
    {
        int prev = Interlocked.CompareExchange(ref state, StateStarting, StateIdle);
        if (prev != StateIdle) return prev == StateStreaming;

        deviceId = devId;
        serviceUuid = svcUuid;
        characteristicUuid = charUuid;
        linkLost = false;
        ConsecutiveWriteFailures = 0;
        abandonedSenders = 0;
        stopEvent.Reset();

        if (!SendCommand(CMD_START, 3))
        {
            Debug.LogError(log + "START handshake write failed. DLL: " + BLE.GetError());
            Volatile.Write(ref state, StateIdle);
            return false;
        }

        // If Stop() ran while the handshake was in flight, don't start streaming.
        if (Interlocked.CompareExchange(ref state, StateStreaming, StateStarting) != StateStarting)
            return false;
        StartSender(false);
        Debug.Log(log + "Streaming started at up to " + SendRateHz + " Hz.");
        return true;
    }

    /// <summary>
    /// Stops streaming and sends STOP. BLOCKING: call from a worker thread.
    /// Returns true if no native write is still in flight (i.e. it is safe to call BLE Quit()).
    /// </summary>
    public bool Stop(int joinTimeoutMs = 1000)
    {
        Volatile.Write(ref state, StateIdle);
        Interlocked.Increment(ref generation);
        stopEvent.Set();

        Thread s = senderThread;
        bool stopped = s == null || !s.IsAlive || s.Join(joinTimeoutMs);

        if (!string.IsNullOrEmpty(deviceId))
        {
            bool ok = false;
            try { ok = SendCommand(CMD_STOP, 2); } catch { }
            if (ok) Debug.Log(log + "STOP queued (write-without-response: delivery is not acknowledged).");
            else Debug.LogWarning(log + "STOP write failed; relying on firmware watchdog/disconnect. DLL: " + BLE.GetError());
            Thread.Sleep(100); // let Windows put the STOP on air before anything closes the link
        }

        CurrentOutputAmplitude = 0;
        MeasuredSendRateHz = 0f;
        return stopped && abandonedSenders == 0;
    }

    /// <summary>
    /// Call from Update() on the main thread. Recovers from the DLL's rare "SendData never
    /// returns" hang by abandoning the stuck sender and starting a new one. While stuck, no
    /// packets flow, so the ESP32 watchdog zeroes the output.
    /// </summary>
    public void CheckForHungWrite()
    {
        if (!IsStreaming) return;
        long since = Interlocked.Read(ref writeInFlightSince);
        if (since == 0) return;

        double ms = (Stopwatch.GetTimestamp() - since) * 1000.0 / Stopwatch.Frequency;
        if (ms < WriteStallTimeoutMs) return;

        Interlocked.Exchange(ref writeInFlightSince, 0);
        abandonedSenders++;
        if (abandonedSenders > MaxAbandonedSenders)
        {
            Interlocked.Increment(ref generation);
            Volatile.Write(ref state, StateIdle);
            Debug.LogError(log + "Native BLE write hung again (" + abandonedSenders + " times). Streaming stopped; restart the app.");
        }
        else
        {
            Debug.LogError(log + "Native BLE write blocked for " + ms.ToString("F0") + " ms. Abandoning that sender thread and starting a new one.");
            StartSender(true);
        }
    }

    // =====================================================================

    private bool SendCommand(byte command, int attempts)
    {
        BLE.PacketWriter writer = new BLE.PacketWriter(deviceId, serviceUuid, characteristicUuid);
        byte[] buf = new byte[GVSPacket.Size];
        new GVSPacket { command = command }.WriteToBuffer(buf); // all other fields zero
        for (int i = 0; i < attempts; i++)
        {
            if (writer.Write(buf, GVSPacket.Size)) return true;
            if (i + 1 < attempts) Thread.Sleep(100);
        }
        return false;
    }

    private void StartSender(bool rearmFirst)
    {
        int gen = Interlocked.Increment(ref generation);
        Thread t = new Thread(() => SenderLoop(gen, rearmFirst))
        {
            IsBackground = true,
            Name = log + "Sender #" + gen,
            Priority = System.Threading.ThreadPriority.AboveNormal
        };
        senderThread = t;
        t.Start();
    }

    private void SenderLoop(int myGen, bool rearmFirst)
    {
        BLE.PacketWriter writer = new BLE.PacketWriter(deviceId, serviceUuid, characteristicUuid);
        byte[] buf = new byte[GVSPacket.Size];
        long freq = Stopwatch.Frequency;
        long next = Stopwatch.GetTimestamp();
        long lastTick = next;
        long rateWindowStart = next;
        int sentInWindow = 0;
        int ticks = 0;
        float slewed = 0f;          // signed uA actually commanded
        bool needsArm = rearmFirst;

        while (IsStreaming && myGen == Volatile.Read(ref generation))
        {
            // ---- Deadline scheduling (no drift; no burst after falling behind) ----
            float rate = Mathf.Clamp(SendRateHz, 5f, 100f);
            if (linkLost) rate = 2f; // while the link is down, probe slowly
            long interval = (long)(freq / rate);
            next += interval;
            long now = Stopwatch.GetTimestamp();
            if (now - next > interval) next = now;
            int waitMs = (int)((next - now) * 1000 / freq);
            if (waitMs > 0 && stopEvent.Wait(waitMs)) break;
            if (!IsStreaming || myGen != Volatile.Read(ref generation)) break;

            now = Stopwatch.GetTimestamp();
            float dt = Mathf.Min((float)(now - lastTick) / freq, 0.05f);
            lastTick = now;

            // ---- Target: zero when stale, muted or link down ----
            int target;
            long stamp;
            lock (targetLock)
            {
                target = targetSignedUa;
                stamp = targetStamp;
            }
            bool stale = stamp == 0 || (now - stamp) * 1000 / freq > StaleTimeoutMs;
            if (stale || Muted || linkLost) target = 0;

            int cap = Mathf.Clamp(MaxAmplitudeMicroAmps, 0, HardMaxAmplitudeMicroAmps);
            target = Mathf.Clamp(target, -cap, cap);

            // ---- Slew limit on the SIGNED value: polarity changes pass through zero ----
            float maxStep = Mathf.Max(500f, SlewRateMicroAmpsPerSec) * dt;
            slewed = Mathf.MoveTowards(slewed, target, maxStep);

            int amp = Mathf.Clamp(Mathf.RoundToInt(Mathf.Abs(slewed)), 0, HardMaxAmplitudeMicroAmps);
            byte dir = (byte)(slewed > 0f ? 1 : 0);

            try
            {
                if (needsArm)
                {
                    // Re-arm after link recovery or sender replacement (the ESP32 may have rebooted).
                    new GVSPacket { command = CMD_START }.WriteToBuffer(buf);
                    if (TrackedWrite(writer, buf, myGen)) needsArm = false;
                    else { OnWriteFailed(ref slewed); continue; }
                }

                new GVSPacket { command = CMD_DATA, direction = dir, frequency = Frequency, amplitude = amp }.WriteToBuffer(buf);
                bool ok = TrackedWrite(writer, buf, myGen);
                if (myGen != Volatile.Read(ref generation)) break; // abandoned while blocked

                if (ok)
                {
                    if (linkLost)
                    {
                        linkLost = false;
                        needsArm = true;
                        Debug.Log(log + "BLE link recovered; re-arming.");
                    }
                    ConsecutiveWriteFailures = 0;
                    CurrentOutputAmplitude = amp;
                    CurrentOutputDirection = dir;
                    sentInWindow++;
                }
                else
                {
                    OnWriteFailed(ref slewed);
                }
            }
            catch (Exception ex)
            {
                if (IsStreaming) Debug.LogWarning(log + "Write exception: " + ex.Message);
                OnWriteFailed(ref slewed);
            }

            // ---- Drain incoming notifications (~10 Hz) so the DLL's queue can't grow forever ----
            if (DrainNotifications && ++ticks % 10 == 0)
            {
                try { NotificationsReceived += BLE.DrainNotifications(16); } catch { }
            }

            // ---- Measured rate ----
            double windowSec = (double)(now - rateWindowStart) / freq;
            if (windowSec >= 1.0)
            {
                MeasuredSendRateHz = (float)(sentInWindow / windowSec);
                sentInWindow = 0;
                rateWindowStart = now;
            }
        }
    }

    private bool TrackedWrite(BLE.PacketWriter writer, byte[] buf, int myGen)
    {
        Interlocked.Exchange(ref writeInFlightSince, Stopwatch.GetTimestamp());
        try
        {
            return writer.Write(buf, GVSPacket.Size);
        }
        finally
        {
            if (myGen == Volatile.Read(ref generation))
                Interlocked.Exchange(ref writeInFlightSince, 0);
        }
    }

    private void OnWriteFailed(ref float slewed)
    {
        ConsecutiveWriteFailures++;
        if (ConsecutiveWriteFailures == 1 && IsStreaming)
        {
            // Note: this DLL reports failed WRITES as "Error subscribing to characteristic ...".
            Debug.LogWarning(log + "BLE write failed. DLL: " + BLE.GetError());
        }
        if (!linkLost && ConsecutiveWriteFailures >= LinkLossFailureThreshold)
        {
            linkLost = true;
            slewed = 0f; // the ESP32 watchdog will have zeroed the output; restart the ramp from zero
            if (IsStreaming)
                Debug.LogError(log + "BLE link LOST after " + ConsecutiveWriteFailures + " failed writes. Retrying at 2 Hz. DLL: " + BLE.GetError());
        }
    }
}