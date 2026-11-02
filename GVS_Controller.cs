using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class GVSController : MonoBehaviour
{
    [Header("Tracking References")]
    public Transform motionSource;
    private Rigidbody motionRb;
    public Transform headTransform;

    [Header("BLE Configuration")]
    public string targetDeviceName = "ESP32-Brocoli";
    public string deviceId = "BluetoothLE#BluetoothLE0c:dd:24:f8:a7:40-20:50:0d:d9:81:92";
    public string serviceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
    public string notifyCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
    public string characteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";

    [Tooltip("Maximum transmission rate in Hz. Writes are blocking, so the real rate is limited by the BLE link.")]
    [Range(10f, 100f)] public float sendRateHz = 30f;

    [Header("GVS Motion Mapping (Roll Axis)")]
    public float accelDeadzone = 0.5f;
    public float maxExpectedAccel = 15.0f;
    public bool useSmoothing = true;
    [Range(5f, 100f)] public float smoothingSpeed = 50f;
    public bool invertDirection = false;

    [Header("Hardware Limits")]
    public int maxAmplitudeMicroAmps = 3000;
    public int defaultFrequencyMHz = 0;

    [Header("Debug Telemetry (Read-Only)")]
    public float rawLateralAccel;
    public float smoothedLateralAccel;
    public int currentOutputAmplitude;
    public byte currentOutputDirection;
    public bool isBleReady = false;

    public const byte CMD_STOP = 0;
    public const byte CMD_START = 1;
    public const byte CMD_DATA = 2;

    public struct GVSPacket
    {
        public byte command;
        public byte direction;
        public int frequency;
        public int amplitude;

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

    private BLE ble;
    private BLE.BLEScan activeScan;
    private Thread bleConnectThread;
    private Thread bleSenderThread;
    private readonly ManualResetEventSlim shutdownEvent = new ManualResetEventSlim(false);

    private Vector3 lastMotionPosition;
    private Vector3 lastMotionVelocity;
    private int warmupTicks = 0;

    private readonly object packetLock = new object();
    private GVSPacket latestPacket;
    private bool hasNewPacketToSend = false;

    private volatile bool isConnectedAndReady = false;
    private volatile bool isRunning = true;
    private int shutdownStarted = 0;
    private int sendIntervalMs = 33;

    void Awake()
    {
        if (sendRateHz > 50f)
        {
            float targetFixedDt = 1f / sendRateHz;
            if (Time.fixedDeltaTime > targetFixedDt)
                Time.fixedDeltaTime = targetFixedDt;
        }
    }

    void Start()
    {
        if (motionSource == null) motionSource = this.transform;
        motionRb = motionSource.GetComponentInParent<Rigidbody>();

        if (headTransform == null)
            headTransform = (Camera.main != null) ? Camera.main.transform : motionSource;

        ResetKinematicsState();

        if (!string.IsNullOrEmpty(deviceId) || !string.IsNullOrEmpty(targetDeviceName))
        {
            InitComm();
        }
    }

    public void ResetKinematicsState()
    {
        warmupTicks = 0;
        rawLateralAccel = 0f;
        smoothedLateralAccel = 0f;
        if (motionSource != null)
        {
            lastMotionPosition = motionSource.position;
            Vector3 evalPos = (headTransform != null) ? headTransform.position : motionSource.position;
            lastMotionVelocity = (motionRb != null && !motionRb.isKinematic)
                ? motionRb.GetPointVelocity(evalPos)
                : Vector3.zero;
        }
    }

    public void InitComm()
    {
        if (ble == null) ble = new BLE();

        Interlocked.Exchange(ref shutdownStarted, 0);
        isRunning = true;
        shutdownEvent.Reset();
        sendIntervalMs = Mathf.Max(10, Mathf.RoundToInt(1000f / sendRateHz));

        bleConnectThread = new Thread(BleConnectionRoutine) { IsBackground = true };
        bleConnectThread.Start();
    }

    private void BleConnectionRoutine()
    {
        try
        {
            // Fallback scan if device ID is not hardcoded
            if (string.IsNullOrEmpty(deviceId))
            {
                Debug.Log("[GVS] Scanning for: " + targetDeviceName);
                using (AutoResetEvent foundSignal = new AutoResetEvent(false))
                {
                    string discoveredId = null;

                    activeScan = BLE.ScanDevices();
                    activeScan.Found = (id, name) =>
                    {
                        if (!string.IsNullOrEmpty(name) && name.IndexOf(targetDeviceName, StringComparison.OrdinalIgnoreCase) >= 0)
                        {
                            discoveredId = id;
                            activeScan.Cancel();
                            foundSignal.Set();
                        }
                    };

                    foundSignal.WaitOne(8000);
                    if (activeScan != null) activeScan.Cancel();

                    if (!isRunning) return;

                    if (string.IsNullOrEmpty(discoveredId))
                    {
                        Debug.LogError("[GVS] Could not find device: " + targetDeviceName);
                        return;
                    }
                    deviceId = discoveredId;
                }
            }

            if (!isRunning) return;

            // Discover Services
            List<string> services = BLE.ScanAndGetServices(deviceId);
            string matchedService = MatchUuid(serviceUuid, services);
            if (string.IsNullOrEmpty(matchedService))
            {
                Debug.LogError("[GVS] Service UUID not found: " + serviceUuid);
                return;
            }
            serviceUuid = matchedService;

            if (!isRunning) return;

            // Discover Characteristics
            Dictionary<string, string> charMap = BLE.ScanAndGetCharacteristics(deviceId, serviceUuid);
            string matchedWriteChar = MatchUuid(characteristicUuid, charMap.Values);
            if (string.IsNullOrEmpty(matchedWriteChar))
            {
                Debug.LogError("[GVS] Write Characteristic not found: " + characteristicUuid);
                return;
            }
            characteristicUuid = matchedWriteChar;

            string matchedNotifyChar = MatchUuid(notifyCharacteristicUuid, charMap.Values);
            notifyCharacteristicUuid = !string.IsNullOrEmpty(matchedNotifyChar) ? matchedNotifyChar : matchedWriteChar;

            if (!isRunning) return;

            Debug.Log($"[GVS] Native connecting to: {notifyCharacteristicUuid}");
            bool connected = ble.Connect(deviceId, serviceUuid, new string[] { notifyCharacteristicUuid });
            if (!connected)
            {
                Debug.LogError("[GVS] ble.Connect returned false.");
                return;
            }

            Thread.Sleep(200);
            if (!isRunning) return;

            // Handshake (blocking: returns once Windows confirms the write)
            byte[] initBuf = new byte[10];
            new GVSPacket { command = CMD_START }.WriteToBuffer(initBuf);
            BLE.WritePackage(deviceId, serviceUuid, characteristicUuid, initBuf, true);

            if (!isRunning) return;

            isConnectedAndReady = true;
            Debug.Log("[GVS] Pipeline Active! Streaming on dedicated thread.");

            bleSenderThread = new Thread(BleSenderLoop) { IsBackground = true };
            bleSenderThread.Start();
        }
        catch (Exception e)
        {
            if (isRunning)
            {
                Debug.LogError("[GVS] BLE Init Exception: " + e.Message);
            }
        }
    }

    // All writes are BLOCKING. The loop can never get ahead of the BLE link,
    // so there is no queue of pending native operations, and the packet we send
    // is always the freshest one.
    private void BleSenderLoop()
    {
        byte[] workerBuffer = new byte[10];
        GVSPacket lastSent = default;
        bool hasLastSent = false;

        while (isRunning && isConnectedAndReady)
        {
            if (shutdownEvent.Wait(sendIntervalMs))
                break;

            GVSPacket packetToSend = default;
            bool shouldSend = false;

            lock (packetLock)
            {
                if (hasNewPacketToSend)
                {
                    packetToSend = latestPacket;
                    hasNewPacketToSend = false;
                    shouldSend = true;
                }
            }

            if (!shouldSend || !isRunning) continue;

            // Don't resend identical data (e.g. zero amplitude while idle)
            if (hasLastSent && packetToSend.SameAs(lastSent)) continue;

            try
            {
                packetToSend.WriteToBuffer(workerBuffer);
                bool ok = BLE.WritePackage(deviceId, serviceUuid, characteristicUuid, workerBuffer, true);
                if (ok)
                {
                    lastSent = packetToSend;
                    hasLastSent = true;
                }
            }
            catch (Exception ex)
            {
                if (isRunning)
                {
                    Debug.LogWarning("[GVS] Write hiccup: " + ex.Message);
                }
            }
        }
    }

    void FixedUpdate()
    {
        if (motionSource == null) return;
        if (motionRb == null) motionRb = motionSource.GetComponentInParent<Rigidbody>();

        if (motionRb != null && !motionRb.isKinematic)
        {
            Vector3 evalPos = (headTransform != null) ? headTransform.position : motionSource.position;
            Vector3 currentVel = motionRb.GetPointVelocity(evalPos);
            StepKinematics(currentVel, Time.fixedDeltaTime);
        }
    }

    void LateUpdate()
    {
        if (motionSource == null) return;

        if (motionRb == null || motionRb.isKinematic)
        {
            float dt = Time.deltaTime;
            if (dt <= 1e-6f) return;

            if (warmupTicks == 0)
            {
                lastMotionPosition = motionSource.position;
                warmupTicks = 1;
                return;
            }

            Vector3 currentVel = (motionSource.position - lastMotionPosition) / dt;
            lastMotionPosition = motionSource.position;

            StepKinematics(currentVel, dt);
        }
    }

    private void StepKinematics(Vector3 currentWorldVelocity, float dt)
    {
        if (dt <= 1e-6f) return;

        isBleReady = isConnectedAndReady;

        if (warmupTicks < 2)
        {
            lastMotionVelocity = currentWorldVelocity;
            warmupTicks++;
            return;
        }

        Vector3 worldAcceleration = (currentWorldVelocity - lastMotionVelocity) / dt;
        lastMotionVelocity = currentWorldVelocity;

        Transform orientation = (headTransform != null) ? headTransform : motionSource;
        rawLateralAccel = Vector3.Dot(worldAcceleration, orientation.right);

        if (useSmoothing)
        {
            float alpha = 1f - Mathf.Exp(-smoothingSpeed * dt);
            smoothedLateralAccel = Mathf.Lerp(smoothedLateralAccel, rawLateralAccel, alpha);
        }
        else
        {
            smoothedLateralAccel = rawLateralAccel;
        }

        if (isConnectedAndReady)
        {
            GVSPacket pkt = ProcessMovement(smoothedLateralAccel);
            lock (packetLock)
            {
                latestPacket = pkt;
                hasNewPacketToSend = true;
            }
        }
    }

    public GVSPacket ProcessMovement(float lateralAccel)
    {
        float absAccel = Mathf.Abs(lateralAccel);

        if (absAccel < accelDeadzone)
        {
            currentOutputAmplitude = 0;
            currentOutputDirection = 0;
            return new GVSPacket
            {
                command = CMD_DATA,
                direction = 0,
                frequency = defaultFrequencyMHz,
                amplitude = 0
            };
        }

        float normalized = Mathf.InverseLerp(accelDeadzone, maxExpectedAccel, absAccel);
        int targetAmplitude = Mathf.Clamp(Mathf.RoundToInt(normalized * maxAmplitudeMicroAmps), 0, maxAmplitudeMicroAmps);

        bool isRightward = lateralAccel > 0f;
        if (invertDirection) isRightward = !isRightward;

        byte dirBit = (byte)(isRightward ? 1 : 0);

        currentOutputAmplitude = targetAmplitude;
        currentOutputDirection = dirBit;

        return new GVSPacket
        {
            command = CMD_DATA,
            direction = dirBit,
            frequency = defaultFrequencyMHz,
            amplitude = targetAmplitude
        };
    }

    private static string CleanUuid(string uuid)
    {
        return string.IsNullOrEmpty(uuid) ? "" : uuid.Trim().Trim('{', '}').ToLowerInvariant();
    }

    private static string MatchUuid(string target, IEnumerable<string> candidates)
    {
        string cleanTarget = CleanUuid(target);
        foreach (string c in candidates)
        {
            if (CleanUuid(c) == cleanTarget)
                return c;
        }
        return null;
    }

    // ---------------------------------------------------------------
    // Shutdown: strictly ordered, runs once.
    //   1. stop producing / sending
    //   2. WAIT until the sender thread is out of native code
    //   3. send STOP (blocking, confirmed)
    //   4. stop connect thread
    //   5. only then call native Quit()
    // ---------------------------------------------------------------
    void OnApplicationQuit() { ShutdownBle(); }
    void OnDestroy() { ShutdownBle(); }

    private void ShutdownBle()
    {// hi
        if (Interlocked.Exchange(ref shutdownStarted, 1) == 1) return;
        Debug.Log("[GVS] Shutdown: begin");

        isRunning = false;
        isConnectedAndReady = false;
        shutdownEvent.Set();

        // Everything that touches native code runs on a worker thread, so the
        // Unity main thread can never be frozen by the DLL. The main thread
        // waits at most 3 s, then moves on no matter what.
        BLE bleToClose = ble;
        ble = null;
        string dev = deviceId, svc = serviceUuid, chr = characteristicUuid;
        Thread sender = bleSenderThread;
        Thread connector = bleConnectThread;
        BLE.BLEScan scan = activeScan;

        Thread teardown = new Thread(() =>
        {
            try
            {
                try { if (scan != null) scan.Cancel(); } catch { }

                bool senderStopped = true;
                if (sender != null && sender.IsAlive)
                    senderStopped = sender.Join(1500);
                Debug.Log("[GVS] Shutdown: sender stopped = " + senderStopped);

                if (senderStopped && bleToClose != null && bleToClose.isConnected && !string.IsNullOrEmpty(dev))
                {
                    byte[] stopBuf = new byte[10];
                    new GVSPacket { command = CMD_STOP }.WriteToBuffer(stopBuf);
                    BLE.WritePackage(dev, svc, chr, stopBuf, true);
                    Debug.Log("[GVS] Shutdown: STOP write confirmed");
                }

                if (connector != null && connector.IsAlive) connector.Join(500);

                Debug.Log("[GVS] Shutdown: calling native Quit()");
                if (bleToClose != null) bleToClose.Close();
                Debug.Log("[GVS] Shutdown: native Quit() returned - BLE cleanly closed");
            }
            catch (Exception e)
            {
                Debug.LogWarning("[GVS] Shutdown exception: " + e.Message);
            }
        })
        { IsBackground = true };

        teardown.Start();

        if (!teardown.Join(3000))
            Debug.LogWarning("[GVS] Shutdown: native teardown still blocked after 3 s - abandoning it so Unity can exit.");
    }
}