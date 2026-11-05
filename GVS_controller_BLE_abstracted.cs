using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.Serialization;

/// <summary>
/// Turns virtual motion into roll-axis GVS targets and streams them to the ESP32.
/// All sending is done by GVSLink (shared with the Demo test bench), so the bench
/// and the game use exactly the same sending code.
/// </summary>
public class GVSController : MonoBehaviour
{
    public const int HardMaxAmplitudeMicroAmps = GVSLink.HardMaxAmplitudeMicroAmps;
    public const byte CMD_STOP = GVSLink.CMD_STOP;
    public const byte CMD_START = GVSLink.CMD_START;
    public const byte CMD_DATA = GVSLink.CMD_DATA;

    // Auto: non-kinematic Rigidbody -> RigidbodyVelocity, kinematic Rigidbody -> FixedUpdatePosition,
    //       no Rigidbody -> LateUpdatePosition.
    public enum SamplingMode
    {
        Auto,
        RigidbodyVelocity,
        FixedUpdatePosition,
        LateUpdatePosition
    }

    [Header("Tracking References")]
    [Tooltip("Vehicle / rig root whose VIRTUAL motion the player sees. Must NOT be the HMD camera: real head motion is already felt by the inner ear and must not drive stimulation.")]
    public Transform motionSource;
    [Tooltip("Used only for orientation (which way is 'right' for the player's head) and as the point where Rigidbody velocity is evaluated.")]
    public Transform headTransform;
    [Tooltip("How motion is sampled. Sample in the same loop that moves the object, or velocity aliases between 0 and 2x and acceleration becomes noise.")]
    public SamplingMode samplingMode = SamplingMode.Auto;

    [Header("BLE Configuration")]
    [Tooltip("Exact advertised name (case-insensitive). Used for scanning only when Target Mac Address is empty.")]
    public string targetDeviceName = "ESP32-Brocoli";
    [Tooltip("Optional WinRT device ID. If set, scanning is skipped (it must still end with Target Mac Address if that is set).")]
    public string deviceId; // e.g. "BluetoothLE#BluetoothLE0c:dd:24:f8:a7:40-20:50:0d:d9:81:92"
    public string serviceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
    public string notifyCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
    public string characteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";
    [Tooltip("If set, ONLY a device whose ID ends with this MAC is accepted. Strongly recommended whenever more than one stimulator exists.")]
    public string targetMacAddress = "20:50:0d:d9:81:92";
    [Range(2f, 30f)] public float scanTimeoutSeconds = 8f;

    [Header("Streaming (shared GVSLink settings)")]
    [Tooltip("Packets per second. Every tick is sent (heartbeat). Check Measured Send Rate Hz for what the link really achieves.")]
    [Range(20f, 100f)] public float sendRateHz = 100f;
    [Tooltip("If the motion target has not been refreshed for this long (pause, disabled object, main-thread hitch), the output ramps to zero.")]
    [Range(30, 500)] public int staleTimeoutMs = 100;
    [Tooltip("Consecutive failed writes before the link is declared lost.")]
    [Range(1, 100)] public int linkLossFailureThreshold = 10;
    [Tooltip("A single native write blocked longer than this is treated as hung (known DLL issue): that sender thread is abandoned and replaced.")]
    [Range(2000, 30000)] public int writeStallTimeoutMs = 10000;
    [Tooltip("Ramp output to zero while the application does not have focus (e.g. headset dashboard open).")]
    public bool zeroOutputWhenUnfocused = true;

    [Header("GVS Motion Mapping (Roll Axis)")]
    public float accelDeadzone = 0.5f;
    public float maxExpectedAccel = 15.0f;
    [Tooltip("World accelerations above this (m/s^2) are treated as teleport / respawn / snap-turn artifacts and ignored.")]
    public float spikeRejectAccel = 50f;
    [Tooltip("Frame-to-frame speeds above this (m/s) are treated as teleports in the position-sampling modes.")]
    public float teleportSpeedThreshold = 50f;
    public bool useSmoothing = true;
    [Range(5f, 100f)] public float smoothingSpeed = 50f;
    [Tooltip("Optional. Adds (yaw rate in rad/s x gain) to the lateral signal so turning on the spot also produces GVS. 0 = off (linear acceleration only).")]
    public float yawRateGain = 0f;
    [Tooltip("Yaw rates above this (deg/s) are treated as snap turns and ignored.")]
    public float maxYawRateDegPerSec = 360f;
    public bool invertDirection = false;

    [Header("Hardware Limits")]
    [Range(0, HardMaxAmplitudeMicroAmps)] public int maxAmplitudeMicroAmps = 3000;
    [Tooltip("Maximum rate of change of the commanded current (uA per second). Polarity reversals ramp through zero. 10000 = 300 ms from 0 to 3 mA.")]
    public float slewRateMicroAmpsPerSec = 10000f;
    [FormerlySerializedAs("defaultFrequencyMHz")]
    [Tooltip("Written verbatim to the packet's frequency field. The unit is whatever the ESP32 firmware defines (0 = DC).")]
    public int stimulationFrequency = 0;

    [Header("Debug Telemetry (Read-Only)")]
    public float rawLateralAccel;
    public float smoothedLateralAccel;
    [Tooltip("Signed target before slew limiting (+ = direction 1).")]
    public int targetAmplitudeSigned;
    [Tooltip("Amplitude actually in the last packet sent.")]
    public int currentOutputAmplitude;
    public byte currentOutputDirection;
    public bool isBleReady = false;
    public float measuredSendRateHz;
    public int consecutiveWriteFailures;
    public int rejectedSpikes;
    public int notificationsReceived;

    // ---------------- BLE state ----------------
    private readonly GVSLink link = new GVSLink("[GVS]");
    private BLE ble;
    private BLE.BLEScan activeScan;
    private Thread bleConnectThread;
    private volatile bool isRunning = false;
    private int shutdownStarted = 0;
    private bool outputEnabled = true;
    private bool appPaused = false;
    private bool appUnfocused = false;

    // ---------------- Kinematics state (main thread only) ----------------
    private Rigidbody motionRb;
    private SamplingMode activeMode;
    private Vector3 lastMotionPosition;
    private Quaternion lastMotionRotation = Quaternion.identity;
    private Vector3 lastMotionVelocity;
    private bool hasLastPosition;
    private bool hasLastVelocity;
    private bool hasLastRotation;

    // =====================================================================
    // Unity lifecycle
    // =====================================================================

    void OnValidate()
    {
        maxAmplitudeMicroAmps = Mathf.Clamp(maxAmplitudeMicroAmps, 0, HardMaxAmplitudeMicroAmps);
        accelDeadzone = Mathf.Max(0f, accelDeadzone);
        maxExpectedAccel = Mathf.Max(accelDeadzone + 0.01f, maxExpectedAccel);
        spikeRejectAccel = Mathf.Max(maxExpectedAccel * 1.5f, spikeRejectAccel);
        teleportSpeedThreshold = Mathf.Max(1f, teleportSpeedThreshold);
        maxYawRateDegPerSec = Mathf.Max(1f, maxYawRateDegPerSec);
        slewRateMicroAmpsPerSec = Mathf.Clamp(slewRateMicroAmpsPerSec, 500f, 100000f);
    }

    void Start()
    {
        OnValidate(); // also enforce limits on values set from code / old scenes
        PushLinkSettings();

        if (motionSource == null) motionSource = this.transform;
        if (headTransform == null)
            headTransform = (Camera.main != null) ? Camera.main.transform : motionSource;

        motionRb = motionSource.GetComponentInParent<Rigidbody>();
        activeMode = ResolveSamplingMode(true);
        WarnAboutSetup();

        ResetKinematicsState();

        if (!string.IsNullOrEmpty(deviceId) || !string.IsNullOrEmpty(targetDeviceName) || !string.IsNullOrEmpty(targetMacAddress))
        {
            InitComm();
        }
    }

    void OnEnable()
    {
        outputEnabled = true;
        UpdateMute();
        // Re-warm so the time spent disabled does not show up as a velocity/acceleration jump.
        hasLastPosition = hasLastVelocity = hasLastRotation = false;
    }

    void OnDisable()
    {
        // FixedUpdate/LateUpdate stop running while disabled; ramp to zero now rather than
        // waiting for the target to go stale.
        outputEnabled = false;
        UpdateMute();
    }

    void OnApplicationPause(bool paused) { appPaused = paused; UpdateMute(); }
    void OnApplicationFocus(bool hasFocus) { appUnfocused = !hasFocus; UpdateMute(); }

    void Update()
    {
        PushLinkSettings();
        UpdateMute();
        link.CheckForHungWrite();

        isBleReady = link.IsStreaming && !link.LinkLost;
        currentOutputAmplitude = link.CurrentOutputAmplitude;
        currentOutputDirection = (byte)link.CurrentOutputDirection;
        measuredSendRateHz = link.MeasuredSendRateHz;
        consecutiveWriteFailures = link.ConsecutiveWriteFailures;
        notificationsReceived = link.NotificationsReceived;
    }

    private void PushLinkSettings()
    {
        link.SendRateHz = sendRateHz;
        link.StaleTimeoutMs = staleTimeoutMs;
        link.SlewRateMicroAmpsPerSec = slewRateMicroAmpsPerSec;
        link.MaxAmplitudeMicroAmps = maxAmplitudeMicroAmps;
        link.Frequency = stimulationFrequency;
        link.LinkLossFailureThreshold = linkLossFailureThreshold;
        link.WriteStallTimeoutMs = writeStallTimeoutMs;
    }

    private void UpdateMute()
    {
        link.Muted = !outputEnabled || appPaused || (zeroOutputWhenUnfocused && appUnfocused);
    }

    void FixedUpdate()
    {
        if (motionSource == null) return;
        RefreshSamplingMode();

        float dt = Time.fixedDeltaTime;
        if (dt <= 1e-6f) return;

        if (activeMode == SamplingMode.RigidbodyVelocity)
        {
            if (motionRb == null) return;
            Vector3 evalPos = (headTransform != null) ? headTransform.position : motionSource.position;
            Vector3 currentVel = motionRb.GetPointVelocity(evalPos);
            float yawRate = Vector3.Dot(motionRb.angularVelocity, motionSource.up);
            if (Mathf.Abs(yawRate) > maxYawRateDegPerSec * Mathf.Deg2Rad) yawRate = 0f;
            StepKinematics(currentVel, yawRate, dt);
        }
        else if (activeMode == SamplingMode.FixedUpdatePosition)
        {
            Vector3 pos = (motionRb != null) ? motionRb.position : motionSource.position;
            Quaternion rot = (motionRb != null) ? motionRb.rotation : motionSource.rotation;
            SamplePosition(pos, rot, dt);
        }
    }

    void LateUpdate()
    {
        if (motionSource == null || activeMode != SamplingMode.LateUpdatePosition) return;

        float dt = Time.deltaTime;
        if (dt <= 1e-6f) return; // paused: stop publishing, the sender ramps to zero when stale

        SamplePosition(motionSource.position, motionSource.rotation, dt);
    }

    // =====================================================================
    // Kinematics
    // =====================================================================

    private SamplingMode ResolveSamplingMode(bool logFallback)
    {
        SamplingMode m = samplingMode;
        if (m == SamplingMode.Auto)
        {
            if (motionRb == null) m = SamplingMode.LateUpdatePosition;
            else m = motionRb.isKinematic ? SamplingMode.FixedUpdatePosition : SamplingMode.RigidbodyVelocity;
        }
        if (m == SamplingMode.RigidbodyVelocity && (motionRb == null || motionRb.isKinematic))
        {
            if (logFallback) Debug.LogWarning("[GVS] RigidbodyVelocity mode needs a non-kinematic Rigidbody on/above motionSource. Falling back to " +
                             (motionRb != null ? "FixedUpdatePosition." : "LateUpdatePosition."));
            m = (motionRb != null) ? SamplingMode.FixedUpdatePosition : SamplingMode.LateUpdatePosition;
        }
        return m;
    }

    private void RefreshSamplingMode()
    {
        if (motionRb == null && motionSource != null) motionRb = motionSource.GetComponentInParent<Rigidbody>();

        // Re-resolved every tick so Inspector changes and isKinematic toggles take effect.
        SamplingMode m = ResolveSamplingMode(false);
        if (m != activeMode)
        {
            activeMode = m;
            ResetKinematicsState(); // switching sources mid-stream would create a fake acceleration step
        }
    }

    private void WarnAboutSetup()
    {
        if (motionSource != null && (motionSource.GetComponent<Camera>() != null ||
                                     (headTransform != null && motionSource == headTransform && headTransform.GetComponent<Camera>() != null)))
        {
            Debug.LogWarning("[GVS] motionSource is the HMD camera. Real head movement would drive stimulation. " +
                             "Assign the vehicle / XR rig root instead (headTransform is used for orientation only).");
        }

        if (activeMode == SamplingMode.LateUpdatePosition && motionRb != null && motionRb.interpolation == RigidbodyInterpolation.None)
        {
            Debug.LogWarning("[GVS] Sampling a Rigidbody-driven object in LateUpdate without Rigidbody interpolation. " +
                             "Velocity will alias (0 / 2x on alternating frames). Enable interpolation or use FixedUpdatePosition.");
        }
    }

    /// <summary>Call from teleport / respawn / scene-reset code.</summary>
    public void ResetKinematicsState()
    {
        rawLateralAccel = 0f;
        smoothedLateralAccel = 0f;
        hasLastPosition = false;
        hasLastVelocity = false;
        hasLastRotation = false;
        if (motionSource != null)
        {
            lastMotionPosition = motionSource.position;
            lastMotionRotation = motionSource.rotation;
        }
    }

    private void SamplePosition(Vector3 pos, Quaternion rot, float dt)
    {
        float yawRate = YawRateFromRotation(rot, dt);

        if (!hasLastPosition)
        {
            lastMotionPosition = pos;
            hasLastPosition = true;
            UpdateOutput(0f, dt);
            return;
        }

        Vector3 vel = (pos - lastMotionPosition) / dt;
        lastMotionPosition = pos;

        if (vel.magnitude > teleportSpeedThreshold)
        {
            // Teleport / respawn: re-warm the velocity stage so the jump never becomes acceleration.
            rejectedSpikes++;
            hasLastVelocity = false;
            UpdateOutput(0f, dt);
            return;
        }

        StepKinematics(vel, yawRate, dt);
    }

    private float YawRateFromRotation(Quaternion current, float dt)
    {
        if (!hasLastRotation)
        {
            lastMotionRotation = current;
            hasLastRotation = true;
            return 0f;
        }

        Quaternion delta = current * Quaternion.Inverse(lastMotionRotation);
        lastMotionRotation = current;

        delta.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (float.IsNaN(axis.x) || float.IsInfinity(axis.x) || axis.sqrMagnitude < 1e-8f) return 0f;
        if (angleDeg > 180f) angleDeg -= 360f;

        float yawRate = angleDeg * Mathf.Deg2Rad * Vector3.Dot(axis.normalized, motionSource.up) / dt;
        if (Mathf.Abs(yawRate) > maxYawRateDegPerSec * Mathf.Deg2Rad) return 0f; // snap turn
        return yawRate;
    }

    private void StepKinematics(Vector3 currentWorldVelocity, float yawRate, float dt)
    {
        if (dt <= 1e-6f) return;

        if (!hasLastVelocity)
        {
            lastMotionVelocity = currentWorldVelocity;
            hasLastVelocity = true;
            UpdateOutput(0f, dt);
            return;
        }

        // Acceleration in WORLD space first, then project onto the head's right axis,
        // so head rotation alone cannot create fake acceleration.
        Vector3 worldAcceleration = (currentWorldVelocity - lastMotionVelocity) / dt;
        lastMotionVelocity = currentWorldVelocity;

        float signal;
        if (worldAcceleration.magnitude > spikeRejectAccel)
        {
            // Teleport, snap turn (rig moves through an arc), respawn, or rb.velocity being set
            // directly. The velocity state has been resynced above; contribute nothing this tick.
            rejectedSpikes++;
            rawLateralAccel = 0f;
            signal = 0f;
        }
        else
        {
            Transform orientation = (headTransform != null) ? headTransform : motionSource;
            rawLateralAccel = Vector3.Dot(worldAcceleration, orientation.right);
            signal = rawLateralAccel + yawRateGain * yawRate;
        }

        UpdateOutput(signal, dt);
    }

    private void UpdateOutput(float signal, float dt)
    {
        if (useSmoothing)
        {
            float alpha = 1f - Mathf.Exp(-smoothingSpeed * dt);
            smoothedLateralAccel = Mathf.Lerp(smoothedLateralAccel, signal, alpha);
        }
        else
        {
            smoothedLateralAccel = signal;
        }

        int signedTarget = ComputeSignedTarget(smoothedLateralAccel);
        targetAmplitudeSigned = signedTarget;

        link.SetTarget(signedTarget);
    }

    /// <summary>Maps lateral acceleration to a signed current target (positive = direction 1).</summary>
    public int ComputeSignedTarget(float lateralAccel)
    {
        float absAccel = Mathf.Abs(lateralAccel);
        if (float.IsNaN(absAccel) || absAccel < accelDeadzone) return 0;

        int cap = Mathf.Clamp(maxAmplitudeMicroAmps, 0, HardMaxAmplitudeMicroAmps);
        float normalized = Mathf.InverseLerp(accelDeadzone, maxExpectedAccel, absAccel);
        int amplitude = Mathf.Clamp(Mathf.RoundToInt(normalized * cap), 0, cap);

        bool isRightward = lateralAccel > 0f;
        if (invertDirection) isRightward = !isRightward;
        return isRightward ? amplitude : -amplitude;
    }

    /// <summary>Unslewed packet for a given acceleration (kept for external callers / tests).</summary>
    public GVSLink.GVSPacket ProcessMovement(float lateralAccel)
    {
        int s = ComputeSignedTarget(lateralAccel);
        return new GVSLink.GVSPacket
        {
            command = CMD_DATA,
            direction = (byte)(s > 0 ? 1 : 0),
            frequency = stimulationFrequency,
            amplitude = Math.Abs(s)
        };
    }

    // =====================================================================
    // BLE connection (discovery + subscribe here; all sending is in GVSLink)
    // =====================================================================

    public void InitComm()
    {
        if ((bleConnectThread != null && bleConnectThread.IsAlive) || link.IsStreaming)
        {
            Debug.LogWarning("[GVS] InitComm ignored: BLE is already connecting or streaming.");
            return;
        }

        if (ble == null) ble = new BLE();

        Interlocked.Exchange(ref shutdownStarted, 0);
        isRunning = true;
        PushLinkSettings();

        bleConnectThread = new Thread(BleConnectionRoutine) { IsBackground = true, Name = "GVS BLE Connect" };
        bleConnectThread.Start();
    }

    private static string NormalizeMac(string mac)
    {
        return string.IsNullOrEmpty(mac) ? "" : mac.Trim().ToLowerInvariant().Replace('-', ':');
    }

    private static bool IdMatchesMac(string id, string normalizedMac)
    {
        return !string.IsNullOrEmpty(id) && id.Trim().ToLowerInvariant().EndsWith(normalizedMac, StringComparison.Ordinal);
    }

    private string ScanForDevice(string mac)
    {
        string wantName = (targetDeviceName ?? "").Trim();
        if (string.IsNullOrEmpty(mac) && string.IsNullOrEmpty(wantName))
        {
            Debug.LogError("[GVS] No deviceId, targetMacAddress or targetDeviceName set.");
            return null;
        }

        Debug.Log("[GVS] Scanning for " + (!string.IsNullOrEmpty(mac) ? "MAC " + mac : "name '" + wantName + "'"));

        // Not disposed on purpose: a late callback from the scan thread must never hit a disposed handle.
        ManualResetEventSlim foundSignal = new ManualResetEventSlim(false);
        string discoveredId = null;

        activeScan = BLE.ScanDevices((id, name) =>
        {
            bool match = !string.IsNullOrEmpty(mac)
                ? IdMatchesMac(id, mac)
                : (!string.IsNullOrEmpty(name) && string.Equals(name.Trim(), wantName, StringComparison.OrdinalIgnoreCase));
            if (!match) return;

            if (Interlocked.CompareExchange(ref discoveredId, id, null) == null)
            {
                Debug.Log("[GVS] Found " + name + " (" + id + ")");
                foundSignal.Set();
            }
        });

        int waitedMs = 0, timeoutMs = Mathf.RoundToInt(scanTimeoutSeconds * 1000f);
        while (isRunning && waitedMs < timeoutMs && !foundSignal.Wait(100)) waitedMs += 100;

        activeScan.Cancel();
        if (!BLE.WaitForScanToFinish(2000))
            Debug.LogWarning("[GVS] Scan thread did not exit within 2 s.");

        if (!isRunning) return null;
        string result = Volatile.Read(ref discoveredId);
        if (string.IsNullOrEmpty(result))
            Debug.LogError("[GVS] Could not find device " + (!string.IsNullOrEmpty(mac) ? "with MAC " + mac : "named '" + wantName + "'") +
                           " within " + scanTimeoutSeconds + " s.");
        return result;
    }

    private void BleConnectionRoutine()
    {
        try
        {
            string mac = NormalizeMac(targetMacAddress);
            string dev = (deviceId ?? "").Trim();

            if (string.IsNullOrEmpty(dev))
            {
                dev = ScanForDevice(mac);
                if (string.IsNullOrEmpty(dev)) return;
                deviceId = dev;
            }
            else
            {
                if (!string.IsNullOrEmpty(mac) && !IdMatchesMac(dev, mac))
                {
                    Debug.LogError("[GVS] deviceId '" + dev + "' does not end with targetMacAddress '" + mac + "'. Refusing to connect to a possibly different stimulator.");
                    return;
                }
                // No scan will run, so clear the DLL's quit flag left by a previous Editor Play session.
                BLE.ResetNativeState();
            }

            if (!isRunning) return;

            // Discover Services
            List<string> services = BLE.ScanAndGetServices(dev);
            string svc = MatchUuid(serviceUuid, services);
            if (string.IsNullOrEmpty(svc))
            {
                Debug.LogError("[GVS] Service UUID not found: " + serviceUuid + " (found " + services.Count + " services). DLL: " + BLE.GetError());
                return;
            }

            if (!isRunning) return;

            // Discover Characteristics
            Dictionary<string, string> charMap = BLE.ScanAndGetCharacteristics(dev, svc);
            string writeChar = MatchUuid(characteristicUuid, charMap.Values);
            if (string.IsNullOrEmpty(writeChar))
            {
                Debug.LogError("[GVS] Write characteristic not found: " + characteristicUuid + ". DLL: " + BLE.GetError());
                return;
            }
            string notifyChar = MatchUuid(notifyCharacteristicUuid, charMap.Values);

            if (!isRunning) return;

            if (!string.IsNullOrEmpty(notifyChar))
            {
                Debug.Log("[GVS] Subscribing to notify characteristic " + notifyChar);
                if (!ble.Connect(dev, svc, new string[] { notifyChar }))
                {
                    Debug.LogError("[GVS] Subscribe to notify characteristic failed. DLL: " + BLE.GetError());
                    return;
                }
                Thread.Sleep(200); // let the CCCD write settle before the handshake
            }
            else
            {
                Debug.LogWarning("[GVS] Notify characteristic " + notifyCharacteristicUuid + " not found; continuing without subscription.");
            }

            if (!isRunning) return;

            // START handshake + streaming thread (same code path the Demo uses).
            link.Start(dev, svc, writeChar);
        }
        catch (Exception e)
        {
            if (isRunning)
                Debug.LogError("[GVS] BLE init exception: " + e.Message + ". DLL: " + BLE.GetError());
        }
    }

    // =====================================================================
    // Helpers
    // =====================================================================

    private static string CleanUuid(string uuid)
    {
        return string.IsNullOrEmpty(uuid) ? "" : uuid.Trim().Trim('{', '}').ToLowerInvariant();
    }

    private static string MatchUuid(string target, IEnumerable<string> candidates)
    {
        string cleanTarget = CleanUuid(target);
        if (cleanTarget.Length == 0) return null;
        foreach (string c in candidates)
        {
            if (CleanUuid(c) == cleanTarget)
                return c;
        }
        return null;
    }

    // =====================================================================
    // Shutdown: runs once, never freezes the main thread.
    //   1. stop the scan, 2. link.Stop(): stop the sender and send STOP,
    //   3. stop the connect thread, 4. native Quit() only if no write is still in flight.
    // The firmware must ALSO zero its outputs on BLE disconnect and on watchdog timeout.
    // =====================================================================
    void OnApplicationQuit() { ShutdownBle(); }
    void OnDestroy() { ShutdownBle(); }

    private void ShutdownBle()
    {
        if (Interlocked.Exchange(ref shutdownStarted, 1) == 1) return;
        if (!isRunning && bleConnectThread == null) return; // never started

        Debug.Log("[GVS] Shutdown: begin");
        isRunning = false;
        link.Muted = true;

        BLE bleToClose = ble;
        ble = null;
        Thread connector = bleConnectThread;
        BLE.BLEScan scan = activeScan;

        Thread teardown = new Thread(() =>
        {
            try
            {
                try { if (scan != null) scan.Cancel(); } catch { }

                bool safeToQuit = link.Stop(1000);

                if (connector != null && connector.IsAlive) connector.Join(500);

                if (safeToQuit)
                {
                    Debug.Log("[GVS] Shutdown: calling native Quit()");
                    if (bleToClose != null) bleToClose.Close();
                    else BLE.Impl.Quit();
                    Debug.Log("[GVS] Shutdown: native Quit() returned");
                }
                else
                {
                    Debug.LogWarning("[GVS] Shutdown: a native write is still in flight; skipping Quit() to avoid a crash in the DLL.");
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning("[GVS] Shutdown exception: " + e.Message);
            }
        })
        { IsBackground = true, Name = "GVS BLE Teardown" };

        teardown.Start();

        if (!teardown.Join(3000))
            Debug.LogWarning("[GVS] Shutdown: native teardown still blocked after 3 s - abandoning it so Unity can exit.");
    }
}