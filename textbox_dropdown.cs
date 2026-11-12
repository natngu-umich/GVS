using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Serialization;
using UnityEngine.UI;
using Debug = UnityEngine.Debug;

/// <summary>
/// Streams roll-axis GVS commands to the ESP32 stimulator over BLE.
///
/// Safety model (defence in depth; the firmware and analog stage must have their own limits too):
///   * Hard amplitude ceiling in code (HardMaxAmplitudeMicroAmps), clamped again right before sending.
///   * Commanded current is slew-limited, and polarity changes always ramp through zero.
///   * Teleports, snap turns and other kinematic spikes are rejected instead of becoming current.
///   * The sender transmits every tick (heartbeat), so a firmware watchdog can detect silence.
///   * If the motion target goes stale (pause, disabled object, hitch, lost focus) the sender
///     ramps the output to zero on its own.
///   * Lost links are detected; output restarts from zero and START is re-sent on recovery.
///   * On shutdown a STOP is sent; the firmware must ALSO zero outputs on BLE disconnect.
/// </summary>
public class GVSController : MonoBehaviour
{
    // Absolute ceiling on commanded current, in microamps. Deliberately not an Inspector field:
    // raising it should be a conscious code change (TLDR safety threshold is < 3.5 mA).
    public const int HardMaxAmplitudeMicroAmps = 3000;

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

    [Header("Streaming")]
    [Tooltip("Packets per second. Every tick is sent (heartbeat), so the ESP32 watchdog can tell silence from 'no change'. Check Measured Send Rate Hz for what the link really achieves.")]
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
    [Tooltip("Maximum rate of change of the commanded current (uA per second). Polarity reversals ramp through zero.")]
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

    [Header("ESP32 Feedback Tracker (on-screen box)")]
    [Tooltip("Show the on-screen box with the values the ESP32 reports back. The box is created automatically. Values are still received and stored when this is off.")]
    public bool showEsp32Tracker = true;
    [Tooltip("Expanded or minimized. Clicking the box's title bar flips this.")]
    public bool esp32TrackerExpanded = true;
    [Tooltip("Distance of the box from the top-left corner of the screen (pixels at 1920x1080).")]
    public Vector2 esp32TrackerOffset = new Vector2(12f, 12f);

    [Header("ESP32 Feedback (Read-Only)")]
    [Tooltip("Latest current measured by the ESP32 (uA), from its notify characteristic.")]
    public float esp32MeasuredCurrentMicroAmps;
    [Tooltip("True if the latest notification could be read as a number.")]
    public bool esp32LatestIsNumeric;
    [Tooltip("Latest notification as readable text.")]
    public string esp32LatestText = "";
    [Tooltip("Latest notification as raw bytes.")]
    public string esp32LatestHex = "";
    [Tooltip("Seconds since the last notification arrived (-1 = none yet).")]
    public float esp32SecondsSinceLastValue = -1f;

    /// <summary>Raw bytes of the latest ESP32 notification (null until one arrives). Do not modify.</summary>
    public byte[] Esp32LatestBytes { get { return esp32LatestBytes; } }

    public const byte CMD_STOP = 0;
    public const byte CMD_START = 1;
    public const byte CMD_DATA = 2;

    public struct GVSPacket
    {
        public const int Size = 10;

        public byte command;
        public byte direction;
        public int frequency;
        public int amplitude;

        public bool SameAs(GVSPacket o)
        {
            return command == o.command && direction == o.direction &&
                   frequency == o.frequency && amplitude == o.amplitude;
        }

        // Little-endian, matches a #pragma pack(1) struct on the ESP32 (also little-endian).
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

    // ---------------- BLE / threading state ----------------
    private BLE ble;
    private BLE.BLEScan activeScan;
    private Thread bleConnectThread;
    private Thread bleSenderThread;
    private readonly ManualResetEventSlim shutdownEvent = new ManualResetEventSlim(false);

    // Resolved connection info. Written by the connect thread before the sender starts.
    private volatile string connDeviceId;
    private volatile string connServiceUuid;
    private volatile string connCharUuid;

    private volatile bool isConnectedAndReady = false;
    private volatile bool isRunning = false;
    private volatile bool linkLost = false;
    private volatile bool outputEnabled = true;
    private volatile bool appPaused = false;
    private volatile bool appUnfocused = false;
    private int shutdownStarted = 0;
    private int senderGeneration = 0;
    private long writeInFlightSince = 0;   // Stopwatch ticks; 0 = no write in progress
    private int abandonedSenders = 0;
    private const int MaxAbandonedSenders = 3;

    // Target handed from the main thread (producer) to the sender (consumer).
    private readonly object targetLock = new object();
    private int sharedTargetSignedUa = 0;
    private long sharedTargetTimestamp = 0; // Stopwatch ticks; 0 = never published

    // ---------------- ESP32 feedback (notifications) ----------------
    // The sender thread pops notifications from the DLL and queues them here;
    // Update() moves them into the public fields on the main thread.
    private readonly object notifyLock = new object();
    private readonly Queue<byte[]> pendingNotifications = new Queue<byte[]>();
    private Action<byte[], int> notificationCallback;
    private byte[] esp32LatestBytes;
    private float esp32LastReceivedTime = -1f;
    private readonly List<string> esp32History = new List<string>();
    private const int Esp32HistoryLines = 5;

    // ---------------- Tracker box (created in code, main thread only) ----------------
    private GameObject trackerRoot;
    private GameObject trackerBody;
    private GameObject trackerEventSystem;
    private RectTransform trackerHeaderRect;
    private RectTransform trackerBodyRect;
    private Text trackerHeaderText;
    private Text trackerBodyText;
    private float nextTrackerRefresh;
    private readonly StringBuilder trackerSb = new StringBuilder(512);
    private const float TrackerWidth = 520f, TrackerHeaderHeight = 40f, TrackerBodyHeight = 270f;

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

        if (motionSource == null) motionSource = this.transform;
        if (headTransform == null)
            headTransform = (Camera.main != null) ? Camera.main.transform : motionSource;

        motionRb = motionSource.GetComponentInParent<Rigidbody>();
        activeMode = ResolveSamplingMode(true);
        WarnAboutSetup();

        ResetKinematicsState();

        // Part of the normal start-up: nothing to create or wire up in the scene.
        if (showEsp32Tracker) BuildTrackerUi();

        if (!string.IsNullOrEmpty(deviceId) || !string.IsNullOrEmpty(targetDeviceName) || !string.IsNullOrEmpty(targetMacAddress))
        {
            InitComm();
        }
    }

    void OnEnable()
    {
        outputEnabled = true;
        // Re-warm so the time spent disabled does not show up as a velocity/acceleration jump.
        hasLastPosition = hasLastVelocity = hasLastRotation = false;
    }

    void OnDisable()
    {
        // FixedUpdate/LateUpdate stop running while disabled; tell the sender to ramp to zero now
        // rather than waiting for the target to go stale.
        outputEnabled = false;
        if (trackerRoot != null) trackerRoot.SetActive(false);
    }

    void OnApplicationPause(bool paused) { appPaused = paused; }
    void OnApplicationFocus(bool hasFocus) { appUnfocused = !hasFocus; }

    void Update()
    {
        isBleReady = isConnectedAndReady && !linkLost;
        CheckForHungWrite();
        ProcessEsp32Notifications();
        UpdateTrackerUi();
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

        lock (targetLock)
        {
            sharedTargetSignedUa = signedTarget;
            sharedTargetTimestamp = Stopwatch.GetTimestamp();
        }
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
    public GVSPacket ProcessMovement(float lateralAccel)
    {
        int s = ComputeSignedTarget(lateralAccel);
        return new GVSPacket
        {
            command = CMD_DATA,
            direction = (byte)(s > 0 ? 1 : 0),
            frequency = stimulationFrequency,
            amplitude = Math.Abs(s)
        };
    }

    // =====================================================================
    // BLE connection
    // =====================================================================

    public void InitComm()
    {
        if ((bleConnectThread != null && bleConnectThread.IsAlive) ||
            (bleSenderThread != null && bleSenderThread.IsAlive))
        {
            Debug.LogWarning("[GVS] InitComm ignored: BLE threads are already running.");
            return;
        }

        if (ble == null) ble = new BLE();
        if (notificationCallback == null) notificationCallback = QueueEsp32Notification;

        Interlocked.Exchange(ref shutdownStarted, 0);
        isRunning = true;
        isConnectedAndReady = false;
        linkLost = false;
        abandonedSenders = 0;
        consecutiveWriteFailures = 0;
        connDeviceId = connServiceUuid = connCharUuid = null;
        outputEnabled = isActiveAndEnabled;
        shutdownEvent.Reset();

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

            // From here on, shutdown can address the device with a STOP even if later steps fail.
            connDeviceId = dev;
            connServiceUuid = svc;
            connCharUuid = writeChar;

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

            // Handshake. Write-without-response: success means Windows queued it, not that the ESP32 got it.
            if (!SendCommandWithRetry(new BLE.PacketWriter(dev, svc, writeChar), CMD_START, 3))
            {
                Debug.LogError("[GVS] START handshake write failed. DLL: " + BLE.GetError());
                return;
            }

            if (!isRunning) return;

            isConnectedAndReady = true;
            Debug.Log("[GVS] Pipeline active. Streaming at up to " + sendRateHz + " Hz on a dedicated thread.");
            StartSender(false);
        }
        catch (Exception e)
        {
            if (isRunning)
                Debug.LogError("[GVS] BLE init exception: " + e.Message + ". DLL: " + BLE.GetError());
        }
    }

    private bool SendCommandWithRetry(BLE.PacketWriter writer, byte command, int attempts)
    {
        byte[] buf = new byte[GVSPacket.Size];
        new GVSPacket { command = command }.WriteToBuffer(buf);
        for (int i = 0; i < attempts; i++)
        {
            if (writer.Write(buf, GVSPacket.Size)) return true;
            if (i + 1 < attempts) Thread.Sleep(100);
        }
        return false;
    }

    private void StartSender(bool rearmFirst)
    {
        int gen = Interlocked.Increment(ref senderGeneration);
        Thread t = new Thread(() => BleSenderLoop(gen, rearmFirst))
        {
            IsBackground = true,
            Name = "GVS BLE Sender #" + gen,
            Priority = System.Threading.ThreadPriority.AboveNormal
        };
        bleSenderThread = t;
        t.Start();
    }

    // Main thread. Recovers from the DLL's rare "SendData never returns" hang by abandoning
    // the stuck thread (it exits on its own if the call ever returns) and starting a new one.
    // While stuck, no packets flow, so the ESP32 watchdog zeroes the output.
    private void CheckForHungWrite()
    {
        if (!isRunning || !isConnectedAndReady) return;
        long since = Interlocked.Read(ref writeInFlightSince);
        if (since == 0) return;

        double ms = (Stopwatch.GetTimestamp() - since) * 1000.0 / Stopwatch.Frequency;
        if (ms < writeStallTimeoutMs) return;

        Interlocked.Exchange(ref writeInFlightSince, 0);
        abandonedSenders++;
        if (abandonedSenders > MaxAbandonedSenders)
        {
            Interlocked.Increment(ref senderGeneration);
            isConnectedAndReady = false;
            Debug.LogError("[GVS] Native BLE write hung again (" + abandonedSenders + " times). Streaming stopped; restart the app.");
        }
        else
        {
            Debug.LogError("[GVS] Native BLE write blocked for " + ms.ToString("F0") + " ms. Abandoning that sender thread and starting a new one.");
            StartSender(true);
        }
    }

    // =====================================================================
    // Sender thread
    // =====================================================================

    private void BleSenderLoop(int myGen, bool rearmFirst)
    {
        BLE.PacketWriter writer = new BLE.PacketWriter(connDeviceId, connServiceUuid, connCharUuid);
        byte[] buf = new byte[GVSPacket.Size];
        long freq = Stopwatch.Frequency;
        long next = Stopwatch.GetTimestamp();
        long lastTick = next;
        long rateWindowStart = next;
        int sentInWindow = 0;
        long lastDrain = next;
        float slewed = 0f;          // signed uA actually commanded
        bool needsArm = rearmFirst;

        while (isRunning && myGen == Volatile.Read(ref senderGeneration))
        {
            // ---- Deadline scheduling (no drift; no burst after falling behind) ----
            float rate = Mathf.Clamp(sendRateHz, 20f, 100f);
            if (linkLost) rate = 2f; // while the link is down, probe slowly
            long interval = (long)(freq / rate);
            next += interval;
            long now = Stopwatch.GetTimestamp();
            if (now - next > interval) next = now;
            int waitMs = (int)((next - now) * 1000 / freq);
            if (waitMs > 0 && shutdownEvent.Wait(waitMs)) break;
            if (!isRunning || myGen != Volatile.Read(ref senderGeneration)) break;

            now = Stopwatch.GetTimestamp();
            float dt = Mathf.Min((float)(now - lastTick) / freq, 0.05f);
            lastTick = now;

            // ---- Target: zero when stale, disabled, paused, unfocused or link down ----
            int target;
            long stamp;
            lock (targetLock)
            {
                target = sharedTargetSignedUa;
                stamp = sharedTargetTimestamp;
            }
            bool stale = stamp == 0 || (now - stamp) * 1000 / freq > staleTimeoutMs;
            bool muted = !outputEnabled || appPaused || (zeroOutputWhenUnfocused && appUnfocused);
            if (stale || muted || linkLost) target = 0;

            int cap = Mathf.Clamp(maxAmplitudeMicroAmps, 0, HardMaxAmplitudeMicroAmps);
            target = Mathf.Clamp(target, -cap, cap);

            // ---- Slew limit on the SIGNED value: polarity changes pass through zero ----
            float maxStep = Mathf.Max(500f, slewRateMicroAmpsPerSec) * dt;
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

                new GVSPacket { command = CMD_DATA, direction = dir, frequency = stimulationFrequency, amplitude = amp }.WriteToBuffer(buf);
                bool ok = TrackedWrite(writer, buf, myGen);
                if (myGen != Volatile.Read(ref senderGeneration)) break; // abandoned while blocked

                if (ok)
                {
                    if (linkLost)
                    {
                        linkLost = false;
                        needsArm = true;
                        Debug.Log("[GVS] BLE link recovered; re-arming.");
                    }
                    consecutiveWriteFailures = 0;
                    currentOutputAmplitude = amp;
                    currentOutputDirection = dir;
                    sentInWindow++;
                }
                else
                {
                    OnWriteFailed(ref slewed);
                }
            }
            catch (Exception ex)
            {
                if (isRunning) Debug.LogWarning("[GVS] Write exception: " + ex.Message);
                OnWriteFailed(ref slewed);
            }

            // ---- Collect incoming notifications (~20 Hz). This also keeps the DLL's queue from growing. ----
            if (now - lastDrain >= freq / 20)
            {
                lastDrain = now;
                try { notificationsReceived += BLE.DrainNotifications(32, notificationCallback); } catch { }
            }

            // ---- Measured rate ----
            double windowSec = (double)(now - rateWindowStart) / freq;
            if (windowSec >= 1.0)
            {
                measuredSendRateHz = (float)(sentInWindow / windowSec);
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
            if (myGen == Volatile.Read(ref senderGeneration))
                Interlocked.Exchange(ref writeInFlightSince, 0);
        }
    }

    private void OnWriteFailed(ref float slewed)
    {
        consecutiveWriteFailures++;
        if (consecutiveWriteFailures == 1 && isRunning)
        {
            // Note: this DLL reports failed WRITES as "Error subscribing to characteristic ...".
            Debug.LogWarning("[GVS] BLE write failed. DLL: " + BLE.GetError());
        }
        if (!linkLost && consecutiveWriteFailures >= linkLossFailureThreshold)
        {
            linkLost = true;
            slewed = 0f; // the ESP32 watchdog will have zeroed the output; restart the ramp from zero
            if (isRunning)
                Debug.LogError("[GVS] BLE link LOST after " + consecutiveWriteFailures + " failed writes. Retrying at 2 Hz. DLL: " + BLE.GetError());
        }
    }

    // =====================================================================
    // ESP32 feedback: values notified by the ESP32 (19b10001...)
    // =====================================================================

    // Sender thread. Copies the payload and hands it to the main thread.
    private void QueueEsp32Notification(byte[] buf, int len)
    {
        if (buf == null || len <= 0) return;
        byte[] copy = new byte[len];
        Buffer.BlockCopy(buf, 0, copy, 0, len);
        lock (notifyLock)
        {
            if (pendingNotifications.Count >= 64) pendingNotifications.Dequeue(); // keep the newest
            pendingNotifications.Enqueue(copy);
        }
    }

    // Main thread. Moves queued notifications into the public fields.
    private void ProcessEsp32Notifications()
    {
        while (true)
        {
            byte[] data;
            lock (notifyLock)
            {
                if (pendingNotifications.Count == 0) break;
                data = pendingNotifications.Dequeue();
            }

            bool isNumeric;
            float value;
            string text = DecodeEsp32Payload(data, out isNumeric, out value);

            esp32LatestBytes = data;
            esp32LatestHex = BitConverter.ToString(data, 0, Mathf.Min(data.Length, 16));
            esp32LatestText = text;
            esp32LatestIsNumeric = isNumeric;
            if (isNumeric) esp32MeasuredCurrentMicroAmps = value;
            esp32LastReceivedTime = Time.unscaledTime;

            esp32History.Insert(0, Time.unscaledTime.ToString("F1", CultureInfo.InvariantCulture) + " s   " + text);
            if (esp32History.Count > Esp32HistoryLines) esp32History.RemoveAt(esp32History.Count - 1);
            nextTrackerRefresh = 0f; // show it right away
        }

        esp32SecondsSinceLastValue = esp32LastReceivedTime < 0f ? -1f : Time.unscaledTime - esp32LastReceivedTime;
    }

    /// <summary>
    /// Turns a notification payload into text (and a number where possible).
    /// EDIT THIS if the firmware changes what it sends. The current sketch sends one
    /// 4-byte little-endian float: the current measured by the INA219, in uA.
    /// </summary>
    private static string DecodeEsp32Payload(byte[] data, out bool isNumeric, out float value)
    {
        isNumeric = false;
        value = 0f;

        // 4 bytes: float32, measured current in uA (what the sketch sends).
        if (data.Length == 4)
        {
            float f = BitConverter.ToSingle(data, 0);
            if (!float.IsNaN(f) && !float.IsInfinity(f) && Mathf.Abs(f) < 1e7f)
            {
                isNumeric = true;
                value = f;
                return f.ToString("F0", CultureInfo.InvariantCulture) + " uA";
            }
        }

        // 10 bytes: a GVS packet echoed back.
        if (data.Length == GVSPacket.Size && data[0] <= CMD_DATA && data[1] <= 1)
        {
            int freq = BitConverter.ToInt32(data, 2);
            int amp = BitConverter.ToInt32(data, 6);
            isNumeric = true;
            value = data[1] == 1 ? amp : -amp;
            return "cmd " + data[0] + "  dir " + data[1] + "  freq " + freq + "  amp " + amp + " uA";
        }

        // Readable text (one trailing NUL allowed), e.g. "1234.5".
        int len = data.Length;
        if (len > 0 && data[len - 1] == 0) len--;
        bool printable = len > 0;
        for (int i = 0; i < len && printable; i++)
        {
            byte b = data[i];
            printable = (b >= 0x20 && b <= 0x7E) || b == (byte)'\r' || b == (byte)'\n' || b == (byte)'\t';
        }
        if (printable)
        {
            string text = Encoding.ASCII.GetString(data, 0, len).Trim();
            isNumeric = float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
            return text;
        }

        return "[" + BitConverter.ToString(data, 0, Mathf.Min(data.Length, 16)) + "]";
    }

    // =====================================================================
    // Tracker box: built entirely in code. Click the title bar to minimize / expand.
    // It is a screen overlay: it shows in the Game window / desktop mirror, not inside the headset.
    // =====================================================================

    private void BuildTrackerUi()
    {
        if (trackerRoot != null) return;

        Font font = null;
        try
        {
#if UNITY_2022_2_OR_NEWER
            font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
#else
            font = Resources.GetBuiltinResource<Font>("Arial.ttf");
#endif
        }
        catch { }
        if (font == null) font = Font.CreateDynamicFontFromOSFont("Arial", 18);

        trackerRoot = new GameObject("GVS ESP32 Tracker (auto-created)");
        Canvas canvas = trackerRoot.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = 30000; // on top of other UI
        CanvasScaler scaler = trackerRoot.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.matchWidthOrHeight = 0.5f;
        trackerRoot.AddComponent<GraphicRaycaster>();

        // Title bar: the clickable part.
        GameObject header = new GameObject("Header", typeof(RectTransform));
        header.transform.SetParent(trackerRoot.transform, false);
        trackerHeaderRect = header.GetComponent<RectTransform>();
        Image headerBg = header.AddComponent<Image>();
        headerBg.color = new Color(0.10f, 0.13f, 0.18f, 0.95f);
        Button button = header.AddComponent<Button>();
        button.targetGraphic = headerBg;
        button.onClick.AddListener(ToggleEsp32Tracker);
        trackerHeaderText = NewTrackerText(header.transform, font, 20, TextAnchor.MiddleLeft, FontStyle.Bold);

        // Body: the values.
        trackerBody = new GameObject("Body", typeof(RectTransform));
        trackerBody.transform.SetParent(trackerRoot.transform, false);
        trackerBodyRect = trackerBody.GetComponent<RectTransform>();
        Image bodyBg = trackerBody.AddComponent<Image>();
        bodyBg.color = new Color(0f, 0f, 0f, 0.78f);
        bodyBg.raycastTarget = false;
        trackerBodyText = NewTrackerText(trackerBody.transform, font, 18, TextAnchor.UpperLeft, FontStyle.Normal);

        // Clicks need an EventSystem. Use the scene's if there is one, otherwise make one.
        if (EventSystem.current == null)
        {
            trackerEventSystem = new GameObject("EventSystem (GVS auto-created)");
            trackerEventSystem.AddComponent<EventSystem>();
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
            // Project uses only the new Input System: use its UI module (looked up by name so this
            // file compiles whether or not the package is referenced).
            Type newModule = Type.GetType("UnityEngine.InputSystem.UI.InputSystemUIInputModule, Unity.InputSystem");
            if (newModule != null) trackerEventSystem.AddComponent(newModule);
            else trackerEventSystem.AddComponent<StandaloneInputModule>();
#else
            trackerEventSystem.AddComponent<StandaloneInputModule>();
#endif
        }

        nextTrackerRefresh = 0f;
    }

    private static Text NewTrackerText(Transform parent, Font font, int size, TextAnchor anchor, FontStyle style)
    {
        GameObject go = new GameObject("Text", typeof(RectTransform));
        go.transform.SetParent(parent, false);
        RectTransform rt = go.GetComponent<RectTransform>();
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.offsetMin = new Vector2(12f, 6f);
        rt.offsetMax = new Vector2(-12f, -6f);

        Text t = go.AddComponent<Text>();
        t.font = font;
        t.fontSize = size;
        t.fontStyle = style;
        t.alignment = anchor;
        t.color = Color.white;
        t.supportRichText = false;
        t.raycastTarget = false;
        t.horizontalOverflow = HorizontalWrapMode.Wrap;
        t.verticalOverflow = VerticalWrapMode.Truncate;
        return t;
    }

    private static void PlaceTopLeft(RectTransform rt, float x, float y, float width, float height)
    {
        rt.anchorMin = rt.anchorMax = rt.pivot = new Vector2(0f, 1f);
        rt.anchoredPosition = new Vector2(x, -y);
        rt.sizeDelta = new Vector2(width, height);
    }

    /// <summary>Minimize if expanded, expand if minimized. Called by a click on the title bar.</summary>
    public void ToggleEsp32Tracker()
    {
        esp32TrackerExpanded = !esp32TrackerExpanded;
        nextTrackerRefresh = 0f;
    }

    private void UpdateTrackerUi()
    {
        if (!showEsp32Tracker)
        {
            if (trackerRoot != null && trackerRoot.activeSelf) trackerRoot.SetActive(false);
            return;
        }

        if (trackerRoot == null) BuildTrackerUi();
        if (!trackerRoot.activeSelf) trackerRoot.SetActive(true);
        if (trackerBody.activeSelf != esp32TrackerExpanded) trackerBody.SetActive(esp32TrackerExpanded);

        if (Time.unscaledTime < nextTrackerRefresh) return;
        nextTrackerRefresh = Time.unscaledTime + 0.1f;

        PlaceTopLeft(trackerHeaderRect, esp32TrackerOffset.x, esp32TrackerOffset.y, TrackerWidth, TrackerHeaderHeight);
        PlaceTopLeft(trackerBodyRect, esp32TrackerOffset.x, esp32TrackerOffset.y + TrackerHeaderHeight, TrackerWidth, TrackerBodyHeight);

        bool hasData = esp32LatestBytes != null;
        string latest = hasData ? esp32LatestText : "no data yet";

        if (!esp32TrackerExpanded)
        {
            trackerHeaderText.text = "[+] ESP32: " + latest;
            return;
        }
        trackerHeaderText.text = "[-] ESP32 current value tracker";

        StringBuilder sb = trackerSb;
        sb.Length = 0;

        sb.Append("Link: ");
        if (!isConnectedAndReady) sb.Append("not connected");
        else if (linkLost) sb.Append("LOST, retrying");
        else sb.Append("streaming at ").Append(measuredSendRateHz.ToString("F0", CultureInfo.InvariantCulture)).Append(" Hz");
        sb.Append('\n');

        sb.Append("Commanded: ").Append(currentOutputAmplitude).Append(" uA, dir ").Append(currentOutputDirection).Append('\n');
        sb.Append("ESP32 reports: ").Append(latest).Append('\n');

        if (hasData)
        {
            sb.Append("Raw: [").Append(esp32LatestHex).Append("]\n");
            sb.Append("Received: ").Append(notificationsReceived)
              .Append("   last ").Append(esp32SecondsSinceLastValue.ToString("F1", CultureInfo.InvariantCulture)).Append(" s ago\n");
            sb.Append("Recent:\n");
            for (int i = 0; i < esp32History.Count; i++)
                sb.Append("  ").Append(esp32History[i]).Append('\n');
        }
        else if (isConnectedAndReady)
        {
            sb.Append("Waiting for the first notification from the ESP32...\n");
        }

        trackerBodyText.text = sb.ToString();
    }

    private void DestroyTrackerUi()
    {
        if (trackerRoot != null) Destroy(trackerRoot);
        if (trackerEventSystem != null) Destroy(trackerEventSystem);
        trackerRoot = null;
        trackerBody = null;
        trackerEventSystem = null;
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
    // Shutdown: strictly ordered, runs once, never freezes the main thread.
    //   1. stop producing / sending
    //   2. wait (bounded) for the sender to leave native code
    //   3. send STOP (queued write-without-response), give the radio time to transmit it
    //   4. stop connect thread
    //   5. call native Quit() only if no native write is still in flight
    // The firmware must ALSO zero its outputs on BLE disconnect and on watchdog timeout.
    // =====================================================================
    void OnApplicationQuit() { ShutdownBle(); }
    void OnDestroy() { ShutdownBle(); DestroyTrackerUi(); }

    private void ShutdownBle()
    {
        if (Interlocked.Exchange(ref shutdownStarted, 1) == 1) return;
        if (!isRunning && bleConnectThread == null) return; // never started

        Debug.Log("[GVS] Shutdown: begin");

        isRunning = false;
        isConnectedAndReady = false;
        outputEnabled = false;
        Interlocked.Increment(ref senderGeneration);
        shutdownEvent.Set();

        BLE bleToClose = ble;
        ble = null;
        string dev = connDeviceId, svc = connServiceUuid, chr = connCharUuid;
        Thread sender = bleSenderThread;
        Thread connector = bleConnectThread;
        BLE.BLEScan scan = activeScan;
        bool anyAbandoned = abandonedSenders > 0;

        Thread teardown = new Thread(() =>
        {
            try
            {
                try { if (scan != null) scan.Cancel(); } catch { }

                bool senderStopped = sender == null || !sender.IsAlive || sender.Join(1000);
                Debug.Log("[GVS] Shutdown: sender stopped = " + senderStopped);

                if (!string.IsNullOrEmpty(dev) && !string.IsNullOrEmpty(svc) && !string.IsNullOrEmpty(chr))
                {
                    byte[] stopBuf = new byte[GVSPacket.Size];
                    new GVSPacket { command = CMD_STOP }.WriteToBuffer(stopBuf);
                    BLE.PacketWriter w = new BLE.PacketWriter(dev, svc, chr);
                    bool ok = w.Write(stopBuf, GVSPacket.Size) || w.Write(stopBuf, GVSPacket.Size);
                    if (ok)
                        Debug.Log("[GVS] Shutdown: STOP queued (write-without-response: delivery is not acknowledged)");
                    else
                        Debug.LogWarning("[GVS] Shutdown: STOP write failed. Relying on firmware disconnect/watchdog. DLL: " + BLE.GetError());

                    // Give Windows time to put the queued STOP on air before the link is torn down.
                    Thread.Sleep(100);
                }

                if (connector != null && connector.IsAlive) connector.Join(500);

                if (senderStopped && !anyAbandoned)
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