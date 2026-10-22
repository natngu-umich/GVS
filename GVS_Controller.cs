using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class GVSController : MonoBehaviour
{
    [Header("Tracking Target")]
    [Tooltip("Leave empty to track the GameObject this script is attached to, or drag a target here.")]
    public Transform targetTransform;
    private Rigidbody targetRb;

    [Header("BLE Configuration")]
    
    [Tooltip("Advertised BLE name of your ESP32 (e.g., 'ESP32-Brocoli'). Used to auto-find deviceId if deviceId is blank.")]
    public string targetDeviceName = "ESP32";
    
    [Tooltip("Windows BLE Device ID (e.g., BluetoothLE#...). Leave blank to auto-scan by targetDeviceName.")]
    public string deviceId = "";
    public string serviceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
    public string characteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

    [Tooltip("Target transmission rate in Hz (100 Hz = 10ms intervals).")]
    [Range(10f, 120f)] public float sendRateHz = 100f;
    
    [Tooltip("Only enable if your ESP32 characteristic also sends Notify packets back to Unity.")]
    public bool subscribeToNotifications = false;

    [Header("GVS Motion Mapping (Roll Axis)")]
    [Tooltip("Minimum lateral acceleration (m/s^2) before GVS activates.")]
    public float accelDeadzone = 0.5f;
    [Tooltip("Lateral acceleration (m/s^2) that maps to maximum GVS current.")]
    public float maxExpectedAccel = 15.0f;
    [Tooltip("Enable low-pass filtering to remove single-frame collision spikes.")]
    public bool useSmoothing = true;
    [Tooltip("Filter speed (50 = ~20ms response time for 100Hz tracking).")]
    [Range(5f, 100f)] public float smoothingSpeed = 50f;
    [Tooltip("Flip left/right direction bit if electrode polarity is reversed.")]
    public bool invertDirection = false;

    [Header("Hardware Limits")]
    [Tooltip("Maximum safe current amplitude in microamps (e.g., 3000 uA = 3.0 mA).")]
    public int maxAmplitudeMicroAmps = 3000;
    public int defaultFrequencyMHz = 0;

    [Header("Debug Telemetry (Read-Only)")]
    public float rawLateralAccel;
    public float smoothedLateralAccel;
    public int currentOutputAmplitude;
    public byte currentOutputDirection;
    public bool isBleReady = false;

    // Command Opcodes
    public const byte CMD_STOP  = 0;
    public const byte CMD_START = 1;
    public const byte CMD_DATA  = 2;

    // 10-byte binary packet matching C/C++ struct alignment on ESP32
    public struct GVSPacket
    {
        public byte command;     // 0 = STOP, 1 = START, 2 = DATA (1 byte)
        public byte direction;   // 1 = Left-to-Right (+Roll), 0 = Right-to-Left (-Roll) (1 byte)
        public int frequency;    // mHz (4 bytes, Little-Endian)
        public int amplitude;    // uA (4 bytes, Little-Endian)

        // Zero-allocation serialization into a pre-allocated 10-byte buffer
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

    // Internal state
    private BLE ble;
    private BLE.BLEScan activeScan;
    private Thread bleWorkerThread;
    private readonly byte[] txBuffer = new byte[10];

    private Vector3 lastWorldPosition;
    private Vector3 lastWorldVelocity;
    private int warmupTicks = 0; // Requires 2 ticks to seed position -> velocity -> acceleration cleanly
    private float nextBleSendTime = 0f;

    private volatile bool bleConnectedInternal = false;
    private volatile bool pendingStartPacket = false;

    void Awake()
    {
        // Ensure Unity's FixedUpdate runs fast enough to support 100 Hz sampling
        if (sendRateHz > 50f)
        {
            float targetFixedDt = 1f / sendRateHz;
            if (Time.fixedDeltaTime > targetFixedDt)
            {
                Time.fixedDeltaTime = targetFixedDt;
            }
        }
    }

    void Start()
    {
        if (targetTransform == null)
            targetTransform = this.transform;

        targetRb = targetTransform.GetComponentInParent<Rigidbody>();
        ResetKinematicsState();

        // Auto-connect if a device ID or target device name is configured
        if (!string.IsNullOrEmpty(deviceId) || !string.IsNullOrEmpty(targetDeviceName))
        {
            InitComm(deviceId, serviceUuid, characteristicUuid);
        }
    }

    public void ResetKinematicsState()
    {
        warmupTicks = 0;
        rawLateralAccel = 0f;
        smoothedLateralAccel = 0f;
        if (targetTransform != null)
        {
            lastWorldPosition = targetTransform.position;
            lastWorldVelocity = HasActiveRigidbody()
                ? targetRb.GetPointVelocity(targetTransform.position)
                : Vector3.zero;
        }
    }

    private bool HasActiveRigidbody()
    {
        return targetRb != null && !targetRb.isKinematic;
    }

    /// <summary>
    /// Initializes BLE connection. Works with an explicit Windows deviceId OR auto-scans for targetDeviceName.
    /// </summary>
    public void InitComm(string devId, string servUuid, string charUuid)
    {
        deviceId = devId;
        serviceUuid = servUuid;
        characteristicUuid = charUuid;

        if (ble == null) ble = new BLE();

        bleWorkerThread = new Thread(BleConnectionRoutine) { IsBackground = true };
        bleWorkerThread.Start();
    }

    private void BleConnectionRoutine()
    {
        try
        {
            // Step 1: If no Windows deviceId was given, scan and find it by targetDeviceName
            if (string.IsNullOrEmpty(deviceId))
            {
                Debug.Log("[GVS] Scanning for BLE device matching name: " + targetDeviceName);
                AutoResetEvent foundSignal = new AutoResetEvent(false);
                string discoveredId = null;

                activeScan = BLE.ScanDevices();
                activeScan.Found = (id, name) =>
                {
                    if (!string.IsNullOrEmpty(name) &&
                        name.IndexOf(targetDeviceName, StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        discoveredId = id;
                        activeScan.Cancel();
                        foundSignal.Set();
                    }
                };

                // Wait up to 10 seconds to find the ESP32
                foundSignal.WaitOne(10000);
                if (activeScan != null) activeScan.Cancel();

                if (string.IsNullOrEmpty(discoveredId))
                {
                    Debug.LogError("[GVS] Could not find BLE device named: " + targetDeviceName);
                    return;
                }

                deviceId = discoveredId;
                Debug.Log("[GVS] Found target device! ID: " + deviceId);
                Thread.Sleep(200); // Allow scan thread to close cleanly
            }

            // Step 2: Scan Services and match UUID (handles curly braces {} and case differences automatically)
            List<string> services = BLE.ScanAndGetServices(deviceId);
            string matchedService = MatchUuid(serviceUuid, services);
            if (string.IsNullOrEmpty(matchedService))
            {
                Debug.LogError("[GVS] Service UUID not found on device. Searched for: " + serviceUuid);
                return;
            }
            serviceUuid = matchedService;

            // Step 3: Scan Characteristics and match UUID (populates BleWinrtDll internal cache for WritePackage)
            Dictionary<string, string> charMap = BLE.ScanAndGetCharacteristics(deviceId, serviceUuid);
            string matchedChar = MatchUuid(characteristicUuid, charMap.Values);
            if (string.IsNullOrEmpty(matchedChar))
            {
                Debug.LogError("[GVS] Characteristic UUID not found in service. Searched for: " + characteristicUuid);
                return;
            }
            characteristicUuid = matchedChar;

            // Step 4: Optional Subscribe (skipped by default so Write-only characteristics don't fail)
            if (subscribeToNotifications)
            {
                bool subOk = BLE.Subscribe(deviceId, serviceUuid, new string[] { characteristicUuid });
                if (!subOk)
                    Debug.LogWarning("[GVS] Subscribe returned false (characteristic may be Write-only). Continuing with Write mode.");
            }

            ble.isConnected = true;
            pendingStartPacket = true;
            bleConnectedInternal = true;
            Debug.Log($"[GVS] BLE Ready! Service: {serviceUuid} | Characteristic: {characteristicUuid}");
        }
        catch (Exception e)
        {
            Debug.LogError("[GVS] BLE initialization exception: " + e.Message);
        }
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
                return c; // Return the exact string format WinRT expects
        }
        return null;
    }

    // Physics-driven objects (non-kinematic Rigidbody) are sampled in FixedUpdate at 100 Hz
    void FixedUpdate()
    {
        if (targetTransform == null) return;
        if (targetRb == null) targetRb = targetTransform.GetComponentInParent<Rigidbody>();

        if (HasActiveRigidbody())
        {
            // GetPointVelocity includes angular velocity (w x r) if placed on a child seat/camera
            Vector3 currentVel = targetRb.GetPointVelocity(targetTransform.position);
            StepKinematicsAndTransmit(currentVel, Time.fixedDeltaTime);
        }
    }

    // Transform-driven objects (no Rigidbody or kinematic Rigidbody) are sampled in LateUpdate
    // after all Update() movement and VR tracking finishes for the current frame.
    void LateUpdate()
    {
        if (targetTransform == null) return;

        if (!HasActiveRigidbody())
        {
            float dt = Time.deltaTime;
            if (dt <= 1e-6f) return;

            if (warmupTicks == 0)
            {
                lastWorldPosition = targetTransform.position;
                warmupTicks = 1;
                return;
            }

            Vector3 currentVel = (targetTransform.position - lastWorldPosition) / dt;
            lastWorldPosition = targetTransform.position;

            StepKinematicsAndTransmit(currentVel, dt);
        }
    }

    private void StepKinematicsAndTransmit(Vector3 currentWorldVelocity, float dt)
    {
        if (dt <= 1e-6f) return;

        isBleReady = bleConnectedInternal;

        // Seed velocity on the first valid velocity tick to prevent startup impulse spikes
        if (warmupTicks < 2)
        {
            lastWorldVelocity = currentWorldVelocity;
            warmupTicks = 2;
            return;
        }

        // 1. Compute World Acceleration vector (captures both linear and centripetal cornering acceleration)
        Vector3 worldAcceleration = (currentWorldVelocity - lastWorldVelocity) / dt;
        lastWorldVelocity = currentWorldVelocity;

        // 2. Project onto target's local Right axis (+X)
        rawLateralAccel = Vector3.Dot(worldAcceleration, targetTransform.right);

        // 3. Frame-rate-independent exponential low-pass filter
        if (useSmoothing)
        {
            float alpha = 1f - Mathf.Exp(-smoothingSpeed * dt);
            smoothedLateralAccel = Mathf.Lerp(smoothedLateralAccel, rawLateralAccel, alpha);
        }
        else
        {
            smoothedLateralAccel = rawLateralAccel;
        }

        // 4. Rate-limited 100 Hz BLE transmission (dispatched solely from Unity main thread)
        if (bleConnectedInternal)
        {
            if (pendingStartPacket)
            {
                pendingStartPacket = false;
                StartStream();
            }

            if (Time.unscaledTime >= nextBleSendTime)
            {
                nextBleSendTime = Time.unscaledTime + (1f / sendRateHz);
                GVSPacket pkt = ProcessMovement(smoothedLateralAccel);
                SendVRMovementData(pkt);
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

    public bool StartStream()
    {
        if (!bleConnectedInternal) return false;
        return SendVRMovementData(new GVSPacket
        {
            command = CMD_START,
            direction = 0,
            frequency = 0,
            amplitude = 0
        });
    }

    public bool EndStream()
    {
        if (!bleConnectedInternal) return false;
        return SendVRMovementData(new GVSPacket
        {
            command = CMD_STOP,
            direction = 0,
            frequency = 0,
            amplitude = 0
        });
    }

    public bool SendVRMovementData(GVSPacket data)
    {
        if (!bleConnectedInternal || string.IsNullOrEmpty(deviceId)) return false;

        try
        {
            data.WriteToBuffer(txBuffer);
            return BLE.WritePackage(deviceId, serviceUuid, characteristicUuid, txBuffer);
        }
        catch (Exception e)
        {
            Debug.LogError("[GVS] BLE Write failed: " + e.Message);
            return false;
        }
    }

    void OnApplicationQuit()
    {
        if (activeScan != null) activeScan.Cancel();

        if (bleConnectedInternal)
        {
            EndStream();
            bleConnectedInternal = false;
            Thread.Sleep(50); // Allow WinRT to flush the 0 uA STOP packet before closing
        }

        if (ble != null)
        {
            ble.Close();
        }
    }
}