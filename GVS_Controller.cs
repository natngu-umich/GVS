using System;
using System.Threading;
using UnityEngine;

public class GVSController : MonoBehaviour
{
    [Header("Tracking Target")]
    [Tooltip("Drag the Vehicle or VR Camera Rig here. Defaults to this GameObject if empty.")]
    public Transform targetTransform;
    private Rigidbody targetRb;

    [Header("BLE Configuration")]
    public string deviceId = ""; // Populate from your Demo UI or hardcode your ESP32 ID
    public string serviceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
    public string characteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
    public float sendRateHz = 20f; // 20 Hz (every 50ms) prevents flooding the Windows BLE stack

    [Header("GVS Motion Mapping (Roll Axis)")]
    [Tooltip("Minimum lateral acceleration (m/s^2) before GVS activates.")]
    public float accelDeadzone = 0.5f;
    [Tooltip("Lateral acceleration (m/s^2) that maps to maximum GVS amplitude.")]
    public float maxExpectedAccel = 15.0f;
    [Tooltip("Low-pass filter smoothing (higher = more responsive, lower = smoother).")]
    [Range(1f, 30f)] public float smoothingSpeed = 10f;

    [Header("Hardware Limits")]
    [Tooltip("Maximum safe current amplitude in microamps (e.g., 3000 uA = 3.0 mA).")]
    public int maxAmplitudeMicroAmps = 3000;
    public int defaultFrequencyMHz = 0; // 0 for DC step/ramp, or set AC frequency in mHz

    [Header("Debug Telemetry (Read-Only)")]
    public float rawLateralAccel;
    public float smoothedLateralAccel;
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
        public int amplitude;    // uA or DAC code (4 bytes, Little-Endian)

        public byte[] ToByteArray()
        {
            byte[] buf = new byte[10];
            buf[0] = command;
            buf[1] = direction;
            Buffer.BlockCopy(BitConverter.GetBytes(frequency), 0, buf, 2, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(amplitude), 0, buf, 6, 4);
            return buf;
        }
    }

    // Internal state
    private BLE ble;
    private Vector3 lastWorldPosition;
    private Vector3 lastWorldVelocity;
    private bool isFirstPhysicsTick = true;
    private float nextBleSendTime = 0f;

    void Start()
    {
        if (targetTransform == null)
            targetTransform = this.transform;

        targetRb = targetTransform.GetComponentInParent<Rigidbody>();

        lastWorldPosition = targetTransform.position;
        lastWorldVelocity = (targetRb != null) ? targetRb.velocity : Vector3.zero;

        if (!string.IsNullOrEmpty(deviceId))
        {
            InitComm(deviceId, serviceUuid, characteristicUuid);
        }
    }

    /// <summary>
    /// Call this from Start() or from your Demo UI once a device is selected.
    /// </summary>
    public void InitComm(string devId, string servUuid, string charUuid)
    {
        deviceId = devId;
        serviceUuid = servUuid;
        characteristicUuid = charUuid;

        ble = new BLE();

        // Connect on a background thread so Unity doesn't freeze during GATT resolution
        new Thread(() =>
        {
            try
            {
                bool connected = ble.Connect(deviceId, serviceUuid, new string[] { characteristicUuid });
                if (connected)
                {
                    isBleReady = true;
                    StartStream();
                    Debug.Log("[GVS] BLE Connected and Stream Started.");
                }
            }
            catch (Exception e)
            {
                Debug.LogError("[GVS] BLE Connection Failed: " + e.Message);
            }
        })
        { IsBackground = true }.Start();
    }

    public bool StartStream()
    {
        if (!isBleReady) return false;
        GVSPacket startPkt = new GVSPacket
        {
            command = CMD_START,
            direction = 0,
            frequency = 0,
            amplitude = 0
        };
        return SendVRMovementData(startPkt);
    }

    void FixedUpdate()
    {
        if (targetTransform == null) return;

        float dt = Time.fixedDeltaTime;
        if (dt <= 0f) return;

        // 1. Sample current World Velocity (works with or without a Rigidbody)
        Vector3 currentWorldVelocity = (targetRb != null && !targetRb.isKinematic)
            ? targetRb.velocity
            : (targetTransform.position - lastWorldPosition) / dt;

        lastWorldPosition = targetTransform.position;

        if (isFirstPhysicsTick)
        {
            lastWorldVelocity = currentWorldVelocity;
            isFirstPhysicsTick = false;
            return;
        }

        // 2. Compute World Acceleration (captures both linear and centripetal turning forces)
        Vector3 worldAcceleration = (currentWorldVelocity - lastWorldVelocity) / dt;
        lastWorldVelocity = currentWorldVelocity;

        // 3. Project onto the target's Local X-axis (Right vector) for lateral/roll stimulation
        rawLateralAccel = Vector3.Dot(worldAcceleration, targetTransform.right);

        // 4. Low-pass filter to remove physics jitter
        smoothedLateralAccel = Mathf.Lerp(smoothedLateralAccel, rawLateralAccel, smoothingSpeed * dt);

        // 5. Rate-limited BLE transmission
        if (isBleReady && Time.time >= nextBleSendTime)
        {
            nextBleSendTime = Time.time + (1f / sendRateHz);
            GVSPacket dataPkt = ProcessMovement(smoothedLateralAccel);
            SendVRMovementData(dataPkt);
        }
    }

    public GVSPacket ProcessMovement(float lateralAccel)
    {
        float absAccel = Mathf.Abs(lateralAccel);

        // Apply deadzone so straight-line driving outputs 0 uA
        if (absAccel < accelDeadzone)
        {
            return new GVSPacket
            {
                command = CMD_DATA,
                direction = 0,
                frequency = defaultFrequencyMHz,
                amplitude = 0
            };
        }

        // Normalize acceleration between [accelDeadzone, maxExpectedAccel] -> [0.0, 1.0]
        float normalized = Mathf.InverseLerp(accelDeadzone, maxExpectedAccel, absAccel);
        int targetAmplitude = Mathf.RoundToInt(normalized * maxAmplitudeMicroAmps);

        return new GVSPacket
        {
            command = CMD_DATA,
            direction = (byte)(lateralAccel > 0f ? 1 : 0), // 1 = Rightward (+X), 0 = Leftward (-X)
            frequency = defaultFrequencyMHz,
            amplitude = targetAmplitude
        };
    }

    public bool SendVRMovementData(GVSPacket data)
    {
        if (!isBleReady || string.IsNullOrEmpty(deviceId)) return false;

        try
        {
            byte[] payload = data.ToByteArray();
            return BLE.WritePackage(deviceId, serviceUuid, characteristicUuid, payload);
        }
        catch (Exception e)
        {
            Debug.LogError("[GVS] Write failed: " + e.Message);
            return false;
        }
    }

    public bool EndStream()
    {
        if (!isBleReady) return false;
        GVSPacket stopPkt = new GVSPacket
        {
            command = CMD_STOP,
            direction = 0,
            frequency = 0,
            amplitude = 0
        };
        return SendVRMovementData(stopPkt);
    }

    void OnApplicationQuit()
    {
        // Always zero out current before closing the BLE link
        if (isBleReady)
        {
            EndStream();
            Thread.Sleep(50); // Brief pause so WinRT flushes the STOP packet
        }
        if (ble != null)
        {
            ble.Close();
        }
    }
}