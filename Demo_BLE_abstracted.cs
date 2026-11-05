using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public class Demo : MonoBehaviour
{
    // Same opcodes, packet and amplitude cap as the game (shared via GVSLink).
    public const byte CMD_STOP = GVSLink.CMD_STOP;
    public const byte CMD_START = GVSLink.CMD_START;
    public const byte CMD_DATA = GVSLink.CMD_DATA;
    public const int HardMaxAmplitudeMicroAmps = GVSLink.HardMaxAmplitudeMicroAmps;

    [Header("State")]
    public bool isScanningDevices = false;
    public bool isSubscribed = false;

    [Header("Screen 1: Scanning UI References")]
    public Text deviceScanButtonText;
    public Text deviceScanStatusText;
    public GameObject deviceScanResultProto;
    public Button serviceScanButton;
    public Text serviceScanStatusText;
    public Dropdown serviceDropdown;
    public Button characteristicScanButton;
    public Text characteristicScanStatusText;
    public Dropdown characteristicDropdown;
    public Button subscribeButton;
    public Text subcribeText;
    public Button writeButton;
    public InputField writeInput;
    public Text errorText;

    [Header("Screen 2: GVS Control UI References")]
    public Dropdown gvsCommandDropdown;
    public Dropdown gvsDirectionDropdown;
    public Slider gvsFrequencySlider;
    public Text gvsFrequencyValueText;
    public Slider gvsAmplitudeSlider;
    public Text gvsAmplitudeValueText;
    public Text gvsPacketPreviewText;
    public Button gvsSendButton;
    public Button gvsSubscribeButton;      // Optional: Subscribe button on Screen 2
    public Text gvsSubscribeText;          // Optional: Incoming data display on Screen 2

    Transform scanResultRoot;
    public string selectedDeviceId;
    public string selectedServiceId;
    public string selectedCharacteristicId;
    private Dictionary<string, string> characteristicMap = new Dictionary<string, string>();

    // Pre-allocated 10-byte buffer and current packet state
    private byte[] gvsTxBuffer = new byte[10];
    private GVSLink.GVSPacket currentPacket;

    // Core API references
    BLE ble;
    BLE.BLEScan scan;
    Thread connectionThread;
    Thread readingThread;
    volatile bool keepReading = false;
    string lastError;
    float nextErrorCheckTime;

    [Header("GVS Streaming (same engine and defaults as GVSController)")]
    [Tooltip("While streaming, amplitude / direction / frequency follow the UI immediately. Off = they only change when you press Send.")]
    public bool liveUpdate = true;
    [Range(20f, 100f)] public float sendRateHz = 100f;
    [Range(30, 500)] public int staleTimeoutMs = 100;
    [Tooltip("Same ramp limit as the game. 10000 = 300 ms from 0 to 3 mA. Raise it to test sharper steps.")]
    public float slewRateMicroAmpsPerSec = 10000f;

    [Header("Live Telemetry (Read-Only)")]
    public bool isStreaming;
    public int targetAmplitudeSigned;
    public int sentAmplitude;
    public int sentDirection;
    public float measuredSendRateHz;
    public int consecutiveWriteFailures;

    // The exact sender the game uses: START handshake, 100 Hz heartbeat thread, slew limit,
    // stale-target zeroing, hard cap, link-loss detection, STOP on shutdown.
    private readonly GVSLink link = new GVSLink("[Demo]");
    private int sentTargetSigned;      // target captured at the last Send (used when liveUpdate is off)
    private int sentFrequency;
    private float nextPreviewTime;
    private volatile bool linkUsed;    // true once Start() has been called on the link

    // Thread-safe UI exchange buffers
    private readonly object uiLock = new object();
    private List<KeyValuePair<string, string>> newDevicesToRender = new List<KeyValuePair<string, string>>();
    private string latestReceivedData = "";
    private bool hasNewData = false;

    private List<string> newServicesToRender = new List<string>();
    private bool hasNewServices = false;

    private List<string> newCharacteristicsToRender = new List<string>();
    private bool hasNewCharacteristics = false;

    void Start()
    {
        ble = new BLE();
        scanResultRoot = deviceScanResultProto.transform.parent;
        deviceScanResultProto.transform.SetParent(null);

        // The read loop below pops notifications itself, so the link must not compete for them.
        link.DrainNotifications = false;
        PushLinkSettings();

        InitializeGVSControls();
    }

    private void PushLinkSettings()
    {
        link.SendRateHz = sendRateHz;
        link.StaleTimeoutMs = staleTimeoutMs;
        link.SlewRateMicroAmpsPerSec = Mathf.Clamp(slewRateMicroAmpsPerSec, 500f, 100000f);
        link.MaxAmplitudeMicroAmps = HardMaxAmplitudeMicroAmps;
        link.Frequency = liveUpdate ? currentPacket.frequency : sentFrequency;
    }

    private int CurrentSignedTarget()
    {
        int amp = Mathf.Clamp(currentPacket.amplitude, 0, HardMaxAmplitudeMicroAmps);
        return currentPacket.direction == 1 ? amp : -amp;
    }

    private void InitializeGVSControls()
    {
        // 1. Setup Command Dropdown (Index 0 = STOP, 1 = START, 2 = DATA)
        if (gvsCommandDropdown != null)
        {
            gvsCommandDropdown.ClearOptions();
            gvsCommandDropdown.AddOptions(new List<string> {
                "0: CMD_STOP",
                "1: CMD_START",
                "2: CMD_DATA"
            });
            gvsCommandDropdown.value = CMD_DATA;
            gvsCommandDropdown.onValueChanged.AddListener(delegate { UpdateGVSPacketFromUI(); });
        }

        // 2. Setup Direction Dropdown (Index 0 = Right-to-Left, 1 = Left-to-Right)
        if (gvsDirectionDropdown != null)
        {
            gvsDirectionDropdown.ClearOptions();
            gvsDirectionDropdown.AddOptions(new List<string> {
                "0: Right-to-Left (-Roll)",
                "1: Left-to-Right (+Roll)"
            });
            gvsDirectionDropdown.value = 0;
            gvsDirectionDropdown.onValueChanged.AddListener(delegate { UpdateGVSPacketFromUI(); });
        }

        // 3. Setup Frequency Slider (mHz)
        if (gvsFrequencySlider != null)
        {
            gvsFrequencySlider.wholeNumbers = true;
            gvsFrequencySlider.onValueChanged.AddListener(delegate { UpdateGVSPacketFromUI(); });
        }

        // 4. Setup Amplitude Slider (uA)
        if (gvsAmplitudeSlider != null)
        {
            gvsAmplitudeSlider.wholeNumbers = true;
            gvsAmplitudeSlider.minValue = 0;
            gvsAmplitudeSlider.maxValue = Mathf.Min(gvsAmplitudeSlider.maxValue, HardMaxAmplitudeMicroAmps);
            gvsAmplitudeSlider.onValueChanged.AddListener(delegate { UpdateGVSPacketFromUI(); });
        }

        UpdateGVSPacketFromUI();
    }

    // Called automatically whenever any GVS Dropdown or Slider changes
    public void UpdateGVSPacketFromUI()
    {
        if (gvsCommandDropdown != null)
            currentPacket.command = (byte)gvsCommandDropdown.value;

        if (gvsDirectionDropdown != null)
            currentPacket.direction = (byte)gvsDirectionDropdown.value;

        if (gvsFrequencySlider != null)
        {
            currentPacket.frequency = Mathf.RoundToInt(gvsFrequencySlider.value);
            if (gvsFrequencyValueText != null)
                gvsFrequencyValueText.text = $"{currentPacket.frequency} mHz";
        }

        if (gvsAmplitudeSlider != null)
        {
            currentPacket.amplitude = Mathf.Clamp(Mathf.RoundToInt(gvsAmplitudeSlider.value), 0, HardMaxAmplitudeMicroAmps);
            if (gvsAmplitudeValueText != null)
                gvsAmplitudeValueText.text = $"{currentPacket.amplitude} uA";
        }

        // Serialize into the 10-byte buffer
        currentPacket.WriteToBuffer(gvsTxBuffer);

        RefreshPreview();
    }

    private void RefreshPreview()
    {
        if (gvsPacketPreviewText == null) return;

        string hexBytes = BitConverter.ToString(gvsTxBuffer);
        string cmdName = currentPacket.command == 0 ? "STOP" : (currentPacket.command == 1 ? "START" : "DATA");
        string dirName = currentPacket.direction == 1 ? "L-to-R (+Roll)" : "R-to-L (-Roll)";
        string live = link.IsStreaming
            ? $"<b>Streaming:</b> {link.MeasuredSendRateHz:F0} Hz | on wire: Amp {link.CurrentOutputAmplitude} uA, Dir {link.CurrentOutputDirection}" +
              (link.LinkLost ? " | LINK LOST" : "")
            : "<b>Not streaming</b> (Send START or DATA to begin)";

        gvsPacketPreviewText.text =
            $"<b>GVSPacket Preview (10 Bytes):</b>\n" +
            $"Cmd: {currentPacket.command} ({cmdName}) | Dir: {currentPacket.direction} ({dirName})\n" +
            $"Freq: {currentPacket.frequency} mHz | Amp: {currentPacket.amplitude} uA\n" +
            $"Raw Hex: [{hexBytes}]\n" + live;
    }

    // Bind this to your Screen 2 "Send" Button OnClick()
    public void SendGVSPacket()
    {
        if (string.IsNullOrEmpty(selectedDeviceId) || string.IsNullOrEmpty(selectedServiceId) || string.IsNullOrEmpty(selectedCharacteristicId))
        {
            Debug.LogWarning("BLE Device, Service, or Characteristic not selected!");
            if (errorText != null) errorText.text = "Select Device, Service & Characteristic first!";
            return;
        }

        UpdateGVSPacketFromUI();
        sentTargetSigned = CurrentSignedTarget();
        sentFrequency = currentPacket.frequency;
        PushLinkSettings();

        string dev = selectedDeviceId, svc = selectedServiceId, chr = selectedCharacteristicId;
        byte cmd = currentPacket.command;

        // Native writes block, so start/stop run on a worker thread, exactly like the game.
        ThreadPool.QueueUserWorkItem(_ =>
        {
            if (cmd == CMD_STOP)
            {
                if (linkUsed)
                {
                    link.Stop(); // ends streaming and sends STOP
                }
                else
                {
                    // Nothing streamed yet: still send a STOP so the button always does what it says.
                    byte[] stop = new byte[GVSLink.GVSPacket.Size];
                    new GVSLink.GVSPacket { command = CMD_STOP }.WriteToBuffer(stop);
                    try { BLE.WritePackage(dev, svc, chr, stop); } catch (Exception e) { Debug.LogError("STOP write failed: " + e.Message); }
                }
            }
            else
            {
                linkUsed = true;
                // START and DATA both go through the game's path: START handshake, then the
                // heartbeat thread streams DATA packets carrying the current target.
                link.SetTarget(sentTargetSigned);
                link.Start(dev, svc, chr);
            }
        });
    }

    void Update()
    {
        // Refresh the target every frame, as the game does. If Unity stalls, the target goes
        // stale and the link ramps to zero by itself.
        PushLinkSettings();
        if (link.IsStreaming)
            link.SetTarget(liveUpdate ? CurrentSignedTarget() : sentTargetSigned);
        link.CheckForHungWrite();

        isStreaming = link.IsStreaming;
        targetAmplitudeSigned = liveUpdate ? CurrentSignedTarget() : sentTargetSigned;
        sentAmplitude = link.CurrentOutputAmplitude;
        sentDirection = link.CurrentOutputDirection;
        measuredSendRateHz = link.MeasuredSendRateHz;
        consecutiveWriteFailures = link.ConsecutiveWriteFailures;

        if (Time.unscaledTime >= nextPreviewTime)
        {
            nextPreviewTime = Time.unscaledTime + 0.1f;
            RefreshPreview();
        }

        lock (uiLock)
        {
            // 1. Render newly discovered devices
            if (newDevicesToRender.Count > 0)
            {
                foreach (var device in newDevicesToRender)
                {
                    GameObject g = Instantiate(deviceScanResultProto, scanResultRoot, false);
                    g.transform.localScale = Vector3.one;
                    g.name = device.Key;
                    g.transform.GetChild(0).GetComponent<Text>().text = device.Value;
                    g.transform.GetChild(1).GetComponent<Text>().text = device.Key;

                    Button btn = g.GetComponent<Button>();
                    if (btn != null) btn.onClick.AddListener(() => SelectDevice(g));
                }
                newDevicesToRender.Clear();
            }

            // 2. Render new Services to the dropdown
            if (hasNewServices)
            {
                serviceDropdown.ClearOptions();
                serviceDropdown.AddOptions(newServicesToRender);
                if (serviceScanStatusText != null) serviceScanStatusText.text = "finished";

                if (newServicesToRender.Count > 0)
                {
                    serviceDropdown.value = 0;
                    SelectService(null);
                }
                hasNewServices = false;
            }

            // 3. Render new Characteristics to the dropdown
            if (hasNewCharacteristics)
            {
                characteristicDropdown.ClearOptions();
                characteristicDropdown.AddOptions(newCharacteristicsToRender);
                if (characteristicScanStatusText != null) characteristicScanStatusText.text = "finished";

                if (newCharacteristicsToRender.Count > 0)
                {
                    characteristicDropdown.value = 0;
                    SelectCharacteristic(null);
                }
                hasNewCharacteristics = false;
            }

            // 4. Safely render incoming payload data on both screens
            if (hasNewData)
            {
                if (subcribeText != null) subcribeText.text = latestReceivedData;
                if (gvsSubscribeText != null) gvsSubscribeText.text = latestReceivedData;
                hasNewData = false;
            }
        }

        // Monitor for BLE Errors (a few times a second is plenty; each call allocates a 1 KB string)
        if (Time.unscaledTime < nextErrorCheckTime) return;
        nextErrorCheckTime = Time.unscaledTime + 0.25f;
        string currentError = BLE.GetError();
        if (currentError != "Ok" && lastError != currentError)
        {
            Debug.LogError(currentError);
            if (errorText != null) errorText.text = currentError;
            lastError = currentError;
        }
    }

    public void StartStopDeviceScan()
    {
        if (!isScanningDevices)
        {
            for (int i = scanResultRoot.childCount - 1; i >= 0; i--)
                Destroy(scanResultRoot.GetChild(i).gameObject);

            selectedDeviceId = null;
            isScanningDevices = true;
            deviceScanButtonText.text = "Stop scan";
            deviceScanStatusText.text = "scanning";

            HashSet<string> renderedDeviceIds = new HashSet<string>();

            // Callbacks are passed in so they are attached before the scan starts (no missed devices).
            try
            {
                scan = BLE.ScanDevices(
                    (id, name) =>
                    {
                        if (string.IsNullOrEmpty(name))
                            return;

                        lock (uiLock)
                        {
                            if (!renderedDeviceIds.Contains(id))
                            {
                                renderedDeviceIds.Add(id);
                                newDevicesToRender.Add(new KeyValuePair<string, string>(id, name));
                            }
                        }
                    },
                    () => { isScanningDevices = false; });
            }
            catch (InvalidOperationException)
            {
                // The previous scan thread is still shutting down.
                isScanningDevices = false;
                deviceScanButtonText.text = "Start scan";
                deviceScanStatusText.text = "busy, try again";
            }
        }
        else
        {
            if (scan != null) scan.Cancel();
            isScanningDevices = false;
            deviceScanButtonText.text = "Start scan";
            deviceScanStatusText.text = "stopped";
        }
    }

    public void SelectDevice(GameObject data)
    {
        for (int i = 0; i < scanResultRoot.transform.childCount; i++)
        {
            var child = scanResultRoot.transform.GetChild(i).gameObject;
            child.transform.GetChild(0).GetComponent<Text>().color = (child == data) ? Color.red : Color.black;
        }

        selectedDeviceId = data.name;
        if (serviceScanButton != null) serviceScanButton.interactable = true;
        if (subscribeButton != null) subscribeButton.interactable = true;
        if (gvsSubscribeButton != null) gvsSubscribeButton.interactable = true;
    }

    public void StartServiceScan()
    {
        if (string.IsNullOrEmpty(selectedDeviceId)) return;

        serviceScanButton.interactable = false;
        serviceScanStatusText.text = "scanning";

        new Thread(() =>
        {
            List<string> services = BLE.ScanAndGetServices(selectedDeviceId);

            lock (uiLock)
            {
                newServicesToRender = services;
                hasNewServices = true;
            }
        })
        { IsBackground = true }.Start();
    }

    public void StartCharacteristicScan()
    {
        if (string.IsNullOrEmpty(selectedDeviceId) || string.IsNullOrEmpty(selectedServiceId)) return;

        characteristicScanButton.interactable = false;
        characteristicScanStatusText.text = "scanning";

        new Thread(() =>
        {
            characteristicMap = BLE.ScanAndGetCharacteristics(selectedDeviceId, selectedServiceId);
            List<string> charNames = new List<string>(characteristicMap.Keys);

            lock (uiLock)
            {
                newCharacteristicsToRender = charNames;
                hasNewCharacteristics = true;
            }
        })
        { IsBackground = true }.Start();
    }

    public void SelectService(GameObject data)
    {
        if (serviceDropdown.options.Count > 0)
        {
            selectedServiceId = serviceDropdown.options[serviceDropdown.value].text;
            if (characteristicScanButton != null) characteristicScanButton.interactable = true;
        }
    }

    public void SelectCharacteristic(GameObject data)
    {
        if (characteristicDropdown.options.Count > 0)
        {
            string selectedName = characteristicDropdown.options[characteristicDropdown.value].text;
            selectedCharacteristicId = characteristicMap[selectedName];

            if (subscribeButton != null) subscribeButton.interactable = true;
            if (writeButton != null) writeButton.interactable = true;
            if (gvsSendButton != null) gvsSendButton.interactable = true;
            if (gvsSubscribeButton != null) gvsSubscribeButton.interactable = true;
        }
    }

    public void Subscribe()
    {
        if (string.IsNullOrEmpty(selectedDeviceId)) return;

        if (subscribeButton != null) subscribeButton.interactable = false;
        if (gvsSubscribeButton != null) gvsSubscribeButton.interactable = false;
        if (subcribeText != null) subcribeText.text = "Connecting...";
        if (gvsSubscribeText != null) gvsSubscribeText.text = "Connecting...";

        connectionThread = new Thread(() =>
        {
            try
            {
                bool success = ble.Connect(selectedDeviceId, selectedServiceId, new string[] { selectedCharacteristicId });
                if (success)
                {
                    isSubscribed = true;
                    keepReading = true;

                    lock (uiLock)
                    {
                        latestReceivedData = "Connected & Subscribed";
                        hasNewData = true;
                    }

                    readingThread = new Thread(ReadLoop) { IsBackground = true };
                    readingThread.Start();
                }
            }
            catch (Exception e)
            {
                Debug.LogError("Connection failed: " + e.Message);
            }
        })
        { IsBackground = true };
        connectionThread.Start();

        if (writeButton != null) writeButton.interactable = true;
        if (gvsSendButton != null) gvsSendButton.interactable = true;
    }

    private void ReadLoop()
    {
        BLE b = ble; // local copy: Shutdown() nulls the field from another thread
        while (keepReading && b != null && b.isConnected)
        {
            // Pops queued notifications without blocking (BLE.ReadBytes does not exist).
            BLE.DrainNotifications(16, (buf, len) =>
            {
                lock (uiLock)
                {
                    // Renders both ASCII and Raw Hex so you can inspect binary or string replies
                    string ascii = Encoding.ASCII.GetString(buf, 0, len);
                    string hex = BitConverter.ToString(buf, 0, len);
                    latestReceivedData = $"ASCII: {ascii} | Hex: [{hex}]";
                    hasNewData = true;
                }
            });
            Thread.Sleep(10);
        }
    }

    public void Write()
    {
        Debug.Log("we are inside of the write function");
        if (string.IsNullOrEmpty(selectedDeviceId)) return;
        Debug.Log("about to make payload");
        byte[] payload = Encoding.ASCII.GetBytes(writeInput.text);
        if (payload.Length == 0 || payload.Length > BLE.MaxPayload) return;

        string dev = selectedDeviceId, svc = selectedServiceId, chr = selectedCharacteristicId;
        ThreadPool.QueueUserWorkItem(_ =>
        {
            try
            {
                if (!BLE.WritePackage(dev, svc, chr, payload))
                    Debug.LogWarning("Write failed: " + BLE.GetError());
            }
            catch (Exception e)
            {
                Debug.LogError("Write failed: " + e.Message);
            }
        });
    }

    private int shutdownDone = 0;
    private void OnApplicationQuit() { Shutdown(); }
    private void OnDestroy() { Shutdown(); }
    private void OnDisable() { link.Muted = true; }
    private void OnEnable() { link.Muted = false; }

    private void Shutdown()
    {
        if (Interlocked.Exchange(ref shutdownDone, 1) == 1) return;

        keepReading = false;
        link.Muted = true;
        if (scan != null) scan.Cancel();

        BLE bleToClose = ble;
        ble = null;
        Thread reader = readingThread;

        // Native calls on a worker so a stuck write can't freeze Unity on exit.
        Thread teardown = new Thread(() =>
        {
            if (reader != null) reader.Join(500);
            bool safeToQuit = link.Stop(1000); // stops streaming, sends STOP (if a device was used)
            if (safeToQuit && bleToClose != null) bleToClose.Close();
        })
        { IsBackground = true };
        teardown.Start();
        teardown.Join(3000);
    }
}