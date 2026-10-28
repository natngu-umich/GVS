using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public class Demo : MonoBehaviour
{
    // Command Opcodes
    public const byte CMD_STOP = 0;
    public const byte CMD_START = 1;
    public const byte CMD_DATA = 2;

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
    private GVSPacket currentPacket;

    // Core API references
    BLE ble;
    BLE.BLEScan scan;
    Thread connectionThread;
    Thread readingThread;
    volatile bool keepReading = false;
    string lastError;

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

        InitializeGVSControls();
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
            currentPacket.amplitude = Mathf.RoundToInt(gvsAmplitudeSlider.value);
            if (gvsAmplitudeValueText != null)
                gvsAmplitudeValueText.text = $"{currentPacket.amplitude} uA";
        }

        // Serialize into the 10-byte buffer
        currentPacket.WriteToBuffer(gvsTxBuffer);

        // Update the explicit packet preview text
        if (gvsPacketPreviewText != null)
        {
            string hexBytes = BitConverter.ToString(gvsTxBuffer);
            string cmdName = currentPacket.command == 0 ? "STOP" : (currentPacket.command == 1 ? "START" : "DATA");
            string dirName = currentPacket.direction == 1 ? "L-to-R (+Roll)" : "R-to-L (-Roll)";

            gvsPacketPreviewText.text =
                $"<b>GVSPacket Preview (10 Bytes):</b>\n" +
                $"Cmd: {currentPacket.command} ({cmdName}) | Dir: {currentPacket.direction} ({dirName})\n" +
                $"Freq: {currentPacket.frequency} mHz | Amp: {currentPacket.amplitude} uA\n" +
                $"Raw Hex: [{hexBytes}]";
        }
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

        try
        {
            BLE.WritePackage(selectedDeviceId, selectedServiceId, selectedCharacteristicId, gvsTxBuffer);
            Debug.Log($"Sent GVS Packet: {BitConverter.ToString(gvsTxBuffer)}");
        }
        catch (Exception e)
        {
            Debug.LogError("GVS Write failed: " + e.Message);
        }
    }

    void Update()
    {
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

        // Monitor for BLE Errors
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

            scan = BLE.ScanDevices();
            HashSet<string> renderedDeviceIds = new HashSet<string>();

            scan.Found = (id, name) =>
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
            };

            scan.Finished = () =>
            {
                isScanningDevices = false;
            };
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
        while (keepReading && ble.isConnected)
        {
            byte[] res = BLE.ReadBytes();
            if (res != null && res.Length > 0)
            {
                lock (uiLock)
                {
                    // Renders both ASCII and Raw Hex so you can inspect binary or string replies
                    string ascii = Encoding.ASCII.GetString(res);
                    string hex = BitConverter.ToString(res);
                    latestReceivedData = $"ASCII: {ascii} | Hex: [{hex}]";
                    hasNewData = true;
                }
            }
            Thread.Sleep(10);
        }
    }

    public void Write()
    {
        if (string.IsNullOrEmpty(selectedDeviceId)) return;

        byte[] payload = Encoding.ASCII.GetBytes(writeInput.text);

        try
        {
            BLE.WritePackage(selectedDeviceId, selectedServiceId, selectedCharacteristicId, payload);
        }
        catch (Exception e)
        {
            Debug.LogError("Write failed: " + e.Message);
        }
    }

    private void OnApplicationQuit()
    {
        keepReading = false;
        if (scan != null) scan.Cancel();
        if (ble != null) ble.Close();
    }
}