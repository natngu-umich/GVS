using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public class Demo : MonoBehaviour
{
    [Header("State")]
    public bool isScanningDevices = false;
    public bool isSubscribed = false;

    [Header("UI References")]
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

    Transform scanResultRoot;
    public string selectedDeviceId;

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

    // UI exchange variables for dropdowns
    private List<string> newServicesToRender = new List<string>();
    private bool hasNewServices = false;

    private List<string> newCharacteristicsToRender = new List<string>();
    private bool hasNewCharacteristics = false;

    void Start()
    {
        ble = new BLE();
        scanResultRoot = deviceScanResultProto.transform.parent;
        deviceScanResultProto.transform.SetParent(null);
    }

    void Update()
    {
        // 1. Safely render newly discovered devices on the Unity Main Thread
        // Safely render UI updates on the Unity Main Thread
        lock (uiLock)
        {
            // 1. Render newly discovered devices
            if (newDevicesToRender.Count > 0)
            {
                foreach (var device in newDevicesToRender)
                {
                    GameObject g = Instantiate(deviceScanResultProto, scanResultRoot);
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

            // 4. Safely render incoming payload data
            if (hasNewData)
            {
                subcribeText.text = latestReceivedData;
                hasNewData = false;
            }
        }

        // 3. Monitor for BLE Errors
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
            // Clear old UI list
            for (int i = scanResultRoot.childCount - 1; i >= 0; i--)
                Destroy(scanResultRoot.GetChild(i).gameObject);

            selectedDeviceId = null;
            isScanningDevices = true;
            deviceScanButtonText.text = "Stop scan";
            deviceScanStatusText.text = "scanning";

            scan = BLE.ScanDevices();

            // Create a set to track IDs and prevent UI duplicates
            HashSet<string> renderedDeviceIds = new HashSet<string>();

            // Background thread callback
            scan.Found = (id, name) =>
            {
                // 1. Ignore anonymous devices with no name (matching the old demo's behavior)
                if (string.IsNullOrEmpty(name))
                    return;

                lock (uiLock)
                {
                    // 2. Prevent adding the same device to the UI multiple times
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
                // UI updates must happen via variables checked in Update()
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
        // Highlight selection
        for (int i = 0; i < scanResultRoot.transform.childCount; i++)
        {
            var child = scanResultRoot.transform.GetChild(i).gameObject;
            child.transform.GetChild(0).GetComponent<Text>().color = (child == data) ? Color.red : Color.black;
        }

        selectedDeviceId = data.name;
        if (serviceScanButton != null) serviceScanButton.interactable = true;
        if (subscribeButton != null) subscribeButton.interactable = true; // Allow direct connect
    }

    // --------------------------------------------------------------------------------
    // Service & Characteristic Flow
    // The new API handles this automatically inside ble.Connect(). 
    // To keep the UI visually functional without throwing errors, these 
    // buttons now inject your target UUIDs directly into the UI dropdowns.
    // --------------------------------------------------------------------------------
    public string selectedServiceId;
    public string selectedCharacteristicId;
    private Dictionary<string, string> characteristicMap = new Dictionary<string, string>();

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
        }
    }
    // --------------------------------------------------------------------------------

    public void Subscribe()
    {
        if (string.IsNullOrEmpty(selectedDeviceId)) return;

        if (subscribeButton != null) subscribeButton.interactable = false;
        subcribeText.text = "Connecting...";

        connectionThread = new Thread(() =>
        {
            try
            {
                // The new Connect method handles the service and characteristic resolution
                bool success = ble.Connect(selectedDeviceId, selectedServiceId, new string[] { selectedCharacteristicId });
                if (success)
                {
                    isSubscribed = true;
                    keepReading = true;

                    // Spin up the persistent reader loop
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
                    // By default, renders incoming data as ASCII text.
                    latestReceivedData = Encoding.ASCII.GetString(res);

                    // If your GVS ESP32 is sending raw struct bytes (like angles/commands), 
                    // comment out the line above and uncomment the line below:
                    // latestReceivedData = BitConverter.ToString(res); 

                    hasNewData = true;
                }
            }
            Thread.Sleep(10);
        }
    }

    public void Write()
    {
        if (string.IsNullOrEmpty(selectedDeviceId)) return;

        // Grabs text from the UI Input Field and converts it to a byte array
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