using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public class BleTest : MonoBehaviour
{
    // Configure to match your target peripheral
    [Header("BLE Configuration")]
    public string targetDeviceName = "EnterDeviceNameHere";
    public string serviceUuid = "{2a2b1072-5199-11eb-ae93-0242ac130002}";
    public string[] characteristicUuids = {
        "{59c2f246-5199-11eb-ae93-0242ac130002}",
        "{617c753e-5199-11eb-ae93-0242ac130002}"
    };

    BLE ble;
    BLE.BLEScan scan;
    bool isScanning = false;
    bool isConnected = false;
    string deviceId = null;

    private readonly object deviceLock = new object();
    private Dictionary<string, string> discoveredDevices = new Dictionary<string, string>();
    private int devicesCount = 0;

    // Worker Threads & Cancellation
    Thread scanningThread, connectionThread, readingThread;
    volatile bool keepReading = false;

    // GUI references
    [Header("UI References")]
    public Text TextDiscoveredDevices;
    public Text TextIsScanning;
    public Text TextTargetDeviceConnection;
    public Text TextTargetDeviceData;
    public Button ButtonEstablishConnection;
    public Button ButtonStartScan;

    int remoteAngle, lastRemoteAngle;

    void Start()
    {
        ble = new BLE();
        if (ButtonEstablishConnection != null)
            ButtonEstablishConnection.enabled = false;

        if (TextTargetDeviceConnection != null)
            TextTargetDeviceConnection.text = targetDeviceName + " not found.";
    }

    void Update()
    {
        if (isScanning)
        {
            if (ButtonStartScan != null && ButtonStartScan.enabled)
                ButtonStartScan.enabled = false;

            int currentCount;
            lock (deviceLock)
            {
                currentCount = discoveredDevices.Count;
            }

            if (currentCount > devicesCount)
            {
                UpdateGuiText("scan");
                devicesCount = currentCount;
            }
        }
        else
        {
            if (TextIsScanning != null && TextIsScanning.text != "Not scanning.")
            {
                TextIsScanning.color = Color.white;
                TextIsScanning.text = "Not scanning.";
            }
        }

        // Target device connection flow
        if (deviceId != null && deviceId != "-1")
        {
            if (ble != null && ble.isConnected && isConnected)
            {
                UpdateGuiText("writeData");
            }
            else if (ble != null && ble.isConnected && !isConnected)
            {
                isConnected = true;
                UpdateGuiText("connected");

                // Start persistent background reader loop
                keepReading = true;
                readingThread = new Thread(ReadBleDataLoop);
                readingThread.IsBackground = true;
                readingThread.Start();
            }
            else if (ButtonEstablishConnection != null && !ButtonEstablishConnection.enabled && !isConnected)
            {
                ButtonEstablishConnection.enabled = true;
                TextTargetDeviceConnection.text = "Found target device:\n" + targetDeviceName;
            }
        }
    }

    private void OnDestroy()
    {
        CleanUp();
    }

    private void OnApplicationQuit()
    {
        CleanUp();
    }

    private void CleanUp()
    {
        keepReading = false;

        try
        {
            if (scan != null)
            {
                scan.Cancel();
                scan = null;
            }

            if (ble != null)
            {
                ble.Close();
            }

            if (scanningThread != null && scanningThread.IsAlive)
            {
                scanningThread.Abort();
                scanningThread = null;
            }

            if (connectionThread != null && connectionThread.IsAlive)
            {
                connectionThread.Abort();
                connectionThread = null;
            }

            if (readingThread != null && readingThread.IsAlive)
            {
                readingThread.Abort();
                readingThread = null;
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning("Exception caught during BLE cleanup: " + e.Message);
        }
    }

    public void StartScanHandler()
    {
        devicesCount = 0;
        isScanning = true;
        lock (deviceLock)
        {
            discoveredDevices.Clear();
        }

        if (TextIsScanning != null)
        {
            TextIsScanning.color = new Color(244f / 255f, 180f / 255f, 26f / 255f);
            TextIsScanning.text = "Scanning...";
        }

        if (TextDiscoveredDevices != null)
            TextDiscoveredDevices.text = "";

        scanningThread = new Thread(ScanBleDevices);
        scanningThread.IsBackground = true;
        scanningThread.Start();
    }

    public void ResetHandler()
    {
        if (TextTargetDeviceData != null)
            TextTargetDeviceData.text = "";

        if (TextTargetDeviceConnection != null)
            TextTargetDeviceConnection.text = targetDeviceName + " not found.";

        lock (deviceLock)
        {
            discoveredDevices.Clear();
        }

        if (TextDiscoveredDevices != null)
            TextDiscoveredDevices.text = "No devices.";

        deviceId = null;
        isConnected = false;
        CleanUp();
    }

    // Persistent reader loop - avoids re-instantiating threads every frame
    private void ReadBleDataLoop()
    {
        while (keepReading && ble != null && ble.isConnected)
        {
            byte[] packageReceived = BLE.ReadBytes();
            if (packageReceived != null && packageReceived.Length > 0)
            {
                remoteAngle = packageReceived[0];
            }
            Thread.Sleep(10); // Throttle polling rate (~100 Hz)
        }
    }

    void UpdateGuiText(string action)
    {
        switch (action)
        {
            case "scan":
                if (TextDiscoveredDevices != null)
                {
                    TextDiscoveredDevices.text = "";
                    lock (deviceLock)
                    {
                        foreach (KeyValuePair<string, string> entry in discoveredDevices)
                        {
                            TextDiscoveredDevices.text += "DeviceID: " + entry.Key + "\nDeviceName: " + entry.Value + "\n\n";
                        }
                    }
                }
                break;

            case "connected":
                if (ButtonEstablishConnection != null)
                    ButtonEstablishConnection.enabled = false;
                if (TextTargetDeviceConnection != null)
                    TextTargetDeviceConnection.text = "Connected to target device:\n" + targetDeviceName;
                break;

            case "writeData":
                if (TextTargetDeviceData != null && remoteAngle != lastRemoteAngle)
                {
                    TextTargetDeviceData.text = "Remote angle: " + remoteAngle;
                    lastRemoteAngle = remoteAngle;
                }
                break;
        }
    }

    void ScanBleDevices()
    {
        scan = BLE.ScanDevices();
        Debug.Log("BLE.ScanDevices() started.");

        scan.Found = (_deviceId, deviceName) =>
        {
            string name = string.IsNullOrEmpty(deviceName) ? "[Unnamed]" : deviceName;
            Debug.Log($"Found: {name} ({_deviceId})");

            lock (deviceLock)
            {
                if (!discoveredDevices.ContainsKey(_deviceId))
                    discoveredDevices.Add(_deviceId, name);
            }

            if (deviceId == null && name == targetDeviceName)
            {
                deviceId = _deviceId;
                Debug.Log("Target device matched: " + deviceName);
            }
        };

        scan.Finished = () =>
        {
            isScanning = false;
            Debug.Log("Scan finished.");
            if (deviceId == null)
                deviceId = "-1";
        };

        // Wait until device is discovered or scan finishes
        while (deviceId == null && isScanning)
        {
            Thread.Sleep(200);
        }

        if (scan != null)
            scan.Cancel();

        scanningThread = null;
        isScanning = false;

        if (deviceId == "-1")
        {
            Debug.Log("Target device not found in this scan.");
        }
    }

    public void StartConHandler()
    {
        connectionThread = new Thread(ConnectBleDevice);
        connectionThread.IsBackground = true;
        connectionThread.Start();
    }

    void ConnectBleDevice()
    {
        if (deviceId != null && deviceId != "-1")
        {
            try
            {
                ble.Connect(deviceId, serviceUuid, characteristicUuids);
            }
            catch (Exception e)
            {
                Debug.LogError("Could not connect to device " + deviceId + ": " + e.Message);
            }
        }

        if (ble.isConnected)
        {
            Debug.Log("Connected to: " + targetDeviceName);
        }
    }
}