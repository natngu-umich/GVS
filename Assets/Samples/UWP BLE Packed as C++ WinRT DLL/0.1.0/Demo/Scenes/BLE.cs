using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using UnityEngine;

public class BLE
{
    // Native DLL P/Invoke bindings matching updated BleWinrtDll
    class Impl
    {
        public enum ScanStatus { PROCESSING, AVAILABLE, FINISHED };

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        public struct DeviceUpdate
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string id;
            [MarshalAs(UnmanagedType.I1)]
            public bool isConnectable;
            [MarshalAs(UnmanagedType.I1)]
            public bool isConnectableUpdated;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 50)]
            public string name;
            [MarshalAs(UnmanagedType.I1)]
            public bool nameUpdated;
        }

        [DllImport("BleWinrtDll.dll", EntryPoint = "StartDeviceScan")]
        public static extern void StartDeviceScan();

        // UPDATED: 'ref' instead of 'out'
        [DllImport("BleWinrtDll.dll", EntryPoint = "PollDevice")]
        public static extern ScanStatus PollDevice(ref DeviceUpdate device, bool block);

        [DllImport("BleWinrtDll.dll", EntryPoint = "StopDeviceScan")]
        public static extern void StopDeviceScan();

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        public struct Service
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string uuid;
        };

        [DllImport("BleWinrtDll.dll", EntryPoint = "ScanServices", CharSet = CharSet.Unicode)]
        public static extern void ScanServices(string deviceId);

        [DllImport("BleWinrtDll.dll", EntryPoint = "PollService")]
        public static extern ScanStatus PollService(out Service service, bool block);

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        public struct Characteristic
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string uuid;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string userDescription;
        };

        [DllImport("BleWinrtDll.dll", EntryPoint = "ScanCharacteristics", CharSet = CharSet.Unicode)]
        public static extern void ScanCharacteristics(string deviceId, string serviceId);

        [DllImport("BleWinrtDll.dll", EntryPoint = "PollCharacteristic")]
        public static extern ScanStatus PollCharacteristic(out Characteristic characteristic, bool block);

        // UPDATED: Added bool block parameter
        [DllImport("BleWinrtDll.dll", EntryPoint = "SubscribeCharacteristic", CharSet = CharSet.Unicode)]
        public static extern bool SubscribeCharacteristic(string deviceId, string serviceId, string characteristicId, bool block);

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        public struct BLEData
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 512)]
            public byte[] buf;
            [MarshalAs(UnmanagedType.I2)]
            public short size;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
            public string deviceId;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
            public string serviceUuid;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
            public string characteristicUuid;
        };

        [DllImport("BleWinrtDll.dll", EntryPoint = "PollData")]
        public static extern bool PollData(out BLEData data, bool block);

        // UPDATED: 'in' reference parameter and added bool block parameter
        [DllImport("BleWinrtDll.dll", EntryPoint = "SendData")]
        public static extern bool SendData(in BLEData data, bool block);

        [DllImport("BleWinrtDll.dll", EntryPoint = "Quit")]
        public static extern void Quit();

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        public struct ErrorMessage
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
            public string msg;
        };

        [DllImport("BleWinrtDll.dll", EntryPoint = "GetError")]
        public static extern void GetError(out ErrorMessage buf);
    }

    public static Thread scanThread;
    public static BLEScan currentScan = new BLEScan();
    public bool isConnected = false;

    public class BLEScan
    {
        public delegate void FoundDel(string deviceId, string deviceName);
        public delegate void FinishedDel();
        public FoundDel Found;
        public FinishedDel Finished;
        internal bool cancelled = false;

        public void Cancel()
        {
            cancelled = true;
            Impl.StopDeviceScan();
        }
    }

    public static BLEScan ScanDevices()
    {
        if (scanThread == Thread.CurrentThread)
            throw new InvalidOperationException("a new scan can not be started from a callback of the previous scan");
        else if (scanThread != null)
            throw new InvalidOperationException("the old scan is still running");
        currentScan.Found = null;
        currentScan.Finished = null;
        scanThread = new Thread(() =>
        {
            Impl.StartDeviceScan();
            Impl.DeviceUpdate res = new Impl.DeviceUpdate();
            List<string> deviceIds = new List<string>();
            Dictionary<string, string> deviceNames = new Dictionary<string, string>();

            // UPDATED: ref res instead of out res
            while (Impl.PollDevice(ref res, true) != Impl.ScanStatus.FINISHED)
            {
                if (res.nameUpdated)
                {
                    deviceIds.Add(res.id);
                    if (!deviceNames.ContainsKey(res.id))
                        deviceNames.Add(res.id, res.name);
                    else
                        deviceNames[res.id] = res.name;
                }

                if (deviceIds.Contains(res.id) && res.isConnectable)
                    currentScan.Found?.Invoke(res.id, deviceNames[res.id]);

                if (currentScan.cancelled)
                    break;
            }
            currentScan.Finished?.Invoke();
            scanThread = null;
        });
        scanThread.Start();
        return currentScan;
    }

    public static List<string> ScanAndGetServices(string deviceId)
    {
        List<string> found = new List<string>();
        Impl.ScanServices(deviceId);
        Impl.Service service = new Impl.Service();
        // Uses block = true to wait for all services to be returned
        while (Impl.PollService(out service, true) != Impl.ScanStatus.FINISHED)
        {
            found.Add(service.uuid);
        }
        return found;
    }

    public static Dictionary<string, string> ScanAndGetCharacteristics(string deviceId, string serviceId)
    {
        Dictionary<string, string> found = new Dictionary<string, string>();
        Impl.ScanCharacteristics(deviceId, serviceId);
        Impl.Characteristic c = new Impl.Characteristic();
        while (Impl.PollCharacteristic(out c, true) != Impl.ScanStatus.FINISHED)
        {
            string name = c.userDescription != "no description available" ? c.userDescription : c.uuid;
            found[name] = c.uuid;
        }
        return found;
    }

    public static void RetrieveProfile(string deviceId, string serviceUuid)
    {
        Impl.ScanServices(deviceId);
        Impl.Service service = new Impl.Service();
        while (Impl.PollService(out service, true) != Impl.ScanStatus.FINISHED)
            Debug.Log("service found: " + service.uuid);

        Thread.Sleep(200);
        Impl.ScanCharacteristics(deviceId, serviceUuid);
        Impl.Characteristic c = new Impl.Characteristic();
        while (Impl.PollCharacteristic(out c, true) != Impl.ScanStatus.FINISHED)
            Debug.Log("characteristic found: " + c.uuid + ", user description: " + c.userDescription);
    }

    public static bool Subscribe(string deviceId, string serviceUuids, string[] characteristicUuids)
    {
        foreach (string characteristicUuid in characteristicUuids)
        {
            // UPDATED: Passes block = true to wait for GATT CCCD descriptor confirmation
            bool res = Impl.SubscribeCharacteristic(deviceId, serviceUuids, characteristicUuid, true);
            if (!res)
                return false;
        }
        return true;
    }

    public bool Connect(string deviceId, string serviceUuid, string[] characteristicUuids)
    {
        if (isConnected)
            return false;
        Debug.Log("retrieving ble profile...");
        RetrieveProfile(deviceId, serviceUuid);
        if (GetError() != "Ok")
            throw new Exception("Connection failed: " + GetError());
        Debug.Log("subscribing to characteristics...");
        bool result = Subscribe(deviceId, serviceUuid, characteristicUuids);
        if (GetError() != "Ok" || !result)
            throw new Exception("Connection failed: " + GetError());
        isConnected = true;
        return true;
    }

    public static bool WritePackage(string deviceId, string serviceUuid, string characteristicUuid, byte[] data)
    {
        if (data == null || data.Length > 512)
            throw new ArgumentOutOfRangeException("Payload must be non-null and <= 512 bytes.");

        Impl.BLEData packageSend;
        packageSend.buf = new byte[512];
        packageSend.size = (short)data.Length;
        packageSend.deviceId = deviceId;
        packageSend.serviceUuid = serviceUuid;
        packageSend.characteristicUuid = characteristicUuid;

        // FIXED: Copy payload into transmission buffer
        Buffer.BlockCopy(data, 0, packageSend.buf, 0, data.Length);

        // UPDATED: in packageSend and block = false (non-blocking transmit)
        return Impl.SendData(in packageSend, false);
    }

    public static void ReadPackage()
    {
        Impl.BLEData packageReceived;
        bool result = Impl.PollData(out packageReceived, true);
        if (result)
        {
            if (packageReceived.size > 512)
                throw new ArgumentOutOfRangeException("Package exceeds 512 bytes.");
            Debug.Log("received package from characteristic: " + packageReceived.characteristicUuid
                + " and size " + packageReceived.size);
        }
    }

    public static byte[] ReadBytes()
    {
        Impl.BLEData packageReceived;
        bool result = Impl.PollData(out packageReceived, true);

        if (result && packageReceived.size > 0)
        {
            if (packageReceived.size > 512)
                throw new ArgumentOutOfRangeException("Package too large.");

            // FIXED: Return exactly the bytes received rather than the full 512-byte buffer
            byte[] actualData = new byte[packageReceived.size];
            Buffer.BlockCopy(packageReceived.buf, 0, actualData, 0, packageReceived.size);
            return actualData;
        }
        else
        {
            return new byte[0];
        }
    }

    public void Close()
    {
        Impl.Quit();
        isConnected = false;
    }

    public static string GetError()
    {
        Impl.ErrorMessage buf;
        Impl.GetError(out buf);
        return buf.msg;
    }

    ~BLE()
    {
        Close();
    }
}