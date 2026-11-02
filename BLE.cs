using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;

public class BLE
{
    public class Impl
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

        [DllImport("BleWinrtDll.dll", EntryPoint = "PollDevice")]
        public static extern ScanStatus PollDevice(ref DeviceUpdate device, bool block);

        [DllImport("BleWinrtDll.dll", EntryPoint = "StopDeviceScan")]
        public static extern void StopDeviceScan();

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        public struct Service
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string uuid;
        }

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
        }

        [DllImport("BleWinrtDll.dll", EntryPoint = "ScanCharacteristics", CharSet = CharSet.Unicode)]
        public static extern void ScanCharacteristics(string deviceId, string serviceId);

        [DllImport("BleWinrtDll.dll", EntryPoint = "PollCharacteristic")]
        public static extern ScanStatus PollCharacteristic(out Characteristic characteristic, bool block);

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
        }

        [DllImport("BleWinrtDll.dll", EntryPoint = "PollData")]
        public static extern bool PollData(out BLEData data, bool block);

        [DllImport("BleWinrtDll.dll", EntryPoint = "SendData")]
        public static extern bool SendData(in BLEData data, bool block);

        [DllImport("BleWinrtDll.dll", EntryPoint = "Quit")]
        public static extern void Quit();

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        public struct ErrorMessage
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
            public string msg;
        }

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
        internal volatile bool cancelled = false;

        public void Cancel()
        {
            cancelled = true;
            try
            {
                Impl.StopDeviceScan();
            }
            catch { }
        }
    }

    public static BLEScan ScanDevices()
    {
        if (scanThread != null && scanThread.IsAlive)
            throw new InvalidOperationException("Previous scan is still running.");

        currentScan.cancelled = false;
        currentScan.Found = null;
        currentScan.Finished = null;

        scanThread = new Thread(() =>
        {
            try
            {
                Impl.StartDeviceScan();
                Impl.DeviceUpdate res = new Impl.DeviceUpdate();
                HashSet<string> deviceIds = new HashSet<string>();
                Dictionary<string, string> deviceNames = new Dictionary<string, string>();

                while (Impl.PollDevice(ref res, true) != Impl.ScanStatus.FINISHED)
                {
                    if (currentScan.cancelled) break;

                    if (res.nameUpdated && !string.IsNullOrEmpty(res.name))
                    {
                        deviceIds.Add(res.id);
                        deviceNames[res.id] = res.name;
                    }

                    if (res.isConnectable && deviceIds.Contains(res.id))
                    {
                        currentScan.Found?.Invoke(res.id, deviceNames[res.id]);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[BLE] Scan worker exception: " + ex.Message);
            }
            finally
            {
                currentScan.Finished?.Invoke();
                scanThread = null;
            }
        })
        {
            IsBackground = true
        };

        scanThread.Start();
        return currentScan;
    }

    public static List<string> ScanAndGetServices(string deviceId)
    {
        List<string> found = new List<string>();
        Impl.ScanServices(deviceId);
        Impl.Service service = new Impl.Service();
        while (Impl.PollService(out service, true) != Impl.ScanStatus.FINISHED)
        {
            if (!string.IsNullOrEmpty(service.uuid))
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
            if (!string.IsNullOrEmpty(c.uuid))
            {
                string name = (!string.IsNullOrEmpty(c.userDescription) && c.userDescription != "no description available")
                    ? c.userDescription
                    : c.uuid;
                found[name] = c.uuid;
            }
        }
        return found;
    }

    public static bool Subscribe(string deviceId, string serviceUuid, string[] characteristicUuids)
    {
        foreach (string characteristicUuid in characteristicUuids)
        {
            if (string.IsNullOrEmpty(characteristicUuid)) continue;
            bool res = Impl.SubscribeCharacteristic(deviceId, serviceUuid, characteristicUuid, true);
            if (!res) return false;
        }
        return true;
    }

    public bool Connect(string deviceId, string serviceUuid, string[] characteristicUuids)
    {
        if (isConnected) return false;

        bool result = Subscribe(deviceId, serviceUuid, characteristicUuids);
        if (!result)
        {
            Debug.LogError("[BLE] Subscribe failed: " + GetError());
            return false;
        }

        isConnected = true;
        return true;
    }

    public static bool WritePackage(string deviceId, string serviceUuid, string characteristicUuid, byte[] data, bool block = true)
    {
        if (data == null || data.Length > 512)
            throw new ArgumentOutOfRangeException("Data must be <= 512 bytes.");

        Impl.BLEData packageSend;
        packageSend.buf = new byte[512];
        packageSend.size = (short)data.Length;
        packageSend.deviceId = deviceId;
        packageSend.serviceUuid = serviceUuid;
        packageSend.characteristicUuid = characteristicUuid;

        Buffer.BlockCopy(data, 0, packageSend.buf, 0, data.Length);

        return Impl.SendData(in packageSend, block);
    }

    public void Close()
    {
        isConnected = false;
        try
        {
            Impl.Quit();
        }
        catch { }
    }

    public static string GetError()
    {
        try
        {
            Impl.ErrorMessage buf;
            Impl.GetError(out buf);
            return string.IsNullOrEmpty(buf.msg) ? "Ok" : buf.msg;
        }
        catch
        {
            return "Unable to retrieve error";
        }
    }
}