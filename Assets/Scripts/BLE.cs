using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;

// Thin managed wrapper around BleWinrtDll.dll.
//
// Facts about the specific DLL build this was checked against (from disassembly):
//   * SendData writes with GattWriteOption.WriteWithoutResponse. A "successful" blocking
//     write means Windows accepted/queued it, NOT that the ESP32 received it.
//   * SendData IGNORES its `block` argument and always waits for the async write to finish.
//   * SendData waits on a condition variable with no predicate, and the async side signals
//     without taking the lock. A wake-up can therefore be lost and SendData can hang forever
//     (rare). Callers must tolerate a write that never returns.
//   * When a write fails, GetError() reports "Error subscribing to characteristic with uuid ..."
//     (copy-paste message in the DLL). It is a WRITE failure.
//   * Quit() sets an internal quit flag that only StartDeviceScan() clears. Inside the Unity
//     Editor the DLL stays loaded between Play sessions, so after one Quit() every later
//     Subscribe fails unless a scan runs first. ResetNativeState() handles that.
//   * Subscribed notifications are queued inside the DLL until PollData() pops them.
//     If nobody polls, that queue grows forever. DrainNotifications() handles that.
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
        public static extern ScanStatus PollDevice(ref DeviceUpdate device, [MarshalAs(UnmanagedType.I1)] bool block);

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
        public static extern ScanStatus PollService(out Service service, [MarshalAs(UnmanagedType.I1)] bool block);

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
        public static extern ScanStatus PollCharacteristic(out Characteristic characteristic, [MarshalAs(UnmanagedType.I1)] bool block);

        // Native signature returns a 1-byte C++ bool. This DLL build happens to zero-extend it,
        // but declaring I1 makes the marshalling correct regardless of how the DLL is compiled.
        [DllImport("BleWinrtDll.dll", EntryPoint = "SubscribeCharacteristic", CharSet = CharSet.Unicode)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool SubscribeCharacteristic(string deviceId, string serviceId, string characteristicId, [MarshalAs(UnmanagedType.I1)] bool block);

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
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool PollData(out BLEData data, [MarshalAs(UnmanagedType.I1)] bool block);

        // NOTE: this DLL build ignores `block` and always waits for the write to complete.
        [DllImport("BleWinrtDll.dll", EntryPoint = "SendData")]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool SendData(in BLEData data, [MarshalAs(UnmanagedType.I1)] bool block);

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

    public const int MaxPayload = 512;

    private static volatile Thread scanThread;

    /// <summary>The most recently started scan (kept for compatibility).</summary>
    public static BLEScan currentScan { get; private set; }

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
            if (cancelled) return;
            cancelled = true;
            try
            {
                Impl.StopDeviceScan();
            }
            catch { }
        }
    }

    /// <summary>
    /// Starts a background scan. Callbacks are attached BEFORE the scan thread starts, so
    /// no early discovery event can be lost. Callbacks run on the scan thread.
    /// </summary>
    public static BLEScan ScanDevices(BLEScan.FoundDel onFound, BLEScan.FinishedDel onFinished = null)
    {
        Thread existing = scanThread;
        if (existing != null && existing.IsAlive)
            throw new InvalidOperationException("Previous scan is still running.");

        // A fresh object per scan: a late callback from an old scan can never fire into a new one.
        BLEScan scan = new BLEScan { Found = onFound, Finished = onFinished };
        currentScan = scan;

        Thread t = new Thread(() =>
        {
            try
            {
                Impl.StartDeviceScan();
                Impl.DeviceUpdate res = new Impl.DeviceUpdate();
                HashSet<string> deviceIds = new HashSet<string>();
                Dictionary<string, string> deviceNames = new Dictionary<string, string>();

                while (!scan.cancelled && Impl.PollDevice(ref res, true) != Impl.ScanStatus.FINISHED)
                {
                    if (scan.cancelled) break;
                    if (string.IsNullOrEmpty(res.id)) continue;

                    if (res.nameUpdated && !string.IsNullOrEmpty(res.name))
                    {
                        deviceIds.Add(res.id);
                        deviceNames[res.id] = res.name;
                    }

                    if (res.isConnectable && deviceIds.Contains(res.id))
                    {
                        scan.Found?.Invoke(res.id, deviceNames[res.id]);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[BLE] Scan worker exception: " + ex.Message);
            }
            finally
            {
                try { scan.Finished?.Invoke(); } catch (Exception ex) { Debug.LogWarning("[BLE] Scan Finished callback threw: " + ex.Message); }
            }
        })
        {
            IsBackground = true,
            Name = "BLE Scan"
        };

        scanThread = t;
        t.Start();
        return scan;
    }

    /// <summary>Compatibility overload. Prefer passing callbacks so none can be missed.</summary>
    public static BLEScan ScanDevices()
    {
        return ScanDevices(null, null);
    }

    /// <summary>Waits for the scan worker thread to exit. Returns true if it is gone.</summary>
    public static bool WaitForScanToFinish(int timeoutMs)
    {
        Thread t = scanThread;
        if (t == null || !t.IsAlive) return true;
        return t.Join(timeoutMs);
    }

    /// <summary>
    /// Clears the DLL's internal quit flag, which only StartDeviceScan() resets. Needed when
    /// connecting to a hard-coded device ID (no scan) after Quit() ran in an earlier Editor
    /// Play session; otherwise SubscribeCharacteristic silently fails.
    /// </summary>
    public static void ResetNativeState()
    {
        Thread t = scanThread;
        if (t != null && t.IsAlive) return; // a running scan already did this
        Impl.StartDeviceScan();
        Thread.Sleep(50);
        Impl.StopDeviceScan();
    }

    public static List<string> ScanAndGetServices(string deviceId)
    {
        List<string> found = new List<string>();
        Impl.ScanServices(deviceId);
        Impl.Service service;
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
        Impl.Characteristic c;
        while (Impl.PollCharacteristic(out c, true) != Impl.ScanStatus.FINISHED)
        {
            if (!string.IsNullOrEmpty(c.uuid))
            {
                string name = (!string.IsNullOrEmpty(c.userDescription) && c.userDescription != "no description available")
                    ? c.userDescription
                    : c.uuid;
                // Two characteristics with the same user description must not overwrite each other.
                if (found.ContainsKey(name)) name = name + " (" + c.uuid + ")";
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

    /// <summary>
    /// Allocation-free writer for one characteristic. Build once, call Write() repeatedly.
    /// Not thread-safe: use one instance per thread.
    /// </summary>
    public sealed class PacketWriter
    {
        private Impl.BLEData pkg;

        public PacketWriter(string deviceId, string serviceUuid, string characteristicUuid)
        {
            pkg.buf = new byte[MaxPayload];
            pkg.size = 0;
            pkg.deviceId = deviceId;
            pkg.serviceUuid = serviceUuid;
            pkg.characteristicUuid = characteristicUuid;
        }

        /// <summary>
        /// Blocking write. Returns true once Windows has accepted the write-without-response.
        /// Can (rarely) never return; see the notes at the top of this file.
        /// </summary>
        public bool Write(byte[] data, int length)
        {
            if (data == null || length < 0 || length > MaxPayload || length > data.Length)
                throw new ArgumentOutOfRangeException(nameof(length), "Data must be <= " + MaxPayload + " bytes.");
            Buffer.BlockCopy(data, 0, pkg.buf, 0, length);
            pkg.size = (short)length;
            return Impl.SendData(in pkg, true);
        }
    }

    /// <summary>Convenience one-off write. Allocates; prefer PacketWriter for streaming.</summary>
    public static bool WritePackage(string deviceId, string serviceUuid, string characteristicUuid, byte[] data, bool block = true)
    {
        if (data == null || data.Length > MaxPayload)
            throw new ArgumentOutOfRangeException(nameof(data), "Data must be <= " + MaxPayload + " bytes.");
        return new PacketWriter(deviceId, serviceUuid, characteristicUuid).Write(data, data.Length);
    }

    /// <summary>
    /// Pops up to maxCount queued notifications without blocking, so the DLL's internal queue
    /// cannot grow without bound. Returns how many were removed. Optional callback receives
    /// (payload bytes, length).
    /// </summary>
    public static int DrainNotifications(int maxCount, Action<byte[], int> onData = null)
    {
        int n = 0;
        while (n < maxCount)
        {
            Impl.BLEData d;
            if (!Impl.PollData(out d, false)) break;
            n++;
            if (onData != null && d.buf != null)
            {
                int len = Mathf.Clamp(d.size, 0, d.buf.Length);
                onData(d.buf, len);
            }
        }
        return n;
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