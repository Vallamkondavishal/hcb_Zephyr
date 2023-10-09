using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Timers;
using LibUsbDotNet;
using LibUsbDotNet.Descriptors;
using LibUsbDotNet.Info;
using LibUsbDotNet.Main;

namespace CSReaderConsole
{
    public enum HCBDataType
    {
        HCB_ID_COMMAND = 0x10,
        HCB_ID_HELLO_REPLY,
        HCB_ID_ADC,
        HCB_ID_UNSOLICITED_TIME,
        HCB_ID_SET_POINT,
        HCB_ID_UNSOLICITED_TIME_2,
        HCB_ID_FT,
        HCB_ID_MAX,
    }
    public class HCBADCData
    {
        public UInt32 timestamp;
        public UInt16[] adc;

        public HCBADCData()
        {
            adc = new UInt16[24];
        }
        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < 24; i++)
            {
                if (i != 0)
                    sb.Append(",");
                sb.Append(adc[i]);
            }
            return $"{timestamp}: {sb}";
        }
    }

    public class HCBFTData
    {
        public UInt32 timestamp;
        public Int32[] ft;

        public HCBFTData()
        {
            ft = new Int32[6];
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < 6; i++)
            {
                if (i != 0)
                    sb.Append(",");
                sb.Append(ft[i]);
            }
            return $"{timestamp}: {sb}";
        }
    }

    public class HCBTargetsData
    {
        public UInt32 timestamp;
        public UInt32[] setpoint;

        public HCBTargetsData()
        {
            setpoint = new UInt32[8];
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < 8; i++)
            {
                if (i != 0)
                    sb.Append(",");
                sb.Append(setpoint[i]);
            }
            return $"{timestamp}: {sb}";
        }
    }

    public class HCBDevice
    {
        private const uint VID = 0x2fe3;
        private const uint PID = 0x0100;

        private const int HCB_REPORT_ID_IN_REPLY = 1;
        private const int HCB_REPORT_ID_OUT_REQUEST = 2;
        private const int HCB_REPORT_ID_IN_UNSOLICITED = 3;

        private readonly static object _lockObj = new object();
        private readonly static List<HCBDevice> _devices = new List<HCBDevice>();

        public event HCBADCDataEventHandler HCBADCDataReceived;
        public delegate void HCBADCDataEventHandler(object source, HCBADCData data);

        public event HCBFTDataEventHandler HCBFTDataReceived;
        public delegate void HCBFTDataEventHandler(object source, HCBFTData data);

        public static bool Init(bool disableMultiple)
        {
            Stop();
            _devices.Clear();

            UsbRegDeviceList allDevices = UsbDevice.AllDevices;
            foreach (UsbRegistry rdev in allDevices)
            {
                Console.WriteLine(rdev.ToString());
                if (HCBDevice.IsHCB(rdev))
                {
                    AddDevice(new HCBDevice(rdev));
                    if (disableMultiple)
                        break;
                }
            }

            return true;
        }

        public static void Start()
        {
            foreach (var dev in _devices)
            {
                dev.StartTask();
            }
        }

        public static void Stop()
        {
            foreach (var dev in _devices)
            {
                if ((dev._startTask != null) && (!dev._startTask.IsCompleted))
                {
                    dev._haltStartTask = true;
                    dev._startTask.Wait();
                }
                else
                {
                    dev.Close();
                }
            }
        }

        public static HCBDevice[] GetDevices()
        {
            return _devices.ToArray();
        }

        private static void AddDevice(HCBDevice dev)
        {
            _devices.Add(dev);
        }

        private UsbDevice _usbDev = null;
        public UsbDevice Device { get { return _usbDev; } }

        private readonly UsbRegistry _reg = null;
        private UsbEndpointReader reader;
        private ReadEndpointID dataEndpointId;
        private WriteEndpointID targetsEndpointId;

        private uint _btnmask = 0;
        private uint _extbtnmask = 0;

        private HCBDevice(UsbRegistry reg)
        {
            _reg = reg;
        }

        private bool Open()
        {
            lock (_lockObj)
            {
                _btnmask = 0;
                _extbtnmask = 0;
                if (Device != null)
                {
                    return true;
                }
                if (!_reg.Open(out _usbDev))
                    return false;

                DecodeEndpoint();

                reader = Device.OpenEndpointReader(dataEndpointId, 1024, EndpointType.Interrupt);
                reader.DataReceived += Reader_DataReceived;
                reader.DataReceivedEnabled = true;
                reader.DataReceivedEnabledChanged += Reader_DataReceivedEnabledChanged;
                return true;
            }
        }

        private void Reader_DataReceivedEnabledChanged(object sender, DataReceivedEnabledChangedEventArgs e)
        {
            if (!e.Enabled)
            {
                StartTask();
            }
        }

        private Task _startTask = null;
        private bool _haltStartTask = false;
        private void StartTask()
        {
            lock (_lockObj)
            {
                _haltStartTask = false;
                if (_startTask != null)
                {
                    Console.WriteLine("StartTask already started");
                    return;
                }
                _startTask = Task.Run(async () =>
                {
                    try
                    {
                        Close();
                        bool connected = false;
                        do
                        {
                            await Task.Delay(500);
                            connected = Open();
                        } while (!connected && !_haltStartTask);
                        if (_haltStartTask && connected)
                            Close();
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine(ex);
                    }
                    finally
                    {
                        _startTask = null;
                    }
                });
            }
        }


        private void Close()
        {
            lock (_lockObj)
            {
                if (reader != null)
                {
                    reader.DataReceivedEnabledChanged -= Reader_DataReceivedEnabledChanged;
                    reader.Dispose();
                }
                reader = null;
                _usbDev?.Close();
                _usbDev = null;
            }
        }

        private void Reader_DataReceived(object sender, EndpointDataEventArgs e)
        {
            HandleReport(e.Buffer, e.Count);
        }

        private void HandleReport(byte[] report, int len)
        {
            if (len <= 0)
                return;

            if (report[0] != HCB_REPORT_ID_IN_UNSOLICITED)
                return;

            OnDataReport(report);
        }

        private void OnDataReport(byte[] buf)
        {
            // TODO check crc
            switch (buf[1])
            {
                case (int)HCBDataType.HCB_ID_ADC:
                    {
                        // TODO check byte count
                        HCBADCData data = new HCBADCData();
                        data.timestamp = (uint)buf[4] | (uint)buf[5] << 8 |
                            (uint)buf[6] << 16 | (uint)buf[7] << 24;
                        for (int i = 0; i < 24; i++)
                        {
                            data.adc[i] = (ushort)(buf[8 + i * 2] | buf[8 + i * 2 + 1] << 8);
                        }
                        HCBADCDataReceived?.Invoke(this, data);
                    }
                    break;

                case (int)HCBDataType.HCB_ID_FT:
                    {
                        // TODO check byte count
                        HCBFTData data = new HCBFTData();
                        data.timestamp = (uint)buf[4] | (uint)buf[5] << 8 |
                            (uint)buf[6] << 16 | (uint)buf[7] << 24;
                        for (int i = 0; i < 6; i++)
                        {
                            data.ft[i] = buf[8 + i * 4] | buf[8 + i * 4 + 1] << 8 |
                                buf[8 + i * 4 + 2] << 16 | buf[8 + i * 4 + 3] << 24;
                        }
                        HCBFTDataReceived?.Invoke(this, data);
                    }
                    break;
            }            
        }

        private void DecodeEndpoint()
        {
            Console.WriteLine(Device.Info.ToString());
            for (int iConfig = 0; iConfig < Device.Configs.Count; iConfig++)
            {
                UsbConfigInfo configInfo = Device.Configs[iConfig];
                Console.WriteLine(configInfo.ToString());

                ReadOnlyCollection<UsbInterfaceInfo> interfaceList = configInfo.InterfaceInfoList;
                foreach (UsbInterfaceInfo interfaceInfo in interfaceList)
                {
                    Console.WriteLine(interfaceInfo.ToString());
                    ReadOnlyCollection<UsbEndpointInfo> endpointList = interfaceInfo.EndpointInfoList;
                    foreach (var ep in endpointList)
                    {
                        Console.WriteLine(ep.ToString());
                        if ((ep.Descriptor.EndpointID & 0x80) == 0x80)
                        {
                            dataEndpointId = (ReadEndpointID)ep.Descriptor.EndpointID;
                        } else
                        {
                            targetsEndpointId = (WriteEndpointID)ep.Descriptor.EndpointID;
                        }
                    }
                }
            }
        }

        public void SendTargets(HCBTargetsData targets)
        {
            byte[] report = new byte[14];
            report[0] = HCB_REPORT_ID_OUT_REQUEST;
            report[1] = (byte)HCBDataType.HCB_ID_SET_POINT;
            report[2] = 14;
            report[3] = 0;
            for (int i=0; i<8; i++) 
            {
                report[i * 4] = (byte)(targets.setpoint[i] & 0xff);
                report[i * 4 + 1] = (byte)((targets.setpoint[i] >> 8) & 0xff);
                report[i * 4 + 2] = (byte)((targets.setpoint[i] >> 16) & 0xff);
                report[i * 4 + 3] = (byte)((targets.setpoint[i] >> 24) & 0xff);
            }
            UsbEndpointWriter writer = Device.OpenEndpointWriter(targetsEndpointId);
            int len;
            writer.Write(report, 1000, out len);
        }

        public byte[] ReadDescriptor(byte type, byte idx, int maxSize = 1024, short langId = 0x0409)
        {
            IntPtr buf = Marshal.AllocHGlobal(maxSize);
            if (!Device.GetDescriptor(type, idx, langId, buf, maxSize, out int len))
            {
                Marshal.FreeHGlobal(buf);
                Console.WriteLine($"{Device.DevicePath} unable to read idx {idx:X}");
                return new byte[0];
            }
            byte[] data = new byte[len];
            Marshal.Copy(buf, data, 0, len);
            Marshal.FreeHGlobal(buf);
            return data;
        }

        public string ByteArrayToString(byte[] data)
        {
            StringBuilder sb = new StringBuilder();
            foreach (byte v in data)
                sb.Append($"{v,2:X} ");
            return sb.ToString();
        }

        public static bool IsHCB(UsbRegistry usbReg)
        {
            return ((uint)usbReg.Vid == VID) && ((uint)usbReg.Pid == PID);
        }

    }

}