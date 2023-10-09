using System;
using System.Threading.Tasks;



namespace CSReaderConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            HCBDevice.Init(false);
            HCBDevice.Start();
            HCBDevice[] devices = HCBDevice.GetDevices();
            if (devices.Length == 0)
                return;

            devices[0].HCBADCDataReceived += HCBADCDataReceived;
            devices[0].HCBFTDataReceived += HCBFTDataReceived;
            while (true)
            { 
                Task.Delay(1000).Wait(); 
            }
        }

        private static void HCBADCDataReceived(object source, HCBADCData data)
        {
            Console.WriteLine($"ADC Data: {data}");
        }

        private static void HCBFTDataReceived(object source, HCBFTData data)
        {
            Console.WriteLine($"FT Data: {data}");
        }
    }
}
