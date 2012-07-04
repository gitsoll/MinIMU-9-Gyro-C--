using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.Netduino;
using Toolbox.NETMF.Hardware;

namespace NetduinoApplicationPololu
{
    public class Program
    {
        public static void Main()
        {
            // write your code here
            Debug.Print("Start app");
            // write your code here

            MinIMU9AHRS.setup();
            while (true)
            {

                MinIMU9AHRS.loop();
                MinIMU9AHRS.printdata();
                Thread.Sleep(500);
            }

        }

    }
}
