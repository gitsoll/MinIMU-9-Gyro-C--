using Microsoft.SPOT;
using Toolbox.NETMF.Hardware;

namespace NetduinoApplicationPololu
{
    public class Program
    {
        public static void Main()
        {
            Debug.Print("Start app Magnometer test");
            int z = 0;
            double xmin = 99999, ymin = 99999, zmin = 99999;
            double xmax = -99999, ymax = -99999, zmax = -99999;
            var a =  MinIMU9.GetInstance();
            while (z < 2000)
            {
                z++;
                a.Magnetometer.Read();
                xmin = System.Math.Min(xmin, a.Magnetometer._rawData.x);
                ymin = System.Math.Min(ymin, a.Magnetometer._rawData.y);
                zmin = System.Math.Min(zmin, a.Magnetometer._rawData.z);

                xmax = System.Math.Max(xmax, a.Magnetometer._rawData.x);
                ymax = System.Math.Max(ymax, a.Magnetometer._rawData.y);
                zmax = System.Math.Max(zmax, a.Magnetometer._rawData.z);
                Debug.Print("Direction =" + a.Magnetometer.Direction.ToString());
                //  Thread.Sleep(250);
            }
            Debug.Print("xmin:" + xmin + " ymin:" + ymin + " zmin:" + zmin);
            Debug.Print("xmax:" + xmax + " ymax:" + ymax + " zmax:" + zmax);

        }
    }
}
          
