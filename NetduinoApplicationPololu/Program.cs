using System.Threading;
using Microsoft.SPOT;
using Toolbox.NETMF.Hardware;

namespace NetduinoApplicationPololu
{
    public class Program
    {
        public static void Main()
        {
            Debug.Print("Start app test");
            int z = 0;

            var a =  MinIMU9.GetInstance();
            //a.Magnetometer.MeasurementComplete += Magnetometer_MeasurementComplete;
            //a.Magnetometer.StartContinuousMeasurements();
            a.Gyro.MeasurementComplete += new Gyro.MeasurementCompleteEventHandler(Gyro_MeasurementComplete);
            a.Gyro.StartContinuousMeasurements();
            //a.Accelaccelerometer.MeasurementComplete += new Accelaccelerometer.MeasurementCompleteEventHandler(Accelaccelerometer_MeasurementComplete);
            //a.Accelaccelerometer.StartContinuousMeasurements();
            while (z < 4000)
            {
                z++;            
                Thread.Sleep(200);
            }
        
        }

        static void Accelaccelerometer_MeasurementComplete(Accelaccelerometer sender, Accelaccelerometer.SensorData sensorData)
        {
            Debug.Print(sensorData.ToString());
        }

        static void Gyro_MeasurementComplete(Gyro sender, Gyro.SensorData sensorData)
        {
            Debug.Print(sensorData.ToString());
        }

        static void Magnetometer_MeasurementComplete(Magnetometer sender, Magnetometer.SensorData sensorData)
        {
            Debug.Print(sensorData.ToString());
        }
    }
}
          
