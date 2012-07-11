using System;
using NetduinoApplicationPololu;

namespace Toolbox.NETMF.Hardware
{
    public  class MinIMU9:IDisposable
    {
        private MinIMU9()
        {           
        }

        private Accelaccelerometer _accelaccelerometer;
        public Accelaccelerometer Accelaccelerometer
        {
            get
            {
                return _instance._accelaccelerometer;
            }

        }

        private Magnetometer _magnetometer;
        public  Magnetometer Magnetometer
        {
            get
            {
                return _instance._magnetometer;
            }
           
        }


        private Gyro _gyro;
        public Gyro Gyro
        {
            get
            {
                return _instance._gyro;
            }

        }

        private static MinIMU9 _instance = null;
        private static readonly object LockObject = new object();

        public static MinIMU9 GetInstance()
        {
            lock (LockObject)
            {
                if (_instance == null)
                {
                    _instance = new MinIMU9();
                    _instance._magnetometer =  Magnetometer.GetInstance();
                    _instance._gyro = Gyro.GetInstance();
                    _instance._accelaccelerometer = Accelaccelerometer.GetInstance();
                }
                return _instance;
            }
        }


        public void Dispose()
        {
            _instance._magnetometer.Dispose();
            _instance._gyro.Dispose();
            _instance._accelaccelerometer.Dispose();
        }
    }
}