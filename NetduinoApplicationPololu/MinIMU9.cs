using System;
using NetduinoApplicationPololu;

namespace Toolbox.NETMF.Hardware
{
    public  class MinIMU9:IDisposable
    {
        private MinIMU9()
        {           
        }

        private Magnetometer _magnetometer;
        public Magnetometer Magnetometer
        {
            get { return _magnetometer; }
            set { _magnetometer = value; }
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
                }
                return _instance;
            }
        }


        public void Dispose()
        {
            throw new NotImplementedException();
        }
    }
}