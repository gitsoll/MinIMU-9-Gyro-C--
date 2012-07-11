using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace NetduinoApplicationPololu
{
    public class Gyro : IDisposable
    {
        private static Gyro _instance;
        private static readonly object LockObject = new object();

        #region variables

        private Timer continuousTimer;

        #endregion

        #region static values

        private int ClockRate = 50;
        private byte GYR_ADDRESS = 0xD2 >> 1;
        public byte L3G4200D_CTRL_REG1 = 0x20;
        public byte L3G4200D_CTRL_REG2 = 0x21;
        public byte L3G4200D_CTRL_REG3 = 0x22;
        public byte L3G4200D_CTRL_REG4 = 0x23;
        private byte L3G4200D_CTRL_REG5 = 0x24;
        private byte L3G4200D_FIFO_CTRL_REG = 0x2E;
        private byte L3G4200D_FIFO_SRC_REG = 0x2F;
        private byte L3G4200D_INT1_CFG = 0x30;
        private byte L3G4200D_INT1_DURATION = 0x38;
        private byte L3G4200D_INT1_SRC = 0x31;
        private byte L3G4200D_INT1_THS_XH = 0x32;
        private byte L3G4200D_INT1_THS_XL = 0x33;
        private byte L3G4200D_INT1_THS_YH = 0x34;
        private byte L3G4200D_INT1_THS_YL = 0x35;
        private byte L3G4200D_INT1_THS_ZH = 0x36;
        private byte L3G4200D_INT1_THS_ZL = 0x37;
        private byte L3G4200D_OUT_TEMP = 0x26;
        private byte L3G4200D_OUT_X_H = 0x29;
        private byte L3G4200D_OUT_X_L = 0x28;
        private byte L3G4200D_OUT_Y_H = 0x2B;
        private byte L3G4200D_OUT_Y_L = 0x2A;
        private byte L3G4200D_OUT_Z_H = 0x2D;
        private byte L3G4200D_OUT_Z_L = 0x2C;
        private byte L3G4200D_REFERENCE = 0x25;
        private byte L3G4200D_STATUS_REG = 0x27;
        public byte L3G4200D_WHO_AM_I = 0x0F;
        private int Timeout = 100;

        #endregion

        private Gyro()
        {
        }

        #region IDisposable Members

        public void Dispose()
        {
            throw new NotImplementedException();
        }

        #endregion

        public static Gyro GetInstance()
        {
            lock (LockObject)
            {
                if (_instance == null)
                {
                    _instance = new Gyro();
                    _instance.Init();
                }
                return _instance;
            }
        }

        public void StartContinuousMeasurements()
        {
            _instance.continuousTimer = new Timer(_instance.MyTimerCallback, null, 0, 200);
        }

        public void StopContinuousMeasurements()
        {
            _instance.continuousTimer = null;
        }

        private void MyTimerCallback(object state)
        {
            var ret = new byte[6];

            I2CBus.GetInstance().ReadRegister(new I2CDevice.Configuration(GYR_ADDRESS, ClockRate),
                                              (byte) (L3G4200D_OUT_X_L | (1 << 7)), ret, Timeout);


            int xla = ret[0];
            int xha = ret[1];
            int yla = ret[2];
            int yha = ret[3];
            int zla = ret[4];
            int zha = ret[5];

            vector _rawData;
            _rawData.x = (Int16) (xha << 8 | xla);
            _rawData.y = (Int16) (yha << 8 | yla);
            _rawData.z = (Int16) (zha << 8 | zla);


            if (_instance.MeasurementComplete != null)
                _instance.MeasurementComplete(_instance,
                                              new SensorData(_rawData.x, _rawData.y,
                                                             _rawData.z));
        }

        private void Init()
        {
            var ret = new byte[6];
            // 0x0F = 0b00001111	
            // Normal power mode, all axes enabled	
            writeReg(L3G4200D_CTRL_REG1, 0x0F);
        }


        // Writes a gyro register
        private void writeReg(byte reg, byte value)
        {
            I2CBus.GetInstance().WriteRegister(new I2CDevice.Configuration(GYR_ADDRESS, ClockRate), reg, value, Timeout);
        }



        #region Measurement Events

        #region Delegates

        public delegate void MeasurementCompleteEventHandler(Gyro sender, SensorData sensorData);

        #endregion

        private MeasurementCompleteEventHandler _OnMeasurementComplete;
        public event MeasurementCompleteEventHandler MeasurementComplete;

        //    public static EventHandler<SensorData> MeasurementComplete;

        /// <summary>
        /// Raises the <see cref="MeasurementComplete"/> event.
        /// </summary>
        /// <param name="sender">The object that raised the event.</param>
        /// <param name="sensorData">The <see cref="SensorData"/> object that contains the results of the measurement.</param>
        public void OnMeasurementCompleteEvent(Gyro sender, SensorData sensorData)
        {
            if (_OnMeasurementComplete == null)
                _OnMeasurementComplete = OnMeasurementCompleteEvent;
        }

        #endregion

        #region Nested type: SensorData

        public class SensorData : EventArgs
        {
            /// <summary>
            /// </summary>
            /// <summary>
            /// </summary>
            /// <param name="x">Raw X-axis sensor data.</param>
            /// <param name="y">Raw Y-axis sensor data.</param>
            /// <param name="z">Raw Z-axis sensor data.</param>            
            public SensorData(double x, double y, double z)
            {
                X = x;
                Y = y;
                Z = z;
            }

            /// <summary>
            /// Raw X-axis sensor data.
            /// </summary>
            public double X { get; private set; }

            /// <summary>
            /// Raw Y-axis sensor data.
            /// </summary>
            public double Y { get; private set; }

            /// <summary>
            /// Raw Z-axis sensor data.
            /// </summary>
            public double Z { get; private set; }

            /// <summary>
            /// Provides a string representation of the <see cref="Compass.SensorData"/> instance.
            /// </summary>
            /// <returns>A string describing the values contained in the object.</returns>
            public override string ToString()
            {
                return " X: " + X.ToString("f2") + " Y: " + Y.ToString("f2") + " Z: " +
                       Z.ToString("f2");
            }
        }

        #endregion

        #region Nested type: vector

        private struct vector
        {
            public Int16 x, y, z;
        };

        #endregion
    }
}