using System;
using System.Threading;
using Microsoft.SPOT;

namespace NetduinoApplicationPololu
{
    public class Accelaccelerometer : IDisposable
    {
             private static Accelaccelerometer _instance;
        private static readonly object LockObject = new object();

        #region variables
        private Timer continuousTimer;
        #endregion

#region static values

        public static byte ACC_ADDRESS_SA0_A_LOW = (0x30 >> 1);
        public static byte ACC_ADDRESS_SA0_A_HIGH = (0x32 >> 1);

        static byte _device = LSM303_DEVICE_AUTO; // chip type (DLH, DLM, or DLHC)
        static byte acc_address = (0x30 >> 1);

        // device types

        public const byte LSM303DLH_DEVICE = 0;
        public const byte LSM303DLM_DEVICE = 1;
        public const byte LSM303DLHC_DEVICE = 2;
        public const byte LSM303_DEVICE_AUTO = 3;

        // SA0_A states

        public static byte LSM303_SA0_A_LOW = 0;
        public static byte LSM303_SA0_A_HIGH = 1;
        public static byte LSM303_SA0_A_AUTO = 2;

        // register addresses

        public static byte LSM303_CTRL_REG1_A = 0x20;
        public static byte LSM303_CTRL_REG2_A = 0x21;
        public static byte LSM303_CTRL_REG3_A = 0x22;
        public static byte LSM303_CTRL_REG4_A = 0x23;
        public static byte LSM303_CTRL_REG5_A = 0x24;
        public static byte LSM303_CTRL_REG6_A = 0x25; // DLHC only
        static int LSM303_HP_FILTER_RESET_A = 0x25;// DLH, DLM only
        static int LSM303_REFERENCE_A = 0x26;
        static int LSM303_STATUS_REG_A = 0x27;

        static int LSM303_OUT_X_L_A = 0x28;
        static int LSM303_OUT_X_H_A = 0x29;
        static int LSM303_OUT_Y_L_A = 0x2A;
        static int LSM303_OUT_Y_H_A = 0x2B;
        static int LSM303_OUT_Z_L_A = 0x2C;
        static int LSM303_OUT_Z_H_A = 0x2D;

        static int LSM303_FIFO_CTRL_REG_A = 0x2E; // DLHC only
        static int LSM303_FIFO_SRC_REG_A = 0x2F;// DLHC only

        static int LSM303_INT1_CFG_A = 0x30;
        static int LSM303_INT1_SRC_A = 0x31;
        static int LSM303_INT1_THS_A = 0x32;
        static int LSM303_INT1_DURATION_A = 0x33;
        static int LSM303_INT2_CFG_A = 0x34;
        static int LSM303_INT2_SRC_A = 0x35;
        static int LSM303_INT2_THS_A = 0x36;
        static int LSM303_INT2_DURATION_A = 0x37;

        static int LSM303_CLICK_CFG_A = 0x38;// DLHC only
        static int LSM303_CLICK_SRC_A = 0x39;// DLHC only
        static int LSM303_CLICK_THS_A = 0x3A;// DLHC only
        static int LSM303_TIME_LIMIT_A = 0x3B; // DLHC only
        static int LSM303_TIME_LATENCY_A = 0x3C; // DLHC only
        static int LSM303_TIME_WINDOW_A = 0x3D; // DLHC only
        private const int ClockRate = 100;
        private const int Timeout = 100;

        #endregion

        private Accelaccelerometer()
        {
        }

        public void Dispose()
        {
            throw new NotImplementedException();
        }
        public static Accelaccelerometer GetInstance()
        {
            lock (LockObject)
            {
                if (_instance == null)
                {
                    _instance = new Accelaccelerometer();
                    _instance.Init();
                }
                return _instance;
            }
        }
        #region Measurement Events
        public delegate void MeasurementCompleteEventHandler(Accelaccelerometer sender, SensorData sensorData);

        public event MeasurementCompleteEventHandler MeasurementComplete;

        private MeasurementCompleteEventHandler _OnMeasurementComplete;

        //    public static EventHandler<SensorData> MeasurementComplete;

        /// <summary>
        /// Raises the <see cref="MeasurementComplete"/> event.
        /// </summary>
        /// <param name="sender">The object that raised the event.</param>
        /// <param name="sensorData">The <see cref="SensorData"/> object that contains the results of the measurement.</param>
        public void OnMeasurementCompleteEvent(Accelaccelerometer sender, SensorData sensorData)
        {
            if (_OnMeasurementComplete == null)
                _OnMeasurementComplete = OnMeasurementCompleteEvent;
        }
        #endregion





        public void StartContinuousMeasurements()
        {
            _instance.continuousTimer = new Timer(_instance.MyTimerCallback, null, 0, 200);
        }

        public void StopContinuousMeasurements()
        {
            _instance.continuousTimer = null;
        }
        struct vector
        {
            public Int16 x, y, z;
        };
        private void MyTimerCallback(object state)
        {
            var values = new byte[6];

            I2CBus.GetInstance().ReadRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(acc_address, ClockRate), (byte)LSM303_OUT_X_L_A, values, Timeout);

            byte xla = values[0];
            byte xha = values[1];
            byte yla = values[2];
            byte yha = values[3];
            byte zla = values[4];
            byte zha = values[5];
            vector _rawData;


            _rawData.x = (Int16)(xha << 8 | xla );
            _rawData.y = (Int16)(yha << 8 | yla);
            _rawData.z = (Int16)(zha << 8 | zla);
     
            if (_instance.MeasurementComplete != null)
                _instance.MeasurementComplete(_instance,
                                              new SensorData((int)_rawData.x, (int)_rawData.y,
                                                             (int)_rawData.z));
        }

        private void Init()
        {
            // Enable Accelerometer
            // 0x27 = 0b00100111
            // Normal power mode, all axes enabled
            Debug.Print("LSM303_CTRL_REG1_A");
            writeAccReg(LSM303_CTRL_REG1_A, 0x27);
        }
        // Writes an accelerometer register
        public static void writeAccReg(byte reg, byte value)
        {
            byte[] values = new byte[6];

            I2CBus.GetInstance().WriteRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(acc_address, ClockRate), reg, value, Timeout);

        }

        // Reads an accelerometer register
        public static byte readAccReg(byte reg)
        {
            var values = new byte[6];

            I2CBus.GetInstance().ReadRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(acc_address, ClockRate), reg, values, Timeout);

            return values[0];
        }
        public class SensorData : EventArgs
        {
            /// <summary>
            /// Raw X-axis sensor data.
            /// </summary>
            public double X
            {
                get;
                private set;
            }

            /// <summary>
            /// Raw Y-axis sensor data.
            /// </summary>
            public double Y
            {
                get;
                private set;
            }

            /// <summary>
            /// Raw Z-axis sensor data.
            /// </summary>
            public double Z
            {
                get;
                private set;
            }

            /// <summary>
            /// Angle of heading in the XY plane, in radians.
            /// </summary>


            /// <summary>
            /// A set of sensor measurements.
            /// </summary>
            /// <param name="angle">Angle of heading in the XY plane, in radians.</param>
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
            /// Provides a string representation of the <see cref="Compass.SensorData"/> instance.
            /// </summary>
            /// <returns>A string describing the values contained in the object.</returns>
            public override string ToString()
            {
                return " X: " + X.ToString("f2") + " Y: " + Y.ToString("f2") + " Z: " +
                       Z.ToString("f2");
            }
        }
    }
}
