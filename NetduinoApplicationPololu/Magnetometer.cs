using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using Math = System.Math;

namespace NetduinoApplicationPololu
{
    public sealed class Magnetometer : IDisposable
    {
        private static Magnetometer _instance;
        private static readonly object LockObject = new object();

        private Magnetometer()
        {
        }

        public static Magnetometer GetInstance()
        {
            lock (LockObject)
            {
                if (_instance == null)
                {
                    _instance = new Magnetometer();
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
            var values = new byte[6];


            I2CBus.GetInstance().ReadRegister(new I2CDevice.Configuration(ADDRESS, ClockRate), (byte) LSM303_OUT_X_H_M,
                                              values, Timeout);

            byte xhm = values[0];
            byte xlm = values[1];


            byte yhm, ylm, zhm, zlm;

            vector _rawData;

            if (_device == LSM303DLH_DEVICE)
            {
                //  DLH: register address for Y comes before Z
                yhm = values[2];
                ylm = values[3];

                zhm = values[4];
                zlm = values[5];
            }
            else
            {
                // DLM, DLHC: register address for Z comes before Y
                zhm = values[2];
                zlm = values[3];
                yhm = values[4];
                ylm = values[5];
            }
           
            // Transform 2�s complement to doubles
            _rawData.x = (Int16) (xhm << 8 | xlm);
            _rawData.y = (Int16) (yhm << 8 | ylm);
            _rawData.z = (Int16) (zhm << 8 | zlm);

            m_min.x = Math.Min(m_min.x, (Int16) (xhm << 8 | xlm));
            m_min.y = Math.Min(m_min.y, (Int16) (yhm << 8 | ylm));
            m_min.z = Math.Min(m_min.z, (Int16) (zhm << 8 | zlm));

            m_max.x = Math.Max(m_max.x, (Int16) (xhm << 8 | xlm));
            m_max.y = Math.Max(m_max.y, (Int16) (yhm << 8 | ylm));
            m_max.z = Math.Max(m_max.z, (Int16) (zhm << 8 | zlm));

            vector shiftedValue;

            shiftedValue.x = ((_rawData.x - m_min.x)/(m_max.x - m_min.x)*2 - 1.0);
            shiftedValue.y = ((_rawData.y - m_min.y)/(m_max.y - m_min.y)*2 - 1.0);
            shiftedValue.z = ((_rawData.z - m_min.z)/(m_max.z - m_min.z)*2 - 1.0);

            double heading = Math.Atan2(shiftedValue.y, shiftedValue.x);

            // Correct for when signs are reversed.
            //if (heading < 0)
            //    heading += 2 * Math.PI;

            // Convert radians to degrees for readability.
            double headingDegrees = heading*180/Math.PI + 180;

            if (_instance.MeasurementComplete != null)
                _instance.MeasurementComplete(_instance,
                                              new SensorData(headingDegrees, (int) _rawData.x, (int) _rawData.y,
                                                             (int) _rawData.z));
        }

        #region Static values

        // device types

        public enum magGain
        {
            magGain_13 = 0x20,
            magGain_19 = 0x40,
            magGain_25 = 0x60,
            magGain_40 = 0x80,
            magGain_47 = 0xA0,
            magGain_56 = 0xC0,
            magGain_81 = 0xE0
        };


        private bool IsInitialized;
        private static vector m_min;
        private static vector m_max;
        public const byte LSM303DLH_DEVICE = 0;
        public const byte LSM303DLM_DEVICE = 1;
        public const byte LSM303DLHC_DEVICE = 2;
        public const byte LSM303_DEVICE_AUTO = 3;
        private const int LSM303_OUT_Y_H_M = -1; // The addresses of the Y and Z magnetometer output registers 
        private const int LSM303_OUT_Y_L_M = -2; // are reversed on the DLM and DLHC relative to the DLH.
        private const int LSM303_OUT_Z_H_M = -3; // These four defines have dummy values so the library can 
        private const int LSM303_OUT_Z_L_M = -4; // determine the correct address based on the device type.
        public static byte ADDRESS = (0x3C >> 1);
        private static byte _device = LSM303_DEVICE_AUTO; // chip type (DLH, DLM, or DLHC)
        private static int ClockRate = 50;
        private static int Timeout = 100;

        // SA0_A states

        private static byte LSM303_SA0_A_LOW;
        private static byte LSM303_SA0_A_HIGH = 1;
        private static byte LSM303_SA0_A_AUTO = 2;

        // register addresses

        private static byte LSM303_CTRL_REG1_A = 0x20;
        private static byte LSM303_CTRL_REG2_A = 0x21;
        private static byte LSM303_CTRL_REG3_A = 0x22;
        private static byte LSM303_CTRL_REG4_A = 0x23;
        private static byte LSM303_CTRL_REG5_A = 0x24;
        private static byte LSM303_CTRL_REG6_A = 0x25; // DLHC only
        private static int LSM303_HP_FILTER_RESET_A = 0x25; // DLH, DLM only
        private static int LSM303_REFERENCE_A = 0x26;
        private static int LSM303_STATUS_REG_A = 0x27;

        private static int LSM303_OUT_X_L_A = 0x28;
        private static int LSM303_OUT_X_H_A = 0x29;
        private static int LSM303_OUT_Y_L_A = 0x2A;
        private static int LSM303_OUT_Y_H_A = 0x2B;
        private static int LSM303_OUT_Z_L_A = 0x2C;
        private static int LSM303_OUT_Z_H_A = 0x2D;

        private static int LSM303_FIFO_CTRL_REG_A = 0x2E; // DLHC only
        private static int LSM303_FIFO_SRC_REG_A = 0x2F; // DLHC only

        private static int LSM303_INT1_CFG_A = 0x30;
        private static int LSM303_INT1_SRC_A = 0x31;
        private static int LSM303_INT1_THS_A = 0x32;
        private static int LSM303_INT1_DURATION_A = 0x33;
        private static int LSM303_INT2_CFG_A = 0x34;
        private static int LSM303_INT2_SRC_A = 0x35;
        private static int LSM303_INT2_THS_A = 0x36;
        private static int LSM303_INT2_DURATION_A = 0x37;

        private static int LSM303_CLICK_CFG_A = 0x38; // DLHC only
        private static int LSM303_CLICK_SRC_A = 0x39; // DLHC only
        private static int LSM303_CLICK_THS_A = 0x3A; // DLHC only
        private static int LSM303_TIME_LIMIT_A = 0x3B; // DLHC only
        private static int LSM303_TIME_LATENCY_A = 0x3C; // DLHC only
        private static int LSM303_TIME_WINDOW_A = 0x3D; // DLHC only

        private static byte LSM303_CRA_REG_M;
        private static byte LSM303_CRB_REG_M = 0x01;
        private static byte LSM303_MR_REG_M = 0x02;

        private static int LSM303_OUT_X_H_M = 0x03;
        private static int LSM303_OUT_X_L_M = 0x04;

        private static int LSM303_SR_REG_M = 0x09;
        private static int LSM303_IRA_REG_M = 0x0A;
        private static int LSM303_IRB_REG_M = 0x0B;
        private static int LSM303_IRC_REG_M = 0x0C;

        private static int LSM303_WHO_AM_I_M = 0x0F; // DLM only

        private static int LSM303_TEMP_OUT_H_M = 0x31; // DLHC only
        private static int LSM303_TEMP_OUT_L_M = 0x32; // DLHC only

        private static int LSM303DLH_OUT_Y_H_M = 0x05;
        private static int LSM303DLH_OUT_Y_L_M = 0x06;
        private static int LSM303DLH_OUT_Z_H_M = 0x07;
        private static int LSM303DLH_OUT_Z_L_M = 0x08;

        private static int LSM303DLM_OUT_Z_H_M = 0x05;
        private static int LSM303DLM_OUT_Z_L_M = 0x06;
        private static int LSM303DLM_OUT_Y_H_M = 0x07;
        private static int LSM303DLM_OUT_Y_L_M = 0x08;

        private static int LSM303DLHC_OUT_Z_H_M = 0x05;
        private static int LSM303DLHC_OUT_Z_L_M = 0x06;
        private static int LSM303DLHC_OUT_Y_H_M = 0x07;
        private static int LSM303DLHC_OUT_Y_L_M = 0x08;

        // HEX  = BIN          RANGE    GAIN X/Y/Z        GAIN Z
        //                               DLH (DLM/DLHC)    DLH (DLM/DLHC)
        // 0x20 = 0b00100000   �1.3     1055 (1100)        950 (980) (default)
        // 0x40 = 0b01000000   �1.9      795  (855)        710 (760)
        // 0x60 = 0b01100000   �2.5      635  (670)        570 (600)
        // 0x80 = 0b10000000   �4.0      430  (450)        385 (400)
        // 0xA0 = 0b10100000   �4.7      375  (400)        335 (355)
        // 0xC0 = 0b11000000   �5.6      320  (330)        285 (295)
        // 0xE0 = 0b11100000   �8.1      230  (230)        205 (205)

        #endregion

        #region delgates and events

        /// Represents the delegate used for the <see cref="MeasurementComplete"/> event.
        /// </summary>
        /// <param name="sender">The object that raised the event.</param>
        /// <param name="sensorData">The <see cref="SensorData"/> object that contains the results of the reading.</param>
        public delegate void MeasurementCompleteEventHandler(Magnetometer sender, SensorData sensorData);

        public event MeasurementCompleteEventHandler MeasurementComplete;

        private MeasurementCompleteEventHandler _OnMeasurementComplete;

        //    public static EventHandler<SensorData> MeasurementComplete;

        /// <summary>
        /// Raises the <see cref="MeasurementComplete"/> event.
        /// </summary>
        /// <param name="sender">The object that raised the event.</param>
        /// <param name="sensorData">The <see cref="SensorData"/> object that contains the results of the measurement.</param>
        public void OnMeasurementCompleteEvent(Magnetometer sender, SensorData sensorData)
        {
            if (_OnMeasurementComplete == null)
                _OnMeasurementComplete = OnMeasurementCompleteEvent;
        }

        #endregion

        private Timer continuousTimer;
        private static bool continuousMeasurement;


        private void Init()
        {
            m_min.x = -613;
            m_min.y = -547;
            m_min.z = -564;
            m_max.x = 385;
            m_max.y = 365;
            m_max.z = 444;
            writeMagReg(LSM303_MR_REG_M, 0x02);
            // Enable Magnetometer
            // 0x00 = 0b00000000
            // Continuous conversion mode

            writeMagReg(LSM303_MR_REG_M, 0x00);
            // 15 Hz default
        }

        // Writes a magnetometer register
        private void writeMagReg(byte reg, byte value)
        {
            I2CBus.GetInstance().WriteRegister(new I2CDevice.Configuration(ADDRESS, ClockRate), reg, value, Timeout);
        }

        #region Nested type: vector

        public struct vector
        {
            public double x, y, z;
        };

        #endregion

        public void Dispose()
        {
            throw new NotImplementedException();
        }

        public class SensorData : EventArgs
        {
            /// <summary>
            /// Raw X-axis sensor data.
            /// </summary>
            public int X { get; private set; }

            /// <summary>
            /// Raw Y-axis sensor data.
            /// </summary>
            public int Y { get; private set; }

            /// <summary>
            /// Raw Z-axis sensor data.
            /// </summary>
            public int Z { get; private set; }

            /// <summary>
            /// Angle of heading in the XY plane, in radians.
            /// </summary>
            public double Angle { get; private set; }

            /// <summary>
            /// A set of sensor measurements.
            /// </summary>
            /// <param name="angle">Angle of heading in the XY plane, in radians.</param>
            /// <param name="x">Raw X-axis sensor data.</param>
            /// <param name="y">Raw Y-axis sensor data.</param>
            /// <param name="z">Raw Z-axis sensor data.</param>            
            public SensorData(double angle, int x, int y, int z)
            {
                Angle = angle;
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
                return "Angle: " + Angle.ToString("f2") + " X: " + X.ToString("f2") + " Y: " + Y.ToString("f2") + " Z: " +
                       Z.ToString("f2");
            }
        }
    }
}