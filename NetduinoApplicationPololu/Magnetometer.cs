using System;
using Microsoft.SPOT.Hardware;


namespace NetduinoApplicationPololu
{
    public sealed  class Magnetometer : IDisposable
    {
        private static Magnetometer _instance = null;
        private static readonly object LockObject = new object();

        private Magnetometer()
        {
            
        }
        public static  Magnetometer GetInstance()
        {
            lock (LockObject)
            {
                if (_instance == null)
                {
                    _instance = new Magnetometer();
                    _instance.Init();
                    for (int i = 0; i < 20; i++)
                    {
                        _instance.Read();
                        //m_min.x = Math.Min(m_min.x, _instance.Data.x);
                        //m_min.y = Math.Min(m_min.y, _instance.Data.x);
                        //m_min.z = Math.Min(m_min.z, _instance.Data.x);
                    }

             
                }
                return _instance;
            }
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


        private bool IsInitialized = false;
        vector m_min = new vector();
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

        public static byte LSM303_SA0_A_LOW;
        public static byte LSM303_SA0_A_HIGH = 1;
        public static byte LSM303_SA0_A_AUTO = 2;

        // register addresses

        public static byte LSM303_CTRL_REG1_A = 0x20;
        public static byte LSM303_CTRL_REG2_A = 0x21;
        public static byte LSM303_CTRL_REG3_A = 0x22;
        public static byte LSM303_CTRL_REG4_A = 0x23;
        public static byte LSM303_CTRL_REG5_A = 0x24;
        public static byte LSM303_CTRL_REG6_A = 0x25; // DLHC only
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

        public static byte LSM303_CRA_REG_M;
        public static byte LSM303_CRB_REG_M = 0x01;
        public static byte LSM303_MR_REG_M = 0x02;

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
        // 0x20 = 0b00100000   ±1.3     1055 (1100)        950 (980) (default)
        // 0x40 = 0b01000000   ±1.9      795  (855)        710 (760)
        // 0x60 = 0b01100000   ±2.5      635  (670)        570 (600)
        // 0x80 = 0b10000000   ±4.0      430  (450)        385 (400)
        // 0xA0 = 0b10100000   ±4.7      375  (400)        335 (355)
        // 0xC0 = 0b11000000   ±5.6      320  (330)        285 (295)
        // 0xE0 = 0b11100000   ±8.1      230  (230)        205 (205)

        #endregion

        public vector _rawData;

        public double Direction
        {
            get
            {
                double heading = Math.Atan2(_rawData.x, _rawData.y);

                // Correct for when signs are reversed.
                if (heading < 0)
                    heading += 2 * Math.PI;

                // Convert radians to degrees for readability.
                double headingDegrees = heading * 180 / Math.PI;

                return headingDegrees;
            }
           
        }

        void Init()
        {
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

        // Reads the 3 magnetometer channels and stores them in vector m
        public void Read()
        {
            var values = new byte[6];


            I2CBus.GetInstance().ReadRegister(new I2CDevice.Configuration(ADDRESS, ClockRate), (byte) LSM303_OUT_X_H_M,
                                              values, Timeout);

            byte xhm = values[0];
            byte xlm = values[1];


            byte yhm, ylm, zhm, zlm;

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

            // Transform 2’s complement to doubles
            _rawData.x = (xhm << 8 | xlm);
            _rawData.y = (yhm << 8 | ylm);
            _rawData.z = (zhm << 8 | zlm);
            
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
    }
}