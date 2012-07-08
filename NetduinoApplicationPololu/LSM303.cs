using System;
using Microsoft.SPOT;

namespace Toolbox.NETMF.Hardware
{
    public static class LSM303
    {
         static int ClockRate = 50;
         static int Timeout = 100;
        // device types

         public  const byte LSM303DLH_DEVICE = 0;
         public  const byte LSM303DLM_DEVICE = 1;
         public  const byte LSM303DLHC_DEVICE = 2;
         public  const byte LSM303_DEVICE_AUTO = 3;

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

         public static byte LSM303_CRA_REG_M = 0x00;
         public static byte LSM303_CRB_REG_M = 0x01;
         public static byte LSM303_MR_REG_M = 0x02;

     static int LSM303_OUT_X_H_M = 0x03;
     static int LSM303_OUT_X_L_M = 0x04;
        const int LSM303_OUT_Y_H_M = -1; // The addresses of the Y and Z magnetometer output registers 
        const int LSM303_OUT_Y_L_M = -2; // are reversed on the DLM and DLHC relative to the DLH.
        const int LSM303_OUT_Z_H_M = -3; // These four defines have dummy values so the library can 
        const int LSM303_OUT_Z_L_M = -4; // determine the correct address based on the device type.

        static int LSM303_SR_REG_M = 0x09;
        static int LSM303_IRA_REG_M = 0x0A;
        static int LSM303_IRB_REG_M = 0x0B;
        static int LSM303_IRC_REG_M = 0x0C;

         static int LSM303_WHO_AM_I_M = 0x0F;// DLM only
  
        static   int LSM303_TEMP_OUT_H_M = 0x31; // DLHC only
         static int LSM303_TEMP_OUT_L_M = 0x32;// DLHC only

        static   int LSM303DLH_OUT_Y_H_M = 0x05;
        static int LSM303DLH_OUT_Y_L_M = 0x06;
        static int LSM303DLH_OUT_Z_H_M = 0x07;
        static int LSM303DLH_OUT_Z_L_M = 0x08;

        static int LSM303DLM_OUT_Z_H_M = 0x05;
        static int LSM303DLM_OUT_Z_L_M = 0x06;
        static int LSM303DLM_OUT_Y_H_M = 0x07;
        static int LSM303DLM_OUT_Y_L_M = 0x08;

        static int LSM303DLHC_OUT_Z_H_M = 0x05;
        static int LSM303DLHC_OUT_Z_L_M = 0x06;
        static int LSM303DLHC_OUT_Y_H_M = 0x07;
        static int LSM303DLHC_OUT_Y_L_M = 0x08;








        public struct vector
        {
            public double x, y, z;
        } ;

        public static vector a; // accelerometer readings
        public static vector m; // magnetometer readings
        public static vector m_max = new vector()
        {
            x = 540,
            y = 500,
            z = 180
        }; // maximum magnetometer values, used for calibration
        public static vector m_min = new vector()
        {
            x = -520,
            y = -570,
            z = -770
        }; // minimum magnetometer values, used for calibration

        public static byte last_status; // status of last I2C transmission

        // HEX  = BIN          RANGE    GAIN X/Y/Z        GAIN Z
        //                               DLH (DLM/DLHC)    DLH (DLM/DLHC)
        // 0x20 = 0b00100000   ±1.3     1055 (1100)        950 (980) (default)
        // 0x40 = 0b01000000   ±1.9      795  (855)        710 (760)
        // 0x60 = 0b01100000   ±2.5      635  (670)        570 (600)
        // 0x80 = 0b10000000   ±4.0      430  (450)        385 (400)
        // 0xA0 = 0b10100000   ±4.7      375  (400)        335 (355)
        // 0xC0 = 0b11000000   ±5.6      320  (330)        285 (295)
        // 0xE0 = 0b11100000   ±8.1      230  (230)        205 (205)
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

      
      static    int io_timeout = 0;  // 0 = no timeout
        static bool did_timeout = false;

        public static byte MAG_ADDRESS = (0x3C >> 1);
        public static byte ACC_ADDRESS_SA0_A_LOW = (0x30 >> 1);
        public static byte ACC_ADDRESS_SA0_A_HIGH = (0x32 >> 1);

      static  byte _device = LSM303_DEVICE_AUTO; // chip type (DLH, DLM, or DLHC)
      static   byte acc_address = (0x30 >> 1);

        public static bool timeoutOccurred()
        {
            return did_timeout;
        }

        public static  void setTimeout(int timeout)
        {
            io_timeout = timeout;
        }

        public static  int getTimeout()
        {
            return io_timeout;
        }

        public static void init(byte device, byte sa0_a)
        {
            _device = device;
            switch (_device)
            {
                case LSM303DLH_DEVICE:
                case LSM303DLM_DEVICE:
                    if (sa0_a == LSM303_SA0_A_LOW)
                        acc_address = ACC_ADDRESS_SA0_A_LOW;
                    else if (sa0_a == LSM303_SA0_A_HIGH)
                        acc_address = ACC_ADDRESS_SA0_A_HIGH;
                    else
                        acc_address = (detectSA0_A() == LSM303_SA0_A_HIGH) ? ACC_ADDRESS_SA0_A_HIGH : ACC_ADDRESS_SA0_A_LOW;
                    break;

                case LSM303DLHC_DEVICE:
                    acc_address = ACC_ADDRESS_SA0_A_HIGH;
                    break;

                default:
                    // try to auto-detect device
                    if (detectSA0_A() == LSM303_SA0_A_HIGH)
                    {
                        // if device responds on 0011001b (SA0_A is high), assume DLHC
                        acc_address = ACC_ADDRESS_SA0_A_HIGH;
                        _device = LSM303DLHC_DEVICE;
                    }
                    else
                    {
                        // otherwise, assume DLH or DLM (pulled low by default on Pololu boards); query magnetometer WHO_AM_I to differentiate these two
                        acc_address = ACC_ADDRESS_SA0_A_LOW;
                        _device = (readMagReg(LSM303_WHO_AM_I_M) == 0x3C) ? LSM303DLM_DEVICE : LSM303DLH_DEVICE;
                    }
                    break;
            }
        }
        // Turns on the LSM303's accelerometer and magnetometers and places them in normal
        // mode.
        public static void enableDefault()
        {
            // Enable Accelerometer
            // 0x27 = 0b00100111
            // Normal power mode, all axes enabled
            Debug.Print("LSM303_CTRL_REG1_A");
       writeAccReg(LSM303_CTRL_REG1_A, 0x27);

            // Enable Magnetometer
            // 0x00 = 0b00000000
            // Continuous conversion mode
       Debug.Print("LSM303_MR_REG_M");
            writeMagReg(LSM303_MR_REG_M, 0x00);
        }
        // Writes an accelerometer register
        public static void writeAccReg(byte reg, byte value)
        {
            byte[] values = new byte[6];

            I2CBus.GetInstance().WriteRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(ACC_ADDRESS_SA0_A_LOW, ClockRate), reg, value, Timeout);

        }

        // Reads an accelerometer register
        public static  byte readAccReg(byte reg)
        {
            var values = new byte[6];

            I2CBus.GetInstance().ReadRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(acc_address, ClockRate), reg, values, Timeout);




            return values[0];
        }

        // Writes a magnetometer register
        public static void writeMagReg(byte reg, byte value)
        {

            I2CBus.GetInstance().WriteRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(MAG_ADDRESS, ClockRate), reg, value, Timeout);
        }

        // Reads a magnetometer register
        public static byte readMagReg(int reg)
        {
            byte value;

            // if dummy register address (magnetometer Y/Z), use device type to determine actual address
            if (reg < 0)
            {
                switch (reg)
                {
                    case LSM303_OUT_Y_H_M:
                        reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_H_M : LSM303DLM_OUT_Y_H_M;
                        break;
                    case LSM303_OUT_Y_L_M:
                        reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_L_M : LSM303DLM_OUT_Y_L_M;
                        break;
                    case LSM303_OUT_Z_H_M:
                        reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_H_M : LSM303DLM_OUT_Z_H_M;
                        break;
                    case LSM303_OUT_Z_L_M:
                        reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_L_M : LSM303DLM_OUT_Z_L_M;
                        break;
                }
            }

            var values = new byte[6];

            I2CBus.GetInstance().ReadRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(MAG_ADDRESS, ClockRate), (byte)reg, values, Timeout);


            return values[0];
        }

        public static void setMagGain(magGain value)
        {
            I2CBus.GetInstance().WriteRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(MAG_ADDRESS, ClockRate), (byte)LSM303_CRB_REG_M, (byte)value, Timeout);
        }


        // Reads the 3 accelerometer channels and stores them in vector a
        public static void readAcc()
        {

            var values = new byte[6];


            I2CBus.GetInstance().ReadRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(acc_address, ClockRate), (byte)LSM303_OUT_X_L_A, values, Timeout);


            did_timeout = false;



            byte xla = values[0];
            byte xha = values[1];
            byte yla = values[2];
            byte yha = values[3];
            byte zla = values[4];
            byte zha = values[5];

            a.x = (xha << 8 | xla) >> 4;
            a.y = (yha << 8 | yla) >> 4;
            a.z = (zha << 8 | zla) >> 4;
        }

        // Reads the 3 magnetometer channels and stores them in vector m
        public static  void readMag()
        {

            var values = new byte[6];


            I2CBus.GetInstance().ReadRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(MAG_ADDRESS, ClockRate), (byte)LSM303_OUT_X_H_M, values, Timeout);


            did_timeout = false;

            byte xhm = values[0];
            byte xlm = values[1];


            byte yhm, ylm, zhm, zlm;

            if (_device == LSM303DLH_DEVICE)
            {
                // DLH: register address for Y comes before Z
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

            m.x = (xhm << 8 | xlm);
            m.y = (yhm << 8 | ylm);
            m.z = (zhm << 8 | zlm);
        }
        // Reads all 6 channels of the LSM303 and stores them in the object variables
        public static void read()
        {
            readAcc();
            readMag();
        }

        // Returns the number of degrees from the -Y axis that it
        // is pointing.
        public static double heading()
        {
            vector a = new vector()
            {
                x = 0,
                y = -1,
                z = 0
            };
            return heading(a);
        }
        // Returns the number of degrees from the From vector projected into
        // the horizontal plane is away from north.
        // 
        // Description of heading algorithm: 
        // Shift and scale the magnetic reading based on calibration data to
        // to find the North vector. Use the acceleration readings to
        // determine the Down vector. The cross product of North and Down
        // vectors is East. The vectors East and North form a basis for the
        // horizontal plane. The From vector is projected into the horizontal
        // plane and the angle between the projected vector and north is
        // returned.
        public static double heading(vector from)
        {
            // shift and scale
            m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
            m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
            m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

            vector temp_a = a;
            // normalize
            vector_normalize(temp_a);
            //vector_normalize(&m);

            // compute E and N
            vector E;
            vector N;
            vector_cross(m, temp_a, out E);
            vector_normalize(E);
            vector_cross(temp_a, E, out  N);

            // compute heading
            double heading = System.Math.Round(System.Math.Atan2(vector_dot(E, from), vector_dot(N, from)) * 180 / System.Math.PI);
            if (heading < 0)
                heading += 360;
            return heading;
        }

        public static void vector_cross(vector a, vector b, out vector o)
        {
            o.x = a.y * b.z - a.z * b.y;
            o.y = a.z * b.x - a.x * b.z;
            o.z = a.x * b.y - a.y * b.x;
        }

        public static double vector_dot(vector a, vector b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        public static void vector_normalize(vector a)
        {
            double mag = System.Math.Sqrt(vector_dot(a, a));
            a.x /= mag;
            a.y /= mag;
            a.z /= mag;
        }


        public static void Compass_Heading()
        {
            double MAG_X;
            double MAG_Y;
            double cos_roll;
            double sin_roll;
            double cos_pitch;
            double sin_pitch;

            cos_roll = (double)System.Math.Cos(MinIMU9AHRS.roll);
            sin_roll = (double)System.Math.Sin(MinIMU9AHRS.roll);
            cos_pitch = (double)System.Math.Cos(MinIMU9AHRS.pitch);
            sin_pitch = (double)System.Math.Sin(MinIMU9AHRS.pitch);

            // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
            MinIMU9AHRS.c_magnetom_x = (double)((MinIMU9AHRS.magnetom_x - MinIMU9AHRS.SENSOR_SIGN[6] * MinIMU9AHRS.M_X_MIN) / (MinIMU9AHRS.M_X_MAX - MinIMU9AHRS.M_X_MIN) - MinIMU9AHRS.SENSOR_SIGN[6] * 0.5);
            MinIMU9AHRS.c_magnetom_y = (double)((MinIMU9AHRS.magnetom_y - MinIMU9AHRS.SENSOR_SIGN[7] * MinIMU9AHRS.M_Y_MIN) / (MinIMU9AHRS.M_Y_MAX - MinIMU9AHRS.M_Y_MIN) - MinIMU9AHRS.SENSOR_SIGN[7] * 0.5);
            MinIMU9AHRS.c_magnetom_z = (double)((MinIMU9AHRS.magnetom_z - MinIMU9AHRS.SENSOR_SIGN[8] * MinIMU9AHRS.M_Z_MIN) / (MinIMU9AHRS.M_Z_MAX - MinIMU9AHRS.M_Z_MIN) - MinIMU9AHRS.SENSOR_SIGN[8] * 0.5);

            // Tilt compensated Magnetic filed X:
            MAG_X = MinIMU9AHRS.c_magnetom_x * cos_pitch + MinIMU9AHRS.c_magnetom_y * sin_roll * sin_pitch + MinIMU9AHRS.c_magnetom_z * cos_roll * sin_pitch;
            // Tilt compensated Magnetic filed Y:
            MAG_Y = MinIMU9AHRS.c_magnetom_y * cos_roll - MinIMU9AHRS.c_magnetom_z * sin_roll;
            // Magnetic Heading
            MinIMU9AHRS.MAG_Heading = (double)System.Math.Atan2(-MAG_Y, MAG_X);
        }

        // Private Methods //////////////////////////////////////////////////////////////

        static int detectSA0_A()
        {
            byte[] value = new byte[6];

            I2CBus.GetInstance().ReadRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(ACC_ADDRESS_SA0_A_LOW, ClockRate), LSM303_CTRL_REG1_A, value, Timeout);

            if (value[0] == 0)
                return LSM303_SA0_A_LOW;
            else
                return LSM303_SA0_A_HIGH;
        }


    }
}
