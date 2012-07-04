using System;
using Microsoft.SPOT;
using System.Threading;
namespace Toolbox.NETMF.Hardware
{
    public  class L3G4200D
    {

        // for byte data type
        // register addresses
         int ClockRate = 50;
         int Timeout = 100;
        public  byte L3G4200D_WHO_AM_I = 0x0F;
        public  byte L3G4200D_CTRL_REG1 = 0x20;
        public  byte L3G4200D_CTRL_REG2 = 0x21;
        public  byte L3G4200D_CTRL_REG3 = 0x22;
        public  byte L3G4200D_CTRL_REG4 = 0x23;
         byte L3G4200D_CTRL_REG5 = 0x24;
         byte L3G4200D_REFERENCE = 0x25;
         byte L3G4200D_OUT_TEMP = 0x26;
         byte L3G4200D_STATUS_REG = 0x27;
         byte L3G4200D_OUT_X_L = 0x28;
         byte L3G4200D_OUT_X_H = 0x29;
         byte L3G4200D_OUT_Y_L = 0x2A;
         byte L3G4200D_OUT_Y_H = 0x2B;
         byte L3G4200D_OUT_Z_L = 0x2C;
         byte L3G4200D_OUT_Z_H = 0x2D;
         byte L3G4200D_FIFO_CTRL_REG = 0x2E;
         byte L3G4200D_FIFO_SRC_REG = 0x2F;
         byte L3G4200D_INT1_CFG = 0x30;
         byte L3G4200D_INT1_SRC = 0x31;
         byte L3G4200D_INT1_THS_XH = 0x32;
         byte L3G4200D_INT1_THS_XL = 0x33;
         byte L3G4200D_INT1_THS_YH = 0x34;
         byte L3G4200D_INT1_THS_YL = 0x35;
         byte L3G4200D_INT1_THS_ZH = 0x36;
         byte L3G4200D_INT1_THS_ZL = 0x37;
         byte L3G4200D_INT1_DURATION = 0x38;
        public struct vector
        {
            public double x, y, z;
        }

        public  vector g; // gyro angular velocity readings	

         byte GYR_ADDRESS = 0xD2 >> 1;

        // Public Methods //////////////////////////////////////////////////////////////
        // Turns on the L3G4200D's gyro and places it in normal mode.
        public  void enableDefault()
        {
            byte[] ret = new byte[6];
            // 0x0F = 0b00001111	
            // Normal power mode, all axes enabled	
            writeReg(L3G4200D_CTRL_REG1, 0x0F);



        }
        // Writes a gyro register
        public  void writeReg(byte reg, byte value)
        {

            I2CBus.GetInstance().WriteRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(GYR_ADDRESS, ClockRate), reg, value, Timeout);


        }

        public  byte[] readReg(byte reg)
        {
            byte[] ret = new byte[6];

            I2CBus.GetInstance().ReadRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(GYR_ADDRESS, ClockRate), reg, ret, Timeout);



            return ret;
        }


        // Reads the 3 gyro channels and stores them in vector g
        public  void read()
        {
            byte[] ret = new byte[6];

            I2CBus.GetInstance().ReadRegister(new Microsoft.SPOT.Hardware.I2CDevice.Configuration(GYR_ADDRESS, ClockRate), (byte)(L3G4200D_OUT_X_L | (1 << 7)), ret, Timeout);



            int xla = ret[0];
            int xha = ret[1];
            int yla = ret[2];
            int yha = ret[3];
            int zla = ret[4];
            int zha = ret[5];
            g.x = xha << 8 | xla;
            g.y = yha << 8 | yla;
            g.z = zha << 8 | zla;

      
        }
   

    }
}
