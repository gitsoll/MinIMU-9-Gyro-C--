using System;
using Microsoft.SPOT;

namespace Toolbox.NETMF.Hardware
{
   public static class I2C
    {
        /*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/



static L3G4200D gyro= new L3G4200D();



public static void Gyro_Init()
{
    gyro.writeReg(gyro.L3G4200D_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
    gyro.writeReg(gyro.L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale
}

public static void Read_Gyro()
{
  gyro.read();

  MinIMU9AHRS.AN[0] = (int)gyro.g.x;
  MinIMU9AHRS.AN[1] = (int)gyro.g.y;
  MinIMU9AHRS.AN[2] = (int)gyro.g.z;
  MinIMU9AHRS.gyro_x = MinIMU9AHRS.SENSOR_SIGN[0] * (MinIMU9AHRS.AN[0] - MinIMU9AHRS.AN_OFFSET[0]);
  MinIMU9AHRS.gyro_y = MinIMU9AHRS.SENSOR_SIGN[1] * (MinIMU9AHRS.AN[1] - MinIMU9AHRS.AN_OFFSET[1]);
  MinIMU9AHRS.gyro_z = MinIMU9AHRS.SENSOR_SIGN[2] * (MinIMU9AHRS.AN[2] - MinIMU9AHRS.AN_OFFSET[2]);
}

public static void Accel_Init()
{
    LSM303.writeAccReg(LSM303.LSM303_CTRL_REG1_A, 0x27); // normal power mode, all axes enabled, 50 Hz
    LSM303.writeAccReg(LSM303.LSM303_CTRL_REG4_A, 0x30); // 8 g full scale
}

// Reads x,y and z accelerometer registers
public static void Read_Accel()
{
    LSM303.readAcc();

    MinIMU9AHRS.AN[3] = (int)LSM303.a.x;
    MinIMU9AHRS.AN[4] = (int)LSM303.a.y;
    MinIMU9AHRS.AN[5] = (int)LSM303.a.z;
  MinIMU9AHRS.accel_x = MinIMU9AHRS.SENSOR_SIGN[3] * (MinIMU9AHRS.AN[3] - MinIMU9AHRS.AN_OFFSET[3]);
  MinIMU9AHRS.accel_y = MinIMU9AHRS.SENSOR_SIGN[4] * (MinIMU9AHRS.AN[4] - MinIMU9AHRS.AN_OFFSET[4]);
  MinIMU9AHRS.accel_z = MinIMU9AHRS.SENSOR_SIGN[5] * (MinIMU9AHRS.AN[5] - MinIMU9AHRS.AN_OFFSET[5]);
}

public static void Compass_Init()
{
    LSM303.enableDefault();
    LSM303.writeMagReg(LSM303.LSM303_MR_REG_M, 0x00); // continuous conversion mode
  // 15 Hz default
}

public static void Read_Compass()
{
    LSM303.readMag();

    MinIMU9AHRS.magnetom_x = (int)(MinIMU9AHRS.SENSOR_SIGN[6] * LSM303.m.x);
    MinIMU9AHRS.magnetom_y = (int)(MinIMU9AHRS.SENSOR_SIGN[7] * LSM303.m.y);
    MinIMU9AHRS.magnetom_z = (int)(MinIMU9AHRS.SENSOR_SIGN[8] * LSM303.m.z);
}

    }
}
