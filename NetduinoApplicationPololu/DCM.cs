using System;
using Microsoft.SPOT;

namespace Toolbox.NETMF.Hardware
{
   public  static class DCM
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

/**************************************************/
public static void Normalize()
{
  double error=0;
  double[][] temporary= new double [3][]{new double [3],new double [3],new double [3]};
  double renorm=0;


  error =(double)( -Vector.Vector_Dot_Product((double[])MinIMU9AHRS.DCM_Matrix[0], (double[])MinIMU9AHRS.DCM_Matrix[1]) * .5); //eq.19

  temporary[0] = Vector.Vector_Scale(MinIMU9AHRS.DCM_Matrix[1], error); //eq.19
  temporary[1] = Vector.Vector_Scale(MinIMU9AHRS.DCM_Matrix[0], error); //eq.19
  
  temporary[0] = Vector.Vector_Add( temporary[0], MinIMU9AHRS.DCM_Matrix[0]);//eq.19
  temporary[1] = Vector.Vector_Add( temporary[1], MinIMU9AHRS.DCM_Matrix[1]);//eq.19
  
  temporary[2] = Vector.Vector_Cross_Product(temporary[0],temporary[1]); // c= a x b //eq.20
  
  renorm= (double).5 *(3 - Vector.Vector_Dot_Product(temporary[0],temporary[0])); //eq.21
  MinIMU9AHRS.DCM_Matrix[0] = Vector.Vector_Scale( temporary[0], renorm);

  renorm = (double).5 * (3 - Vector.Vector_Dot_Product(temporary[1], temporary[1])); //eq.21
  MinIMU9AHRS.DCM_Matrix[1] = Vector.Vector_Scale( temporary[1], renorm);

  renorm = (double).5 * (3 - Vector.Vector_Dot_Product(temporary[2], temporary[2])); //eq.21
  MinIMU9AHRS.DCM_Matrix[2] = Vector.Vector_Scale( temporary[2], renorm);
}

/**************************************************/
public static void Drift_correction()
{
  double mag_heading_x;
  double mag_heading_y;
  double errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
   double[] Scaled_Omega_P = new double[3];
   double[] Scaled_Omega_I = new double[3];
  double Accel_magnitude;
  double Accel_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = (double)System.Math.Sqrt(MinIMU9AHRS.Accel_Vector[0] * MinIMU9AHRS.Accel_Vector[0] + MinIMU9AHRS.Accel_Vector[1] * MinIMU9AHRS.Accel_Vector[1] + MinIMU9AHRS.Accel_Vector[2] * MinIMU9AHRS.Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / MinIMU9AHRS.GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  var _wtemp = 1 - 2 * System.Math.Abs(1 - Accel_magnitude);
  if (_wtemp < 0.5)
      Accel_weight = 0;
  else if(_wtemp>1.5)
      Accel_weight = 0;
  else
      Accel_weight=1;
//  Accel_weight = constrain(1 - 2*System.Math.Abs(1 - Accel_magnitude),0,1);  //  

  MinIMU9AHRS.errorRollPitch = Vector.Vector_Cross_Product(MinIMU9AHRS.Accel_Vector, MinIMU9AHRS.DCM_Matrix[2]); //adjust the ground of reference
  MinIMU9AHRS.Omega_P = Vector.Vector_Scale(MinIMU9AHRS.errorRollPitch,(double) MinIMU9AHRS.Kp_ROLLPITCH * Accel_weight);

  Scaled_Omega_I = Vector.Vector_Scale(MinIMU9AHRS.errorRollPitch,(double) MinIMU9AHRS.Ki_ROLLPITCH * Accel_weight);
  MinIMU9AHRS.Omega_I = Vector.Vector_Add(MinIMU9AHRS.Omega_I, Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading

  mag_heading_x = (double)System.Math.Cos(MinIMU9AHRS.MAG_Heading);
  mag_heading_y = (double)System.Math.Sin(MinIMU9AHRS.MAG_Heading);
  errorCourse = (MinIMU9AHRS.DCM_Matrix[0][0] * mag_heading_y) - (MinIMU9AHRS.DCM_Matrix[1][0] * mag_heading_x);  //Calculating YAW error
  MinIMU9AHRS.errorYaw = Vector.Vector_Scale(MinIMU9AHRS.DCM_Matrix[2], errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

  Scaled_Omega_P = Vector.Vector_Scale(MinIMU9AHRS.errorYaw, (double)MinIMU9AHRS.Kp_YAW);//.01proportional of YAW.
  MinIMU9AHRS.Omega_P = Vector.Vector_Add( MinIMU9AHRS.Omega_P, Scaled_Omega_P);//Adding  Proportional.

 Scaled_Omega_I =  Vector.Vector_Scale( MinIMU9AHRS.errorYaw,(double) MinIMU9AHRS.Ki_YAW);//.00001Integrator
  MinIMU9AHRS.Omega_I = Vector.Vector_Add( MinIMU9AHRS.Omega_I, Scaled_Omega_I);//adding integrator to the Omega_I
}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
}
*/
/**************************************************/

public static void Matrix_update()
{
    MinIMU9AHRS.Gyro_Vector[0] =(double) MinIMU9AHRS.Gyro_Scaled_X(MinIMU9AHRS.gyro_x); //gyro x roll
    MinIMU9AHRS.Gyro_Vector[1] = (double)MinIMU9AHRS.Gyro_Scaled_Y(MinIMU9AHRS.gyro_y); //gyro y pitch
    MinIMU9AHRS.Gyro_Vector[2] = (double)MinIMU9AHRS.Gyro_Scaled_Z(MinIMU9AHRS.gyro_z); //gyro Z yaw

    MinIMU9AHRS.Accel_Vector[0] = MinIMU9AHRS.accel_x;
    MinIMU9AHRS.Accel_Vector[1] = MinIMU9AHRS.accel_y;
    MinIMU9AHRS.Accel_Vector[2] = MinIMU9AHRS.accel_z;

    MinIMU9AHRS.Omega = Vector.Vector_Add( MinIMU9AHRS.Gyro_Vector, MinIMU9AHRS.Omega_I);  //adding proportional term
   MinIMU9AHRS.Omega_Vector =  Vector.Vector_Add( MinIMU9AHRS.Omega, MinIMU9AHRS.Omega_P); //adding Integrator term

  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement

   if (MinIMU9AHRS.OUTPUTMODE == 1)
  {
      MinIMU9AHRS.Update_Matrix[0][0] = 0;
      MinIMU9AHRS.Update_Matrix[0][1] =(double) -MinIMU9AHRS.G_Dt * MinIMU9AHRS.Omega_Vector[2];//-z
      MinIMU9AHRS.Update_Matrix[0][2] = (double)MinIMU9AHRS.G_Dt * MinIMU9AHRS.Omega_Vector[1];//y
      MinIMU9AHRS.Update_Matrix[1][0] = (double)MinIMU9AHRS.G_Dt * MinIMU9AHRS.Omega_Vector[2];//z
      MinIMU9AHRS.Update_Matrix[1][1] = 0;
      MinIMU9AHRS.Update_Matrix[1][2] = (double)-MinIMU9AHRS.G_Dt * MinIMU9AHRS.Omega_Vector[0];//-x
      MinIMU9AHRS.Update_Matrix[2][0] = (double)-MinIMU9AHRS.G_Dt * MinIMU9AHRS.Omega_Vector[1];//-y
      MinIMU9AHRS.Update_Matrix[2][1] = (double)MinIMU9AHRS.G_Dt * MinIMU9AHRS.Omega_Vector[0];//x
      MinIMU9AHRS.Update_Matrix[2][2] = 0;
  }
  else                    // Uncorrected data (no drift correction)
  {
      MinIMU9AHRS.Update_Matrix[0][0] = 0;
      MinIMU9AHRS.Update_Matrix[0][1] = (double)-MinIMU9AHRS.G_Dt * MinIMU9AHRS.Gyro_Vector[2];//-z
      MinIMU9AHRS.Update_Matrix[0][2] = (double)MinIMU9AHRS.G_Dt * MinIMU9AHRS.Gyro_Vector[1];//y
      MinIMU9AHRS.Update_Matrix[1][0] = (double)MinIMU9AHRS.G_Dt * MinIMU9AHRS.Gyro_Vector[2];//z
      MinIMU9AHRS.Update_Matrix[1][1] = 0;
      MinIMU9AHRS.Update_Matrix[1][2] = (double)-MinIMU9AHRS.G_Dt * MinIMU9AHRS.Gyro_Vector[0];
      MinIMU9AHRS.Update_Matrix[2][0] = (double)-MinIMU9AHRS.G_Dt * MinIMU9AHRS.Gyro_Vector[1];
      MinIMU9AHRS.Update_Matrix[2][1] = (double)MinIMU9AHRS.G_Dt * MinIMU9AHRS.Gyro_Vector[0];
      MinIMU9AHRS.Update_Matrix[2][2] = 0;
  }

  Matrix.Matrix_Multiply(MinIMU9AHRS.DCM_Matrix, MinIMU9AHRS.Update_Matrix, MinIMU9AHRS.Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
        MinIMU9AHRS.DCM_Matrix[x][y] += MinIMU9AHRS.Temporary_Matrix[x][y];
    } 
  }
}

public static void Euler_angles()
{
    MinIMU9AHRS.pitch = (double)-System.Math.Asin(MinIMU9AHRS.DCM_Matrix[2][0]);
    MinIMU9AHRS.roll =(double) System.Math.Atan2(MinIMU9AHRS.DCM_Matrix[2][1], MinIMU9AHRS.DCM_Matrix[2][2]);
    MinIMU9AHRS.yaw = (double)System.Math.Atan2(MinIMU9AHRS.DCM_Matrix[1][0], MinIMU9AHRS.DCM_Matrix[0][0]);
}

    }
}
