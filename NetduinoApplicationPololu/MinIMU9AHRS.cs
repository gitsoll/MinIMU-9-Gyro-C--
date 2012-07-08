using System;
using Microsoft.SPOT;
using System.Threading;

namespace Toolbox.NETMF.Hardware
{
    public static class MinIMU9AHRS
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

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
public static int[] SENSOR_SIGN= new int[9] {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168



// LSM303 accelerometer: 8 g sensitivity
// 3.8 mg/digit; 1 g = 256
public static int GRAVITY= 256;  //this equivalent to 1G in the raw data coming from the accelerometer 

public static double  ToRad(double x) {return (x*System.Math.PI/180);}  // *pi/180
public static double  ToDeg(double x) {return (x*180/System.Math.PI);}  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07

public static double Gyro_Gain_X =0.07; //X axis Gyro gain

public static double Gyro_Gain_Y= 0.07; //Y axis Gyro gain

public static double Gyro_Gain_Z =0.07; //Z axis Gyro gain

public static double Gyro_Scaled_X( double x) { return x*ToRad(Gyro_Gain_X);} //Return the scaled ADC raw data of the gyro in radians for second

public static double Gyro_Scaled_Y(double x){ return x*ToRad(Gyro_Gain_Y);} //Return the scaled ADC raw data of the gyro in radians for second

public static double Gyro_Scaled_Z(double x) {return x*ToRad(Gyro_Gain_Z);} //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
public static int M_X_MIN = -796;
public static int M_Y_MIN =-457;
public static int M_Z_MIN =-424;
public static int M_X_MAX =197;
public static int M_Y_MAX =535;
public static int M_Z_MAX =397;

public static double Kp_ROLLPITCH =0.02;
public static double Ki_ROLLPITCH = 0.00002;
public static double  Kp_YAW  =1.2;
public static double  Ki_YAW =  0.00002;

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
public static int  OUTPUTMODE= 1;

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
public static int PRINT_ANALOGS =0 ;//Will print the analog raw data
public static int PRINT_EULER = 1;   //Will print the Euler angles Roll, Pitch and Yaw

public static int STATUS_LED  = 13; 

public static double G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

static DateTime timer ;   //general purpuse timer
static DateTime timer_old;
static long timer24 = 0; //Second timer used to print values 
public static int[] AN = new int[6]; //array that stores the gyro and accelerometer data
public static int[] AN_OFFSET = new int[6] { 0, 0, 0, 0, 0, 0 }; //Array that stores the Offset of the sensors

public static int gyro_x;
public static int gyro_y;
public static int gyro_z;
public static int accel_x;
public static int accel_y;
public static int accel_z;
public static int magnetom_x;
public static int magnetom_y;
public static int magnetom_z;
public static double c_magnetom_x;
public static double c_magnetom_y;
public static double c_magnetom_z;
public static double MAG_Heading;

public static double[] Accel_Vector = new double[3] { 0, 0, 0 }; //Store the acceleration in a vector
public static double[] Gyro_Vector= new double[3] {0,0,0};//Store the gyros turn rate in a vector
public static double[] Omega_Vector = new double[3] { 0, 0, 0 }; //Corrected Gyro_Vector data
public static double[] Omega_P = new double[3] { 0, 0, 0 };//Omega Proportional correction
public static double[] Omega_I = new double[3] { 0, 0, 0 };//Omega Integrator
public static double[] Omega = new double[3] { 0, 0, 0 };

// Euler angles
public static double roll;
public static double pitch;
public static double yaw;

public static double[] errorRollPitch = new double[3] { 0, 0, 0 };
public static double[] errorYaw = new double[3] { 0, 0, 0 };

static int counter = 0;
static byte gyro_sat = 0;

public static double[][] DCM_Matrix = new double[3][] {
 new double[3] {
    1,0,0  }
  ,new double[3]{
    0,1,0  }
  ,new double[3]{
    0,0,1  }
};
public static double[][] Update_Matrix = new double[3][] { new double[3] { 0, 1, 2 }, new double[3] { 3, 4, 5 }, new double[3] { 6, 7, 8 } }; //Gyros here


public static double[][] Temporary_Matrix = new double[3][]{
 new double[3]{
    0,0,0  }
  ,new double[3]{
    0,0,0  }
  ,new double[3]{
    0,0,0  }
};

public static void setup()
{ 


  Debug.Print("Pololu MinIMU-9 + Arduino AHRS");
 
  I2C.Accel_Init();
  I2C.Compass_Init();
  I2C.Gyro_Init();
  
  Thread.Sleep(20);
  
  for(int i=0;i<32;i++)    // We take some readings...
    {
        I2C.Read_Gyro();
        I2C.Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    Thread.Sleep(20);
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  
  ////Debug.Println("Offset:");
  //for(int y=0; y<6; y++)
  //  Debug.Println(AN_OFFSET[y]);

  Thread.Sleep(2000);


  timer = DateTime.Now;
  Thread.Sleep(20);
  counter=0;
}

public static  void loop() //Main Loop
{
   if (DateTime.Now.Subtract(timer).Milliseconds >= 20)  // Main loop runs at 50Hz
  {
      Debug.Print("timer run");
    counter++;
    timer_old = timer;
    timer = DateTime.Now;
    if (timer>timer_old)
      G_Dt = timer.Subtract(timer_old).Milliseconds/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data adquisition
    I2C.Read_Gyro();   // This read gyro data
    I2C.Read_Accel();     // Read I2C accelerometer
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
      counter=0;
      I2C.Read_Compass();    // Read I2C magnetometer
      LSM303.Compass_Heading(); // Calculate magnetic heading  
      }
    
    // Calculations...
    DCM.Matrix_update();
    DCM.Normalize();
    DCM.Drift_correction();
    DCM.Euler_angles();
    // ***

  }
}
  public static void printdata()
{
   Debug.Print(string.Concat("Roll:", roll.ToString(), " Pitch:", pitch.ToString(), " Yaw:", yaw.ToString(), "Gyro X:", AN[0].ToString(), " Gyro Y:", AN[1].ToString(), " Gyro Z:", AN[2].ToString(), " Acc X:", AN[3].ToString(), " Acc y:", AN[4].ToString(), " Acc Z:", AN[5].ToString(), " Mag x:", magnetom_x.ToString(), " Mag y:", magnetom_y.ToString(), " Mag z:", magnetom_z.ToString()));
      

      //if (PRINT_EULER == 1)
      //{
      //Debug.Print("ANG:");
      //Debug.Print(ToDeg(roll).ToString());
      //Debug.Print(",");
      //Debug.Print(ToDeg(pitch).ToString());
      //Debug.Print(",");
      //Debug.Print(ToDeg(yaw).ToString());
      // }

      //if (PRINT_ANALOGS == 1)
      //{
      //    Debug.Print(",AN:");
      //    Debug.Print(AN[0].ToString());  //(int)read_adc(0)
      //    Debug.Print(",");
      //    Debug.Print(AN[1].ToString());
      //    Debug.Print(",");
      //    Debug.Print(AN[2].ToString());
      //    Debug.Print(",");
      //    Debug.Print(AN[3].ToString());
      //    Debug.Print(",");
      //    Debug.Print(AN[4].ToString());
      //    Debug.Print(",");
      //    Debug.Print(AN[5].ToString());
      //    Debug.Print(",");
      //    Debug.Print(magnetom_x.ToString());
      //    Debug.Print(",");
      //    Debug.Print(magnetom_y.ToString());
      //    Debug.Print(",");
      //    Debug.Print(magnetom_z.ToString());
      //}

   // Debug.Print(String.Concat("DCM", ToDeg(DCM_Matrix[0][0]).ToString(), ",", ToDeg(DCM_Matrix[0][1]).ToString(), ",", ToDeg(DCM_Matrix[0][2]).ToString(), ",", ToDeg(DCM_Matrix[1][0]).ToString(), ",", ToDeg(DCM_Matrix[1][1]).ToString(), ",", ToDeg(DCM_Matrix[1][2]).ToString(), ",", ToDeg(DCM_Matrix[2][0]).ToString(), ",", ToDeg(DCM_Matrix[2][1]).ToString(), ",", ToDeg(DCM_Matrix[2][2]).ToString()));

      /*#if PRINT_DCM == 1
      Debug.Print (",DCM:");
      Debug.Print(convert_to_dec(DCM_Matrix[0][0]));
      Debug.Print (",");
      Debug.Print(convert_to_dec(DCM_Matrix[0][1]));
      Debug.Print (",");
      Debug.Print(convert_to_dec(DCM_Matrix[0][2]));
      Debug.Print (",");
      Debug.Print(convert_to_dec(DCM_Matrix[1][0]));
      Debug.Print (",");
      Debug.Print(convert_to_dec(DCM_Matrix[1][1]));
      Debug.Print (",");
      Debug.Print(convert_to_dec(DCM_Matrix[1][2]));
      Debug.Print (",");
      Debug.Print(convert_to_dec(DCM_Matrix[2][0]));
      Debug.Print (",");
      Debug.Print(convert_to_dec(DCM_Matrix[2][1]));
      Debug.Print (",");
      Debug.Print(convert_to_dec(DCM_Matrix[2][2]));
      #endif*/
 
      

}

   
    }


}
