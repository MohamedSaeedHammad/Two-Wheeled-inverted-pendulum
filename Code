// IMU_Parameters
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

//IMU_Parameters_End

//===============================================

//Motor_Parameters

//#define L_motor1 8
#define L_motor_dir 7
#define L_motor_E 6

//#define R_motor1 9
#define R_motor_dir 7
#define R_motor_E 6

//Motor_Parameters_End

//===============================================

//PID_Parameters
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,5.0,0.7,0, DIRECT);  //5.7  0.35
//PID_Parameters_End

//===============================================
int reading = 0;
int lastDebounceTime = 0;
int debounceDelay = 250;
int LastState = 0;
int prev_out = 0;

void setup() {
  // IMU_Setup
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  
//IMU_Setup_End
  
//=======================================

//Motor_setup
//pinMode(L_motor1,OUTPUT);
pinMode(L_motor_dir,OUTPUT);
pinMode(L_motor_E,OUTPUT);

//pinMode(R_motor1,OUTPUT);
pinMode(R_motor_dir,OUTPUT);
pinMode(R_motor_E,OUTPUT);
//Motor_setup_end

//=======================================

//PID_Setup

//initialize the variables we're linked to
  //Input = imu_read();
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255,255);
  myPID.SetSampleTime(60);
  
//PID_Setup_End
}

void loop() {
 
 Input = imu_read();
 myPID.Compute();
/* 
 // Debounce 
 if(Output > 0)
 {
   reading = 1;
 }
 else 
 {
   reading = 0; 
 }
 if (reading != LastState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
   
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != LastState) {
      LastState = reading;
      Output = prev_out;
    }
  }
 }

  prev_out = Output;
 
  //Debounce_End
  //=============================================
 */
 
 // soft change
 
 if (Output > 20 + prev_out){
   Output = 20 + prev_out;
 }
 else if(Output < prev_out - 20){
   Output = prev_out - 20;
 } 
 prev_out = Output; //=================================
 
 
 Serial.print(Output);
 Serial.print("\t");
 Serial.print(imu_read()); 
 Serial.print("\n");
 delay(2);
 if (Output > 20){
 //digitalWrite(R_motor1,0);
 //digitalWrite(R_motor2,1);
 //analogWrite(R_motor_E,Output);
 //digitalWrite(L_motor1,0);
       digitalWrite(L_motor_dir,0);
       analogWrite(L_motor_E,abs(Output));
 }
 if (Output < -20){
 //digitalWrite(R_motor1,1);
 //digitalWrite(R_motor2,0);
 //analogWrite(R_motor_E,abs(Output));
 //digitalWrite(L_motor1,1);
             digitalWrite(L_motor_dir,1);
          analogWrite(L_motor_E,abs(Output));
 }

 
 
}
