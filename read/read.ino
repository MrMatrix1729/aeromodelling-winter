#include <Wire.h>
#include <MPU6050.h>
#include "Servo.h"

MPU6050 mpu;
Servo elevatorL;
Servo elevatorR;
Servo rudder;

#define elevatorPinL 9
#define elevatorPinR 10
#define rudderPin 11

#define FREQUENCY 100

// PID parameters
double setpoint = 0.0;  // Desired rotation (in degrees)
double kp = 5.0;        // Proportional gain
double ki = 0.1;        // Integral gain
double kd = 0.01;       // Derivative gain
double integral = 0.0;  // Changing integral gain

// Modular control
int dir = 1;

// Stores error in prior run
double previousError = [0,0,0];

// Sensor Errors
double accXError = 0.0;
double accYError = 0.0;
double gyroXError = 0.0;
double gyroYError = 0.0;
double gyroZError = 0.0;

// angle
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;


void setup() {
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();

  // Attach the Servo variable to a pin:
  elevatorL.attach(elevatorPinL);
  elevatorR.attach(elevatorPinR);
  rudder.attach(rudderPin);

  // Calibrating and finding the intial errors in the sensor outputs
  calibrate();
}

void loop() {
  // Read gyroscope data
  mpu.getMotion6();

  // Store gyroscope data in format we need with adjustments for error, 16384 and 131 are taken from datasheet of MPU6050 to convert raw values
  accX = mpu.getAccelerationX() / 16384.0;
  accY = mpu.getAccelerationY() / 16384.0;
  accZ = mpu.getAccelerationZ() / 16384.0;
  gyroX = mpu.getRotationX() / 131.0 + gyroXError;
  gyroZ = mpu.getRotationZ() / 131.0 - gyroYError;
  gyroY = mpu.getRotationY() / 131.0 + gyroZError;

  // To account for accumalating errors in gyro and accelerometer readings. Gyro reading is taken with 0.96 factor
  double correctedX = correctedAngle(gyroX, accX, accY, accZ, 0);
  double correctedY = correctedAngle(gyroY, accX, accY, accZ, 1);

  // Calculating PID values in X and Y axis
  double XPID = findPID(correctedX, 0);
  double YPID = findPID(correctedY, 1);

  // Setting motor angle based on PID values
  setMotorAngle(elevatorL, XPID);
  setMotorAngle(elevatorR, XPID);
  setMotorAngle(rudder, YPID);

  // Adjust the delay or use a timer for the desired control loop frequency
  delay(FREQUENCY);
}

double correctedAngle(int gyro, int accX, int accY, int accZ, int axis) {
  // Calculating the angle of drone using sensor data from accelerometer
  double accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) - XError; 
  double accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) + YError; //correct the errors

  // Storing in an array to make the code modular
  double accAngle = [accAngleX, accAngleY];

  // Adjusting based on proportion
  double angle = gyro * 0.96 + accAngle[axis];

  return angle;
}

double findPID(double gyroA, int axis) {
  // Calculate error
  double error = setpoint - gyroA;
  
  // Calculate PID terms
  double proportional = kp * error;
  integral += ki * error;
  double derivative = kd * (error - previousError[axis]);

  // Calculate PID output
  double output = proportional + integral + derivative;

  // Update variables for the next iteration
  previousError[axis] = error;

  return output;

}

void setMotorAngle(Servo motor, int angle) {
  // Set servo motor to opposite of the output above
  motor.write(dir * angle);
  // Print cahnge in angle
  Serial.println("Setting Motor Angle: " + String(angle));
}

void calibrate () {

  while (c < 200) {
    mpu.getMotion6();
    accX = mpu.getAccelerationX() / 16384.0 ;
    accY = mpu.getAccelerationY() / 16384.0 ;
    accZ = mpu.getAccelerationZ() / 16384.0 ;
    // Sum all readings
    accXError = accXError + ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));
    accYError = accYError + ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  accXError = accXError / 200;
  accYError = accYError / 200;


  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    mpu.getMotion6();
    gyroY = mpu.getRotationX();
    gyroZ = mpu.getRotationY();
    gyroX = mpu.getRotationZ();
    // Sum all readings
    gyroXError = gyroXError + (gyroX / 131.0);
    gyroYError = gyroYError + (gyroY / 131.0);
    gyroZError = gyroZError + (gyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  gyroXError = gyroXError / 200;
  gyroYError = gyroYError / 200;
  gyroZError = gyroZError / 200;
}