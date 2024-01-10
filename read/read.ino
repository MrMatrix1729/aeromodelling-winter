#include "Servo.h"
#include <MPU6050.h>
#include <MS5611.h>
#include <Wire.h>

MPU6050 mpu;
MS5611 ms5611;
Servo elevatorL;
Servo elevatorR;
Servo aierlonL;
Servo aielonR;
Servo rudder;

#define elevatorPinL 9
#define elevatorPinR 10
#define rudderPin 11
#define aierlonPinL 11
#define aierlonPinR 11

#define FREQUENCY 100

// sensor defaults
#define A_SENSITIVITY 16384
#define G_SENSITIVITY 131

// PID parameters
double setpoint = 0.0; // Desired rotation (in degrees)
double kp = 5.0;       // Proportional gain
double ki = 0.1;       // Integral gain
double kd = 0.01;      // Derivative gain
double integral = 0.0; // Changing integral gain

// Modular control
int dir = 1;
float smoothingFactor = 0.05;

// Stores error in prior run
double previousError = [ 0, 0, 0 ];
double altitude = 0.0;

// Sensor Errors
double accXError = 0.0;
double accYError = 0.0;
double gyroXError = 0.0;
double gyroYError = 0.0;
double gyroZError = 0.0;
double seaPressure = 0.0;

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

  // Setting up the barometer MS5611
  while (!ms5611.begin()) {
    // Throws error if not detected
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    delay(500);
  }

  // Calibrating and finding the intial errors in the sensor outputs
  calibrate();

  // Setting initial value for smoothAlt based on pressure at time of launch
  altitude = 44330 * (1 - pow(seaPressure / 101325, 0.1903));
}

void loop() {
  // Read gyroscope data
  mpu.getMotion6();

  // Store gyroscope data in format we need with adjustments for error, 16384
  // and 131 are taken from datasheet of MPU6050 to convert raw values
  accX = mpu.getAccelerationX() / A_SENSITIVITY;
  accY = mpu.getAccelerationY() / A_SENSITIVITY;
  accZ = mpu.getAccelerationZ() / A_SENSITIVITY;
  gyroX = mpu.getRotationX() / G_SENSITIVITY + gyroXError;
  gyroZ = mpu.getRotationZ() / G_SENSITIVITY - gyroYError;
  gyroY = mpu.getRotationY() / G_SENSITIVITY + gyroZError;

  // To account for accumalating errors in gyro and accelerometer readings. Gyro
  // reading is taken with 0.96 factor
  double correctedX = correctedAngle(gyroX, accX, accY, accZ, 0);
  double correctedY = correctedAngle(gyroY, accX, accY, accZ, 1);

  // Calculate new altitude and calls a smoothning function with previous
  // altitude and
  double newAlt = 44330 * (1 - pow(ms5611.getPressure() / 101325, 0.1903));
  altitude = ema(altitude, newAlt);

  // Calculating PID values in X and Y axis
  double XPID = findPID(correctedX, 0);
  double YPID = findPID(correctedY, 1);

  // Setting motor angle based on PID values
  // Controls Roll
  setMotorAngle(aierlonL, XPID);
  setMotorAngle(aierlonR, -XPID);
  // Controls Pitch
  setMotorAngle(elevatorL, YPID);
  setMotorAngle(elevatorR, YPID);

  // Adjust the delay or use a timer for the desired control loop frequency
  delay(FREQUENCY);
}

double correctedAngle(int gyro, int accX, int accY, int accZ, int axis) {
  // Calculating the angle of drone using sensor data from accelerometer
  double accAngleX =
      (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) - accXError;
  double accAngleY =
      (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) +
      accYError; // correct the errors

  // Storing in an array to make the code modular
  double accAngle = [ accAngleX, accAngleY ];

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

double ema(double oldValue, double newValue) {
  // have a smoothing factor to define the ratio to take
  double Value =
      (1.0 - smoothingFactor) * oldValue + smoothingFactor * newValue;
  return Value;
}

void calibrate() {

  while (c < 200) {
    mpu.getMotion6();
    accX = mpu.getAccelerationX() / A_SENSITIVITY;
    accY = mpu.getAccelerationY() / A_SENSITIVITY;
    accZ = mpu.getAccelerationZ() / A_SENSITIVITY;
    // Sum all readings
    accXError =
        accXError +
        ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));
    accYError = accYError +
                ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) *
                  180 / PI));
    c++;
  }
  // Divide the sum by 200 to get the error value
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
    gyroXError = gyroXError + (gyroX / G_SENSITIVITY);
    gyroYError = gyroYError + (gyroY / G_SENSITIVITY);
    gyroZError = gyroZError + (gyroZ / G_SENSITIVITY);
    c++;
  }
  // Divide the sum by 200 to get the error value
  gyroXError = gyroXError / 200;
  gyroYError = gyroYError / 200;
  gyroZError = gyroZError / 200;

  c = 0;
  // Read MS5611 200 times
  while (c < 200) {
    seaPressure += ms5611.getPressure();
  }
  seaPressure = seaPressure / 200;
}