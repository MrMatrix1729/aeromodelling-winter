#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
    Wire.begin();
    Serial.begin(9600);

    mpu.initialize();

    // Optional: Set the gyroscope and accelerometer scale using setFullScaleGyroRange and setFullScaleAccelRange
    // mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
}

void loop() {
    // Read both accelerometer and gyroscope data in a single call
    mpu.getMotion6();

    // Retrieve data
    int16_t accelX = mpu.getAccelerationX();
    int16_t accelY = mpu.getAccelerationY();
    int16_t accelZ = mpu.getAccelerationZ();
    int16_t gyroX = mpu.getRotationX();
    int16_t gyroY = mpu.getRotationY();
    int16_t gyroZ = mpu.getRotationZ();

    // Print data to Serial Monitor
    Serial.print("AccelX: "); Serial.print(accelX);
    Serial.print(" | AccelY: "); Serial.print(accelY);
    Serial.print(" | AccelZ: "); Serial.print(accelZ);
    Serial.print(" | GyroX: "); Serial.print(gyroX);
    Serial.print(" | GyroY: "); Serial.print(gyroY);
    Serial.print(" | GyroZ: "); Serial.println(gyroZ);

    delay(1000); // Adjust the delay as needed
}