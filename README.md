# Aeromodelling Winter 2023
This is a project showcasing how to PID control a basic self levelled plane using arduino equipped with sensors like MPU6050 and MS5611. 

# Sensors
We have used the libray `Wire.h` as a interfacing library for both sesnosrs, MPU6050 and MS5611. 
## MPU6050
The library `MPU6050.h` was used to access the functionallity of the sensor. 

This sensor come equipped with both a accelerometer and a gyroscope. We can calculate the angle of pitch and roll from both the components. We can directly get the angle from gyroscope. But for acceleromter we used the following formula.

![equation](https://latex.codecogs.com/svg.image?{\color{Cyan}\text{accAngleX}=\tan^{-1}\left(\frac{a_Y}{\sqrt{a_X^2&plus;a_Z^2}}\right)\cdot\frac{180}{\pi}})  
![equation](https://latex.codecogs.com/svg.image?{\color{Cyan}\text{accAngleY}=\tan^{-1}\left(\frac{-a_X}{\sqrt{a_Y^2&plus;a_Z^2}}\right)\cdot\frac{180}{\pi}})

However there are slight errors in the reading. The reading from gyroscope drifts over time away from true value, whike the accelerometer reading is noisy.

We have used used complementary fucntion with an alpha factor to eliminate these. The complementary function used a `ALPHA` value to control in what proportion both the values are added. Alpha can be tuned to minimize the errors in th efinal reading.

We also have a `calibrate()` function which can help find the zero error in the readings and with that it corrects the readings that follow from the sensor.

## MS5611
We have used the library `MS5611.h` to access the MS5611 barometric sensor.
The barometric sensor only outputs the current pressure and not the altitude. We used the [barometric equation](https://en.m.wikipedia.org/wiki/Pressure_altitude) to convert to the correct altitude.

The barometer reading is passed through a [Exponential Moving average](https://www.investopedia.com/terms/e/ema.asp) so as to create a smooth pattern on changes in the readings of barometer. 


# PID
We have a PID function to find the change in sensor readings and correct for them. The Kp, Ki, and Kd can be tuned as to give efficient perfomance. The PID ensures stability in the plane. The input is feeded as combined reading from the accelerometer and gyroscope. 


# Other comments
We have tried to keep the code as modular as possible so it is possible to read and adjust on a later date. We have also tried to correct for common errors in readings and calibration of the sensors.