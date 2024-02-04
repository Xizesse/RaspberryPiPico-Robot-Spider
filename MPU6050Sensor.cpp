#include "MPU6050Sensor.h"
#include <Arduino.h>
#include <Wire.h>

MPU6050Sensor::MPU6050Sensor(uint8_t sda, uint8_t scl) : sdaPin(sda), sclPin(scl), angleX(0), angleY(0), angleZ(0), gyroZero(0) {}

bool MPU6050Sensor::begin() {
    Wire1.setSDA(sdaPin);
    Wire1.setSCL(sclPin);
    Wire1.begin();

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);


    previous = millis();

    angleX = 0.0;
    angleY = 0.0;
    angleZ = 0.0;
    gyroZero = 0.0;

    return mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire1);

}

void MPU6050Sensor::update(unsigned long now){

    duration = now - previous;
    dtime = duration / 1000.0;
    previous = now;
    mpu.getEvent(&a, &g, &temp);
    X = atan(a.acceleration.x/sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * RAD_TO_DEG;
    Y = atan(a.acceleration.y/sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * RAD_TO_DEG;
    
    angleX = 0.9 * (angleX + g.gyro.x * dtime * RAD_TO_DEG) + 0.1 * X;
    angleY = 0.9 * (angleY + g.gyro.y * dtime * RAD_TO_DEG) + 0.1 * Y;
    angleZ += (g.gyro.z-zbias) * dtime * RAD_TO_DEG; // Integrate gyroscope data over time to calculate yaw
}

float MPU6050Sensor::getAngleX()  { return angleX ; }
float MPU6050Sensor::getAngleY()  { return angleY ; }
float MPU6050Sensor::getAngleZ()  { return angleZ ; }

void MPU6050Sensor::resetZ() { angleZ = 0; }