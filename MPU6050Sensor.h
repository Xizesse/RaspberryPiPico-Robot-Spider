#ifndef MPU6050SENSOR_H
#define MPU6050SENSOR_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class MPU6050Sensor {
public:
    MPU6050Sensor(uint8_t sda, uint8_t scl);

    bool begin();
    void update(unsigned long now);
    float getAngleX() ;
    float getAngleY() ;
    float getAngleZ() ;
    void resetZ();
    

private:
    Adafruit_MPU6050 mpu;
    const uint8_t sdaPin;
    const uint8_t sclPin;
    sensors_event_t a, g, temp;
    float X, Y, Z;
    float angleX, angleY, angleZ;
    float gyroZero, gyroOut, gyroRate, gyroAngle;
    unsigned long now, previous, duration;
    float zbias = 0.01 - 0.001 + 0.0005;
    float dtime;
    int count;
};

#endif 