#ifndef MYSERVO_H
#define MYSERVO_H

#include <Arduino.h>
#include <Servo.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "RP2040_PWM.h"

class MyServo {
public:
    MyServo();
    void attach(int pin);
    void calibrate(float max, float min, float pos0, float pos45, float pos90, float pos110);
    int getPin();
    void updateAngle();
    void setAngle(float angle);
    
      
   float angulo;

private:
    float max, min;
    float pos110, pos90, pos45, pos0;
    float pin;
    
    RP2040_PWM * ServoPWM;
};

#endif 
