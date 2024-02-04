
#include "MyServo.h"
#include <Arduino.h>



MyServo::MyServo() : max(2500), min(500), pos110(1400), pos90(1400), pos45(1500), pos0(1600) {}

void MyServo::attach(int pin) {
    this->pin = pin;
    ServoPWM = new RP2040_PWM(pin, 50 ,5);
    //servo.attach(pin, min, max);
    Serial.println("Servo attached on pin " + String(pin));
}

void MyServo::calibrate(float maxVal, float minVal, float pos0Val, float pos45Val, float pos90Val, float pos110Val) {
    max = maxVal;
    min = minVal;
    pos110 = pos110Val;
    pos90 = pos90Val;
    pos45 = pos45Val;
    pos0 = pos0Val;

    angulo = 45;
}


int MyServo::getPin() {
    return pin;
}


void MyServo::setAngle(float angle) {
    angulo = angle;
    //updateAngle();
}

void MyServo::updateAngle() {
    

    
    int microseconds;
    
    if (angulo < 0) {
        angulo = 0;
    }
    else if (angulo > 110) {
        angulo = 110;
    }
    
    if (angulo <= 45) {
        microseconds = map(angulo, 0, 45, pos0, pos45);
    } else if (angulo <= 90) {
        microseconds = map(angulo, 45, 90, pos45, pos90);
    }
    else {
        microseconds = map(angulo, 90, 110, pos90, pos110);
    }

    float minPulse = std::min(pos0, pos110);
    float maxPulse = std::max(pos0, pos110);

    microseconds = constrain(microseconds, minPulse, maxPulse);

    //servo.writeMicroseconds(microseconds);
   // Serial.println("Servo " + String(pin) + " set to " + String(angulo) + " degrees");
    ServoPWM->setPWM((this->pin), 50, (float)microseconds*50/10000);
}




