#include "MyServo.h"


#ifndef LEG_H
#define LEG_H

class Leg {
public:
    Leg(MyServo& ombroServo, MyServo& cotoveloServo); // Constructor that takes existing MyServo objects
    void moveLegToPosition(float ombroPosition, float cotoveloPosition); // Move leg to a specific position
    void testLeg(); // Test function to move the leg through a range of positions
    void moveLegToPositionSmooth( float ombroEnd, float cotoveloEnd, float steps, float delayBetweenSteps);
    MyServo& ombro; // Reference to the "shoulder" servo object
    MyServo& cotovelo; 
private:
   // Reference to the "elbow" servo object
    float lastOmbro;
    float lastCotovelo;
};

#endif 
