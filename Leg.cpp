#include "Leg.h"

Leg::Leg(MyServo& ombroServo, MyServo& cotoveloServo)
    : ombro(ombroServo), cotovelo(cotoveloServo) {
    
}

void Leg::moveLegToPositionSmooth( float ombroEnd, float cotoveloEnd, float steps, float delayBetweenSteps) {
  /* for (int i = 0; i <= steps; i++) {
    float currentOmbro = lastOmbro + (i * (ombroEnd - lastOmbro) / steps);
    float currentCotovelo = lastCotovelo + (i * (cotoveloEnd - lastCotovelo) / steps);
    moveLegToPosition(currentOmbro, currentCotovelo);
    lastCotovelo=currentCotovelo;
    lastOmbro=currentOmbro;
    delay(delayBetweenSteps);
  } */
}



void Leg::moveLegToPosition(float ombroPosition, float cotoveloPosition) {
    /* ombro.setAngle(ombroPosition);
    lastOmbro=ombroPosition;

    cotovelo.setAngle(cotoveloPosition);
    lastCotovelo=cotoveloPosition;
    // Add any necessary delays or additional functionality */
}
