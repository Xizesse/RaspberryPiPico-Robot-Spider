#include <Arduino.h>
#include <Servo.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "Leg.h"
#include "RP2040_PWM.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include "MPU6050Sensor.h" 

#define BASE 0
#define FRENTE 1
#define RODAPOSITIVO 2
#define RODANEGATIVO 3
#define ESQUERDAF 4
#define ESQUERDAL 7
#define ESQUERDAT 8
#define DIREITA 5
#define SWIM 6
#define PUSHUP 9
#define RODAPOSITIVOPOUCO 10
#define RODANEGATIVOPOUCO 11
#define RODANEGATIVOMICRO 13
#define RODAPOSITIVOMICRO 14
#define RETO 12
#define ESQUERDA 15
#define SUBIR 16


#define OFFSETESQUERDA 1500
#define COLLISION 150



#define TOF 
#define IMU

//!Sensores

#ifdef TOF
VL53L0X tof ;
float distance;
#endif

#ifdef IMU
MPU6050Sensor mpu(18, 19);
float angleX;
float angleY;
float angleZ;

float desiredZ, errorZ;
float SPINCORRECTION = 15.0;
float SPINCORRECTIONMICRO = 10.0;
#endif

//!Servos
MyServo myservo[8];
Leg FrontLeft(myservo[1], myservo[0]);
Leg FrontRight(myservo[4], myservo[5]);
Leg BackLeft(myservo[3], myservo[2]);
Leg BackRight(myservo[7], myservo[6]);

uint32_t curr_time = 0;
int flag = 0;
int action;
int rodarflag = 0;
int speed = 0;
uint32_t t;
uint32_t t_secs;
uint32_t loop_interval = 0;
uint32_t last_loop_time = 0;
uint32_t last_collision = 0;
uint32_t last_print_time = 0;
uint32_t print_interval = 10;
uint32_t lateral_time =0;
uint32_t a = 0;
bool spiderMutex = false;
int prevstate = 0;

typedef struct {
  int state, new_state;
  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// Our finite state machines
fsm_t fsm_walkForwardFAST, 
      fsm_rodaNegativo, 
      fsm_rodaPositivo,
      fsm_control,
      fsm_swim,
      fsm_pushup,
      fsm_right,
      fsm_left,
      fsm_rodaNegativoPouco,
      fsm_rodaPositivoPouco,
      fsm_subir,
      fsm_rodaNegativoMicro, 
      fsm_rodaPositivoMicro;
    


//! State Machine Functions
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state; 

    fsm.tes = millis();
    fsm.tis = 0;
  }
}

void update_tis()
{
  fsm_walkForwardFAST.tis = millis() - fsm_walkForwardFAST.tes;
  fsm_rodaNegativo.tis = millis() - fsm_rodaNegativo.tes;
  fsm_rodaPositivo.tis = millis() - fsm_rodaPositivo.tes;
  fsm_control.tis = millis() - fsm_control.tes;
  fsm_swim.tis = millis() - fsm_swim.tes;
  fsm_pushup.tis = millis() - fsm_pushup.tes;
  fsm_right.tis = millis() - fsm_right.tes;
  fsm_left.tis = millis() - fsm_left.tes;
  fsm_rodaNegativoPouco.tis = millis() - fsm_rodaNegativoPouco.tes;
  fsm_rodaPositivoPouco.tis = millis() - fsm_rodaPositivoPouco.tes;
  fsm_subir.tis= millis() - fsm_subir.tes;
  fsm_rodaNegativoMicro.tis = millis() - fsm_rodaNegativoMicro.tes;
  fsm_rodaPositivoMicro.tis = millis() - fsm_rodaPositivoMicro.tes;



}

void update_fsms()
{
  set_state(fsm_walkForwardFAST, fsm_walkForwardFAST.new_state);
  set_state(fsm_rodaNegativo, fsm_rodaNegativo.new_state);
  set_state(fsm_rodaPositivo, fsm_rodaPositivo.new_state);
  set_state(fsm_swim, fsm_swim.new_state);
  set_state(fsm_pushup, fsm_pushup.new_state);
  set_state(fsm_right, fsm_right.new_state);
  set_state(fsm_left, fsm_left.new_state);
  set_state(fsm_control, fsm_control.new_state);
  set_state(fsm_rodaNegativoPouco,fsm_rodaNegativoPouco.new_state);
  set_state(fsm_rodaPositivoPouco,fsm_rodaPositivoPouco.new_state);
  set_state(fsm_subir,fsm_subir.new_state);
  set_state(fsm_rodaNegativoMicro,fsm_rodaNegativoMicro.new_state);
  set_state(fsm_rodaPositivoMicro,fsm_rodaPositivoMicro.new_state);
 

}

float calculateParabolicAngle(float t, float start_angle, float peak_angle, float end_angle, float total_time) {

     // Vertex (h, etk) of thae parabotla
    float h = total_time / 2;
    float k = peak_angle;

    float a = (start_angle - k) / (h * h);
    float angle = a * (t - h) * (t - h) + k;

    return angle;
}

void calibrate()
{
   
   myservo[0].calibrate(2500, 500,2400, 1900, 1400, 1400); //cotovelo
   
   myservo[1].calibrate(2500, 500, 700, 1250, 1750, 1950); //ombro
   
   myservo[2].calibrate(2500, 500, 625, 1100, 1500, 1500); //cotovelo
   
   myservo[3].calibrate(2500, 500, 2400, 1900, 1400, 1150); //ombro
  
   myservo[4].calibrate(2500, 500, 2500, 2050, 1600, 1350); //
  
   myservo[5].calibrate(2500, 500, 550, 1000, 1550, 1550); //
 
   myservo[6].calibrate(2500, 500, 2150, 1700, 1100, 1100); //cotovelo

   myservo[7].calibrate(2500, 500, 550, 1000, 1500, 1750); //ombro
   
}

void setup() 
{

  Serial.begin(115200);
  for (int i = 0; i < 8; i++) {
    myservo[i].attach(i+3);
  }
  
  //!Sensor de Distancia
  #ifdef TOF
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();
  tof.setTimeout(500);
  while (!tof.init())
   {
      Serial.println("Failed to detect and initialize sensor!");
   }

   Serial.println("TOF Sensor Found!\n");
   tof.startContinuous();
    #endif

  //! IMU
  #ifdef IMU
  mpu.begin();
  #endif
  set_state(fsm_walkForwardFAST, 0);
  fsm_walkForwardFAST.new_state = 0;
  set_state(fsm_rodaNegativo, 0);
  fsm_rodaNegativo.new_state = 0;
  set_state(fsm_rodaPositivo, 0);
  fsm_rodaPositivo.new_state = 0;
  set_state(fsm_swim, 0);
  fsm_swim.new_state = 0;
  set_state(fsm_control, 0);
  fsm_control.new_state = 0;
  set_state(fsm_pushup, 0);
  fsm_pushup.new_state = 0;
  set_state(fsm_right, 0);  
  fsm_right.new_state = 0;
  set_state(fsm_left, 0);
  fsm_left.new_state = 0;
  set_state(fsm_subir, 0);
  fsm_subir.new_state = 0;
  set_state(fsm_rodaNegativoPouco, 0);
  fsm_rodaNegativoPouco.new_state = 0;
  set_state(fsm_rodaPositivoPouco, 0);
  fsm_rodaPositivoPouco.new_state = 0;
  set_state(fsm_rodaNegativoMicro, 0);
  fsm_rodaNegativoMicro.new_state = 0;
  set_state(fsm_rodaPositivoMicro, 0);
  fsm_rodaPositivoMicro.new_state = 0;
  


  calibrate();

}

//! Position Setters
void estica()
{
  FrontLeft.cotovelo.setAngle(0);
  FrontRight.cotovelo.setAngle(0);
  BackLeft.cotovelo.setAngle(0);
  BackRight.cotovelo.setAngle(0);
  FrontLeft.ombro.setAngle(0);
  FrontRight.ombro.setAngle(0);
  BackLeft.ombro.setAngle(0);
  BackRight.ombro.setAngle(0);
}

void base()
{
  FrontLeft.cotovelo.setAngle(45);
  FrontRight.cotovelo.setAngle(45);
  BackLeft.cotovelo.setAngle(45);
  BackRight.cotovelo.setAngle(45);
  FrontLeft.ombro.setAngle(45);
  FrontRight.ombro.setAngle(45);
  BackLeft.ombro.setAngle(45);
  BackRight.ombro.setAngle(45);
}

void reto()
{
  FrontLeft.cotovelo.setAngle(90);
  FrontRight.cotovelo.setAngle(90);
  BackLeft.cotovelo.setAngle(90);
  BackRight.cotovelo.setAngle(90);
  FrontLeft.ombro.setAngle(90);
  FrontRight.ombro.setAngle(90);
  BackLeft.ombro.setAngle(90);
  BackRight.ombro.setAngle(90);

}

void cotovelosRetos(){
  FrontLeft.ombro.setAngle(45);
  FrontRight.ombro.setAngle(45);
  BackLeft.ombro.setAngle(45);
  BackRight.ombro.setAngle(45);
  
  FrontLeft.cotovelo.setAngle(90);
  FrontRight.cotovelo.setAngle(90);
  BackLeft.cotovelo.setAngle(90);
  BackRight.cotovelo.setAngle(90);
 }

void juntos()
{
  FrontLeft.cotovelo.setAngle(110);
  FrontRight.cotovelo.setAngle(110);
  BackLeft.cotovelo.setAngle(110);
  BackRight.cotovelo.setAngle(110);
  FrontLeft.ombro.setAngle(110);
  FrontRight.ombro.setAngle(110);
  BackLeft.ombro.setAngle(110);
  BackRight.ombro.setAngle(110);
}

//! Actions

void subir(int speed){
  float offset = 10;
  
  t_secs = fsm_subir.tis/1000;
  t = fsm_subir.tis;
  
  switch (fsm_subir.state)
  {
    case 0:
        // não faz nada 
        if( action == SUBIR && spiderMutex == false)
        {
          fsm_subir.new_state = 1;
          spiderMutex = true;
        }
    break;

    case 1:
  
      BackRight.cotovelo.angulo = calculateParabolicAngle(t, 45, 30, 60, speed);

      FrontLeft.ombro.angulo = map(t, 0, speed, 45, 60);
      BackRight.ombro.angulo = map(t, 0, speed, 45, 80);
      // FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
      FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 45, 30, 60, speed);
      

       if(t > speed)
      {
        fsm_subir.new_state = 2;
      }

      break;

      case 2:
      FrontRight.ombro.angulo = map(t, 0, speed, 45, 110);
      FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 45,30, 60, speed);
      BackLeft.ombro.angulo = map(t, 0, speed, 45, 60);
      BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 45, 30, 60, speed);

    
       if(t > speed)
      {
        fsm_subir.new_state = 3;
      }
      break;
    
    case 3:
      FrontRight.ombro.angulo = map(t, 0, speed, 110, 30-offset);
      FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);

      FrontLeft.ombro.angulo = map(t, 0, speed, 30+offset, 110);
      BackRight.ombro.angulo = map(t, 0, speed, 110, 30-offset);
      // FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
      BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 90, 0, 90, speed);
      BackLeft.ombro.angulo = map(t, 0, speed, 30+offset, 110);

      if(t > speed)
      {
        fsm_subir.new_state = 4;
        break;
      }

      break;

    case 4:
      FrontLeft.ombro.angulo = map(t, 0, speed, 110, 30+offset);
      FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
      FrontRight.ombro.angulo = map(t, 0, speed, 30-offset, 110);
      BackLeft.ombro.angulo = map(t, 0, speed, 110, 30+offset);
      //FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
      BackRight.cotovelo.angulo = calculateParabolicAngle(t, 90, 0, 90, speed);
      BackRight.ombro.angulo = map(t, 0, speed, 30, 110);
      

      if(fsm_subir.tis > speed)
      {
        if( action == SUBIR)
        {
          fsm_subir.new_state = 3;

        }
        else
        {
            fsm_subir.new_state = 0;
            spiderMutex = false; //give up mutex
        }
      }
      break;

  }


}

void walkForwardFAST(int speed)
{
  float offset = 0;
  
  t_secs = fsm_walkForwardFAST.tis/1000;
  t = fsm_walkForwardFAST.tis;
  
  switch (fsm_walkForwardFAST.state)
  {
    case 0:
        // não faz nada 
        if( action == FRENTE && spiderMutex == false)
        {
          fsm_walkForwardFAST.new_state = 1;
          spiderMutex = true;
        }
    break;

    case 1:
  
      BackRight.cotovelo.angulo = calculateParabolicAngle(t, 45, 30, 60, speed);

      FrontLeft.ombro.angulo = map(t, 0, speed, 45, 60);
      BackRight.ombro.angulo = map(t, 0, speed, 45, 80);
      // FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
      FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 45, 30, 60, speed);
      

       if(t > speed)
      {
        fsm_walkForwardFAST.new_state = 2;
      }

      break;

      case 2:
      FrontRight.ombro.angulo = map(t, 0, speed, 45, 110);
      FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 45,30, 60, speed);
    BackLeft.ombro.angulo = map(t, 0, speed, 45, 60);
     BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 45, 30, 60, speed);

    
       if(t > speed)
      {
        fsm_walkForwardFAST.new_state = 3;
      }
      break;
    
    case 3:
      FrontRight.ombro.angulo = map(t, 0, speed, 110, 60-offset);
      FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);

      FrontLeft.ombro.angulo = map(t, 0, speed, 60+offset, 110);
      BackRight.ombro.angulo = map(t, 0, speed, 110, 60-offset);
      // FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
      BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      BackLeft.ombro.angulo = map(t, 0, speed, 60+offset, 110);

      if(t > speed)
      {
        fsm_walkForwardFAST.new_state = 4;
      }

      break;

    case 4:
      FrontLeft.ombro.angulo = map(t, 0, speed, 110, 60+offset);
      FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      FrontRight.ombro.angulo = map(t, 0, speed, 60-offset, 110);
      BackLeft.ombro.angulo = map(t, 0, speed, 110, 60+offset);
      //FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
      BackRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      BackRight.ombro.angulo = map(t, 0, speed, 60, 110);
      

      if(fsm_walkForwardFAST.tis > speed)
      {
        if( action == FRENTE)
        {
          fsm_walkForwardFAST.new_state = 3;

        }
        else
        {
            fsm_walkForwardFAST.new_state = 0;
            spiderMutex = false; //give up mutex
        }
      }
      break;

  }
}

void rodaPositivo(int speed) {
  float offset = 15;
  t = fsm_rodaPositivo.tis;
  t_secs = t/1000;

  switch(fsm_rodaPositivo.state) {

    case 0: 
    if (action ==RODAPOSITIVO && spiderMutex == false)
    {
      fsm_rodaPositivo.new_state = 1;
      spiderMutex = true;
    }
    break;

    case 1:

     if(fsm_rodaPositivo.tis > speed)
    {
      fsm_rodaPositivo.new_state = 2;
    }

    FrontRight.ombro.angulo = map(t, 0, speed, 0+offset, 90-offset);
    FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
    BackLeft.ombro.angulo = map(t, 0, speed, 0+offset, 90-offset);
    BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
    BackRight.ombro.angulo = map(t, 0, speed, 0+offset,90-offset);
    FrontLeft.ombro.angulo = map(t, 0, speed, 0+offset,90-offset);

   
    break;

    case 2:

    


    FrontLeft.ombro.angulo = map(t, 0, speed, 90-offset, 0+offset);
    FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
    BackRight.ombro.angulo = map(t, 0, speed, 90-offset, 0+offset);
    BackRight.cotovelo.angulo = calculateParabolicAngle(t,90, 45, 90, speed);

    BackLeft.ombro.angulo = map(t, 0, speed, 90-offset, 0+offset);
    FrontRight.ombro.angulo = map(t, 0, speed, 90-offset, 0+offset);

    if(fsm_rodaPositivo.tis > speed)
    {
      if (action == RODAPOSITIVO)
      {
        fsm_rodaPositivo.new_state = 1;
      }
      else
      {
        fsm_rodaPositivo.new_state = 0;
        spiderMutex = false;
      }
    }
    break;
  }
 }

void rodaNegativo(int speed){
    float offset = -10;
    t = fsm_rodaNegativo.tis;
    t_secs = t/1000;

  switch (fsm_rodaNegativo.state){
      case 0:
        if (action == RODANEGATIVO && spiderMutex == false)
        {
          fsm_rodaNegativo.new_state = 1;
          spiderMutex = true;
        }
        break;

      case 1:
        FrontLeft.ombro.angulo = map(t, 0, speed, offset, 90 - offset);
        FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
        BackRight.ombro.angulo = map(t, 0, speed, offset, 90 - offset);
        BackRight.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
        BackLeft.ombro.angulo = map(t, 0, speed, offset, 90 - offset);
        FrontRight.ombro.angulo = map(t, 0, speed, offset, 90 - offset);

        if(fsm_rodaNegativo.tis > speed)
        {
          fsm_rodaNegativo.new_state = 2;
        }
        break;

      case 2:
        FrontRight.ombro.angulo = map(t, 0, speed, 90 - offset, offset);
        FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
        BackLeft.ombro.angulo = map(t, 0, speed, 90 - offset, offset);
        BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
        BackRight.ombro.angulo = map(t, 0, speed, 90 - offset, offset);
        FrontLeft.ombro.angulo = map(t, 0, speed, 90 - offset, offset);

        if(fsm_rodaNegativo.tis > speed)
        {
          if (action == RODANEGATIVO)
          {
            fsm_rodaNegativo.new_state = 1;
          }
          else
          {
            fsm_rodaNegativo.new_state = 0;
            spiderMutex = false;
          }
          
        }
        break;
    }
  }

void rodaPositivoPouco(int speed) {
  float offset = 0;
  t = fsm_rodaPositivoPouco.tis;
  t_secs = t/1000;

  switch(fsm_rodaPositivoPouco.state) {

    case 0: 
    if (action == RODAPOSITIVOPOUCO && spiderMutex == false)
    {
      fsm_rodaPositivoPouco.new_state = 1;
      spiderMutex = true;
    }
    break;

    case 1:

     if(fsm_rodaPositivoPouco.tis > speed)
    {
      fsm_rodaPositivoPouco.new_state = 2;
    }

    FrontRight.ombro.angulo = map(t, 0, speed, 30+offset, 60-offset);
    FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    BackLeft.ombro.angulo = map(t, 0, speed, 30+offset, 60-offset);
    BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    BackRight.ombro.angulo = map(t, 0, speed, 30+offset,60-offset);
    FrontLeft.ombro.angulo = map(t, 0, speed, 30+offset,60-offset);

    break;

    case 2:

    FrontLeft.ombro.angulo = map(t, 0, speed, 60-offset, 30+offset);
    FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    BackRight.ombro.angulo = map(t, 0, speed, 60-offset, 30+offset);
    BackRight.cotovelo.angulo = calculateParabolicAngle(t,60, 45, 60, speed);

    BackLeft.ombro.angulo = map(t, 0, speed, 60-offset, 30+offset);
    FrontRight.ombro.angulo = map(t, 0, speed, 60-offset, 30+offset);

    if(fsm_rodaPositivoPouco.tis > speed)
    {
      if (action == RODAPOSITIVOPOUCO)
      {
        fsm_rodaPositivoPouco.new_state = 1;
      }
      else
      {
        fsm_rodaPositivoPouco.new_state = 0;
        spiderMutex = false;
      }
    }

    
    break;
  }
 }

void rodaNegativoPouco(int speed){
    float offset = 0;
    t = fsm_rodaNegativoPouco.tis;
    t_secs = t/1000;

  switch (fsm_rodaNegativoPouco.state){
      case 0:
        if (action == RODANEGATIVOPOUCO && spiderMutex == false)
        {
          fsm_rodaNegativoPouco.new_state = 1;
          spiderMutex = true;
        }
        break;

      case 1:
        FrontLeft.ombro.angulo = map(t, 0, speed, 30+offset, 60 - offset);
        FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
        BackRight.ombro.angulo = map(t, 0, speed, 30+offset, 60 - offset);
        BackRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
        BackLeft.ombro.angulo = map(t, 0, speed, 30+offset, 60 - offset);
        FrontRight.ombro.angulo = map(t, 0, speed, 30+offset, 60 - offset);

        if(fsm_rodaNegativoPouco.tis > speed)
        {
          fsm_rodaNegativoPouco.new_state = 2;
        }
        break;

      case 2: 
      
        FrontRight.ombro.angulo = map(t, 0, speed, 60- offset, 30+offset);
        FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
        BackLeft.ombro.angulo = map(t, 0, speed, 60 - offset, 30+offset);
        BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
        BackRight.ombro.angulo = map(t, 0, speed, 60 - offset, 30 + offset);
        FrontLeft.ombro.angulo = map(t, 0, speed, 60 - offset, 30 + offset);


          if(fsm_rodaNegativoPouco.tis > speed){
        
          if (action == RODANEGATIVOPOUCO)
          {
            fsm_rodaNegativoPouco.new_state = 1;
          }
          else
          {
            fsm_rodaNegativoPouco.new_state = 0;
            spiderMutex = false;
          }
          
        }
     
        break;
    }
  }

void rodaPositivoMicro(int speed) {
  float offset = 0;
  t = fsm_rodaPositivoMicro.tis;
  t_secs = t/1000;

  switch(fsm_rodaPositivoMicro.state) {

    case 0: 
    if (action == RODAPOSITIVOMICRO && spiderMutex == false)
    {
      fsm_rodaPositivoMicro.new_state = 1;
      spiderMutex = true;
    }
    break;

    case 1:

     if(fsm_rodaPositivoMicro.tis > speed)
    {
      fsm_rodaPositivoMicro.new_state = 2;
    }

    FrontRight.ombro.angulo = map(t, 0, speed, 40+offset, 50-offset);
    FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
    BackLeft.ombro.angulo = map(t, 0, speed, 40+offset, 50-offset);
    BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
    BackRight.ombro.angulo = map(t, 0, speed, 40+offset,50-offset);
    FrontLeft.ombro.angulo = map(t, 0, speed, 40+offset,50-offset);

    break;

    case 2:

    FrontLeft.ombro.angulo = map(t, 0, speed, 50-offset, 40+offset);
    FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
    BackRight.ombro.angulo = map(t, 0, speed, 50-offset, 40+offset);
    BackRight.cotovelo.angulo = calculateParabolicAngle(t,90, 45, 90, speed);

    BackLeft.ombro.angulo = map(t, 0, speed, 50-offset, 40+offset);
    FrontRight.ombro.angulo = map(t, 0, speed, 50-offset, 40+offset);

    if(fsm_rodaPositivoMicro.tis > speed)
    {
      if (action == RODAPOSITIVOMICRO)
      {
        fsm_rodaPositivoMicro.new_state = 1;
      }
      else
      {
        fsm_rodaPositivoMicro.new_state = 0;
        spiderMutex = false;
      }
    }

    
    break;
  }
 }

void rodaNegativoMicro(int speed){
    float offset = 0;
    t = fsm_rodaNegativoMicro.tis;
    t_secs = t/1000;

  switch (fsm_rodaNegativoMicro.state){
      case 0:
        if (action == RODANEGATIVOMICRO && spiderMutex == false)
        {
          fsm_rodaNegativoMicro.new_state = 1;
          spiderMutex = true;
        }
        break;

      case 1:
        FrontLeft.ombro.angulo = map(t, 0, speed, 40+offset, 50 - offset);
        FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
        BackRight.ombro.angulo = map(t, 0, speed, 40+offset, 50 - offset);
        BackRight.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
        BackLeft.ombro.angulo = map(t, 0, speed, 40+offset, 50 - offset);
        FrontRight.ombro.angulo = map(t, 0, speed, 40+offset, 50 - offset);

        if(fsm_rodaNegativoMicro.tis > speed)
        {
          fsm_rodaNegativoMicro.new_state = 2;
        }
        break;

      case 2: 
      
        FrontRight.ombro.angulo = map(t, 0, speed, 50- offset, 40+offset);
        FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
        BackLeft.ombro.angulo = map(t, 0, speed, 50 - offset, 40+offset);
        BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 90, 45, 90, speed);
        BackRight.ombro.angulo = map(t, 0, speed, 50 - offset, 40 + offset);
        FrontLeft.ombro.angulo = map(t, 0, speed, 50 - offset, 40 + offset);


          if(fsm_rodaNegativoMicro.tis > speed){
        
          if (action == RODANEGATIVOMICRO)
          {
            fsm_rodaNegativoMicro.new_state = 1;
          }
          else
          {
            fsm_rodaNegativoMicro.new_state = 0;
            spiderMutex = false;
          }
          
        }
     
        break;
    }
  }

void swim(float speed)
{
  t = fsm_swim.tis;
  
  switch (fsm_swim.state)
  {
    case 0:
        // não faz nada 
        if(/* fsm_objectAvoidance.state == SWIM &&*/ spiderMutex == false)
        {
          fsm_swim.new_state = 1;
          spiderMutex = true;
        }
    break;
    
    case 1:

      BackLeft.cotovelo.angulo =  map(t, 0, speed, 0, 45);
      BackRight.cotovelo.angulo =  map(t, 0, speed, 45, 0);
      BackLeft.ombro.angulo =  0;
      BackRight.ombro.angulo =  0;

      FrontRight.ombro.angulo = map(t, 0, speed, 0, 45);
      FrontLeft.ombro.angulo = map(t, 0, speed, 0, 45);
      FrontRight.cotovelo.angulo = 0;
      FrontLeft.cotovelo.angulo = 0;


      if(t > speed)
      {
        fsm_swim.new_state = 2;
      }

      break;

    case 2:

      BackLeft.cotovelo.angulo =  map(t, 0, speed, 45, 0);
      BackRight.cotovelo.angulo =  map(t, 0, speed, 0, 45);

      FrontRight.ombro.angulo = map(t, 0, speed, 45, 90);
      FrontLeft.ombro.angulo = map(t, 0, speed, 45, 90);

      if(t > speed)
      {
         fsm_swim.new_state = 3;
      }

      break;

    case 3:
      BackLeft.cotovelo.angulo =  map(t, 0, speed, 0, 45);
      BackRight.cotovelo.angulo =  map(t, 0, speed, 45, 0);

      FrontLeft.cotovelo.angulo =  map(t, 0, speed, 0, 45);
      FrontRight.cotovelo.angulo =  map(t, 0, speed, 0, 45);

      if(t > speed)
      {
         fsm_swim.new_state = 4;
      }

      break;

    case 4:
      BackLeft.cotovelo.angulo =  map(t, 0, speed, 45, 0);
      BackRight.cotovelo.angulo =  map(t, 0, speed, 0, 45);

      FrontLeft.cotovelo.angulo =  map(t, 0, speed, 45, 90);
      FrontRight.cotovelo.angulo =  map(t, 0, speed, 45, 90);

      if(t > speed)
      {
         fsm_swim.new_state = 5;
      }

      break;

    case 5:
      BackLeft.cotovelo.angulo =  map(t, 0, speed, 0, 45);
      BackRight.cotovelo.angulo =  map(t, 0, speed, 45, 0);

      FrontLeft.cotovelo.angulo = map(t, 0, speed, 90, 45);
      FrontRight.cotovelo.angulo = map(t, 0, speed, 90, 45);
      FrontLeft.ombro.angulo = map(t, 0, speed, 90, 45);
      FrontRight.ombro.angulo = map(t, 0, speed, 90, 45);

      if(t > speed)
      {
         fsm_swim.new_state = 6;
      }

      break;


    case 6:
      BackLeft.cotovelo.angulo =  map(t, 0, speed, 45, 0);
      BackRight.cotovelo.angulo =  map(t, 0, speed, 0, 45);

      FrontLeft.cotovelo.angulo = map(t, 0, speed, 45, 0);
      FrontRight.cotovelo.angulo = map(t, 0, speed, 45, 0);
      FrontLeft.ombro.angulo = map(t, 0, speed, 45, 0);
      FrontRight.ombro.angulo = map(t, 0, speed, 45, 0);

      

      if( fsm_swim.tis > speed)
      {
        if(fsm_control.state == FRENTE)
        {
           fsm_swim.new_state = 1;

        }
        else
        {
            fsm_swim.new_state = 0;
            spiderMutex = false; //give up mutex
        }
      }
      break;

  }
}

void pushup(float speed)
{
  t = fsm_pushup.tis;
  
  switch (fsm_pushup.state)
  {
    case 0:
        // não faz nada 
        if(/* fsm_objectAvoidance.state == PUSHUP &&*/ spiderMutex == false)
        {
          fsm_pushup.new_state = 1;
          spiderMutex = true;
        }
    break;
    
    case 1:

      BackLeft.cotovelo.angulo =  45;
      BackRight.cotovelo.angulo =  45;
      BackLeft.ombro.angulo =  0;
      BackRight.ombro.angulo =  0;

      FrontRight.cotovelo.angulo = map(t, 0, speed, 90, 45);
      FrontLeft.cotovelo.angulo = map(t, 0, speed, 90, 45);
      FrontRight.ombro.angulo = 90;
      FrontLeft.ombro.angulo = 90;


      if(t > speed)
      {
        fsm_pushup.new_state = 2;
      }

      break;

    case 2:
      FrontRight.cotovelo.angulo = map(t, 0, speed, 45, 90);
      FrontLeft.cotovelo.angulo = map(t, 0, speed, 45, 90);
      

      if( fsm_pushup.tis > speed)
      {
        if(/*fsm_objectAvoidance.state == FRENTE*/ true)
        {
           fsm_pushup.new_state = 1;

        }
        else
        {
            fsm_pushup.new_state = 0;
            spiderMutex = false; //give up mutex
        }
      }
      break;

  }
}

void left(int speed){

    t = fsm_left.tis;
    t_secs = t/1000;

  switch (fsm_left.state){

    case 0:

    if (action == ESQUERDA  && spiderMutex == false)
    {
      fsm_left.new_state = 1;
      spiderMutex = true;
    }

    break;

    case 1:

     if(fsm_left.tis > speed)
    {
      fsm_left.new_state = 2;
    }

    FrontLeft.ombro.angulo = map(t, 0, speed,0, 45);
    FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);

    BackLeft.ombro.angulo = map(t, 0, speed, 45, 0);
    FrontRight.ombro.angulo = map(t, 0, speed, 0, 45);
    // FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
    BackRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    BackRight.ombro.angulo = map(t, 0, speed, 45, 0);


    break;

    case 2:

    if(fsm_left.tis > speed)
    {
      if (action == ESQUERDA)
      {
        fsm_left.new_state = 1;
      }
      else
      {
        fsm_left.new_state = 0;
        spiderMutex = false;
      }
    }

     

    BackLeft.ombro.angulo = map(t, 0, speed, 0, 45);
    BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    FrontLeft.ombro.angulo = map(t, 0, speed, 45, 0);
    BackRight.ombro.angulo = map(t, 0, speed, 0, 45);
  //FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
    FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    FrontRight.ombro.angulo = map(t, 0, speed, 45, 0);
    break;

    }
  }

void right( int speed) {

     t = fsm_right.tis;
    t_secs = t/1000;
    int offset = 0;
    



  switch(fsm_right.state){

    case 0:

      if(action == DIREITA && spiderMutex == false)
    {
      fsm_right.new_state = 1;
      spiderMutex = true;
    }

    break;


    case 1:
    
     if(fsm_right.tis > speed)
    {
      fsm_right.new_state = 2;
    }

  
    BackRight.ombro.angulo = map(t, 0, speed, 0, 45+ offset);
    BackRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);

    FrontRight.ombro.angulo = map(t, 0, speed, 45, 0);
    BackLeft.ombro.angulo = map(t, 0, speed, 0, 45-offset);
    // FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
    FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    FrontLeft.ombro.angulo = map(t, 0, speed, 45, 0);

    break;

  case 2:

    if(fsm_right.tis > speed)
    {
      if (action == DIREITA)
      {
        fsm_right.new_state = 1;
      }
      else
      {
        fsm_right.new_state = 0;
        spiderMutex = false;
      }
    }
    FrontRight.ombro.angulo = map(t, 0, speed, 0, 45-offset);
    FrontRight.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    BackRight.ombro.angulo = map(t, 0, speed, 45, 0);
    FrontLeft.ombro.angulo = map(t, 0, speed, 0, 45+offset);
  //FrontLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 0, 60, speed);
    BackLeft.cotovelo.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    BackLeft.ombro.angulo = map(t, 0, speed, 45, 0);
    break;
 }
}

//! Control State Machines

void AroundObject(int speed)
{
  SPINCORRECTIONMICRO = 20;
  SPINCORRECTION = 40;

  speed = 500;
  errorZ = mpu.getAngleZ() - desiredZ;
  switch (fsm_control.state)
  {
  case BASE:

  action= BASE;
    if(spiderMutex == false )
    {
      base();
    }
    if (fsm_control.tis > 5000)
    {
      mpu.resetZ();
      desiredZ = 0;
      fsm_control.new_state = FRENTE;
      Serial.println("\n\n\nFRENTE");
    }
    break;
  
  case FRENTE:
    action = SUBIR;
    if (distance < COLLISION)
    {
      fsm_control.new_state = ESQUERDAF;
      last_collision = curr_time;
    }
    if (errorZ > SPINCORRECTIONMICRO)
    {
      prevstate = FRENTE;
      fsm_control.new_state = RODAPOSITIVOMICRO;

      break;
    }
    if  (errorZ < -SPINCORRECTIONMICRO)
    {
      prevstate= FRENTE;
      fsm_control.new_state = RODANEGATIVOMICRO;
      break;
    }
    break;
  
  
  case ESQUERDAF:
    action= ESQUERDA;
    if (distance < COLLISION) {last_collision = curr_time; break;}
    if(curr_time - last_collision > OFFSETESQUERDA){
   
      lateral_time = fsm_control.tis;
      desiredZ= - 90;
      fsm_control.new_state = ESQUERDAT;
      
      break;
    }
    /* if (errorZ > SPINCORRECTIONMICRO)
    {
      prevstate = ESQUERDAF;
      fsm_control.new_state = RODAPOSITIVOMICRO;

      break;
    }
    if  (errorZ < -SPINCORRECTIONMICRO)
    {
      prevstate= ESQUERDAF;
      fsm_control.new_state = RODANEGATIVOMICRO;
      break;
    } */
      
    break;

    case ESQUERDAT:
      action = ESQUERDA;
      if (distance < COLLISION)
      {
        fsm_control.new_state = ESQUERDAL;
      }
      if (errorZ > SPINCORRECTIONMICRO)
      {
        prevstate = ESQUERDAT;
        fsm_control.new_state = RODAPOSITIVOMICRO;
        break;
      } 
      if  (errorZ < -SPINCORRECTIONMICRO)
    {
      prevstate= ESQUERDAT;
      fsm_control.new_state = RODANEGATIVOMICRO;
      break;
    }
    break;


  case ESQUERDAL:

  action = ESQUERDA;
    if (distance < COLLISION) {last_collision = curr_time; break;}
     if(curr_time - last_collision > OFFSETESQUERDA){
  
        fsm_control.new_state = DIREITA;
        desiredZ= 0;
        break;
      }
    /* if (errorZ > SPINCORRECTIONMICRO)
    {
      prevstate = ESQUERDAL;
      fsm_control.new_state = RODAPOSITIVOMICRO;

      break;
    }
    if  (errorZ < -SPINCORRECTIONMICRO)
    {
      prevstate= ESQUERDAL;
      fsm_control.new_state = RODANEGATIVOMICRO;
      break;
    }
    */
  break; 
  
  case DIREITA:

    action = DIREITA;
    if (fsm_control.tis > lateral_time)
    {
      fsm_control.new_state = FRENTE;
    }
    if (errorZ > SPINCORRECTION)
    {
      prevstate = DIREITA;
      fsm_control.new_state = RODAPOSITIVOMICRO;

      break;
    }
    if  (errorZ < -SPINCORRECTIONMICRO)
    {
      prevstate= DIREITA;
      fsm_control.new_state = RODANEGATIVOMICRO;
      break;
    }
  break;


  case RODAPOSITIVOMICRO:
    action = RODAPOSITIVOMICRO;
    if ( errorZ > SPINCORRECTION)
    {
      fsm_control.new_state = RODAPOSITIVOPOUCO;
      break;
    }
    if (errorZ < SPINCORRECTIONMICRO/4)
    {
      fsm_control.new_state = prevstate;
    }

    break;

  
  case RODANEGATIVOMICRO:
    action = RODANEGATIVOMICRO;
    if ( errorZ < -SPINCORRECTION)
    {
      fsm_control.new_state = RODANEGATIVOPOUCO;
      break;
    }
    if (errorZ > -SPINCORRECTIONMICRO/4)
    {
      fsm_control.new_state = prevstate;
      break;
    }
    break;

  
  

  case RODAPOSITIVOPOUCO:
    action = RODAPOSITIVOPOUCO;
    if (errorZ < SPINCORRECTION)
    {
      fsm_control.new_state = RODAPOSITIVOMICRO;
    }
    break;

  
  case RODANEGATIVOPOUCO:
    action = RODANEGATIVOPOUCO;
    if (errorZ > -SPINCORRECTION)
    {
      fsm_control.new_state = RODANEGATIVOMICRO;
    }
    break;
  }

}

void ObstacleAvoidance(int speed)
{
  speed = 500;
  switch (fsm_control.state)
  {
  case BASE:
  action= BASE;

    if(spiderMutex == false)
    {
      base();
    }
    if (fsm_control.tis > 5000)
    {
      fsm_control.new_state = FRENTE;
    }
    break;
  
  case FRENTE:
      action = FRENTE;
    if (distance < COLLISION)
    {
      fsm_control.new_state = RODAPOSITIVO;
      last_collision = curr_time;
    }
    break;
  
  case RODAPOSITIVO:

  action = RODAPOSITIVO;
  
    if (fsm_control.tis > 500)
    {
      fsm_control.new_state = FRENTE;
    }
    break;

  
  case RODANEGATIVO:


  action = RODANEGATIVO;


     
    if (fsm_control.tis > 500)
    {
      fsm_control.new_state = ESQUERDAF;
    }
    break;

  
  case ESQUERDAF:

  action= ESQUERDAF;
    if (distance < COLLISION) {last_collision = curr_time; break;}
     if(curr_time - last_collision > OFFSETESQUERDA){
    if(rodarflag == 0)
    {
      lateral_time= fsm_control.tis;
      rodarflag = 1;
      fsm_control.new_state = RODANEGATIVO;
    }
    else
    {
      rodarflag = 0;
      fsm_control.new_state = RODAPOSITIVO;
    }}
   
    

      
    break;

  
  case DIREITA:

  action= DIREITA;
    if (fsm_control.tis > lateral_time)
    {
      fsm_control.new_state = FRENTE;
    }
    break;


  }
  
}

void StraighLineWithIMU(int speed)
{
  speed = 200;
  SPINCORRECTIONMICRO = 30;

  desiredZ = 0;
  errorZ = mpu.getAngleZ() - desiredZ;

  switch (fsm_control.state)
  {
  case BASE:
    Serial.println("BASE");

    action= BASE;
    if(spiderMutex == false)
    {
      spiderMutex = true;
      base();
    }
    if (fsm_control.tis > 5000)
    {
      spiderMutex = false;
      fsm_control.new_state = FRENTE;
      mpu.resetZ();

    }
    break;
  
  case FRENTE:
    action = FRENTE;
  
    if (errorZ > SPINCORRECTIONMICRO)
    {
      //spiderMutex = false;
      fsm_control.new_state = RODAPOSITIVOMICRO;
      break;
    }
    if  (errorZ < -SPINCORRECTIONMICRO)
    {
      //spiderMutex = false;
      fsm_control.new_state = RODANEGATIVOMICRO;
      break;
    }
  break;
  
  
  case RODAPOSITIVOMICRO:
    action = RODAPOSITIVOMICRO;
    if (errorZ < SPINCORRECTIONMICRO/4)
    {
      fsm_control.new_state = FRENTE;
    }
    break;

  
  case RODANEGATIVOMICRO:
    action = RODANEGATIVOMICRO;
    if (errorZ > -SPINCORRECTIONMICRO/4)
    {
      fsm_control.new_state = FRENTE;
    }
    break;

  }
}

void Compass(int speed)
{
  speed = 100;
  desiredZ = 0;
  errorZ = mpu.getAngleZ() - desiredZ;

  switch (fsm_control.state)
  {
  case BASE:
    action = BASE;
    if(spiderMutex == false)
    {
      spiderMutex = true;
      cotovelosRetos();
    }
    if (errorZ > SPINCORRECTION)
    {
      spiderMutex = false;
      fsm_control.new_state = RODAPOSITIVOPOUCO;
      break;
    }

  
    if  (errorZ < -SPINCORRECTION)
    {
      spiderMutex = false;
      fsm_control.new_state = RODANEGATIVOPOUCO;
      break;
    }

    if (errorZ > SPINCORRECTIONMICRO)
    {
      spiderMutex = false;
      fsm_control.new_state = RODAPOSITIVOMICRO;
      break;
    }
    if  (errorZ < -SPINCORRECTIONMICRO)
    {
      spiderMutex = false;
      fsm_control.new_state = RODANEGATIVOMICRO;
      break;
    }

  break;
  
  
  case RODAPOSITIVOPOUCO:
    action = RODAPOSITIVOPOUCO;
    if (errorZ < SPINCORRECTION)
    {
      fsm_control.new_state = BASE;
    }
    break;

  
  case RODANEGATIVOPOUCO:
    action = RODANEGATIVOPOUCO;
    if (errorZ > -SPINCORRECTION)
    {
      fsm_control.new_state = BASE;
    }
    break;

   
  case RODAPOSITIVOMICRO:
    action = RODAPOSITIVOMICRO;
    if (errorZ > SPINCORRECTION)
    {
      fsm_control.new_state = RODAPOSITIVOPOUCO;
    }
    
    if (errorZ < SPINCORRECTIONMICRO)
    {
      fsm_control.new_state = BASE;
    }
    break;


  case RODANEGATIVOMICRO:
    action = RODANEGATIVOMICRO;
    if (errorZ < -SPINCORRECTION)
    {
      fsm_control.new_state= RODANEGATIVOPOUCO;
      
    }
    if (errorZ > -SPINCORRECTIONMICRO)
    {
      fsm_control.new_state = BASE;
    }
    break;

  

  

    
  }
}

void SubirIMU(int speed){

  SPINCORRECTIONMICRO = 50;

  desiredZ = 0;
  errorZ = mpu.getAngleZ() - desiredZ;

  switch (fsm_control.state)
  {
  case BASE:
    Serial.println("BASE");

    action= BASE;
    if(spiderMutex == false)
    {
      spiderMutex = true;
      base();
    }
    if (fsm_control.tis > 5000)
    {
      spiderMutex = false;
      fsm_control.new_state = SUBIR;
    }
    break;
  
  case SUBIR:
    action = SUBIR;
  
    if (errorZ > SPINCORRECTION)
    {
      fsm_control.new_state = RODAPOSITIVOPOUCO;
      break;
    }
    if  (errorZ < -SPINCORRECTION)
    {
      fsm_control.new_state = RODANEGATIVOPOUCO;
      break;
    }
  break;
  
  
  case RODAPOSITIVOPOUCO:
    action = RODAPOSITIVOPOUCO;
    if (errorZ < SPINCORRECTION/4)
    {
      fsm_control.new_state = SUBIR;
    }
    break;

  
  case RODANEGATIVOPOUCO:
    action = RODANEGATIVOPOUCO;
    if (errorZ > -SPINCORRECTION/4)
    {
      fsm_control.new_state = SUBIR;
    }
    break;

  }}
//!
void set_outputs()
{
  BackRight.ombro.updateAngle();
  BackRight.cotovelo.updateAngle();
  FrontRight.ombro.updateAngle();
  FrontRight.cotovelo.updateAngle();
  FrontLeft.ombro.updateAngle();
  FrontLeft.cotovelo.updateAngle();
  BackLeft.ombro.updateAngle();
  BackLeft.cotovelo.updateAngle();
}

void print_states()
{
  if (curr_time - last_print_time < print_interval) {}
  else
  {
    last_print_time = curr_time;
    //Serial.print(String(fsm_control.state) + " ");
    Serial.print(String(action) + " ");
    Serial.print(String(fsm_walkForwardFAST.state) + " ");
    Serial.print(String(fsm_rodaPositivo.state) + " ");
    Serial.print(String(fsm_rodaNegativo.state) + " ");
    Serial.print(String(fsm_left.state) + " ");
    Serial.print(String(fsm_right.state) + " ");
    Serial.print(String(fsm_rodaPositivoPouco.state)+ " ");
    Serial.print(String(fsm_rodaNegativoPouco.state)+ " ");
    Serial.print(String(fsm_rodaPositivoMicro.state)+ " ");
    Serial.print(String(fsm_rodaNegativoMicro.state)+ " ");
    Serial.print(String(fsm_subir.state)+ " ");
    Serial.print("Mutex "+String(spiderMutex) + " ");
    #ifdef TOF
    Serial.print(distance);
    #endif
    #ifdef IMU
    //Serial.print("Angle X: ");
    //Serial.print(mpu.getAngleX());
    //Serial.print("°, Angle Y: ");
    //Serial.print(mpu.getAngleY());
    Serial.print("°, Angle Z: ");
    Serial.print(mpu.getAngleZ());
    Serial.print("° ");
    Serial.print(errorZ);
    #endif
    
    Serial.println();
    
  } 

}

void loop()
{
  curr_time = millis();
  if(curr_time - last_loop_time < loop_interval){}

  else
  {
    last_loop_time = curr_time;

    #ifdef IMU
    mpu.update(curr_time);
    #endif

    #ifdef TOF
    distance = tof.readRangeContinuousMillimeters();
    if (tof.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    #endif

    update_tis();
   
    //!Comand State Machines
    //AroundObject(500);
    //ObstacleAvoidance();
    //SubirIMU(500);
    //StraighLineWithIMU(200);
    //Compass(200);
    //reto();

    //! Movement State Machines
    //walkForwardFAST(200);
    //rodaNegativo(200);
    //rodaPositivo(100);
    //left(500);
    //right(500);
    //rodaNegativoPouco(200);
    //rodaPositivoPouco(200);
    //rodaNegativoMicro(200);
    //rodaPositivoMicro(200);
    //subir(500);
    //swim(200);
//pushup(500);

    
    update_fsms();
    set_outputs();
    //print_states();

  }

}
