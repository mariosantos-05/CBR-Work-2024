#include <Arduino.h>
#include <DC_Motor.h>
#include <Stepper.h>
#include <Servo.h>
#include <Color_Sensor.h>
#include <Ultrassonic_Sensor.h>
#include <IRSensor.h>
#include <AccelStepper.h>

// Definiçao de portas    (orientação baseada na garra)

// Motor front 
const int Front_RPWM = 13;
const int Front_LPWM = 12;
const int EncA_F = 1 ;
const int EncB_F = 1;

MotorDC FM(EncA_F, EncB_F, Front_RPWM, Front_LPWM);

// Motor Right
const int Right_RPWM = 10;
const int Right_LPWM = 11;
const int EncA_R = 1;
const int EncB_R = 1;

MotorDC RM(EncA_R, EncB_R, Right_RPWM, Right_LPWM);

// Motor Left
const int Left_RPWM = 8;
const int Left_LPWM = 9;
const int EncA_L = 1;
const int EncB_L = 1;
MotorDC LM(EncA_L, EncB_L, Left_RPWM, Left_LPWM);

// Motor back
const int Back_RPWM = 7;
const int Back_LPWM = 6;
const int EncA_B = 1;
const int EncB_B = 1;
MotorDC BM(EncA_B, EncB_B, Back_RPWM, Back_LPWM);

// claw
const int Servo1 = 9;
Servo claw;

// Fork lift
const int STEP_PIN = 2;
const int DIR_PIN = 3; 
AccelStepper Stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Gyro


// ULtrassonic Sensor

const int Front1_trig = 30;
const int Front1_echo = 32;
Sensor F1(Front1_trig, Front1_echo);
const int Front2_trig = 29;
const int Front2_echo = 31;
Sensor F2(Front2_trig, Front2_echo);
const int Front3_trig = 25;
const int Front3_echo = 27;
Sensor F3(Front3_trig, Front3_echo);

const int Right1_trig = 1;
const int Right1_echo = 1;
Sensor R1(Right1_trig, Right1_echo);
const int Right2_trig = 1;
const int Right2_echo = 1;
Sensor R2(Right2_trig, Right2_echo);

const int Left1_trig = 1;
const int Left1_echo = 1;
Sensor L1(Left1_trig, Left1_trig);
const int Left2_trig = 1;
const int Left2_echo = 1;
Sensor L2(Left2_trig, Left2_trig);


const int Back1_trig = 1;
const int Back1_echo = 1;
Sensor L3(Back1_trig, Back1_echo);
const int Back2_trig = 1;
const int Back2_echo = 1;
Sensor L4(Back2_trig, Back2_echo);


// Infra 

const int infra_Right = 1;
const int infra_Center = 1;
const int infra_Left = 1;

IRSensor IR_R(infra_Right);
IRSensor IR_C(infra_Center);
IRSensor IR_L(infra_Left);


// Color Sensor
const int S0 = 1;
const int S1 = 1;
const int S2 = 1;
const int S3 = 1;
const int Out = 1;

ColorSensor color_front(S0,S1,S2,S3,Out);



void setup() {
  claw.attach(Servo1);
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, OUTPUT);
  Stepper.setMaxSpeed(1000);
  Stepper.setAcceleration(500);

}

void loop() {
  Stepper.moveTo(200);
  Stepper.runToPosition();
  delay(1000);
  Stepper.moveTo(-200);
  Stepper.runToPosition();
  delay(1000);
  
}
