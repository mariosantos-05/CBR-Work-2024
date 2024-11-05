#include <Arduino.h>
#include <DC_Motor.h>
#include <Stepper.h>
#include <Servo.h>
#include <Color_Sensor.h>
#include <Ultrassonic_Sensor.h>
#include <IRSensor.h>

// Definiçao de portas    (orientação baseada na garra)

// Motor front 
const int Front_en = 1;
const int Front_RPWM = 1;
const int Front_LPWM = 1;
const int EncA_F = 1 ;
const int EncB_F = 1;

MotorDC FM(EncA_F, EncB_F, Front_RPWM, Front_LPWM);

// Motor Right
const int Right_en = 1;
const int Right_RPWM = 1;
const int Right_LPWM = 1;
const int EncA_R = 1;
const int EncB_R = 1;

MotorDC RM(EncA_R, EncB_R, Right_RPWM, Right_LPWM);

// Motor Left
const int Left_en = 1;
const int Left_RPWM = 1;
const int Left_LPWM = 1;
const int EncA_L = 1;
const int EncB_L = 1;
MotorDC LM(EncA_L, EncB_L, Left_RPWM, Left_LPWM);

// Motor back
const int Back_en = 1;
const int Back_RPWM = 1;
const int Back_LPWM = 1;
const int EncA_B = 1;
const int EncB_B = 1;
MotorDC BM(EncA_B, EncB_B, Back_RPWM, Back_LPWM);

// claw
const int Servo1 = 9;
Servo claw;

// Fork lift
const int IN1 = 1;
const int IN2 = 1;
const int IN3 = 1;
const int IN4 = 1;
const int StepPerRevolution = 2048;
Stepper ForkLift(StepPerRevolution,IN1,IN2,IN3,IN4);

// Gyro


// ULtrassonic Sensor

const int Front1_trig = 1;
const int Front1_echo = 1;
Sensor F1(Front1_trig, Front1_echo);
const int Front2_trig = 51;
const int Front2_echo = 53;
Sensor F2(Front2_trig, Front2_echo);
const int Front3_trig = 42;
const int Front3_echo = 40;
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
  pinMode(Right_en, OUTPUT);
  pinMode(Left_en, OUTPUT);
  digitalWrite(Right_en, HIGH);
  digitalWrite(Left_en, HIGH);
  Serial1.begin(9600);

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5, OUTPUT);

}

void loop() {
  digitalWrite(13, 255); 
  digitalWrite(11, 255);
  digitalWrite(9, 255);
  digitalWrite(6, 20);
  digitalWrite(12, 0); 
  digitalWrite(10, 0);
  digitalWrite(8, 0);
  digitalWrite(5, 0);
  
}