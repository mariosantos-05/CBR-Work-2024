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
const int EncA_F;
const int EncB_F;

MotorDC FM(EncA_F, EncB_F, Front_RPWM, Front_LPWM);

// Motor Right
const int Right_en = 1;
const int Right_RPWM = 1;
const int Right_LPWM = 1;
const int EncA_R;
const int EncB_R;

MotorDC RM(EncA_R, EncB_R, Right_RPWM, Right_LPWM);

// Motor Left
const int Left_en = 1;
const int Left_RPWM = 1;
const int Left_LPWM = 1;
const int EncA_L;
const int EncB_L;
MotorDC LM(EncA_L, EncB_L, Left_RPWM, Left_LPWM);

// Motor back
const int Back_en = 1;
const int Back_RPWM = 1;
const int Back_LPWM = 1;
const int EncA_B;
const int EncB_B;
MotorDC BM(EncA_B, EncB_B, Back_RPWM, Back_LPWM);

// claw
const int Claw_en = 1;
const int Claw_RPWM = 1;
const int Claw_LPWM = 1;
const int EncA_Claw;
const int EncB_Claw;
MotorDC FLM(EncA_Claw, EncB_Claw, Claw_RPWM, Claw_LPWM);

// Fork lift
const int IN1;
const int IN2;
const int IN3;
const int IN4;
const int StepPerRevolution = 2048;
Stepper ForkLift(StepPerRevolution,IN1,IN2,IN3,IN4);

// Gyro
const int SCL = 1;
const int SDA = 1;

// ULtrassonic Sensor

const int Front1_trig = 1;
const int Front1_echo = 1;
Sensor F1(Front1_trig, Front1_echo);
const int Front2_trig = 1;
const int Front2_echo = 1;
Sensor F2(Front2_trig, Front2_echo);
const int Front3_trig = 1;
const int Front3_echo = 1;
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
Sensor L1(Left2_trig, Left2_trig);


const int Back1_trig = 1;
const int Back1_echo = 1;
Sensor L1(Back1_trig, Back1_echo);
const int Back2_trig = 1;
const int Back2_echo = 1;
Sensor L1(Back2_trig, Back2_echo);


// Infra 

const int infra_Right = 1;
const int infra_Center = 1;
const int infra_Left = 1;

IRSensor IR_R(infra_Right);
IRSensor IR_C(infra_Center);
IRSensor IR_L(infra_Left);


// Color Sensor
const int S0;
const int S1;
const int S2;
const int S3;
const int Out;

ColorSensor color_front(S0,S1,S2,S3,Out);



void setup() {

  pinMode(Right_en, OUTPUT);
  pinMode(Left_en, OUTPUT);
  digitalWrite(Right_en, HIGH);
  digitalWrite(Left_en, HIGH);
  Serial1.begin(9600);
}

void loop() {
  RM.turn_on_motor(FORWARD,255);
  LM.turn_on_motor(FORWARD,255);

}