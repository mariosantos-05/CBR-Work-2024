#include <Arduino.h>
#include <DC_Motor.h>
#include <Servo.h>
#include <Color_Sensor.h>
#include <Ultrassonic_Sensor.h>
#include <IRSensor.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>


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

ColorSensor color_Clawn(S0,S1,S2,S3,Out);

const int SF0 = 1;
const int SF1 = 1;
const int SF2 = 1;
const int SF3 = 1;
const int OutF = 1;

ColorSensor color_front(SF0,SF1,SF2,SF3,OutF);


// Global Variables
SoftwareSerial raspy(10, 11);
String receivedData = ""; // Armazena a mensagem recebida

const int QUARTER_TURN  = 1000;
const int HALF_TURN = 10000;
const long FULL_TURN = 100000;
const int RUN_TIME = 1000;

void setup() {
  claw.attach(Servo1);
  Serial.begin(115200); // Inicializa a comunicação serial
  raspy.begin(115200);
  digitalWrite(12, OUTPUT);
  Stepper.setMaxSpeed(1000);
  Stepper.setAcceleration(500);
}




void table_high(int hight){
  if(hight == 5){
    Stepper.moveTo(200);
    Stepper.runToPosition();
    delay(1000);
  }
  else if(hight == 10){
    Stepper.moveTo(200);
    Stepper.runToPosition();
    delay(1000);
  }
  
  else{
    Stepper.moveTo(200);
    Stepper.runToPosition();
    delay(1000);    
  }
}

void follow_line(int entrance){
  int flag = 0;
  int L = IR_L.isLineDetected();
  int C = IR_C.isLineDetected();
  int R = IR_R.isLineDetected();
  if((L == 0 and C == 1) and R == 0){
    // just go forward
    LM.move_straight(100);
    RM.move_straight(100);
    delay(1000);
  }
  else if((L == 1 and C == 0) and R == 0){
    // right motor go a little
    FM.move_straight(100);
    BM.move_straight(100);
    delay(1000);
  }
  else if((L == 0 and C == 0) and R == 1){
    // Left motor go a little
    FM.turn_on_motor(REVERSE, 100);
    BM.turn_on_motor(REVERSE, 100);
    delay(1000);
  }
  else if((L == 1 and C == 1) and R == 0){
    flag++;
  }
  else if((L == 0 and C == 1) and R == 1){
    flag++;
  }
  else if(entrance == 1){
    //turn 90 to left
    if((L == 1 and C == 1) and R == 0){
      FM.turn_on_motor(FORWARD, 100);
      BM.turn_on_motor(REVERSE, 100);
      RM.turn_on_motor(FORWARD, 100);
      LM.turn_on_motor(REVERSE, 100);
      delay(QUARTER_TURN);
    }
  }
  else if(entrance == 2 and flag == 1){
    if((L == 1 and C == 1) and R == 0){
      FM.turn_on_motor(FORWARD, 100);
      BM.turn_on_motor(REVERSE, 100);
      RM.turn_on_motor(FORWARD, 100);
      LM.turn_on_motor(REVERSE, 100);
      delay(QUARTER_TURN);
    }
  }
}


String Rasp_Data(){
   // Verifica se há dados disponíveis para leitura
    if (raspy.available() > 0) {
        char incomingChar = raspy.read(); // Lê o caractere recebido

        // Armazena o caractere na string até encontrar '\n'
        if (incomingChar != '\n') {
            receivedData += incomingChar;
        } else {
            // Verifica a mensagem completa recebida
            if (receivedData == "V") {
                return receivedData;
            }
            else {
                // Converte o ID da tag recebida para um número inteiro
                int tagId = receivedData.toInt();
                if (tagId > 0) { // Verifica se foi recebido um ID válido
                    Serial.print("AprilTag detectada com ID: ");
                    Serial.println(tagId);
                    // Aqui você pode adicionar lógica específica para cada ID
                    return receivedData;
                }
            }
            // Limpa a string para receber a próxima mensagem
            receivedData = "";
        }
    }
    return receivedData;
}

void service_table(){

}

void precision_table(){

}

void container_table(){

}

void shelve(){

}

void Start(){
  while (F1.getDistance()> 10){
    RM.turn_on_motor(FORWARD, 100);
    LM.turn_on_motor(FORWARD, 100);
  }
  service_table(); 
  while (!IR_C.isLineDetected()){
    BM.turn_on_motor(FORWARD, 100);
    FM.turn_on_motor(FORWARD, 100);  
  }
  // 180 turn
  BM.turn_on_motor(FORWARD, 100);
  FM.turn_on_motor(REVERSE, 100);
  LM.turn_on_motor(FORWARD, 100);
  RM.turn_on_motor(REVERSE, 100);
  delay(10000);
}

void Finish(bool shelve){
  int L = IR_L.isLineDetected();
  int R = IR_R.isLineDetected();
  if (shelve){
    if(L){
      while (F1.getDistance()> 10){
        RM.turn_on_motor(FORWARD, 100);
        LM.turn_on_motor(FORWARD, 100);
      }
    }
  }
  else{
    if(R){
      FM.turn_on_motor(FORWARD, 100);
      BM.turn_on_motor(REVERSE, 100);
      RM.turn_on_motor(FORWARD, 100);
      LM.turn_on_motor(REVERSE, 100);
      delay(QUARTER_TURN);
    }
  }
}



void case_base(){
  Start();
  int bifurcation = 1; 
  follow_line(bifurcation);
  //  Insert here the desired table
  // Usually were gonna have more than 1 table,
  bool shelve = false;
  Finish(shelve);
}

void loop() {
  //Testing the infra red 

}
