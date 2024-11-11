#include <Arduino.h>
#include <DC_Motor.h>
#include <Servo.h>
#include <Color_Sensor.h>
#include <Ultrassonic_Sensor.h>
#include <IRSensor.h>
#include <SoftwareSerial.h>


// Definiçao de portas    (orientação baseada na garra)

// Motor front 
const int Front_LPWM = 11;
const int Front_RPWM = 10;
const int EncA_F = 1 ;
const int EncB_F = 1;

MotorDC FM(EncA_F, EncB_F, Front_RPWM, Front_LPWM);

// Motor Right
const int Right_RPWM = 4;
const int Right_LPWM = 5;
const int EncA_R = 1;
const int EncB_R = 1;

MotorDC RM(EncA_R, EncB_R, Right_RPWM, Right_LPWM);

// Motor Left
const int Left_RPWM = 6;
const int Left_LPWM = 7;
const int EncA_L = 1;
const int EncB_L = 1;
MotorDC LM(EncA_L, EncB_L, Left_RPWM, Left_LPWM);

// Motor back
const int Back_RPWM = 8;
const int Back_LPWM = 9;
const int EncA_B = 1;
const int EncB_B = 1;
MotorDC BM(EncA_B, EncB_B, Back_RPWM, Back_LPWM);

// claw
const int Servo1 = 13;
Servo claw;



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

const int infra_Right = 68;
const int infra_Center = 66;
const int infra_Left = 64;

IRSensor IR_R(infra_Right);
IRSensor IR_C(infra_Center);
IRSensor IR_L(infra_Left);


// Color Sensor
const int S0 = 1;
const int S1 = 2;
const int S2 = 5;
const int S3 = 6;
const int Out = 4;

ColorSensor color_FL(S0,S1,S2,S3,Out);

const int S0 = 1;
const int S1 = 2;
const int S2 = 5;
const int S3 = 6;
const int Out = 4;

ColorSensor color_FC(S0,S1,S2,S3,Out);
const int S0 = 1;
const int S1 = 2;
const int S2 = 5;
const int S3 = 6;
const int Out = 4;

ColorSensor color_FR(S0,S1,S2,S3,Out);

// Color Sensor
const int S0 = 1;
const int S1 = 2;
const int S2 = 5;
const int S3 = 6;
const int Out = 4;

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
float R = 0.15;


void calc_speed(float x_dot, float y_dot, float theta_dot , float scale_x ) {
  // Calcula o PWM para cada roda
  float pwmFrente   = -R * theta_dot - y_dot;
  float pwmDireito  = R * theta_dot + x_dot;
  float pwmTras     = -R * theta_dot + y_dot;
  float pwmEsquerdo = R * theta_dot - x_dot;

  // Determina a direção para cada roda
  int dirFrente   = (pwmFrente > 0) - (pwmFrente < 0);
  int dirDireito  = (pwmDireito > 0) - (pwmDireito < 0);
  int dirTras     = (pwmTras > 0) - (pwmTras < 0);
  int dirEsquerdo = (pwmEsquerdo > 0) - (pwmEsquerdo < 0);

  // Escala o PWM para a faixa de 0 a 255 e usa o valor absoluto
  pwmFrente   = (int)abs(pwmFrente * scale_x);
  pwmDireito  = (int)abs(pwmDireito * scale_x);
  pwmTras     = (int)abs(pwmTras * scale_x);
  pwmEsquerdo = (int)abs(pwmEsquerdo * scale_x);

  // Aplica direção e velocidade para o motor da frente
  if (dirFrente == 1) { // Movimento para frente
    FM.turn_on_motor(FORWARD, pwmFrente);
  } else if (dirFrente == -1) { // Movimento para trás
    FM.turn_on_motor(REVERSE, pwmFrente);
  } else { // Parado
    FM.move_straight(STOP);
  }

  // Aplica direção e velocidade para o motor da direita
  if (dirDireito == 1) { // Movimento para frente
    RM.turn_on_motor(FORWARD, pwmDireito);
  } else if (dirDireito == -1) { // Movimento para trás
    RM.turn_on_motor(REVERSE,pwmDireito);
  } else { // Parado
    RM.move_straight(STOP);
  }

  // Aplica direção e velocidade para o motor de trás
  if (dirTras == 1) { // Movimento para frente
    BM.turn_on_motor(FORWARD, pwmTras);
  } else if (dirTras == -1) { // Movimento para trás
    BM.turn_on_motor(REVERSE, pwmTras);
  } else { // Parado
    BM.move_straight(STOP);
  }

   // Aplica direção e velocidade para o motor de trás
  if (dirEsquerdo == 1) { // Movimento para frente
    LM.turn_on_motor(FORWARD, pwmEsquerdo);
  } else if (dirEsquerdo == -1) { // Movimento para trás
    LM.turn_on_motor(REVERSE, pwmEsquerdo);
  } else { // Parado
    LM.move_straight(STOP);
  }

}

void setup() {
  Serial.begin(9600);
  color_Clawn.begin();
}

void follow_line(int entrance){
  int r1, g1, b1;
  int r2, g2, b2;
  int r3, g3, b3;

  int L, C, R;

  color_FL.readColor(&r1, &g1, &b1);
  color_FC.readColor(&r2, &g2, &b2);
  color_FR.readColor(&r3, &g3, &b3);

  if(r1 >= 8) L = 1;
  else L = 0;

  if(r2 >= 8) C = 1;
  else C = 0;

  if(r3 >= 8) R = 1;
  else R = 0;

  if (L == 0 && C == 1 && R == 0) {

    calc_speed(1, 0, 0, 80);
  }
  else if (L == 1 && C == 0 && R == 0) {

    calc_speed(0.5, 0.5, 0, 80);
  }
  else if (L == 0 && C == 0 && R == 1) {

    calc_speed(0.5, -0.5, 0, 80);
  }
  else if (L == 1 && C == 1 && R == 0) {

    calc_speed(0, -1, 0, 80);
  }
  else if (L == 0 && C == 1 && R == 1) {
    calc_speed(0, 1, 0, 80);
  }
  else if (L == 1 && C == 1 && R == 1) {

    calc_speed(0, 0, 0, 0); 
  }
  delay(100);
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
  while (F2.getDistance() > 10){
    calc_speed(1,0,0,110);
    delay(1000);
  }
  service_table(); 
  while (!IR_C.isLineDetected()){
    calc_speed(0,1,0,110);
    delay(1000);
  }
    calc_speed(1,1,1,110);
    delay(FULL_TURN);
}

void Finish(bool shelve){
  int L = IR_L.isLineDetected();
  int R = IR_R.isLineDetected();
  if (shelve){
    if(L){
      while (F2.getDistance()> 10){
        calc_speed(1,0,0,110);
      }
    }
  }
  else{
    if(R){
      calc_speed(1,1,1,100);
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

void loop(){

  int ri, g, b;
  color_Clawn.readColor(&ri,&g,&b);

  if(ri >= 8){
    Serial.println("EH LINHA PORRA, BORA BORA");
  }
  else{
    Serial.println("Eh esse branco vei paia...");
  }


}