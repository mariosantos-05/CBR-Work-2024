#include <Arduino.h>
#include <DC_Motor.h>
#include <Servo.h>
#include <Color_Sensor.h>
#include <Ultrassonic_Sensor.h>
#include <IRSensor.h>
#include <SoftwareSerial.h>
#include "L298N.h"
#include <cor2.h>


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

// ULtrassonic Sensor

Sensor F1(A1, A0);

Sensor F2(A3, A2);

Sensor F3(A5, A4);

Sensor R1(A11, A10);

Sensor R2I(A13, A12);

Sensor R2(A15, A14);

Sensor L1(A7, A6);

Sensor L2(A9, A8);



const int claw_trig = 37;
const int claw_echo = 39;
Sensor claw_sensor(claw_trig, claw_echo);

// Color Sensor

SensorCorTCS230 teste_Dir(49, 51, 45, 43, 47);
SensorCorTCS230 teste_Cent(40, 38, 34, 42, 36);
SensorCorTCS230 teste_Esq(50, 52, 48, 44, 46);
SensorCorTCS230 claw_Color(29, 31, 25, 23, 27);


L298N motor(3, 2);

// Global Variables
SoftwareSerial raspy(10, 11);
String receivedData = ""; // Armazena a mensagem recebida

Servo claw;

const int QUARTER_TURN  = 1000;
const int HALF_TURN = 10000;
const long FULL_TURN = 100000;
const int RUN_TIME = 1000;
const int PWM_X = 70;
const int PWM_Y = 70;
const int OPEN = 0;
const int CLOSED = 100;
const int CUBE = 10;
const int Limite_E = 35;
const int Limite_D = 35;
const int Limite_e = 35;
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

void claw_hight(int altura){

}

void Container_table(String cor){
  while(L1.getDistance() > 10){
    calc_speed(0,-1,0,PWM_X);
  }
  while(F2.getDistance() < 18){
    while(claw_Color.identificarCor() == cor ){
      calc_speed(0,1,0,PWM_X);
      delay(CUBE);
      claw.write(OPEN);
      delay(1000);
    }
  }
}

void finish() {
  motor.forward(80);
  delay(100);
  calc_speed(1,0,0,PWM_X);
  delay(1000);
  calc_speed(0,0,1,PWM_X);
  delay(1000);
  calc_speed(1,0,0,PWM_X);
  
  delay(1000);
}

void service_table(String *A){
  String color = "";
  //claw_hight(15);
  while(R1.getDistance() >= 10){
    calc_speed(0,-1,0,PWM_X);
  }

  calc_speed(0,0,0,0);
  delay(1000);

  calc_speed(0,1,0,PWM_Y);
  delay(3000);

  calc_speed(0,0,0,0);
  delay(1000);

  calc_speed(0,1,0, PWM_Y);
  delay(100000); //teste
  calc_speed(0,0,0,0);
  delay(1000);

  calc_speed(0,0,1,70);
  delay(HALF_TURN);

  calc_speed(0,1,0, PWM_Y);
  delay(10000);

  calc_speed(0,0,0,0);
  delay(10000);

  calc_speed(1,0,0,PWM_Y);
  delay(1000);

  calc_speed(0,0,0,0);
  delay(1000);

  calc_speed(0,-1,0,PWM_Y);
  delay(4000);

  calc_speed(0,0,0,0);
  delay(1000);

  calc_speed(1,0,0,PWM_X);
  delay(3000);

  calc_speed(0,1,0, PWM_X);
  delay(2000);
  
  calc_speed(0,0,0,0);
  delay(1000);

}

void start(){
  int L, C, R;

  teste_Cent.lerCores();
  teste_Dir.lerCores();
  teste_Esq.lerCores();

  if(teste_Esq.getVermelho() >= 8) L = 1;
  else L = 0;

  if(teste_Cent.getVermelho() >= 8) C = 1;
  else C = 0;

  if(teste_Dir.getVermelho()>= 8) R = 1;
  else R = 0;

  while(int(F1.getDistance()) > 15){
    calc_speed(1,0,0,PWM_X);
  }
  calc_speed(0,0,0,0);

  String Cor_cubo;
  service_table(&Cor_cubo);
  while((!L  && C)&& !R){
    calc_speed(1,0,0,100);
  }
  while((!L  && C)&& !R){
    calc_speed(0,0,1,180);
  }
  //follow_line();
}

void run1(){
  int L, C, R;

  while(int(F1.getDistance()) > 15){
    calc_speed(1,0,0,PWM_X);
  }
  calc_speed(0,0,0,0);

  String Cor_cubo;
  service_table(&Cor_cubo);
  

  //follow_line();
}

void setup() {
  Serial.begin(9600);
  motor.begin();
  claw.attach(35);
}

bool passou = true;

void loop() {
  motor.forward(70);
  delay(100);
  motor.stop();

  delay(1000);

}