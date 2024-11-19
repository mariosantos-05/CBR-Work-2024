#include <Arduino.h>
#include <DC_Motor.h>
#include <Servo.h>
#include <Ultrassonic_Sensor.h>
#include <SoftwareSerial.h>
#include "L298N.h"
#include <RunningAverage.h>
#include <BTS7960.h>
#include "../src/controleGeral/seguidorLinha.h"
#include "../src/controleGeral/cinematicaRobo.h"
#include "../src/controleGeral/sensoresCor.h"
    

// Pin definition (using as ref the claw)

// Were not using Encoder, so just a random pin 
const int EncA_F = 1 ;
const int EncB_F = 1;
const int EncA_R = 1;
const int EncB_R = 1;
const int EncA_L = 1;
const int EncB_L = 1;
const int EncA_B = 1;
const int EncB_B = 1;

const int Front_LPWM = 11;
const int Front_RPWM = 10;
const int Right_RPWM = 4;
const int Right_LPWM = 5;
const int Left_RPWM = 6;
const int Left_LPWM = 7;
const int Back_RPWM = 8;
const int Back_LPWM = 9;
MotorDC FM(EncA_F, EncB_F, Front_RPWM, Front_LPWM);
MotorDC RM(EncA_R, EncB_R, Right_RPWM, Right_LPWM);
MotorDC LM(EncA_L, EncB_L, Left_RPWM, Left_LPWM);
MotorDC BM(EncA_B, EncB_B, Back_RPWM, Back_LPWM);

// ULtrassonic Sensor

Sensor F1(A1, A0);

Sensor F2(A3, A2);

Sensor F3(A5, A4);

Sensor R2(A11, A10);

Sensor R2I(A13, A12);

Sensor R3(A15, A14);

Sensor L1(A7, A6);

Sensor L2(A9, A8);

const int claw_trig = 39;
const int claw_echo = 37;
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
const int PWM_X = 80;
const int PWM_Y = 80;
const int OPEN = 0;
const int CLOSED = 100;
const int CUBE = 10;
const int Limite_E = 120;
const int Limite_D = 180;
const int Limite_C = 70;
float R = 0.15;

void calc_speed2(float x_dot, float y_dot, float theta_dot , float scale_x ) {
  
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

void finish() {
  motor.forward(80);
  delay(100);
  calc_speed2(1,0,0,PWM_X);
  delay(1000);
  calc_speed2(0,0,1,PWM_X);
  delay(1000);
  calc_speed2(1,0,0,PWM_X);
  
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  motor.begin();
  claw.attach(35);
  claw.write(OPEN);
  limpaMedia();
}

bool pega_cubo(){

  claw.write(OPEN);
  delay(2000);

  while(claw_sensor.getDistance() > 20 ){
    calc_speed2(0,0,0,0);
    delay(500);  

    calc_speed2(0,-1,0,PWM_Y);
    delay(100); 
  }
  

  calc_speed2(0,0,0,0);
  delay(2000);
  
  unsigned long T1 = millis();
  unsigned long T2;
  while(claw_sensor.getDistance() > 2){
    calc_speed2(0,0,0,0);
    delay(100);
    calc_speed2(1,0,0,PWM_X);
    delay(100);
    T2 = millis();;
    if(T2-T1 > 8000 and claw_sensor.getDistance()>8){
      calc_speed2(-1,0,0,PWM_X);
      delay(1000);
      while (F1.getDistance() > 19){
        calc_speed2(1,0,0,PWM_X);
        delay(150);
        calc_speed2(0,0,0,0);
        delay(150);
      }
      return 0;
    
    }
  }

  return 1;
}

void loop(){
  // bm forward é direita
  // fm forward é esquerda
  calc_speed2(1,0,0,PWM_X);
  delay(1000);
  while (F1.getDistance() > 20)
  {
    calc_speed2(1,0,0,PWM_X);
    delay(150);
    calc_speed2(0,0,0,0);
    delay(150);

    if(R2.getDistance() < 5 and R2I.getDistance() > 5){
      BM.turn_on_motor(FORWARD, 100);
      delay(100);

    }
    else if (R2.getDistance() < R2I.getDistance()){
      FM.turn_on_motor(FORWARD,100);
      delay(100);
    } 
  }


  bool pegou = pega_cubo();
  while(!pegou)
    pegou = pega_cubo();
  
  claw.write(CLOSED);
  delay(1000);

  calc_speed2(-1,0,0,PWM_X);
  delay(1000);

  calc_speed2(0,0,0,0);
  delay(1000);

  calc_speed2(0,-1,0,PWM_X);
  delay(3500);


  calc_speed2(1,0,0,PWM_X);
  delay(1000);


}


