#include "cinematicaRobo.h"
#include <BTS7960.h>


// Pinos dos motores (Cinza = Esquerdo, Roxo = Direito)
const int  Front_LPWM= 11;
const int  Front_RPWM= 10;
const int  Back_LPWM =9;
const int  Back_RPWM =8;
const int  Left_LPWM =7;
const int  Left_RPWM =6;
const int  Right_LPWM =5;
const int  Right_RPWM =4;

// Objetos dos motores
BTS7960 MotorFrente (1, Front_LPWM, Front_RPWM);
BTS7960 MotorTras (1, Back_LPWM, Back_RPWM);
BTS7960 MotorEsquerdo (1, Left_RPWM, Left_RPWM);
BTS7960 MotorDireito (1, Right_LPWM, Right_RPWM);

// Função para calcular o movimento com base no PWM
void calc_speedy( float y_dot, float scale_y1 , float scale_y2) {
  // Calcula o PWM para cada roda
  float pwmFrente   =   y_dot;
  float pwmTras     =   -y_dot;

  // Determina a direção para cada roda (1 para frente, -1 para trás, 0 para parado)
  int dirFrente   = (pwmFrente > 0) - (pwmFrente < 0);
  int dirTras     = (pwmTras > 0) - (pwmTras < 0);

  // Escala o PWM para a faixa de 0 a 255 e usa o valor absoluto
  pwmFrente   = (int)abs(pwmFrente * scale_y1);
  pwmTras     = (int)abs(pwmTras * scale_y2);

  // Aplica direção e velocidade para o motor da frente
  if (dirFrente == 1) {  // Movimento para frente
    MotorFrente.TurnRight(pwmFrente);
  } else if (dirFrente == -1) {  // Movimento para trás
    MotorFrente.TurnLeft(pwmFrente);
  } else {  // Parado
    MotorFrente.Stop();
  }

  // Aplica direção e velocidade para o motor de trás
  if (dirTras == 1) {  // Movimento para frente
    MotorTras.TurnRight(pwmTras);
  } else if (dirTras == -1) {  // Movimento para trás
    MotorTras.TurnLeft(pwmTras);
  } else {  // Parado
    MotorTras.Stop();
  }
  
}


void calc_speed(float x_dot, float y_dot, float theta_dot, float scale_x_esquerdo, float scale_x_direito, float scale_y) {
  // Calcula o PWM para cada roda
  float pwmFrente   = theta_dot + y_dot;
  float pwmDireito  = theta_dot + x_dot;
  float pwmTras     = theta_dot - y_dot;
  float pwmEsquerdo = theta_dot - x_dot;

  // Determina a direção para cada roda (1 para frente, -1 para trás, 0 para parado)
  int dirFrente   = (pwmFrente > 0) - (pwmFrente < 0);
  int dirDireito  = (pwmDireito > 0) - (pwmDireito < 0);
  int dirTras     = (pwmTras > 0) - (pwmTras < 0);
  int dirEsquerdo = (pwmEsquerdo > 0) - (pwmEsquerdo < 0);

  // Escala o PWM para a faixa de 0 a 255 e usa o valor absoluto
  pwmFrente   = (int)abs(pwmFrente * scale_y);
  pwmDireito  = (int)abs(pwmDireito * scale_x_direito);
  pwmTras     = (int)abs(pwmTras * scale_y);
  pwmEsquerdo = (int)abs(pwmEsquerdo * scale_x_esquerdo);

  // Aplica direção e velocidade para o motor da frente
  if (dirFrente == 1) {  // Movimento para frente
    MotorFrente.TurnRight(pwmFrente);
  } else if (dirFrente == -1) {  // Movimento para trás
    MotorFrente.TurnLeft(pwmFrente);
  } else {  // Parado
    MotorFrente.Stop();
  }

  // Aplica direção e velocidade para o motor da direita
  if (dirDireito == 1) {  // Movimento para frente
    MotorDireito.TurnRight(pwmDireito);
  } else if (dirDireito == -1) {  // Movimento para trás
    MotorDireito.TurnLeft(pwmDireito);
  } else {  // Parado
    MotorDireito.Stop();
  }

  // Aplica direção e velocidade para o motor de trás
  if (dirTras == 1) {  // Movimento para frente
    MotorTras.TurnRight(pwmTras);
  } else if (dirTras == -1) {  // Movimento para trás
    MotorTras.TurnLeft(pwmTras);
  } else {  // Parado
    MotorTras.Stop();
  }

  // Aplica direção e velocidade para o motor da esquerda
  if (dirEsquerdo == 1) {  // Movimento para frente
    MotorEsquerdo.TurnRight(pwmEsquerdo);
  } else if (dirEsquerdo == -1) {  // Movimento para trás
    MotorEsquerdo.TurnLeft(pwmEsquerdo);
  } else {  // Parado
    MotorEsquerdo.Stop();
  }
}
