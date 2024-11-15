#ifndef CINEMATICA_ROBO_H
#define CINEMATICA_ROBO_H
#include <Arduino.h>

// Declarações de funções e objetos para controle de movimento do robô
void calc_speed(float x_dot, float y_dot, float theta_dot, float scale_x_esquerdo, float scale_x_direito, float scale_y);
void calc_speedy( float y_dot, float scale_y1 , float scale_y2);
#endif
