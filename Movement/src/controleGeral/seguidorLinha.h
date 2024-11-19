#ifndef SEGUIDOR_LINHA_H
#define SEGUIDOR_LINHA_H

#include "sensoresCor.h"
#include <Arduino.h>


// Definições das constantes para PID
extern const int KP;
extern const int KI;
extern const int KD;

// Definições de tolerância e valores médios para os sensores
extern const int valorMedioMeio;
extern const int valorMedioEsquerda;
extern const int valorMedioDireito;

extern const int toleranciaMeio;
extern const int toleranciaEsquerda;
extern const int toleranciaDireita;

extern const int valorMedioEsquerda_A;
extern const int valorMedioDireita_A;
extern const int valorMedioMeio_B;

extern const int intervalo_A_Esquerdo_P;
extern const int intervalo_A_Direito_P;
extern const int intervalo_B_Meio_P;

extern const int intervalo_A_Esquerdo_N;
extern const int intervalo_A_Direito_N;
extern const int intervalo_B_Meio_N;

extern const int intervaloPMeio;
extern const int intervaloNMeio;
extern const int intervaloPEsquerdo;
extern const int intervaloNEsquerdo;
extern const int intervaloPDireito;
extern const int intervaloNDireito;

// Declaração dos sensores de cor
extern SensorCorTCS230 sensorDireito;
extern SensorCorTCS230 sensorMeio;
extern SensorCorTCS230 sensorEsquerdo;

// Função que retorna o erro com base nas leituras dos sensores
int retornaErro();

// Função de controle PID
void PID();

void procurarReferenciaLateral();
void procurarReferenciaGirando_ENZ ();
void procurarReferenciaGirando_EZ ();
void primeiraVirada ();
void SegundaVirada ();
void terceiraVirada ();
int retornaSensor ();
 void limpaMedia() ;

#endif // HEADER_H
