#include "sensoresCor.h"

// Construtor que inicializa os pinos e configurações do sensor
SensorCorTCS230::SensorCorTCS230(int s0, int s1, int s2, int s3, int out)
    : pinoS0(s0), pinoS1(s1), pinoS2(s2), pinoS3(s3), pinoOut(out) {
    pinMode(pinoS0, OUTPUT);
    pinMode(pinoS1, OUTPUT);
    pinMode(pinoS2, OUTPUT);
    pinMode(pinoS3, OUTPUT);
    pinMode(pinoOut, INPUT);

    digitalWrite(pinoS0, HIGH);  // Configuração inicial
    digitalWrite(pinoS1, LOW);
}

// Função para ler a cor vermelha e armazenar o valor
void SensorCorTCS230::lerCores() {
    // Ler vermelho
    digitalWrite(pinoS2, LOW);
    digitalWrite(pinoS3, LOW);
    vermelho = pulseIn(pinoOut, LOW);  // Lê o valor do vermelho
}

// Função para retornar o valor do vermelho
int SensorCorTCS230::getVermelho() {
  return vermelho;
}
