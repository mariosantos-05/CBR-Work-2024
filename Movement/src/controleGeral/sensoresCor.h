#ifndef SENSORES_COR_H
#define SENSORES_COR_H

#include <Arduino.h>

class SensorCorTCS230 {
  public:
    // Construtor que inicializa os pinos e configurações do sensor
    SensorCorTCS230(int s0, int s1, int s2, int s3, int out);

    // Função para ler a cor vermelha e armazenar o valor
    void lerCores();

    // Função para retornar o valor do vermelho
    int getVermelho();

  private:
    int pinoS0, pinoS1, pinoS2, pinoS3, pinoOut;
    int vermelho;  // Armazenar o valor do vermelho
};

#endif  // Fim da proteção contra múltiplas inclusões
