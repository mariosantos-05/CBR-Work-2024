#include "cinematicaRobo.h"
#include "sensoresCor.h"
#include <Arduino.h>
#include "RunningAverage.h"


const int KP = 2;
const int KI = 0;
const int KD = 0;

const int toleranciaMeio = 15;
const int toleranciaEsquerda = 5;
const int toleranciaDireita = 5;

const String desvio = "indefinido";
const int valorMedioMeio = 50;
const int valorMedioEsquerda = 44;
const int valorMedioDireito = 33;

RunningAverage mediaSensorFrente(3);  // Janela de 10 leituras
RunningAverage mediaSensorEsquerdo(3);  // Janela de 10 leituras
RunningAverage mediaSensorDireito(3);  // Janela de 10 leituras

//valor azul sensores direita e esquerdo

const int valorMedioEsquerda_A = 201;
const int valorMedioDireita_A = 60;

//valro branco sensor meio

const int valorMedioMeio_B = 18;

//Intervalos novos

const int intervalo_A_Esquerdo_N = valorMedioEsquerda_A - 20;
const int intervalo_A_Direito_N = valorMedioDireita_A - 5;
const int intervalo_B_Meio_N = valorMedioMeio_B - 5;

const int intervalo_A_Esquerdo_P = valorMedioEsquerda_A + 20;
const int intervalo_A_Direito_P = valorMedioDireita_A + 5;
const int intervalo_B_Meio_P = valorMedioMeio_B + 5;


const int intervaloPMeio = valorMedioMeio + toleranciaMeio;
const int intervaloNMeio = valorMedioMeio - toleranciaMeio;
const int intervaloPEsquerdo = valorMedioEsquerda + toleranciaEsquerda;
const int intervaloNEsquerdo = valorMedioEsquerda - toleranciaEsquerda;
const int intervaloPDireito = valorMedioDireito + toleranciaDireita;
const int intervaloNDireito = valorMedioDireito - toleranciaDireita;

SensorCorTCS230 sensorDireito(49, 51, 45, 43, 47);
SensorCorTCS230 sensorMeio(40, 38, 34, 42, 36);
SensorCorTCS230 sensorEsquerdo(52, 50, 48, 44, 46);

int retornaErro() {

    for (int i = 0; i < 5; i++) {  // Realiza 5 leituras consecutivas
        sensorMeio.lerCores();
        sensorDireito.lerCores();
        sensorEsquerdo.lerCores();

        mediaSensorFrente.addValue(sensorMeio.getVermelho());
        mediaSensorEsquerdo.addValue(sensorEsquerdo.getVermelho());
        mediaSensorDireito.addValue(sensorDireito.getVermelho());
    }

    Serial.println(mediaSensorEsquerdo.getAverage() );

    // Verifica a posição do robô com relação à linha
    if (mediaSensorFrente.getAverage() >= intervaloNMeio && mediaSensorFrente.getAverage() <= intervaloPMeio) {
        return 0;  // Robô centralizado na linha
    } else if (mediaSensorEsquerdo.getAverage()  > intervaloPEsquerdo) {
        return 60;  // Erro para a esquerda
    } else if (mediaSensorDireito.getAverage() > intervaloPDireito) {
        return -60;  // Erro para a direita
    } else {
        //procurarReferenciaLateral();
        return 0;  // Erro crítico ou linha perdida
    }
}


void PID() {
  static int erro_anterior = 0;
  static int somatoriaErro = 0;
  const int pwmInicial = 60;  // Valor inicial de PWM (ajuste conforme necessário)

  // Calcula o erro com base nas leituras dos sensores
  int erro = retornaErro();


  somatoriaErro += erro;
  int erroDerivativo = erro - erro_anterior;
 

  // Ajuste de PID
  int ajustePID = (int)(KP * erro + KI * somatoriaErro + KD * erroDerivativo);

   erro_anterior = erro;

  if (ajustePID > 255) {
    ajustePID = 255;
  } else if  (ajustePID <=0) {
    ajustePID = 0;
  }

  // Controle dos motores com o ajuste calculado
  if (erro == 0) {
    calc_speed(1, 0, 0, pwmInicial, pwmInicial, 0);  // Robô centralizado
  } else if (erro == 60) {
    calc_speed(1, 0, 0, pwmInicial-ajustePID, pwmInicial + ajustePID, 0);  // Corrige para a direita
  } else if (erro == -60) {
    calc_speed(1, 0, 0, pwmInicial + ajustePID, pwmInicial - ajustePID, 0);  // Corrige para a esquerda
  }
}


void procurarReferenciaLateral() {
  int media = -10;
  while (media <= intervaloNMeio || media >= intervaloPMeio) {

    for (int i = 0; i <= 4; i++) {
      sensorMeio.lerCores();
      media = sensorMeio.getVermelho() + media;
    }
    media = media / 4;
    Serial.println(media);

    if (media >= intervaloNMeio && media <= intervaloPMeio) {
      break;
    } else {
      calc_speed(0, 1, 0, 0, 0, 60);
      delay(100);

      calc_speed(0, 0, 0, 0, 0, 0);
      delay(500);
    }
    media = 0;
  }
}

void procurarReferenciaGirando_EZ() {
  
  mediaSensorFrente.clear();
  mediaSensorEsquerdo.clear();
  mediaSensorDireito.clear();


  Serial.println(mediaSensorFrente.getAverage());

  while (sensorMeio.getVermelho() <= intervaloNMeio || sensorMeio.getVermelho()  >= intervaloPMeio) {
    Serial.println("entrou");
    Serial.println(mediaSensorFrente.getAverage());
    for (int i = 0; i <= 5; i++) {
      sensorMeio.lerCores();
      mediaSensorFrente.addValue(sensorMeio.getVermelho());
    }

    if (mediaSensorFrente.getAverage() >= intervaloNMeio && mediaSensorFrente.getAverage()  <= intervaloPMeio) {
      calc_speed(0, 0, 0, 0, 0, 0);
      delay(100000);
      break;
    } else {
      calc_speed(0, 0, 1, 60, 60, 60);
      delay(100);
      calc_speed(0, 0, 0, 0, 0, 0);
      delay(1000);
    }
  }
}

void procurarReferenciaGirando_ENZ() {
  
  mediaSensorFrente.clear();
  mediaSensorEsquerdo.clear();
  mediaSensorDireito.clear();


  Serial.println(mediaSensorFrente.getAverage());

  while (sensorMeio.getVermelho() <= intervaloNMeio || sensorMeio.getVermelho()  >= intervaloPMeio) {
    Serial.println("entrou");
    Serial.println(mediaSensorFrente.getAverage());
    for (int i = 0; i <= 5; i++) {
      sensorMeio.lerCores();
      mediaSensorFrente.addValue(sensorMeio.getVermelho());
    }

    if (mediaSensorFrente.getAverage() >= intervaloNMeio && mediaSensorFrente.getAverage()  <= intervaloPMeio) {
      calc_speed(0, 0, 0, 0, 0, 0);
      delay(100000);
      break;
    } else {
      calc_speed(0, 0, -1, 60, 60, 60);
      delay(100);
      calc_speed(0, 0, 0, 0, 0, 0);
      delay(1000);
    }
  }
}


/*struct Erros {
    int erroRodaEsquerda;
    int erroRodaDireita;
    int erroAlinhamentoEsquerdo;
    int erroAlinhamentoDireito;
};

Erros alinharParede_Erros() {
    Erros erros;

    // Cálculo dos erros para o ajuste das rodas individualmente
    erros.erroRodaEsquerda = leituraSensor1 - dist;
    erros.erroRodaDireita = leituraSensor2 - dist;

    // Erros adicionais para garantir que o robô permaneça alinhado
    if (leituraSensor1 > dist + tolerancia) {
        erros.erroAlinhamentoEsquerdo = 10;  // Ajuste fino para roda esquerda
    } else if (leituraSensor1 < dist - tolerancia) {
        erros.erroAlinhamentoEsquerdo = -10;
    } else {
        erros.erroAlinhamentoEsquerdo = 0;
    }

    if (leituraSensor2 > dist + tolerancia) {
        erros.erroAlinhamentoDireito = 10;  // Ajuste fino para roda direita
    } else if (leituraSensor2 < dist - tolerancia) {
        erros.erroAlinhamentoDireito = -10;
    } else {
        erros.erroAlinhamentoDireito = 0;
    }

    return erros;
}

void alinharParedeControl() {

    Erros erro = alinharParede_Erros();
    int kp = 1;
    int ki = 0.5;
    int kd = 0;

    static int erro_direito = 
    int pidEsquerda = kp * erro.erroRodaEsquerda + ki * ();
    int pidDireita = kp * erro.erroRodaDireita + ki * ();

    // Controla cada roda baseado nos erros de alinhamento e distância
    if (erro.erroAlinhamentoEsquerdo == 0 && erro.erroAlinhamentoDireito == 0) {
        calc_speedy(1, 0, 0);  // Nenhum ajuste necessário
    } else {
        // Ajuste conforme os erros de alinhamento
        if (erro.erroAlinhamentoEsquerdo == 10) {
            calc_speedy(1, pidEsquerda, 0);
        } else if (erro.erroAlinhamentoEsquerdo == -10) {
            calc_speedy(1, -pidEsquerda, 0);
        }
      
        if (erro.erroAlinhamentoDireito == 10) {
            calc_speedy(1, 0, pidDireita);
        } else if (erro.erroAlinhamentoDireito == -10) {
            calc_speedy(1, 0, -pidDireita);
        }
    }
}*/

