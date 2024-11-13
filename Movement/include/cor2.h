class SensorCorTCS230 {
  private:
    int pinoS0, pinoS1, pinoS2, pinoS3, pinoOut;
    int vermelho, verde, azul;

  public:
    // Construtor que recebe os pinos como argumentos
    SensorCorTCS230(int s0, int s1, int s2, int s3, int out) 
      : pinoS0(s0), pinoS1(s1), pinoS2(s2), pinoS3(s3), pinoOut(out) {
      pinMode(pinoS0, OUTPUT);
      pinMode(pinoS1, OUTPUT);
      pinMode(pinoS2, OUTPUT);
      pinMode(pinoS3, OUTPUT);
      pinMode(pinoOut, INPUT);

      digitalWrite(pinoS0, HIGH); // Configuração inicial
      digitalWrite(pinoS1, LOW);
    }

    // Função para ler as cores e armazenar os valores nas variáveis RGB
    void lerCores() {
      // Ler vermelho
      digitalWrite(pinoS2, LOW);
      digitalWrite(pinoS3, LOW);
      vermelho = digitalRead(pinoOut) == HIGH ? pulseIn(pinoOut, LOW) : pulseIn(pinoOut, HIGH);

      // Ler azul
      digitalWrite(pinoS3, HIGH);
      azul = digitalRead(pinoOut) == HIGH ? pulseIn(pinoOut, LOW) : pulseIn(pinoOut, HIGH);

      // Ler verde
      digitalWrite(pinoS2, HIGH);
      verde = digitalRead(pinoOut) == HIGH ? pulseIn(pinoOut, LOW) : pulseIn(pinoOut, HIGH);
    }

    // Funções para retornar os valores RGB
    int getVermelho() const { return vermelho; }
    int getVerde() const { return verde; }
    int getAzul() const { return azul; }

    // Função para identificar a cor predominante
    String identificarCor() {
      if (vermelho < azul && vermelho < verde) return "VERMELHO";
      if (azul < vermelho && azul < verde) return "AZUL";
      if (verde < vermelho && verde < azul) return "VERDE";
      return "COR DESCONHECIDA";
    }
};