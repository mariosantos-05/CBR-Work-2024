#include <Servo.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

Servo myServo;
int servoPin = 9; // Pino do servo
SoftwareSerial raspy(10, 11);

String receivedData = ""; // Armazena a mensagem recebida

void setup() {
    Serial.begin(115200); // Inicializa a comunicação serial
    raspy.begin(115200);
    myServo.attach(servoPin); // Anexa o servo ao pino 9
    Serial.println("Conexão serial estabelecida.");
}

void loop() {
    // Verifica se há dados disponíveis para leitura
    if (raspy.available() > 0) {
        char incomingChar = raspy.read(); // Lê o caractere recebido

        // Armazena o caractere na string até encontrar '\n'
        if (incomingChar != '\n') {
            receivedData += incomingChar;
        } else {
            // Verifica a mensagem completa recebida
            if (receivedData == "V") {
                Serial.println("Fita detectada. Movendo o servo.");
                myServo.write(0); // Retorna o servo para a posição inicial (0 graus)
            } //else if (receivedData == "S") {
              //  myServo.write(90); // Ajusta o servo para a posição desejada (ex: 90 graus)
               // Serial.println("Fita não detectada");
            //} 
            else {
                // Converte o ID da tag recebida para um número inteiro
                int tagId = receivedData.toInt();
                if (tagId > 0) { // Verifica se foi recebido um ID válido
                    Serial.print("AprilTag detectada com ID: ");
                    Serial.println(tagId);
                    // Aqui você pode adicionar lógica específica para cada ID
                }
            }
            // Limpa a string para receber a próxima mensagem
            receivedData = "";
        }
    }
}

