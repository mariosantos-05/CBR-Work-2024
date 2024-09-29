#include <Arduino.h>

int motorPin = 13;
int motorPin3 = 11;


int motorPin2 = 12;
int motorPin4 = 10;

void setup(){
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

}

void loop(){
  analogWrite(motorPin, 255);
  analogWrite(motorPin2, 100);
  analogWrite(motorPin3, 255);
  analogWrite(motorPin4, 97);
  delay(5000);

  analogWrite(motorPin2, 255);
  analogWrite(motorPin4, 20);

  delay(4000);

  analogWrite(motorPin2, 20);
  analogWrite(motorPin4, 255);

  delay(4000);
  
  analogWrite(motorPin, 255);
  analogWrite(motorPin2, 100);
  analogWrite(motorPin3, 255);
  analogWrite(motorPin4, 97);
  delay(3000);

}