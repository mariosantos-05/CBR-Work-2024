#include "IRSensor.h"

IRSensor::IRSensor(int pin) {
  _pin = pin;
  pinMode(_pin, INPUT); 
}

bool IRSensor::isLineDetected() {
  return digitalRead(_pin) == LOW; // Retorna true se a linha (escura) for detectada
}
