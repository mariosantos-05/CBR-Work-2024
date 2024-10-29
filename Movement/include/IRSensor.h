#ifndef IRSensor_h
#define IRSensor_h

#include "Arduino.h"

class IRSensor {
  public:
    IRSensor(int pin);
    bool isLineDetected(); // Modificado para detectar a linha

  private:
    int _pin;
};

#endif
