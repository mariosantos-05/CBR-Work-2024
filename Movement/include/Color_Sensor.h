#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>

class ColorSensor {
private:
    uint8_t s0Pin;
    uint8_t s1Pin;
    uint8_t s2Pin;
    uint8_t s3Pin;
    uint8_t outPin;
    
    void setFrequency(uint8_t freq);

public:
    ColorSensor(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t out);
    void begin();
    void readColor(int *red, int *green, int *blue);
};

#endif
