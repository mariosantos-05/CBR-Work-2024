#ifndef L298N_H
#define L298N_H

#include <Arduino.h>

class L298N {
public:
    // Constructor: accepts motor direction pins only (no enable pin)
    L298N(uint8_t in1Pin, uint8_t in2Pin);

    // Initializes motor pins as outputs
    void begin();

    // Sets motor speed (0 to 255 for PWM)

    // Motor control methods
    void forward(int motorSpeed);
    void backward(int motorSpeed);
    void stop();

private:
    uint8_t in1Pin;
    uint8_t in2Pin;
    uint8_t motorSpeed;
};

#endif
