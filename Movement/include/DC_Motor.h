#ifndef MOTORDC_H
#define MOTORDC_H

#include <Arduino.h>

class MotorDC {
private:
    int pin1, pin2;       // Direction pins
    int enablepin;        // PWM pin for speed control
    int encoderPinA, encoderPinB; // Encoder pins
    volatile long encoderPosition; // Current position
    long targetPosition; // Target position for movement
    static MotorDC* instance;

    void updateEncoder();

public:
    MotorDC(int p1, int p2, int en, int encA, int encB);
    void Backward();
    void Forward();
    void Stop();
    void Turn(float angle); // Angle in degrees
    void updatePosition();
    static void updateEncoderA();
    static void updateEncoderB();
};

#endif
