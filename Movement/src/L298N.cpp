#include "L298N.h"

// Constructor to initialize the motor direction pins
L298N::L298N(uint8_t in1Pin, uint8_t in2Pin)
    : in1Pin(in1Pin), in2Pin(in2Pin) {}

// Initializes the motor pins as outputs
void L298N::begin() {
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
}


// Drives the motor forward using PWM on IN1 and LOW on IN2
void L298N::forward(int motorSpeed) {
    analogWrite(in1Pin, motorSpeed); // PWM signal on IN1
    digitalWrite(in2Pin, LOW);       // Set IN2 to LOW
}

// Drives the motor backward using PWM on IN2 and LOW on IN1
void L298N::backward(int motorSpeed) {
    digitalWrite(in1Pin, LOW);       // Set IN1 to LOW
    analogWrite(in2Pin, motorSpeed); // PWM signal on IN2
}

// Stops the motor by setting both IN1 and IN2 to LOW
void L298N::stop() {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
}
