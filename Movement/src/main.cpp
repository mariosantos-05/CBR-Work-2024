#include <Arduino.h>
#include "DC_Motor.h"

// Motor and Encoder Pins
const int En_D = 49;   // Enable pin for right motor
const int En_E = 50;   // Enable pin for left motor
const int Rpwm_D = 13; // Right motor PWM (forward)
const int Lpwm_D = 12; // Right motor PWM (reverse)
const int Rpwm_E = 11; // Left motor PWM (forward)
const int Lpwm_E = 10; // Left motor PWM (reverse)
const int EncA_D = 18; // Right motor Encoder A
const int EncB_D = 20; // Right motor Encoder B
const int EncA_E = 19; // Left motor Encoder A
const int EncB_E = 21; // Left motor Encoder B

// Create MotorDC object for the right and left motors
MotorDC MotorR(EncA_D, EncB_D, Rpwm_D, Lpwm_D);
MotorDC MotorL(EncA_E, EncB_E, Rpwm_E, Lpwm_E);

void setup() {
    // Enable pin setup
    pinMode(En_D, OUTPUT);
    pinMode(En_E, OUTPUT);  
    digitalWrite(En_D, HIGH);
    digitalWrite(En_E, HIGH);

    // Configure the right motor with encoder parameters
    MotorR.configure(1740 , 50.0, 0.0, 0.0);  // Set encoder ticks per revolution and PID constants
    MotorL.configure(1850, 50.0, 0.0, 0.0);

    // Set up the encoder pins as inputs
    pinMode(EncA_D, INPUT);
    pinMode(EncB_D, INPUT);
    pinMode(EncA_E, INPUT);
    pinMode(EncB_E, INPUT);

    // Start Serial communication for debugging
    Serial.begin(9600);
}

void loop() {
    // Turn on motors
    MotorR.turn_on_motor(FORWARD, 100); 
    MotorL.turn_on_motor(FORWARD, 100);  
    delay(1000);
}
