#include "DC_Motor.h"
#include "Arduino.h"
#include "Wire.h"

// Constructor of the MotorDC class
MotorDC::MotorDC(const int ENCA, const int ENCB, const int RPWM, const int LPWM) {
    this->ENCA = ENCA;
    this->ENCB = ENCB;
    this->RPWM = RPWM;
    this->LPWM = LPWM;
    pinMode(ENCA, INPUT);
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
}

void MotorDC::configure(int ticks_per_revolution, float kp, float ki, float kd) {
    this->encoder_revolution = ticks_per_revolution;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void MotorDC::turn_on_motor(Direction direction, int pwmVal) {
    dir = direction;
    if (dir == FORWARD) {
        analogWrite(RPWM, pwmVal);  // Set PWM value on RPWM for forward motion
        analogWrite(LPWM, 0);       // Stop reverse motion by setting LPWM to 0
    } else if (dir == REVERSE) {
        analogWrite(RPWM, 0);       // Stop forward motion by setting RPWM to 0
        analogWrite(LPWM, pwmVal);  // Set PWM value on LPWM for reverse motion
    } else if (dir == STOP) {
        analogWrite(RPWM, 0);       // Stop both directions
        analogWrite(LPWM, 0);
    }
}

void MotorDC::read_encoder() {
    posi += dir; // Increment position based on direction
}

void MotorDC::reset_encoder() {
    posi = 0;
    eprev = 0;
    eintegral = 0;
    revolutions = 0;
    previous_revolutions = 0;
}

void MotorDC::move_straight(int velocity_rpm) {
    rpm_reference = velocity_rpm;

    unsigned long previous_time = millis();
    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0; // Time in seconds

    volatile double current_posi = 0;
    noInterrupts();
    current_posi = posi;
    interrupts();

    previous_revolutions = revolutions;
    revolutions = current_posi / encoder_revolution;
    rps = (revolutions - previous_revolutions) / dt;

    double e = rpm_reference - (rps * 60);
    float p = kp * e;

    eintegral += e * dt;
    float i = ki * eintegral;
    float d = kd * ((e - eprev) / dt);
    float u = p + i + d;

    int pwmVal = constrain(fabs(u), 0, 255); // Limits PWM between 0 and 255

    // Set motor direction based on value of u
    if (u > 0) {
        dir = FORWARD;
    } else if (u < 0) {
        dir = REVERSE;
    } else {
        dir = STOP;
    }

    if (velocity_rpm != 0) {
        turn_on_motor(dir, pwmVal);
    } else {
        turn_on_motor(STOP, 0);
    }

    eprev = e;
}
