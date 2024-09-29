#include "DC_Motor.h"
#include "Arduino.h"
#include "Wire.h"

// Constructor of the MotorDC class
MotorDC::MotorDC(const int ENCA, const int ENCB, const int PWM, const int IN1, const int IN2) {
    this->ENCA = ENCA;
    this->ENCB = ENCB;
    this->PWM = PWM;
    this->IN1 = IN1;
    this->IN2 = IN2;
    pinMode(ENCA, INPUT);
    pinMode(PWM, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
}

void MotorDC::configure(int ticks_per_revolution, float kp, float ki, float kd) {
    this->encoder_revolution = ticks_per_revolution;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void MotorDC::turn_on_motor(Direction direction, int pwmVal) {
    dir = direction;
    analogWrite(PWM, pwmVal);
    digitalWrite(IN1, dir == FORWARD);
    digitalWrite(IN2, dir == REVERSE);
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
