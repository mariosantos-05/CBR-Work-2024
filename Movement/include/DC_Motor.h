#ifndef MOTORDC_H
#define MOTORDC_H

#include "Arduino.h"

// Enum for motor directions
enum Direction {
    STOP = 0,
    FORWARD = 1,
    REVERSE = -1
};

// MotorDC class to control the DC motor
class MotorDC {
public:
    MotorDC(const int ENCA, const int ENCB, const int RPWM, const int LPWM);
    void configure(int ticks_per_revolution, float kp, float ki, float kd);
    void turn_on_motor(Direction direction, int pwmVal);
    void read_encoder();
    void reset_encoder();
    void move_straight(int velocity_rpm);
    
    volatile double posi; // motor position in encoder ticks
    double rps = 0; // current speed of the motor in rotations per second
    int encoder_revolution; // encoder value for one complete revolution
    double wheel_circumference = 30.47344874; // measurement of the actual wheel radius

private:
    int ENCA; // Yellow wire
    int ENCB; // White wire
    int RPWM; // Pin to control speed in forward direction
    int LPWM; // Pin to control speed in reverse direction
    double revolutions = 0; // number of revolutions of the motor
    double previous_revolutions = 0; // previous number of revolutions
    float kp, ki, kd; // PID constants
    int rpm_reference; // desired speed
    double rpm_max = 87; // maximum speed of the motor
    float eprev = 0, eintegral = 0; // accumulated error
    Direction dir = STOP; // direction of the motor
};


#endif
