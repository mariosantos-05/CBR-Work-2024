#include "DC_Motor.h"

MotorDC* MotorDC::instance = nullptr;

MotorDC::MotorDC(int p1, int p2, int en, int encA, int encB)
    : pin1(p1), pin2(p2), enablepin(en), encoderPinA(encA), encoderPinB(encB), encoderPosition(0), targetPosition(0) {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(enablepin, OUTPUT);
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);

    // Set the instance pointer to this object
    instance = this;

    // Attach interrupt service routines for the encoder pins
    attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoderB, CHANGE);
}

void MotorDC::Backward() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(enablepin, 255); // Full speed
}

void MotorDC::Forward() {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(enablepin, 255); // Full speed
}

void MotorDC::Stop() {
    analogWrite(enablepin, 0); // Stop the motor
}

void MotorDC::Turn(float angle) {
    // Assuming one revolution equals 360 degrees
    long countsPerRevolution = 1440; // This value depends on your encoder
    long targetCounts = (angle / 360.0) * countsPerRevolution;

    targetPosition = encoderPosition + targetCounts;

    if (targetCounts > 0) {
        Forward();
    } else {
        Backward();
    }

    // Run the motor until the target position is reached
    while (abs(targetPosition - encoderPosition) > 1) {
        // Optionally include a small delay to prevent excessive CPU usage
        delay(10);
    }

    Stop();
}

void MotorDC::updateEncoderA() {
    if (instance) {
        instance->updateEncoder();
    }
}

void MotorDC::updateEncoderB() {
    if (instance) {
        instance->updateEncoder();
    }
}

void MotorDC::updateEncoder() {
    static uint8_t lastEncoded = 0;
    uint8_t MSB = digitalRead(encoderPinA); // Most significant bit
    uint8_t LSB = digitalRead(encoderPinB); // Least significant bit

    uint8_t encoded = (MSB << 1) | LSB; // Convert the 2 pin value to single value
    uint8_t sum = (lastEncoded << 2) | encoded; // Add the previous encoded value

    // Determine the direction based on the encoder sequence
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;

    lastEncoded = encoded; // Store this value for next time
}

void MotorDC::updatePosition() {
    // This method can be called periodically to update the encoder position
    updateEncoder();
}
