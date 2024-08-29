#include "Color_Sensor.h"

ColorSensor::ColorSensor(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t out)
    : s0Pin(s0), s1Pin(s1), s2Pin(s2), s3Pin(s3), outPin(out) {}

void ColorSensor::begin() {
    pinMode(s0Pin, OUTPUT);
    pinMode(s1Pin, OUTPUT);
    pinMode(s2Pin, OUTPUT);
    pinMode(s3Pin, OUTPUT);
    pinMode(outPin, INPUT);
    
    // Set frequency to 20% (or change as needed)
    setFrequency(2);
}

void ColorSensor::setFrequency(uint8_t freq) {
    digitalWrite(s0Pin, freq & 0x01);
    digitalWrite(s1Pin, (freq >> 1) & 0x01);
}

void ColorSensor::readColor(int *red, int *green, int *blue) {
    // Set sensor to read red
    digitalWrite(s2Pin, LOW);
    digitalWrite(s3Pin, LOW);
    delay(100); // Allow sensor to settle
    *red = pulseIn(outPin, LOW);
    
    // Set sensor to read green
    digitalWrite(s2Pin, HIGH);
    digitalWrite(s3Pin, HIGH);
    delay(100); // Allow sensor to settle
    *green = pulseIn(outPin, LOW);
    
    // Set sensor to read blue
    digitalWrite(s2Pin, LOW);
    digitalWrite(s3Pin, HIGH);
    delay(100); // Allow sensor to settle
    *blue = pulseIn(outPin, LOW);
}
