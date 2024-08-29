#ifndef SENSOR_H
#define SENSOR_H
#include <Arduino.h>

class Sensor{
private:
    int trigPin;
    int echoPin;
public:
    Sensor(int trig, int echo);
    long getDistance();
};

            
#endif