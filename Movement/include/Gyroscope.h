#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Wire.h>

class Gyroscope {
private:
    const int address = 0x68;  // MPU-6050 I2C address
    int16_t gyroX, gyroY, gyroZ;  // Raw gyro data

public:
    // Constructor
    Gyroscope() {}

    // Initialize gyroscope
    void begin() {
        Wire.begin();
        Wire.beginTransmission(address);
        Wire.write(0x6B);  // Power management register
        Wire.write(0);     // Wake the sensor up
        Wire.endTransmission();
    }

    // Read gyroscope data
    void readGyro() {
        Wire.beginTransmission(address);
        Wire.write(0x43);  // Starting register for gyroscope data
        Wire.endTransmission(false);
        Wire.requestFrom(address, 6, true);  // Request 6 bytes of data

        gyroX = Wire.read() << 8 | Wire.read();  // X-axis data
        gyroY = Wire.read() << 8 | Wire.read();  // Y-axis data
        gyroZ = Wire.read() << 8 | Wire.read();  // Z-axis data
    }

    // Get methods to retrieve the angular velocity
    float getGyroX() { return gyroX / 131.0; }  // Convert raw data to dps
    float getGyroY() { return gyroY / 131.0; }
    float getGyroZ() { return gyroZ / 131.0; }
};

#endif
