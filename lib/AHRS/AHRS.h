#ifndef AHRS_H
#define AHRS_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

class AHRS {
public:
    AHRS();
    ~AHRS();
    bool begin();
    void update();
    
    // Motion detection
    bool isMoving();
    float getDisplacementX();
    float getDisplacementY();
    float getDisplacementZ();
    void resetDisplacement();
    
    // Orientation (Euler angles)
    float getRoll();
    float getPitch();
    float getYaw();
    
    // Accelerometer data (m/s^2)
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    
    // Gyroscope data (deg/s)
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    
    // Magnetometer data (uT)
    float getMagX();
    float getMagY();
    float getMagZ();
    
    // Quaternion
    float getQuatW();
    float getQuatX();
    float getQuatY();
    float getQuatZ();
    
    // Temperature (Celsius)
    float getTemperature();
    
    // Calibration
    void calibrateAccelGyro();
    void calibrateMag();
    bool isCalibrated();
    
private:
    MPU9250* mpu;
    bool initialized;
    bool calibrated;
    
    // For displacement tracking
    float velocityX, velocityY, velocityZ;
    float displacementX, displacementY, displacementZ;
    unsigned long lastUpdateTime;
    
    // Motion threshold
    static constexpr float MOTION_THRESHOLD = 0.1; // m/s^2
};

#endif // AHRS_H
