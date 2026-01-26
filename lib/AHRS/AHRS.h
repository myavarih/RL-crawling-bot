#ifndef AHRS_H
#define AHRS_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <math.h>


class AHRS
{
public:
    AHRS();
    ~AHRS();
    bool begin();
    void update();
    void calibrate();

    // Motion detection
    bool isMoving();
    void resetPosition();

    // Position (m)
    float getPositionX() { return position[0]; }
    float getPositionY() { return position[1]; }
    float getPositionZ() { return position[2]; }

    // Velocity (m/s)
    float getVelocityX() { return velocity[0]; }
    float getVelocityY() { return velocity[1]; }
    float getVelocityZ() { return velocity[2]; }
    float getSpeed() { return sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]); }

    // Linear acceleration (m/s^2) - gravity removed
    float getLinearAccelX() { return linearAccel[0]; }
    float getLinearAccelY() { return linearAccel[1]; }
    float getLinearAccelZ() { return linearAccel[2]; }

    // Orientation (Euler angles)
    float getRoll() { return initialized ? mpu->getRoll() : 0; }
    float getPitch() { return initialized ? mpu->getPitch() : 0; }
    float getYaw() { return initialized ? mpu->getYaw() : 0; }

    // Accelerometer data (g)
    float getAccelX() { return initialized ? mpu->getAccX() * G_CONST : 0; }
    float getAccelY() { return initialized ? mpu->getAccY() * G_CONST : 0; }
    float getAccelZ() { return initialized ? mpu->getAccZ() * G_CONST : 0; }

    // Gyroscope data (deg/s)
    float getGyroX() { return initialized ? mpu->getGyroX() : 0; }
    float getGyroY() { return initialized ? mpu->getGyroY() : 0; }
    float getGyroZ() { return initialized ? mpu->getGyroZ() : 0; }

    // Magnetometer data (uT)
    float getMagX() { return initialized ? mpu->getMagX() : 0; }
    float getMagY() { return initialized ? mpu->getMagY() : 0; }
    float getMagZ() { return initialized ? mpu->getMagZ() : 0; }

    // Quaternion
    float getQuatW() { return initialized ? mpu->getQuaternionW() : 1; }
    float getQuatX() { return initialized ? mpu->getQuaternionX() : 0; }
    float getQuatY() { return initialized ? mpu->getQuaternionY() : 0; }
    float getQuatZ() { return initialized ? mpu->getQuaternionZ() : 0; }

    // Temperature (Celsius)
    float getTemperature() { return initialized ? mpu->getTemperature() : 0; }

private:
    MPU9250 *mpu;
    bool initialized;
    bool isStatic;
    unsigned long stationaryStartTime;
    unsigned long lastUpdateTime;

    float velocity[3];    // m/s
    float position[3];    // m
    float linearAccel[3]; // m/s^2 (gravity removed)

    // Constants
    static constexpr float G_CONST = 9.80665f;                // m/s^2
    static constexpr float MOTION_THRESHOLD = 0.1f;          // m/s^2 - linear accel threshold
    static constexpr float GYRO_THRESHOLD = 10.0f;           // deg/s - rotation threshold
    static constexpr unsigned long STATIONARY_TIME_MS = 200; // ms - time to confirm stationary
};

#endif // AHRS_H
