#include "AHRS.h"

AHRS::AHRS() : initialized(false), isStatic(true), stationaryStartTime(0)
{
    mpu = new MPU9250();
    for (int i = 0; i < 3; i++)
        velocity[i] = position[i] = linearAccel[i] = 0;
}

AHRS::~AHRS()
{
    delete mpu;
}

bool AHRS::begin()
{
    Wire.begin();
    if (!mpu->setup(0x68))
        return false;

    initialized = true;
    lastUpdateTime = millis();
    return true;
}

void AHRS::calibrate()
{
    if (!initialized)
        return;
    mpu->calibrateAccelGyro();
    mpu->calibrateMag();
}

void AHRS::update()
{
    if (!initialized || !mpu->update())
        return;

    unsigned long now = millis();
    float dt = (now - lastUpdateTime) * 0.001f;
    lastUpdateTime = now;

    if (dt <= 0 || dt > 0.1f)
        return;

    // Get raw accelerometer in body frame (units: g, convert to m/s²)
    float AccX = mpu->getAccX() * G_CONST;
    float AccY = mpu->getAccY() * G_CONST;
    float AccZ = mpu->getAccZ() * G_CONST;


    linearAccel[0] = mpu->getLinearAccX();
    linearAccel[1] = mpu->getLinearAccY();
    linearAccel[2] = mpu->getLinearAccZ();

    // Calculate magnitudes for motion detection
    float accNorm = sqrt(
        linearAccel[0] * linearAccel[0] +
        linearAccel[1] * linearAccel[1] +
        linearAccel[2] * linearAccel[2]);
    

    float gyroNorm = sqrt(
        sq(mpu->getGyroX()) +
        sq(mpu->getGyroY()) +
        sq(mpu->getGyroZ()));

    // ZUPT: Zero Velocity Update
    if (accNorm < MOTION_THRESHOLD && gyroNorm < GYRO_THRESHOLD)
    {
        if (stationaryStartTime == 0)
        {
            stationaryStartTime = now;
        }

        // If stationary for sufficient time, zero velocity
        if (now - stationaryStartTime > STATIONARY_TIME_MS)
        {
            isStatic = true;
            velocity[0] = velocity[1] = velocity[2] = 0;
            return;
        }
    }
    else
    {
        stationaryStartTime = 0;
        isStatic = false;
    }

    // 6️⃣ Integration (only when moving)
    if (!isStatic)
    {
        for (int i = 0; i < 3; i++)
        {
            velocity[i] += linearAccel[i] * dt;
            position[i] += velocity[i] * dt;
        }
    }
}

bool AHRS::isMoving()
{
    return !isStatic;
}

void AHRS::resetPosition()
{
    for (int i = 0; i < 3; i++)
        velocity[i] = position[i] = 0;
}
