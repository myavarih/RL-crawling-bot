#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoControl {
public:
    ServoControl(uint8_t pinDown, uint8_t pinUp);
    void begin();
    void moveDown(int angle);
    void moveUp(int angle);
    void moveDownSmooth(int targetAngle, int stepDelay = 10);
    void moveUpSmooth(int targetAngle, int stepDelay = 10);
    void setInitialPosition();
    void setTestPosition();
    int getCurrentDownAngle();
    int getCurrentUpAngle();

private:
    Servo servoDown;
    Servo servoUp;
    uint8_t pinDown;
    uint8_t pinUp;
    int currentDownAngle;
    int currentUpAngle;

    static const int INITIAL_DOWN_ANGLE = 0;
    static const int INITIAL_UP_ANGLE = 180;
    
    void moveServoSmooth(Servo &servo, int &currentAngle, int targetAngle, int stepDelay);
};

#endif // SERVO_CONTROL_H
