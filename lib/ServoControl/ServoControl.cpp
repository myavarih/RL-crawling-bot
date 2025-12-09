#include "ServoControl.h"

ServoControl::ServoControl(uint8_t pinDown, uint8_t pinUp) 
    : pinDown(pinDown), pinUp(pinUp), 
      currentDownAngle(INITIAL_DOWN_ANGLE), 
      currentUpAngle(INITIAL_UP_ANGLE) {
}

void ServoControl::begin() {
    servoDown.attach(pinDown, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    servoUp.attach(pinUp, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    setInitialPosition();
}

void ServoControl::moveDown(int angle) {
    angle = constrain(angle, 0, 180);
    servoDown.write(angle);
    currentDownAngle = angle;
}

void ServoControl::moveUp(int angle) {
    angle = constrain(angle, 0, 180);
    servoUp.write(angle);
    currentUpAngle = angle;
}

void ServoControl::moveDownSmooth(int targetAngle, int stepDelay) {
    targetAngle = constrain(targetAngle, 0, 180);
    moveServoSmooth(servoDown, currentDownAngle, targetAngle, stepDelay);
}

void ServoControl::moveUpSmooth(int targetAngle, int stepDelay) {
    targetAngle = constrain(targetAngle, 0, 180);
    moveServoSmooth(servoUp, currentUpAngle, targetAngle, stepDelay);
}

void ServoControl::setInitialPosition() {
    moveDown(INITIAL_DOWN_ANGLE);
    moveUp(INITIAL_UP_ANGLE);
}

void ServoControl::setTestPosition() {
    moveDown(90);
    moveUp(90);
}

int ServoControl::getCurrentDownAngle() {
    return currentDownAngle;
}

int ServoControl::getCurrentUpAngle() {
    return currentUpAngle;
}

void ServoControl::moveServoSmooth(Servo &servo, int &currentAngle, int targetAngle, int stepDelay) {
    if (currentAngle < targetAngle) {
        for (int angle = currentAngle; angle <= targetAngle; angle += 2) {
            servo.write(angle);
            delay(stepDelay);
        }
    } else {
        for (int angle = currentAngle; angle >= targetAngle; angle -= 2) {
            servo.write(angle);
            delay(stepDelay);
        }
    }
    // Ensure we reach the exact target angle
    servo.write(targetAngle);
    currentAngle = targetAngle;
}
