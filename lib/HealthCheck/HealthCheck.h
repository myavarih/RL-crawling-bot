#ifndef HEALTH_CHECK_H
#define HEALTH_CHECK_H

#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>

class HealthCheck
{
public:
    HealthCheck(Display *display, AHRS *ahrs, ServoControl *servoControl);
    void run();

private:
    Display *display;
    AHRS *ahrs;
    ServoControl *servoControl;
};

#endif // HEALTH_CHECK_H
