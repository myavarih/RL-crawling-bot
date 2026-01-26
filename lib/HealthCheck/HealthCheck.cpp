#include "HealthCheck.h"

HealthCheck::HealthCheck(Display *display, AHRS *ahrs, ServoControl *servoControl)
    : display(display), ahrs(ahrs), servoControl(servoControl)
{
}

void HealthCheck::run()
{
    display->clear();
    display->setCursor(0, 0);
    display->print("Health Check");
    display->refresh();
    delay(500);

    // Check AHRS
    display->clear();
    display->setCursor(0, 0);
    if (ahrs->isMoving())
    {
        display->print("AHRS: Moving");
    }
    else
    {
        display->print("AHRS: Still");
    }

    // Display orientation
    display->setCursor(0, 16);
    display->print("R:");
    display->print((int)ahrs->getRoll());
    display->print(" P:");
    display->print((int)ahrs->getPitch());
    display->refresh();
    delay(500);

    // Test servos
    display->clear();
    display->setCursor(0, 0);
    display->print("Testing Servos");
    display->refresh();
    servoControl->setTestPosition();
    delay(500);

    servoControl->setInitialPosition();
    display->clear();
    display->setCursor(0, 0);
    display->print("Servos Reset");
    display->refresh();
    delay(500);

    display->clear();
    display->setCursor(0, 0);
    display->print("Health Check");
    display->setCursor(0, 16);
    display->print("Complete");
    display->refresh();
    delay(500);
    display->clear();
}
