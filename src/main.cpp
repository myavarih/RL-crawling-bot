#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <Network.h>
#include <Training.h>
#include <HealthCheck.h>

// Pin definitions
const uint8_t SERVO_PIN_DOWN = 16;
const uint8_t SERVO_PIN_UP = 15;

// Global objects
Display display;
AHRS ahrs;
ServoControl servoControl(SERVO_PIN_DOWN, SERVO_PIN_UP);
Network *network;
Training training;
HealthCheck healthCheck(&display, &ahrs, &servoControl);

// Interval tracking for training display
static unsigned long lastMeasurement = 0;
static float lastPosX = 0.0f;
static float lastPosY = 0.0f;
static float lastPosZ = 0.0f;
static float speedSumCms = 0.0f;
static float accelSumMps2 = 0.0f;
static uint32_t sampleCount = 0;

static void resetIntervalTracking(unsigned long now)
{
    lastMeasurement = now;
    lastPosX = ahrs.getPositionX();
    lastPosY = ahrs.getPositionY();
    lastPosZ = ahrs.getPositionZ();
    speedSumCms = 0.0f;
    accelSumMps2 = 0.0f;
    sampleCount = 0;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    display.begin();
    display.clear();
    display.print("RL Robot V2", 0, 0);
    display.setCursor(0, 16);
    display.print("Booting...");
    display.refresh();
    delay(1000);

    network = new Network(&display);
    network->begin();
    network->startOTATask();

    int robotNum = network->getRobotNumber();
    display.clear();
    display.print("Robot ID:", 0, 0);
    display.print(robotNum, 70, 0);
    display.setCursor(0, 16);
    display.print("Setup...");
    display.refresh();
    delay(1000);

    display.clear();
    display.print("Init AHRS...", 0, 0);
    if (ahrs.begin())
    {
        display.setCursor(0, 16);
        display.print("AHRS OK");
        display.refresh();
    }
    else
    {
        display.setCursor(0, 16);
        display.print("AHRS FAIL");
        display.refresh();
        while (1)
            ; // Stop if sensor fails
    }
    delay(500);

    display.clear();
    display.print("Init Servos...", 0, 0);
    servoControl.begin();
    servoControl.moveDownSmooth(140);
    servoControl.moveUpSmooth(40);
    display.setCursor(0, 16);
    display.print("Servos OK");
    display.refresh();
    delay(500);

    display.clear();
    display.print("Calibrating...", 0, 0);
    display.setCursor(0, 16);
    display.print("Keep Still!");
    display.refresh();
    Serial.println("Calibrating...");
    ahrs.calibrate();
    delay(500);

    display.clear();
    display.print("Setup Complete", 0, 0);
    display.refresh();
    delay(1000);

    training.begin();
    training.startTraining();

    healthCheck.run();

    display.clear();
    display.setCursor(0, 0);
    display.print("Hello World");
    display.setCursor(0, 16);
    display.print("Phase 1");
    display.refresh();
    delay(1500);

    display.clear();
    display.setCursor(0, 0);
    display.print("Bye-bye");
    display.setCursor(0, 16);
    display.print("Waving...");
    display.refresh();

    servoControl.moveUpSmooth(90, 8);
    for (uint8_t i = 0; i < 3; ++i)
    {
        servoControl.moveUpSmooth(120, 8);
        delay(120);
        servoControl.moveUpSmooth(60, 8);
        delay(120);
    }
    servoControl.moveUpSmooth(90, 8);
    delay(500);

    ahrs.resetPosition();
    resetIntervalTracking(millis());
}

void loop()
{
    ahrs.update();

    unsigned long currentTime = millis();
    if (lastMeasurement == 0)
    {
        resetIntervalTracking(currentTime);
    }

    float speedCms = ahrs.getSpeed() * 100.0f;
    float accelX = ahrs.getLinearAccelX();
    float accelY = ahrs.getLinearAccelY();
    float accelZ = ahrs.getLinearAccelZ();
    float accelMag = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    speedSumCms += speedCms;
    accelSumMps2 += accelMag;
    ++sampleCount;

    float deltaTime = (currentTime - lastMeasurement) / 1000.0f;
    if (deltaTime >= 0.5f)
    {
        float posX = ahrs.getPositionX();
        float posY = ahrs.getPositionY();
        float posZ = ahrs.getPositionZ();

        float dX = posX - lastPosX;
        float dY = posY - lastPosY;
        float dZ = posZ - lastPosZ;
        float deltaDistanceCm = sqrt(dX * dX + dY * dY + dZ * dZ) * 100.0f;

        float avgSpeedCms = sampleCount ? (speedSumCms / sampleCount) : 0.0f;
        float avgAccel = sampleCount ? (accelSumMps2 / sampleCount) : 0.0f;

        bool actionChosen = false;
        Training::StepResult stepResult = {};

        if (training.isTraining())
        {
            stepResult = training.step(
                deltaDistanceCm,
                avgSpeedCms,
                avgAccel,
                servoControl.getCurrentDownAngle(),
                servoControl.getCurrentUpAngle());
            actionChosen = true;
        }

        display.clear();
        const uint8_t lineHeight = 10;

        display.setCursor(0, 0);
        display.print("Dist: ");
        display.print(String(deltaDistanceCm, 1));
        display.print(" cm");

        display.setCursor(0, lineHeight);
        display.print("Spd: ");
        display.print(String(avgSpeedCms, 1));
        display.print(" cm/s");

        display.setCursor(0, lineHeight * 2);
        display.print("Acc: ");
        display.print(String(avgAccel, 2));
        display.print(" m/s2");

        display.setCursor(0, lineHeight * 3);
        display.print("Time: ");
        display.print(String(deltaTime, 1));
        display.print(" s");

        if (actionChosen)
        {
            display.setCursor(0, lineHeight * 4);
            display.print("Act: ");
            display.print(stepResult.actionIndex);

            display.setCursor(0, lineHeight * 5);
            display.print("Reward: ");
            display.print(String(stepResult.reward, 3));
        }

        display.refresh();

        Serial.print("Distance: ");
        Serial.print(deltaDistanceCm);
        Serial.print(" cm, Speed: ");
        Serial.print(avgSpeedCms);
        Serial.print(" cm/s, Accel: ");
        Serial.print(avgAccel);
        Serial.println(" m/s2");

        if (actionChosen)
        {
            Serial.print("Action: ");
            Serial.print(stepResult.actionIndex);
            Serial.print(" tDown: ");
            Serial.print(stepResult.targetDownAngle);
            Serial.print(" tUp: ");
            Serial.print(stepResult.targetUpAngle);
            Serial.print(" reward: ");
            Serial.println(stepResult.reward, 4);

            servoControl.moveDownSmooth(stepResult.targetDownAngle);
            servoControl.moveUpSmooth(stepResult.targetUpAngle);
        }

        resetIntervalTracking(currentTime);
    }

    // TODO: Implement main loop logic
    // - Read sensors
    // - Process data
    // - Execute training if active
    // - Execute learned behavior
    // - Control servos
}
