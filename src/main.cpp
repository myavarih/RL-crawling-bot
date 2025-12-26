#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <Network.h>
#include <Training.h>
#include <HealthCheck.h>

// Pin definitions
const uint8_t SERVO_PIN_DOWN = 16; // P1 on board
const uint8_t SERVO_PIN_UP = 15;   // P2 on board

// Global objects
Display display;
AHRS ahrs;
ServoControl servoControl(SERVO_PIN_DOWN, SERVO_PIN_UP);
Network *network; // Heap allocation needed for initialization order with display
Training training;
HealthCheck healthCheck(&display, &ahrs, &servoControl);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Initialize Display
    display.begin();
    display.clear();
    display.print("RL Robot V2", 0, 0);
    display.setCursor(0, 16);
    display.print("Initializing...");
    delay(1000);

    // Initialize Network
    network = new Network(&display);
    network->begin();
    network->startOTATask();

    // Display robot info
    display.clear();
    display.print("Robot #", 0, 0);
    display.print(network->getRobotNumber());
    display.setCursor(0, 16);
    display.print("Setup...");
    delay(1000);

    // Initialize AHRS
    display.clear();
    display.print("Init AHRS...", 0, 0);
    if (ahrs.begin())
    {
        display.setCursor(0, 16);
        display.print("AHRS OK");
    }
    else
    {
        display.setCursor(0, 16);
        display.print("AHRS Failed!");
    }
    delay(1000);

    // Initialize Servos
    display.clear();
    display.print("Init Servos...", 0, 0);
    servoControl.begin();
    display.setCursor(0, 16);
    display.print("Servos OK");
    delay(1000);

    // Initialize Training
    training.begin();

    // Setup complete
    display.clear();
    display.print("Setup Complete", 0, 0);
    delay(2000);

    // Run health check
    healthCheck.run();

    // Phase 1 demo: Hello World + bye-bye wave
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

    // Reset measurement for interval tracking
    ahrs.resetMeasurement();
}

void loop()
{
    // Update AHRS
    ahrs.update();

    // ===== MODE 1: Real-time continuous display =====
    // Uncomment this section to show current instantaneous values
    /*
    display.clear();

    // Line 1: Speed (cm/s)
    display.setCursor(0, 0);
    display.print("Spd: ");
    display.print(ahrs.getSpeed() * 100, 1);
    display.print(" cm/s");

    // Line 2: Acceleration (m/s^2)
    display.setCursor(0, 12);
    display.print("Acc: ");
    display.print(ahrs.getAccelMagnitude(), 2);
    display.print(" m/s2");

    // Line 3: Displacement X (cm)
    display.setCursor(0, 24);
    display.print("X: ");
    display.print(ahrs.getDisplacementX() * 100, 1);
    display.print(" cm");

    // Line 4: Displacement Y (cm)
    display.setCursor(0, 36);
    display.print("Y: ");
    display.print(ahrs.getDisplacementY() * 100, 1);
    display.print(" cm");

    display.refresh();
    delay(1000);
    */

    // ===== MODE 2: Interval measurement (every 2 seconds) =====
    // This shows average movement parameters since last measurement
    static unsigned long lastMeasurement = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastMeasurement >= 2000)
    { // Every 2 seconds
        lastMeasurement = currentTime;

        // Get measurement data
        AHRS::MovementSnapshot measurement = ahrs.getMeasurement();

        // Display the interval data
        display.clear();

        // Line 1: Distance moved in interval (cm)
        display.setCursor(0, 0);
        display.print("Dist: ");
        display.print(measurement.deltaDistance, 1);
        display.print(" cm");

        // Line 2: Average speed in interval (cm/s)
        display.setCursor(1, 0);
        display.print("Spd: ");
        display.print(measurement.avgSpeed, 1);
        display.print(" cm/s");

        // Line 3: Average acceleration in interval (m/s^2)
        display.setCursor(2, 0);
        display.print("Acc: ");
        display.print(measurement.avgAcceleration, 2);
        display.print(" m/s2");

        // Line 4: Time interval
        display.setCursor(3, 0);
        display.print("Time: ");
        display.print(measurement.deltaTime, 1);
        display.print(" s");

        display.refresh();

        // Reset for next measurement
        ahrs.resetMeasurement();

        // Print to serial for debugging
        Serial.print("Distance: ");
        Serial.print(measurement.deltaDistance);
        Serial.print(" cm, Speed: ");
        Serial.print(measurement.avgSpeed);
        Serial.print(" cm/s, Accel: ");
        Serial.print(measurement.avgAcceleration);
        Serial.println(" m/s2");
    }

    // TODO: Implement main loop logic
    // - Read sensors
    // - Process data
    // - Execute training if active
    // - Execute learned behavior
    // - Control servos
}
