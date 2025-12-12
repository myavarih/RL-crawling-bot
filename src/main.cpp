#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <Network.h>
#include <Training.h>
#include <HealthCheck.h>

// Pin definitions
const uint8_t SERVO_PIN_DOWN = 15;
const uint8_t SERVO_PIN_UP = 16;

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
}

void loop()
{
    // Update AHRS
    ahrs.update();

    // Display status
    display.clear();

    // Line 1: Robot number
    display.setCursor(0, 0);
    display.print("Robot #");
    display.print(network->getRobotNumber());

    // Line 2: Status
    display.setCursor(0, 10);
    display.print("Running...");

    // Line 3: Yaw
    display.setCursor(0, 20);
    display.print("Yaw: ");
    display.print((int)ahrs.getYaw());
    display.print(" deg");

    // Line 4: Temperature
    display.setCursor(0, 30);
    display.print("Temp: ");
    display.print((int)ahrs.getTemperature());
    display.print("C");

    display.refresh();

    delay(1000);

    // TODO: Implement main loop logic
    // - Read sensors
    // - Process data
    // - Execute training if active
    // - Execute learned behavior
    // - Control servos
}
