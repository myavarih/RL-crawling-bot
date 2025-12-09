#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <Network.h>
#include <Training.h>

// Pin definitions
const uint8_t SERVO_PIN_DOWN = 32;
const uint8_t SERVO_PIN_UP = 33;

// Global objects
Display display;
AHRS ahrs;
ServoControl servoControl(SERVO_PIN_DOWN, SERVO_PIN_UP);
Network* network;  // Heap allocation needed for initialization order with display
Training training;

void healthCheck() {
    display.clear();
    display.print("Health Check", 0, 0);
    delay(1000);
    
    // Check AHRS
    display.clear();
    if (ahrs.isMoving()) {
        display.print("AHRS: Moving", 0, 0);
    } else {
        display.print("AHRS: Still", 0, 0);
    }
    
    // Display orientation
    display.setCursor(0, 16);
    display.print("R:");
    display.print((int)ahrs.getRoll());
    display.print(" P:");
    display.print((int)ahrs.getPitch());
    delay(1000);
    
    // Test servos
    display.clear();
    display.print("Testing Servos", 0, 0);
    servoControl.setTestPosition();
    delay(1000);
    
    servoControl.setInitialPosition();
    display.clear();
    display.print("Servos Reset", 0, 0);
    delay(1000);
    
    display.clear();
    display.print("Health Check", 0, 0);
    display.setCursor(0, 16);
    display.print("Complete");
    delay(1500);
    display.clear();
}

void setup() {
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
    if (ahrs.begin()) {
        display.setCursor(0, 16);
        display.print("AHRS OK");
    } else {
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
    healthCheck();
}

void loop() {
    // Update AHRS
    ahrs.update();
    
    // Display status
    display.clear();
    display.print("Robot #", 0, 0);
    display.print(network->getRobotNumber());
    
    display.setCursor(0, 16);
    display.print("Running...");
    
    // Display AHRS data
    display.setCursor(0, 32);
    display.print("Y:");
    display.print((int)ahrs.getYaw());
    display.print(" T:");
    display.print((int)ahrs.getTemperature());
    display.print("C");
    
    delay(1000);
    
    // TODO: Implement main loop logic
    // - Read sensors
    // - Process data
    // - Execute training if active
    // - Execute learned behavior
    // - Control servos
}

