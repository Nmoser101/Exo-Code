/*
//This code is designed by the Iowa State Robotics for use on a RMD-X motor
//Exoskeleton control implementation
*/

// Libraries for using the CAN board
#include <SPI.h>
#include <mcp2515.h>
#include "MotorController.h"

struct can_frame canMsg;
MCP2515 mcp2515(10); // CS pin 10
MotorController *motorController;

// Constants
const int CAN_LED = 8;      // LED pin on CAN shield
const int HALL_SENSOR = A0; // Analog pin 0 for hall effect sensor

// Thresholds based on observed values
const float FORWARD_THRESHOLD = 3.00;  // Above this = Forward
const float BACKWARD_THRESHOLD = 2.20; // Below this = Backward

// Motor position tracking
int currentAngle = 0; // Current motor angle in degrees
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_DELAY = 500; // Delay between commands in ms

// For debugging
int failedCommandCount = 0;
const int MAX_FAILED_COMMANDS = 10;
bool motorActive = true;
unsigned long lastSerialPrint = 0;
const unsigned long SERIAL_PRINT_INTERVAL = 1000; // Print diagnostic info every 1 second

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== Exoskeleton Motor Control System ===");
  Serial.println("Based on RMD-X Motor Protocol");

  // Configure pins
  pinMode(CAN_LED, OUTPUT);
  pinMode(HALL_SENSOR, INPUT);

  Serial.println("\nThresholds:");
  Serial.println("BACKWARD: <2.20V");
  Serial.println("NOTHING: 2.20V-3.00V");
  Serial.println("FORWARD: >3.00V");

  // Initialize CAN controller
  Serial.println("\nInitializing CAN controller...");
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  Serial.println("CAN controller initialized");

  // Create motor controller instance
  Serial.println("Creating motor controller...");
  motorController = new MotorController(mcp2515);

  // Flash LED to indicate ready
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(CAN_LED, HIGH);
    delay(100);
    digitalWrite(CAN_LED, LOW);
    delay(100);
  }

  Serial.println("\nSetup complete!");
}

// Get average of multiple hall sensor readings for stability
float getHallVoltage()
{
  const int numReadings = 3;
  int total = 0;

  for (int i = 0; i < numReadings; i++)
  {
    total += analogRead(HALL_SENSOR);
    delay(5);
  }

  float avgReading = total / (float)numReadings;
  return avgReading * (5.0 / 1023.0);
}

void loop()
{
  // Get current time for timing operations
  unsigned long currentTime = millis();

  // Read hall effect sensor with averaging for stability
  float hallVolt = getHallVoltage();

  // Print diagnostic info at regular intervals rather than every loop
  if (currentTime - lastSerialPrint >= SERIAL_PRINT_INTERVAL)
  {
    // Get raw reading for diagnostic output
    int hallSensor = analogRead(HALL_SENSOR);

    // Print status information
    Serial.print("\n----- STATUS -----\n");
    Serial.print("Raw Hall: 0x");
    Serial.print(hallSensor, HEX);
    Serial.print(" -> ");
    Serial.print(hallVolt, 2);
    Serial.print("V");

    if (hallVolt > FORWARD_THRESHOLD)
    {
      Serial.println(" (FORWARD)");
    }
    else if (hallVolt < BACKWARD_THRESHOLD)
    {
      Serial.println(" (BACKWARD)");
    }
    else
    {
      Serial.println(" (NEUTRAL)");
    }

    Serial.print("Current Angle: ");
    Serial.print(currentAngle);
    Serial.print("° (Motor reports: ");
    Serial.print(motorController->getCurrentAngle());
    Serial.println("°)");

    Serial.print("Motor Active: ");
    Serial.println(motorActive ? "YES" : "NO");

    Serial.print("Failed Commands: ");
    Serial.println(failedCommandCount);

    lastSerialPrint = currentTime;
  }

  // Check if motor is active and time to process a command
  if (motorActive && currentTime - lastCommandTime >= COMMAND_DELAY)
  {
    if (hallVolt > FORWARD_THRESHOLD)
    {
      // FORWARD: Move 1 degree forward
      int targetAngle = currentAngle + 1;

      // Ensure we stay within bounds
      if (targetAngle <= 90)
      {
        Serial.print("\nMoving FORWARD from ");
        Serial.print(currentAngle);
        Serial.print("° to ");
        Serial.println(targetAngle);

        // Try direct torque control instead of position
        bool result = motorController->moveForward();

        if (result)
        {
          currentAngle = targetAngle;
          digitalWrite(CAN_LED, HIGH);
          failedCommandCount = 0; // Reset counter on success
        }
        else
        {
          failedCommandCount++;
          Serial.println("WARNING: Forward movement failed");
        }
      }
      else
      {
        Serial.println("\nMaximum angle reached (90°)");
      }

      lastCommandTime = currentTime;
    }
    else if (hallVolt < BACKWARD_THRESHOLD)
    {
      // BACKWARD: Move 1 degree backward
      int targetAngle = currentAngle - 1;

      // Ensure we stay within bounds
      if (targetAngle >= 0)
      {
        Serial.print("\nMoving BACKWARD from ");
        Serial.print(currentAngle);
        Serial.print("° to ");
        Serial.println(targetAngle);

        // Try direct torque control instead of position
        bool result = motorController->moveBackward();

        if (result)
        {
          currentAngle = targetAngle;
          digitalWrite(CAN_LED, HIGH);
          failedCommandCount = 0; // Reset counter on success
        }
        else
        {
          failedCommandCount++;
          Serial.println("WARNING: Backward movement failed");
        }
      }
      else
      {
        Serial.println("\nMinimum angle reached (0°)");
      }

      lastCommandTime = currentTime;
    }
    else
    {
      // NEUTRAL: No movement
      digitalWrite(CAN_LED, LOW);
    }

    // Safety check - if too many commands fail, stop trying
    if (failedCommandCount >= MAX_FAILED_COMMANDS)
    {
      motorActive = false;
      Serial.println("\n!!! CRITICAL ERROR !!!");
      Serial.println("Too many failed commands. Motor control disabled.");
      Serial.println("Reset the system to try again.");

      // Rapid flash pattern to indicate error
      for (int i = 0; i < 10; i++)
      {
        digitalWrite(CAN_LED, HIGH);
        delay(50);
        digitalWrite(CAN_LED, LOW);
        delay(50);
      }
    }
  }

  // Small delay for stability
  delay(10);
}

// Helper function to print CAN messages for debugging
void printCANMessage(unsigned long ID, unsigned char buf[8], const char *prefix)
{
  Serial.print(prefix);
  Serial.print(" CAN ID: 0x");
  Serial.print(ID, HEX);
  Serial.print(" Data: ");
  for (int i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (buf[i] < 0x10)
      Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
