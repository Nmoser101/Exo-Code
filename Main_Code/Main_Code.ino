/*
//This code is designed by the Iowa State Robotics for use on a RMD-X motor
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
const int ANGLE_STEP = -5;  // Move -5 degrees per command

// Thresholds based on observed values
const float CENTER_VOLTAGE = 1.94;     // Observed center voltage
const float FORWARD_THRESHOLD = 3.20;  // Above this = Forward
const float BACKWARD_THRESHOLD = 1.60; // Below this = Backward

// Motor position tracking
int currentPosition = 1; // 1 = stop, 2 = forward, 3 = backward
int currentAngle = 0;    // Current motor angle in degrees
unsigned long lastPositionChange = 0;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Configure pins
  pinMode(CAN_LED, OUTPUT);
  pinMode(HALL_SENSOR, INPUT);

  Serial.println("\n=== Hall Effect Motor Control ===");
  Serial.println("Thresholds:");
  Serial.println("BACKWARD: <2.20V");
  Serial.println("NOTHING: 2.20V-3.00V");
  Serial.println("FORWARD: >3.00V");

  // Initialize CAN controller
  Serial.println("\nInitializing CAN controller...");
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  // Create motor controller instance
  Serial.println("Creating motor controller...");
  motorController = new MotorController(mcp2515);

  Serial.println("\nSetup complete!");
}

void loop()
{
  // Read hall effect sensor and calculate voltage
  int hallSensor = analogRead(HALL_SENSOR);
  float hallVolt = hallSensor * (5.0 / 1023.0);

  // Determine position and control motor
  String position;
  if (hallVolt > 3.00)
  {
    position = "FORWARD";
    motorController->moveMotorTo(90); // Move to max angle
    digitalWrite(CAN_LED, HIGH);
  }
  else if (hallVolt < 2.20)
  {
    position = "BACKWARD";
    motorController->moveMotorTo(0); // Move to min angle
    digitalWrite(CAN_LED, HIGH);
  }
  else
  {
    position = "NOTHING";
    motorController->moveMotorTo(45); // Move to middle position
    digitalWrite(CAN_LED, LOW);
  }

  // Print status
  Serial.print("Raw: 0x");
  Serial.print(hallSensor, HEX);
  Serial.print(" -> ");
  Serial.print(hallVolt, 2);
  Serial.print("V -> ");
  Serial.print(position);
  Serial.print(" (Angle: ");
  Serial.print(motorController->getCurrentAngle());
  Serial.println("°)");

  delay(500); // Wait 500ms between readings
}

// Sets up so that Can's 8 bit is proccessed correctly
void printserial(unsigned long ID, unsigned char buf[8])
{
  Serial.print(ID, HEX);
  Serial.print(": ");
  for (int i = 0; i < 8; i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print("  ");
  }
}
