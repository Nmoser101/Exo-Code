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
const int CAN_LED = 8;                // LED pin on CAN shield
const int HALL_SENSOR = A5;           // Analog pin 5 on CAN bus shield
const float FORWARD_THRESHOLD = 2.0;  // Above 2.0V = Forward
const float BACKWARD_THRESHOLD = 1.5; // Below 1.5V = Backward
const int ANGLE_STEP = 5;             // Degrees to move per command

// Motor position tracking
int currentPosition = 1; // 1 = stop, 2 = forward, 3 = backward
int currentAngle = 0;    // Current motor angle in degrees

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Configure pins
  pinMode(CAN_LED, OUTPUT);
  pinMode(HALL_SENSOR, INPUT);

  Serial.println("\n=== Starting Motor Control Program ===");

  // Initialize CAN controller
  Serial.println("Initializing CAN controller...");
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  // Create motor controller instance
  Serial.println("Creating motor controller...");
  motorController = new MotorController(mcp2515);

  Serial.println("Setup complete!");
  Serial.println("Waiting for hall effect input...");
  Serial.println("Thresholds: <1.5V=Backward, >2.0V=Forward");
}

void loop()
{
  // Read hall effect sensor
  int sensorValue = analogRead(HALL_SENSOR);
  float voltage = sensorValue * (5.0 / 1023.0);

  // Determine position based on voltage
  int newPosition;
  if (voltage > FORWARD_THRESHOLD)
  {
    newPosition = 2; // Forward
  }
  else if (voltage < BACKWARD_THRESHOLD)
  {
    newPosition = 3; // Backward
  }
  else
  {
    newPosition = 1; // Stop
  }

  // Print status
  Serial.println("\n--- Status ---");
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.print("V -> Position: ");
  switch (newPosition)
  {
  case 1:
    Serial.println("STOP");
    break;
  case 2:
    Serial.println("FORWARD");
    break;
  case 3:
    Serial.println("BACKWARD");
    break;
  }

  // Only act if position changed
  if (newPosition != currentPosition)
  {
    currentPosition = newPosition;

    switch (currentPosition)
    {
    case 2: // Forward
      if (currentAngle < 90)
      {
        currentAngle += ANGLE_STEP;
        Serial.print("Moving forward to angle: ");
        Serial.println(currentAngle);
        motorController->moveForward();
      }
      break;

    case 3: // Backward
      if (currentAngle > 0)
      {
        currentAngle -= ANGLE_STEP;
        Serial.print("Moving backward to angle: ");
        Serial.println(currentAngle);
        motorController->moveBackward();
      }
      break;

    case 1: // Stop
      Serial.println("Stopping motor");
      motorController->stop();
      break;
    }
  }

  // Visual feedback on LED
  digitalWrite(CAN_LED, currentPosition != 1); // LED on when moving

  delay(100); // Small delay between readings
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
