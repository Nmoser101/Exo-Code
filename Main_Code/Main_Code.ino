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
const int ANGLE_STEP = 5;   // Degrees to move per command

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

  // Configure pin
  pinMode(A0, INPUT);

  Serial.println("\n=== Hall Effect Sensor Test ===");
  Serial.println("Format: Raw (Hex) -> Voltage (V) -> Position");
}

void loop()
{
  // Read hall effect sensor and calculate voltage
  int hallSensor = analogRead(A0);
  float hallVolt = hallSensor * (5.0 / 1023.0);

  // Determine position
  String position;
  if (hallVolt > 3.20)
  {
    position = "FORWARD";
  }
  else if (hallVolt < 2.00)
  {
    position = "BACKWARD";
  }
  else
  {
    position = "NOTHING";
  }

  // Print values in hex
  Serial.print("Raw: 0x");
  Serial.print(hallSensor, HEX);
  Serial.print(" -> ");
  Serial.print(hallVolt, 2);
  Serial.print("V -> ");
  Serial.println(position);

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
