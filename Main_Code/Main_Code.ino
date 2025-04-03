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
const int HALL_SENSOR = A5; // Analog pin 0 for hall effect sensor
const int ANGLE_STEP = 5;   // Degrees to move per command

// Fixed thresholds based on observed values
const float CENTER_VOLTAGE = 1.30;                               // Your observed resting voltage
const float VOLTAGE_RANGE = 0.15;                                // Amount of change needed for movement
const float FORWARD_THRESHOLD = CENTER_VOLTAGE + VOLTAGE_RANGE;  // Above this = Forward
const float BACKWARD_THRESHOLD = CENTER_VOLTAGE - VOLTAGE_RANGE; // Below this = Backward

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

  Serial.println("\n=== Starting Hall Effect Sensor Test ===");
  Serial.println("Reading from analog pin A0");
  Serial.println("Format: Raw Value -> Voltage (V)");
}

void loop()
{
  // Read hall effect sensor and calculate voltage
  int hallSensor = analogRead(A0);
  float hallVolt = hallSensor * (5.0 / 1023.0);

  // Print values
  Serial.print("Raw: ");
  Serial.print(hallSensor);
  Serial.print(" -> ");
  Serial.print(hallVolt, 2);
  Serial.println("V");

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
