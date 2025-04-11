/*
//This code is designed by the Iowa State Robotics for use on a RMD-X motor
//Exoskeleton control implementation
*/

// Libraries for using the CAN board
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10); // CS pin 10

// Constants
const int CAN_LED = 8;      // LED pin on CAN shield
const int HALL_SENSOR = A0; // Analog pin 0 for hall effect sensor

// Thresholds based on observed values
const float FORWARD_THRESHOLD = 3.00;  // Above this = Forward
const float BACKWARD_THRESHOLD = 2.20; // Below this = Backward

// Motor position tracking
int currentAngle = 0; // Current motor angle in degrees
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_DELAY = 1000; // Delay between commands in ms

// Function to print CAN messages for debugging
void printCANMessage(unsigned long ID, unsigned char buf[8])
{
  Serial.print(ID, HEX);
  Serial.print(": ");
  for (int i = 0; i < 8; i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print("  ");
  }
  Serial.println();
}

// Function to send CAN message and receive response
bool sendCANMessage(byte cmdByte, int value = 0)
{
  // Setup CAN message
  canMsg.can_id = 0x141; // CAN motor identifier
  canMsg.can_dlc = 0x08; // 8 bytes of data

  canMsg.data[0] = cmdByte;

  if (cmdByte == 0xA4)
  { // Position command
    // Convert angle to position value (0.01 degree per LSB)
    int32_t posValue = value * 100; // Convert to 0.01 degree units

    canMsg.data[1] = posValue & 0xFF;         // Position bytes 0-1 (LSB)
    canMsg.data[2] = (posValue >> 8) & 0xFF;  // Position bytes 0-1
    canMsg.data[3] = (posValue >> 16) & 0xFF; // Position bytes 2-3
    canMsg.data[4] = (posValue >> 24) & 0xFF; // Position bytes 2-3 (MSB)
    // Set a low max speed for safety (100 dps)
    canMsg.data[5] = 0x64; // Max speed (100 dps) low byte
    canMsg.data[6] = 0x00; // Max speed high byte
    canMsg.data[7] = 0x00; // Reserved

    Serial.print("Sending position command for angle: ");
    Serial.println(value);
  }
  else
  {
    // Default all other bytes to zero for other commands
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
  }

  // Print the command being sent
  Serial.print("Sending: ");
  printCANMessage(canMsg.can_id, canMsg.data);

  // Send the message
  bool success = mcp2515.sendMessage(&canMsg);
  if (!success)
  {
    Serial.println("ERROR: Failed to send CAN message!");
    return false;
  }
  Serial.println("CAN message sent successfully");

  // Wait for response similar to the working code
  int len = 10;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK))
  {
    /* checks for CAN response */
    delay(1);
    len--;
    if ((len <= 0))
    {
      break;
    }
  }

  if (len > 0)
  {
    // if an answer is received
    Serial.print("Received: ");
    printCANMessage(canMsg.can_id, canMsg.data);
    return true;
  }
  else
  {
    // if no answer
    Serial.println("ERROR: NO RESPONSE FROM MOTOR");
    return false;
  }
}

// Function to move motor forward one degree
bool moveForward()
{
  if (currentAngle >= 90)
  {
    Serial.println("Maximum angle reached (90°), cannot move forward");
    return false;
  }

  Serial.print("Moving forward from ");
  Serial.print(currentAngle);
  Serial.print("° to ");
  Serial.print(currentAngle + 1);
  Serial.println("°");

  // Use absolute position command (0xA4) to move to new position
  return sendCANMessage(0xA4, currentAngle + 1);
}

// Function to move motor backward one degree
bool moveBackward()
{
  if (currentAngle <= 0)
  {
    Serial.println("Minimum angle reached (0°), cannot move backward");
    return false;
  }

  Serial.print("Moving backward from ");
  Serial.print(currentAngle);
  Serial.print("° to ");
  Serial.print(currentAngle - 1);
  Serial.println("°");

  // Use absolute position command (0xA4) to move to new position
  return sendCANMessage(0xA4, currentAngle - 1);
}

// Function to stop the motor
bool stopMotor()
{
  Serial.println("Stopping motor");
  return sendCANMessage(0x81); // Stop command
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== Exoskeleton Motor Control System ===");

  // Configure pins
  pinMode(CAN_LED, OUTPUT);
  pinMode(HALL_SENSOR, INPUT);

  Serial.println("Thresholds:");
  Serial.println("BACKWARD: <2.20V");
  Serial.println("NOTHING: 2.20V-3.00V");
  Serial.println("FORWARD: >3.00V");

  // Initialize CAN controller using same settings as working code
  Serial.println("Initializing CAN controller...");
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  Serial.println("CAN controller initialized");

  // Flash LED to indicate ready
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(CAN_LED, HIGH);
    delay(100);
    digitalWrite(CAN_LED, LOW);
    delay(100);
  }

  // Stop the motor to ensure it's in a known state
  stopMotor();

  Serial.println("Setup complete!");
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
  int hallSensor = analogRead(HALL_SENSOR);

  // Print status periodically
  Serial.print("Hall Sensor: 0x");
  Serial.print(hallSensor, HEX);
  Serial.print(" -> ");
  Serial.print(hallVolt, 2);
  Serial.print("V -> ");

  if (hallVolt > FORWARD_THRESHOLD)
  {
    Serial.print("FORWARD");
  }
  else if (hallVolt < BACKWARD_THRESHOLD)
  {
    Serial.print("BACKWARD");
  }
  else
  {
    Serial.print("NEUTRAL");
  }

  Serial.print(" (Current Angle: ");
  Serial.print(currentAngle);
  Serial.println("°)");

  // Check if it's time to process a new command
  if (currentTime - lastCommandTime >= COMMAND_DELAY)
  {
    if (hallVolt > FORWARD_THRESHOLD)
    {
      // Forward motion - increment angle by 1 degree
      if (moveForward())
      {
        currentAngle++;
        digitalWrite(CAN_LED, HIGH);
      }
    }
    else if (hallVolt < BACKWARD_THRESHOLD)
    {
      // Backward motion - decrement angle by 1 degree
      if (moveBackward())
      {
        currentAngle--;
        digitalWrite(CAN_LED, HIGH);
      }
    }
    else
    {
      // No motion - turn off LED
      digitalWrite(CAN_LED, LOW);
    }

    lastCommandTime = currentTime;
  }

  // Small delay for stability
  delay(100);
}
