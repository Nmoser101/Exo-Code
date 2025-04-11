/*
//This code is designed by the Iowa State Robotics for use on a RMD-X motor
//Exoskeleton control implementation with troubleshooting
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

// Testing variables - try different command approaches
int commandApproach = 0; // 0 = position, 1 = speed, 2 = torque, 3 = motor read
bool responseReceived = false;
int retryCount = 0;
const int MAX_RETRIES = 3;
int motorId = 0x141; // Try different motor IDs: 0x141, 0x140, 0x142, etc.
bool tryDifferentIds = true;
unsigned long lastIdChangeTime = 0;
const unsigned long ID_CHANGE_INTERVAL = 10000; // 10 seconds

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
bool sendCANMessage(byte cmdByte, int32_t value = 0)
{
  // Setup CAN message
  canMsg.can_id = motorId; // CAN motor identifier (changeable)
  canMsg.can_dlc = 0x08;   // 8 bytes of data

  // Zero out all data bytes by default
  for (int i = 0; i < 8; i++)
  {
    canMsg.data[i] = 0x00;
  }

  Serial.print("Command type: 0x");
  Serial.print(cmdByte, HEX);
  Serial.print(" - ");

  if (cmdByte == 0x4A || cmdByte == 0xA4)
  { // Position command - try both formats
    // According to whiteboard example:
    // 0 - 0x4A - command
    // 1 - 0x00 - null
    // 2-3 - 0xF4, 0x01 - target speed (500 dps)
    // 4 - 0xA6 - position byte 1
    // 5 - 0x86 - position byte 2
    // 6 - 0x00 - position byte 3
    // 7 - 0x00 - position byte 4

    // Convert angle to position value (0.01 degree per LSB)
    int32_t posValue = value * 100; // Convert to 0.01 degree units

    // Using exact format from whiteboard
    canMsg.data[0] = 0x4A;                   // Command byte - EXACT VALUE FROM WHITEBOARD
    canMsg.data[1] = 0x00;                   // Null
    canMsg.data[2] = 0xF4;                   // Speed low byte (500 dps)
    canMsg.data[3] = 0x01;                   // Speed high byte
    canMsg.data[4] = posValue & 0xFF;        // Position byte 1 (LSB)
    canMsg.data[5] = (posValue >> 8) & 0xFF; // Position byte 2
    canMsg.data[6] = 0x00;                   // Position byte 3
    canMsg.data[7] = 0x00;                   // Position byte 4 (MSB)

    Serial.print("Position command to ");
    Serial.print(value);
    Serial.println("° using whiteboard format with 0x4A command");
  }
  else if (cmdByte == 0xF4)
  { // Speed command - using whiteboard format
    // Set speed in dps (degrees per second)
    int32_t speedValue = value; // 1 dps/LSB

    // From whiteboard: 0x??/0x01 speed values
    canMsg.data[0] = 0xF4; // Command based on whiteboard
    canMsg.data[1] = 0x00;
    canMsg.data[2] = speedValue & 0xFF;        // Speed low byte
    canMsg.data[3] = (speedValue >> 8) & 0xFF; // Speed high byte
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;

    Serial.print("Speed command at ");
    Serial.print(value);
    Serial.println(" dps using whiteboard format");
  }
  else if (cmdByte == 0xA1)
  { // Torque command
    // From protocol: Torque command format (0.01A/LSB)
    int16_t torqueValue = value;

    canMsg.data[0] = 0xA1;                      // Command byte
    canMsg.data[1] = torqueValue & 0xFF;        // Torque low byte
    canMsg.data[2] = (torqueValue >> 8) & 0xFF; // Torque high byte
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;

    Serial.print("Torque command at ");
    Serial.print(value);
    Serial.println(" (0.01A)");
  }
  else if (cmdByte == 0x81)
  { // Stop command from whiteboard
    // All zeros for the stop command per protocol
    canMsg.data[0] = 0x81;
    // Rest already initialized to 0
    Serial.println("Stop command");
  }
  else if (cmdByte == 0x9A)
  { // Read motor status 1
    canMsg.data[0] = 0x9A;
    Serial.println("Read motor status command");
  }
  else if (cmdByte == 0x60)
  { // Read encoder position
    canMsg.data[0] = 0x60;
    Serial.println("Read encoder position command");
  }
  else
  {
    canMsg.data[0] = cmdByte; // Use whatever command byte was provided
    Serial.println("Other command");
  }

  // Print the command being sent and motor ID
  Serial.print("Sending to Motor ID 0x");
  Serial.print(motorId, HEX);
  Serial.print(": ");
  printCANMessage(canMsg.can_id, canMsg.data);

  // Send the message with multiple attempts if needed
  bool success = false;
  for (int attempt = 0; attempt < 3; attempt++)
  {
    success = mcp2515.sendMessage(&canMsg);
    if (success)
      break;
    delay(10); // Small delay between attempts
  }

  if (!success)
  {
    Serial.println("ERROR: Failed to send CAN message after 3 attempts!");
    return false;
  }

  Serial.println("CAN message sent successfully");

  // Wait for response with longer timeout
  int len = 30; // Increased timeout further
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK))
  {
    delay(1);
    len--;
    if (len <= 0)
    {
      break;
    }
  }

  if (len > 0)
  {
    // if an answer is received
    Serial.print("Received: ");
    printCANMessage(canMsg.can_id, canMsg.data);
    responseReceived = true;
    retryCount = 0; // Reset retry counter on success
    return true;
  }
  else
  {
    // if no answer
    Serial.println("ERROR: NO RESPONSE FROM MOTOR");
    responseReceived = false;
    retryCount++;
    return false;
  }
}

// Read status from the motor
bool readMotorStatus()
{
  Serial.println("Reading motor status");
  return sendCANMessage(0x9A); // Motor status command
}

// Read encoder position from the motor
bool readEncoderPosition()
{
  Serial.println("Reading encoder position");
  return sendCANMessage(0x60); // Read encoder position
}

// Function to send a motor stop command
bool stopMotor()
{
  Serial.println("Stopping motor");
  return sendCANMessage(0x81); // Stop command
}

// Try different approaches to move the motor forward
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

  bool result = false;

  // Trying different approaches in sequence if previous ones fail
  switch (commandApproach)
  {
  case 0: // Position approach - using exact whiteboard command
    Serial.println("Approach: Position control (0x4A)");
    result = sendCANMessage(0x4A, currentAngle + 1);
    break;

  case 1: // Speed approach - using exact whiteboard command
    Serial.println("Approach: Speed control (0xF4)");
    result = sendCANMessage(0xF4, 10); // 10 dps forward
    break;

  case 2: // Torque approach
    Serial.println("Approach: Torque control");
    result = sendCANMessage(0xA1, 100); // Small positive torque
    break;

  case 3: // Read motor status approach
    Serial.println("Approach: Read motor status");
    result = readMotorStatus();
    break;

  case 4: // Read encoder position
    Serial.println("Approach: Read encoder position");
    result = readEncoderPosition();
    break;
  }

  return result;
}

// Try different approaches to move the motor backward
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

  bool result = false;

  // Trying different approaches in sequence if previous ones fail
  switch (commandApproach)
  {
  case 0: // Position approach - using exact whiteboard command
    Serial.println("Approach: Position control (0x4A)");
    result = sendCANMessage(0x4A, currentAngle - 1);
    break;

  case 1: // Speed approach - using exact whiteboard command
    Serial.println("Approach: Speed control (0xF4)");
    result = sendCANMessage(0xF4, -10); // 10 dps backward
    break;

  case 2: // Torque approach
    Serial.println("Approach: Torque control");
    result = sendCANMessage(0xA1, -100); // Small negative torque
    break;

  case 3: // Read motor status approach
    Serial.println("Approach: Read motor status");
    result = readMotorStatus();
    break;

  case 4: // Read encoder position
    Serial.println("Approach: Read encoder position");
    result = readEncoderPosition();
    break;
  }

  return result;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== Exoskeleton Motor Control System ===");
  Serial.println("Using Whiteboard Commands");

  // Configure pins
  pinMode(CAN_LED, OUTPUT);
  pinMode(HALL_SENSOR, INPUT);

  Serial.println("Thresholds:");
  Serial.println("BACKWARD: <2.20V");
  Serial.println("NOTHING: 2.20V-3.00V");
  Serial.println("FORWARD: >3.00V");

  // Initialize CAN controller
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

  // Use fixed motor ID 0x141 and position control to start
  motorId = 0x141;
  commandApproach = 0;     // Position control
  tryDifferentIds = false; // Start with a fixed ID to test exact whiteboard format

  Serial.println("Setup complete!");
  Serial.print("Using Motor ID: 0x");
  Serial.println(motorId, HEX);
  Serial.println("Starting with POSITION CONTROL (0x4A) approach");

  // Send initial stop command
  Serial.println("\n--- INITIAL STOP COMMAND ---");
  stopMotor();
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

  // Try different motor IDs if enabled
  if (tryDifferentIds && (currentTime - lastIdChangeTime >= ID_CHANGE_INTERVAL))
  {
    // Cycle through IDs: 0x140, 0x141, 0x142, 0x143
    motorId = (motorId >= 0x143) ? 0x140 : motorId + 1;

    Serial.print("\n*** SWITCHING TO MOTOR ID 0x");
    Serial.print(motorId, HEX);
    Serial.println(" ***\n");

    // Reset with a read status command when changing IDs
    readMotorStatus();

    lastIdChangeTime = currentTime;
    retryCount = 0; // Reset retry count on ID change
  }

  // Read hall effect sensor with averaging for stability
  float hallVolt = getHallVoltage();
  int hallSensor = analogRead(HALL_SENSOR);

  // Print status information
  Serial.print("Hall: 0x");
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

  Serial.print(" (Angle: ");
  Serial.print(currentAngle);
  Serial.print("°, Approach: ");
  Serial.print(commandApproach);
  Serial.print(", ID: 0x");
  Serial.print(motorId, HEX);
  Serial.println(")");

  // Check if it's time to process a new command
  if (currentTime - lastCommandTime >= COMMAND_DELAY)
  {
    // Change approach if we've failed multiple times
    if (retryCount >= MAX_RETRIES)
    {
      commandApproach = (commandApproach + 1) % 5; // Now using 5 approaches
      Serial.print("\n*** SWITCHING TO APPROACH ");
      Serial.print(commandApproach);
      Serial.println(" ***\n");
      retryCount = 0;
    }

    bool result = false;

    if (hallVolt > FORWARD_THRESHOLD)
    {
      // Forward motion - increment angle by 1 degree
      result = moveForward();
      if (result && responseReceived)
      {
        currentAngle++;
        digitalWrite(CAN_LED, HIGH);
      }
    }
    else if (hallVolt < BACKWARD_THRESHOLD)
    {
      // Backward motion - decrement angle by 1 degree
      result = moveBackward();
      if (result && responseReceived)
      {
        currentAngle--;
        digitalWrite(CAN_LED, HIGH);
      }
    }
    else
    {
      // No motion - read motor status
      if (commandApproach >= 3)
      { // Only for read approaches
        result = readMotorStatus();
      }
      digitalWrite(CAN_LED, LOW);
    }

    lastCommandTime = currentTime;
  }

  // Small delay for stability
  delay(100);
}
