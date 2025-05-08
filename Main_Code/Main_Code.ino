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

// New voltage thresholds for different rates
const float THRESHOLD_1 = 1.0; // 0-1V: 3 degrees
const float THRESHOLD_2 = 2.0; // 1-2V: 1 degree
const float THRESHOLD_3 = 3.0; // 2-3V: no change
const float THRESHOLD_4 = 4.0; // 3-4V: 1 degree
// 4-5V: 3 degrees

// Motor position tracking
int currentAngle = 35;     // Current motor angle in degrees, start at 35 (middle of -10 to 80 range)
int referenceAngle = -10;  // Reference angle for relative positioning
bool referenceSet = false; // Flag to track if reference position has been set
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

int incomingByte = 1; // Default to STOP

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
  if (currentAngle >= 80)
  {
    Serial.println("Maximum angle reached (80°), cannot move forward");
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
  if (currentAngle <= -10)
  {
    Serial.println("Minimum angle reached (-10°), cannot move backward");
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

void setup()
{
  while (!Serial)
    ;
  Serial.begin(115200);

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

  // Set initial position to 35 degrees
  Serial.println("\n--- SETTING INITIAL POSITION TO 35 DEGREES ---");
  canMsg.can_id = motorId;
  canMsg.can_dlc = 0x08;
  canMsg.data[0] = 0x4A; // Position command
  canMsg.data[1] = 0x00; // Null
  canMsg.data[2] = 0xF4; // Speed low byte (500 dps)
  canMsg.data[3] = 0x01; // Speed high byte

  // Set position to 35 degrees (3500 in 0.01 degree units)
  int32_t initialPos = 3500;
  canMsg.data[4] = initialPos & 0xFF;         // Position byte 1 (LSB)
  canMsg.data[5] = (initialPos >> 8) & 0xFF;  // Position byte 2
  canMsg.data[6] = (initialPos >> 16) & 0xFF; // Position byte 3
  canMsg.data[7] = (initialPos >> 24) & 0xFF; // Position byte 4 (MSB)

  mcp2515.sendMessage(&canMsg);
  delay(2000); // Wait longer for motor to reach initial position

  Serial.println("Setup done");
  Serial.println("Send 1=STOP, 2=FORWARD, 3=BACKWARD via Serial Monitor");
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
  float hallVolt = getHallVoltage();
  int hallSensor = analogRead(HALL_SENSOR);

  Serial.print("Hall: 0x");
  Serial.print(hallSensor, HEX);
  Serial.print(" -> ");
  Serial.print(hallVolt, 2);
  Serial.print("V -> ");

  canMsg.can_id = 0x141;
  canMsg.can_dlc = 0x08;

  // Determine rate of change based on voltage range
  int rateOfChange = 0;

  if (hallVolt < THRESHOLD_1)
  {
    // 0-1V: 3 degrees backward
    rateOfChange = -3;
    Serial.print("BACKWARD (3°)");
  }
  else if (hallVolt < THRESHOLD_2)
  {
    // 1-2V: 1 degree backward
    rateOfChange = -1;
    Serial.print("BACKWARD (1°)");
  }
  else if (hallVolt < THRESHOLD_3)
  {
    // 2-3V: no change
    rateOfChange = 0;
    Serial.print("NEUTRAL");
  }
  else if (hallVolt < THRESHOLD_4)
  {
    // 3-4V: 1 degree forward
    rateOfChange = 1;
    Serial.print("FORWARD (1°)");
  }
  else
  {
    // 4-5V: 3 degrees forward
    rateOfChange = 3;
    Serial.print("FORWARD (3°)");
  }

  // Update current angle with bounds checking
  int newAngle = currentAngle + rateOfChange;
  if (newAngle >= -10 && newAngle <= 80)
  {
    currentAngle = newAngle;
  }

  // Calculate position value (0.01 degree per LSB)
  int32_t posValue = currentAngle * 100;
  canMsg.data[0] = 0xA4;                    // Command byte for position control
  canMsg.data[1] = 0x00;                    // NULL
  canMsg.data[2] = 0xF4;                    // Speed limit low byte (500dps)
  canMsg.data[3] = 0x01;                    // Speed limit high byte (500dps)
  canMsg.data[4] = posValue & 0xFF;         // Position byte 1 (LSB)
  canMsg.data[5] = (posValue >> 8) & 0xFF;  // Position byte 2
  canMsg.data[6] = (posValue >> 16) & 0xFF; // Position byte 3
  canMsg.data[7] = (posValue >> 24) & 0xFF; // Position byte 4 (MSB)

  // LED feedback
  if (rateOfChange != 0)
  {
    digitalWrite(CAN_LED, HIGH);
  }
  else
  {
    digitalWrite(CAN_LED, LOW);
  }

  Serial.print(" | Target Angle: ");
  Serial.print(currentAngle);
  Serial.println("°");

  mcp2515.sendMessage(&canMsg);
  printserial(canMsg.can_id, canMsg.data);
  Serial.println();

  // Wait for response
  int len = 10;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK))
  {
    delay(1);
    len--;
    if (len <= 0)
      break;
  }
  if (len > 0)
  {
    Serial.print("Recv: ");
    printserial(canMsg.can_id, canMsg.data);
    Serial.println();
  }
  else
  {
    Serial.println("Recv: NO ANSWER");
  }

  delay(100);
}
