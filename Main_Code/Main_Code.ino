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

// Joystick pins
const int JOYSTICK_X = A0; // Analog pin for X axis
const int JOYSTICK_Y = A1; // Analog pin for Y axis

void setup()
// This sets up the Can Sheild and check to make sure it is working
{
  Serial.begin(9600);
  Serial.println("Starting setup...");

  // Initialize joystick pins
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);

  // Initialize CAN controller
  Serial.println("Initializing CAN controller...");
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ); // set the CAN bus speed to 500KBPS and the frequency of the crystal oscillator to 16 MHz
  mcp2515.setNormalMode();

  // Create motor controller instance
  Serial.println("Creating motor controller...");
  motorController = new MotorController(mcp2515);

  Serial.println("Setup complete!");
  Serial.println("Waiting for joystick input...");
  Serial.println("0 = backward, 1 = stop, 2 = forward");
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

int getJoystickCommand()
{
  // Read analog value (0-1023)
  int yValue = analogRead(JOYSTICK_Y);

  Serial.print("Raw Joystick Y: ");
  Serial.println(yValue);

  // Convert to command (0, 1, or 2)
  if (yValue < 400)
  {
    return 0; // Backward
  }
  else if (yValue > 600)
  {
    return 2; // Forward
  }
  else
  {
    return 1; // Stop
  }
}

void loop()
{
  static unsigned long lastDebugTime = 0;
  static unsigned long lastJoystickRead = 0;
  unsigned long currentTime = millis();

  // Read joystick every 50ms
  if (currentTime - lastJoystickRead >= 50)
  {
    int command = getJoystickCommand();
    Serial.print("Joystick Command: ");
    Serial.println(command);

    // Send command to motor controller
    switch (command)
    {
    case 0: // Backward
      motorController->moveBackward();
      break;
    case 2: // Forward
      motorController->moveForward();
      break;
    default: // Stop
      motorController->stop();
      break;
    }

    lastJoystickRead = currentTime;
  }

  // Print debug info every second
  if (currentTime - lastDebugTime > 1000)
  {
    Serial.print("Current angle: ");
    Serial.println(motorController->getCurrentAngle());
    lastDebugTime = currentTime;
  }

  // Small delay to prevent overwhelming the CAN bus
  delay(10);
}
