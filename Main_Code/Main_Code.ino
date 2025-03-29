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
  // Basic Serial test
  Serial.begin(115200);
  delay(3000); // Long delay to ensure Serial is ready

  // Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Clear the Serial buffer
  while (Serial.available())
  {
    Serial.read();
  }

  Serial.println("\n\n");
  Serial.println("=====================================");
  Serial.println("        System Starting Up          ");
  Serial.println("=====================================");

  // Configure analog reference
  analogReference(DEFAULT); // Use 5V as reference

  // Initialize joystick pins
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // Use built-in LED for visual feedback

  // Test joystick connection
  Serial.println("\nTesting Joystick Connection...");
  digitalWrite(LED_BUILTIN, HIGH);

  // Take multiple readings with longer delays
  long total = 0;
  int readings = 0;
  int maxReading = 0;
  int minReading = 1023;

  for (int i = 0; i < 20; i++)
  {
    delay(100); // Longer delay between readings
    int reading = analogRead(JOYSTICK_Y);

    // Update min/max
    if (reading < minReading)
      minReading = reading;
    if (reading > maxReading)
      maxReading = reading;

    // Only include "stable" readings
    if (reading > 0 && reading < 1023)
    {
      total += reading;
      readings++;
    }

    Serial.print(i);
    Serial.print(": ");
    Serial.print(reading);
    Serial.print(" (");
    Serial.print(5.0 * reading / 1023.0, 2);
    Serial.println("V)");
  }

  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("\nJoystick Analysis:");
  Serial.print("Average reading: ");
  Serial.println(readings > 0 ? total / readings : 0);
  Serial.print("Min reading: ");
  Serial.println(minReading);
  Serial.print("Max reading: ");
  Serial.println(maxReading);
  Serial.print("Valid readings: ");
  Serial.print(readings);
  Serial.print(" out of 20 (");
  Serial.print(readings * 5);
  Serial.println("%)");

  if (readings < 15)
  {
    Serial.println("\n!!! WARNING !!!");
    Serial.println("Joystick readings are unstable!");
    Serial.println("Please check:");
    Serial.println("1. All connections are secure");
    Serial.println("2. No loose wires");
    Serial.println("3. Power supply is stable");
    Serial.println("4. Connections:");
    Serial.println("   - VCC → 5V");
    Serial.println("   - GND → GND");
    Serial.println("   - VRy → A1");
  }

  // Initialize CAN controller
  Serial.println("Initializing CAN controller...");
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ); // set the CAN bus speed to 500KBPS and the frequency of the crystal oscillator to 16 MHz
  mcp2515.setNormalMode();

  // Create motor controller instance
  Serial.println("Creating motor controller...");
  motorController = new MotorController(mcp2515);

  // Print the full analog range once during setup to help calibrate
  printJoystickRange();

  Serial.println("Setup complete!");
  Serial.println("Waiting for joystick input...");
  Serial.println("0 = backward, 1 = stop, 2 = forward");
  Serial.println("=== Setup Complete ===\n");

  Serial.println("\n=== Setup Complete ===");
  Serial.println("Monitoring joystick values...");
  Serial.println("Format: Raw Value (Voltage)");
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
  // Read analog value multiple times and average for stability
  int sum = 0;
  const int samples = 5;

  for (int i = 0; i < samples; i++)
  {
    int reading = analogRead(JOYSTICK_Y);
    sum += reading;
    Serial.print("Raw reading ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(reading);
    delay(50); // Increased delay between readings
  }
  int yValue = sum / samples;

  Serial.print("Averaged Joystick Y: ");
  Serial.println(yValue);

  // Adjusted thresholds based on your actual joystick range
  if (yValue < 350)
  { // Below center
    Serial.println("Command: BACKWARD (0)");
    return 0; // Backward
  }
  else if (yValue > 400)
  { // Above center
    Serial.println("Command: FORWARD (2)");
    return 2; // Forward
  }
  else
  {
    Serial.println("Command: STOP (1)");
    return 1; // Stop - middle range
  }
}

// Print the full analog range once during setup to help calibrate
void printJoystickRange()
{
  Serial.println("\n=== Joystick Calibration ===");
  Serial.println("Move joystick through its full range...");

  int minVal = 1023;
  int maxVal = 0;

  // Read for 5 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 5000)
  {
    int val = analogRead(JOYSTICK_Y);
    minVal = min(minVal, val);
    maxVal = max(maxVal, val);

    Serial.print("Current: ");
    Serial.print(val);
    Serial.print(" | Min: ");
    Serial.print(minVal);
    Serial.print(" | Max: ");
    Serial.println(maxVal);

    delay(100);
  }

  Serial.println("\n=== Calibration Results ===");
  Serial.print("Minimum value: ");
  Serial.println(minVal);
  Serial.print("Maximum value: ");
  Serial.println(maxVal);
  Serial.println("=========================\n");
}

void loop()
{
  // Read joystick
  int rawValue = analogRead(JOYSTICK_Y);
  float voltage = (rawValue * 5.0) / 1023.0;

  // Print values
  Serial.print("Joystick: ");
  Serial.print(rawValue);
  Serial.print(" (");
  Serial.print(voltage, 2);
  Serial.println("V)");

  // Visual feedback on LED if value is too low
  if (rawValue < 50)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

  delay(200); // Read 5 times per second
}
