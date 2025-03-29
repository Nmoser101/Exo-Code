#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

class MotorController
{
private:
    // Motor constraints
    static const int MIN_ANGLE = 0;
    static const int MAX_ANGLE = 90;
    static const int MOVEMENT_DELAY_MS = 5000; // Reduced to 500ms for more responsive movement

    // Motor state
    int currentAngle;
    unsigned long lastMovementTime;
    MCP2515 &mcp2515;
    bool isMoving;

    // CAN message structure
    struct can_frame canMsg;

    // Helper functions
    bool isWithinBounds(int angle)
    {
        bool valid = angle >= MIN_ANGLE && angle <= MAX_ANGLE;
        if (!valid)
        {
            Serial.print("Angle out of bounds: ");
            Serial.println(angle);
        }
        return valid;
    }

    void printCANMessage(const char *prefix)
    {
        Serial.print(prefix);
        Serial.print(" CAN ID: 0x");
        Serial.print(canMsg.can_id, HEX);
        Serial.print(" Data: ");
        for (int i = 0; i < 8; i++)
        {
            Serial.print("0x");
            if (canMsg.data[i] < 0x10)
                Serial.print("0");
            Serial.print(canMsg.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    bool sendMotorCommand(int command)
    {
        Serial.print("\n--- Sending Motor Command: 0x");
        Serial.println(command, HEX);

        canMsg.can_id = 0x141;
        canMsg.can_dlc = 0x08;

        switch (command)
        {
        case 0x81: // Stop command
            Serial.println("Command Type: STOP");
            canMsg.data[0] = 0x81;
            canMsg.data[1] = 0x00;
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            isMoving = false;
            break;

        case 0x82: // Forward command
            Serial.println("Command Type: FORWARD");
            canMsg.data[0] = 0xA1; // Changed to match RMD-X protocol
            canMsg.data[1] = 0x00;
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            isMoving = true;
            break;

        case 0x83: // Backward command
            Serial.println("Command Type: BACKWARD");
            canMsg.data[0] = 0xA2; // Changed to match RMD-X protocol
            canMsg.data[1] = 0x00;
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            isMoving = true;
            break;

        default: // Position command (0-90)
            Serial.println("Command Type: POSITION");
            Serial.print("Target angle: ");
            Serial.println(command);
            canMsg.data[0] = 0xA4;           // Changed to match RMD-X protocol
            canMsg.data[1] = command & 0xFF; // Target position (0-90)
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            break;
        }

        printCANMessage("Sending");

        // Send command
        bool success = mcp2515.sendMessage(&canMsg);
        if (!success)
        {
            Serial.println("ERROR: Failed to send CAN message!");
            return false;
        }
        Serial.println("CAN message sent successfully");

        // Wait for response
        int len = 10;
        bool gotResponse = false;
        while (len > 0)
        {
            if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
            {
                printCANMessage("Received");
                gotResponse = true;
                break;
            }
            delay(1);
            len--;
        }

        if (!gotResponse)
        {
            Serial.println("ERROR: No response received from motor!");
            return false;
        }

        return true;
    }

public:
    MotorController(MCP2515 &canController)
        : mcp2515(canController), currentAngle(0), lastMovementTime(0), isMoving(false)
    {
        Serial.println("MotorController initialized");
        // Initialize motor to starting position
        moveMotorTo(0);
    }

    void moveForward()
    {
        unsigned long currentTime = millis();
        if (currentTime - lastMovementTime < MOVEMENT_DELAY_MS)
        {
            Serial.println("Waiting for movement delay...");
            return;
        }

        if (isWithinBounds(currentAngle + 1))
        {
            Serial.println("Moving forward");
            if (sendMotorCommand(0x82))
            { // Send forward command
                currentAngle++;
                lastMovementTime = currentTime;
                Serial.print("Angle increased to: ");
                Serial.println(currentAngle);
            }
        }
        else
        {
            stop(); // Stop if we hit the limit
        }
    }

    void moveBackward()
    {
        unsigned long currentTime = millis();
        if (currentTime - lastMovementTime < MOVEMENT_DELAY_MS)
        {
            Serial.println("Waiting for movement delay...");
            return;
        }

        if (isWithinBounds(currentAngle - 1))
        {
            Serial.println("Moving backward");
            if (sendMotorCommand(0x83))
            { // Send backward command
                currentAngle--;
                lastMovementTime = currentTime;
                Serial.print("Angle decreased to: ");
                Serial.println(currentAngle);
            }
        }
        else
        {
            stop(); // Stop if we hit the limit
        }
    }

    void stop()
    {
        if (isMoving)
        {
            Serial.println("Stopping motor");
            if (sendMotorCommand(0x81))
            { // Send stop command
                isMoving = false;
                Serial.println("Motor stopped");
            }
        }
    }

    // Function to move motor to specific angle
    void moveMotorTo(int targetAngle)
    {
        Serial.print("Moving to angle: ");
        Serial.println(targetAngle);

        if (!isWithinBounds(targetAngle))
        {
            Serial.println("Target angle out of bounds, ignoring command");
            return; // Safety check
        }

        // Send command to move to target angle
        if (sendMotorCommand(targetAngle))
        {
            // Update current angle
            currentAngle = targetAngle;
            lastMovementTime = millis();

            Serial.print("Current angle updated to: ");
            Serial.println(currentAngle);
        }
    }

    // Get current angle
    int getCurrentAngle() const
    {
        return currentAngle;
    }

    // Check if motor is currently moving
    bool isMotorMoving() const
    {
        return isMoving;
    }
};

#endif // MOTOR_CONTROLLER_H