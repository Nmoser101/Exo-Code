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
    static const int MOVEMENT_DELAY_MS = 75; // Delay between movements

    // Motor state
    int currentAngle;
    unsigned long lastMovementTime;
    MCP2515 &mcp2515;

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

    void sendMotorCommand(int command)
    {
        Serial.print("Sending command: 0x");
        Serial.println(command, HEX);

        canMsg.can_id = 0x141;
        canMsg.can_dlc = 0x08;

        switch (command)
        {
        case 0x81: // Stop command
            Serial.println("Sending STOP command");
            canMsg.data[0] = 0x81;
            canMsg.data[1] = 0x00;
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            break;

        case 0x82: // Forward command
            Serial.println("Sending FORWARD command");
            canMsg.data[0] = 0x82;
            canMsg.data[1] = 0x00;
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            break;

        case 0x83: // Backward command
            Serial.println("Sending BACKWARD command");
            canMsg.data[0] = 0x83;
            canMsg.data[1] = 0x00;
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            break;

        default: // Position command (0-90)
            Serial.print("Sending POSITION command to angle: ");
            Serial.println(command);
            canMsg.data[0] = 0x84;           // Position command identifier
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
        if (!mcp2515.sendMessage(&canMsg))
        {
            Serial.println("Failed to send CAN message!");
            return;
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
            Serial.println("No response received from motor!");
        }
    }

public:
    MotorController(MCP2515 &canController)
        : mcp2515(canController), currentAngle(0), lastMovementTime(0)
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
            return;
        }

        if (isWithinBounds(currentAngle + 1))
        {
            Serial.println("Moving forward");
            moveMotorTo(currentAngle + 1);
            currentAngle++;
            lastMovementTime = currentTime;
        }
    }

    void moveBackward()
    {
        unsigned long currentTime = millis();
        if (currentTime - lastMovementTime < MOVEMENT_DELAY_MS)
        {
            return;
        }

        if (isWithinBounds(currentAngle - 1))
        {
            Serial.println("Moving backward");
            moveMotorTo(currentAngle - 1);
            currentAngle--;
            lastMovementTime = currentTime;
        }
    }

    void stop()
    {
        Serial.println("Stopping motor");
        sendMotorCommand(0x81); // Stop command
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
        sendMotorCommand(targetAngle);

        // Update current angle
        currentAngle = targetAngle;
        lastMovementTime = millis();

        Serial.print("Current angle updated to: ");
        Serial.println(currentAngle);
    }

    // Get current angle
    int getCurrentAngle() const
    {
        return currentAngle;
    }
};

#endif // MOTOR_CONTROLLER_H