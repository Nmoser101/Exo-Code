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
        return angle >= MIN_ANGLE && angle <= MAX_ANGLE;
    }

    void sendMotorCommand(int command)
    {
        canMsg.can_id = 0x141;
        canMsg.can_dlc = 0x08;

        switch (command)
        {
        case 0x81: // Stop command
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
            canMsg.data[0] = 0x83;
            canMsg.data[1] = 0x00;
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            break;

        default:                             // Position command (0-90)
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

        // Send command
        mcp2515.sendMessage(&canMsg);

        // Wait for response
        int len = 10;
        while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK))
        {
            delay(1);
            len--;
            if (len <= 0)
                break;
        }
    }

public:
    MotorController(MCP2515 &canController)
        : mcp2515(canController), currentAngle(0), lastMovementTime(0)
    {
        // Initialize motor to starting position
        moveMotorTo(0);
    }

    // Main control function to be called in loop()
    void update()
    {
        unsigned long currentTime = millis();

        // Check if enough time has passed since last movement
        if (currentTime - lastMovementTime < MOVEMENT_DELAY_MS)
        {
            return;
        }

        // Read joystick input from CAN
        if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
        {
            // Assuming joystick input is in canMsg.data[0]
            int joystickInput = canMsg.data[0];

            // Process input
            switch (joystickInput)
            {
            case 0: // Move backward
                if (isWithinBounds(currentAngle - 1))
                {
                    moveMotorTo(currentAngle - 1);
                    currentAngle--;
                    lastMovementTime = currentTime;
                }
                break;

            case 2: // Move forward
                if (isWithinBounds(currentAngle + 1))
                {
                    moveMotorTo(currentAngle + 1);
                    currentAngle++;
                    lastMovementTime = currentTime;
                }
                break;

            case 1: // No movement
                // Stop motor
                sendMotorCommand(0x81);
                break;
            }
        }
    }

    // Function to move motor to specific angle
    void moveMotorTo(int targetAngle)
    {
        if (!isWithinBounds(targetAngle))
        {
            return; // Safety check
        }

        // Send command to move to target angle
        sendMotorCommand(targetAngle);

        // Update current angle
        currentAngle = targetAngle;
        lastMovementTime = millis();
    }

    // Get current angle
    int getCurrentAngle() const
    {
        return currentAngle;
    }
};

#endif // MOTOR_CONTROLLER_H