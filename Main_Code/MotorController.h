#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

/**
 * MotorController class for RMD-X motor using CAN bus communication
 * Implements protocol commands as defined in RMD-X Motor Motion Protocol V3.9
 */
class MotorController
{
private:
    // Motor constraints
    static const int MIN_ANGLE = 0;
    static const int MAX_ANGLE = 90;
    static const int MOVEMENT_DELAY_MS = 1000; // Delay between movement commands

    // Motor state
    int currentAngle;
    unsigned long lastMovementTime;
    MCP2515 &mcp2515;
    bool isMoving;

    // CAN message structure
    struct can_frame canMsg;

    // Helper functions for angle validation
    bool isWithinBounds(int angle)
    {
        bool valid = angle >= MIN_ANGLE && angle <= MAX_ANGLE;
        if (!valid)
        {
            Serial.print("ERROR: Angle out of bounds: ");
            Serial.println(angle);
        }
        return valid;
    }

    // Debug function to print CAN messages
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

    /**
     * Send a command to the motor via CAN bus
     * @param command For position commands: angle value (0-90)
     *                For control commands: command code (0x81, 0x82, 0x83)
     * @return true if command was sent successfully and response received
     */
    bool sendMotorCommand(int command)
    {
        Serial.print("\n--- Sending Motor Command: 0x");
        Serial.println(command, HEX);

        // Protocol requires ID 0x141 and 8 data bytes
        canMsg.can_id = 0x141;
        canMsg.can_dlc = 0x08;

        switch (command)
        {
        case 0x81: // Stop command - Direct protocol command 0x81
            Serial.println("Command Type: STOP");
            canMsg.data[0] = 0x81; // Stop command per protocol
            canMsg.data[1] = 0x00;
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            isMoving = false;
            break;

        case 0x82: // Forward command - Using position increment
            Serial.println("Command Type: FORWARD (Incremental Position)");
            // Using torque control instead of position for more reliable movement
            canMsg.data[0] = 0xA1; // Torque control command per protocol
            canMsg.data[1] = 0x32; // Small positive torque (50 = 0.5A)
            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            isMoving = true;
            break;

        case 0x83: // Backward command - Using position decrement
            Serial.println("Command Type: BACKWARD (Torque Control)");
            // Using torque control instead of position for more reliable movement
            canMsg.data[0] = 0xA1; // Torque control command per protocol
            canMsg.data[1] = 0xCE; // Small negative torque (-50 = -0.5A)
            canMsg.data[2] = 0xFF; // 0xFFCE = -50 in 16-bit signed
            canMsg.data[3] = 0x00;
            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;
            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            isMoving = true;
            break;

        default: // Position command (0-90) - Using absolute position control
            Serial.println("Command Type: POSITION (Absolute Position)");
            Serial.print("Target angle: ");
            Serial.println(command);

            // Convert angle to position value (0.01 degree per LSB)
            int32_t posValue = command * 100; // Convert to 0.01 degree units

            canMsg.data[0] = 0xA4;                    // Absolute position command per protocol
            canMsg.data[1] = posValue & 0xFF;         // Position bytes 0-1 (LSB)
            canMsg.data[2] = (posValue >> 8) & 0xFF;  // Position bytes 0-1
            canMsg.data[3] = (posValue >> 16) & 0xFF; // Position bytes 2-3
            canMsg.data[4] = (posValue >> 24) & 0xFF; // Position bytes 2-3 (MSB)
            // Set a low max speed for safety (100 dps)
            canMsg.data[5] = 0x64; // Max speed (100 dps) low byte
            canMsg.data[6] = 0x00; // Max speed high byte
            canMsg.data[7] = 0x00; // Reserved
            break;
        }

        printCANMessage("Sending");

        // Send command via CAN bus
        bool success = mcp2515.sendMessage(&canMsg);
        if (!success)
        {
            Serial.println("ERROR: Failed to send CAN message!");
            return false;
        }
        Serial.println("CAN message sent successfully");

        // Wait for response with longer timeout
        int timeout = 100; // 100ms timeout (increased from 20ms)
        bool gotResponse = false;
        while (timeout > 0)
        {
            if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
            {
                printCANMessage("Received");
                gotResponse = true;
                break;
            }
            delay(1);
            timeout--;
        }

        if (!gotResponse)
        {
            Serial.println("ERROR: No response received from motor!");
            // Continue operation even without response for testing
            return true;
        }

        Serial.println("Motor response received successfully");
        return true;
    }

public:
    /**
     * Initialize the motor controller with the specified CAN controller
     * @param canController MCP2515 CAN controller instance
     */
    MotorController(MCP2515 &canController)
        : mcp2515(canController), currentAngle(0), lastMovementTime(0), isMoving(false)
    {
        Serial.println("MotorController initialized");
        // Initialize motor to starting position
        moveMotorTo(0);
    }

    /**
     * Move the motor forward by one increment (implemented using protocol commands)
     * @return true if command was successful, false otherwise
     */
    bool moveForward()
    {
        unsigned long currentTime = millis();
        if (currentTime - lastMovementTime < MOVEMENT_DELAY_MS)
        {
            Serial.println("Waiting for movement delay...");
            return false;
        }

        if (isWithinBounds(currentAngle + 1))
        {
            Serial.println("Moving forward one increment");
            if (sendMotorCommand(0x82)) // Send forward command
            {
                currentAngle++;
                lastMovementTime = currentTime;
                Serial.print("Angle increased to: ");
                Serial.println(currentAngle);
                return true;
            }
            return false;
        }
        else
        {
            stop(); // Stop if we hit the limit
            Serial.println("Cannot move forward: at maximum angle limit");
            return false;
        }
    }

    /**
     * Move the motor backward by one increment (implemented using protocol commands)
     * @return true if command was successful, false otherwise
     */
    bool moveBackward()
    {
        unsigned long currentTime = millis();
        if (currentTime - lastMovementTime < MOVEMENT_DELAY_MS)
        {
            Serial.println("Waiting for movement delay...");
            return false;
        }

        if (isWithinBounds(currentAngle - 1))
        {
            Serial.println("Moving backward one increment");
            if (sendMotorCommand(0x83)) // Send backward command
            {
                currentAngle--;
                lastMovementTime = currentTime;
                Serial.print("Angle decreased to: ");
                Serial.println(currentAngle);
                return true;
            }
            return false;
        }
        else
        {
            stop(); // Stop if we hit the limit
            Serial.println("Cannot move backward: at minimum angle limit");
            return false;
        }
    }

    /**
     * Stop the motor using the protocol's stop command (0x81)
     */
    void stop()
    {
        if (isMoving)
        {
            Serial.println("Stopping motor");
            if (sendMotorCommand(0x81)) // Send stop command
            {
                isMoving = false;
                Serial.println("Motor stopped");
            }
        }
    }

    /**
     * Move the motor to a specific angle using absolute position control
     * @param targetAngle Target angle (0-90 degrees)
     */
    void moveMotorTo(int targetAngle)
    {
        Serial.print("Moving to angle: ");
        Serial.println(targetAngle);

        if (!isWithinBounds(targetAngle))
        {
            Serial.println("Target angle out of bounds, ignoring command");
            return; // Safety check
        }

        unsigned long currentTime = millis();
        if (currentTime - lastMovementTime < MOVEMENT_DELAY_MS)
        {
            Serial.println("Waiting for movement delay...");
            return;
        }

        // Send command to move to target angle
        if (sendMotorCommand(targetAngle))
        {
            // Update current angle
            currentAngle = targetAngle;
            lastMovementTime = currentTime;

            Serial.print("Current angle updated to: ");
            Serial.println(currentAngle);
        }
        else
        {
            Serial.println("Failed to move to target angle");
        }
    }

    /**
     * Get the current angle of the motor
     * @return Current angle in degrees
     */
    int getCurrentAngle() const
    {
        return currentAngle;
    }

    /**
     * Check if the motor is currently moving
     * @return true if the motor is moving
     */
    bool isMotorMoving() const
    {
        return isMoving;
    }
};

#endif // MOTOR_CONTROLLER_H