# Exoskeleton Motor Control System

## Overview

This project implements a control system for an exoskeleton using RMD-X motors with CAN communication. The system uses a hall effect sensor for position control and implements a range of motion from -10° to 80° with a center position at 35°.

## Hardware Requirements

- Arduino board (compatible with SPI)
- MCP2515 CAN controller
- RMD-X motor
- Hall effect sensor
- CAN transceiver
- Power supply (compatible with RMD-X motor specifications)

## Pin Configuration

- CAN CS Pin: 10
- CAN LED: 8
- Hall Sensor: A0 (Analog input)

## Dependencies

- SPI.h
- mcp2515.h

## Features

1. **Position Control**

   - Range: -10° to 80°
   - Center position: 35°
   - Position resolution: 0.01° per LSB

2. **Hall Sensor Control**

   - Voltage thresholds for different movement rates:
     - 0-1V: 3° backward movement
     - 1-2V: 1° backward movement
     - 2-3V: Neutral (no movement)
     - 3-4V: 1° forward movement
     - 4-5V: 3° forward movement

3. **Motor Communication**

   - CAN bus communication at 500kbps
   - Motor ID: 0x141
   - Command types:
     - Position control (0x4A/0xA4)
     - Speed control (0xF4)
     - Torque control (0xA1)
     - Stop command (0x81)
     - Status read (0x9A)
     - Encoder position read (0x60)

4. **Safety Features**
   - Position limits enforcement
   - Command delay protection
   - Multiple retry attempts for failed communications
   - LED feedback for movement status

## Setup Instructions

1. Connect the hardware components according to the pin configuration
2. Install required libraries (SPI.h and mcp2515.h)
3. Upload the code to the Arduino board
4. Open Serial Monitor at 115200 baud rate
5. The system will initialize and set the motor to the center position (35°)

## Usage

The system can be controlled through:

1. **Serial Commands**

   - Send 1 for STOP
   - Send 2 for FORWARD
   - Send 3 for BACKWARD

2. **Hall Sensor Control**
   - Apply voltage to the hall sensor input (A0)
   - System will respond based on voltage thresholds

## Troubleshooting

1. **CAN Communication Issues**

   - Check CAN LED for activity
   - Verify motor ID (0x141)
   - Ensure proper power supply
   - Check CAN bus termination

2. **Position Control Issues**
   - Verify hall sensor voltage readings
   - Check position limits
   - Monitor serial output for error messages

## Technical Details

- CAN Message Format: 8 bytes
- Position Resolution: 0.01° per LSB
- Default Speed Limit: 500 dps
- Command Delay: 1000ms between commands
- Maximum Retry Attempts: 3

## Safety Considerations

- Always ensure proper power supply
- Monitor motor temperature
- Keep within specified angle limits
- Follow proper shutdown procedures

## Contributing

Please follow these steps for contributing:

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

[Specify your license here]

## Contact

[Add contact information here]
