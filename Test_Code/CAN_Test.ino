/*
 * Basic CAN bus test for MCP2515
 */

#include <SPI.h>
#include <mcp2515.h>

// Configuration
const int SPI_CS_PIN = 10;
const int LED_PIN = 8;

// Global objects
MCP2515 mcp2515(SPI_CS_PIN);
struct can_frame canMsg;
struct can_frame rxMsg;

void setup()
{
    // Setup LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Initialize serial communication
    Serial.begin(9600);
    delay(1000);
    Serial.println(F("CAN Test"));

    // Initialize SPI
    SPI.begin();
    delay(100);

    // Initialize CAN controller
    mcp2515.reset();
    delay(100);

    // Set bitrate and mode
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    delay(100);
    mcp2515.setNormalMode();
    delay(100);

    // Signal ready
    digitalWrite(LED_PIN, LOW);
    Serial.println(F("Ready"));
}

void loop()
{
    // Prepare a test message
    canMsg.can_id = 0x141;
    canMsg.can_dlc = 8;
    canMsg.data[0] = 0x9A; // Read Motor Status 1 command
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;

    // Print message to send
    Serial.print(F("TX:"));
    Serial.print(canMsg.can_id, HEX);
    Serial.print(F(" "));
    for (int i = 0; i < 8; i++)
    {
        if (canMsg.data[i] < 0x10)
            Serial.print('0');
        Serial.print(canMsg.data[i], HEX);
        Serial.print(' ');
    }
    Serial.println();

    // Try to send message
    digitalWrite(LED_PIN, HIGH); // LED on during transmit
    bool sendResult = mcp2515.sendMessage(&canMsg);

    if (sendResult)
    {
        Serial.println(F("TX:OK"));
    }
    else
    {
        Serial.println(F("TX:Fail"));
    }

    // Wait for response
    digitalWrite(LED_PIN, LOW);
    delay(100);

    // Try to read a message
    bool gotResponse = false;
    for (int i = 0; i < 20; i++)
    {
        if (mcp2515.readMessage(&rxMsg) == MCP2515::ERROR_OK)
        {
            // Print received message
            Serial.print(F("RX:"));
            Serial.print(rxMsg.can_id, HEX);
            Serial.print(F(" "));
            for (int j = 0; j < 8; j++)
            {
                if (rxMsg.data[j] < 0x10)
                    Serial.print('0');
                Serial.print(rxMsg.data[j], HEX);
                Serial.print(' ');
            }
            Serial.println();

            gotResponse = true;
            break;
        }
        delay(10);
    }

    if (!gotResponse)
    {
        Serial.println(F("RX:None"));
    }

    // Show result with LED
    if (gotResponse)
    {
        // Success - quick double blink
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
    }
    else
    {
        // Failure - long blink
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
    }

    // Wait before next attempt
    Serial.println(F("---"));
    delay(2000);
}