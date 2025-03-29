/*
//This code is designed by the Iowa State Robotics for use on a RMD-X motor
*/

// Libraries for using the CAN board
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10); // sets reading to pin 10
int position = 1;    // sets a variable to be read later on, 1 is starting position

void setup()
// This sets up the Can Sheild and check to make sure it is working
{
  while (Serial)
    ;
  Serial.begin(9600); // set to serial baud monitor rate

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ); // set the CAN bus speed to 500KBPS and the frequency of the crystal oscillator to 16 MHz
  mcp2515.setNormalMode();

  Serial.print("Setup done\n\n");
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

// example line

void loop()
{
  int incomingByte = 0;
  canMsg.can_id = 0x141; // CAN motor identifier
  canMsg.can_dlc = 0x08; // I think sets data at 8 bits
  if (1 == position)     // sets inital comand for the motor (stop)
  {
    // This command sets the motor to stop
    canMsg.data[0] = 0x81;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
  }

  if (2 == position)
  {
    // This command set the motor forward (still needs changed)
    canMsg.data[0] = 0x81;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
  }
  if (3 == position)
  {
    // This command set the motor backwards (still needs changed)
    canMsg.data[0] = 0x81;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
  }
  mcp2515.sendMessage(&canMsg);            // sends selected CAN command
  printserial(canMsg.can_id, canMsg.data); // prints selected CAN command
  Serial.print("\n");

  //
  int len = 10;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK))
  {
    /* checks for CAN response */
    delay(1);
    len--;
    if ((len <= 0))
    {
      break;
    }
  }

  if (len > 0)
  {
    // if an answer is produced
    Serial.print("Recv   : ");
    printserial(canMsg.can_id, canMsg.data);
    Serial.print("\n");
  }
  else
  {
    // if no answer
    Serial.print("Recv   : NO ANSWER");
    Serial.print("\n");
  }
  Serial.print("\n");

  int pause_3000 = 30;
  while (pause_3000)
  {
    if (Serial.available() > 0)
    {
      // read the incoming byte:
      incomingByte = Serial.read();
      incomingByte = incomingByte - 0x30; // "1" -> 1

      while (Serial.available() > 0)
      {
        Serial.read();
      }
      break;
    }

    pause_3000--;
    delay(100);
  }
}
