/*
Code by Zeaan.
This code recieves a number 1 - 9 from the serial monitor. Each number is linked to a preset Amp/Torque value that is sent out to the motor.

Basic RMD-X code vir Torque control op A1. Speed control is cod A2.

Vir beheer:
Sien die manual: RMD-X servo motor control protocal V3.8 - onder aan bl. 45. Manual is in crop sprayer folder onder notes.

Torque: waarde is n target current wat die motor haandaf. Torque control is A1 vir buf 0. En die waarde word geskryf op buf 4 en 5.
        buf 4 is low byte en buf 5 is high byte. dit saam gee die spoed of torque waarde. hierdie is HEX waardes.
        bv buf[4] 0x64 en buf[5] 0x00 is saam Hex 0x0064 en is gelyk aan dec 100. 100 * 0.001A is 1A wat die motor sal probeer haandaf.

Spoed is die selfde konsep maar buf 4 tot 7 word gebruik. buf 4 is die laagste bit en buf 7 die hoogste.


*/

#include <SPI.h>     //Library for using SPI Communication
#include <mcp2515.h> //Library for using CAN Communication  autowp-mcp2515 by autowp . V1.0.3

struct can_frame canMsg;
MCP2515 mcp2515(10);
int incomingByte = 1; // 1 stop; 2 start;

void setup()
{
  while (!Serial)
    ;
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ); // set the CAN bus speed to 1 Mbps and the frequency of the crystal oscillator to 16 MHz
  mcp2515.setNormalMode();

  Serial.print("Setup done\n\n");
}

void printserial(unsigned long ID, unsigned char buf[8])
{
  // converts buf[8] to a character string
  Serial.print(ID, HEX);
  Serial.print(": ");
  for (int i = 0; i < 8; i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print("  ");
  }
}

void loop()
{
  canMsg.can_id = 0x141; // set CAN id
  canMsg.can_dlc = 0x08;

  if (1 == incomingByte) //   Stop and brake the motor
  {
    canMsg.data[0] = 0x81; // code 81
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    Serial.print("Send(1): ");
  }

  if (2 == incomingByte)
  {
    canMsg.data[0] = 0xA1; // A2 is Speed command.
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00; // 64Hex = 100dez =  1Amp for torque control
    canMsg.data[4] = 0x64; // speed lowest byte: A0. But we do torque here, not speed control
    canMsg.data[5] = 0x00; // speed 2de byte   : 86
    canMsg.data[6] = 0x00; // speed 3de byte   : 01
    canMsg.data[7] = 0x00; // speed Highest byte: 00. - Speed command is HEX 000186A0 = dec 100 000 en word gedeel deur 100. so 100000*0.01 = 1000dps.
                           // dps is "degrees per second per least significant bit"heel meentlik arc degrees. sien skyentific se youtube videos
    Serial.print("Send(2): ");
  }

  if (3 == incomingByte)
  {
    canMsg.data[0] = 0xA1;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x32; // 32Hex = 50dec =  0.5Amp for torque control
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    Serial.print("Send(3): ");
  }

  if (4 == incomingByte)
  {
    canMsg.data[0] = 0xA1; // NOTE is op A1 vir torque control maar die HEX values is 0 daarom kan die motor vrylik draai.
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00; // Torque low byte:  0 - so die motor kan vrylik draai !!!
    canMsg.data[5] = 0x00; // Torque high byte: 0
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    Serial.print("Send(4): ");
  }

  if (5 == incomingByte) // motor reverse @0.5Amp. -50dec = CE FF in hex
  {
    canMsg.data[0] = 0xA1; // NOTE is op A1 vir torque control maar die HEX values is 0 daarom kan die motor vrylik draai.
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0xCE; // Torque low byte:  0 - so die motor kan vrylik draai !!!
    canMsg.data[5] = 0xFF; // Torque high byte: 0
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    Serial.print("Send(4): ");
  }

  mcp2515.sendMessage(&canMsg); // send CAN message
  printserial(canMsg.can_id, canMsg.data);
  Serial.print("\n");

  int len = 10;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK))
  {
    /* check received response CAN */
    delay(1);
    len--;
    if ((len <= 0))
    {
      break;
    }
  }

  if (len > 0)
  {
    // came the answer
    Serial.print("Recv   : ");
    printserial(canMsg.can_id, canMsg.data);
    Serial.print("\n");
  }
  else
  {
    // no answer
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