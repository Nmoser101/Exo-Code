// *
//This code is designed by the Iowa State Robotics for use on a RMD-X motor
//*

//Libraries for using the CAN board
#include <SPI.h>       
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10); //sets reading to pin 10
int position = 1; // sets a variable to be read later on, 1 is starting position





void setup() 
//This sets up the Can Sheild and check to make sure it is working
{
    while (!Serial);
  Serial.begin(9600); // set to serial baud monitor rate 
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS,MCP_16MHZ); //set the CAN bus speed to 500KBPS and the frequency of the crystal oscillator to 16 MHz
  mcp2515.setNormalMode();

  Serial.print("Setup done\n\n");
}

//Sets up so that Can's 8 bit is proccessed correctly
void printserial(unsigned long ID, unsigned char buf[8]) {
  // converts buf[8] to a character string 
    Serial.print(ID, HEX);
    Serial.print(": ");
    for (int i = 0; i < 8; i++)
    {
      Serial.print(buf[i], HEX);
      Serial.print("  ");
    }
}


//example line

void loop() {

}
