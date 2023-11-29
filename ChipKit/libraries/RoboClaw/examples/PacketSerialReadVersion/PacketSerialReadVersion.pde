#include "BMSerial.h"
#include "RoboClaw.h"

#define address 0x80

BMSerial terminalSerial(0,1);

//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

RoboClaw rc(10,11);

void setup() {
  terminalSerial.begin(9600);
  rc.begin(38400);
}

void loop() {
  char version[32];

  if(rc.ReadVersion(address,version)){
    terminalSerial.println(version);  
  }

  delay(100);
}

