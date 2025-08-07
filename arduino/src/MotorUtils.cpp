#include "Arduino.h"
#include <stdint.h>
#include "BigSpoon.h"

/*
Function: Calculate the checksum of a set of data
Input: buffer data to be verified
size The number of data to be verified
output: checksum  
*/
uint8_t getCheckSum(uint8_t *buffer, uint8_t size) {
  uint8_t i;
  uint16_t sum = 0;
  for (i = 0; i < size; i++) {
    sum += buffer[i];  //Calculate accumulated value
  }
  return (sum & 0xFF); 
}

/*
Function: Wait for the response from the MKS 42D servo controller
*/
long waitingForACK()
{
  long retVal;
  uint8_t rxByte;      
  uint8_t packetLength = 0;
  
  rxCnt = 0;           //Receive count value set to 0
  while(1)
  {
    if (Serial1.available() > 0)     //The serial port receives data
    {
      rxByte = Serial1.read();       //read 1 byte data
      if(rxCnt != 0)
      {
        rxBuffer[rxCnt++] = rxByte; //Storing data
      }
      else if(rxByte == 0xFB)       //Determine whether the frame header
      {
        rxBuffer[rxCnt++] = rxByte;   //store frame header
      }
    }
        
    if(rxCnt == 3) // Function Code
    {
      switch (rxBuffer[2]){
        case 0x31:
        packetLength = 10;
        break;  
        case 0xF6:
        packetLength = 5;
        break;            
        default:
        packetLength = 0;
        break;
      }
    }
    
    if (packetLength > 0) {
      if(rxCnt == packetLength)    //Receive complete
      {
        if(rxBuffer[packetLength-1] == getCheckSum(rxBuffer,packetLength-1))
        {
          switch (rxBuffer[2]) {
            case 0x31: {
              // Encoder Value
              long sum = 0;
              for (int i = 0; i<6; i++) {
                sum = (sum << 8);
                sum += rxBuffer[3+i];
              }
              retVal = sum;
              break;
            }
            case 0xF6: {
              // Speed Mode Command
              retVal = rxBuffer[3];
              break;   
            }
          }
          
          rxCnt = 0;
          break;
        }
        else
        {
          rxCnt = 0;  //Verification error, re-receive the response
          retVal = -69;
          break;
        }
      }
    }
    
  }
  delay(1);
  return(retVal);
}

/*
Function: read real-time location information
Input: slaveAddr slave address
output: none
*/
long getEncoderValue(uint8_t slaveAddr) {
  txBuffer[0] = 0xFA;       //frame header
  txBuffer[1] = slaveAddr;  //slave address
  txBuffer[2] = 0x31;       //function code
  txBuffer[3] = getCheckSum(txBuffer,3);  //Calculate checksum

  Serial1.write(txBuffer,4);   //The serial port issues a command to read the real-time position
  Serial1.flush();

  return waitingForACK();  
}

long positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses) {
  txBuffer[0] = 0xFA;       //frame header
  txBuffer[1] = slaveAddr;  //slave address
  txBuffer[2] = 0xFD;       //function code
  txBuffer[3] = (dir<<7) | ((speed>>8)&0x0F); //High 4 bits for direction and speed
  txBuffer[4] = speed&0x00FF;   //8 bits lower
  txBuffer[5] = acc;            //acceleration
  txBuffer[6] = (pulses >> 24)&0xFF;  //Pulse bit31 - bit24
  txBuffer[7] = (pulses >> 16)&0xFF;  //Pulse bit23 - bit16
  txBuffer[8] = (pulses >> 8)&0xFF;   //Pulse bit15 - bit8
  txBuffer[9] = (pulses >> 0)&0xFF;   //Pulse bit7 - bit0
  txBuffer[10] = getCheckSum(txBuffer,10);  //Calculate checksum
  
  Serial1.write(txBuffer,11);
  Serial1.flush();

  return waitingForACK(); 
}

long positionMode2Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t pulses) {
  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0xFE;                                //function code
  txBuffer[3] = (speed >> 8) & 0xFF;                 //8 bit higher speed
  txBuffer[4] = speed & 0x00FF;                      //8 bits lower
  txBuffer[5] = acc;                                 //acceleration
  txBuffer[6] = (pulses >> 24) & 0xFF;               //Absolute pulses bit31 - bit24
  txBuffer[7] = (pulses >> 16) & 0xFF;               //Absolute pulses bit23 - bit16
  txBuffer[8] = (pulses >> 8) & 0xFF;                //Absolute pulses bit15 - bit8
  txBuffer[9] = (pulses >> 0) & 0xFF;                //Absolute pulses bit7 - bit0      
  txBuffer[10] = getCheckSum(txBuffer, 10);          //Calculate checksum
  
  Serial1.write(txBuffer, 11);
  Serial1.flush();

  return waitingForACK(); 
}

long positionMode3Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position) {
  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0xF4;                                //function code
  txBuffer[3] = (speed >> 8) & 0xFF;                 //8 bit higher speed
  txBuffer[4] = speed & 0x00FF;                      //8 bits lower
  txBuffer[5] = acc;                                 //acceleration
  txBuffer[6] = (position >> 24) & 0xFF;             //Relative coordinates bit31 - bit24
  txBuffer[7] = (position >> 16) & 0xFF;             //Relative coordinates bit23 - bit16
  txBuffer[8] = (position >> 8) & 0xFF;              //Relative coordinates bit15 - bit8
  txBuffer[9] = (position >> 0) & 0xFF;              //Relative coordinates bit7 - bit0      
  txBuffer[10] = getCheckSum(txBuffer, 10);          //Calculate checksum
  
  Serial1.write(txBuffer, 11);
  Serial1.flush();

  return waitingForACK();  
}

long positionMode4Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position) {
  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0xF5;                                //function code
  txBuffer[3] = (speed >> 8) & 0xFF;                 //8 bit higher speed
  txBuffer[4] = speed & 0x00FF;                      //8 bits lower
  txBuffer[5] = acc;                                 //acceleration
  txBuffer[6] = (position >> 24) & 0xFF;             //Absolute coordinates bit31 - bit24
  txBuffer[7] = (position >> 16) & 0xFF;             //Absolute coordinates bit23 - bit16
  txBuffer[8] = (position >> 8) & 0xFF;              //Absolute coordinates bit15 - bit8
  txBuffer[9] = (position >> 0) & 0xFF;              //Absolute coordinates bit7 - bit0      
  txBuffer[10] = getCheckSum(txBuffer, 10);          //Calculate checksum
  
  Serial1.write(txBuffer, 11);
  Serial1.flush();

  return waitingForACK(); 
}

/*
Function: Serial port sends speed mode operation command
Input: slaveAddr slave address
dir running direction
speed running speed
acc acceleration
*/
long speedModeRun(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc) {
  if (speed == 0) {
    speed = 1; // Trick the motor into not sending the stop command
  }
  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0xF6;                                //function code
  txBuffer[3] = (dir << 7) | ((speed >> 8) & 0x0F);  //High 4 bits for direction and speed
  txBuffer[4] = speed & 0x00FF;                      //8 bits lower
  txBuffer[5] = acc;                                 //acceleration
  txBuffer[6] = getCheckSum(txBuffer, 6);            //Calculate checksum
  
  Serial1.write(txBuffer, 7);
  Serial1.flush();

  return waitingForACK(); 
}



void blinkLight(uint8_t times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
  delay(2000);
}