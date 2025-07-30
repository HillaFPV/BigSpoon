#include "PWM.hpp"
#include <math.h>
#include "1euroFilter.h"

static OneEuroFilter panRCFilter;
static OneEuroFilter panPTermF;
// Frequency of your incoming noisy data
// If you are able to provide timestamps, the frequency is automatically determined
#define FREQUENCY   1   // [Hz] 
#define MINCUTOFF   1.0   // Think D-term. 0.05-2.00 range is good
#define BETA        0.0   // Think P-term. 0.00-0.005 range is good

unsigned long start_time;
uint8_t loops = 0;
uint8_t txBuffer[20];  //Sends data array
uint8_t rxBuffer[20];  //Receive data array
uint8_t rxCnt = 0;     //Receive data count
uint8_t statusCode;
uint8_t rxByte;

uint8_t getCheckSum(uint8_t *buffer, uint8_t ackMessageLen);
void speedModeRun(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc);
void blinkLight(uint8_t times);
long power5(float i);
long waitingForACK();

int32_t manualRC = 0;
String inputString = "";
char inputChar;

int32_t panPosition = 0;
int32_t filteredPanPosition = 0;
float elapsedTimeInSeconds  = 0.0;
long bootUpEncoderValue;
long encoderValue;
int16_t panRC;
int16_t filteredPanRC;
int16_t filteredPTerm;
int16_t tiltRC;
bool motorIsBusy = false;

long maxAngle = 100000;
long minAngle = -100000;

PWM panServoPWM(2);  // Setup pin 2 for pan PWM
PWM tiltServoPWM(3); // Setup pin 3 for tilt PWM

/*
Function: read real-time location information
Input: slaveAddr slave address
output: none
*/
void getEncoderValue(uint8_t slaveAddr)
{
  
  txBuffer[0] = 0xFA;       //frame header
  txBuffer[1] = slaveAddr;  //slave address
  txBuffer[2] = 0x31;       //function code
  txBuffer[3] = getCheckSum(txBuffer,3);  //Calculate checksum
  Serial1.write(txBuffer,4);   //The serial port issues a command to read the real-time position
  Serial1.flush();
}

void positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses) {
  
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
}

void positionMode2Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t pulses) {
  
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
}

void positionMode3Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position) {
  
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
}

void positionMode4Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position) {
  
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
}

void setup() {
  // Set the LED light port as output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // setup filters
  panRCFilter.begin(FREQUENCY, MINCUTOFF, BETA);
  panPTermF.begin(1, 0.05, 0.00005);
  
  // Start the serial ports and wait for connect
  Serial.begin(115200);
  Serial1.begin(115200);
  
  while (!Serial || !Serial1) {
    ; 
  }
  
  // Start listening for PWM HIGH
  panServoPWM.begin(true);
  tiltServoPWM.begin(true);
  
  start_time = micros();
  
  int32_t motorBootCode = 0;
  while(motorBootCode == 0) {
    getEncoderValue(1);
    motorBootCode = waitingForACK();
  }
  panPosition = motorBootCode;
  encoderValue = motorBootCode;
  bootUpEncoderValue = motorBootCode;
}

void loop() {
  elapsedTimeInSeconds = 1E-6 * (micros() - start_time);
  
  /* Check if PWM signal is out of range.
     if so, then ELRS isn't connected. Don't run the loop
     until we get RC link.
    */
  int panPWMValue = panServoPWM.getValue();
  if (panPWMValue < 900 || panPWMValue > 2100) {
    return;
  }

  panRC = (panPWMValue - 1500) * 3.8; // 1500 is the center-stick value for these PWM signals. They range from 1000-2000us.
  filteredPanRC = panRCFilter.filter(panRC, elapsedTimeInSeconds);

  panPosition += filteredPanRC;

  if (panPosition < (bootUpEncoderValue + minAngle)) {
    panPosition = bootUpEncoderValue + minAngle;
  }
  else if (panPosition >= (bootUpEncoderValue + maxAngle)) {
    panPosition = bootUpEncoderValue + maxAngle;
  }

  getEncoderValue(1);
  encoderValue = waitingForACK();      //Wait for the motor to answer
  
  long error = encoderValue - panPosition;
  int dir = 0;
  if (error < 0) {
    dir = 1;
  }
  
  uint16_t pTerm = min(abs(error * 0.10), 2000);
  
  elapsedTimeInSeconds = 1E-6 * (micros() - start_time);
  filteredPTerm = panPTermF.filter(pTerm, elapsedTimeInSeconds);
   
  Serial.print(">panPosition:");
  Serial.print(panPosition);
  Serial.print(",panRC:");
  Serial.print(panRC);
  Serial.print(",filteredPanRC:");
  Serial.print(filteredPanRC);
  Serial.print(",error:");
  Serial.print(error);
  Serial.print(",filteredPTerm:");
  Serial.print(filteredPTerm);  
  Serial.print(",pTerm:");
  Serial.print(pTerm);    
  Serial.print(",encoderValue:");  
  Serial.println(encoderValue);  
  
  speedModeRun(1, dir, pTerm, 0);
  int motorResponse = waitingForACK();      //Wait for the motor to answer
  loops++;
}

/*
Function: Serial port sends speed mode operation command
Input: slaveAddr slave address
dir running direction
speed running speed
acc acceleration
*/
void speedModeRun(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc) {
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
}

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
  return (sum & 0xFF);  //return checksum
}

/*
Function: Wait for the response from the lower computer, set the timeout time to 3000ms
enter:
delayTime waiting time (ms),
delayTime = 0 , wait indefinitely
output:
Position mode 2 control start 1
Position mode 2 control completed 2
Position mode 2 control failure 0
timeout no reply 0
*/
long waitingForACK()
{
  long retVal;       //return value
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
  return(retVal);
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
