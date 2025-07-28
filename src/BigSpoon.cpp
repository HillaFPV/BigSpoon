#include "PWM.hpp"
#include <math.h>
#include "1euroFilter.h"

static OneEuroFilter f; // not enabled yet, setup has to be called later
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
uint8_t waitingForACK(uint8_t ackMessageLen);
void blinkLight(uint8_t times);
long power5(float i);

int32_t manualRC = 0;
String inputString = "";
char inputChar;
uint16_t speed = 3000;
int32_t panPosition = 0;
int32_t filteredPosition = 0;

PWM panServoPWM(2);  // Setup pin 2 for pan PWM
PWM tiltServoPWM(3); // Setup pin 3 for tilt PWM


void positionMode4Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position) {
  
  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0xF5;                                //function code
  txBuffer[3] = (speed >> 8) & 0xFF;
  txBuffer[4] = speed & 0x00FF;                      //8 bits lower
  txBuffer[5] = acc;                                 //acceleration
  txBuffer[6] = (position & 0xff000000) >> 24;
  txBuffer[7] = (position & 0x00ff0000) >> 16;
  txBuffer[8] = (position & 0x0000ff00) >> 8;
  txBuffer[9] = (position & 0x000000ff);       
  txBuffer[10] = getCheckSum(txBuffer, 10);          //Calculate checksum
  
  Serial1.write(txBuffer, 11);
}

void setup() {
  // Set the LED light port as output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // setup filter
  f.begin(FREQUENCY, MINCUTOFF, BETA);
  
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
}

void loop() {
  
  uint8_t ackStatus;
  uint16_t panRC;
  uint16_t tiltRC;
  
  panRC = panServoPWM.getValue();
  tiltRC = tiltServoPWM.getValue();
  
  float elapsed_time = 1E-6 * (micros() - start_time); // in seconds
  
  float pan_filtered_signal = f.filter(panRC, elapsed_time);
  // float tilt_filtered_signal = f.filter(tiltRC, elapsed_time); 
  
  // if (Serial.available() > 0) {
  //   inputChar = Serial.read();
  //   Serial.write(inputChar);
  //   if (inputChar == '\r') {
      
  //   }
  //   else if (inputChar == '\n') {
  //     manualRC =  (int32_t) strtol( inputString.c_str(), 0, 16);
      
  //     Serial.print(">manualRC:");
  //     Serial.print(manualRC);
  //     Serial.print("filteredPosition: ");
  //     Serial.print(filteredPosition);
  //     Serial.print(",inputString:");
  //     Serial.println(inputString);
      
  //     inputString = "";
  //   }
  //   else {
  //     inputString.concat(inputChar);
  //   }
  // }
  
  // uint8_t manualRunDir = 0;
  // int16_t manualRunSpeed = (1500 - manualRC) * 6;
  
  // if (manualRunSpeed < 0) {
  //   manualRunDir = 0;
  // } else {
  //   manualRunDir = 1;
  // }
  
  // uint16_t manualAbsRunSpeed = min(abs(manualRunSpeed), 3000);
  
  if (loops % 100 == 0) {
    panPosition += pan_filtered_signal/10;
  }
  
  filteredPosition = (int32_t)f.filter(panPosition, elapsed_time);
  Serial.print(">filteredPosition: ");
  Serial.print(filteredPosition);
  Serial.print(",panRC: ");
  Serial.print(panRC);
  Serial.print(",pan_filtered_signal: ");
  Serial.print(pan_filtered_signal);
  Serial.print(", panPosition:");
  Serial.println(panPosition);

  positionMode4Run(1, speed, 254, filteredPosition);
  
  // ackStatus = waitingForACK(5);      //Wait for the motor to answer
  // if (Serial1.available() > 0)  //The serial port receives data
  // {
  //   rxByte = Serial1.read();  //read 1 byte data
  //   if (rxCnt != 0) {
  //     rxBuffer[rxCnt++] = rxByte;  //Storing data
  //   } else if (rxByte == 0xFB)     //Determine whether the frame header
  //   {
  //     rxBuffer[rxCnt++] = rxByte;  //store frame header
  //   }
  // }
  
  // uint8_t ackMessageLen = 5;
  
  // if (rxCnt == ackMessageLen)  //Receive complete at 5 bytes
  // {
  //   if (rxBuffer[ackMessageLen - 1] == getCheckSum(rxBuffer, ackMessageLen - 1)) {
  //     statusCode = rxBuffer[3];  //checksum correct
  //   } else {
  //     rxCnt = 0;  //Verification error, re-receive the response
  //   }
  // }
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
  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0xF6;                                //function code
  txBuffer[3] = (dir << 7) | ((speed >> 8) & 0x0F);  //High 4 bits for direction and speed
  txBuffer[4] = speed & 0x00FF;                      //8 bits lower
  txBuffer[5] = acc;                                 //acceleration
  txBuffer[6] = getCheckSum(txBuffer, 6);            //Calculate checksum
  
  Serial1.write(txBuffer, 7);
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
Input: ackMessageLen Length of the response frame
output:
Run successfully 1
failed to run 0
timeout no reply 0
*/
uint8_t waitingForACK(uint8_t ackMessageLen) {
  uint8_t retVal;       //return value
  uint8_t rxByte;

  if (Serial1.available() > 0)  //The serial port receives data
  {
    rxByte = Serial1.read();  //read 1 byte data
    if (rxCnt != 0) {
      rxBuffer[rxCnt++] = rxByte;  //Storing data
    } else if (rxByte == 0xFB)     //Determine whether the frame header
    {
      rxBuffer[rxCnt++] = rxByte;  //store frame header
    }
  }
  
  if (rxCnt == ackMessageLen)  //Receive complete
  {
    if (rxBuffer[ackMessageLen - 1] == getCheckSum(rxBuffer, ackMessageLen - 1)) {
      retVal = rxBuffer[3];  //checksum correct
    } else {
      rxCnt = 0;  //Verification error, re-receive the response
    }
  }

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
