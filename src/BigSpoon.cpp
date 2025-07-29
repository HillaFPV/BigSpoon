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
void blinkLight(uint8_t times);
long power5(float i);
long waitingForACK(uint32_t delayTime);

int32_t manualRC = 0;
String inputString = "";
char inputChar;
uint16_t speed = 500;
int32_t panPosition = 0;
int32_t filteredPanPosition = 0;
float elapsedTimeInSeconds  = 0.0;
long ackStatus;
int16_t panRC;
int16_t filteredPanRC;
int16_t tiltRC;
bool motorIsBusy = false;
  
PWM panServoPWM(2);  // Setup pin 2 for pan PWM
PWM tiltServoPWM(3); // Setup pin 3 for tilt PWM

String printToMonitor(uint8_t *value)
{
  int32_t iValue;
  String  tStr;
  iValue = (int32_t)(
                      ((uint32_t)value[0] << 24)    |
                      ((uint32_t)value[1] << 16)    |
                      ((uint32_t)value[2] << 8)     |
                      ((uint32_t)value[3] << 0)
                    );

  
  tStr = String(iValue);
  return tStr;
}

/*
Function: read real-time location information
Input: slaveAddr slave address
output: none
 */
void readRealTimeLocation(uint8_t slaveAddr)
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

  speedModeRun(1, 1, 100, 1);
}

void loop() {
  elapsedTimeInSeconds = 1E-6 * (micros() - start_time);

  panRC = (panServoPWM.getValue() - 1500) * 1; //1500 is the center-stick value for these PWM signals. They range from 1000-2000us.
  filteredPanRC = f.filter(panRC, elapsedTimeInSeconds);

  panPosition += filteredPanRC;

  readRealTimeLocation(1); //Slave address = 1, issue a query position information command

  ackStatus = waitingForACK(1000);      //Wait for the motor to answer
  Serial.print(">ackStatus: ");
  Serial.println(ackStatus);

  // if(ackStatus == true)        //Received location information
  // {
  //   printToMonitor(&rxBuffer[5]);
  // } else {
  //   Serial.print(">ackStatus:");
  //   Serial.println(ackStatus);

  // }

  // Serial.print(">panPosition:");
  // Serial.print(panPosition);
  // Serial.print(",motorIsBusy:");
  // Serial.println(motorIsBusy);

  // Serial.println("Firing up the motor");
  //positionMode4Run(1, 600, 254, panPosition);

  // Serial.println("Waiting for ack");
  //ackStatus = waitingForACK(3000);      //Wait for the position control to start answering

  // Serial.print("ackStatus: ");
  // Serial.println(ackStatus);

  // if(ackStatus == 1)                    //Position control starts
  // {
  //   Serial.println("Position control starts");
  //   ackStatus = waitingForACK(2000);     //Wait for the position control to complete the response

  //   Serial.println("Receipt of position control complete response");
  //   if(ackStatus == 2)                //Receipt of position control complete response
  //   {
  //     // Do something with the response      
  //     Serial.println("Do something with the response      ");
  //   }
  //   else                        //Location complete reply not received 
  //   {
  //     Serial.println("Location complete reply not received ");
  //     while(1)                //The flashing light indicates failure
  //     {
  //       digitalWrite(LED_BUILTIN, HIGH);     delay(100);
  //       digitalWrite(LED_BUILTIN, LOW);      delay(100);
  //     }
  //   }
  // }
  // else                      //Position control failed
  // {
  //   Serial.println("Position control failed");
  //   while(1)                //The flashing light indicates failure
  //   {
  //     digitalWrite(LED_BUILTIN, HIGH);     delay(200);
  //     digitalWrite(LED_BUILTIN, LOW);      delay(200);
  //   }
  // }

  loops++;
  // Serial.println("Looping");
}

/*
Function: Gets encoder value of a servo
Input: slaveAddr slave address
*/
void getEncoderValue(uint8_t slaveAddr) {
  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0x31;                                //function code
  txBuffer[3] = getCheckSum(txBuffer, 3);            //Calculate checksum
  
  Serial1.write(txBuffer, 4);
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
long waitingForACK(uint32_t delayTime)
{
  long retVal;       //return value
  unsigned long sTime;  //timing start time
  unsigned long time;  //current moment
  uint8_t rxByte;      
  uint8_t packetLength = 0;

  sTime = millis();    //get the current moment
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
        default:
          packetLength = 3;
          break;
      }
    }

    if (packetLength > 0) {
      if(rxCnt == packetLength)    //Receive complete
      {
        if(rxBuffer[packetLength-1] == getCheckSum(rxBuffer,packetLength-1))
        {
          switch (rxBuffer[2]) {
            case 0x31:
              retVal = 69;
          }
          // Checksum Correct
          long sum = 0;
          for (int i = 0; i<6; i++) {
            sum = (sum << 8);
            sum += rxBuffer[3+i];
          }
          retVal = sum;
          break;                  //exit while(1)
        }
        else
        {
          rxCnt = 0;  //Verification error, re-receive the response
          retVal = 100;
          break;
        }
      }
    }

    time = millis();
    if((delayTime != 0) && ((time - sTime) > delayTime))   //Judging whether to time out
    {
      retVal = 0;
      break;                    //timeout, exit while(1)
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
