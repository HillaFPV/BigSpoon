#include "PWM.hpp"
#include <math.h>
#include "1euroFilter.h"

static OneEuroFilter f; // not enabled yet, setup has to be called later
// Frequency of your incoming noisy data
// If you are able to provide timestamps, the frequency is automatically determined
#define FREQUENCY   100   // [Hz] 
#define MINCUTOFF   10.0   // [Hz] needs to be tuned according to your application
#define BETA        0.0   // needs to be tuned according to your application
unsigned long start_time;


uint8_t loops = 0;
uint8_t txBuffer[20];  //Sends data array
uint8_t rxBuffer[20];  //Receive data array
uint8_t rxCnt = 0;     //Receive data count


uint8_t statusCode;
uint8_t rxByte;

uint8_t getCheckSum(uint8_t *buffer, uint8_t len);
void speedModeRun(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc);
uint8_t waitingForACK(uint8_t len);
void blinkLight(uint8_t times);
long power5(float i);
float applyActualRates(const int axis, float rcCommandf, const float rcCommandfAbs);

PWM panServoPWM(2);  // Setup pin 2 for pan PWM
PWM tiltServoPWM(3); // Setup pin 3 for tilt PWM

void positionMode1Run(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc, int pulses) {
  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0xFD;                                //function code
  txBuffer[3] = (dir << 7) | ((speed >> 8) & 0x0F);  //High 4 bits for direction and speed
  txBuffer[4] = speed & 0x00FF;                      //8 bits lower
  txBuffer[5] = acc;                                 //acceleration
  txBuffer[6] = 0x00;
  txBuffer[7] = 0x00;
  txBuffer[8] = 0xFF;
  txBuffer[9] = 0x00;       
  txBuffer[10] = getCheckSum(txBuffer, 10);            //Calculate checksum

  Serial.write(txBuffer, 11);
}


void positionMode4Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position) {
  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0xF5;                                //function code
  txBuffer[3] = speed >> 8;  
  txBuffer[4] = speed & 0x00FF;                      //8 bits lower
  txBuffer[5] = acc;                                 //acceleration
  txBuffer[6] = position & 0xFF000000 >> 24;
  txBuffer[7] = position & 0x00FF0000 >> 16;
  txBuffer[8] = position & 0x0000FF00 >> 8;
  txBuffer[9] = position & 0x000000FF;       
  txBuffer[10] = getCheckSum(txBuffer, 10);            //Calculate checksum

  Serial.write(txBuffer, 11);
}

void setup() {
  // Set the LED light port as output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // setup filter
  // f.begin(FREQUENCY, MINCUTOFF, BETA);

  // Start the serial port, set the rate to 38400
  Serial.begin(38400);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  blinkLight(1);
  delay(500);

  // Start listening for PWM HIGH
  panServoPWM.begin(true);
  tiltServoPWM.begin(true);

  start_time = micros();

  // positionMode1Run(1, 1, 3000, 0, 0);

  // delay(10000);
  // positionMode1Run(1, 0, 3000, 0, 0);

  positionMode4Run(1, 1000, 254, 250);
  delay(250);
  positionMode4Run(1, 500, 254, 0);
  delay(250);
  positionMode4Run(1, 2000, 254, 250);
  delay(250);
  positionMode4Run(1, 300, 254, 0);
  delay(250);
  positionMode4Run(1, 3000, 254, 250);
  delay(250);
  positionMode4Run(1, 50, 254, 0);
  delay(250);
  positionMode4Run(1, 2000, 254, 250);
  delay(250);
  positionMode4Run(1, 3000, 128, 0);

}

void loop() {

  uint8_t ackStatus;
  uint16_t panRC;
  uint16_t tiltRC;

  panRC = panServoPWM.getValue();
  tiltRC = tiltServoPWM.getValue();

  // float elapsed_time = 1E-6 * (micros() - start_time); // in seconds

  // float pan_filtered_signal = f.filter(panRC, elapsed_time) ;
  // float tilt_filtered_signal = f.filter(tiltRC, elapsed_time) ;

  // Serial.print("panRC:");
  // Serial.print(panRC, DEC);
  // Serial.print(",");
  // // Serial.print("pan_filtered_signal:");
  // // Serial.println(pan_filtered_signal, 3);
  // // Serial.print(",");
  // Serial.print("tiltRC:");
  // Serial.println(tiltRC, DEC);
  // Serial.print(",");
  // Serial.print("tilt_filtered_signal:");
  // Serial.println(tilt_filtered_signal,3);
 

  uint8_t panRunDir = 0;
  int16_t panRunSpeed = (1500 - panRC) * 6;

  if (panRunSpeed < 0) {
    panRunDir = 0;
  } else {
    panRunDir = 1;
  }

  uint16_t panAbsRunSpeed = min(abs(panRunSpeed), 3000);

  uint8_t tiltRunDir = 0;
  int16_t tiltRunSpeed = (1500 - tiltRC) * 6;

  if (tiltRunSpeed < 0) {
    tiltRunDir = 0;
  } else {
    tiltRunDir = 1;
  }

  uint16_t tiltAbsRunSpeed = min(abs(tiltRunSpeed), 3000);

  // Serial.print(">panRunSpeed:");
  // Serial.println(panRunSpeed);
  // Serial.print(">tiltRunSpeed:");
  // Serial.println(tiltRunSpeed);


  // speedModeRun(1, panRunDir, panAbsRunSpeed, 0);
  // speedModeRun(2, tiltRunDir, tiltAbsRunSpeed, 0);


  // uint16_t expo = 80;
  // uint16_t centerStick = 120;
  // uint16_t maxRate = 500;

  // uint16_t angleRate = applyActualRates(expo, centerStick, maxRate, stickPos, abs(stickPos));

  //   ackStatus = waitingForACK(5);      //Wait for the motor to answer
//   if (Serial.available() > 0)  //The serial port receives data
//     {
//       rxByte = Serial.read();  //read 1 byte data
//       if (rxCnt != 0) {
//         rxBuffer[rxCnt++] = rxByte;  //Storing data
//       } else if (rxByte == 0xFB)     //Determine whether the frame header
//       {
//         rxBuffer[rxCnt++] = rxByte;  //store frame header
//       }
//     }

//     uint8_t len = 5;

//     if (rxCnt == len)  //Receive complete at 5 bytes
//     {
//       if (rxBuffer[len - 1] == getCheckSum(rxBuffer, len - 1)) {
//         statusCode = rxBuffer[3];  //checksum correct
//       } else {
//         rxCnt = 0;  //Verification error, re-receive the response
//       }
//     }

//   if (statusCode != 1) {
//     blinkLight(ackStatus + 1);
//   }

  loops++;
  delay(10);
}




long power5(float i) {
  double ans = pow((double)i, 5);
  Serial.print(",ans:");
  Serial.print(ans);
  return ans;
}


float applyActualRates(uint16_t rcExpo, uint16_t rcRates, uint16_t rates, float rcCommandf, float rcCommandfAbs) {
  Serial.print("rcExpo:");
  Serial.print(rcExpo);
  Serial.print(",rcRates:");
  Serial.print(rcRates);
  Serial.print(",rates:");
  Serial.print(rates);
  Serial.print(",rcCommandf:");
  Serial.print(rcCommandf);
  Serial.print(",rcCommandfAbs:");
  Serial.print(rcCommandfAbs);

  double expof = 0.0008f;
  Serial.print(",expof:");
  Serial.print(expof);

  expof = rcCommandfAbs * (power5(rcCommandf) * expof + rcCommandf * (1 - expof));
  Serial.print(",power5:");
  Serial.print(power5(rcCommandf));

  Serial.print(",chonker:");
  Serial.print(expof + rcCommandf * (1 - expof));

  Serial.print(",rcCommandfAbs:");
  Serial.print(rcCommandfAbs);

  const float centerSensitivity = rcRates * 10.0f;
  Serial.print(",centerSensitivity:");
  Serial.print(centerSensitivity);

  const float stickMovement = max(0, rates * 10.0f - centerSensitivity);
  Serial.print(",stickMovement:");
  Serial.print(stickMovement);

  const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;
  Serial.print(",angleRate:");
  Serial.println(angleRate);

  return angleRate;
}

/*
Function: Serial port sends speed mode operation command
Input: slaveAddr slave address
       dir running direction
       speed running speed
       acc acceleration
*/
void speedModeRun(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc) {
  int i;
  uint16_t checkSum = 0;

  txBuffer[0] = 0xFA;                                //frame header
  txBuffer[1] = slaveAddr;                           //slave address
  txBuffer[2] = 0xF6;                                //function code
  txBuffer[3] = (dir << 7) | ((speed >> 8) & 0x0F);  //High 4 bits for direction and speed
  txBuffer[4] = speed & 0x00FF;                      //8 bits lower
  txBuffer[5] = acc;                                 //acceleration
  txBuffer[6] = getCheckSum(txBuffer, 6);            //Calculate checksum

  Serial.write(txBuffer, 7);
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

float calculateAverage(int arr[], int size) {
  long sum = 0;  // Use long to prevent overflow for large sums

  // Sum all elements in the array
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }

  return (uint16_t)sum / size;
}

/*
Function: Wait for the response from the lower computer, set the timeout time to 3000ms
Input: len Length of the response frame
output:
   Run successfully 1
   failed to run 0
   timeout no reply 0
*/
uint8_t waitingForACK(uint8_t len) {
  uint8_t retVal;       //return value
  unsigned long sTime;  //timing start time
  unsigned long time;   //current moment
  uint8_t rxByte;

  sTime = millis();  //get the current moment
  rxCnt = 0;         //Receive count value set to 0
  while (1) {
    if (Serial.available() > 0)  //The serial port receives data
    {
      rxByte = Serial.read();  //read 1 byte data
      if (rxCnt != 0) {
        rxBuffer[rxCnt++] = rxByte;  //Storing data
      } else if (rxByte == 0xFB)     //Determine whether the frame header
      {
        rxBuffer[rxCnt++] = rxByte;  //store frame header
      }
    }

    if (rxCnt == len)  //Receive complete
    {
      if (rxBuffer[len - 1] == getCheckSum(rxBuffer, len - 1)) {
        retVal = rxBuffer[3];  //checksum correct
        break;                 //exit while(1)
      } else {
        rxCnt = 0;  //Verification error, re-receive the response
      }
    }

    time = millis();
    if ((time - sTime) > 500)  //Judging whether to time out
    {
      retVal = 5;
      break;  //timeout, exit while(1)
    }
  }
  return (retVal);
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
