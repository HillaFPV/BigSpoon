#include "PWM.hpp"
#include <math.h>
#include "1euroFilter.h"
#include "MotorUtils.h"

static OneEuroFilter panRCFilter;
static OneEuroFilter panPTermF;
// Frequency of your incoming noisy data
// If you are able to provide timestamps, the frequency is automatically determined
#define FREQUENCY   1   // [Hz] 
#define MINCUTOFF   1.0   // Think D-term. 0.05-2.00 range is good
#define BETA        0.0   // Think P-term. 0.00-0.005 range is good

#define RCPANSCALE = 3.8;

unsigned long start_time;
uint8_t loops = 0;
uint8_t txBuffer[20];  //Sends data array
uint8_t rxBuffer[20];  //Receive data array
uint8_t rxCnt = 0;     //Receive data count

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
float rcPanScale = 3.8;

PWM panServoPWM(2);  // Setup pin 2 for pan PWM
PWM tiltServoPWM(3); // Setup pin 3 for tilt PWM

void setup() {
  // Set the LED light port as output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // setup filters
  panRCFilter.begin(10, 0.05, 0.00005);
  panPTermF.begin(10, 0.05, 0.00005);
  
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
  
  int32_t motorBootCode = getEncoderValue(1);
  panPosition = motorBootCode;
  encoderValue = motorBootCode;
  bootUpEncoderValue = motorBootCode;
}

void loop() {
  /* Check if PWM signal is out of range.
     if so, then ELRS isn't connected. Don't run the loop
     until we get RC link.
    */
  int panPWMValue = panServoPWM.getValue();
  if (panPWMValue < 900 || panPWMValue > 2100) {
    return;
  }

  panRC = (panPWMValue - 1500) * rcPanScale; // 1500 is the center-stick value for these PWM signals. They range from 1000-2000us.
  
  elapsedTimeInSeconds = 1E-6 * (micros() - start_time);
  filteredPanRC = panRCFilter.filter(panRC, elapsedTimeInSeconds);

  panPosition += filteredPanRC;

  // Set soft-limits for how far the servo can rotate CW or CCW
  if (panPosition < (bootUpEncoderValue + minAngle)) {
    panPosition = bootUpEncoderValue + minAngle;
  }
  else if (panPosition >= (bootUpEncoderValue + maxAngle)) {
    panPosition = bootUpEncoderValue + maxAngle;
  }

  encoderValue = getEncoderValue(1);
  
  long error = encoderValue - panPosition;
  int dir = 0;
  if (error < 0) {
    dir = 1;
  }
  
  uint16_t pTerm = min(abs(error * 0.10), 2000);
  elapsedTimeInSeconds = 1E-6 * (micros() - start_time);
  filteredPTerm = panPTermF.filter(pTerm, elapsedTimeInSeconds);
  
  speedModeRun(1, dir, pTerm, 0);

  loops++;
}
