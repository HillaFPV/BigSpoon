#include <math.h>
#include "PWM.hpp"
#include "1euroFilter.h"
#include "MotorUtils.h"

static OneEuroFilter panRCFilter;
static OneEuroFilter panPTermFilter;

static OneEuroFilter tiltRCFilter;
static OneEuroFilter tiltPTermFilter;

static OneEuroFilter panPositionFilter;
// Frequency of your incoming noisy data
// If you are able to provide timestamps, the frequency is automatically determined
#define FREQUENCY   1      // [Hz] 
#define MINCUTOFF   0.05    // Think D-term. 0.05-2.00 range is good
#define BETA        0.00005 // Think P-term. 0.00-0.005 range is good

unsigned long start_time;
long loops = 0;
uint8_t txBuffer[20];  //Sends data array
uint8_t rxBuffer[20];  //Receive data array
uint8_t rxCnt = 0;     //Receive data count

double elapsedTimeInSeconds  = 0.0;

long panPosition;
long filteredPanPosition;

long tiltPosition;
long filteredTiltPosition;

long panStartingEncoderValue;
long panEncoderValue;

long tiltStartingEncoderValue;
long tiltEncoderValue;

int16_t panRC;
int16_t filteredPanRC;
int16_t filteredPanPTerm;

int16_t tiltRC;
int16_t filteredTiltRC;
int16_t filteredTiltPTerm;

long maxPanAngle = 102400;
long minPanAngle = -102400;

long maxTiltAngle = 102400;
long minTiltAngle = -102400;

float rcPanScale = 3.0;
float rcTiltScale = 3.0;

PWM panServoPWM(2);
PWM tiltServoPWM(3);

void setup() {
  // Set the LED light port as output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // setup filters
  panRCFilter.begin(FREQUENCY, MINCUTOFF, BETA);
  panPTermFilter.begin(FREQUENCY, MINCUTOFF, BETA);
 
  tiltRCFilter.begin(FREQUENCY, MINCUTOFF, BETA);
  tiltPTermFilter.begin(FREQUENCY, MINCUTOFF, BETA);
  

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

  panStartingEncoderValue = getEncoderValue(1);
  panPosition = panStartingEncoderValue;
  panEncoderValue = panStartingEncoderValue;
 
  tiltStartingEncoderValue = 0; //getEncoderValue(2);
  tiltPosition = tiltStartingEncoderValue;
  tiltEncoderValue = tiltStartingEncoderValue;
}

void loop() {
  // Check if PWM signal is out of range.
  // if so, then ELRS isn't connected. Don't run the loop
  // until we get RC link.
  int panPWMValue = panServoPWM.getValue();
  if (panPWMValue < 900 || panPWMValue > 2100) {
    return;
  }

  int tiltPWMValue = tiltServoPWM.getValue();
  if (tiltPWMValue < 900 || tiltPWMValue > 2100) {
    return;
  }

  // 1500 is the center-stick value for these PWM signals. They range from 1000-2000us.
  panRC = (panPWMValue - 1500) * rcPanScale; 
  tiltRC = (tiltPWMValue - 1500) * rcTiltScale;

  filteredPanRC = panRCFilter.filter(panRC, elapsedTimeInSeconds);
  filteredTiltRC = tiltRCFilter.filter(tiltRC, elapsedTimeInSeconds);

  panPosition += filteredPanRC;
  tiltPosition += filteredTiltRC;

  // Set soft-limits for how far the servo can rotate CW or CCW
  if (panPosition < (panStartingEncoderValue + minPanAngle)) {
    panPosition = panStartingEncoderValue + minPanAngle;
  }
  else if (panPosition >= (panStartingEncoderValue + maxPanAngle)) {
    panPosition = panStartingEncoderValue + maxPanAngle;
  }

  if (tiltPosition < (tiltStartingEncoderValue + minTiltAngle)) {
    tiltPosition = tiltStartingEncoderValue + minTiltAngle;
  }
  else if (tiltPosition >= (tiltStartingEncoderValue + maxTiltAngle)) {
    tiltPosition = tiltStartingEncoderValue + maxTiltAngle;
  }

  panEncoderValue = getEncoderValue(1);
  tiltEncoderValue = 0; //getEncoderValue(2);
  
  long filteredPanPosition = panPositionFilter.filter(panPosition, elapsedTimeInSeconds);

  long panError = panEncoderValue - filteredPanPosition;
  int panDirection = 0;
  if (panError < 0) {
    panDirection = 1;
  }

  long tiltError = tiltEncoderValue - tiltPosition;
  int tiltDirection = 0;
  if (tiltError < 0) {
    tiltDirection = 1;
  }
  
  uint16_t panPTerm = min(abs(panError * 0.05), 2000);
  // filteredPanPTerm = panPTermFilter.filter(panPTerm, elapsedTimeInSeconds);
  // uint16_t tiltPTerm = min(abs(tiltError * 0.10), 2000);
  // filteredTiltPTerm = tiltPTermFilter.filter(tiltPTerm, elapsedTimeInSeconds);

  // Input = panEncoderValue;
  // Setpoint = filteredPanPosition;
  // myPID.Compute();

  // Serial.print(">panPTerm:");
  // Serial.print(panPTerm);
  // Serial.print(",panPWMValue:");
  // Serial.print(panPWMValue);
  // Serial.print(",panError:");
  // Serial.print(panError);  
  // Serial.print(",panRC:");
  // Serial.print(panRC);
  // Serial.print(",panPosition:");
  // Serial.print(panPosition);
  // Serial.print(",panEncoderValue:");
  // Serial.print(panEncoderValue);
  // Serial.print(",filteredPanRC:");
  // Serial.println(filteredPanRC);

  // Serial.print(",tiltPTerm:");
  // Serial.print(tiltPTerm);
  // Serial.print(",tiltError:");
  // Serial.print(tiltError);
  // Serial.print(",tiltRC:");
  // Serial.print(tiltRC);
  // Serial.print(",tiltPosition:");
  // Serial.print(tiltPosition);
  // Serial.print(",tiltEncoderValue:");
  // Serial.print(tiltEncoderValue);
  // Serial.print(",filteredTiltRC:");
  // Serial.print(filteredTiltRC);
  // Serial.print(",filteredTiltPTerm:");
  // Serial.println(filteredTiltPTerm);

  speedModeRun(1, panDirection, panPTerm, 0);
  //speedModeRun(2, tiltDirection, tiltPTerm, 0);
  
  loops++;
  elapsedTimeInSeconds = 1E-6 * (micros() - start_time);

}
