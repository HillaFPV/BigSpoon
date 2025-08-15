#include <math.h>
#include "PWM.hpp"
#include "1euroFilter.h"
#include "MotorUtils.h"
#include "Debug.h"
#include "filter_lib.h"


// Filters and parameters
#define FREQUENCY   100     // [Hz] Frequency of your incoming noisy data
#define MINCUTOFF   1.00
#define BETA        1.00
#define DCUTOFF     1.00

// Hey girl, you like filters?
static OneEuroFilter panRCFilter;
static OneEuroFilter tiltRCFilter;

static OneEuroFilter panPositionFilter;
static OneEuroFilter tiltPositionFilter;

lowpass_filter panLowpassFilter(3); 
lowpass_filter tiltLowpassFilter(3);

// PWM on digital interrupt pins 2 and 3
PWM panServoPWM(2);
PWM tiltServoPWM(3);

// Serial data buffers for RS-485 packets between the motor
uint8_t txBuffer[20];
uint8_t rxBuffer[20];
uint8_t rxCnt = 0;

// Timestamps for filters
unsigned long start_time;
double elapsedTimeInSeconds  = 0.0;

// PWM pulse width in microseconds
int panPWMValue;
int tiltPWMValue;

// Setpoint location raw and filtered values on the encoder 
long panPosition;
long filteredPanPosition;

long tiltPosition;
long filteredTiltPosition;

// Initial encoder values to perform "center on startup"
long panStartingEncoderValue;
long panEncoderValue;

long tiltStartingEncoderValue;
long tiltEncoderValue;

// RC raw and filter values from the PWM
int16_t panRC;
int16_t filteredPanRC;

int16_t tiltRC;
int16_t filteredTiltRC;

// Proportional (P) terms
uint16_t panPTerm;
uint16_t tiltPTerm;

// Error values (distance between setpoint and encoder value)
long panError;
long tiltError;

// Maximum and minimum range in motor steps the motor will turn
long maxPanAngle = 51200;
long minPanAngle = -51200;

long maxTiltAngle = 10240;
long minTiltAngle = -10240;

// Scalar value for rates
float rcPanScale = 3.0;
float rcTiltScale = 3.0;

void setup() {
  // Setup filters
  panRCFilter.begin(FREQUENCY, MINCUTOFF, 1, 1);
  tiltRCFilter.begin(FREQUENCY, MINCUTOFF, BETA, DCUTOFF);
  panPositionFilter.begin(FREQUENCY, MINCUTOFF, BETA, DCUTOFF);
  tiltPositionFilter.begin(FREQUENCY, MINCUTOFF, BETA, DCUTOFF);

  // Start the serial ports and wait for connect
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial || !Serial1) {
    ; 
  }
  
  // Start listening for PWM HIGH
  panServoPWM.begin(true);
  tiltServoPWM.begin(true);

  // Get initial encoder value from motors
  // TODO: If the motor doesn't respond (powered off) this will
  //       hang forever. We should implement a timeout on `getEncoderValue`
  //       and run this in a loop until a connection is established.
  //       otherwise, the motors must be powered and booted up
  //       before the Arduino.
  panStartingEncoderValue = getEncoderValue(1);
  panPosition = panStartingEncoderValue;
  panEncoderValue = panStartingEncoderValue;

  tiltStartingEncoderValue = getEncoderValue(2);
  tiltPosition = tiltStartingEncoderValue;
  tiltEncoderValue = tiltStartingEncoderValue;

  // Keep track of start time for filters
  start_time = micros();
}

void loop() {
  // Check if PWM signal is out of range.
  // if so, then ELRS isn't connected. Don't run the loop
  // until we get RC link.
  panPWMValue = panServoPWM.getValue();
  if (panPWMValue < 900 || panPWMValue > 2100) {
    return;
  }
  tiltPWMValue = tiltServoPWM.getValue();
  if (tiltPWMValue < 900 || tiltPWMValue > 2100) {
    return;
  }

  // Squash out noise around center stick
  if (panPWMValue > 1450 && panPWMValue < 1550) {
    panPWMValue = ((panPWMValue - 1500) / 10) + 1500;
  }
  if (tiltPWMValue > 1450 && tiltPWMValue < 1550) {
    tiltPWMValue = ((tiltPWMValue - 1500) / 10) + 1500;
  }   
  
  // Squash more noise with a lowpass filter the PWM input
  panPWMValue = panLowpassFilter.filter(panPWMValue);
  tiltPWMValue = tiltLowpassFilter.filter(tiltPWMValue);

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
  
  filteredPanPosition = panPositionFilter.filter(panPosition, elapsedTimeInSeconds);
  filteredTiltPosition = tiltPositionFilter.filter(tiltPosition, elapsedTimeInSeconds);

  panEncoderValue = getEncoderValue(1);
  tiltEncoderValue = getEncoderValue(2);

  panError = panEncoderValue - filteredPanPosition;
  int panDirection = 0;
  if (panError < 0) {
    panDirection = 1;
  }

  tiltError = tiltEncoderValue - filteredTiltPosition;
  int tiltDirection = 0;
  if (tiltError < 0) {
    tiltDirection = 1;
  }
  
  // Define a proportional (P) term to bring the setpoint/encoder error to 0.
  panPTerm = min(abs(panError * 0.05), 2000);
  tiltPTerm = min(abs(tiltError * 0.05), 2000);
  
  // Command the motors to run according to the P-term
  speedModeRun(1, panDirection, panPTerm, 0);
  speedModeRun(2, tiltDirection, tiltPTerm, 0);

  // debug();

  // Finished the loop. Note the total time it took to run so the filter can
  // infer Hz
  elapsedTimeInSeconds = 1E-6 * (micros() - start_time);
}
