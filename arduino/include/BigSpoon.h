#include <stdint.h>

// Serial data buffers for RS-485 packets between the motor
extern uint8_t txBuffer[20];
extern uint8_t rxBuffer[20];
extern uint8_t rxCnt;

// Timestamps for filters
extern unsigned long start_time;
extern double elapsedTimeInSeconds;

// PWM pulse width in microseconds
extern int panPWMValue;
extern int tiltPWMValue;

// Setpoint location raw and filtered values on the encoder 
extern long panPosition;
extern long filteredPanPosition;

extern long tiltPosition;
extern long filteredTiltPosition;

// Initial encoder values to perform "center on startup"
extern long panStartingEncoderValue;
extern long panEncoderValue;

extern long tiltStartingEncoderValue;
extern long tiltEncoderValue;

// RC raw and filter values from the PWM
extern int16_t panRC;
extern int16_t filteredPanRC;

extern int16_t tiltRC;
extern int16_t filteredTiltRC;

// Proportional (P) terms
extern uint16_t panPTerm;
extern uint16_t tiltPTerm;

// Error values (distance between setpoint and encoder value)
extern long panError;
extern long tiltError;

// Maximum and minimum range in motor steps the motor will turn
extern long maxPanAngle;
extern long minPanAngle;

extern long maxTiltAngle;
extern long minTiltAngle;

// Scalar value for rates
extern float rcPanScale;
extern float rcTiltScale;