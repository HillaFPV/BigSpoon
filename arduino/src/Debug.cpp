#include "Arduino.h"
#include "BigSpoon.h"

void debug() {
    Serial.print(">zero:");
    Serial.print(0);

    Serial.print(",panPWMValue:");
    Serial.print(panPWMValue);
    // Serial.print(",tiltPWMValue:");
    // Serial.print(tiltPWMValue);

    // Setpoint location raw and filtered values on the encoder 
    Serial.print(",panPosition:");
    Serial.print(panPosition);
    Serial.print(",filteredPanPosition:");
    Serial.print(filteredPanPosition);

    // Serial.print(",tiltPosition:");
    // Serial.print(tiltPosition);
    // Serial.print(",filteredTiltPosition:");
    // Serial.print(filteredTiltPosition);

    // Initial encoder values to perform "center on startup"
    Serial.print(",panStartingEncoderValue:");
    Serial.print(panStartingEncoderValue);
    Serial.print(",panEncoderValue:");
    Serial.print(panEncoderValue);

    // Serial.print(",tiltStartingEncoderValue:");
    // Serial.print(tiltStartingEncoderValue);
    // Serial.print(",tiltEncoderValue:");
    // Serial.print(tiltEncoderValue);

    // RC raw and filter values from the PWM
    Serial.print(",panRC:");
    Serial.print(panRC);
    Serial.print(",filteredPanRC:");
    Serial.print(filteredPanRC);

    // Serial.print(",tiltRC:");
    // Serial.print(tiltRC);
    // Serial.print(",filteredTiltRC:");
    // Serial.print(filteredTiltRC);

    // Proportional (P) terms
    Serial.print(",panPTerm:");
    Serial.print(panPTerm);
    // Serial.print(",tiltPTerm:");
    // Serial.print(tiltPTerm);

    // Error values (distance between setpoint and encoder value)
    Serial.print(",panError:");
    Serial.print(panError);
    // Serial.print(",tiltError:");
    // Serial.print(tiltError);

    Serial.println();
}