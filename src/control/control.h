#ifndef WSC_CONTROL_H
#define WSC_CONTROL_H

#include <Arduino.h>
#include "system_api.h"

// Initialize and start the control FreeRTOS task.
void controlBegin();

// Enable / disable automatic cascaded control
void controlEnable(bool enabled);
bool controlIsEnabled();

// Set/get target yaw (degrees, -180..180)
void controlSetTargetYaw(float yaw);
float controlGetTargetYaw();

// PID tuning
void controlSetAnglePID(float kp, float ki, float kd);
void controlSetRatePID(float kp, float ki, float kd);

// Model-based phi PID (from control_PID.cpp) - setup & tuning
void controlSetModelPhiGains(float kp, float ki, float kd);
void controlSetModelDeltaLimits(float max_rad, float soft_rad);
void controlSetModelTauDelta(float tau_seconds);

// Diagnostics / telemetry for model output (delta = control surface deflection)
float controlGetLastDeltaRaw();      // radians (raw before filtering)
float controlGetLastDeltaFiltered(); // radians (after tau_delta filter)

// Diagnostics / telemetry
float controlGetLastDesiredRate();
float controlGetLastRateCommand();

/* your algoiwefjsd */


#endif // WSC_CONTROL_H