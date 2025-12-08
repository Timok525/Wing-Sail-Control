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

// Diagnostics / telemetry
float controlGetLastDesiredRate();
float controlGetLastRateCommand();

/* your algoiwefjsd */


#endif // WSC_CONTROL_H