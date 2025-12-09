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

// Debug control loop printing
void controlSetDebug(bool enable);
bool controlIsDebug();

// Flip control->actuator sign (useful if servo wiring or coordinate frames
// are reversed). When inverted is true, control-generated deltas will be
// negated before being applied to the servo.
void controlSetInvertOutput(bool invert);
bool controlIsOutputInverted();

// Limit the actuator angle relative to the center position (in degrees).
// The controller will clamp its next angle to center +/- maxOffset. Default will be set in the .cpp
void controlSetMaxAngleOffset(float maxOffset);
float controlGetMaxAngleOffset();

/* your algoiwefjsd */


#endif // WSC_CONTROL_H