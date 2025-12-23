// Common shared types and system interface used by main and control component
#ifndef WSC_SYSTEM_API_H
#define WSC_SYSTEM_API_H

#include <Arduino.h>

// Sensor data structure used by IMU/MPU task and control component
typedef struct SensorData {
  float yawAngle;        // Filtered yaw angle (degrees, -180..180)
  float rawYaw;          // Raw integrated yaw (degrees, -180..180)
  float gyroRate;        // Angular velocity (deg/s)
  float pitchAngle;      // Pitch angle (degrees) from accelerometer
  float rollAngle;       // Roll angle (degrees) from accelerometer
  float dt;              // Time step (seconds)
  unsigned long timestamp; // Microsecond timestamp
  bool valid;            // Data validity flag
} SensorData;

// System API - functions implemented in main.cpp that control component will call
// Read current sensor snapshot (thread-safe copy)
SensorData getSensorData();

// Get current servo angle (0..180) in a thread-safe way
int getCurrentServoAngle();

// Set servo target angle (0..180) in a thread-safe way
void setServoAngle(int angle);

// Return whether MPU has been successfully initialized
bool mpuIsInitialized();

#endif // WSC_SYSTEM_API_H
