#include "control.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Control loop configuration
static const TickType_t CONTROL_LOOP_MS = pdMS_TO_TICKS(20); // 50 Hz
static const int CONTROL_TASK_STACK = 8192; // words/bytes depends on port, match project
static const int CONTROL_TASK_PRIORITY = 2; // same as MPU task

// PID controller with simple anti-windup
class PID {
public:
  float kp, ki, kd;
  float integrator;
  float prevError;
  bool first;
  float outMin, outMax;
  float intMin, intMax;

  PID(float p=0, float i=0, float d=0, float oMin=-1e6f, float oMax=1e6f, float iMin=-1e6f, float iMax=1e6f)
    : kp(p), ki(i), kd(d), integrator(0), prevError(0), first(true), outMin(oMin), outMax(oMax), intMin(iMin), intMax(iMax) {}

  void reset() {
    integrator = 0.0f;
    prevError = 0.0f;
    first = true;
  }

  void setGains(float p, float i, float d) { kp = p; ki = i; kd = d; }

  float update(float error, float dt) {
    if (dt <= 0.0f) dt = 0.02f;

    integrator += error * dt;
    if (integrator > intMax) integrator = intMax;
    if (integrator < intMin) integrator = intMin;

    float derivative = 0.0f;
    if (!first) derivative = (error - prevError) / dt;
    prevError = error;
    first = false;

    float out = kp * error + ki * integrator + kd * derivative;
    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;
    return out;
  }
};

// Helper: shortest angular difference (target - current) -> [-180, 180]
static float angleDiff(float target, float current) {
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// Internal control state
static SemaphoreHandle_t ctrlMutex = NULL;
static bool enabled = false;
static float targetYaw = 0.0f;

// Defaults (tune to your mechanical system)
static PID anglePID(2.0f, 0.05f, 0.02f, -360.0f, 360.0f, -1000.0f, 1000.0f);
static PID ratePID(0.6f, 0.02f, 0.0f, -10.0f, 10.0f, -50.0f, 50.0f);

static float maxDesiredRate = 120.0f;     // deg/s
static float maxServoDeltaPerStep = 6.0f; // deg per loop

// Diagnostics (exposed read-only)
static volatile float lastDesiredRate = 0.0f;
static volatile float lastRateCmd = 0.0f;

// Control task implementation
static void controlTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // Sleep until next cycle
    vTaskDelayUntil(&lastWake, CONTROL_LOOP_MS);

    // Read configuration under lock
    bool run = false;
    float runningTarget = 0.0f;
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
      run = enabled;
      runningTarget = targetYaw;
      xSemaphoreGive(ctrlMutex);
    }

    if (!run) continue;

    // Get snapshot of sensors
    SensorData data = getSensorData();
    if (!data.valid) continue; // skip if sensor not ready

    float dt = data.dt;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f; // protect against invalid dt

    // Outer loop -> desired rate (deg/s)
    float angleError = angleDiff(runningTarget, data.yawAngle);
    float desiredRate = anglePID.update(angleError, dt);
    if (desiredRate > maxDesiredRate) desiredRate = maxDesiredRate;
    if (desiredRate < -maxDesiredRate) desiredRate = -maxDesiredRate;
    lastDesiredRate = desiredRate;

    // Inner loop -> servo delta (deg)
    float rateError = desiredRate - data.gyroRate;
    float servoDelta = ratePID.update(rateError, dt);
    if (servoDelta > maxServoDeltaPerStep) servoDelta = maxServoDeltaPerStep;
    if (servoDelta < -maxServoDeltaPerStep) servoDelta = -maxServoDeltaPerStep;
    lastRateCmd = servoDelta;

    // Apply actuator change incrementally
    int current = getCurrentServoAngle();
    float nextAngleF = (float)current + servoDelta;
    if (nextAngleF < 0.0f) nextAngleF = 0.0f;
    if (nextAngleF > 180.0f) nextAngleF = 180.0f;
    int nextAngle = (int)roundf(nextAngleF);

    setServoAngle(nextAngle);
  }
}

// Public API
void controlBegin() {
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  // Create the RTOS task if not already present
  xTaskCreate(controlTask, "ControlTask", CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIORITY, NULL);
}

void controlEnable(bool e) {
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10))) {
    enabled = e;
    // Reset internal integrators when enabling to avoid sudden transients
    if (enabled) {
      anglePID.reset();
      ratePID.reset();
    }
    xSemaphoreGive(ctrlMutex);
  }
}

bool controlIsEnabled() {
  bool e = false;
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) { e = enabled; xSemaphoreGive(ctrlMutex); }
  return e;
}

void controlSetTargetYaw(float yaw) {
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  // Normalize yaw within [-180,180]
  while (yaw > 180.0f) yaw -= 360.0f;
  while (yaw < -180.0f) yaw += 360.0f;
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10))) { targetYaw = yaw; xSemaphoreGive(ctrlMutex); }
}

float controlGetTargetYaw() {
  float y = 0.0f;
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) { y = targetYaw; xSemaphoreGive(ctrlMutex); }
  return y;
}

void controlSetAnglePID(float kp, float ki, float kd) { anglePID.setGains(kp, ki, kd); }
void controlSetRatePID(float kp, float ki, float kd)  { ratePID.setGains(kp, ki, kd); }

float controlGetLastDesiredRate() { return lastDesiredRate; }
float controlGetLastRateCommand()  { return lastRateCmd; }
