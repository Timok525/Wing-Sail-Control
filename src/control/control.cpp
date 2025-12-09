#include "control.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <cmath>

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
// (Legacy cascaded PIDs left here for optional tuning/backwards compatibility)
static PID anglePID(2.0f, 0.05f, 0.02f, -360.0f, 360.0f, -1000.0f, 1000.0f);
static PID ratePID(0.6f, 0.02f, 0.0f, -10.0f, 10.0f, -50.0f, 50.0f);

static float maxDesiredRate = 120.0f;     // deg/s (cascaded fallback)
static float maxServoDeltaPerStep = 6.0f; // deg per loop (cascaded fallback)

// Diagnostics (exposed read-only)
static volatile float lastDesiredRate = 0.0f; // reused for legacy/telemetry
static volatile float lastRateCmd = 0.0f;
// --- Model-based phi PID parameters (from control_PID.cpp)
struct ModelParams {
  float Kp_phi = 4.55f;   // proportional (phi)
  float Kd_phi = 7.6f;    // derivative (dphi)
  float Ki_phi = 0.53f;   // integrator multiplier applied to phi integral

  float phi_int_max = 0.9f;      // rad
  float phi_int_thresh = 0.175f; // rad
  float phi_error_dead = 0.035f; // rad deadband

  float delta_max = 1.5f;  // rad
  float delta_soft = 1.2f; // rad (soft limit)
  float tau_delta = 2.0f;  // seconds, first-order filter time constant

  // integrator shaping gain for phi integral (as in control_PID.cpp d_phi_int = phi_error * 0.6)
  float phi_int_gain = 0.6f;

  // mapping from delta (rad) to servo degrees (scale). Default uses 1 rad -> 57.2958 deg
  float servo_scale = 180.0f / M_PI;
  float servo_center_deg = 90.0f; // absolute servo center angle
} modelParams;

// Model controller state
static float phiIntegral = 0.0f;      // discrete integrator state (rad)
static float deltaDelayed = 0.0f;     // filtered delta (rad)
static volatile float lastDeltaRaw = 0.0f;      // radians
static volatile float lastDeltaFiltered = 0.0f; // radians

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

    // --- Model-based phi PID controller (port from control_PID.cpp)
    // Convert sensor angles (degrees) -> radians for model computations
    const float DEG2RAD = M_PI / 180.0f;
    const float RAD2DEG = 180.0f / M_PI;
    float phi = data.yawAngle * DEG2RAD;    // measured yaw in rad (phi)
    float dphi = data.gyroRate * DEG2RAD;   // yaw rate in rad/s
    float targetRad = runningTarget * DEG2RAD; // target yaw in rad

    // phi error (simulation uses phi - target)
    float phiError = phi - targetRad;
    if (fabs(phiError) < modelParams.phi_error_dead) phiError = 0.0f;

    // phi integrator derivative (discrete approximation)
    float dPhiInt = 0.0f;
    if (fabs(phiError) <= modelParams.phi_int_thresh && phiError != 0.0f) {
      // only integrate when integrator is not already marching away from error (same sign)
      if (!((phiError > 0.0f && phiIntegral > 0.0f) || (phiError < 0.0f && phiIntegral < 0.0f))) {
        dPhiInt = phiError * modelParams.phi_int_gain;
      }
    }
    // integrate
    phiIntegral += dPhiInt * dt;
    // clamp
    if (phiIntegral > modelParams.phi_int_max) phiIntegral = modelParams.phi_int_max;
    if (phiIntegral < -modelParams.phi_int_max) phiIntegral = -modelParams.phi_int_max;

    // compute PID components (units: radians or rad/s)
    float phi_p = modelParams.Kp_phi * phiError;
    float phi_d = modelParams.Kd_phi * dphi;
    float phi_i = modelParams.Ki_phi * phiIntegral;

    // delta (control surface deflection) raw command in radians
    float deltaRaw = phi_p + phi_d + phi_i;

    // soft-saturate delta (match soft_saturate behavior from the model)
    auto soft_saturate = [](float u, float max_u, float soft_u) {
      if (max_u <= soft_u) soft_u = max_u * 0.7f;
      if (fabs(u) <= soft_u) return u;
      float k = (fabs(u) - soft_u) / (max_u - soft_u);
      k = tanh(k * 5.0f);
      return copysignf(1.0f, u) * (soft_u + k * (max_u - soft_u));
    };

    deltaRaw = soft_saturate(deltaRaw, modelParams.delta_max, modelParams.delta_soft);
    lastDeltaRaw = deltaRaw;

    // Apply first-order filter (tau_delta)
    float alpha = dt / (modelParams.tau_delta + dt); // discrete low-pass
    deltaDelayed += (deltaRaw - deltaDelayed) * alpha;
    lastDeltaFiltered = deltaDelayed;

    // Convert delta (rad) -> servo degrees using servo_scale and center
    float deltaDeg = deltaDelayed * modelParams.servo_scale;
    float servoAngleF = modelParams.servo_center_deg + deltaDeg;
    if (servoAngleF < 0.0f) servoAngleF = 0.0f;
    if (servoAngleF > 180.0f) servoAngleF = 180.0f;
    int servoAngle = (int)roundf(servoAngleF);

    // apply to actuator
    setServoAngle(servoAngle);
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
      // reset model-internal state
      phiIntegral = 0.0f;
      deltaDelayed = 0.0f;
      lastDeltaRaw = 0.0f;
      lastDeltaFiltered = 0.0f;
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

// Model tuning API
void controlSetModelPhiGains(float kp, float ki, float kd) {
  modelParams.Kp_phi = kp;
  modelParams.Ki_phi = ki;
  modelParams.Kd_phi = kd;
}
void controlSetModelDeltaLimits(float max_rad, float soft_rad) {
  modelParams.delta_max = max_rad;
  modelParams.delta_soft = soft_rad;
}
void controlSetModelTauDelta(float tau_seconds) {
  if (tau_seconds > 0.0f) modelParams.tau_delta = tau_seconds;
}

float controlGetLastDeltaRaw() { return lastDeltaRaw; }
float controlGetLastDeltaFiltered() { return lastDeltaFiltered; }
