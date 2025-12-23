#include "control.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Control loop configuration
static const TickType_t CONTROL_LOOP_MS = pdMS_TO_TICKS(20); // 50 Hz
static const int CONTROL_TASK_STACK = 8192; // words/bytes depends on port, match project
static const int CONTROL_TASK_PRIORITY = 2; // same as MPU task

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Parameter structure from controller_PID.cpp
struct Params {
    // Basic physical parameters
    double x_OA = 0.1;    double y_OA = 0.0;    double z_OA = -1.0;
    double x_OR = 1.0;    double y_OR = 0.0;    double z_OR = -0.7;
    double z_OC = -0.3;   double C_SC = 1.8;    double C_RC = 0.8;
    double S_S = 1.8;     double S_R = 0.6;     double m_T = 0.005;
    double m_C = 1.2;     double g = 9.81;      double J_z = 0.09;
    double l = 0.3;       double V_S = 2.5;     double p_s = 1.225;

    // Phi threshold
    double phi_thresh = 25.0 * M_PI / 180.0;
    double coupling_min = 0.01;
    double coupling_gain = 0.01;

    // Limits
    double alpha_max = 0.5;    double beta_max = 0.5;     double phi_max = 1.5;
    double d_alpha_base = 0.08; 
    double d_beta_base = 0.05; 
    double d_phi_max = 0.2;
    double bound_ratio = 0.4;

    // Smoothing
    double tau_smooth_alpha = 0.15; 
    double tau_smooth_beta = 0.2;   

    // PID Gains
    double Kp_alpha = 0.18;  double Kd_alpha = 0.2;   double Ki_alpha = 0.07;  
    double Kp_beta = 0.08;   double Kd_beta = 0.1;    double Ki_beta = 0.03;   
    
    // Phi Gains (Main Control)
    double Kp_phi = 4.55;    double Kd_phi = 7.6;     double Ki_phi = 0.53;    

    // Integral limits
    double phi_int_max = 0.9;     double phi_int_thresh = 0.175;
    // Deadband: Reduced to ~0.5 degrees to improve centering accuracy
    double phi_error_dead = 0.008; // was 0.035 (2 deg)

    // Output limits
    double delta_max = 1.5;  double delta_soft = 1.2;

    // Delay/Noise (Simulation only, unused here)
    double tau_delta = 2.0;       double delta_noise_amp = 0.005;

    // Target
    double phi_target = 0.0;

    // Damping
    double alpha_damping_gain = 0.3;
    double beta_damping_gain = 0.3;
};

static Params params;

// Soft saturate function
static double soft_saturate(double u, double max_u, double soft_u) {
    if (max_u <= soft_u) {
        soft_u = max_u * 0.7;
    }
    if (std::abs(u) <= soft_u) {
        return u;
    } else {
        double k = (std::abs(u) - soft_u) / (max_u - soft_u);
        k = std::tanh(k * 5.0);
        return std::copysign(1.0, u) * (soft_u + k * (max_u - soft_u));
    }
}

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

// PID controller class
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

    // Conditional integration (anti-windup + simulation logic)
    // Only integrate if error is small or opposes current integral
    bool allowIntegrate = true;
    // Simple anti-windup: if output saturated, don't integrate same direction? 
    // Here we use the simulation's logic:
    // if (abs(error) > thresh && sign(error) == sign(integrator)) allow = false;
    // We'll stick to standard clamping for simplicity in this class, 
    // but the outer loop can manage the integrator reset if needed.
    
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

// Cascaded PIDs
// Outer Loop: Angle Error -> Desired Rate
// Gains derived from Simulation: Kp_out = Kp_sim / Kd_sim = 4.55 / 7.6 = 0.6
//                                Ki_out = Ki_sim / Kd_sim = 0.53 / 7.6 = 0.07
static PID anglePID(0.6f, 0.07f, 0.0f, -120.0f, 120.0f, -20.0f, 20.0f); 

// Inner Loop: Rate Error -> Servo Angle (Absolute from center)
// Gains derived from Simulation: Kp_in = Kd_sim = 7.6
static PID ratePID(7.6f, 0.0f, 0.0f, -90.0f, 90.0f, -10.0f, 10.0f);

// Diagnostics
static volatile float lastDesiredRate = 0.0f;
static volatile float lastRateCmd = 0.0f;

static bool controlDebug = false;
static const float servoCenter = 90.0f;
static float maxServoOffset = 25.0f;
static bool invertOutput = false;

// Control task implementation
static void controlTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();

  // Update PID gains from Params struct (one-time init or dynamic?)
  // Let's sync them here to ensure they match the struct if it changes
  // 映射仿真参数到串级PID参数：
  // 外环 P = Kp_sim / Kd_sim (将角度误差转换为期望角速度)
  float k_outer_p = params.Kp_phi / params.Kd_phi;
  // 外环 I = Ki_sim / Kd_sim (消除稳态误差)
  float k_outer_i = params.Ki_phi / params.Kd_phi;
  // 内环 P = Kd_sim (将角速度误差转换为舵偏角，提供阻尼)
  // 速度环对噪声/扰动更敏感，默认将其响应适当降低一些（可根据实际再调）
  float k_inner_p = params.Kd_phi * 0.6f;// 60% 增益以减少抖动
  
  anglePID.setGains(k_outer_p, k_outer_i, 0.0f);
  ratePID.setGains(k_inner_p, 0.0f, 0.0f);

  for (;;) {
    vTaskDelayUntil(&lastWake, CONTROL_LOOP_MS);

    bool run = false;
    float runningTarget = 0.0f;
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
      run = enabled;
      runningTarget = targetYaw;
      xSemaphoreGive(ctrlMutex);
    }

    if (!run) {
        anglePID.reset();
        ratePID.reset();
        continue;
    }

    SensorData data = getSensorData();
    if (!data.valid) continue;

    float dt = data.dt;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;

    // --- Cascaded Control Structure (串级控制结构) ---

    // 1. Outer Loop: Angle Control (外环：角度控制)
    // Input: Angle Error (deg) -> 目标角度与当前角度的差值
    // Output: Desired Rate (deg/s) -> 期望的旋转速度
    float angleError = angleDiff(runningTarget, data.yawAngle);
    
    // Deadband from params (死区控制：误差极小时忽略，防止舵机抖动)
    if (std::abs(angleError * M_PI/180.0) < params.phi_error_dead) angleError = 0.0f;

    float desiredRate = anglePID.update(angleError, dt);
    lastDesiredRate = desiredRate;

    // 2. Inner Loop: Rate Control (内环：角速度控制)
    // Input: Rate Error (deg/s) -> 期望速度与实际陀螺仪速度的差值
    // Output: Servo Angle Offset (deg) -> 舵机偏转角度（绝对位置模式）
    // Note: Simulation uses radians, but our PIDs here are tuned with the ratio, so units cancel out 
    // as long as we are consistent. 
    // However, params.Kp_phi is 4.55 (for radians). 
    // If we input degrees, we need to be careful.
    // Simulation: delta(rad) = 4.55 * err(rad). 
    // Here: delta(deg) = 4.55 * err(deg) ? 
    // Yes, if K is dimensionless or 1/s, linear scaling works.
    // delta(deg) = delta(rad) * 180/pi = 4.55 * err(rad) * 180/pi = 4.55 * err(deg).
    // So the gains are valid for degrees too.

    float rateError = desiredRate - data.gyroRate;
    float servoDelta = ratePID.update(rateError, dt);
    
    // Soft saturation (optional, using the helper)
    // servoDelta = soft_saturate(servoDelta, params.delta_max * 180.0/M_PI, params.delta_soft * 180.0/M_PI);
    
    // --- Actuation Logic (执行器逻辑) ---

    // --- Feedforward Compensation from Pitch/Roll (倾角补偿) ---
    // 目标：如果帆翼发生倾斜（Pitch/Roll），说明物理上已经偏离平衡位置。
    // 即使 Yaw 积分误差为 0，我们也希望产生一定的恢复力矩。
    // 策略：将 Pitch/Roll 角度作为额外的前馈项叠加到舵机输出上。
    // 系数 K_tilt 需要根据实验调整，正负号取决于倾斜方向与期望恢复力矩的关系。
    
    // 假设：向左倾斜（Roll < 0）需要向右打舵来恢复（或反之，取决于气动特性）。
    // 这里简单实现为线性叠加： servo_offset += K_roll * roll + K_pitch * pitch
    
    float tiltCompensation = 0.0f;
    // 仅当倾角超过一定阈值（如 3 度）时才介入，避免噪声干扰
    if (abs(data.rollAngle) > 3.0f) {
        // 示例系数：每倾斜 0.5 度，舵机额外偏转 1.0 度
        // 请根据实际方向调整正负号！
        tiltCompensation += 2.0f * data.rollAngle; 
    }
    // Pitch 补偿同理（如果需要）
    // if (abs(data.pitchAngle) > 5.0f) {
    //    tiltCompensation += 0.3f * data.pitchAngle;
    // }

    // 将补偿量叠加到 PID 输出上
    servoDelta += tiltCompensation;

    // Apply inversion (反向控制：如果舵机安装方向相反，取反输出)
    if (invertOutput) servoDelta = -servoDelta;

    // Apply user-defined limit (maxServoOffset) (输出限幅：保护机械结构)
    if (servoDelta > maxServoOffset) servoDelta = maxServoOffset;
    if (servoDelta < -maxServoOffset) servoDelta = -maxServoOffset;

    // Calculate final servo angle (Absolute Position Mode) (计算最终舵机角度 - 绝对位置模式)
    // Note: Original code used incremental (next = current + delta).
    // This implementation uses absolute (next = center + delta) because
    // the simulation model assumes a direct mapping from error to deflection.
    // 绝对模式：舵偏角直接叠加在中位（90度）上，而不是累加在当前角度上。
    // 这消除了积分漂移，并确保系统在无控制信号时自动回中。
    int targetServoAngle = (int)roundf(servoCenter + servoDelta);

    // Clamp to hardware limits (硬件限幅：防止超出舵机物理行程 0-180)
    if (targetServoAngle < 0) targetServoAngle = 0;
    if (targetServoAngle > 180) targetServoAngle = 180;

    setServoAngle(targetServoAngle);
    lastRateCmd = servoDelta;

    // Debug printing
    static int dbgCount = 0;
    if (controlDebug) {
      dbgCount++;
      if (dbgCount >= 10) {
        dbgCount = 0;
        Serial.print("CTRL_CAS,tgt:"); Serial.print(runningTarget, 1);
        Serial.print(",yaw:"); Serial.print(data.yawAngle, 1);
        Serial.print(",roll:"); Serial.print(data.rollAngle, 1); // Print Roll
        Serial.print(",comp:"); Serial.print(tiltCompensation, 1); // Print Compensation
        Serial.print(",out:"); Serial.println(servoDelta, 1);
      }
    }
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

void controlReset() {
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10))) {
    anglePID.reset();
    ratePID.reset();
    targetYaw = 0.0f;
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

void controlSetAnglePID(float kp, float ki, float kd) { 
    // Map to Phi PID
    params.Kp_phi = kp;
    params.Ki_phi = ki;
    params.Kd_phi = kd;
}
void controlSetRatePID(float kp, float ki, float kd)  { 
    // Unused in this architecture
}

float controlGetLastDesiredRate() { return lastDesiredRate; }
float controlGetLastRateCommand()  { return lastRateCmd; }

// Debug control loop printing
void controlSetDebug(bool enable) {
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10))) {
    controlDebug = enable;
    xSemaphoreGive(ctrlMutex);
  }
}

void controlSetInvertOutput(bool inv) {
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10))) {
    invertOutput = inv;
    xSemaphoreGive(ctrlMutex);
  }
}

void controlSetMaxAngleOffset(float maxOffset) {
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10))) {
    if (maxOffset < 0.0f) maxOffset = 0.0f;
    // clamp to reasonable upper bound
    if (maxOffset > 90.0f) maxOffset = 90.0f;
    maxServoOffset = maxOffset;
    xSemaphoreGive(ctrlMutex);
  }
}

float controlGetMaxAngleOffset() {
  float v = 0.0f;
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) { v = maxServoOffset; xSemaphoreGive(ctrlMutex); }
  return v;
}

bool controlIsOutputInverted() {
  bool v = false;
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) { v = invertOutput; xSemaphoreGive(ctrlMutex); }
  return v;
}

bool controlIsDebug() {
  bool v = false;
  if (ctrlMutex == NULL) ctrlMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) { v = controlDebug; xSemaphoreGive(ctrlMutex); }
  return v;
}
