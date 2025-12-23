#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>  // Add SimpleKalmanFilter library

#include "control/control.h"

// Add FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Servo control parameters
#define SERVO_PIN 1         // GPIO 1 for servo control
#define LEDC_CHANNEL 0      // Using LEDC channel 0
#define LEDC_TIMER_BIT 12   // 12-bit resolution (0-4095)
#define LEDC_BASE_FREQ 50   // 50Hz PWM frequency

// LED status indicators
#define LED_PIN LED_BUILTIN // Built-in LED on XIAO-ESP32S3
#define LED_AP_MODE 1000    // AP mode - slow blink (1000ms)
#define LED_WIFI_CONNECTING 250 // WiFi connecting - fast blink (250ms)
#define LED_SERVO_ACTIVE 50 // Servo movement - very short blink (50ms)

// Servo angle PWM values
#define SERVO_MIN_PULSE 103   // 0.5ms/20ms * 4095 ≈ 103 (0 degrees)
#define SERVO_MID_PULSE 307   // 1.5ms/20ms * 4095 ≈ 307 (90 degrees)
#define SERVO_MAX_PULSE 512   // 2.5ms/20ms * 4095 ≈ 512 (180 degrees)

// I2C pins for MPU6050
#define SDA_PIN 5  // GPIO5 for SDA according to XIAO ESP32S3 pinout
#define SCL_PIN 6  // GPIO6 for SCL according to XIAO ESP32S3 pinout

// WiFi configuration
#define ENABLE_WIFI false        // Set to false for serial-only mode (no WiFi/WebServer)
#define EEPROM_SIZE 512
#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64

// FreeRTOS task definitions
#define WEBSERVER_TASK_PRIORITY 3
#define MPU_TASK_PRIORITY 2
#define SERIAL_TASK_PRIORITY 2
#define LED_TASK_PRIORITY 1
#define STACK_SIZE 8192

// Serial output settings
#define SERIAL_PRINT_INTERVAL 100  // Print every 100ms

// Web server
WebServer server(80);

// Global variables
int currentServoAngle = 90;  // Current servo angle
String apSSID = "ServoControl_AP";
String apPassword = "12345678";
bool wifiConfigured = false;
unsigned long lastLedToggle = 0;
int ledBlinkInterval = 1000; // Default blink interval
bool ledState = false;       // LED state
bool restartPending = false;
unsigned long restartTime = 0;

// MPU6050 variables
Adafruit_MPU6050 mpu;
float measuredAngle = 0.0f;         // Filtered yaw angle (degrees)
volatile bool mpuInitialized = false;  // MPU6050 initialization status (volatile for multi-task access)

// Kalman filter variables
SimpleKalmanFilter kalmanX(1, 1, 0.01);  // SimpleKalmanFilter(e_mea, e_est, q)
float gyroZoffset = 0.0f;           // Gyroscope Z-axis zero drift compensation
unsigned long timer;                // Timer for calculating dt between readings

// Sensor snapshot used by MPU and control task (defined in include/system_api.h)
SensorData currentSensorData = {0.0f, 0.0f, 0.0f, 0.0f, 0, false}; //0.0f表示直接初始化为浮点型0

// FreeRTOS synchronization
SemaphoreHandle_t angleMutex;   // Protects sensor data
SemaphoreHandle_t servoMutex;   // Protects servo state
SemaphoreHandle_t wifiMutex;    // Protects WiFi config
SemaphoreHandle_t ledMutex;     // Protects LED state

// Structure to store WiFi configuration
struct {
  char ssid[MAX_SSID_LENGTH] = {0};
  char password[MAX_PASSWORD_LENGTH] = {0};
  bool configured = false;
} wifiConfig;

// Function prototypes
void webServerTask(void *parameter);
void mpuTask(void *parameter);
void ledControlTask(void *parameter);
void serialTask(void *parameter);
void setServoAngle(int angle);
float readMPUAngle();
bool initMPU6050();
float mapFloat();
void processSerialCommand();

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to read a file from SPIFFS
String readFile(const char* path) {
  File file = SPIFFS.open(path, "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return String();
  }
  
  String fileContent;
  while (file.available()) {
    fileContent += (char)file.read();
  }
  file.close();
  return fileContent;
}

// LED control functions
void setLedMode(int blinkInterval) {
  if (xSemaphoreTake(ledMutex, portMAX_DELAY)) {
    ledBlinkInterval = blinkInterval;
    lastLedToggle = millis();
    xSemaphoreGive(ledMutex);
  }
}

void turnLedOn() {
  if (xSemaphoreTake(ledMutex, portMAX_DELAY)) {
    digitalWrite(LED_PIN, LOW); // LOW turns on the LED on most ESP32 boards
    ledState = true;
    ledBlinkInterval = 0;
    xSemaphoreGive(ledMutex);
  }
}

void turnLedOff() {
  if (xSemaphoreTake(ledMutex, portMAX_DELAY)) {
    digitalWrite(LED_PIN, HIGH); // HIGH turns off the LED on most ESP32 boards
    ledState = false;
    ledBlinkInterval = 0;
    xSemaphoreGive(ledMutex);
  }
}

// Convert angle to PWM value
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// Set servo angle with thread safety
void setServoAngle(int angle) {
  // Limit angle to valid range
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  // Convert angle to PWM value
  int pulse = angleToPulse(angle);
  
  // Take servo mutex for atomic update
  if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(100))) {
    ledcWrite(LEDC_CHANNEL, pulse);
    currentServoAngle = angle;
    xSemaphoreGive(servoMutex);
    
    // Print outside critical section to avoid blocking
    Serial.print("Servo: ");
    Serial.print(angle);
    Serial.print("° PWM:");
    Serial.println(pulse);
  } else {
    Serial.println("ERROR: Failed to acquire servo mutex");
  }
}

// MPU6050 initialization function
bool initMPU6050() {
  // Initialize I2C with specified SDA and SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }
  
  Serial.println("MPU6050 Found!");
  
  // Configure accelerometer range (±2, 4, 8, or 16 G)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // Configure gyroscope range (250, 500, 1000, or 2000 degrees/second)
  // Lower range = higher sensitivity for small movements
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // Configure filter bandwidth (5, 10, 21, 44, 94, 184, 260 Hz)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Calibrate gyroscope - calculate zero drift offset
  Serial.println("Calibrating gyroscope (Z-axis), keep the sensor still...");
  for (int i = 0; i < 2000; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZoffset += g.gyro.z;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  gyroZoffset /= 2000.0f;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZoffset);
  
  // Initialize timer for dt calculation
  timer = micros();
  
  Serial.println("Kalman filter initialized");
  return true;
}

// Global variables for IMU tracking
static float rawYaw = 0.0f;
static bool yawInitialized = false;
static float gyroRateFiltered = 0.0f;

// Reset IMU state to zero
void resetIMU() {
  if (xSemaphoreTake(angleMutex, portMAX_DELAY)) {
    measuredAngle = 0.0f;
    rawYaw = 0.0f;
    yawInitialized = false; 
    gyroRateFiltered = 0.0f;
    kalmanX = SimpleKalmanFilter(1, 1, 0.01);
    xSemaphoreGive(angleMutex);
    Serial.println("IMU data reset to zero");
  }
}

// Update sensor data with timestamp - called by MPU task
void updateSensorData() {
  if (!mpuInitialized) {
    currentSensorData.valid = false;
    return;
  }

  if (xSemaphoreTake(angleMutex, portMAX_DELAY)) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = micros();
    float dt = (now - timer) / 1000000.0f;
    timer = now;

    // Guard against invalid or excessively large time steps
    if (dt <= 0.0f || dt > 0.2f) {
      dt = 0.02f; // Assume nominal 50 Hz update if timing glitch occurs
    }

    // Convert rad/s to deg/s (bias compensated)
    const float gyroRateDeg = (g.gyro.z - gyroZoffset) * 180.0f / PI;

    // Light low-pass on gyro rate to reduce noise sensitivity in the rate loop
    // tau=0.05s gives small delay but noticeably less jitter
    const float tauRate = 0.05f;
    const float alphaRate = dt / (tauRate + dt);
    gyroRateFiltered += alphaRate * (gyroRateDeg - gyroRateFiltered);

    // Online bias tracking (reduces yaw drift over time):
    // When the system is near-stationary, slowly adapt gyroZoffset towards current reading.
    // This helps compensate temperature drift and vibration-induced bias changes.
    const int servoAngleSnapshot = currentServoAngle; // ok to read without mutex (int is atomic on ESP32)
    const bool nearCenter = (abs(servoAngleSnapshot - 90) <= 3);
    const bool nearStill = (fabsf(gyroRateFiltered) <= 0.5f);
    if (nearCenter && nearStill) {
      const float tauBias = 20.0f; // seconds (slow)
      const float alphaBias = dt / (tauBias + dt);
      gyroZoffset += alphaBias * (g.gyro.z - gyroZoffset); // gyroZoffset is in rad/s
    }

    if (!yawInitialized) {
      rawYaw = measuredAngle;
      yawInitialized = true;
    }

    rawYaw += gyroRateFiltered * dt;

    // Washout filter (High-pass behavior for Yaw):
    // Slowly decay the yaw angle towards zero to prevent long-term drift accumulation.
    // Since we don't have a compass, we can't hold absolute heading forever.
    // This makes the system act like a "rate damper" that resists change but eventually accepts new headings as "zero".
    // tau_washout = 60.0s means errors decay much slower, reducing offset after short disturbances.
    const float tauWashout = 60.0f; 
    const float alphaWashout = dt / (tauWashout + dt);
    rawYaw = rawYaw * (1.0f - alphaWashout);

    // Keep raw yaw within [-180, 180] to avoid overflow
    if (rawYaw > 180.0f) rawYaw -= 360.0f;
    if (rawYaw < -180.0f) rawYaw += 360.0f;

    measuredAngle = kalmanX.updateEstimate(rawYaw);

    // Calculate Pitch and Roll from Accelerometer (in degrees)
    // Pitch (Y-axis rotation): atan2(accX, accZ)
    // Roll (X-axis rotation): atan2(accY, accZ)
    // Note: This assumes the sensor is roughly horizontal.
    float pitch = atan2(a.acceleration.x, a.acceleration.z) * 180.0f / PI;
    float roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;

    // Update sensor data structure atomically
    currentSensorData.yawAngle = measuredAngle;
    currentSensorData.rawYaw = rawYaw;
    currentSensorData.gyroRate = gyroRateFiltered;
    currentSensorData.pitchAngle = pitch;
    currentSensorData.rollAngle = roll;
    currentSensorData.dt = dt;
    currentSensorData.timestamp = now;
    currentSensorData.valid = true;

    xSemaphoreGive(angleMutex);
  }
}

// Get current sensor data (thread-safe)
SensorData getSensorData() {
  SensorData data;
  if (xSemaphoreTake(angleMutex, portMAX_DELAY)) {
    data = currentSensorData;
    xSemaphoreGive(angleMutex);
  }
  return data;
}

// Read MPU6050 angle - returns mapped angle for web interface (0-180)
float readMPUAngle() {
  if (!mpuInitialized) {
    return mapFloat(constrain(measuredAngle, -90.0f, 90.0f), -90.0f, 90.0f, 0.0f, 180.0f);
  }
  
  float clamped = constrain(measuredAngle, -90.0f, 90.0f);
  return mapFloat(clamped, -90.0f, 90.0f, 0.0f, 180.0f);
}

// Load WiFi configuration from EEPROM
void loadWiFiConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, wifiConfig);
  EEPROM.end();
  
  if (wifiConfig.configured) {
    Serial.println("WiFi configuration loaded from EEPROM");
    Serial.print("SSID: ");
    Serial.println(wifiConfig.ssid);
    wifiConfigured = true;
  } else {
    Serial.println("No WiFi configuration found in EEPROM");
    wifiConfigured = false;
  }
}

// Save WiFi configuration to EEPROM
void saveWiFiConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, wifiConfig);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("WiFi configuration saved to EEPROM");
}

// Start AP mode
void startAPMode() {
  WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP mode started. IP address: ");
  Serial.println(IP);
  
  // Set LED to slow blink for AP mode
  setLedMode(LED_AP_MODE);
}

// Try to connect to WiFi
bool connectToWiFi() {
  if (!wifiConfigured) return false;
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(wifiConfig.ssid);
  
  // Set LED to fast blink for WiFi connecting
  setLedMode(LED_WIFI_CONNECTING);
  
  WiFi.begin(wifiConfig.ssid, wifiConfig.password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("Connected to WiFi. IP address: ");
    Serial.println(WiFi.localIP());
    
    // Set LED to solid ON for connected state
    turnLedOn();
    
    return true;
  } else {
    Serial.println("");
    Serial.println("Failed to connect to WiFi");
    return false;
  }
}

// Handle root page (Servo control page)
void handleRoot() {
  String html = readFile("/index.html");
  
  // Read servo angle with mutex protection
  int angle = 90; // Default value
  if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(10))) {
    angle = currentServoAngle;
    xSemaphoreGive(servoMutex);
  }
  
  // Replace angle placeholder with current value
  html.replace("%ANGLE%", String(angle));
  server.send(200, "text/html", html);
}

// WiFi configuration page
void handleWiFiConfig() {
  String html = readFile("/wifi_config.html");
  server.send(200, "text/html", html);
}

// Handle measured angle requests
void handleGetMeasuredAngle() {
  float angle = readMPUAngle();
  server.send(200, "application/json", "{\"angle\":" + String(angle, 2) + ", \"status\":" + String(mpuInitialized ? "true" : "false") + "}");
}

// Save WiFi configuration
void handleSaveWiFi() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  
  if (xSemaphoreTake(wifiMutex, portMAX_DELAY)) {
    // Save configuration with proper null termination
    strncpy(wifiConfig.ssid, ssid.c_str(), MAX_SSID_LENGTH - 1);
    wifiConfig.ssid[MAX_SSID_LENGTH - 1] = '\0';
    
    strncpy(wifiConfig.password, password.c_str(), MAX_PASSWORD_LENGTH - 1);
    wifiConfig.password[MAX_PASSWORD_LENGTH - 1] = '\0';
    
    wifiConfig.configured = true;
    saveWiFiConfig();
    
    xSemaphoreGive(wifiMutex);
  }
  
  String html = readFile("/wifi_save.html");
  server.send(200, "text/html", html);
  
  // Schedule restart with LED indication
  setLedMode(100);
  restartPending = true;
  restartTime = millis();
}

// Set servo angle handler
void handleSetAngle() {
  if (server.hasArg("angle")) {
    int angle = server.arg("angle").toInt();
    setServoAngle(angle);
    server.send(200, "text/plain", "Angle set to " + String(angle));
  } else {
    server.send(400, "text/plain", "Missing angle parameter");
  }
}

// Web server task
void webServerTask(void *parameter) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to prevent watchdog triggers
  }
}

// MPU sensor reading task - high priority for control loop timing
void mpuTask(void *parameter) {
  const TickType_t xDelay = pdMS_TO_TICKS(20); // 20ms = 50Hz update rate
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    if (mpuInitialized) {
      updateSensorData(); // Update sensor data with precise timing
    }
    // Use vTaskDelayUntil for precise periodic execution (important for control loops)
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

// LED control task
void ledControlTask(void *parameter) {
  const TickType_t xDelay = pdMS_TO_TICKS(50); // 50ms for LED control
  
  for (;;) {
    if (xSemaphoreTake(ledMutex, portMAX_DELAY)) {
      // Check if blinking is required
      if (ledBlinkInterval > 0) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastLedToggle >= ledBlinkInterval) {
          lastLedToggle = currentMillis;
          ledState = !ledState;
          digitalWrite(LED_PIN, ledState ? LOW : HIGH);
        }
      }
      
      // Check for restart pending
      if (restartPending && (millis() - restartTime >= 5000)) {
        ESP.restart();
      }
      
      xSemaphoreGive(ledMutex);
    }
    vTaskDelay(xDelay);
  }
}

// Serial output task - prints key information with timing data
void serialTask(void *parameter) {
  const TickType_t xDelay = pdMS_TO_TICKS(SERIAL_PRINT_INTERVAL);
  
  for (;;) {
    if (mpuInitialized) {
      SensorData data = getSensorData();
      
      if (data.valid) {
        // Read servo angle with mutex protection
        int servoAngle = 0;
        if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(10))) {
          servoAngle = currentServoAngle;
          xSemaphoreGive(servoMutex);
        }
        
        // Format: MSG,yaw,gyro_rate,servo_angle,dt,timestamp
        Serial.print("MSG,");
        Serial.print(data.yawAngle, 2);
        Serial.print(",");
        Serial.print(data.gyroRate, 2);
        Serial.print(",");
        Serial.print(servoAngle);
        Serial.print(",");
        Serial.print(data.dt * 1000.0f, 2); // dt in milliseconds
        Serial.print(",");
        Serial.println(data.timestamp);
      } else {
        Serial.println("MSG,INVALID,INVALID,INVALID,INVALID,INVALID");
      }
    }
    vTaskDelay(xDelay);
  }
}

// Process serial commands (format: s<angle>)
void processSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0 && (command.charAt(0) == 's' || command.charAt(0) == 'S')) {
      // Extract angle from command (e.g., "s90" -> 90)
      String angleStr = command.substring(1);
      int angle = angleStr.toInt();
      
      if (angle >= 0 && angle <= 180) {
        setServoAngle(angle);
        Serial.print("OK: Servo set to ");
        Serial.println(angle);
      } else {
        Serial.println("ERROR: Angle must be 0-180");
      }
    } else if (command.length() > 0 && (command.charAt(0) == 'c')) {
      // Enable/disable cascaded auto-control: c1=c ON, c0=c OFF, c toggle
      if (command.length() == 1) {
        bool cur = controlIsEnabled();
        controlEnable(!cur);
        Serial.print("Auto-control ");
        Serial.println(!cur ? "enabled" : "disabled");
      } else {
        String arg = command.substring(1);
        arg.trim();
        if (arg == "1") { controlEnable(true); Serial.println("Auto-control enabled"); }
        else if (arg == "0") { controlEnable(false); Serial.println("Auto-control disabled"); }
        else { Serial.println("ERROR: Unknown argument for c. Use c0 or c1"); }
      }
    } else if (command.length() > 0 && (command.charAt(0) == 't' || command.charAt(0) == 'T')) {
      // Set target yaw for auto control: t<deg> (e.g., t-30)
      String angleStr = command.substring(1);
      angleStr.trim();
      float yaw = angleStr.toFloat();
      if (yaw >= -360.0f && yaw <= 360.0f) {
        // normalize
        while (yaw > 180.0f) yaw -= 360.0f;
        while (yaw < -180.0f) yaw += 360.0f;
        controlSetTargetYaw(yaw);
        Serial.print("OK: target yaw set to ");
        Serial.println(yaw);
      } else {
        Serial.println("ERROR: yaw out of range (-360..360)");
      }
    } else if (command.length() > 0 && (command.charAt(0) == 'C')) {
      // Print control status (telemetry) — adapt for current controller API
      Serial.print("CTRL,");
      Serial.print(controlIsEnabled() ? "ENABLED," : "DISABLED,");
      Serial.print(controlGetTargetYaw(), 2);
      Serial.print(",desired_rate(deg/s):");
      Serial.print(controlGetLastDesiredRate(), 2);
      Serial.print(",rate_cmd(deg):");
      Serial.print(controlGetLastRateCommand(), 2);
      Serial.print(",limit:");
      Serial.print(controlGetMaxAngleOffset(), 2);
      Serial.print(",invert:");
      Serial.print(controlIsOutputInverted() ? "1" : "0");
      Serial.print(",servo:");
      Serial.println(getCurrentServoAngle());
    } else if (command.length() > 0 && (command.charAt(0) == 'd' || command.charAt(0) == 'D')) {
      // Toggle or set control debug printing: d  -> toggle, d1 -> enable, d0 -> disable
      String arg = command.substring(1);
      arg.trim();
      if (arg.length() == 0) {
        bool cur = controlIsDebug();
        controlSetDebug(!cur);
        Serial.print("Control debug "); Serial.println(!cur ? "enabled" : "disabled");
      } else if (arg == "1") {
        controlSetDebug(true);
        Serial.println("Control debug enabled");
      } else if (arg == "0") {
        controlSetDebug(false);
        Serial.println("Control debug disabled");
      } else {
        Serial.println("ERROR: d usage: d (toggle) | d1 (on) | d0 (off)");
      }
    } else if (command.length() > 0 && (command.charAt(0) == 'o' || command.charAt(0) == 'O')) {
      // Toggle or set inversion of control output: o  -> toggle, o1 -> enable, o0 -> disable
      String arg = command.substring(1);
      arg.trim();
      if (arg.length() == 0) {
        bool cur = controlIsOutputInverted();
        controlSetInvertOutput(!cur);
        Serial.print("Control invert "); Serial.println(!cur ? "enabled" : "disabled");
      } else if (arg == "1") {
        controlSetInvertOutput(true);
        Serial.println("Control invert enabled");
      } else if (arg == "0") {
        controlSetInvertOutput(false);
        Serial.println("Control invert disabled");
      } else {
        Serial.println("ERROR: o usage: o (toggle) | o1 (on) | o0 (off)");
      }
    } else if (command.length() > 0 && (command.charAt(0) == 'm' || command.charAt(0) == 'M')) {
      // Set or query max angle offset: m -> print current, m25 -> set to 25 degrees
      String arg = command.substring(1);
      arg.trim();
      if (arg.length() == 0) {
        Serial.print("Current control max angle offset: ");
        Serial.println(controlGetMaxAngleOffset(), 2);
      } else {
        float val = arg.toFloat();
        if (val < 0.0f || val > 90.0f) {
          Serial.println("ERROR: max offset must be 0..90 deg");
        } else {
          controlSetMaxAngleOffset(val);
          Serial.print("OK: max angle offset set to "); Serial.println(val, 2);
        }
      }
    } else if (command.length() > 0) {
      Serial.println("ERROR: Unknown command. Use s<angle>, c<0|1>, t<yaw>, C for status");
    }
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Servo Control System Initializing with FreeRTOS...");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Start with LED off
  
  // Create mutexes
  angleMutex = xSemaphoreCreateMutex();
  servoMutex = xSemaphoreCreateMutex();
  wifiMutex = xSemaphoreCreateMutex();
  ledMutex = xSemaphoreCreateMutex();
  
  // Verify mutex creation
  if (!angleMutex || !servoMutex || !wifiMutex || !ledMutex) {
    Serial.println("FATAL: Failed to create mutexes");
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }
  
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    // Error pattern - very fast blink
    ledBlinkInterval = 50;
    return;
  }
  Serial.println("SPIFFS mounted successfully");
  
  // Configure LEDC channel
  ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  
  // Attach LEDC channel to GPIO pin
  ledcAttachPin(SERVO_PIN, LEDC_CHANNEL);
  
  // Set servo to center position (90 degrees)
  setServoAngle(90);

  // Start control task (cascaded PID) so auto-control actually runs
  controlBegin();
  controlEnable(true); // Enable auto-control by default
  Serial.println("Control task started");
  
  // Initialize MPU6050 sensor
  mpuInitialized = initMPU6050();
  if (mpuInitialized) {
    Serial.println("MPU6050 initialized successfully");
    // Explicitly reset IMU data to zero
    resetIMU();
  } else {
    Serial.println("MPU6050 initialization failed - continuing without angle measurement");
  }

  // Explicitly reset control state (PIDs, target yaw)
  controlReset();
  Serial.println("System state reset: Servo centered, IMU zeroed, Control reset.");
  
#if ENABLE_WIFI
  Serial.println("WiFi mode enabled");
  
  // Load WiFi configuration
  loadWiFiConfig();
  
  // Try to connect to saved WiFi
  bool connected = false;
  if (wifiConfigured) {
    connected = connectToWiFi();
  }
  
  // If cannot connect, start AP mode
  if (!connected) {
    startAPMode();
    
    // Set web server routes for AP mode
    server.on("/", HTTP_GET, handleWiFiConfig);
    server.on("/saveWiFi", HTTP_POST, handleSaveWiFi);
  } else {
    // Set web server routes for STA mode
    server.on("/", HTTP_GET, handleRoot);
    server.on("/setAngle", HTTP_GET, handleSetAngle);
    server.on("/getMeasuredAngle", HTTP_GET, handleGetMeasuredAngle);
    
    // Add static file handler for all other files
    server.serveStatic("/", SPIFFS, "/");
  }
  
  // Start web server
  server.begin();
  Serial.println("Web server started");
  
  if (connected) {
    Serial.println("System ready in STA mode");
    Serial.print("Access control panel at: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("System ready in AP mode");
    Serial.print("Connect to WiFi network: ");
    Serial.println(apSSID);
    Serial.print("Password: ");
    Serial.println(apPassword);
    Serial.print("Then navigate to: http://");
    Serial.println(WiFi.softAPIP());
  }
#else
  Serial.println("WiFi disabled - Serial-only mode");
  Serial.println("System ready for serial control");
  Serial.println("Commands: s<angle> (e.g., s90 to set servo to 90 degrees)");
  Serial.println("  c0|c1 or c - disable/enable auto-control");
  Serial.println("  t<angle> - set target yaw (deg, -180..180)");
  Serial.println("  C - show control status (target, desired_rate, rate_cmd)");
  Serial.println("  o | o1 | o0 - toggle | enable | disable invert of control output (reverse direction)");
  Serial.println("  m<deg> - set controller +/- angle limit in degrees (e.g. m25). No arg prints current limit.");
  Serial.println("  d | d1 | d0 - toggle | enable | disable control debug prints");
  turnLedOn(); // Solid LED in serial mode
#endif
  
  // Create FreeRTOS tasks
#if ENABLE_WIFI
  xTaskCreate(
    webServerTask,    // Task function
    "WebServerTask",  // Name for debugging
    STACK_SIZE,       // Stack size (bytes)
    NULL,             // Task parameters
    WEBSERVER_TASK_PRIORITY,  // Priority (higher number = higher priority)
    NULL              // Task handle
  );
#endif
  
  xTaskCreate(
    mpuTask,
    "MPUTask",
    STACK_SIZE,
    NULL,
    MPU_TASK_PRIORITY,
    NULL
  );
  
  xTaskCreate(
    ledControlTask,
    "LEDTask",
    STACK_SIZE,
    NULL,
    LED_TASK_PRIORITY,
    NULL
  );
  
  xTaskCreate(
    serialTask,
    "SerialTask",
    STACK_SIZE,
    NULL,
    SERIAL_TASK_PRIORITY,
    NULL
  );
  
  // FreeRTOS is now running, main loop will not be used
}

void loop() {
  // Process serial commands in the main loop
  processSerialCommand();
  vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent tight loop
}

// Return the current servo angle (thread-safe)
int getCurrentServoAngle() {
  int angle = 0;
  if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(10))) {
    angle = currentServoAngle;
    xSemaphoreGive(servoMutex);
  }
  return angle;
}