#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <SPIFFS.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>
// #include <SimpleKalmanFilter.h>  // Add SimpleKalmanFilter library
// #include <HardwareSerial.h> // Moved to h30_imu.cpp

#include "control/control.h"
#include "h30_imu.h" // Add H30 module

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

// I2C pins for MPU6050 (Disabled)
// #define SDA_PIN 5  // GPIO5 for SDA according to XIAO ESP32S3 pinout
// #define SCL_PIN 6  // GPIO6 for SCL according to XIAO ESP32S3 pinout

// UART pins for H30
#define H30_RX_PIN 3  // Module TXD connected here (ESP32 RX)
#define H30_TX_PIN 2  // Module RXD connected here (ESP32 TX)
#define H30_BAUD 921600

// UART pins for LORA
#define LORA_RX_PIN 5  // LORA TXD connected here (ESP32 RX)
#define LORA_TX_PIN 4  // LORA RXD connected here (ESP32 TX)
#define LORA_BAUD 115200

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

// Use UART2 for LoRa
HardwareSerial LoRaSerial(2);

// Dual channel output (USB + LoRa)
class DualPrint : public Print {
public:
  size_t write(uint8_t c) override {
    size_t n = Serial.write(c);
    LoRaSerial.write(c);
    return n;
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    size_t n = Serial.write(buffer, size);
    LoRaSerial.write(buffer, size);
    return n;
  }
} Console;

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

// H30 IMU variables
float measuredAngle = 0.0f;           // Yaw angle
// volatile bool mpuInitialized = false;  // Removed, use H30_IsInitialized()
// float yawOffset = 0.0f;               // Moved to H30 module
// volatile bool zeroingRequest = true;  // Moved to H30 module

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
    // Console.print("Servo: ");
    // Console.print(angle);
    // Console.print("* PWM:");
    // Console.println(pulse);
  } else {
    Console.println("ERROR: Failed to acquire servo mutex");
  }
}

// Helper timer
unsigned long timer = 0; // Timer for calculating dt between readings

// Global variables for IMU tracking
// static float rawYaw = 0.0f;
// static bool yawInitialized = false;
// static float gyroRateFiltered = 0.0f;

// Reset IMU state to zero
void resetIMU() {
    H30_RequestZero();
}

// Update sensor data with timestamp - called by MPU task
void updateSensorData() {
  if (xSemaphoreTake(angleMutex, portMAX_DELAY)) {
    unsigned long now = micros();
    float dt = (now - timer) / 1000000.0f;
    timer = now;
    
    // Guard against invalid or excessively large time steps
    if (dt <= 0.0f || dt > 0.2f) {
      dt = 0.02f; // Assume nominal 50 Hz update if timing glitch occurs
    }
    
    // Get Data from H30 Module
    float yaw = H30_GetYaw();
    float rawYaw = H30_GetRawYaw();
    float pitch = H30_GetPitch();
    float roll = H30_GetRoll();
    float gyroRateZ = H30_GetGyroZ();

    measuredAngle = yaw;

    // Update sensor data structure atomically
    currentSensorData.yawAngle = yaw;
    currentSensorData.rawYaw = rawYaw; // Keep absolute raw
    currentSensorData.gyroRate = gyroRateZ;
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

// Read IMU angle - returns mapped angle for web interface (0-180)
float readMPUAngle() {
  float clamped = constrain(measuredAngle, -90.0f, 90.0f);
  return mapFloat(clamped, -90.0f, 90.0f, 0.0f, 180.0f);
}

// Load WiFi configuration from EEPROM
void loadWiFiConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, wifiConfig);
  EEPROM.end();
  
  if (wifiConfig.configured) {
    Console.println("WiFi configuration loaded from EEPROM");
    Console.print("SSID: ");
    Console.println(wifiConfig.ssid);
    wifiConfigured = true;
  } else {
    Console.println("No WiFi configuration found in EEPROM");
    wifiConfigured = false;
  }
}

// Save WiFi configuration to EEPROM
void saveWiFiConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, wifiConfig);
  EEPROM.commit();
  EEPROM.end();
  Console.println("WiFi configuration saved to EEPROM");
}

// Start AP mode
void startAPMode() {
  WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  IPAddress IP = WiFi.softAPIP();
  Console.print("AP mode started. IP address: ");
  Console.println(IP);
  
  // Set LED to slow blink for AP mode
  setLedMode(LED_AP_MODE);
}

// Try to connect to WiFi
bool connectToWiFi() {
  if (!wifiConfigured) return false;
  
  Console.print("Connecting to WiFi: ");
  Console.println(wifiConfig.ssid);
  
  // Set LED to fast blink for WiFi connecting
  setLedMode(LED_WIFI_CONNECTING);
  
  WiFi.begin(wifiConfig.ssid, wifiConfig.password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Console.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Console.println("");
    Console.print("Connected to WiFi. IP address: ");
    Console.println(WiFi.localIP());
    
    // Set LED to solid ON for connected state
    turnLedOn();
    
    return true;
  } else {
    Console.println("");
    Console.println("Failed to connect to WiFi");
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
  server.send(200, "application/json", "{\"angle\":" + String(angle, 2) + ", \"status\":" + String(H30_IsInitialized() ? "true" : "false") + "}");
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

// H30 sensor reading task - reads serial data
void mpuTask(void *parameter) {
  const TickType_t xDelay = pdMS_TO_TICKS(2); // Short delay to allow other tasks but check serial frequently
  
  for (;;) {
    while (H30_Available()) {
        if (H30_ParseByte(H30_Read())) {
             updateSensorData();
        }
    }
    vTaskDelay(xDelay); 
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
// NOTE: Disabled during maneuverTask to avoid serial conflicts
void serialTask(void *parameter) {
  const TickType_t xDelay = pdMS_TO_TICKS(SERIAL_PRINT_INTERVAL);
  
  for (;;) {
    // Disabled: maneuverTask handles TRACK output, avoid conflicts
    /*
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
    */
    vTaskDelay(xDelay);
  }
}

// Process serial commands (format: s<angle>)
void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  if (command.equalsIgnoreCase("help") || command == "?") {
    Console.println("=== Commands ===");
    Console.println("s<angle>: Set servo angle (0-180). e.g. s90");
    Console.println("c[0|1]  : Auto-Control enable/disable (c, c1, c0)");
    Console.println("t<angle>: Set target yaw (-180..180). e.g. t30");
    Console.println("C       : Status (target, rate, servo)");
    Console.println("m<deg>  : Set max angle offset (0-90). e.g. m25");
    Console.println("o[0|1]  : Invert control output (o, o1, o0)");
    Console.println("d[0|1]  : Toggle debug prints (d, d1, d0)");
    Console.println("help/?  : Show this help");
    return;
  }

  if (command.charAt(0) == 's' || command.charAt(0) == 'S') {
    // Extract angle from command (e.g., "s90" -> 90)
    String angleStr = command.substring(1);
    int angle = angleStr.toInt();
    
    if (angle >= 0 && angle <= 180) {
      setServoAngle(angle);
      Console.print("OK: Servo set to ");
      Console.println(angle);
    } else {
      Console.println("ERROR: Angle must be 0-180");
    }
  } else if (command.charAt(0) == 'c') {
    // Enable/disable cascaded auto-control: c1=c ON, c0=c OFF, c toggle
    if (command.length() == 1) {
      bool cur = controlIsEnabled();
      controlEnable(!cur);
      Console.print("Auto-control ");
      Console.println(!cur ? "enabled" : "disabled");
    } else {
      String arg = command.substring(1);
      arg.trim();
      if (arg == "1") { controlEnable(true); Console.println("Auto-control enabled"); }
      else if (arg == "0") { controlEnable(false); Console.println("Auto-control disabled"); }
      else { Console.println("ERROR: Unknown argument for c. Use c0 or c1"); }
    }
  } else if (command.charAt(0) == 't' || command.charAt(0) == 'T') {
    // Set target yaw for auto control: t<deg> (e.g., t-30)
    String angleStr = command.substring(1);
    angleStr.trim();
    float yaw = angleStr.toFloat();
    if (yaw >= -360.0f && yaw <= 360.0f) {
      // normalize
      while (yaw > 180.0f) yaw -= 360.0f;
      while (yaw < -180.0f) yaw += 360.0f;
      controlSetTargetYaw(yaw);
      Console.print("OK: target yaw set to ");
      Console.println(yaw);
    } else {
      Console.println("ERROR: yaw out of range (-360..360)");
    }
  } else if (command.charAt(0) == 'C') {
    // Print control status (telemetry) — adapt for current controller API
    Console.print("CTRL,");
    Console.print(controlIsEnabled() ? "ENABLED," : "DISABLED,");
    Console.print(controlGetTargetYaw(), 2);
    Console.print(",desired_rate(deg/s):");
    Console.print(controlGetLastDesiredRate(), 2);
    Console.print(",rate_cmd(deg):");
    Console.print(controlGetLastRateCommand(), 2);
    Console.print(",limit:");
    Console.print(controlGetMaxAngleOffset(), 2);
    Console.print(",invert:");
    Console.print(controlIsOutputInverted() ? "1" : "0");
    Console.print(",servo:");
    Console.println(getCurrentServoAngle());
  } else if (command.charAt(0) == 'd' || command.charAt(0) == 'D') {
    // Toggle or set control debug printing: d  -> toggle, d1 -> enable, d0 -> disable
    String arg = command.substring(1);
    arg.trim();
    if (arg.length() == 0) {
      bool cur = controlIsDebug();
      controlSetDebug(!cur);
      Console.print("Control debug "); Console.println(!cur ? "enabled" : "disabled");
    } else if (arg == "1") {
      controlSetDebug(true);
      Console.println("Control debug enabled");
    } else if (arg == "0") {
      controlSetDebug(false);
      Console.println("Control debug disabled");
    } else {
      Console.println("ERROR: d usage: d (toggle) | d1 (on) | d0 (off)");
    }
  } else if (command.charAt(0) == 'o' || command.charAt(0) == 'O') {
    // Toggle or set inversion of control output: o  -> toggle, o1 -> enable, o0 -> disable
    String arg = command.substring(1);
    arg.trim();
    if (arg.length() == 0) {
      bool cur = controlIsOutputInverted();
      controlSetInvertOutput(!cur);
      Console.print("Control invert "); Console.println(!cur ? "enabled" : "disabled");
    } else if (arg == "1") {
      controlSetInvertOutput(true);
      Console.println("Control invert enabled");
    } else if (arg == "0") {
      controlSetInvertOutput(false);
      Console.println("Control invert disabled");
    } else {
      Console.println("ERROR: o usage: o (toggle) | o1 (on) | o0 (off)");
    }
  } else if (command.charAt(0) == 'm' || command.charAt(0) == 'M') {
    // Set or query max angle offset: m -> print current, m25 -> set to 25 degrees
    String arg = command.substring(1);
    arg.trim();
    if (arg.length() == 0) {
      Console.print("Current control max angle offset: ");
      Console.println(controlGetMaxAngleOffset(), 2);
    } else {
      float val = arg.toFloat();
      if (val < 0.0f || val > 90.0f) {
        Console.println("ERROR: max offset must be 0..90 deg");
      } else {
        controlSetMaxAngleOffset(val);
        Console.print("OK: max angle offset set to "); Console.println(val, 2);
      }
    }
  } else {
    Console.println("ERROR: Unknown command. Use s<angle>, c<0|1>, t<yaw>, C for status");
  }
}

void processSerialCommand() {
  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }
  if (LoRaSerial.available() > 0) {
    handleCommand(LoRaSerial.readStringUntil('\n'));
  }
}

// Maneuver task for automated testing sequence
void maneuverTask(void *parameter) {
  // Wait for system initialization and sensor stabilization
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  // Re-zero IMU after stabilization period
  resetIMU();
  vTaskDelay(pdMS_TO_TICKS(200)); // Wait for next sensor update to apply the offset

  Console.println("=== Maneuver Sequence Started ===");
  
  // Simple test: Hold at 0 degrees indefinitely
  Console.println("Holding at 0 deg...");
  controlSetTargetYaw(0.0f);
  
  // Keep outputting tracking data at 50Hz
  unsigned long trackingStart = millis();
  while (true) {
    SensorData data = getSensorData();
    int sAngle = getCurrentServoAngle();
    int sPulse = angleToPulse(sAngle);
    
    Console.print("TRACK,");
    Console.print(millis() - trackingStart);
    Console.print(",");
    Console.print(0.0f, 2);  // Target is always 0
    Console.print(",");
    Console.print(data.yawAngle, 2);
    Console.print(",");
    Console.print(data.gyroRate, 2);
    Console.print(",");
    Console.print(sAngle);
    Console.print(",");
    Console.println(sPulse);
    
    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update
  }
  
  /* 
  // Phase 1: Hold 0 degrees for 10s
  Serial.println("Phase 1: Target 0 deg (10s)");
  controlSetTargetYaw(0.0f);
  vTaskDelay(pdMS_TO_TICKS(10000));
  
  // Phase 2: Hold 25 degrees for 10s
  Serial.println("Phase 2: Target 25 deg (10s)");
  controlSetTargetYaw(25.0f);
  vTaskDelay(pdMS_TO_TICKS(10000));

  // Phase 2.5: Return to 0 degrees for 5s
  Serial.println("Phase 2.5: Target 0 deg (5s)");
  controlSetTargetYaw(0.0f);
  vTaskDelay(pdMS_TO_TICKS(5000));
  
  // Phase 3: Hold -25 degrees for 10s
  Serial.println("Phase 3: Target -25 deg (10s)");
  controlSetTargetYaw(-25.0f);
  vTaskDelay(pdMS_TO_TICKS(10000));

  // Phase 3.5: Return to 0 degrees for 5s (Prepare for tracking)
  Serial.println("Phase 3.5: Return to 0 deg (5s)");
  controlSetTargetYaw(0.0f);
  vTaskDelay(pdMS_TO_TICKS(5000));
  
  // Phase 4: Trapezoid Wave Tracking (Random segments)
  // More suitable for systems with significant delay - has flat holding periods
  Serial.println("Phase 4: Trapezoid Wave Tracking (Random Segments)");
  
  // Define a random trapezoid sequence: {target_angle, ramp_time_ms, hold_time_ms}
  // Format: Ramp from previous target to this target over ramp_time, then hold for hold_time
  struct TrapezoidSegment {
    float targetAngle;
    unsigned long rampTimeMs;
    unsigned long holdTimeMs;
  };
  
  // Pre-defined "random" trapezoid sequence for repeatable testing
  TrapezoidSegment segments[] = {
    {  0.0f, 1000, 5000 },  // Start at 0, hold 5s
    { 20.0f, 2000, 5000 },  // Ramp to 20 over 2s, hold 4s
    { 10.0f, 1500, 5000 },  // Ramp to 10 over 1.5s, hold 3s
    {-15.0f, 2500, 5000 },  // Ramp to -15 over 2.5s, hold 4s
    { -5.0f, 1000, 5000 },  // Ramp to -5 over 1s, hold 3s
    { 25.0f, 3000, 5000 },  // Ramp to 25 over 3s, hold 5s
    {  0.0f, 2500, 5000 },  // Ramp to 0 over 2.5s, hold 3s
    {-25.0f, 2000, 5000 },  // Ramp to -25 over 2s, hold 4s
    { 15.0f, 3000, 5000 },  // Ramp to 15 over 3s, hold 4s
    {  0.0f, 2000, 5000 },  // Return to 0 over 2s, hold 3s (end)
  };
  const int numSegments = sizeof(segments) / sizeof(segments[0]);
  
  float currentTarget = 0.0f; // Starting point
  
  for (int i = 0; i < numSegments; i++) {
    float startAngle = currentTarget;
    float endAngle = segments[i].targetAngle;
    unsigned long rampTime = segments[i].rampTimeMs;
    unsigned long holdTime = segments[i].holdTimeMs;
    
    Serial.print("Segment "); Serial.print(i + 1);
    Serial.print(": Ramp "); Serial.print(startAngle, 1);
    Serial.print(" -> "); Serial.print(endAngle, 1);
    Serial.print(" ("); Serial.print(rampTime); Serial.print("ms), Hold ");
    Serial.print(holdTime); Serial.println("ms");
    
    // Ramp phase: linearly interpolate from startAngle to endAngle
    unsigned long rampStart = millis();
    unsigned long trackingStart = millis(); // For plotting timestamp
    while (millis() - rampStart < rampTime) {
      float t = (float)(millis() - rampStart) / (float)rampTime; // 0.0 to 1.0
      float target = startAngle + t * (endAngle - startAngle);
      controlSetTargetYaw(target);
      
      // Output tracking data: TRACK,time_ms,target,measured
      SensorData data = getSensorData();
      Serial.print("TRACK,");
      Serial.print(millis() - trackingStart);
      Serial.print(",");
      Serial.print(target, 2);
      Serial.print(",");
      Serial.print(data.yawAngle, 2);
      Serial.print(",");
      Serial.println(data.gyroRate, 2);
      
      vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update
    }
    
    // Ensure we hit the exact target at end of ramp
    controlSetTargetYaw(endAngle);
    currentTarget = endAngle;
    
    // Hold phase: maintain the target angle, keep outputting data
    unsigned long holdStart = millis();
    while (millis() - holdStart < holdTime) {
      SensorData data = getSensorData();
      Serial.print("TRACK,");
      Serial.print(millis() - trackingStart);
      Serial.print(",");
      Serial.print(endAngle, 2);
      Serial.print(",");
      Serial.print(data.yawAngle, 2);
      Serial.print(",");
      Serial.println(data.gyroRate, 2);
      
      vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update
    }
  }
  */
  
  Serial.println("=== Maneuver Sequence Completed ===");
  Serial.println("Holding final target (0 deg)...");
  controlSetTargetYaw(0.0f);
  
  // Task deletes itself when done
  vTaskDelete(NULL);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  
  Console.println("Servo Control System Initializing with FreeRTOS...");
  
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
    Console.println("FATAL: Failed to create mutexes");
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }
  
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Console.println("An error occurred while mounting SPIFFS");
    // Error pattern - very fast blink
    ledBlinkInterval = 50;
    return;
  }
  Console.println("SPIFFS mounted successfully");
  
  // Configure LEDC channel
  ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  
  // Attach LEDC channel to GPIO pin
  ledcAttachPin(SERVO_PIN, LEDC_CHANNEL);
  
  // Set servo to center position (90 degrees)
  setServoAngle(90);

  // Start control task (cascaded PID) so auto-control actually runs
  controlBegin();
  controlEnable(true); // Enable auto-control by default
  Console.println("Control task started");
  
  // Initialize H30 sensor
  initH30(H30_RX_PIN, H30_TX_PIN, H30_BAUD);
  Console.println("H30 Serial opened, waiting for data...");
  // mpuInitialized will be set true in mpuTask when data arrives

  // Explicitly reset control state (PIDs, target yaw)
  controlReset();
  Console.println("System state reset: Servo centered, IMU zeroed, Control reset.");
  
  // Start the maneuver task
  xTaskCreate(maneuverTask, "ManeuverTask", 4096, NULL, 1, NULL);
  
#if ENABLE_WIFI
  Console.println("WiFi mode enabled");
  
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
  Console.println("Web server started");
  
  if (connected) {
    Console.println("System ready in STA mode");
    Console.print("Access control panel at: http://");
    Console.println(WiFi.localIP());
  } else {
    Console.println("System ready in AP mode");
    Console.print("Connect to WiFi network: ");
    Console.println(apSSID);
    Console.print("Password: ");
    Console.println(apPassword);
    Console.print("Then navigate to: http://");
    Console.println(WiFi.softAPIP());
  }
#else
  Console.println("WiFi disabled - Serial-only mode");
  Console.println("System ready for serial control");
  Console.println("Commands: s<angle> (e.g., s90 to set servo to 90 degrees)");
  Console.println("  c0|c1 or c - disable/enable auto-control");
  Console.println("  t<angle> - set target yaw (deg, -180..180)");
  Console.println("  C - show control status (target, desired_rate, rate_cmd)");
  Console.println("  o | o1 | o0 - toggle | enable | disable invert of control output (reverse direction)");
  Console.println("  m<deg> - set controller +/- angle limit in degrees (e.g. m25). No arg prints current limit.");
  Console.println("  d | d1 | d0 - toggle | enable | disable control debug prints");
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