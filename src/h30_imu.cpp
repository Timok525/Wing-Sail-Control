#include "h30_imu.h"
#include <HardwareSerial.h>

// H30 State Machine
typedef enum {
    H30_WAITHEAD = 0,
    H30_STARTRECV,
    H30_CHECKSUM,
    H30_HANDLEDATA,
}H30Serial_Status;

static H30Serial_Status H30_State = H30_WAITHEAD;
static H30FrameType_t H30Frame = { 0 };
static const uint16_t H30FrameLen = sizeof(H30FrameType_t);
static uint8_t H30FrameBuf[sizeof(H30FrameType_t)] = { 0 };
static uint16_t H30FrameRecvIndex = 0;

static HardwareSerial H30Serial(1);
static volatile bool isInitialized = false;

// Zeroing variables
static float yawOffset = 0.0f;
static volatile bool zeroingRequest = true;

// Internal helper for normalization
static float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static uint16_t H30_CheckSum(uint8_t* data, uint16_t len) {
    uint8_t ck1 = 0, ck2 = 0;
    for(uint16_t i = 0; i < len; i++) {
        ck1 += data[i];
        ck2 += ck1;
    }
    uint16_t ck = (ck1 << 8) | ck2;
    return ck;
}

bool initH30(int8_t rxPin, int8_t txPin, long baud) {
    H30Serial.begin(baud, SERIAL_8N1, rxPin, txPin);
    return true;
}

int H30_Available() {
    return H30Serial.available();
}

uint8_t H30_Read() {
    return H30Serial.read();
}

void H30_RequestZero() {
    zeroingRequest = true;
    Serial.println("IMU Zero Request Sent to Module"); 
}

bool H30_IsInitialized() {
    return isInitialized;
}

bool H30_ParseByte(uint8_t recv) {
    static uint8_t lastrecv;
    bool isFrameReady = false;
    uint16_t checksumVal = 0;
    
    switch (H30_State) {
      case H30_WAITHEAD:
          if( lastrecv == 0x59 && recv == 0x53 ) {
              H30FrameBuf[0] = 0x59;
              H30FrameBuf[1] = 0x53;
              H30FrameRecvIndex = 2;
              H30_State = H30_STARTRECV;
          }
          break;
      case H30_STARTRECV:
          H30FrameBuf[H30FrameRecvIndex++] = recv;
          if( H30FrameRecvIndex == H30FrameLen ) {
              H30_State = H30_CHECKSUM;
          }
          break;
  
      case H30_CHECKSUM:
          checksumVal = H30_CheckSum(&H30FrameBuf[2], H30FrameLen-4);
          if( (checksumVal>>8&0xff) == H30FrameBuf[H30FrameLen-2] && (checksumVal&0xff) == H30FrameBuf[H30FrameLen-1] ) {
              H30_State = H30_HANDLEDATA;
          } else {
              H30_State = H30_WAITHEAD;
          }
          break;
      case H30_HANDLEDATA:
          memcpy(&H30Frame, H30FrameBuf, H30FrameLen);
          H30_State = H30_WAITHEAD;
          isFrameReady = true;
          isInitialized = true;
          
          // Handle Zeroing inside parsing to ensure atomic snapshot logic
          if (zeroingRequest) {
              float rawYaw = (float)H30Frame.attitude.yaw * 0.000001f;
              yawOffset = rawYaw;
              zeroingRequest = false;
              Serial.print("IMU Zeroed. Raw: ");
              Serial.print(rawYaw);
              Serial.print(", Offset: ");
              Serial.println(yawOffset);
          }
          break;
      default:
          break;
    }
    lastrecv = recv;
    return isFrameReady;
}

float H30_GetYaw() {
    float rawYaw = (float)H30Frame.attitude.yaw * 0.000001f;
    return normalizeAngle(rawYaw - yawOffset);
}

// Helper to see absolute reading
float H30_GetRawYaw() {
    return (float)H30Frame.attitude.yaw * 0.000001f;
}

float H30_GetPitch() {
    return (float)H30Frame.attitude.pitch * 0.000001f;
}

float H30_GetRoll() {
    return (float)H30Frame.attitude.roll * 0.000001f;
}

float H30_GetGyroZ() {
    return (float)H30Frame.gyro.gz * 0.000001f;
}
