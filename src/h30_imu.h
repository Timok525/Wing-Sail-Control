#ifndef H30_IMU_H
#define H30_IMU_H

#include <Arduino.h>

// Struct definitions
#pragma pack(1)
typedef struct{
    uint8_t dataId;
    uint8_t dataLen;
    int32_t ax;
    int32_t ay;
    int32_t az;
}AccelRawType_t;

typedef struct{
    uint8_t dataId;
    uint8_t dataLen;
    int32_t gx;
    int32_t gy;
    int32_t gz;
}GyroRawType_t;

typedef struct{
    uint8_t dataId;
    uint8_t dataLen;
    int32_t pitch;
    int32_t roll;
    int32_t yaw;
}AttitudeType_t;

typedef struct{
    uint8_t dataId;
    uint8_t dataLen;
    int32_t q0;
    int32_t q1;
    int32_t q2;
    int32_t q3;
}QuaternionType_t;

typedef struct{
    uint8_t head1;   // Head
    uint8_t head2;
    uint16_t FrameNum; // Frame Num
    uint8_t packLen;   // Length

    AccelRawType_t accel; // 14 bytes
    GyroRawType_t gyro;                           // 14 bytes
    AttitudeType_t attitude;                      // 14 bytes
    QuaternionType_t quaternion;                  // 18 bytes

    uint8_t ck1; // Checksum
    uint8_t ck2;
}H30FrameType_t;
#pragma pack()

// Functions
bool initH30(int8_t rxPin, int8_t txPin, long baud);
void H30_RequestZero();
bool H30_IsInitialized();

// Serial processing
bool H30_ParseByte(uint8_t recv);
int H30_Available();
uint8_t H30_Read();

// Data Getters (return units in Deg, Deg/s)
float H30_GetYaw();
float H30_GetPitch();
float H30_GetRoll();
float H30_GetGyroZ();
float H30_GetRawYaw(); // Helper for debug

#endif
