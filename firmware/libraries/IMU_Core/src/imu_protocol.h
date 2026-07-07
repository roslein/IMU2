#ifndef _IMU_PROTOCOL_H_
#define _IMU_PROTOCOL_H_

#include <Arduino.h>

#define PACKET_SIZE 39
#define START_BYTE 0xAA
#define END_BYTE 0x55

#pragma pack(push, 1)
struct IMUPacket {
  uint8_t start_byte;  // 0xAA
  float accel[3];      // 3축 가속도 (12 Bytes)
  float gyro[3];       // 3축 자이로 (12 Bytes)
  float mag[3];        // 3축 지자기 (12 Bytes)
  uint8_t checksum;    // XOR 체크섬
  uint8_t end_byte;    // 0x55
};
#pragma pack(pop)

// 인라인 체크섬 함수 정의 (헤더 단독 탑재용)
inline uint8_t calculate_imu_checksum(const float* accel, const float* gyro, const float* mag) {
    uint8_t xor_sum = START_BYTE;
    const uint8_t* ptr = (const uint8_t*)accel;
    for (size_t i = 0; i < 36; i++) { // float 9개 = 36 Bytes 페이로드
        xor_sum ^= ptr[i];
    }
    return xor_sum;
}

#endif // _IMU_PROTOCOL_H_
