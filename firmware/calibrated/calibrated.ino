/*
 * Real-world IMU Phase 2 Calibrated Firmware (calibrated.ino)
 * Target Sensors: ISM330DHCX (6DoF IMU) & MMC5983MA (3축 자력계)
 * 통신 규격: 39-Byte Binary Packet Stream (100Hz ODR)
 * 패킷 프레임: [0xAA] + [Float 9축 보정 완료 데이터 (36 Bytes)] + [XOR Checksum] + [0x55]
 * 특징: calib_params.h 영점 상수를 실시간 행렬 연산하여 완벽히 교정된 자세 추정용 신호를 송출합니다.
 */

#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include "calibration/calib_params.h"

// 1. 센서 드라이버 객체 생성
SparkFun_ISM330DHCX myISM;
SFE_MMC5983MA myMag;

// 2. 샘플링 및 동기화 주기 관리 변수
unsigned long lastUpdate = 0;
const float sampleRate = 100.0; // 100Hz 목표 (10ms 주기)
const unsigned long intervalMs = 1000.0 / sampleRate;

// 3. 39-Byte 패킷 구조체 정의 (패딩 차단을 위한 1바이트 정렬)
#pragma pack(push, 1)
struct IMUPacket {
  uint8_t start_byte;  // 0xAA
  float accel[3];      // 3축 보정 완료 가속도 물리 단위 1g (12 Bytes)
  float gyro[3];       // 3축 자이로 rad/s (12 Bytes)
  float mag[3];        // 3축 보정 완료 자자기 구면화 Norm 1.0 (12 Bytes)
  uint8_t checksum;    // 시작 바이트부터 데이터 페이로드까지의 XOR 연산 값
  uint8_t end_byte;    // 0x55
};
#pragma pack(pop)

// 4. 로우/정밀 센서 데이터 저장 구조체
sfe_ism_data_t accelData, gyroData;

// 5. I2C 레지스터 직접 제어 도우미 함수 (HAL & Driver)
void writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  // 센서 감지 검증
  if (!myISM.begin()) {
    while (1) {
      Serial.println("Error: ISM330DHCX I2C 연결 실패!");
      delay(2000);
    }
  }
  if (!myMag.begin()) {
    while (1) {
      Serial.println("Error: MMC5983MA I2C 연결 실패!");
      delay(2000);
    }
  }

  // 센서 하드웨어 리셋 및 안전 딜레이
  myISM.deviceReset();
  myMag.softReset();
  delay(100);

  // [ISM330DHCX] 정밀 드라이버 설정 (가속도/자이로)
  // 가속도: 범위 ±4g, 속도 104Hz
  myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
  myISM.setAccelFullScale(ISM_4g);

  // 자이로: 범위 ±500dps, 속도 104Hz
  myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
  myISM.setGyroFullScale(ISM_500dps);

  // 하드웨어 로우 패스 필터(LPF) 활성화 (FTYPE 약 12.5Hz 차단 주파수)
  writeRegister(0x6B, 0x15, 0x02);

  // [MMC5983MA] 정밀 드라이버 설정 (자력계)
  // 하드웨어 자동 Set/Reset(온도/고착 왜곡 상쇄) 활성화
  writeRegister(0x30, 0x09, 0x2A);

  // 100Hz 연속 측정 모드 설정
  myMag.setContinuousModeFrequency(100);
  myMag.enableContinuousMode();
}

void loop() {
  unsigned long currentMillis = millis();

  // 100Hz ODR 스케줄링 동기화
  if (currentMillis - lastUpdate >= intervalMs) {
    lastUpdate += intervalMs;

    // 1. 센서 칩 상태 체크 및 raw 버퍼 로드
    myISM.checkStatus();
    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);

    uint32_t rawMx, rawMy, rawMz;
    myMag.readFieldsXYZ(&rawMx, &rawMy, &rawMz);

    // 2. 물리 단위 환산 및 축 위상 반전
    
    // [가속도계]: raw LSB count
    float ax = accelData.xData;
    float ay = accelData.yData;
    float az = accelData.zData;

    // [자이로]: 500dps 감도 ➔ 라디안/s 물리 단위 변환
    float gx = (gyroData.xData * 0.0175) * (PI / 180.0);
    float gy = (gyroData.yData * 0.0175) * (PI / 180.0);
    float gz = (gyroData.zData * 0.0175) * (PI / 180.0);

    // [자력계]: 18-bit Unsigned ➔ Zero-center Signed 정렬
    float mx = (float)rawMx - 131072.0;
    float my = (float)rawMy - 131072.0;
    float mz = (float)rawMz - 131072.0;
    mz = -mz; // 자력계 Z축 위상 반전

    // 🎯 3D 캘리브레이션 실시간 보정 연산 적용 (2중 다이렉트 이식 파라미터 활용)
    
    // 가속도계 12-Parameter 보정 적용: acc_cal = ACC_W * (acc_raw - ACC_B)
    float ax_cal = ACC_W[0][0] * (ax - ACC_B[0]) + ACC_W[0][1] * (ay - ACC_B[1]) + ACC_W[0][2] * (az - ACC_B[2]);
    float ay_cal = ACC_W[1][0] * (ax - ACC_B[0]) + ACC_W[1][1] * (ay - ACC_B[1]) + ACC_W[1][2] * (az - ACC_B[2]);
    float az_cal = ACC_W[2][0] * (ax - ACC_B[0]) + ACC_W[2][1] * (ay - ACC_B[1]) + ACC_W[2][2] * (az - ACC_B[2]);

    // 자력계 9-Parameter 타원체 보정 적용: mag_cal = MAG_W * (mag_raw - MAG_B)
    float mx_cal = MAG_W[0][0] * (mx - MAG_B[0]) + MAG_W[0][1] * (my - MAG_B[1]) + MAG_W[0][2] * (mz - MAG_B[2]);
    float my_cal = MAG_W[1][0] * (mx - MAG_B[0]) + MAG_W[1][1] * (my - MAG_B[1]) + MAG_W[1][2] * (mz - MAG_B[2]);
    float mz_cal = MAG_W[2][0] * (mx - MAG_B[0]) + MAG_W[2][1] * (my - MAG_B[1]) + MAG_W[2][2] * (mz - MAG_B[2]);

    // 3. Binary 패킷 프레이밍 (보정된 초정밀 데이터로 전환 실장)
    IMUPacket packet;
    packet.start_byte = 0xAA;
    
    packet.accel[0] = ax_cal; packet.accel[1] = ay_cal; packet.accel[2] = az_cal;
    packet.gyro[0]  = gx;     packet.gyro[1]  = gy;     packet.gyro[2]  = gz;
    packet.mag[0]   = mx_cal; packet.mag[1]   = my_cal; packet.mag[2]   = mz_cal;

    // 4. XOR 체크섬 계산
    uint8_t xor_sum = packet.start_byte;
    uint8_t* ptr = (uint8_t*)&packet.accel[0];
    for (int i = 0; i < 36; i++) {
      xor_sum ^= ptr[i];
    }
    packet.checksum = xor_sum;
    packet.end_byte = 0x55;

    // 5. 고속 바이너리 스트리밍 송출
    Serial.write((uint8_t*)&packet, sizeof(IMUPacket));
  }
}
