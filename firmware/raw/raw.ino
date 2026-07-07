/*
 * Real-world IMU Phase 1 Firmware (raw.ino) - Modularized
 * Target Sensors: ISM330DHCX (6DoF IMU) & MMC5983MA (3축 자력계)
 * 통신 규격: 39-Byte Binary Packet Stream (100Hz ODR)
 * 패킷 프레임: [0xAA] + [Float 9축 데이터 (36 Bytes)] + [XOR Checksum] + [0x55]
 * 특징: imu_protocol.h 와 imu_hardware.h 공용 라이브러리를 활용하여 모듈화되었습니다.
 */

#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <imu_protocol.h>
#include <imu_hardware.h>

// 1. 센서 드라이버 객체 생성
SparkFun_ISM330DHCX myISM;
SFE_MMC5983MA myMag;

// 2. 샘플링 및 동기화 주기 관리 변수
unsigned long lastUpdate = 0;
const float sampleRate = 100.0; // 100Hz 목표 (10ms 주기)
const unsigned long intervalMs = 1000.0 / sampleRate;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  // 컴파일 타임 패킷 구조체 39-Byte 정합성 검증 강제 가드
  static_assert(sizeof(IMUPacket) == 39, "Error: IMUPacket struct padding alignment mismatch!");

  // 공용 하드웨어 HAL 셋업 기동
  if (!setup_imu_sensors(myISM, myMag)) {
    while (1) {
      Serial.println("Error: IMU Hardware Setup Fail!");
      delay(2000);
    }
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // 100Hz ODR 스케줄링 동기화 (누적 오차 제거)
  if (currentMillis - lastUpdate >= intervalMs) {
    lastUpdate += intervalMs;

    // 1. 센서 칩 상태 체크 및 raw 버퍼 로드
    sfe_ism_data_t accelData, gyroData;
    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);

    uint32_t rawMx, rawMy, rawMz;
    myMag.readFieldsXYZ(&rawMx, &rawMy, &rawMz);

    // 2. 물리 단위 환산 및 축 위상 반전 (Preprocessing & HAL)
    float ax = accelData.xData;
    float ay = accelData.yData;
    float az = accelData.zData;

    // 자이로: 500dps 감도 ➔ rad/s 단위 환산
    float gx = (gyroData.xData / 1000.0) * (PI / 180.0);
    float gy = (gyroData.yData / 1000.0) * (PI / 180.0);
    float gz = (gyroData.zData / 1000.0) * (PI / 180.0);

    // 자력계: 18-bit Unsigned ➔ Zero-center Signed 정렬
    float mx = (float)rawMx - 131072.0;
    float my = (float)rawMy - 131072.0;
    float mz = -((float)rawMz - 131072.0); // 자력계 Z축 위상 반전

    // 3. Binary 패킷 프레이밍
    IMUPacket packet;
    packet.start_byte = START_BYTE;
    
    packet.accel[0] = ax; packet.accel[1] = ay; packet.accel[2] = az;
    packet.gyro[0]  = gx; packet.gyro[1]  = gy; packet.gyro[2]  = gz;
    packet.mag[0]   = mx; packet.mag[1]   = my; packet.mag[2]   = mz;

    // 4. 공용 체크섬 연산 적용
    packet.checksum = calculate_imu_checksum(packet.accel, packet.gyro, packet.mag);
    packet.end_byte = END_BYTE;

    // 5. 바이너리 패킷 송출
    Serial.write((uint8_t*)&packet, sizeof(packet));
  }
}
