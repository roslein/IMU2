#include <Wire.h>
#include <Arduino.h>
#include "imu_hardware.h"

void writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.write(value);
    Wire.endTransmission();
}

bool setup_imu_sensors(SparkFun_ISM330DHCX &ism, SFE_MMC5983MA &mag) {
    // 1. 센서 시작 감지 검증
    if (!ism.begin()) return false;
    if (!mag.begin()) return false;

    // 2. 센서 하드웨어 소프트 리셋 및 안전 딜레이
    ism.deviceReset();
    mag.softReset();
    delay(100);

    // 3. ISM330DHCX 가속도 ±4g, 속도 104Hz / 자이로 ±500dps, 속도 104Hz 설정
    ism.setAccelDataRate(ISM_XL_ODR_104Hz);
    ism.setAccelFullScale(ISM_4g);
    ism.setGyroDataRate(ISM_GY_ODR_104Hz);
    ism.setGyroFullScale(ISM_500dps);

    // 4. 고주파 모터 진동 차단 및 안티 에일리어싱을 위한 LPF 활성화 (12.5Hz 차단 주파수)
    // 0x6B (ISM330DHCX I2C 주소), 0x15 (CTRL6_C 레지스터 주소), 0x02 (FTYPE 필터 비트값)
    writeRegister(0x6B, 0x15, 0x02); 

    // 5. MMC5983MA 자동 Set/Reset(온도 드리프트 상쇄) 및 100Hz 연속 측정 모드 기동
    // 0x30 (MMC5983MA I2C 주소), 0x09 (Internal Control 0 레지스터 주소), 0x2A (Auto_SR 및 Period_Set 설정값)
    writeRegister(0x30, 0x09, 0x2A); 
    
    mag.setContinuousModeFrequency(100);
    mag.enableContinuousMode();

    return true;
}
