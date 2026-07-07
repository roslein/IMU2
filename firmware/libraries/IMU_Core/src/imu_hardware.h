#ifndef _IMU_HARDWARE_H_
#define _IMU_HARDWARE_H_

#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

// I2C 레지스터 직접 제어 도우미 함수
void writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value);

// 센서 통합 셋업 및 ODR/LPF 하드웨어 가드 기동
bool setup_imu_sensors(SparkFun_ISM330DHCX &ism, SFE_MMC5983MA &mag);

#endif // _IMU_HARDWARE_H_
