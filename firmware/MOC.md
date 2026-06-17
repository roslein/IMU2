# Firmware MOC

---

## 미가공 센서 데이터 스트리밍

[[raw/raw.ino]]
: 아두이노 혹은 ESP32 하드웨어단에서 ISM330DHCX(IMU) 및 MMC5983MA(자력계)의 데이터를 패킷 구조에 맞춰 PC로 고속 송출하는 펌웨어.

[[raw/calibration/calib_params.h]]
: 미가공 데이터를 관측하기 위한 기본 단위 스케일 및 제로 오프셋 세팅용 C++ 기본 매개변수 헤더 파일.

---

## 보정 완료 센서 데이터 스트리밍

[[calibrated/calibrated.ino]]
: generate_calib_params.py 툴체인을 통해 도출된 calib_params.h 교정 파라미터(12-param 및 9-param)를 온칩에서 가속도, 자이로, 자력 데이터에 실시간 대수 연산 적용한 뒤 교정된 데이터를 PC로 전송하는 펌웨어.

[[calibrated/calibration/calib_params.h]]
: 12-parameter 가속도 오차 행렬, 9-parameter 자력계 오차 행렬, 자이로 오프셋이 정밀 이식되어 calibrated.ino 구동 시 직접 참조되는 실시간 보정 상수 헤더 파일.
