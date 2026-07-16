# 9축 통합 Task-Aware 캘리브레이션 및 이식 파이프라인

본 프로젝트는 정20면체 지그 및 수평 회전판을 활용하여 실물 고성능 MEMS 관성/자기 센서(ISM330DHCX & MMC5983MA)의 정밀 캘리브레이션 및 쿼터니언 기반 정적 무회전 3D 자세 추정(Orientation Tracking)을 수행하는 통합 개발 시스템입니다.

## 📋 1. Requirements & Prerequisites (필수 요구 사양 및 의존성)

### 1.1 하드웨어 명세 (Hardware Requirements)
| 장치/모듈명 | 역할 및 기능 | 물리적 사양 및 CAD 링크 |
| --- | --- | --- |
| ESP32 Thing Plus C | 메인 제어 및 센서 HAL, 100Hz 바이너리 스트리밍 송출 | [SparkFun ESP32 Thing Plus - C](https://www.sparkfun.com/products/18029) |
| ISM330DHCX | 6DoF 관성 센서 (가속도 및 자이로 LSB 계측) | I2C 인터페이스 (0x6B) |
| MMC5983MA | 3축 지자기 센서 (자기장 LSB 계측, Z축 부호 반전 정렬) | I2C 인터페이스 (0x30) |
| 정20면체 캘리브레이션 지그 | 20 Positions 자율 안착 수집용 3D CAD STL 규격 | [cad/sensor_20_v1.stl](cad/sensor_20_v1.stl) |
| 정6면체 보정 지그 | 6 Positions 단순 보조 수집용 3D CAD STL 규격 | [cad/sensor_6_v1.stl](cad/sensor_6_v1.stl) |

### 1.2 소프트웨어 의존성 (Software Dependencies)
- Python 3.10 이상 (NumPy, SciPy, OpenCV, Matplotlib)
- Arduino IDE 및 Arduino CLI (ESP32 Board Package v3.x)
- 외부 의존 아두이노 라이브러리: `SparkFun_ISM330DHCX`, `SparkFun_MMC5983MA_Arduino_Library`

## 1. 상황

- 정20면체 지그 x 수평 회전판 결합(240포인트) 데이터를 수집하여 ISM330DHCX/MMC5983MA 센서의 오프셋/스케일을 일괄 보정해야 한다.
- SVD 정합 시 Heading(Yaw) 추정 정확도(목표 1.5도 이내)를 극대화하기 위해 Task-Aware 2-Stage 비선형 최적화를 수행하고 보정 계수를 calibrated.ino 펌웨어로 자동 이식해야 한다.

### Project Tree
```text
/IMU
├── /firmware/           # MCU (아두이노 C++) 펌웨어
│   ├── /raw/            # raw LSB 원시 데이터 송출 펌웨어 (raw.ino)
│   ├── /calibrated/     # 보정 헤더 적용 실시간 교정 데이터 송출 펌웨어 (calibrated.ino)
│   └── /libraries/      # 공용 아두이노 C++ 라이브러리 소스코드
│        └── /IMU_Core/
│             ├── library.properties  (아두이노 라이브러리 명세)
│             └── /src/
│                  ├── imu_protocol.h  (IMUPacket 구조체 및 체크섬 인라인 함수)
│                  ├── imu_hardware.h  (I2C 레지스터 제어 및 HAL 인터페이스 선언)
│                  └── imu_hardware.cpp (ISM330DHCX/MMC5983MA 센서 필터 설정 구현)
│
├── /calibration_tool/   # PC (Python) 오프라인 240포인트 수집 및 통합 보정 러너
│   ├── data_collection.py  # 20면 x 12눈금(240포인트) 9축 통합 수집기 (오토세이브/이어받기)
│   ├── integrated_calibration.py # [Core] 9축 통합 캘리브레이션 및 calib_params.h 빌드/자동이식 툴체인
│   └── /output/            # 수집 데이터셋(.npz) 및 백업 파라미터 보관 폴더
│
├── /imu_core/           # 보정 연산 코어 모듈
│   ├── math.py          # 12-param 가속도, 자이로 오프셋, 자력계 기하/Task-Aware 최적화 pure solvers
│   └── icosahedron.py   # 정20면체 기하학적 면 법선 벡터 매칭 분석 모듈
│
├── /orientation_tracking/ # 절대 자세 추정 모듈
│   └── static_initialization.py # 인천 표준 복각(54.3 deg) 절대 NED 레퍼런스 기반 3D 자세 복조기
│
└── /verification_tool/  # 오프라인 성능 검증 및 윈도우 스캔 시각화 툴
    ├── test_phase1.py   # 시리얼 해상도 및 gyro static bias 기본 확인
    ├── test_phase2.py   # 가속도 1g 구면 피팅 정합도 검증
    └── test_phase3.py   # 절대 자세 정합도 오차 검증 및 Modulo 회전 보상 Quiver 렌더러
```

## 2. Core Cue

> 20면 x 12눈금 데이터를 수집하고, integrated_calibration.py를 구동하여 최적 보정 계수 헤더를 펌웨어 컴파일 경로로 즉시 실시간 배포하라.

## 3. Battle Drill

1. `firmware/raw/raw.ino` 펌웨어를 MCU 보드에 업로드하여 raw binary 텔레메트리 송출을 시작한다.
2. `calibration_tool/data_collection.py` 를 실행하여 3D guide view 안내에 맞추어 20면 x 12눈금(총 240포인트) 9축 데이터를 수집해 `collected_data_9axis.npz`로 저장한다.
3. `calibration_tool/integrated_calibration.py` 를 구동하여 가속도/자이로/자력(Task-Aware) 보정 솔버를 연쇄 기동하고, 최적 RMSE 모델 낙찰 및 `calib_params.h` 헤더를 펌웨어 경로에 덮어쓰기 이식한다.
4. `firmware/calibrated/calibrated.ino` 보정 펌웨어를 MCU 보드에 업로드한다.
5. `orientation_tracking/static_initialization.py` 를 실행하여 인천 표준 복각 레퍼런스 정합 기반 절대 자세 Roll, Pitch, Yaw를 계산 및 저장한다.

## 4. 실행 중 체크

- [x] `data_collection.py` 내의 `collected_data_9axis.npz`에 가속도, 자력, 자이로, `yaw_gt` 리스트가 누락 없이 패킹되었는지 확인한다.
- [x] 자력계 Stage 2 최적화 솔버 `calibrate_mag_task_aware` 의 입력 초기값으로 Stage 1 Ellipsoid 피팅 결과가 정상 전달되는지 확인한다.
- [x] EKF 필터가 아닌, 순수 SVD align_vectors 3D 정합 가정이 적용되었으므로 중력 가속도 물리 방향이 Downward로 올바르게 수렴하는지 확인한다.
- [x] 수집 중단 시 `checkpoint_data_9axis.npz` 파일이 생성되며, 재기동 시 정상적으로 기존 인덱스부터 이어서 수집을 개시하는지 확인한다.
- [x] **[Phase 1 검증]** `data_collection.py` 구동 전후로 `verification_tool/test_phase1.py`를 실행하여 39-Byte 패킷 체크섬 성공률 및 자이로 바이어스 정합성이 확인되었는가?
- [x] **[Phase 2 검증]** 보정 러너 가동 후 `verification_tool/test_phase2.py`를 실행하여 가속도 RMSE가 1.0g 구면에 수렴하고 자력계 틸트 보정 방향이 Upward 규격을 준수하는지 검증하였는가?
- [x] **[Phase 3 검증]** 보정 펌웨어 탑재 및 자세 추정 후 `verification_tool/test_phase3.py`를 실행하여 20개 포지션 방위각 바이어스 및 Modulo 보상 Quiver 3D 화살표 정합 상태를 시각적으로 확인하였는가?

## 5. 주의 및 경고 사항 (Known Limitations & Caution)

> [!CAUTION]
> **사용 시 주의 사항**
> - **9축 데이터는 반드시 동일한 측정 세션에서 동시에 수집하십시오.** 가속도계, 자이로스코프, 자력계를 서로 다른 시점에 수집한 데이터를 혼합하면 보정 및 자세 추정 결과가 올바르지 않을 수 있습니다.
> - **자력계 보정은 자기 간섭이 적은 환경에서 수행하십시오.** 자석, 철제 구조물, 모터, 전원장치 및 대형 금속 물체는 측정값을 왜곡하여 보정 성능을 저하시킬 수 있습니다.
> - **보정 후에는 반드시 성능을 검증하십시오.** 하나의 보정 모델만 신뢰하지 말고, Heading RMSE, Closed-loop Error, Quaternion Error 등의 실제 자세 오차를 함께 확인하는 것을 권장합니다.
> - **수집 도중 회전판 조작 실수 시 대처 방법**: 특정 면의 13눈금 수집 중 회전판을 잘못 회전시켰거나 정렬이 틀어졌을 경우, 프로그램을 즉시 `Ctrl+C`로 강제 종료하고 재기동하십시오. 체크포인트 이어받기(`Y`) 시, 완료되지 않은 해당 면에 한해서는 **0도(첫 눈금)부터 안전하게 다시 처음부터 재수집**하도록 면 단위(Face-level) 무결성을 지원합니다.

> [!WARNING]
> **알려진 제한 사항**
> - **본 프로젝트는 정적(Quasi-static) 자세 추정을 대상으로 설계되었습니다.** 급격한 움직임이나 지속적인 동적 환경에서의 자세 추정은 지원 대상이 아닙니다.
> - **자이로스코프는 정적 바이어스만 보정합니다.** Scale Factor, 축 비직교성(Non-orthogonality), 온도 보정은 현재 포함되어 있지 않습니다.
> - **고차원 자력계 보정은 충분한 자세 다양성이 확보된 데이터셋을 필요로 합니다.** 데이터 분포가 부족하면 6-Parameter 및 9-Parameter 보정은 불안정하거나 오히려 Heading 성능을 저하시킬 수 있습니다.

## 6. 기대 효과 및 결론 (Expected Results & Conclusion)

- **고속 Binary 패킷 프로토콜 및 비동기 링 버퍼링 (Battle Drill Step 1 대응)**:
  - ASCII 문자열 변환 부하를 차단한 39-Byte raw binary 통신과 비동기 링 버퍼링을 결합하여, 100Hz ODR 가동 시 PC 수신 데이터 유실률 0%의 데이터 정밀 수집을 달성합니다.
- **실시간 GUI 가이드 및 체크포인트 오토세이브 (Battle Drill Step 2 대응)**:
  - 정20면체 법선 매칭 3D 프리뷰 갱신으로 거치 정합성을 향상시키며, 통신 중단 시 `checkpoint_data_9axis.npz` 오토세이브 복원 기능을 통해 대규모 240포인트 데카르트 곱 수집 작업의 피로도를 최소화하고 데이터 신뢰성을 확보합니다.
- **2-Stage 하이브리드 지자기 보정 / Task-Aware 최적화 (Battle Drill Step 3 대응)**:
  - 기하학적 3D 타원체 복원에 그치지 않고, Yaw GT 오차 자체를 직접 최소화하는 비선형 최적화(Scipy `least_squares`)를 통해 최종 Heading 추정 정확도를 극대화하여 절대 자세 제어 루프의 정밀도를 향상시킵니다.
- **원클릭 통합 보정 및 펌웨어 실시간 이식 (Battle Drill Step 3 대응)**:
  - `integrated_calibration.py` 를 통해 9축 통합 보정 및 보정 계수의 `calib_params.h` 자동 실시간 이식을 수행하여, 캘리브레이션과 컴파일 간의 수동 데이터 전송에 의한 휴먼 에러를 원천 배제합니다.
- **인천 복각 표준 절대 레퍼런스 고정 적용 (Battle Drill Step 4~5 대응)**:
  - 공간 왜곡에 의한 국소적 지구 자기장 변동을 배제하고 대한민국 인천의 표준 복각 상수 벡터(`[0.583503, 0.0, 0.812108]`)를 정적 절대 레퍼런스로 영구 고정 적용하여, 절대 자세(Roll, Pitch, Yaw) 추정의 장기적 안정성을 보장합니다.

## 7. 참고 자료 (References)

### 공식 문서 (Official Documentation)

- MATLAB Magnetometer Calibration Guide  
  https://kr.mathworks.com/help/fusion/ug/magnetometer-calibration.html

- MATLAB Estimating Orientation Using Inertial Sensor Fusion  
  https://kr.mathworks.com/help/fusion/ug/Estimating-Orientation-Using-Inertial-Sensor-Fusion-and-MPU-9250.html

- MATLAB Understanding Sensor Fusion Lecture Series  
  https://youtube.com/playlist?list=PLn8PRpmsu08rneZErjW_NIBs0Rl_vcgSw

- MATLAB Understanding Kalman Filters  
  https://kr.mathworks.com/videos/series/understanding-kalman-filters.html

---

### 대표 논문 (Key Papers)

- Li, Q. & Griffiths, J. G. (2004).
  Least Squares Ellipsoid Specific Fitting for Magnetometer Calibration.

- Kok, M., Hol, J. D., & Schön, T. B. (2012).
  An Optimization-Based Approach to Human Body Motion Capture Using Inertial Sensors.

- Gebre-Egziabher, D., Elkaim, G. H., Powell, J. D., & Parkinson, B. W.
  Calibration of Strapdown Magnetometers in Magnetic Field Domain.

- Vasconcelos, J. F. et al.
  Geometric Approach to Strapdown Magnetometer Calibration.

---

### 자세 추정 및 센서 융합

- Stanford EE267 Notes – IMU & Orientation Tracking
  https://web.stanford.edu/class/ee267/notes/notes_imu.pdf

- Shuster, M. D.
  A Survey of Attitude Representations.

- Wahba, G.
  A Least Squares Estimate of Satellite Attitude.

- Davenport, P. B.
  A Vector Approach to the Algebra of Rotations.

- QUEST (Quaternion Estimator)

- TRIAD (Tri-Axial Attitude Determination)

