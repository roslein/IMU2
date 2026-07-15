# 9축 통합 Task-Aware 캘리브레이션 및 이식 파이프라인

본 프로젝트는 정20면체 지그 및 수평 회전판을 활용하여 실물 고성능 MEMS 관성/자기 센서(ISM330DHCX & MMC5983MA)의 정밀 캘리브레이션 및 쿼터니언 기반 정적 무회전 3D 자세 추정(Orientation Tracking)을 수행하는 통합 개발 시스템입니다.

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

## 4. 구현 중 체크

- [x] `data_collection.py` 내의 `collected_data_9axis.npz`에 가속도, 자력, 자이로, `yaw_gt` 리스트가 누락 없이 패킹되었는지 확인한다.
- [x] 자력계 Stage 2 최적화 솔버 `calibrate_mag_task_aware` 의 입력 초기값으로 Stage 1 Ellipsoid 피팅 결과가 정상 전달되는지 확인한다.
- [x] EKF 필터가 아닌, 순수 SVD align_vectors 3D 정합 가정이 적용되었으므로 중력 가속도 물리 방향이 Downward로 올바르게 수렴하는지 확인한다.
- [x] 수집 중단 시 `checkpoint_data_9axis.npz` 파일이 생성되며, 재기동 시 정상적으로 기존 인덱스부터 이어서 수집을 개시하는지 확인한다.
- [x] **[Phase 1 검증]** `data_collection.py` 구동 전후로 `verification_tool/test_phase1.py`를 실행하여 39-Byte 패킷 체크섬 성공률 및 자이로 바이어스 정합성이 확인되었는가?
- [x] **[Phase 2 검증]** 보정 러너 가동 후 `verification_tool/test_phase2.py`를 실행하여 가속도 RMSE가 1.0g 구면에 수렴하고 자력계 틸트 보정 방향이 Upward 규격을 준수하는지 검증하였는가?
- [x] **[Phase 3 검증]** 보정 펌웨어 탑재 및 자세 추정 후 `verification_tool/test_phase3.py`를 실행하여 20개 포지션 방위각 바이어스 및 Modulo 보상 Quiver 3D 화살표 정합 상태를 시각적으로 확인하였는가?

## 5. 주의 및 경고 사항 (System Inversion)

> [!CAUTION]
> **오작동 위험 행동 (Absolute Don'ts)**
> - **파편화 데이터 수집 금지**: 9축 원시 데이터를 하나의 파일(`collected_data_9axis.npz`)로 동시 결합 수집하지 않고, 가속도/자력/자이로 데이터를 각각 서로 다른 시점에 개별 수집하여 정합하면 자세 계산 시 기하 결합성이 무너집니다.
> - **보정 로직 하드코딩 방치 금지**: `imu_core` 내에 모듈화된 솔버 연산 구조를 무시하고, 개별 스크립트에 피팅 수식을 복사 붙여넣기로 하드코딩 방치 시 버전 파편화 오차가 유발됩니다.

> [!WARNING]
> **성능 저하 행위 (Recurring Anti-patterns)**
> - **수동 헤더 복사 금지**: 통합 보정 후 펌웨어 경로 내 `calib_params.h`를 자동 이식하지 않고 수동 복사 이송 시, 휴먼 에러로 인한 오이송 누락 에러가 반복 발생할 수 있습니다.
> - **고차원 모델 무조건적 맹신 금지**: 6-parameter 및 9-parameter 피팅 시 복각(Dip Angle) RMSE가 지나치게 왜곡되거나 수축(Collapse)할 경우, 3-parameter(Offset Only) 물리적 강건 대조군과의 수치적 비교 없이 적용하지 마십시오.

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

- **MATLAB 공식 융합/보정 강의 및 예제 (Web Links)**:
  - [MATLAB Understanding Sensor Fusion and Tracking Lecture Series](https://youtube.com/playlist?list=PLn8PRpmsu08rneZErjW_NIBs0Rl_vcgSw&si=8yu4xianW--leL4q) (MathWorks 공식 센서 융합 및 추적 유튜브 재생목록 강의)
  - [MATLAB Understanding Kalman Filters Video Series](https://kr.mathworks.com/videos/series/understanding-kalman-filters.html?s_eid=PSM_22735) (MathWorks 공식 칼만 필터의 이해 비디오 시리즈)
  - [MATLAB Magnetometer Calibration Guide](https://kr.mathworks.com/help/fusion/ug/magnetometer-calibration.html) (MathWorks 공식 자력계 하드/소프트 아이언 보정 가이드 및 수식 설명)
  - [MATLAB Estimating Orientation Using Inertial Sensor Fusion](https://kr.mathworks.com/help/fusion/ug/Estimating-Orientation-Using-Inertial-Sensor-Fusion-and-MPU-9250.html) (MathWorks 공식 관성 센서 융합 및 MPU-9250 기반 자세 추정 예제)

- **학술 연구 강의 노트 (Web Links)**:
  - [Stanford EE267 Notes: IMU & Orientation Tracking](https://web.stanford.edu/class/ee267/notes/notes_imu.pdf) (Stanford 대학교 EE267 가속도/자력/자이로 3-DOF 정합 강의 노트)
