# 🎯 Real-world IMU & Orientation Tracking Project (v0.3.0)

본 프로젝트는 정20면체 지그 및 수평 회전판을 활용하여 실물 고성능 MEMS 관성/자기 센서(**`ISM330DHCX`** & **`MMC5983MA`**)의 정밀 캘리브레이션 및 쿼터니언 기반 정적 무회전 3D 자세 추정(Orientation Tracking)을 수행하는 통합 개발 시스템입니다.

---

## 🏛️ 1. 핵심 아키텍처 및 설계 특징

*   **2-Stage 하이브리드 지자기 보정 (Task-Aware 최적화)**:
    *   **Stage 1 (기하 초기화)**: 자력계 3-param(Offset), 6-param(Scale), 9-param(Soft-iron Ellipsoid) 피팅을 가동하여 초기 왜곡 모델 복원.
    *   **Stage 2 (Task-Aware 최적화)**: 수평 회전판의 실제 회전 각도(Yaw GT)와 비교하여, Heading(Yaw) 추정 오차 자체를 직접 최소화하는 Scipy `least_squares` 비선형 최소제곱 최적화 수행.
*   **원클릭 통합 보정 및 펌웨어 실시간 이식**:
    *   [integrated_calibration.py](file:///d:/바탕화면/CS-Study-Tracker/IMU/calibration_tool/integrated_calibration.py) 단 한 번의 실행으로 가속도계 12-parameter LS 피팅, 자이로 Global 오프셋 바이어스, 자력계 Task-Aware 최적화 및 5대 지표 대조 모델 자동 낙찰을 순차적으로 수행합니다.
    *   도출된 최종 교정 파라미터는 펌웨어 컴파일 디렉토리의 [calib_params.h](file:///d:/바탕화면/CS-Study-Tracker/IMU/firmware/calibrated/calibration/calib_params.h) 헤더 파일로 실시간 즉시 이식되어 빌드 툴체인을 결합합니다.
*   **환경 지자기 매핑 폐기 및 인천 복각 표준 절대 고정**:
    *   Yaw 정합도를 저해하던 3D 환경 자북 지도 생성 단계를 완전 소거하고, 대한민국 인천의 표준 지자기 복각 상수 벡터($m_{ned\_ref} = [0.583503, 0.0, 0.812108]$)를 절대 자세 추정기([static_initialization.py](file:///d:/바탕화면/CS-Study-Tracker/IMU/orientation_tracking/static_initialization.py))의 절대 레퍼런스로 고정 인가합니다.
*   **고속 Binary 패킷 프로토콜**:
    *   ASCII 전송 및 문자열 파싱 부하 차단 ➔ 39-Byte raw binary 패킷 구조를 확립했습니다.
    *   `[START (0xAA)] + [Payload (Float 9축 데이터 36 Bytes)] + [XOR Checksum] + [END (0x55)]`
*   **통신 링 버퍼(Ring Buffer) 및 실시간 GUI 가이드**:
    *   비동기 링 버퍼링을 사용해 패킷 유실률 0%를 달성했으며, [data_collection.py](file:///d:/바탕화면/CS-Study-Tracker/IMU/calibration_tool/data_collection.py) 실행 시 정20면체 법선 매칭 실시간 3D 프리뷰와 임시 수집 체크포인트(`checkpoint_data_9axis.npz`)를 통한 자동 복원(이어받기)을 지원합니다.

---

## 🚀 2. 프로젝트 트리 구조 (Project Tree)

```text
/IMU
├── /firmware/           # MCU (아두이노 C++) 펌웨어
│   ├── /raw/            # raw LSB 원시 데이터 송출 펌웨어 (raw.ino)
│   ├── /calibrated/     # 보정 헤더 적용 실시간 교정 데이터 송출 펌웨어 (calibrated.ino)
│   └── /libraries/      # 공용 아두이노 C++ 라이브러리 소스코드
│        └── /IMU_Core/
│             ├── library.properties  (아두이노 라이브러리 규약 명세)
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

---

## 🛠️ 3. 시작하기 (Quick Start)

### 1) 1단계: 원시 데이터 수집 및 인터페이스 검증 (Phase 1)
1. **펌웨어 업로드**: 공용 라이브러리 `IMU_Core`를 아두이노 IDE에 마운트한 후, `firmware/raw/raw.ino`를 업로드하여 미보정 LSB 데이터를 송출합니다.
2. **실측 데이터 수집**: `python calibration_tool/data_collection.py` 를 실행합니다.
   * 지그의 안내에 따라 20개 면을 수평 회전판에 밀착 안착시키고, 면당 12개 눈금(30도 간격)을 회전시키며 [Enter] 키로 1.5초(150샘플) 평균 데이터를 획득합니다 (총 240포인트 수집).
   * 중간에 연결이 끊겨도 다시 실행 시 자동으로 기존 지점부터 **이어받기(Resume)**가 가동됩니다.
3. **통신 무결성 검증**:
   * `python verification_tool/test_phase1.py` 를 통해 바이너리 수신 스레드 정합률과 체크섬 성공 여부를 확인합니다.

### 2) 2단계: 원클릭 통합 보정 및 펌웨어 헤더 실시간 이식 (Phase 2)
1. **통합 보정 기동**:
   * `python calibration_tool/integrated_calibration.py` 를 가동합니다.
   * `collected_data_9axis.npz` 파일이 로드되어 가속도 12-parameter, 자이로 바이어스, 자력 3p/6p/9p 기하 초기화 및 Stage 2 Task-Aware 최적화가 연쇄 실행됩니다.
   * 5대 평가지표 RMSE가 최소인 모델이 자동 낙찰되고, `calib_params.npz` 통합 파라미터 파일이 백업됩니다.
   * 보정 계수가 컴파일 완료된 `calib_params.h` 파일이 펌웨어 경로([calib_params.h](file:///d:/바탕화면/CS-Study-Tracker/IMU/firmware/calibrated/calibration/calib_params.h))로 **즉시 실시간 덮어쓰기 이식**됩니다.

### 3) 3단계: 보정 펌웨어 탑재 및 절대 자세 추정 정합 (Phase 3)
1. **보정 펌웨어 업로드**: `firmware/calibrated/calibrated.ino` 를 MCU에 컴파일 업로드합니다. 보정이 전역 인가된 9축 Float 데이터가 실시간 100Hz 속도로 스트리밍됩니다.
2. **절대 자세 복조**:
   * `python orientation_tracking/static_initialization.py` 를 기동합니다.
   * 센서에서 송출되는 보정 데이터를 인천 고정 복각 레퍼런스 방향과 SVD 정합(align_vectors)하여 절대 Roll, Pitch, Yaw 각도를 계산합니다.

---

## 📚 4. 학술 및 이론 지식 네트워크 (Obsidian PARA)

*   [[실물_IMU_캘리브레이션_및_자세추정_구현_계획]] ➔ 프로젝트의 전체 마일스톤 및 체크리스트
*   [[지자기 센서 보정 및 Task-Aware 최적화 계획 v0.3.md]] ➔ Task-Aware 비선형 최소제곱 수렴 오차 설계안
*   [[5. Theory_AccelMag_기반_NED자세추정]] ➔ SVD Wahba 문제 해결 기법 및 DCM 구성 이론
*   [[4. 쿼터니온과 수학적 원리]] ➔ 짐벌 락(Gimbal Lock) 방지 및 복소 지수 쿼터니언 수학 원리
*   Shortest Arc 최소 회전 정합 및 Modulo 회전 대칭성 보상 기법 ➔ 20 Positions 지그 안착에 따른 기하학적 거치 편차 소거 기법
