# 🎯 Real-world IMU & Orientation Tracking Project

본 프로젝트는 정20면체(정이십면체) 지그를 활용한 실물 고성능 MEMS 관성/자기 센서(**`ISM330DHCX`** & **`MMC5983MA`**)의 정밀 캘리브레이션 및 쿼터니언 기반 정적 무회전 3D 자세 추정(Orientation Tracking) 통합 시스템입니다.

---

## 🏛️ 1. 핵심 아키텍처 및 설계 특징

*   **임베디드 펌웨어 및 PC 애플리케이션 이원화 아키텍처**:
    *   **Firmware 계층 (임베디드 HAL 및 Driver 통합)**: MCU(아두이노/ESP32) 상에서 I2C 물리 드라이버 제어, 센서 레지스터 직접 설정(ISM330DHCX 하드웨어 LPF 및 MMC5983MA 자동 Set/Reset 노이즈 제거 활성화), 자력계 Z축 부호 반전(-1 곱하기) 보정을 원칩에서 일괄 통합 처리하여 신뢰성 높은 고속 바이너리 텔레메트리 송출을 담당함.
    *   **Calibration 계층 (PC 호스트 - 오프라인 보정)**: 수집된 20개 포지션 원시 데이터를 로드하여 가속도 12-parameter 및 자력 9-parameter 최적화 보정 파라미터를 도출하고, C++ 헤더(calib_params.h)를 자동 컴파일 및 펌웨어 디렉토리로 이식함.
    *   **Orientation Tracking 계층 (PC 호스트 - 실시간/정적 자세 추정 및 오프라인 검증)**: 자북 레퍼런스 맵핑, SVD align_vectors 정적 절대 자세 융합 추정, 윈도우 스캔 분석 및 Modulo 대칭성 기하 오차 보상 등의 하이브리드 검증 알고리즘을 수행함.
*   **고속 Binary 통신 프로토콜**:
    *   ASCII 전송 문자열 변환 부하 차단 ➔ 38-Byte raw binary 패킷 구조 확립
    *   `[START (0xAA)] + [Payload (Float 9축 데이터)] + [XOR Checksum] + [END (0x55)]`
*   **PC 수신 원형 링 버퍼(Ring Buffer)**:
    *   독립 수신 스레드와 비동기 링 버퍼링을 통한 데이터 유실률 0% 달성

---

## 🚀 2. 프로젝트 트리 구조 (Project Tree)

```text
/IMU
├── /firmware/           # MCU (아두이노/ESP32 C++) 펌웨어
│   ├── /raw/            # raw LSB 원시 데이터 송출 펌웨어
│   └── /calibrated/     # 보정 행렬 적용 실시간 교정 데이터 송출 펌웨어
│
├── /calibration_tool/   # PC (Python) 오프라인 20면체 보정 및 자북 매퍼
│   ├── data_collection.py  # 20면체 수집 가이드 및 데이터 적재 (자동 백업 지원)
│   ├── accel_calibration.py # 12-parameter 가속도계 보정 솔버
│   ├── mag_calibration.py   # 9-parameter 자력계 보정 솔버
│   ├── gyro_bias_calibration.py # 20 Positions 기반 전역 자이로 오프셋 보정
│   ├── generate_calib_params.py # calib_params.h C++ 헤더 생성기
│   ├── mag_environment_mapping.py # SVD 기반 3D 환경 자북 벡터 추출기
│   └── MOC.md               # 캘리브레이션 툴 디렉토리 맵
│
├── /orientation_tracking/ # 실시간/정적 절대 자세 추정 모듈
│   ├── static_initialization.py # SVD 정합 기반 정적 절대 자세(Yaw) 복조기
│   └── MOC.md               # 자세 추정 디렉토리 맵
│
└── /verification_tool/  # 오프라인 성능 검증 및 윈도우 스캔 시각화 툴
    ├── test_phase1.py   # 시리얼 해상도 및 gyro static bias 기본 확인
    ├── test_phase2.py   # 가속도 1g 구면 피팅 정합도 검증
    ├── test_phase3.py   # 20 Positions 절대 자세 오차 검증 기본형
    ├── test_phase3_1_static_orientation.py # SVD 정합 및 Modulo 보상 quiver 3D 렌더러
    ├── test_phase3_2_window_analysis.py # T_cal 및 T_est 격자 윈도우 스캔 3D Surface 분석기
    └── MOC.md               # 검증 툴 디렉토리 맵
```

---

## 🛠️ 3. 시작하기 (Quick Start)

### 1) 1단계: 원시 데이터 수집 및 캘리브레이션 (Calibration)
1. 펌웨어 업로드: firmware/raw/raw.ino 를 보드에 업로드하여 무가공 원시 데이터(LSB)를 스트리밍합니다.
2. 실측 데이터 수집: calibration_tool/data_collection.py 를 실행합니다. 20면체를 바닥에 안착시키고 스페이스바/엔터 키를 입력해 3초 평균 raw 데이터 20개 포지션을 수집합니다. (기존 완성본이 있다면 시간 정보를 포함한 백업 파일로 자동 안전 전환 보관됩니다.)
3. 가속도계 12-parameter 교정: calibration_tool/accel_calibration.py 를 기동하여 지면 경사각을 Recursive하게 보상 수렴한 최적 acc_params.npz 파라미터를 도출합니다.
4. 자력계 Hard/Soft iron 교정: calibration_tool/mag_calibration.py 를 기동하여 타원체 피팅 솔버 연산 후 mag_params.npz 파라미터를 저장합니다.
5. 자이로스코프 바이어스 교정: calibration_tool/gyro_bias_calibration.py 를 실행하여 20 Positions 전체 자이로 데이터의 참 평균값을 역산해 gyro_params.npz 파라미터를 저장합니다.
6. C++ 헤더 통합 생성: calibration_tool/generate_calib_params.py 를 실행해 개별 센서 파라미터를 취합하여 calibrated.ino 컴파일 경로로 헤더(calib_params.h)를 자동 이식합니다.

### 2) 2단계: 보정 펌웨어 탑재 및 3D 정적 자세 추정 (Orientation Tracking)
1. 보정 펌웨어 업로드: firmware/calibrated/calibrated.ino 를 보드에 빌드 업로드합니다. 실시간 100Hz로 센서 레벨 보정이 적용된 텔레메트리 스트리밍이 기동됩니다.
2. 환경 지자기 맵핑: calibration_tool/mag_environment_mapping.py 를 가동하여 해당 공간(방)의 왜곡된 3D 자북 레퍼런스 벡터(env_params.npz)를 추출해 둡니다.
3. 정적 절대 3D 자세 역산 기동: orientation_tracking/static_initialization.py 를 실행합니다. 센서를 평평한 바닥에 두고 엔터 키를 치면 SVD align_vectors 정밀 정합을 통해 절대 3D 자세각(Roll, Pitch, Yaw)을 복조하여 static_orientation.npz 에 저장합니다.

### 3) 3단계: 3D 정적 자세 쿼터니언 정량 오차 독립 검증 (Validation)
1. 3D Quiver 및 상대 정렬 검증: verification_tool/test_phase3_1_static_orientation.py 를 가동합니다. 0번 포지션 방위각 설치 바이어스를 제거하고 120도 삼각형 대칭 오차를 Modulo 보상하여, 순수 센서 자세 RMSE 및 1대1로 일치된 3D 화살표 정합 상태를 시각적으로 검증합니다.
2. 시간 윈도우 스캔 분석: 분석 가동 전에 반드시 verification_tool/data_collection_100s.py를 구동하여 20개 포지션에 대한 15초 원시 시계열 데이터(collected_data_100s.npz)를 사전 수집 구축해 두어야 합니다. 이후 verification_tool/test_phase3_2_window_analysis.py 를 가동하여 보정/측정 시간 변화에 따른 통계적 앙상블 평균 RMSE의 수렴 형태를 3D Surface 화면 팝업으로 조작 분석하고 정량 텍스트 리포트(window_analysis_report.txt)를 확보합니다.

⚠️ 주의 (이중 보정 방지 가이드라인)
- 시간 윈도우 스캔 분석(test_phase3_2_window_analysis.py) 기동 시 사용되는 collected_data_100s.npz는 반드시 raw.ino 펌웨어 상태에서 수집된 미가공 raw LSB 데이터여야 합니다.
- 스크립트 내부에서 12-parameter 및 9-parameter 캘리브레이션 솔버가 직접 가동되므로, 이미 교정 완료된 calibrated.ino 스트리밍 데이터를 수집해 입력하면 이중 보정 오차가 유발됩니다.

---

## 📚 4. 학술 및 이론 지식 네트워크 (Obsidian PARA)

*   [[실물_IMU_캘리브레이션_및_자세추정_구현_계획]] ➔ 프로젝트의 전체 마일스톤 및 체크리스트
*   [[아두이노_센서제어_실무지식_및_가이드]] ➔ I2C 드라이버 구동 핵심 전처리 및 수식 가이드
*   [[5. Theory_AccelMag_기반_NED자세추정]] ➔ TRIAD 직교화 유도 및 DCM 구성 이론
*   [[4. 쿼터니온과 수학적 원리]] ➔ 짐벌 락(Gimbal Lock) 방지 및 복소 지수 쿼터니언 수학 원리
*   SVD 기반 Wahba 문제 해결 기법 ➔ `static_initialization.py` 및 `test_phase3_1_static_orientation.py` 내의 정적 절대 자세(Yaw) 복조 및 한국 복각 Sign Flip 방지용으로 구현
*   Huber Loss 및 Median-MAD 통계적 필터링 기법 ➔ `test_phase3_2_window_analysis.py` 내의 통계적 전기 서지 아웃라이어 차단용으로 구현
*   Shortest Arc 최소 회전 정합 및 Modulo 회전 대칭성 보상 기법 ➔ `test_phase3_1_static_orientation.py` 내의 20 Positions 지그 안착에 따른 기하학적 거치 편차 소거용으로 구현
*   독립 세션 다중 이터레이션 및 3D Surface 스캔 분석 기법 ➔ `test_phase3_2_window_analysis.py` 내의 보정/측정 시간 변화에 따른 오차 수렴성(Saturation) 검증용으로 구현
