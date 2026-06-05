# 🎯 Real-world IMU & Orientation Tracking Project

본 프로젝트는 정20면체(정이십면체) 지그를 활용한 실물 고성능 MEMS 관성/자기 센서(**`ISM330DHCX`** & **`MMC5983MA`**)의 정밀 캘리브레이션 및 쿼터니언 기반 정적 무회전 3D 자세 추정(Orientation Tracking) 통합 시스템입니다.

---

## 🏛️ 1. 핵심 아키텍처 및 설계 특징

*   **3계층 격리 설계 (HAL - Driver - Application)**:
    *   **HAL 계층**: I2C 물리 통신 제어 및 **자력계 Z축 부호 반전(-1 곱하기) 보상**
    *   **Driver 계층**: 센서 내부 레지스터 직접 제어 (ISM330DHCX 하드웨어 LPF, MMC5983MA 자동 Set/Reset 노이즈 제거 활성화)
    *   **Application 계층**: 보정 계수 수식 연산 및 쿼터니언 기반 자세 융합
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
│   └── /main/           # 아두이노 IDE 매치 룰 메인 폴더
│       ├── /driver/     # ISM330DHCX, MMC5983MA 개별 드라이버 및 HAL
│       └── /calibration/# 보정 파라미터 적용 모듈
│
├── /calibration_tool/   # PC (Python) 오프라인 20면체 보정 툴
│   ├── main.py          # 사용자 트리거 수집 및 최적화 실행 엔트리
│   └── icosahedron.py   # 정20면체 법선벡터 LUT (황금비 기반)
│
└── /verification_tool/  # PC (Python) 실시간 3D 자세 시각화 검증 툴
    ├── complementary.py # 가속도+자력 기반 정적 LPF 융합 알고리즘
    └── render_3d.py     # 실시간 3D 자세 렌더링 엔진
```

---

## 🛠️ 3. 시작하기 (Quick Start)

### 1) 1단계: 원시 데이터 수집 및 캘리브레이션 (Calibration)
1. 펌웨어 업로드: firmware/raw/raw.ino 를 보드에 빌드 업로드하여 순수 LSB 무가공 원시 데이터 송출 환경을 구축합니다. (전압 3.3V 결선 및 COM 포트 점유 충돌 주의)
2. 실측 데이터 수집: calibration_tool/data_collection.py 를 실행하여 20면체 지그를 바닥에 안착시키며 엔터 트리거를 통해 3초 평균 raw 데이터 20개 포지션을 전수 수집 및 calibration_tool/output/collected_data.npz 에 저장합니다.
3. 가속도계 12-parameter 교정: calibration_tool/accel_calibration.py 를 기동하여 지면 경사각을 Recursive하게 보상 수렴한 최적 acc_params.npz 파라미터를 역산 저장합니다.
4. 자력계 Hard iron and soft iron 교정 및 헤더 자동 빌드: calibration_tool/mag_calibration.py 를 기동하여 타원체 피팅 솔버를 Levenberg-Marquardt 연산한 뒤, 최종 C++ calib_params.h 보정 상수를 빌드하여 EKF 트래킹 펌웨어 전용 경로인 firmware/calibrated/calibration/calib_params.h 로 실시간 덮어쓰기 자동 이식을 수행합니다.

### 2) 2단계: 보정 펌웨어 탑재 및 3D 정적 자세 추정 (Orientation Tracking)
1. EKF 보정 펌웨어 업로드: firmware/calibrated/calibrated.ino 를 보드에 업로드합니다. calib_params.h 상수가 매 순간(100Hz) 곱해져서 뿜어 나오는 최종 정합 보정 바이너리 스트리밍 텔레메트리가 실시간 기동됩니다.
2. 정적 절대 3D 자세 역산 기동: verification_tool/static_initialization.py 를 실행합니다. 센서를 평평한 바닥에 정지 거치한 후 시작 트리거(Enter)를 치면 정적 LPF 가이드라인 5.0초(500샘플) 동안 Box LPF 평균을 계산하여 노이즈를 극소화한 TRIAD-NED 기반 오일러각(Roll, Pitch, Yaw) 및 쿼터니언을 복조 보고하고 static_orientation.npz 에 저장합니다.

### 3) 3단계: 정적 회전 자세 쿼터니언 정량 오차 독립 검증 (Validation)
1. 검증 유틸 기동: calibrated.ino 펌웨어가 작동하는 실물 보드를 둔 상태에서 verification_tool/test_phase3.py 를 기동합니다.
2. 보정 데이터 신규 수집: 20 Positions 가이드에 따라 20개 면을 새로 안착시키고 엔터 트리거로 5초 평균 보정 데이터를 실시간으로 중복 없이 새로 획득하여 new_calib_collected.npz 에 저장합니다.
3. 쿼터니언 오차 검증: 사전 수학적으로 정의해 둔 20 Positions ideal 거치 기하 Alignment 회전 행렬 기반 이론적 GT 쿼터니언 q_gt[i] 와 실시간 수집 역산된 q_est[i] 간의 회전 각도 오차(Quaternion Angle Error)를 1대1 대조하여 최종 RMSE 수렴 오차를 CLI 에 인쇄 보고하고, 포지션별 오차 Bar 플롯인 verification_tool/test_phase3_result.png 그래프를 자동 저장합니다.

---

## 📚 4. 학술 및 이론 지식 네트워크 (Obsidian PARA)
*   [[실물_IMU_캘리브레이션_및_자세추정_구현_계획]] ➔ 프로젝트의 전체 마일스톤 및 체크리스트
*   [[아두이노_센서제어_실무지식_및_가이드]] ➔ I2C 드라이버 구동 핵심 전처리 및 수식 가이드
*   [[5. Theory_AccelMag_기반_NED자세추정]] ➔ TRIAD 직교화 유도 및 DCM 구성 이론
*   [[4. 쿼터니온과 수학적 원리]] ➔ 짐벌 락(Gimbal Lock) 방지 및 복소 지수 쿼터니언 수학 원리
