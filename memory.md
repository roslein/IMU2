# 🧠 Zettelkasten Memory — Real-world IMU Project (ISM330DHCX & MMC5983MA)

## 🔴 [CHRONIC] 만성적 치명적 실수
*   **ID**: CHRONIC-IMU-01
*   **유형**: [CHRONIC]
*   **내용**: MMC5983MA 자력계 Z축 데이터의 부호 반전(-1 곱하기) 누락으로 인해, 가속도계 좌표계와 자력계 좌표계 불일치 발생 ➔ 방위각(Yaw) 추정 각도가 파멸적으로 틀어지는 참사 발생 위험. 데이터 획득 최하단(HAL)에서 무조건 반전 처리할 것.
*   **ID**: CHRONIC-IMU-02
*   **유형**: [CHRONIC]
*   **내용**: 실물 센서 I2C 전원에 5V를 직접 인가 시 3.3V 전용 MEMS 칩 파손 위험. 무조건 3.3V LDO 출력 전원 라인 활용할 것.

---

## 📌 [MISTAKE] 디버깅 및 에러 기록

*   **ID**: MISTAKE-IMU-02
*   **유형**: [MISTAKE]
*   **내용**: CAD ideal 법선 기반 NN 매칭 가동 시 중앙 센서 30도 비틀림 마운팅 오차에 따른 기하학적 데드존 병목 발생. SVD Kabsch 알고리즘을 가동하여 마운팅 오일러각(Yaw=-160.64, Pitch=-29.79, Roll=-20.40)을 역산하고 get_rotated_normals() 로 ideal 법선들을 사전 회전 정합시켜 오차 꼬임을 완벽 분리 상쇄해야 함.

*   **ID**: MISTAKE-IMU-03
*   **유형**: [MISTAKE]
*   **내용**: 16번 케이블 인출면 안착 시 점퍼선 장력 요동으로 I2C 통신 버스 하드웨어 락업 및 0.0 고착 예외로 Matplotlib 강제 튕김 발생. 0.0 고착 체크 전단 탑재 및 calibration_tool/output/checkpoint_data.npz 에 실시간 이어받기(Resume) 오토세이브 임시 체크포인트를 상설 탑재하여 유실을 원천 방지해야 함.

*   **ID**: CONCEPT-IMU-02
*   **유형**: [깨달음]
*   **내용**: I2C 복구용 USB 물리 리셋 탈부착 시 케이블 곡률 변화에 따른 유도 전류 자기장 시프팅 및 차단/인가 과도기 전류 서지에 의한 금속 강자성 브리지 내 잔류 자화 격차 유입으로 자력계 16번 데이터만 독자 아웃라이어(Outlier) Shifting 튕김 발생 주의.

*   **ID**: CONCEPT-IMU-03
*   **유형**: [깨달음]
*   **내용**: 실물 실험의 케이블 들뜸 및 전원 자성 요동 재발 방지를 위해, 19개 골드 데이터만 선택 사용(대안 1)하거나 1차 가평가 피팅 후 3배 표준편차 특이점을 솔버가 스스로 마스킹 소거하는 Robust Outlier Rejection 메커니즘 내장(대안 2)을 교수님과 연계 검토할 것.

*   **ID**: CHRONIC-IMU-03
*   **유형**: [CHRONIC]
*   **내용**: 재보정 안전 아키텍처(raw.ino 와 calibrated.ino) 물리 이원화 후, 자동 제너레이션되는 calib_params.h 경로를 반드시 EKF 프로젝트 디렉토리인 firmware/calibrated/calibration 에 덮어쓰도록 툴체인 정합성을 상시 유지해야 함. (main/calibration 으로 오이송 누락 에러 주의)

---

## 🛠 [PATTERN] 재사용 코드 및 컨벤션
*   **ID**: PATTERN-IMU-01
*   **유형**: [PATTERN]
*   **내용**: 고속 시리얼 Binary 데이터 프레이밍 컨벤션:
    `[START_BYTE (0xAA)] + [Payload (Bytes)] + [Checksum (XOR)] + [END_BYTE (0x55)]`
    PC 수신 단은 반드시 비동기 스레드 + Ring Buffer 구조를 준수하여 패킷 유실을 차단함.

---

## 🎓 [CORE_CONCEPT] 학술적 깨달음 및 지식
*   **ID**: CONCEPT-IMU-01
*   **유형**: [깨달음]
*   **내용**: 자력계 브리지 오프셋(온도 드리프트 및 고착 왜곡)은 센서 내부의 SET/RESET 고압 전류 펄스(순방향/역방향 자화) 및 `(A - B) / 2` 차분 연산을 통해 완벽히 극복 가능함. 하드웨어 `Auto_SR_en` 설정을 통해 무부하 오프셋 제거 메커니즘 획득.
