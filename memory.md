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
*(실물 센서 데이터 획득 단계 시 에러 로깅 예정)*

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
