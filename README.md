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

### 1) 하드웨어 결선
*   **동작 전압**: **`3.3V`** 엄수 (5V 인가 금지)
*   **통신**: I2C 결선
    *   SCL ➔ 아두이노 `A5` (I2C Clock)
    *   SDA ➔ 아두이노 `A4` (I2C Data)

### 2) 펌웨어 업로드
1.  아두이노 IDE에서 `firmware/main/main.ino`를 로드합니다.
2.  라이브러리 매니저에서 `ISM330DHCX`, `MMC5983MA` 공식 라이브러리를 설치합니다.
3.  보드 및 COM 포트를 마운트하고 **Upload**를 실행합니다.

---

## 📚 4. 학술 및 이론 지식 네트워크 (Obsidian PARA)
*   [[실물_IMU_캘리브레이션_및_자세추정_구현_계획]] ➔ 프로젝트의 전체 마일스톤 및 체크리스트
*   [[아두이노_센서제어_실무지식_및_가이드]] ➔ I2C 드라이버 구동 핵심 전처리 및 수식 가이드
*   [[5. Theory_AccelMag_기반_NED자세추정]] ➔ TRIAD 직교화 유도 및 DCM 구성 이론
*   [[4. 쿼터니온과 수학적 원리]] ➔ 짐벌 락(Gimbal Lock) 방지 및 복소 지수 쿼터니언 수학 원리
