# 👤 User Profile & Style — Real-world IMU Project (ISM330DHCX & MMC5983MA)

*   *목표*:
    1. 실물 센서 데이터 기반 20면체 정밀 캘리브레이션 툴 빌드
    2. 정적 무회전 상태의 실물 센서 Orientation Tracking 알고리즘 구현
*   *하드웨어 명세 (Hardware Specs)*:
    - 가속도계/자이로: ISM330DHCX (6DoF IMU)
    - 자력계: MMC5983MA (3축 고감도 자력계, 18-bit)
    - MCU 보드: SparkFun ESP32 Thing Plus C (FQBN: esp32:esp32:esp32thing_plus_c)
*   *설계 가이드라인 (Design Guidelines)*:
    - *HAL 좌표계 일치화*: 가속도계와 자력계의 마운트 방향 차이로 인해, *자력계 Z축 데이터는 수신 즉시 -1을 곱해 축 방향을 가속도계와 물리적으로 반드시 정렬*할 것.
    - *하드웨어 레벨 오차 상쇄*:
        - 자이로/가속도 오프셋은 소프트웨어가 아닌 센서 내부 레지스터(`X_OFS_USR` 등)에 직접 쓰기 처리.
        - 자력계는 내부 제어 레지스터(09h) 설정에서 `Auto_SR_en`(자동 Set/Reset) 및 `En_prd_set`을 켜서 하드웨어 단에서 오프셋/온도 드리프트를 자동 제거할 것.
    - *하드웨어 필터링*: ISM330DHCX `CTRL6_C` 레지스터의 LPF 차단 주파수(FTYPE)를 하드웨어적으로 12.5Hz 등으로 세팅하여 안티 에일리어싱 확보.
    - *고속 전송 프로토콜*: 아두이노/ESP32에서 PC로 텍스트(ASCII) 대신 *바이너리 패킷([시작]+[데이터]+[체크섬]+[종료])* 형태로 스트리밍하여 직렬 전송 병목 차단. PC 파이썬에서는 비동기 스레드 및 링 버퍼(Ring Buffer)로 수신.
    - *실물 코딩 및 자세 융합 컨벤션*:
        - 1. 정적 오리엔테이션 윈도우: TRIAD-NED 기반 정적 절대 3D 자세각 추정 시 가우시안 진동 잡음을 완전 상쇄하기 위해 반드시 최적 수집 시간 5.0초(Fs = 100Hz 기준 500샘플) Box LPF 누적 평균화 기법을 적용할 것. (5초 이후 오차 개선 Saturation 정량 실증 반영)
        - 2. 자이로 바이어스 정적 제약: 정지 무회전 상태(Static Constraint)를 활용한 3축 자이로 5.0초 LPF 평균값은 차기 동적 EKF 자세 추정기 기동 시 실시간 자이로 바이어스(Gyro Bias) 초기화(Static Initialization)의 강력한 상호 대조 환류 레퍼런스로 활용 가능함.
        - 3. 자이로 Global Average 바이어스 보정: 20면체 정지 데이터 20x3 전수 수집 시 자이로스코프 데이터셋도 gathered_data.npz 에 함께 누적 적립 저장한 후, 20개 면 정적 gyro 데이터의 전체 평균(Global Average 참 평균)을 계산해 초정밀 gyro_bias 오프셋 상수를 역산할 것. (gyro_bias_calibration.py 구동)
        - 4. CP949 이모지 인코딩 충돌 방지: 모든 Python 수집 유틸 및 분석 스크립트 최상단에 sys.stdout.reconfigure(encoding='utf-8') 구문을 상시 기재할 것.
        - 5. COM 포트 Permission Error 13: 외부 시리얼 모니터 점유 충돌 시의 센서 물리 리셋 및 강제 종료 예외 탈출 분기를 data_collection 및 static_initialization 에 상설 기재할 것.
        - 6. 아두이노 CLI 빌드 자동화: 로컬 빌드 툴체인(D:/arduino-cli/arduino-cli.exe) 및 FQBN(esp32:esp32:esp32thing_plus_c)을 사용하여 COM3 포트에 펌웨어를 직접 컴파일/업로드할 수 있도록 환경을 연동함.
