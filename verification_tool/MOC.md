# Verification Tool MOC (검증 툴 파이프라인 맵)

---

## 1. 데이터 획득기 (Data Collectors)

### [[data_collection_100s.py]] (평가/검증용 연속 데이터 수집기)
- 용도: 20면체 각 안착 포즈에서 노이즈 및 장기 거동을 평가하기 위한 연속 시계열 데이터 획득
- 입력(Input): 100Hz 시리얼 바이너리 스트림 (IMU raw 센서 패킷)
- 출력(Output): collected_data_100s.npz (20개 포즈별 10초, 총 1,000샘플씩의 연속 시계열 원시 데이터셋)
- 의존성 흐름: 이 출력 파일은 verification_tool 폴더 내의 모든 기존 성능 검증기(test_phase2.py, test_phase3.py 등)의 유일한 시계열 입력 소스로 공급됨.
- 참고 (외부 참조): [[../calibration_tool/data_collection.py]] (보정용 정적 데이터 수집기)
  - 역할 차이: calibration_tool 하위 수집기는 정지 포즈당 3초 평균값을 취해 '보정 솔버'의 피팅 입력 값으로 공급하며, 본 검증 폴더 하위 수집기는 10초 연속 시계열을 통째로 받아 '보정 후 자세 정확도 및 노이즈 검증'에 활용함.

### [[data_collection_task_aware.py]] (NEW - v0.3.0 예정)
- 용도: 수평 회전판 데카르트 곱 기반의 고밀도 자기장 및 Yaw 각도 데이터 획득
- 입력(Input): 100Hz 시리얼 바이너리 스트림 + 수동 눈금 조작 트리거 (Yaw GT 각도)
- 출력(Output): task_aware_raw_data.npz (대표 8면 x 12눈금 = 총 96포인트 데카르트 곱 원시 데이터셋)
- 의존성 흐름: v0.3.0의 신규 Task-Aware 자력계 보정 솔버(test_phase2_4_task_aware_cal.py)의 전용 입력 소스로 공급됨.

---

## 2. 센서 및 자세 성능 검증기 (Evaluators)

### [[test_phase1.py]] (기본 스트리밍 및 자이로 정적 바이어스 검증기)
- 입력(Input): 시리얼 실시간 데이터 스트리밍
- 출력(Output): 터미널 콘솔 로그 (스트림 수신율 및 정지 상태 자이로 바이어스 진단 결과)

### [[test_phase2.py]] (가속도계 보정 정합성 분석기)
- 입력(Input): collected_data_100s.npz + 기존 가속도 보정 파라미터(acc_params.npz)
- 출력(Output): 가속도 보정 데이터의 중력 Norm(1.0g) 구면 수렴도 및 놈 표준편차 통계 콘솔 보고

### [[test_phase2_2_mag_cal.py]] (자력계 기하 보정 다중 모델 비교기)
- 입력(Input): collected_data_100s.npz + 기존 가속도 보정 파라미터(acc_params.npz)
- 출력(Output): 3p, 6p, 9p-Sym 등 기하학적 피팅 모델 후보군별 Sphere Norm RMSE 및 Dip Angle RMSE 비교 결과 리포트

### [[test_phase2_3_mag_cal.py]] (자이로 1D 적분 및 자력계 Yaw 통합 실증기)
- 입력(Input): collected_data_100s.npz + 기존 가속도 보정 파라미터(acc_params.npz) + 0도 안착 자이로 실시간 영점 바이어스
- 출력(Output): 자이로 적분 궤적과 자력계 Yaw 각도기 간의 Yaw RMSE 리포트 및 대조 시계열 그래프 이미지 (test_phase2_result.png)

### [[test_phase2_4_task_aware_cal.py]] (NEW - v0.3.0 예정)
- 입력(Input): task_aware_raw_data.npz + 기존 가속도 보정 파라미터(acc_params.npz)
- 출력(Output): 최종 최적 보정 파라미터(mag_params.npz), 5대 평가 지표 및 반복성 분산 분석 결과 콘솔 리포트, calib_params.h 자동 갱신 트리거 호출
- 의존성 흐름: 생성된 mag_params.npz는 자세 융합 평가 및 calibrated.ino 헤더 파일 배포의 입력 소스로 사용됨.

### [[test_phase3.py]] (3D 자세 추정 통합 오차 분석기)
- 입력(Input): collected_data_100s.npz + acc_params.npz + mag_params.npz
- 출력(Output): 20개 정적 포즈 전체에 대한 SVD(Wahba) 자세 쿼터니언 각도 오차 통계 보고서

### [[test_phase3_1_static_orientation.py]] (3D 정적 자세 시각화 검증기)
- 입력(Input): collected_data_100s.npz + acc_params.npz + mag_params.npz
- 출력(Output): SVD ideal GT 기준 120도 대칭 보상 알고리즘 3D quiver 매칭 창 및 알고리즘 자세 RMSE 비교 플롯 그래프

### [[test_phase3_2_window_analysis.py]] (측정 윈도우 스케일 종합 분석기)
- 입력(Input): collected_data_100s.npz + acc_params.npz + mag_params.npz
- 출력(Output): 3D Surface 격자 시각화 창 및 오차 포화 분석 수치 보고서 (window_analysis_report.txt)
