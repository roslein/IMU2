# Verification Tool MOC

---

## 검증 데이터 획득기

[[data_collection_100s.py]]
: 포지션 분석 스캔의 통계적 유효성을 정합하기 위해 각 면에서 10초(1000샘플) 동안 원시 시계열을 수작업 딜레이 없이 한 번에 연속 적재해 collected_data_100s.npz 데이터셋으로 컴파일해두는 툴.

---

## 센서 및 자세 성능 검증기

[[test_phase1.py]]
: 스트리밍 패킷의 수신율, 소수점 자리 해상도 점검 및 gyro static bias 등의 초기 가동 이상 유무 검증기.

[[test_phase2.py]]
: 20면체 거치 정합 데이터의 물리 스케일이 중력가속도 1g 구면에 완벽 정합되는지 분석하고 가속도계 놈의 편차 분포를 입증하는 툴.

[[test_phase3.py]]
: 20개 포지션 데이터 전체에 대한 3D 절대 자세 추정의 정합성 오차를 구하고 각 포지션별 쿼터니언 각도 오차 경향을 정량 입증하는 기본 분석 도구.

[[test_phase3_1_static_orientation.py]]
: SVD 최소회전(Shortest Arc) ideal GT 유도, 0번 정렬 보상 및 120도 삼각형 대칭 Modulo 오차 제거를 통해 순수 알고리즘 자세 RMSE 분포(초록색 바)와 quiver visual 1대1 일치도를 3D로 시각화해주는 검증 툴.

[[test_phase3_2_window_analysis.py]]
: T_cal(보정 시간)과 T_est(측정 시간) 격자 스케일별로 10초 데이터를 겹치지 않는 독립 구간으로 분할하여 다중 이터레이션(5회) 앙상블 평균 RMSE를 유도하고, 성능 포화(Saturation) 현상을 3D Surface로 시각화 및 수치 보고서(window_analysis_report.txt)로 자동 저장해주는 종합 분석기.
