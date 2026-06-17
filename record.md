# IMU Project Modification Record

## 1. 수정 이유
Yaw 축 회전의 무작위 오차에 종속적인 기존 자력계 평가 지표 대신, 회전에 무관하고 강건한 물리량인 지자기 크기(Norm) 오차 및 중력선-지자기 사잇각(복각, Dip Angle) 오차를 기반으로 윈도우 최적화 분석을 재설계하기 위함.

## 2. 수정 계획 및 예상 결과
- 분석 스크립트인 test_phase3_2_window_analysis.py 내 연산 식을 지자기 Norm-1.0 및 복각 RMSE 기준으로 교체.
- 최대 윈도우 분석 시간을 10.0초로 상한 설정.
- 6회 슬라이딩 윈도우 앙상블 평균을 적용하여 윈도우별 통계적 편향 제거.
- 예상 결과: 3D Surface Plot 결과 이미지와 정량 텍스트 보고서(window_analysis_report.txt)가 정상 생성되어 윈도우 크기에 따른 오차 수렴성을 증명함.

## 3. 수정 내용
- verification_tool/test_phase3_2_window_analysis.py 수정:
  - 10.0초 스캔 상한선 적용.
  - 6회 이터레이션 슬라이딩 윈도우 앙상블 평균 연산 루프 구현.
  - 지자기 Norm 오차(Norm - 1.0) 및 복각 오차(Dip - dip_ref_mean) 기반 RMSE 계산식 도입.
  - 3D Surface Plot 제목 및 축 라벨을 Accel / Mag Norm / Dip Angle 로 갱신.

## 4. 실제 결과
- window_analysis_report.txt 결과 파일 정상 생성 확인.
- 복각 레퍼런스 평균(dip_ref_mean)이 -2.8054 deg로 산출되었으며, 윈도우 스캔에 따른 RMSE 변화 추이가 안정적으로 계산됨.
- test_phase3_2_window_analysis_result.png 및 test_phase3_2_window_analysis_result_spheres.png 정상 생성 완료.
