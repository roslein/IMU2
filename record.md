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

---

## 5. 추가 수정 (v0.2.2 지자기 센서 보정 및 검증 구조 개편 계획 수립)

### 5.1 수정 이유
9-parameter 비선형 자력계 피팅의 회전 비식별성(Identifiability) 오버피팅 및 38.6도 복각 RMSE 오류 문제를 근본적으로 해결하기 위해, 다양한 보정 알고리즘(3/6/9-param)과 데이터셋 분할 활용(전체 데이터 피팅 vs 20면 평균 검증)을 대조 평가하는 로드맵 계획 문서를 작성하기 위함.

### 5.2 수정 계획 및 예상 결과
- 위치: D:\Obsidian Vault\01_PARA_동적\1. Projects-목표나 기한이 명확한 작업\1. 연구실 활동\3. 개인연구(imu)\3. 구현 및 결과(데이터,발표)\2. 보정\2. 실제 실험\지자기 센서 보정 정확도 측정 및 평가 계획 및 결과(v0.2.2).md 생성.
- 예상 결과: 보정 개편 계획 및 결과서가 버전 v0.2.2로 문서화되며, 후속 정량 분석 파일 및 캘리브레이션 툴체인 이식 가이드라인이 명확히 수립됨.

### 5.3 수정 내용
- 해당 경로에 지자기 센서 보정 정확도 측정 및 평가 계획 및 결과(v0.2.2).md 문서 생성 완료.
- 9자유도 피팅의 회전 꼬임 기하학적 문제 진단, 데이터 이원화(30,000점 전체 원시 데이터 피팅 vs 20점 평균 데이터 검증), 9가지 교차 비교군 모델 설계, Magnitude Norm 및 Dip Angle RMSE 등의 정량 평가 지표를 정리하여 명문화함.
- 6-parameter 및 9-parameter Symmetric 변환 행렬 W_mag 에 대해 Cholesky Parameterization (W = L * L^T, L 대각 성분 exp 처리) 수학적 명세를 적용하여, 솔버 내부에서 SPD(대칭 양의정부호)를 100% 강제하고 고유값의 음수/수축 방지를 보장하도록 설계를 고도화함.

### 5.4 실제 결과 및 인사이트
- 9가지 교차 검증 연산 결과, Cholesky Parameterization 기법으로 100% SPD를 강제하였음에도 불구하고 6-param 및 9-param Symmetric 보정 모두 복각 RMSE가 65.9~66.0도 수준으로 붕괴하는 동일한 현상을 목동 확인하였습니다.
- 고유값 정밀 분석 결과, 6-param Diagonal W의 고유값은 [1.075e-7, 6.979e-6, 6.367e-6] (조건수 65배), 9-param Symmetric W의 고유값은 [6.597e-8, 6.901e-6, 4.746e-6] (조건수 105배)로 산출되어 극단적인 비등방성 찌그러짐이 발견되었습니다.
- 이는 SPD 제약 부재 때문이 아니라, Norm RMSE 최소화만을 추구하는 비선형 솔버가 x축 성분의 스케일을 강제로 0에 가깝게 죽여 데이터를 Y-Z 2D 원반(Disk) 형태로 평면 수축(Collapse)시켰기 때문인 것으로 실증되었습니다.
- 결론적으로 W_mag의 비등방성 찌그러짐 과적합을 차단하고 3D 기하 방향을 완벽히 복원하기 위해 W_mag = Identity/avg_radius 로 고정하는 3-parameter (Offset Only) 모델이 유일하고도 가장 강력한 물리적 해법임을 수학적으로 확정하여 v0.2.2 결과 보고서에 이식 완료하였습니다.
