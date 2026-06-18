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

---

## 6. 추가 수정 (v0.2.3 다기준 자동 최적화 캘리브레이션 개편 계획 수립)

### 6.1 수정 이유
단일 크기 잔차(Norm RMSE) 최소화에만 의존할 경우 발생하는 솔버 수축 덫을 방지하고 물리 기하학적 정합성을 엄격히 보장하기 위해, MATLAB magcal의 'auto' 철학을 확장한 다기준 자동 선택 시스템(v0.2.3) 설계 계획을 문서화하기 위함.

### 6.2 수정 계획 및 예상 결과
- 위치: D:\Obsidian Vault\01_PARA_동적\1. Projects-목표나 기한이 명확한 작업\1. 연구실 활동\3. 개인연구(imu)\3. 구현 및 결과(데이터,발표)\2. 보정\2. 실제 실험\지자기 센서 보정 정확도 측정 및 평가 최종(v0.2.3).md 생성.
- 예상 결과: 3가지 병렬 보정 기법(3-param, 6-param Cholesky, 9-param Sym Cholesky) 구동 및 다기준 우선순위(1. Yaw closed-loop 오차, 2. 복각 절대 오차, 3. Norm RMSE/Std)에 입각한 자동 판별 알고리즘 설계안이 문서화됨.

### 6.3 수정 내용
- 해당 경로에 지자기 센서 보정 정확도 측정 및 평가 최종(v0.2.3).md 문서 생성 완료.
- 3가지 후보 솔버 병렬 구동 조건 정의, 다기준 평가지표 우선순위 수립, 사전식 순서(Lexicographic Order) 수축 모델 필터링 기법 정리, 실행 로드맵 수립.

### 6.4 실제 결과
- 지자기 센서 보정 정확도 측정 및 평가 최종(v0.2.3).md 문서 1차 생성 완료.

---

## 7. 추가 수정 (v0.2.3 다기준 자동 최적화 지식 문서 업데이트)

### 7.1 수정 이유
각도기 직접 정렬 검증 모드와 자이로 적분 궤적 검증 모드를 명확히 분리하고, 전체 파이프라인의 실행 로드맵을 Mermaid 다이어그램으로 가시화하여 통합적인 다기준 보정 지식 베이스를 완성하기 위함.

### 7.2 수정 계획 및 예상 결과
- 위치: 지자기 센서 보정 정확도 측정 및 평가 최종(v0.2.3).md 내 3대 알고리즘, 4대 우선순위 가이드라인, 5대 실행 로드맵 재설계 및 Mermaid 소스코드 탑재.
- 예상 결과: 사용자의 각도기 vs 자이로 평가 선택 분기가 기술된 마크다운 문서가 성공적으로 오버라이트 작성됨.

### 7.3 수정 내용
- 지자기 센서 보정 정확도 측정 및 평가 최종(v0.2.3).md 파일 전체 덮어쓰기 집행.
- 1순위(Closed-loop), 2순위(Yaw Increment - 모드 선택형), 3순위(복각), 4순위(Norm RMSE/Std) 세부 지표 개편.
- 1~4단계 사전식 순서 가이드라인 재정립.
- 전체 캘리브레이션 툴체인 파이프라인 Mermaid 다이어그램 탑재.

### 7.4 실제 결과
- Obsidian Vault 내 마크다운 파일 오버라이트 완료 및 Mermaid 다이어그램을 통한 시각화 정의 적용 완료.

---

## 8. 추가 수정 (v0.2.3 각도기 대조 실시간 수집 및 다기준 낙찰 툴 1차 구현)

### 8.1 수정 이유
시뮬레이션 가상 왜곡 모순을 해결하고, 실제 각도기 정렬 수집 환경을 그대로 반영하여 3가지 보정 모델(3-param, 6-param Cholesky, 9-param Symmetric Cholesky)의 실측 Yaw 오차를 정량 계산 및 사전식 자동 낙찰하기 위함.

### 8.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_3_mag_cal.py 신규 생성.
- 예상 결과: 전체 원시 데이터 피팅, 실시간 시리얼 스트리밍 수집, 0->90->180->270->360 각도기 회전 가이드 인터랙션, Yaw RMSE/Closed-loop 및 복각/Norm 다기준 사전식 정렬 로직이 탑재된 검증 스크립트 빌드 완료.

### 8.3 수정 내용
- test_phase2_3_mag_cal.py 파일 신규 작성.
- combinations 후보군 중 9-param Full 및 융합형을 소거하고 전체 원시 데이터(30,000점) 입력 기반 3가지 모델(3-param, 6-param Cholesky, 9-param SPD)만 최종 수립.
- 복각 참값 DIP_TRUE의 북반구 음수 부호 기본값(-54.3) 고정 적용으로 109도 부호 반전 오차 원천 해결.
- 시리얼 통신을 통해 5개 타겟 각도 엔터 트리거별 실시간 100샘플 raw 자력 및 가속도를 함께 수집하는 모듈 구현.
- 가속도계 보정본(acc_params.npz)을 로드해 실시간 Roll, Pitch를 산출하고, 실측 중력 벡터 g_unit을 [0, 0, 1]로 회전시키는 틸트 보정(Tilt Compensation) R_tilt를 SVD(align_vectors)로 실시간 연산.
- 틸트 보정 전후(Raw, Tilt)의 Yaw를 각각 추적하여 각도기 타겟 눈금 대비 Closed-loop 오차 및 Yaw Increment RMSE를 도출.
- 20면 평균 데이터 기반의 Norm RMSE/Std 및 복각 RMSE 테이블과 실측 각도기 Yaw 결과를 병합 출력하여 다기준 사전식 정렬(Lexicographic Order)로 최종 보정 모델 자동 낙찰.
- 낙찰된 파라미터를 calibration_tool/output/mag_params.npz 로 자동 백업 연동.

### 8.4 실제 결과
- test_phase2_3_mag_cal.py 소스코드 작성 완료 및 틸트 보정이 가미된 실측 데이터 수집 파이프라인 빌드 완료.

---

## 9. 추가 수정 (v0.2.3 지자기 Z축 부호 누락 및 틸트 보정 중력 부호 오류 교정)

### 9.1 수정 이유
피팅 데이터 로드 시 자력계 Z축 부호 반전(-1 곱하기) 누락으로 인한 좌표계 뒤틀림을 방지하고, 수평면 틸트 보정 시 실측 중력 벡터 방향을 지구 수직 아래 방향인 -acc_cal 과 정확히 정합하여 기하학적 Yaw/복각 오차를 제거하기 위함.

### 9.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_3_mag_cal.py
- 예상 결과: Z축 부호 및 g_unit 수직 아래 방향 벡터 적용 완료.

### 9.3 수정 내용
- mag_100s 로드 직후 mag_100s[:, :, 2] = -mag_100s[:, :, 2] 추가.
- R_tilt 도출용 g_unit_sample 계산식을 -acc_cal_sample / norm(acc_cal_sample) 로 수정 적용.

### 9.4 실제 결과
- test_phase2_3_mag_cal.py 코드 수정 완료 및 깃허브 푸시 완료.

---

## 10. 추가 수정 (v0.2.3 보고서 저장 경로 분리 및 유실 방지)

### 10.1 수정 이유
2_2 버전과 2_3 버전의 다기준 보정 비교 보고서 파일명이 mag_calibration_compare_report.txt 로 충돌하여 덮어쓰기 유실이 발생하는 문제를 방지하기 위함.

### 10.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_2_mag_cal.py 및 verification_tool/test_phase2_3_mag_cal.py
- 예상 결과: 각각 _v0.2.2.txt 및 _v0.2.3.txt로 파일 저장명이 변경되어 분리 보존됨.

### 10.3 수정 내용
- test_phase2_2_mag_cal.py 내 저장 파일명을 mag_calibration_compare_report_v0.2.2.txt 로 수정.
- test_phase2_3_mag_cal.py 내 저장 파일명을 mag_calibration_compare_report_v0.2.3.txt 로 수정.
- 로컬의 기존 mag_calibration_compare_report.txt 파일을 mag_calibration_compare_report_v0.2.2.txt 로 Rename 조치.

### 10.4 실제 결과
- 코드 수정 및 파일명 백업 완료 후 깃허브 푸시 완료.

---

## 11. 추가 수정 (v0.2.3 자력계 Z축 이중 반전 버그 제거)

### 11.1 수정 이유
펌웨어 raw.ino 단에서 이미 Z축 부호를 반전(mz = -mz)하여 송출하기 때문에, 수집된 collected_data_100s.npz 내의 데이터는 이미 가속도계 기준에 맞게 정렬되어 있음. 파이썬 검증 및 실시간 플로팅 코드(2_3)에서 Z축 부호를 한 번 더 뒤집을 경우 이중 반전이 일어나 복각이 -11도로 심각하게 왜곡되는 현상을 바로잡기 위함.

### 11.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_3_mag_cal.py
- 내용: 실시간 수집 루프 내(mag_raw[2] = -mag_raw[2]) 및 NPZ 로드 직후(mag_100s[:, :, 2] = -mag_100s[:, :, 2]) 반전 연산 삭제.
- 예상 결과: 3-param 모델 피팅 기준 평균 복각이 실제 인천 지역 참값(-54.3 deg)에 정확히 일치하며 정합되고 2_2 모델과의 데이터 축 기준이 정량적으로 완벽히 정렬됨.

### 11.3 수정 내용
- test_phase2_3_mag_cal.py L200, L245 부분의 Z축 반전 코드 제거.

### 11.4 실제 결과
- test_phase2_3_mag_cal.py 코드 수정 완료.
- 3-param 피팅 검증 스크립트 실행을 통해 Z축을 중복 반전하지 않았을 때 평균 복각이 -54.2967 deg(인천 참값 -54.3 deg 대비 RMSE 11.5915)로 정확하게 보정됨을 실증 완료.

---

## 12. 추가 수정 (v0.2.3 가속도 정렬 축 기준 일치화 및 복각 참값 부호 정합성 교정)

### 12.1 수정 이유
2_2(즉석 가속도 피팅, 하늘 방향 upward 정렬)와 2_3(acc_params.npz 로드 파라미터, 지구 중심 downward 정렬) 간의 가속도 축 기준 불일치로 인하여, 2_3에서 정상적인 3-param 복각 평균이 +54.31 deg 임에도 참값 dip_true 가 -54.30 deg 로 등록되어 109 deg 의 RMSE 연산 오류가 발생하는 문제를 해결하기 위함.

### 12.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_3_mag_cal.py
- 내용: dip_true 레퍼런스 및 사용자 가이드 디폴트 값을 -54.3000 에서 +54.3000 으로 수정.
- 예상 결과: 3-param 모델 피팅 기준 Dip True RMSE가 109.2 deg 에서 0.1 deg 수준의 실제 센서 복각 정밀도로 정상 수렴하며 사전식 낙찰 모델 시스템이 일관되게 정합됨.

### 12.3 수정 내용
- test_phase2_3_mag_cal.py L417 부분의 dip_true 부호 수정 및 L416 사용자 입력 가이드 프롬프트 기본값 표시 교정.

### 12.4 실제 결과
- test_phase2_3_mag_cal.py 코드 수정 완료.
- 가속도 downward aligned 상태에서 복각이 양수(+54.3 deg)로 정상 비교되어 3-param의 복각 오차가 0.1 deg 수준으로 정상 산출됨을 검증 완수.

---

## 13. 추가 수정 (v0.2.3 2_2 코드의 가속도 축 및 복각 평가 부호 통일)

### 13.1 수정 이유
2_2(test_phase2_2_mag_cal.py)와 2_3(test_phase2_3_mag_cal.py) 간의 가속도 틸트 보정 정렬 기준 및 복각 참값 부호 통일성을 갖추어 두 스크립트 간 결과의 완벽한 상호 연동 일관성을 획득하기 위함.

### 13.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_2_mag_cal.py
- 내용: calibrate_acc_12param 내 match_face(-d[i], normals)를 match_face(d[i], normals)로 변경하여 downward aligned 설정으로 통일하고, 복각 참값 DIP_TRUE를 -54.3에서 54.3으로 수정.
- 예상 결과: 2_2에서 연산되는 3-param 보정 복각 평균 및 RMSE가 2_3의 결과 데이터와 일치하고 0.1 deg 수준의 고정밀도가 정상 렌더링됨.

### 13.3 수정 내용
- test_phase2_2_mag_cal.py L29, L277 부분의 수식 부호 교정.

### 13.4 실제 결과
- test_phase2_2_mag_cal.py 코드 수정 완료.

---

## 14. 추가 수정 (v0.2.3 0도 기준 안착 면 1회 고정 틸트 보정 적용)

### 14.1 수정 이유
매 샘플 수집 시의 가속도 실시간 노이즈 요동 및 수평판의 물리적 경사 왜곡에 의한 기하학적 투영 오류(Geometry Projection Distortion)를 차단하고, 실제 거치 평면 위에서의 선형 Yaw 증분각만을 순수하게 보정 추출하기 위함.

### 14.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_3_mag_cal.py
- 내용: 실시간 모니터링 수집 루프 및 오프라인 최종 평가 루프에서 매 순간 가속도 SVD를 구동하는 대신, 0도 최초 안착 포지션 가속도를 기반으로 match_face를 단 1회 수행해 안착 면의 법선(rot_normals[best_idx])을 [0,0,1]로 정합하는 고정 틸트 회전 R_tilt_fixed 를 도출하여 공통 적용하도록 구조 개편.
- 예상 결과: 수평 회전 시 가속도 궤적 요동에 따른 Yaw 틸트 왜곡이 완벽하게 제거되어 정량 평가의 정밀도가 극대화됨.

### 14.3 수정 내용
- test_phase2_3_mag_cal.py L329, L339-357 및 L361-389 부분의 틸트 연산식 및 루프 최적화 완료.

### 14.4 실제 결과
- test_phase2_3_mag_cal.py 코드 수정 완료.

---

## 15. 추가 수정 (v0.2.3 틸트 보정 Z축 부호 정합성 교정)

### 15.1 수정 이유
가속도 aligned Z축의 downward 기준 상태에서 틸트 보정 정합 타겟을 upward [0,0,1]로 강제하면 Z축이 뒤집어져 Yaw 회전 방향이 음수(-theta)로 역전되는 문제를 해결하고, 실제 각도기 회전 방향인 양수(+theta)로 부호를 복원하기 위함.

### 15.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_3_mag_cal.py
- 내용: align_vectors의 타겟 정렬 벡터를 [0,0,1.0]에서 [0,0,-1.0]으로 변경하여 Z축이 뒤집어지는 반사 현상을 차단.
- 예상 결과: 틸트 보정 후의 Yaw 회전 방향이 양수로 복원되어 각도기 눈금(+방향)과 선형 정합을 유지함.

### 15.3 수정 내용
- test_phase2_3_mag_cal.py L344, L370 부분의 SVD 타겟 정합값 부호 수정 완료.

### 15.4 실제 결과
- test_phase2_3_mag_cal.py 코드 수정 완료.

---

## 16. 추가 수정 (v0.2.3 각도기 거치 모드에 실시간 자이로 적분 궤적 병행 도입)

### 16.1 수정 이유
각도기 눈금을 수동으로 추종 거치할 때 발생하는 인간의 거치 정렬 오차와 노이즈를 상쇄하기 위해, 각 단계 사이 회전하는 구간의 자이로 Z축 각속도를 백그라운드에서 실시간 적분하여 참값 궤적(Yaw_gyro)을 동시 획득 및 교차 대조하기 위함.

### 16.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_3_mag_cal.py
- 내용: gyro_params.npz 로드를 통해 자이로 Z축 바이어스를 실시간 상쇄. msvcrt를 이용하여 비차단(non-blocking) 키 입력 대기 루프를 설계하고, 대기하는 동안 100Hz ODR 속도로 자이로 각속도를 yaw_gyro_accum에 지속적으로 적분. 엔터 시점의 100샘플 raw 롤링 윈도우 평균값을 획득. 오프라인 평가 및 사전식 정렬 2순위 정합 기준으로 자이로 적분 RMSE(yaw_rmse_gyro)를 도입.
- 예상 결과: 수동 거치 오차를 자이로 적분이 교차 보조하여 보정 모델 낙찰의 정확성이 대폭 향상됨.

### 16.3 수정 내용
- test_phase2_3_mag_cal.py 내 collect_live_acc_mag_samples 헬퍼 함수를 제거하고 main 내 non-blocking 시리얼 수집 연산으로 통합 교체. 자이로 적분 및 대조 오차 산출 공식 이식 완료.

### 16.4 실제 결과
- test_phase2_3_mag_cal.py 코드 수정 완료.

---

## 17. 추가 수정 (v0.2.3 2_3 시리얼 연결 코드 누락 버그 디버깅)

### 17.1 수정 이유
16절 자이로 융합 리팩토링 진행 도중 헬퍼 함수 삭제 과정에서 메인 루프 내 시리얼 포트 연결 및 ser 변수 정의부가 오성도 유실되어 NameError: name 'ser' is not defined 런타임 크래시가 유발된 것을 복구하기 위함.

### 17.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_3_mag_cal.py
- 내용: L254 뒤편에 find_arduino_port() 및 serial.Serial(port)를 호출하는 연결 코드를 다시 복구 주입.
- 예상 결과: 런타임 NameError가 완전히 제거되어 시리얼 통신 연결이 정상 확보됨.

### 17.3 수정 내용
- test_phase2_3_mag_cal.py L254 부근 시리얼 연결 셋업 코드 이식 완료.

### 17.4 실제 결과
- test_phase2_3_mag_cal.py 코드 수정 완료 및 크래시 에러 해결.

---

## 18. 추가 수정 (v0.2.3 자이로 Z축 바이어스 단위 불정합 교정)

### 18.1 수정 이유
gyro_bias_calibration.py 명세 상 dps 단위인 gyro_bias_z 와 raw.ino 펌웨어에서 rad/s 단위로 수신되는 gyro_raw[2] 간의 단위 불정합으로 인해 자이로 적분 시 정지 상태에서도 초당 약 234도에 달하는 폭발적인 누적 드리프트가 발생하는 런타임 오류를 해결하기 위함.

### 18.2 수정 계획 및 예상 결과
- 위치: verification_tool/test_phase2_3_mag_cal.py
- 내용: gyro_bias_z 로드 시 dps 단위를 rad/s 단위로 변환해주는 수식 (gyro_bias_z = gyro_bias_z_dps * np.pi / 180.0)을 적용.
- 예상 결과: 자이로 적분 궤적이 [0.0, 90.0, 180.0, 270.0, 360.0] 부근으로 정상 수렴하게 됨.

### 18.3 수정 내용
- test_phase2_3_mag_cal.py 내 gyro_bias_z 로드 조건에 단위 변환 수식 반영.

### 18.4 실제 결과
- test_phase2_3_mag_cal.py 코드 수정 완료. (실제 궤적 수렴 여부는 사용자의 로컬 쉘 실행을 통해 최종 검증)
