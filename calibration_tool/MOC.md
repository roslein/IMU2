# Calibration Tool MOC

---

## 캘리브레이션 알고리즘 코어

[[icosahedron.py]]
: 정20면체의 기하학적 12개 꼭짓점 좌표와 20개 면 법선 단위 벡터를 사전 계산하고 최적 매칭하는 유틸리티.

[[accel_calibration.py]]
: 20개 안착 가속도 데이터를 기반으로 경사각 보상을 포함한 12-parameter 가속도계 선형 최소제곱 보정 솔버.

[[mag_calibration.py]]
: 20개 안착 지자기 데이터를 기반으로 Hard Iron 및 Soft Iron 왜곡을 보정하는 9-parameter 타구체 피팅 비선형 최소제곱 솔버.

[[gyro_bias_calibration.py]]
: 20면체 거치 측정 데이터셋 전체를 로드하여 정지 자이로스코프 데이터의 전역 평균으로 초정밀 자이로 바이어스를 역산하는 모듈.

[[gyro_scale_calibration.py]]
: 향후 자이로 스케일 왜곡 추가 교정을 위해 단위 행렬 기반으로 연산을 사전 탑재하여 초기화하는 보조 모듈.

---

## 데이터 수집 및 파이프라인 빌더

[[data_collection.py]]
: 정20면체 지그 안착 상태를 3D 공간에 실시간 렌더링하고, 사용자의 트리거 신호에 맞춰 바이너리 시리얼 패킷 데이터를 수집 및 체크포인트 백업을 지원하는 데이터 획득 도구.

[[generate_calib_params.py]]
: 가속도, 자이로, 자력계의 보정 결과 npz 백업 파일을 취합하여 펌웨어 calibrated.ino 및 raw.ino에 바로 이식할 수 있는 calib_params.h C++ 헤더 파일을 일괄 빌드해주는 통합 툴체인.

[[mag_environment_mapping.py]]
: 20개 전방위 안착 데이터를 분석하여 해당 실험 장소(방) 고유의 3D 자북 레퍼런스 벡터를 추출해 내는 독립 맵핑 도구.
