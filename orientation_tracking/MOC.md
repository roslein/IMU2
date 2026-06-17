# Orientation Tracking MOC

---

## 정적 절대 자세 초기화 및 트래킹

[[static_initialization.py]]
: 단일 포지션 거치 상태에서 로드된 자북 레퍼런스(env_params.npz)를 사용하여 SVD align_vectors 정밀 정합을 수행해 Sign Flip Singularity가 배제된 정적 절대 3D 자세(Yaw) 및 초기 쿼터니언을 복조해내는 알고리즘 모듈.

[[orientation_tracking.py]]
: 센서 데이터 스트림과 동적 필터 알고리즘을 융합하여 센서의 실시간 3차원 자세각을 지속 트래킹하는 코어 모듈.
