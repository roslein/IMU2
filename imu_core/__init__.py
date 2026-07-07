# imu_core 패키지 공용 API 네임스페이스 정의 (__init__.py)

# 1. icosahedron 기하학 모듈 함수 노출
from .icosahedron import (
    get_icosahedron_normals,
    get_jig_to_sensor_rotation,
    get_rotated_normals,
    match_face
)

# 2. math 수치 연산 및 물리 캘리브레이션 함수 노출
from .math import (
    apply_gyro_scale,
    project_gyro_tilt,
    compute_closed_loop_error,
    align_vectors_svd,
    compute_geodesic_distance,
    q_mult,
    q_conj,
    compute_dip_angle,
    compute_dip_angle_error,
    calibrate_sensor_accel,
    calibrate_sensor_mag
)
