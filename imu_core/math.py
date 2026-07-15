import numpy as np
from scipy.spatial.transform import Rotation as R_scipy
from scipy.optimize import least_squares
import imu_core.constants as const
import imu_core.icosahedron as icosahedron

def apply_gyro_scale(gyro_raw: np.ndarray) -> np.ndarray:
    """mdps 단위 원시 자이로 각속도를 rad/s로 환산"""
    return (gyro_raw / 1000.0) * (np.pi / 180.0)

def project_gyro_tilt(gyro_cal: np.ndarray, R_tilt_fixed: np.ndarray) -> np.ndarray:
    """바이어스가 차감된 3축 자이로에서 고정 틸트 보정 행렬을 활용하여 지구 수직 Z축(Yaw) 각속도 성분을 추출"""
    if gyro_cal.ndim == 1:
        return R_tilt_fixed[2, :] @ gyro_cal
    else:
        return gyro_cal @ R_tilt_fixed[2, :].T

def compute_closed_loop_error(integrated_yaw: np.ndarray) -> float:
    """1회전 수평 회전 시 최종 적분 Yaw와 최초 Yaw + 360도의 편차를 계산"""
    return float(integrated_yaw[-1] - (integrated_yaw[0] + 360.0))

def align_vectors_svd(acc_cal: np.ndarray, mag_cal: np.ndarray, m_ned_ref: np.ndarray = None) -> np.ndarray:
    """
    SVD(Kabsch) 기반의 3D 절대 자세(쿼터니언) 복원
    acc_cal과 mag_cal을 지구 기준 벡터(중력: [0, 0, 1], 지자기: m_ned_ref)와 정합
    """
    if m_ned_ref is None:
        # 서울 표준 복각 기준 지자기 레퍼런스 [cos(54 deg), 0, sin(54 deg)]
        cos_dip = np.cos(const.DIP_IDEAL_SEOUL_RAD)
        sin_dip = np.sin(const.DIP_IDEAL_SEOUL_RAD)
        m_ned_ref = np.array([cos_dip, 0.0, sin_dip])

    acc_norm = np.linalg.norm(acc_cal)
    mag_norm = np.linalg.norm(mag_cal)
    
    g_sens = acc_cal / (acc_norm if acc_norm > 0 else 1.0)
    m_sens = mag_cal / (mag_norm if mag_norm > 0 else 1.0)
    
    v_sensor = np.array([g_sens, m_sens])
    v_ned = np.array([np.array([0.0, 0.0, 1.0]), m_ned_ref])
    
    res_rot, _ = R_scipy.align_vectors(v_ned, v_sensor)
    q_scipy = res_rot.as_quat() # [x, y, z, w]
    # [w, x, y, z] 표준 포맷으로 재정렬
    return np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])

def compute_geodesic_distance(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """두 쿼터니언 [w, x, y, z] 간의 측지선 구면 각도 오차 계산 (단위: deg)"""
    # 내적 크기
    dots = np.sum(q1 * q2, axis=-1)
    clipped = np.clip(np.abs(dots), -1.0, 1.0)
    return np.degrees(2.0 * np.arccos(clipped))

def q_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """두 쿼터니언 [w, x, y, z]의 곱 계산"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

def q_conj(q: np.ndarray) -> np.ndarray:
    """쿼터니언 [w, x, y, z]의 켤레(conjugate) 반환"""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def compute_dip_angle(mag_cal: np.ndarray, q_est: np.ndarray) -> float:
    """보정 자력을 추정 쿼터니언을 통해 지구 NED로 투영하여 엎침각(Dip Angle) 복원 (단위: rad)"""
    # 쿼터니언을 이용한 벡터 회전: q_est * [0, mag_cal] * q_est_conj
    q_mag = np.array([0.0, mag_cal[0], mag_cal[1], mag_cal[2]])
    q_est_conj = q_conj(q_est)
    q_temp = q_mult(q_est, q_mag)
    q_global = q_mult(q_temp, q_est_conj)
    mag_global = q_global[1:4]
    
    horizontal_norm = np.sqrt(mag_global[0]**2 + mag_global[1]**2)
    return np.arctan2(mag_global[2], horizontal_norm)

def compute_dip_angle_error(mag_cal_seq: np.ndarray, q_est_seq: np.ndarray, dip_ideal_rad: float = None) -> float:
    """20개 안착 지점 전체에서 복원한 엎침각들과 서울 이상적 엎침각 간의 RMSE 평가"""
    if dip_ideal_rad is None:
        dip_ideal_rad = const.DIP_IDEAL_SEOUL_RAD
    
    n_points = len(mag_cal_seq)
    dip_errors = []
    for i in range(n_points):
        dip_est = compute_dip_angle(mag_cal_seq[i], q_est_seq[i])
        dip_errors.append(dip_est - dip_ideal_rad)
    return float(np.sqrt(np.mean(np.array(dip_errors)**2)))

def calibrate_sensor_accel(acc_raw: np.ndarray, W_acc: np.ndarray, b_acc: np.ndarray) -> np.ndarray:
    """가속도계 12-parameter affine 보정"""
    if acc_raw.ndim == 1:
        return W_acc @ (acc_raw - b_acc)
    else:
        return (W_acc @ (acc_raw - b_acc).T).T

def calibrate_sensor_mag(mag_raw: np.ndarray, W_mag: np.ndarray, b_mag: np.ndarray, mode: int = 9) -> np.ndarray:
    """지자기 센서 3/6/9-parameter 보정 수식 분기"""
    if mode == 3:
        # 3-param: 바이어스 소거만 수행 (W_mag 강제 단위행렬)
        W_cal = np.eye(3)
        b_cal = b_mag
    elif mode == 6:
        # 6-param: 대각 스케일링 성분만 연산
        W_cal = np.diag(np.diag(W_mag))
        b_cal = b_mag
    else:
        # 9-param: full SPD 보정 행렬곱 연산
        W_cal = W_mag
        b_cal = b_mag

    if mag_raw.ndim == 1:
        return W_cal @ (mag_raw - b_cal)
    else:
        return (W_cal @ (mag_raw - b_cal).T).T

def compute_yaw_rmse(integrated_yaw: np.ndarray, gt_yaw: np.ndarray) -> float:
    """1D 회전 궤적 전체 Yaw RMSE 편차 연산"""
    # 360도 랩핑에 따른 편차를 고려하여 각도 차이의 최단 거리 계산
    diffs = (integrated_yaw - gt_yaw + 180.0) % 360.0 - 180.0
    return float(np.sqrt(np.mean(diffs**2)))

def match_face_index(acc_meas: np.ndarray, normals: np.ndarray) -> tuple:
    """실시간 20면체 안착면 감지 매칭 (icosahedron 모듈 위임)"""
    return icosahedron.match_face(acc_meas, normals)

def slice_window_statistics(raw_data: np.ndarray, window_size: int) -> np.ndarray:
    """
    윈도우 시간 분할 데이터 평균 가공 헬퍼
    입력: (N, 3) 시계열 데이터
    출력: (N_slices, 3) 앙상블 평균 데이터
    """
    n_samples = len(raw_data)
    slices = []
    for start in range(0, n_samples - window_size + 1, window_size):
        slice_mean = np.mean(raw_data[start : start + window_size], axis=0)
        slices.append(slice_mean)
    return np.array(slices)

# ==============================================================================
# 🎯 9축 센서 모듈화 피팅 솔버 (Calibration Solvers)
# ==============================================================================

def calibrate_acc_12param_icosahedron(d: np.ndarray, normals: np.ndarray, max_iter: int = 50) -> tuple:
    """
    정20면체 12-parameter 가속도계 경사보상 Recursive 최소제곱 솔버
    d: 실측된 20개 정적 평균 가속도 벡터 (20, 3)
    normals: 정20면체의 이론적 20개 면 법선 벡터 (20, 3)
    반환: W_est (3x3 보정행렬), b_est (3x1 오프셋), alpha (Pitch경사), beta (Roll경사)
    """
    n_points = len(d)
    matched_normals = np.zeros_like(d)
    
    for i in range(n_points):
        best_idx, _ = icosahedron.match_face(-d[i], normals)
        matched_normals[i] = normals[best_idx]
        
    W_est = np.eye(3)
    b_est = np.zeros(3)
    alpha, beta = 0.0, 0.0
    
    for iteration in range(max_iter):
        R_tilt = R_scipy.from_euler('yx', [beta, alpha]).as_matrix()
        g_ref = -(R_tilt @ matched_normals.T).T
        A = np.hstack([g_ref, np.ones((n_points, 1))])
        
        sol, _, _, _ = np.linalg.lstsq(A, d, rcond=None)
        M_T = sol[:3, :]
        b_new = sol[3, :]
        
        W_new = np.linalg.inv(M_T.T)
        d_cal = (W_new @ (d - b_new).T).T
        
        pitch_errs = []
        roll_errs = []
        for i in range(n_points):
            ref = matched_normals[i]
            cal = d_cal[i] / np.linalg.norm(d_cal[i])
            pitch_errs.append(cal[1] - ref[1])
            roll_errs.append(cal[0] - ref[0])
            
        alpha = np.arcsin(np.clip(np.mean(pitch_errs), -1.0, 1.0))
        beta = np.arcsin(np.clip(np.mean(roll_errs), -1.0, 1.0))
        
        if np.allclose(b_est, b_new, atol=1e-8):
            W_est, b_est = W_new, b_new
            break
        W_est, b_est = W_new, b_new
        
    return W_est, b_est, alpha, beta


def calibrate_mag_3param(mag_raw: np.ndarray) -> tuple:
    """3-parameter 자력계 오프셋 피팅 (W_mag는 스케일 역수 Identity 고정)"""
    mean_raw = np.mean(mag_raw, axis=0)
    dev_norms = np.linalg.norm(mag_raw - mean_raw, axis=1)
    scale_factor = np.mean(dev_norms) if np.mean(dev_norms) > 0 else 1.0
    d = (mag_raw - mean_raw) / scale_factor
    
    def residuals(b, x):
        return np.linalg.norm(x - b, axis=1) - 1.0
    p0 = np.array([0.0, 0.0, 0.0])
    res = least_squares(residuals, p0, args=(d,), method='lm')
    
    b_est = mean_raw + scale_factor * res.x
    W_est = np.eye(3) / scale_factor
    return W_est, b_est


def calibrate_mag_6param(mag_raw: np.ndarray) -> tuple:
    """6-parameter 자력계 대각 스케일링 + 오프셋 피팅"""
    mean_raw = np.mean(mag_raw, axis=0)
    dev_norms = np.linalg.norm(mag_raw - mean_raw, axis=1)
    scale_factor = np.mean(dev_norms) if np.mean(dev_norms) > 0 else 1.0
    d = (mag_raw - mean_raw) / scale_factor

    def residuals(p, x):
        b = p[:3]
        W = np.diag(np.exp(p[3:6]))
        cal = (W @ (x - b).T).T
        return np.linalg.norm(cal, axis=1) - 1.0
    p0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    res = least_squares(residuals, p0, args=(d,), method='lm')
    
    p = res.x
    b_est = mean_raw + scale_factor * p[:3]
    W_est = np.diag(np.exp(p[3:6])) / scale_factor
    return W_est, b_est


def calibrate_mag_ellipsoid(mag_raw: np.ndarray) -> tuple:
    """9-parameter 자력계 대칭 타원체(Soft/Hard Iron) 피팅"""
    mean_mag = np.mean(mag_mag_raw := mag_raw, axis=0)
    mag_norms = np.linalg.norm(mag_mag_raw - mean_mag, axis=1)
    avg_radius = np.mean(mag_norms) if np.mean(mag_norms) > 0 else 1.0
    init_scale = 1.0 / avg_radius
    
    def residuals(p, d):
        b = p[:3]
        W = np.array([
            [p[3], p[4], p[5]],
            [p[4], p[6], p[7]],
            [p[5], p[7], p[8]]
        ])
        cal = (W @ (d - b).T).T
        return np.linalg.norm(cal, axis=1) - 1.0
        
    p0 = np.concatenate([
        mean_mag, 
        [init_scale, 0.0, 0.0, init_scale, 0.0, init_scale]
    ])
    
    res = least_squares(residuals, p0, args=(mag_raw,), method='lm')
    p = res.x
    b_est = p[:3]
    W_est = np.array([
        [p[3], p[4], p[5]],
        [p[4], p[6], p[7]],
        [p[5], p[7], p[8]]
    ])
    
    return W_est, b_est


def calibrate_gyro_bias_global(gyro_raw: np.ndarray) -> np.ndarray:
    """20-Positions (또는 240포인트) 전체 자이로 데이터 참 평균을 통한 정적 오프셋 도출"""
    return np.mean(gyro_raw, axis=0)


def calibrate_mag_task_aware(mag_raw: np.ndarray, acc_cal: np.ndarray, yaw_gt: np.ndarray, 
                             W_mag_0: np.ndarray, b_mag_0: np.ndarray, 
                             m_ned_ref: np.ndarray = None, mode: int = 9) -> tuple:
    """
    Stage 2: 수평 회전판의 Yaw GT 각도 편차 자체를 최소화하는 Task-Aware 비선형 최적화 솔버
    mode: 3 (Offset Only), 6 (Diagonal W), 9 (Symmetric W)
    """
    if m_ned_ref is None:
        # 인천 표준 복각 54.3도 기준 디폴트 레퍼런스
        m_ned_ref = np.array([0.583503, 0.0, 0.812108])

    # 1. 최적화 변수 초기값 구성
    if mode == 3:
        p0 = b_mag_0.copy()
    elif mode == 6:
        # log(diag(W)) 적용하여 양수 제약 강제
        diag_val = np.diag(W_mag_0)
        diag_val = np.where(diag_val <= 0, 1e-5, diag_val)
        p0 = np.concatenate([b_mag_0, np.log(diag_val)])
    else: # mode == 9
        p0 = np.concatenate([
            b_mag_0, 
            [W_mag_0[0,0], W_mag_0[0,1], W_mag_0[0,2], W_mag_0[1,1], W_mag_0[1,2], W_mag_0[2,2]]
        ])

    # 2. 잔차 함수 정의
    def residuals(p, m_raw, a_cal, y_gt, ref_vec):
        if mode == 3:
            b = p[:3]
            W = W_mag_0
        elif mode == 6:
            b = p[:3]
            W = np.diag(np.exp(p[3:6]))
        else: # mode == 9
            b = p[:3]
            W = np.array([
                [p[3], p[4], p[5]],
                [p[4], p[6], p[7]],
                [p[5], p[7], p[8]]
            ])
            
        n_points = len(m_raw)
        errors = []
        for i in range(n_points):
            m_cal = W @ (m_raw[i] - b)
            # SVD 절대 자세 복조 q = [qw, qx, qy, qz]
            q_est = align_vectors_svd(a_cal[i], m_cal, ref_vec)
            # scipy quaternion [qx, qy, qz, qw] 정렬
            res_rot = R_scipy.from_quat([q_est[1], q_est[2], q_est[3], q_est[0]])
            yaw_est = res_rot.as_euler('xyz', degrees=True)[2]
            
            # Wraparound 360도 보정한 각도 편차
            diff = (yaw_est - y_gt[i] + 180.0) % 360.0 - 180.0
            errors.append(diff)
            
        return np.array(errors)

    # 3. 비선형 최적화 구동
    res = least_squares(residuals, p0, args=(mag_raw, acc_cal, yaw_gt, m_ned_ref), method='trf')
    
    # 4. 결과 파라미터 복원
    p_opt = res.x
    if mode == 3:
        b_final = p_opt[:3]
        W_final = W_mag_0
    elif mode == 6:
        b_final = p_opt[:3]
        W_final = np.diag(np.exp(p_opt[3:6]))
    else: # mode == 9
        b_final = p_opt[:3]
        W_final = np.array([
            [p_opt[3], p_opt[4], p_opt[5]],
            [p_opt[4], p_opt[6], p_opt[7]],
            [p_opt[5], p_opt[7], p_opt[8]]
        ])
        
    return W_final, b_final

