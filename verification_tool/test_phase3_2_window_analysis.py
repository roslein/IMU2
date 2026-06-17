"""
Real-world IMU Phase 3.2 2D Window Analysis (test_phase3_2_window_analysis.py)
목적: 보정 윈도우 시간(T_cal)과 자세 추정 윈도우 시간(T_est)을 0.1초에서 100초까지 그리드 스캔하여
      가속도계 정합, 자력계 정합, 3D 쿼터니언 자세(Modulo 보상)의 RMSE 변화를 
      3차원 곡면(3D Surface Plot)으로 시각화하고 최적의 윈도우 파라미터를 규명합니다.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_scipy
from scipy.optimize import least_squares

# 로컬 모듈 탐색 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMU_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.append(IMU_ROOT)
sys.path.append(os.path.join(IMU_ROOT, 'calibration_tool'))
sys.path.append(os.path.join(IMU_ROOT, '..', 'imu_simulation'))

import icosahedron
from utils.quaternion_math import q_angle_error, q_mult, q_conj

# 12-parameter 가속도계 보정 솔버
def calibrate_acc_12param(d, normals, max_iter=30):
    n_points = len(d)
    matched_normals = np.zeros_like(d)
    best_indices = []
    
    for i in range(n_points):
        best_idx, _ = icosahedron.match_face(-d[i], normals)
        matched_normals[i] = normals[best_idx]
        best_indices.append(best_idx)
        
    W_est = np.eye(3)
    b_est = np.zeros(3)
    alpha, beta = 0.0, 0.0
    
    for iteration in range(max_iter):
        R_tilt = R_scipy.from_euler('yx', [beta, alpha]).as_matrix()
        g_ref = (R_tilt @ matched_normals.T).T
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
        
    return W_est, b_est, best_indices

# 9-parameter 자력계 보정 솔버
def calibrate_mag_9param(mag_raw):
    def residuals(p, d):
        b = p[:3]
        W = np.array([
            [p[3], p[4], p[5]],
            [p[4], p[6], p[7]],
            [p[5], p[7], p[8]]
        ])
        cal = (W @ (d - b).T).T
        return np.linalg.norm(cal, axis=1) - 1.0
        
    mean_mag = np.mean(mag_raw, axis=0)
    mag_norms = np.linalg.norm(mag_raw - mean_mag, axis=1)
    avg_radius = np.mean(mag_norms) if np.mean(mag_norms) > 0 else 1.0
    init_scale = 1.0 / avg_radius
    
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

def compute_theoretical_gt_quaternions(normals_jig, R_mount):
    q_gt_lut = []
    R_sensor_to_ned_lut = []
    for i in range(20):
        n_jig = normals_jig[i]
        res_rot, _ = R_scipy.align_vectors(np.array([[0.0, 0.0, 1.0]]), np.array([n_jig]))
        R_jig_to_ned = res_rot.as_matrix()
        R_sensor_to_ned = R_jig_to_ned @ R_mount.T
        R_sensor_to_ned_lut.append(R_sensor_to_ned)
        
        r = R_scipy.from_matrix(R_sensor_to_ned)
        q_raw = r.as_quat()
        q_gt = np.array([q_raw[3], q_raw[0], q_raw[1], q_raw[2]])
        q_gt_lut.append(q_gt)
    return np.array(q_gt_lut), R_sensor_to_ned_lut

def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
        
    print("=" * 60)
    print(" 🎯 IMU Phase 3.2 Calibration vs Est Window Multi-Analysis")
    print("=" * 60)
    
    # 1. 실측 데이터 로드
    data_path = os.path.join(SCRIPT_DIR, "output", "collected_data_100s.npz")
    
    if not os.path.exists(data_path):
        print("❌ 실측 원시 데이터가 존재하지 않습니다.")
        print(f"💡 가동 전 {data_path} 경로에 data_collection_100s.py를 실행하여 데이터를 먼저 수집하십시오.")
        sys.exit(1)
        
    data = np.load(data_path)
    acc_100s = data["acc"]  # (20, N_samples, 3)
    mag_100s = data["mag"]  # (20, N_samples, 3)
    
    N_samples = acc_100s.shape[1]
    max_time = N_samples / 100.0  # 100Hz 기준 최대 가용 시간
    print(f"📂 실측 {max_time}초 원시 데이터셋 로드 완료 (Shape: {acc_100s.shape})")
    
    # 2. 기하 파라미터 및 R_mount 로드
    normals_jig = icosahedron.get_icosahedron_normals()
    R_mount = icosahedron.get_jig_to_sensor_rotation()
    
    q_gt_lut, R_sensor_to_ned_lut = compute_theoretical_gt_quaternions(normals_jig, R_mount)
    rot_normals = icosahedron.get_rotated_normals()
    
    # 3. 전체 데이터 기준 자북 레퍼런스 벡터 사전 구정 (Inclination 바이어스 튜닝)
    acc_mean_all = np.mean(acc_100s, axis=1)
    mag_mean_all = np.mean(mag_100s, axis=1)
    
    # 가속도 12-param 피팅으로 전체 정합
    W_acc_all, b_acc_all, best_indices = calibrate_acc_12param(acc_mean_all, rot_normals)
    W_mag_all, b_mag_all = calibrate_mag_9param(mag_mean_all)
    
    acc_cal_all = (W_acc_all @ (acc_mean_all - b_acc_all).T).T
    mag_cal_all = (W_mag_all @ (mag_mean_all - b_mag_all).T).T
    
    m_ned_list = []
    for i in range(20):
        best_idx = best_indices[i]
        m_ned = R_sensor_to_ned_lut[best_idx] @ mag_cal_all[i]
        m_ned_list.append(m_ned)
    m_ned_mean = np.mean(m_ned_list, axis=0)
    m_ned_ref = m_ned_mean / np.linalg.norm(m_ned_mean)
    print(f"📡 튜닝된 3D 자북 레퍼런스 벡터 (m_ned_ref): {m_ned_ref}")
    
    # 4. 윈도우 그리드 정의 (수집 데이터 크기에 비례하여 동적 선언)
    ratios = np.array([0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 0.9, 1.0])
    T_cal_list = np.round(max_time * ratios, 1)
    T_est_list = np.round(max_time * ratios, 1)
    
    # 최소 윈도우가 0.1초 미만이 되지 않도록 보장
    T_cal_list = np.clip(T_cal_list, 0.1, None)
    T_est_list = np.clip(T_est_list, 0.1, None)
    
    # 중복 제거 및 정렬
    T_cal_list = np.unique(T_cal_list)
    T_est_list = np.unique(T_est_list)
    
    # 3D 표면도용 그리드 메쉬
    X, Y = np.meshgrid(T_cal_list, T_est_list)
    Z_acc = np.zeros_like(X)
    Z_mag = np.zeros_like(X)
    Z_quat = np.zeros_like(X)
    
    # 🎯 정량 수치 보고서 텍스트 저장을 위한 리스트 초기화
    txt_report_lines = []
    txt_report_lines.append("=" * 80)
    txt_report_lines.append(" 🎯 IMU Phase 3.2 Calibration vs Est Window Analysis Report")
    txt_report_lines.append("=" * 80)
    txt_report_lines.append(f"Data Source: collected_data_100s.npz")
    txt_report_lines.append(f"3D Local Earth Magnetic Vector Reference (NED): {m_ned_ref.tolist()}")
    txt_report_lines.append("-" * 80)
    txt_report_lines.append(f"{'T_cal (s)':<12}{'T_est (s)':<12}{'Accel RMSE (deg)':<20}{'Mag RMSE (deg)':<18}{'Quaternion RMSE (deg)':<22}")
    txt_report_lines.append("-" * 80)
    
    # 대표 T_cal (최소, 중간, 최대) 선정 및 데이터 저장소 정의
    t_cal_min = T_cal_list[0]
    t_cal_mid = T_cal_list[len(T_cal_list) // 2]
    t_cal_max = T_cal_list[-1]
    
    representative_cal_data = {
        t_cal_min: {},
        t_cal_mid: {},
        t_cal_max: {}
    }
    
    print(f"📊 [동적 격자 스캔] T_cal 후보군: {T_cal_list}")
    print(f"📊 [동적 격자 스캔] T_est 후보군: {T_est_list}")
    print("\n📊 2D 윈도우 하이브리드 파라미터 그리드 스캔 개시...")
    
    # 100Hz 기준 윈도우 인덱스 매핑 사전 계산
    sample_indices_cal = [int(t * 100) for t in T_cal_list]
    sample_indices_est = [int(t * 100) for t in T_est_list]
    
    # 5. 그리드 루프 구동
    for i, t_cal in enumerate(T_cal_list):
        n_cal = sample_indices_cal[i]
        # 보정 윈도우 슬라이싱 평균
        acc_cal_raw = np.mean(acc_100s[:, :n_cal, :], axis=1)
        mag_cal_raw = np.mean(mag_100s[:, :n_cal, :], axis=1)
        
        # 보정 파라미터 산출
        W_acc, b_acc, _ = calibrate_acc_12param(acc_cal_raw, rot_normals)
        W_mag, b_mag = calibrate_mag_9param(mag_cal_raw)
        
        # 대표 T_cal에 매칭 시 시각화용 데이터 캡처
        if t_cal in representative_cal_data:
            acc_cal_cal = (W_acc @ (acc_cal_raw - b_acc).T).T
            mag_cal_cal = (W_mag @ (mag_cal_raw - b_mag).T).T
            representative_cal_data[t_cal] = {
                "acc_raw": acc_cal_raw,
                "mag_raw": mag_cal_raw,
                "acc_cal": acc_cal_cal,
                "mag_cal": mag_cal_cal
            }
        
        for j, t_est in enumerate(T_est_list):
            n_est = sample_indices_est[j]
            # 자세 측정 윈도우 슬라이싱 평균
            acc_est_raw = np.mean(acc_100s[:, :n_est, :], axis=1)
            mag_est_raw = np.mean(mag_100s[:, :n_est, :], axis=1)
            
            # 보정 대수식 적용
            acc_est_cal = (W_acc @ (acc_est_raw - b_acc).T).T
            mag_est_cal = (W_mag @ (mag_est_raw - b_mag).T).T
            
            # 가속도 및 자력계 정합 오차(각도 에러) 계산
            g_errors = []
            m_errors = []
            q_est_list = []
            
            for k in range(20):
                best_idx = best_indices[k]
                
                # 가속도 각도 오차
                R_s2n = R_sensor_to_ned_lut[best_idx]
                g_gt_sensor = R_s2n.T @ np.array([0.0, 0.0, 1.0])
                g_est_sensor = acc_est_cal[k] / np.linalg.norm(acc_est_cal[k])
                dot_g = np.clip(np.dot(g_gt_sensor, g_est_sensor), -1.0, 1.0)
                g_errors.append(np.degrees(np.arccos(dot_g)))
                
                # 자력계 각도 오차
                m_gt_sensor = R_s2n.T @ m_ned_ref
                m_est_sensor = mag_est_cal[k] / np.linalg.norm(mag_est_cal[k])
                dot_m = np.clip(np.dot(m_gt_sensor, m_est_sensor), -1.0, 1.0)
                m_errors.append(np.degrees(np.arccos(dot_m)))
                
                # SVD 자세 추정
                v_sensor = np.array([g_est_sensor, m_est_sensor])
                v_ned = np.array([np.array([0.0, 0.0, 1.0]), m_ned_ref])
                res_rot, _ = R_scipy.align_vectors(v_ned, v_sensor)
                q_scipy = res_rot.as_quat()
                q_est = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])
                q_est_list.append(q_est)
                
            # 0번 면 기준 Yaw 정렬 오프셋 산출
            q_gt0 = q_gt_lut[best_indices[0]]
            q_est0 = q_est_list[0]
            q_offset = q_mult(q_gt0, q_conj(q_est0))
            q_offset /= np.linalg.norm(q_offset)
            
            # Modulo 보상 후 쿼터니언 순수 오차 산출
            ideal_jig_offsets = [0.0, 60.0, 120.0, 180.0]
            q_errors = []
            for k in range(20):
                best_idx = best_indices[k]
                q_est_aligned = q_mult(q_offset, q_est_list[k])
                q_est_aligned /= np.linalg.norm(q_est_aligned)
                err_deg = q_angle_error(q_gt_lut[best_idx], q_est_aligned)
                residual_err = min(abs(err_deg - offset) for offset in ideal_jig_offsets)
                q_errors.append(residual_err)
                
            # 20개 포지션에 대한 RMSE 도출
            rmse_acc = np.sqrt(np.mean(np.array(g_errors)**2))
            rmse_mag = np.sqrt(np.mean(np.array(m_errors)**2))
            rmse_quat = np.sqrt(np.mean(np.array(q_errors)**2))
            
            Z_acc[j, i] = rmse_acc
            Z_mag[j, i] = rmse_mag
            Z_quat[j, i] = rmse_quat
            
            # 수치 정보 라인 누적
            line_str = f"{t_cal:<12.1f}{t_est:<12.1f}{rmse_acc:<20.4f}{rmse_mag:<18.4f}{rmse_quat:<22.4f}"
            txt_report_lines.append(line_str)
            
        print(f"   ↳ [스캔 완료] T_cal = {t_cal:5.1f}s | T_est 수집 루프 분석 완료")
        
    print("\n🎉 모든 파라미터 그리드 스캔이 완수되었습니다. 3D Surface 시각화를 개시합니다.")
    
    # 6. 3D Surface 시각화 및 플롯 저장
    fig = plt.figure(figsize=(18, 5.5))
    fig.suptitle("3D Surface: IMU Calibration & Estimation Window Size Analysis", fontsize=14, fontweight='bold')
    
    # Subplot 1: 가속도계 정합 오차
    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    surf1 = ax1.plot_surface(X, Y, Z_acc, cmap='coolwarm', edgecolor='none', alpha=0.9)
    ax1.set_title("Accelerometer Alignment RMSE (deg)", fontsize=10)
    ax1.set_xlabel("T_cal Window (s)")
    ax1.set_ylabel("T_est Window (s)")
    ax1.set_zlabel("RMSE [deg]")
    fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=10)
    
    # Subplot 2: 자력계 정합 오차
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    surf2 = ax2.plot_surface(X, Y, Z_mag, cmap='coolwarm', edgecolor='none', alpha=0.9)
    ax2.set_title("Magnetometer Alignment RMSE (deg)", fontsize=10)
    ax2.set_xlabel("T_cal Window (s)")
    ax2.set_ylabel("T_est Window (s)")
    ax2.set_zlabel("RMSE [deg]")
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10)
    
    # Subplot 3: 쿼터니언 자세 오차
    ax3 = fig.add_subplot(1, 3, 3, projection='3d')
    surf3 = ax3.plot_surface(X, Y, Z_quat, cmap='coolwarm', edgecolor='none', alpha=0.9)
    ax3.set_title("Quaternion Orientation Pure RMSE (deg)", fontsize=10)
    ax3.set_xlabel("T_cal Window (s)")
    ax3.set_ylabel("T_est Window (s)")
    ax3.set_zlabel("RMSE [deg]")
    fig.colorbar(surf3, ax=ax3, shrink=0.5, aspect=10)
    
    plt.tight_layout()
    
    # 결과 그래프 자동 백업 저장
    output_verify_dir = os.path.join(SCRIPT_DIR, "output")
    if not os.path.exists(output_verify_dir):
        os.makedirs(output_verify_dir)
        
    # 🎯 정량 수치 보고서 텍스트 파일 저장
    txt_save_path = os.path.join(output_verify_dir, "window_analysis_report.txt")
    with open(txt_save_path, "w", encoding="utf-8") as f:
        f.write("\n".join(txt_report_lines))
    print(f"\n📂 [정량 수치 저장 성공] 윈도우 스캔 결과 보고서 저장 완료 ➔ {txt_save_path}")
    
    save_path = os.path.join(output_verify_dir, "test_phase3_2_window_analysis_result.png")
    plt.savefig(save_path, dpi=300)
    print(f"🎉 [시각화 완료] 3D Surface 플롯 저장 완료 ➔ {save_path}")
    # plt.close()  # 대화형 창 동시 팝업을 위해 닫지 않음
    
    # 7. 대표 T_cal 크기별 3D 구면 정합 Scatter 대조 플로팅 (test_phase2 대조 형태)
    fig2 = plt.figure(figsize=(18, 11))
    fig2.suptitle("IMU Calibration Sphere Fitting vs Subsampling Window (T_cal)", fontsize=16, fontweight='bold')
    
    # 이상적 3D 구면 메쉬
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    x_sphere = np.outer(np.cos(u), np.sin(v))
    y_sphere = np.outer(np.sin(u), np.sin(v))
    z_sphere = np.outer(np.ones(np.size(u)), np.cos(v))
    
    rep_times = [t_cal_min, t_cal_mid, t_cal_max]
    
    # 2행 3열 서브플롯 순차 렌더링
    for idx, t in enumerate(rep_times):
        cal_data = representative_cal_data[t]
        if not cal_data:
            continue
            
        # 행 1: 가속도계 (열 idx+1)
        ax_acc = fig2.add_subplot(2, 3, idx + 1, projection='3d')
        ax_acc.set_title(f"Accel: T_cal = {t}s", fontsize=12)
        
        # Raw 데이터 (1000 cnt 기준 1.0스케일 정규화 플롯)
        ax_acc.scatter(cal_data["acc_raw"][:, 0]/1000.0, 
                       cal_data["acc_raw"][:, 1]/1000.0, 
                       cal_data["acc_raw"][:, 2]/1000.0, 
                       color='red', s=40, alpha=0.8, edgecolors='black', label='Before (Raw/1000)')
                       
        # Calibrated 데이터
        ax_acc.scatter(cal_data["acc_cal"][:, 0], 
                       cal_data["acc_cal"][:, 1], 
                       cal_data["acc_cal"][:, 2], 
                       color='green', s=60, alpha=0.9, edgecolors='black', label='After (Calibrated 1g)')
                       
        ax_acc.plot_wireframe(x_sphere, y_sphere, z_sphere, color='gray', alpha=0.15, linewidth=0.5)
        ax_acc.set_xlim([-1.5, 1.5])
        ax_acc.set_ylim([-1.5, 1.5])
        ax_acc.set_zlim([-1.5, 1.5])
        ax_acc.set_xlabel('X')
        ax_acc.set_ylabel('Y')
        ax_acc.set_zlabel('Z')
        if idx == 2:
            ax_acc.legend(loc='upper right')
        ax_acc.grid(True)
        
        # 행 2: 자력계 (열 idx+4)
        ax_mag = fig2.add_subplot(2, 3, idx + 4, projection='3d')
        ax_mag.set_title(f"Mag: T_cal = {t}s", fontsize=12)
        
        # Raw 데이터 (10000 cnt 기준 1.0스케일 정규화 플롯)
        ax_mag.scatter(cal_data["mag_raw"][:, 0]/10000.0, 
                       cal_data["mag_raw"][:, 1]/10000.0, 
                       cal_data["mag_raw"][:, 2]/10000.0, 
                       color='orange', s=40, alpha=0.8, edgecolors='black', label='Before (Raw/10000)')
                       
        # Calibrated 데이터
        ax_mag.scatter(cal_data["mag_cal"][:, 0], 
                       cal_data["mag_cal"][:, 1], 
                       cal_data["mag_cal"][:, 2], 
                       color='blue', s=60, alpha=0.9, edgecolors='black', label='After (Calibrated Norm=1.0)')
                       
        ax_mag.plot_wireframe(x_sphere, y_sphere, z_sphere, color='gray', alpha=0.15, linewidth=0.5)
        ax_mag.set_xlim([-1.8, 1.8])
        ax_mag.set_ylim([-1.8, 1.8])
        ax_mag.set_zlim([-1.8, 1.8])
        ax_mag.set_xlabel('X')
        ax_mag.set_ylabel('Y')
        ax_mag.set_zlabel('Z')
        if idx == 2:
            ax_mag.legend(loc='upper right')
        ax_mag.grid(True)
        
    plt.tight_layout()
    save_path_sphere = os.path.join(output_verify_dir, "test_phase3_2_window_analysis_result_spheres.png")
    plt.savefig(save_path_sphere, dpi=300)
    print(f"🎉 [시각화 완료] 3D Sphere 대조 플롯 저장 완료 ➔ {save_path_sphere}")
    plt.show()

if __name__ == "__main__":
    main()
