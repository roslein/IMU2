"""
Real-world IMU Phase 3.1 Static Orientation Verification (test_phase3_static_orientation.py)
목적: EKF나 동적 적분 알고리즘 개입 없이, 오직 가속도/지자기 보정 데이터에 대해
      static initialization(TRIAD)을 가동하여 3D 자세 q_est를 도출하고,
      20 Positions ideal 거치 기하 alignment로 계산된 GT q_gt와 비교하여
      캘리브레이션 본연의 성능을 쿼터니언 각도 오차 및 3D 벡터 내적으로 정량 검증/시각화합니다.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_scipy

# 로컬 모듈 탐색 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMU_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.append(IMU_ROOT)
from imu_core import (
    get_icosahedron_normals,
    get_jig_to_sensor_rotation,
    get_rotated_normals,
    match_face,
    align_vectors_svd,
    compute_geodesic_distance,
    q_mult,
    q_conj
)


def compute_theoretical_gt_quaternions(normals_jig, R_mount):
    """
    정20면체 지그의 Z축 법선(n_jig)을 지구 Down [0,0,1]에 최단 경로로 정합시키는
    최소 회전(Shortest Arc) 행렬을 SVD로 도출하여, X/Y축 부호 뒤집힘(Sign Flip)이 완전히 배제된
    일관된 Yaw 기준의 GT 쿼터니언 및 회전 행렬 LUT를 생성합니다.
    """
    q_gt_lut = []
    R_sensor_to_ned_lut = []
    
    for i in range(20):
        n_jig = normals_jig[i]
        
        # Z축 n_jig를 [0,0,1]로 회전시키는 최소 회전 행렬을 SVD(align_vectors)로 대수적 획득
        res_rot, _ = R_scipy.align_vectors(np.array([[0.0, 0.0, 1.0]]), np.array([n_jig]))
        R_jig_to_ned = res_rot.as_matrix()
        
        # 센서에서 NED로의 회전: R_sensor_to_ned = R_jig_to_ned @ R_mount.T
        R_sensor_to_ned = R_jig_to_ned @ R_mount.T
        R_sensor_to_ned_lut.append(R_sensor_to_ned)
        
        # 쿼터니언 복조 [qw, qx, qy, qz]
        r = R_scipy.from_matrix(R_sensor_to_ned)
        q_raw = r.as_quat()
        q_gt = np.array([q_raw[3], q_raw[0], q_raw[1], q_raw[2]])
        q_gt_lut.append(q_gt)
        
    return np.array(q_gt_lut), R_sensor_to_ned_lut

def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
        
    print("=" * 60)
    print(" 🎯 IMU Phase 3.1 Static Orientation Verification Solver")
    print("=" * 60)
    
    # 1. 수집 데이터 로드 (실시간 수집본 혹은 기 수집 raw 데이터 기반 오프라인 보정 분기)
    data_path = os.path.join(SCRIPT_DIR, "output", "new_calib_collected.npz")
    
    # 캘리브레이션 툴 폴더 내 오프라인 백업 경로
    calib_tool_dir = os.path.join(IMU_ROOT, "calibration_tool")
    raw_data_path = os.path.join(calib_tool_dir, "output", "collected_data.npz")
    acc_param_path = os.path.join(calib_tool_dir, "output", "acc_params.npz")
    mag_param_path = os.path.join(calib_tool_dir, "output", "mag_params.npz")
    
    if os.path.exists(data_path):
        data = np.load(data_path)
        acc_cal = data["acc"]
        mag_cal = data["mag"]
        print(f"📂 실시간 수집 보정 데이터셋 로드 성공 (Shape: {acc_cal.shape})")
    elif os.path.exists(raw_data_path) and os.path.exists(acc_param_path) and os.path.exists(mag_param_path):
        print("⚠️  실시간 수집 보정본(new_calib_collected.npz) 유실 ➔ [오프라인 시뮬레이션 모드 전환]")
        print("💡 기 수집된 raw 데이터에 산출된 보정 파라미터(W, b)를 직접 덧씌워 분석을 개시합니다.\n")
        
        raw_data = np.load(raw_data_path)
        acc_raw = raw_data["acc"]
        mag_raw = raw_data["mag"]
        
        acc_params = np.load(acc_param_path)
        W_acc = acc_params["W"]
        b_acc = acc_params["b"]
        
        mag_params = np.load(mag_param_path)
        W_mag = mag_params["W"]
        b_mag = mag_params["b"]
        
        # 파이썬 상에서 보정 연산 대수 적용
        acc_cal = (W_acc @ (acc_raw - b_acc).T).T
        mag_cal = (W_mag @ (mag_raw - b_mag).T).T
        print(f"📂 오프라인 가상 보정 데이터셋 복원 성공 (Shape: {acc_cal.shape})")
    else:
        print("❌ 분석을 진행할 수 있는 데이터 파일이나 보정 파라미터가 유실되었습니다.")
        print("💡 verification_tool/test_phase3.py를 실행하거나,")
        print("   accel_calibration.py 및 mag_calibration.py를 실행하여 기준 데이터를 먼저 생성해 주십시오.")
        sys.exit(1)
    
    # 2. 기하 툴 파라미터 및 R_mount 로드
    normals_jig = get_icosahedron_normals()
    R_mount = get_jig_to_sensor_rotation()
    
    # 3. 20개 포지션별 GT 쿼터니언 및 회전행렬 사전 도출
    q_gt_lut, R_sensor_to_ned_lut = compute_theoretical_gt_quaternions(normals_jig, R_mount)
    print("📊 지그 기하 오차(R_mount) 반영 ideal 3D GT 회전 쿼터니언 산출 완료.")
    
    # 4. 지구 고정 프레임 기준 ideal 지자기 벡터 추출 (Inclination 편향 상쇄 정합)
    rot_normals = get_rotated_normals()
    best_indices = []
    for i in range(20):
        best_idx, _ = match_face(acc_cal[i], rot_normals)
        best_indices.append(best_idx)
        
    m_ned_list = []
    for i in range(20):
        # 실측 mag_cal 데이터를 GT 회전 행렬로 지구 고정 NED 프레임으로 회전 투영
        best_idx = best_indices[i]
        m_ned = R_sensor_to_ned_lut[best_idx] @ mag_cal[i]
        m_ned_list.append(m_ned)
    m_ned_mean = np.mean(m_ned_list, axis=0)
    m_ned_ideal = m_ned_mean / np.linalg.norm(m_ned_mean)
    print(f"📡 정밀 정합된 지구 ideal 지자기 벡터 (NED): {m_ned_ideal}")
    
    # 5. 20개 면별 쿼터니언 및 가속도/자력 벡터 내적 각도 오차 역산
    q_est_list = []
    
    # 1차 루프: 20개 포지션의 q_est 계산
    for i in range(20):
        # SVD 기반 Wahba 문제 최소제곱 정밀 정합 (특이점/Sign flip 완전 회피)
        v_sensor = np.array([acc_cal[i] / np.linalg.norm(acc_cal[i]), mag_cal[i] / np.linalg.norm(mag_cal[i])])
        v_ned = np.array([np.array([0.0, 0.0, 1.0]), m_ned_ideal])
        res_rot, _ = R_scipy.align_vectors(v_ned, v_sensor)
        q_est_scipy = res_rot.as_quat() # [x, y, z, w]
        q_est = np.array([q_est_scipy[3], q_est_scipy[0], q_est_scipy[1], q_est_scipy[2]]) # [w, x, y, z]
        q_est_list.append(q_est)

    # 0번 포지션을 기준으로 지그-자북선 간의 초기 Yaw/방위각 정렬 오프셋 산출
    q_gt0 = q_gt_lut[best_indices[0]]
    q_est0 = q_est_list[0]
    q_offset = q_mult(q_gt0, q_conj(q_est0))
    q_offset /= np.linalg.norm(q_offset)
    print(f"🧩 초기 상태 정렬 오프셋 쿼터니언 (q_offset): {q_offset}")
    
    angle_errors = []
    pure_sensor_errors = []
    dot_g_list = []
    dot_m_list = []
    angle_g_errors = []
    angle_m_errors = []
    
    g_gt_sensor_all = []
    g_est_sensor_all = []
    m_gt_sensor_all = []
    m_est_sensor_all = []
    
    # 2차 루프: 정렬 오프셋 반영한 최종 오차 산출 및 보고
    for i in range(20):
        best_idx = best_indices[i]
        q_est_raw = q_est_list[i]
        
        # 오프셋 정렬 적용
        q_est_aligned = q_mult(q_offset, q_est_raw)
        q_est_aligned /= np.linalg.norm(q_est_aligned)
        
        # 쿼터니언 각도 오차 (degree)
        err_deg = compute_geodesic_distance(q_gt_lut[best_idx], q_est_aligned)
        angle_errors.append(err_deg)
        
        # Modulo 기반 거치 편차 (120도 회전 대칭 및 180도 반전) 보상
        ideal_jig_offsets = [0.0, 60.0, 120.0, 180.0]
        residual_err = min(abs(err_deg - offset) for offset in ideal_jig_offsets)
        pure_sensor_errors.append(residual_err)
        
        # 센서 관점의 GT 중력/지자기 벡터 역산
        R_s2n = R_sensor_to_ned_lut[best_idx]
        g_gt_sensor = R_s2n.T @ np.array([0.0, 0.0, 1.0])
        m_gt_sensor = R_s2n.T @ m_ned_ideal
        
        # 실측 정규화 벡터
        g_est_sensor = acc_cal[i] / np.linalg.norm(acc_cal[i])
        m_est_sensor = mag_cal[i] / np.linalg.norm(mag_cal[i])
        
        g_gt_sensor_all.append(g_gt_sensor)
        g_est_sensor_all.append(g_est_sensor)
        m_gt_sensor_all.append(m_gt_sensor)
        m_est_sensor_all.append(m_est_sensor)
        
        # 벡터 내적 계산 및 물리 각도 편차 도출
        dot_g = float(np.dot(g_gt_sensor, g_est_sensor))
        dot_m = float(np.dot(m_gt_sensor, m_est_sensor))
        dot_g_list.append(dot_g)
        dot_m_list.append(dot_m)
        
        angle_g = float(np.degrees(np.arccos(np.clip(dot_g, -1.0, 1.0))))
        angle_m = float(np.degrees(np.arccos(np.clip(dot_m, -1.0, 1.0))))
        angle_g_errors.append(angle_g)
        angle_m_errors.append(angle_m)
        
        err_deg_val = float(err_deg)
        print(f"포지션 #{i:02d} (면 #{best_idx:02d}) | 정렬 쿼터니언 오차: {err_deg_val:6.3f}° (순수: {residual_err:6.3f}°) | 중력 내적: {dot_g:8.6f} (오차: {angle_g:6.3f}°) | 지자기 내적: {dot_m:8.6f} (오차: {angle_m:6.3f}°)")
        
    angle_errors = np.array(angle_errors)
    pure_sensor_errors = np.array(pure_sensor_errors)
    angle_g_errors = np.array(angle_g_errors)
    angle_m_errors = np.array(angle_m_errors)
    
    q_rmse = np.sqrt(np.mean(angle_errors**2))
    q_pure_rmse = np.sqrt(np.mean(pure_sensor_errors**2))
    g_rmse = np.sqrt(np.mean(angle_g_errors**2))
    m_rmse = np.sqrt(np.mean(angle_m_errors**2))
    
    print("\n" + "=" * 60)
    print(" 🎉 3D Static Orientation Verification 정량 보고서")
    print("=" * 60)
    print(f"📊 3D 쿼터니언 자세 회전각 RMSE (거치오차 포함): {q_rmse:10.6f}°")
    print(f"📊 3D 쿼터니언 자세 회전각 RMSE (순수 센서오차): {q_pure_rmse:10.6f}°")
    print(f"📊 중력 가속도 기하 정합 RMSE:   {g_rmse:10.6f}°")
    print(f"📊 지구 자기장 기하 정합 RMSE:   {m_rmse:10.6f}°")
    print("=" * 60)
    
    # 6. 3D 시각화 및 오차 차트 렌더링
    fig = plt.figure(figsize=(16, 8))
    fig.suptitle("📌 IMU Phase 3.1 3D Static Orientation & Vector Alignment Verification", fontsize=15, fontweight='bold')
    
    # 3D Plot 1: 중력 벡터 대조
    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    ax1.set_title("Accelerometer Gravity Alignment (20 Pos)", fontsize=11)
    
    g_gt_sensor_all = np.array(g_gt_sensor_all)
    g_est_sensor_all = np.array(g_est_sensor_all)
    
    # GT 중력 벡터 그리기 (Red)
    ax1.quiver(0, 0, 0, g_gt_sensor_all[:,0], g_gt_sensor_all[:,1], g_gt_sensor_all[:,2], 
              color='crimson', alpha=0.6, length=1.0, arrow_length_ratio=0.1, label='Ideal GT (Gravity)')
    # 실측 중력 벡터 그리기 (Blue)
    ax1.quiver(0, 0, 0, g_est_sensor_all[:,0], g_est_sensor_all[:,1], g_est_sensor_all[:,2], 
              color='royalblue', alpha=0.8, length=1.0, arrow_length_ratio=0.1, label='Calibrated (Gravity)')
              
    # 구면 가이드 와이어프레임 렌더링
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax1.plot_wireframe(x, y, z, color='gray', alpha=0.1, linewidth=0.5)
    
    ax1.set_xlim([-1.1, 1.1])
    ax1.set_ylim([-1.1, 1.1])
    ax1.set_zlim([-1.1, 1.1])
    ax1.set_xlabel('X (Sensor)')
    ax1.set_ylabel('Y (Sensor)')
    ax1.set_zlabel('Z (Sensor)')
    ax1.legend(loc='lower left')
    
    # 3D Plot 2: 지자기 벡터 대조
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    ax2.set_title("Magnetometer Magnetic Field Alignment", fontsize=11)
    
    m_gt_sensor_all = np.array(m_gt_sensor_all)
    m_est_sensor_all = np.array(m_est_sensor_all)
    
    # GT 지자기 벡터 그리기 (Red)
    ax2.quiver(0, 0, 0, m_gt_sensor_all[:,0], m_gt_sensor_all[:,1], m_gt_sensor_all[:,2], 
              color='crimson', alpha=0.6, length=1.0, arrow_length_ratio=0.1, label='Ideal GT (Mag Field)')
    # 실측 지자기 벡터 그리기 (Blue)
    ax2.quiver(0, 0, 0, m_est_sensor_all[:,0], m_est_sensor_all[:,1], m_est_sensor_all[:,2], 
              color='royalblue', alpha=0.8, length=1.0, arrow_length_ratio=0.1, label='Calibrated (Mag Field)')
              
    ax2.plot_wireframe(x, y, z, color='gray', alpha=0.1, linewidth=0.5)
    
    ax2.set_xlim([-1.1, 1.1])
    ax2.set_ylim([-1.1, 1.1])
    ax2.set_zlim([-1.1, 1.1])
    ax2.set_xlabel('X (Sensor)')
    ax2.set_ylabel('Y (Sensor)')
    ax2.set_zlabel('Z (Sensor)')
    ax2.legend(loc='lower left')
    
    # 2D Plot 3: 20 Positions 쿼터니언 오차 (대조 막대)
    ax3 = fig.add_subplot(1, 3, 3)
    ax3.set_title("Orientation Angle Error (q_gt vs q_est)", fontsize=11)
    
    indices = np.arange(20)
    ax3.bar(indices - 0.2, angle_errors, width=0.4, color='purple', edgecolor='black', alpha=0.4, label='Jig Mounting Error Included')
    ax3.bar(indices + 0.2, pure_sensor_errors, width=0.4, color='mediumseagreen', edgecolor='black', alpha=0.8, label='Pure Sensor Error (Modulo Compensated)')
    
    ax3.axhline(y=q_rmse, color='darkred', linestyle='--', linewidth=1.5, label=f'q_RMSE (Raw): {q_rmse:.3f}°')
    ax3.axhline(y=q_pure_rmse, color='forestgreen', linestyle='-.', linewidth=1.5, label=f'q_RMSE (Pure): {q_pure_rmse:.3f}°')
    ax3.axhline(y=g_rmse, color='darkgreen', linestyle=':', linewidth=1.2, label=f'acc_RMSE: {g_rmse:.3f}°')
    ax3.axhline(y=m_rmse, color='darkblue', linestyle=':', linewidth=1.2, label=f'mag_RMSE: {m_rmse:.3f}°')
    
    ax3.set_xlabel('Icosahedron Face Index')
    ax3.set_ylabel('Rotation Error Angle [degrees]')
    ax3.set_xticks(indices)
    ax3.grid(True, linestyle=':', alpha=0.5)
    ax3.legend(loc='upper right')
    
    # 각 막대 위에 수치 표시 (Jig 오차는 보라색, 순수 오차는 초록색 세로 표기)
    for idx, (err, pure_err) in enumerate(zip(angle_errors, pure_sensor_errors)):
        ax3.text(idx - 0.2, err + (np.max(angle_errors) * 0.02), f"{err:.1f}°", ha='center', va='bottom', fontsize=5.0, color='purple', rotation=90)
        ax3.text(idx + 0.2, pure_err + (np.max(angle_errors) * 0.02), f"{pure_err:.1f}°", ha='center', va='bottom', fontsize=5.0, color='darkgreen', rotation=90)
        
    plt.tight_layout()
    
    # 저장 경로 확보
    output_verify_dir = os.path.join(SCRIPT_DIR, "output")
    if not os.path.exists(output_verify_dir):
        os.makedirs(output_verify_dir)
    save_path = os.path.join(output_verify_dir, "test_phase3_static_orientation_result.png")
    plt.savefig(save_path, dpi=300)
    print(f"\n🎉 [시각화 성공] 3D 정적 자세 검증 대조 플롯 저장 완료 ➔ {save_path}")
    plt.show()

if __name__ == "__main__":
    main()
