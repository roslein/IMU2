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
sys.path.append(os.path.join(IMU_ROOT, 'calibration_tool'))
sys.path.append(os.path.join(IMU_ROOT, '..', 'imu_simulation'))

import icosahedron
from utils.quaternion_math import q_angle_error, accel_mag_to_quaternion, quat_to_euler

def compute_theoretical_gt_quaternions(normals_jig, R_mount):
    """
    정20면체 지그 ideal 법선 벡터와 마운팅 회전 행렬을 결합하여,
    20개 안착 포지션에 매핑되는 센서 기준 이론적 GT 쿼터니언 및 회전 행렬 LUT를 생성합니다.
    """
    q_gt_lut = []
    R_sensor_to_ned_lut = []
    
    for i in range(20):
        n_jig = normals_jig[i]
        
        # Z축 n_jig와 외적하여 정규 직교 기저를 이룰 ideal Y 기저 정의
        y_axis = np.array([0.0, 1.0, 0.0])
        if abs(np.dot(n_jig, y_axis)) > 0.99:
            y_axis = np.array([1.0, 0.0, 0.0])
        
        x_jig = np.cross(y_axis, n_jig)
        x_jig /= np.linalg.norm(x_jig)
        y_jig = np.cross(n_jig, x_jig)
        
        # 지그 프레임에서 지구 고정 NED 프레임으로의 ideal 정합 회전
        R_jig_to_ned = np.column_stack((x_jig, y_jig, n_jig)).T
        
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
    normals_jig = icosahedron.get_icosahedron_normals()
    R_mount = icosahedron.get_jig_to_sensor_rotation()
    
    # 3. 20개 포지션별 GT 쿼터니언 및 회전행렬 사전 도출
    q_gt_lut, R_sensor_to_ned_lut = compute_theoretical_gt_quaternions(normals_jig, R_mount)
    print("📊 지그 기하 오차(R_mount) 반영 ideal 3D GT 회전 쿼터니언 산출 완료.")
    
    # 4. 지구 고정 프레임 기준 ideal 지자기 벡터 추출 (Inclination 편향 상쇄 정합)
    rot_normals = icosahedron.get_rotated_normals()
    best_indices = []
    for i in range(20):
        best_idx, _ = icosahedron.match_face(-acc_cal[i], rot_normals)
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
    angle_errors = []
    dot_g_list = []
    dot_m_list = []
    angle_g_errors = []
    angle_m_errors = []
    
    g_gt_sensor_all = []
    g_est_sensor_all = []
    m_gt_sensor_all = []
    m_est_sensor_all = []
    
    for i in range(20):
        # SVD 기반 Wahba 문제 최소제곱 정밀 정합 (특이점/Sign flip 완전 회피)
        v_sensor = np.array([acc_cal[i] / np.linalg.norm(acc_cal[i]), mag_cal[i] / np.linalg.norm(mag_cal[i])])
        v_ned = np.array([np.array([0.0, 0.0, 1.0]), m_ned_ideal])
        res_rot, _ = R_scipy.align_vectors(v_ned, v_sensor)
        q_est_scipy = res_rot.as_quat() # [x, y, z, w]
        q_est = np.array([q_est_scipy[3], q_est_scipy[0], q_est_scipy[1], q_est_scipy[2]]) # [w, x, y, z]
        
        # 쿼터니언 각도 오차 (degree)
        best_idx = best_indices[i]
        err_deg = q_angle_error(q_gt_lut[best_idx], q_est)
        angle_errors.append(err_deg)
        
        # 센서 관점의 GT 중력/지자기 벡터 역산
        R_s2n = R_sensor_to_ned_lut[best_idx]
        # ideal 중력가속도 (NED Down [0, 0, 1]을 센서 좌표계로 역투영)
        g_gt_sensor = R_s2n.T @ np.array([0.0, 0.0, 1.0])
        # ideal 지자기 (지구 ideal 지자기를 센서 좌표계로 역투영)
        m_gt_sensor = R_s2n.T @ m_ned_ideal
        
        # 실측 정규화 벡터
        g_est_sensor = acc_cal[i] / np.linalg.norm(acc_cal[i]) # 이미 중력 가속도(Down) 방향임
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
        print(f"포지션 #{i:02d} (면 #{best_idx:02d}) | 쿼터니언 오차: {err_deg_val:6.3f}° | 중력 내적: {dot_g:8.6f} (오차: {angle_g:6.3f}°) | 지자기 내적: {dot_m:8.6f} (오차: {angle_m:6.3f}°)")
        
    angle_errors = np.array(angle_errors)
    angle_g_errors = np.array(angle_g_errors)
    angle_m_errors = np.array(angle_m_errors)
    
    q_rmse = np.sqrt(np.mean(angle_errors**2))
    g_rmse = np.sqrt(np.mean(angle_g_errors**2))
    m_rmse = np.sqrt(np.mean(angle_m_errors**2))
    
    print("\n" + "=" * 60)
    print(" 🎉 3D Static Orientation Verification 정량 보고서")
    print("=" * 60)
    print(f"📊 3D 쿼터니언 자세 회전각 RMSE: {q_rmse:10.6f}°")
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
              color='royalblue', alpha=0.8, length=0.9, arrow_length_ratio=0.1, label='Calibrated (Gravity)')
              
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
              color='royalblue', alpha=0.8, length=0.9, arrow_length_ratio=0.1, label='Calibrated (Mag Field)')
              
    ax2.plot_wireframe(x, y, z, color='gray', alpha=0.1, linewidth=0.5)
    
    ax2.set_xlim([-1.1, 1.1])
    ax2.set_ylim([-1.1, 1.1])
    ax2.set_zlim([-1.1, 1.1])
    ax2.set_xlabel('X (Sensor)')
    ax2.set_ylabel('Y (Sensor)')
    ax2.set_zlabel('Z (Sensor)')
    ax2.legend(loc='lower left')
    
    # 2D Plot 3: 20 Positions 쿼터니언 오차
    ax3 = fig.add_subplot(1, 3, 3)
    ax3.set_title("Orientation Angle Error (q_gt vs q_est)", fontsize=11)
    
    indices = np.arange(20)
    ax3.bar(indices, angle_errors, color='purple', edgecolor='black', alpha=0.7, label='Quaternion Angle Error')
    ax3.axhline(y=q_rmse, color='darkred', linestyle='--', linewidth=1.5, label=f'q_RMSE: {q_rmse:.4f}°')
    ax3.axhline(y=g_rmse, color='darkgreen', linestyle=':', linewidth=1.2, label=f'acc_RMSE: {g_rmse:.4f}°')
    ax3.axhline(y=m_rmse, color='darkblue', linestyle=':', linewidth=1.2, label=f'mag_RMSE: {m_rmse:.4f}°')
    
    ax3.set_xlabel('Icosahedron Face Index')
    ax3.set_ylabel('Rotation Error Angle [degrees]')
    ax3.set_xticks(indices)
    ax3.grid(True, linestyle=':', alpha=0.5)
    ax3.legend(loc='upper right')
    
    # 각 막대 위에 수치 표시 (겹침 방지를 위한 90도 세로 회전 및 폰트 축소 적용)
    for idx, err in enumerate(angle_errors):
        ax3.text(idx, err + (np.max(angle_errors) * 0.02), f"{err:.2f}°", ha='center', va='bottom', fontsize=5.5, color='purple', rotation=90)
        
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
