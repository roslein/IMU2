"""
Real-world IMU Phase 2 Verification & 3D Sphere Plotter (test_phase2.py)
목적: 수집된 20개 포지션의 raw 가속도/자력 데이터를 기반으로
      accel_calibration.py 및 mag_calibration.py에서 산출된
      보정 파라미터(W, b)를 적용하여 보정 전/후 3D 구면 분포 격차를
      Matplotlib 3D Scatter와 정밀 Wireframe 구면으로 극적이게 가시화하여 저장합니다.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

# 로컬 모듈 탐색 경로 추가 (calibration_tool 내부 import 허용)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, '..'))
sys.path.append(os.path.join(SCRIPT_DIR, '..', 'calibration_tool'))

def main():
    # Windows CP949 콘솔 이모지 인코딩 충돌 방지 강제 설정
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
        
    print("=" * 60)
    print(" 🎯 Real-world IMU Phase 2 3D Calibration Visualizer")
    print("=" * 60)
    
    # 1. 데이터 로드 및 데모 모드 스위칭
    imu_root = os.path.dirname(SCRIPT_DIR)
    data_path = os.path.join(imu_root, "calibration_tool", "output", "collected_data.npz")
    acc_param_path = os.path.join(imu_root, "calibration_tool", "output", "acc_params.npz")
    
    if not os.path.exists(data_path):
        print("⚠️ 실측 수집 데이터(.npz)를 감지하지 못했습니다.")
        print("💡 [데모 모드 가동] 시뮬레이션 왜곡 가상 데이터 세트를 자동 생성하여 3D 시각화 구동합니다...\n")
        
        # 1. 20면체 법선 벡터 구하기
        import icosahedron
        normals = icosahedron.get_icosahedron_normals()
        
        # 2. 가속도계 가상 왜곡 데이터 생성 (Scale 팩터, 비직교성, Bias 오염 주입)
        W_acc_true = np.array([
            [0.88, 0.04, -0.03],
            [0.02, 1.12, 0.05],
            [-0.01, -0.02, 0.96]
        ])
        b_acc_true = np.array([150.0, -220.0, 80.0])
        M_acc_true = np.linalg.inv(W_acc_true)
        
        # 가속도 raw 데이터 생성 (1g = 1000 cnt 가정 및 노이즈 가산)
        acc_raw = []
        np.random.seed(42)
        for n in normals:
            g_vec = -n * 1000.0  # 중력은 아래 방향
            raw = (M_acc_true @ g_vec) + b_acc_true + np.random.normal(0.0, 5.0, 3)
            acc_raw.append(raw)
        acc_raw = np.array(acc_raw)
        
        # 3. 자력계 가상 왜곡 데이터 생성 (찌그러진 타원체 왜곡 및 Hard-iron 편차 주입)
        W_mag_true = np.array([
            [1.35, 0.08, -0.06],
            [0.08, 0.85, 0.12],
            [-0.06, 0.12, 1.05]
        ])
        b_mag_true = np.array([3000.0, -4000.0, 2000.0])
        M_mag_true = np.linalg.inv(W_mag_true)
        
        # 자력 raw 데이터 생성
        mag_raw = []
        for n in normals:
            raw = (M_mag_true @ (n * 10000.0)) + b_mag_true + np.random.normal(0.0, 100.0, 3)
            mag_raw.append(raw)
        mag_raw = np.array(mag_raw)
        
        # 4. 실시간 가속도계 12-parameter(9-param 기하) 솔버 구동
        import accel_calibration
        W_acc, b_acc, _, _ = accel_calibration.calibrate_acc_12param_icosahedron(acc_raw, normals)
        print("✅ 가속도계 9-Parameter 기하 솔버 시뮬레이션 피팅 완료.")
    else:
        data = np.load(data_path)
        acc_raw = data["acc"]
        mag_raw = data["mag"]
        
        # 2. 보정 파라미터 로드
        if os.path.exists(acc_param_path):
            acc_params = np.load(acc_param_path)
            W_acc = acc_params["W"]
            b_acc = acc_params["b"]
            print("✅ 가속도계 실측 파라미터 로드 완료.")
        else:
            W_acc = np.eye(3)
            b_acc = np.zeros(3)
            print("⚠️ 가속도계 보정 파라미터가 없습니다. 미보정 상태로 시각화합니다.")
        
    # 자력계 보정 파라미터 로드 (mag_calibration을 돌려 output/calib_params.h 또는 임시 저장된 mag W, b가 있는지 확인)
    # mag_calibration 결과를 대수적으로 긁어오기 위해 mag_calibration.py 가 산출한 W, b를 구함
    # mag_calibration.py에서 save 기능을 사용하지 않았었으므로, 여기서 mag 9-parameter 피팅을 직접 1회 수행해 파라미터 획득!
    print("🧠 자력계 9-Parameter 대칭 타원체 피팅 실시간 동시 연산 중...")
    from scipy.optimize import least_squares
    
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
    avg_radius = np.mean(np.linalg.norm(mag_raw - mean_mag, axis=1))
    init_scale = 1.0 / avg_radius
    p0 = np.concatenate([mean_mag, [init_scale, 0.0, 0.0, init_scale, 0.0, init_scale]])
    
    res = least_squares(residuals, p0, args=(mag_raw,), method='lm')
    b_mag = res.x[:3]
    W_mag = np.array([
        [res.x[3], res.x[4], res.x[5]],
        [res.x[4], res.x[6], res.x[7]],
        [res.x[5], res.x[7], res.x[8]]
    ])
    print("✅ 자력계 파라미터 실시간 피팅 완료.")
    
    # 3. 보정 연산 대입
    acc_cal = (W_acc @ (acc_raw - b_acc).T).T
    mag_cal = (W_mag @ (mag_raw - b_mag).T).T
    
    # 4. Matplotlib 3D 대조 플로팅 시동
    fig = plt.figure(figsize=(16, 7))
    fig.suptitle("📌 IMU Phase 2 3D Calibration Sphere Fitting Result", fontsize=16, fontweight='bold')
    
    # 이상적 3D 구면 메쉬(Wireframe) 생성을 위한 데이터
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    x_sphere = np.outer(np.cos(u), np.sin(v))
    y_sphere = np.outer(np.sin(u), np.sin(v))
    z_sphere = np.outer(np.ones(np.size(u)), np.cos(v))
    
    # ----------------------------------------------------
    # Plot 1: 가속도계 3D 구면 대조 (Before vs After)
    # ----------------------------------------------------
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.set_title("Accelerometer Calibration (20-Positions)", fontsize=14)
    
    # 보정 전 찌그러진 분포 (raw 카운트 기준이므로 1000배 축소하여 플로팅)
    ax1.scatter(acc_raw[:,0]/1000.0, acc_raw[:,1]/1000.0, acc_raw[:,2]/1000.0, 
               color='red', s=60, alpha=0.8, edgecolors='black', label='Before (Raw Counts / 1000)')
               
    # 보정 후 정합 구면 분포
    ax1.scatter(acc_cal[:,0], acc_cal[:,1], acc_cal[:,2], 
               color='green', s=80, alpha=0.9, edgecolors='black', label='After (Calibrated 1g)')
               
    # 기준 1g 구면 렌더링
    ax1.plot_wireframe(x_sphere, y_sphere, z_sphere, color='gray', alpha=0.15, linewidth=0.5)
    
    ax1.set_xlabel('X Axis')
    ax1.set_ylabel('Y Axis')
    ax1.set_zlabel('Z Axis')
    ax1.set_xlim([-1.5, 1.5])
    ax1.set_ylim([-1.5, 1.5])
    ax1.set_zlim([-1.5, 1.5])
    ax1.legend(loc='upper right')
    ax1.grid(True)
    
    # ----------------------------------------------------
    # Plot 2: 자력계 3D 구면 대조 (Before vs After)
    # ----------------------------------------------------
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    ax2.set_title("Magnetometer Ellipsoid Sphere Fitting", fontsize=14)
    
    # 보정 전 찌그러진 타원체 및 오프셋 치우침 분포 (자력계 카운트 기준 10000배 축소하여 플로팅)
    ax2.scatter(mag_raw[:,0]/10000.0, mag_raw[:,1]/10000.0, mag_raw[:,2]/10000.0, 
               color='orange', s=60, alpha=0.8, edgecolors='black', label='Before (Raw Cnts / 10000)')
               
    # 보정 후 이상적 1.0 구면 분포
    ax2.scatter(mag_cal[:,0], mag_cal[:,1], mag_cal[:,2], 
               color='blue', s=80, alpha=0.9, edgecolors='black', label='After (Calibrated Norm=1.0)')
               
    # 기준 1.0 구면 렌더링
    ax2.plot_wireframe(x_sphere, y_sphere, z_sphere, color='gray', alpha=0.15, linewidth=0.5)
    
    ax2.set_xlabel('X Axis')
    ax2.set_ylabel('Y Axis')
    ax2.set_zlabel('Z Axis')
    ax2.set_xlim([-1.8, 1.8])
    ax2.set_ylim([-1.8, 1.8])
    ax2.set_zlim([-1.8, 1.8])
    ax2.legend(loc='upper right')
    ax2.grid(True)
    
    plt.tight_layout()
    
    # 이미지 파일 저장
    output_filename = os.path.join(SCRIPT_DIR, "test_phase2_result.png")
    plt.savefig(output_filename, dpi=300)
    print(f"\n🎉 [시각화 성공] 3D 구면 대조 분석 플롯이 안전하게 저장되었습니다!")
    print(f"   ↳ 저장 위치: {output_filename}")
    plt.show()

if __name__ == "__main__":
    main()
