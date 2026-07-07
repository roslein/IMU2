import os
import matplotlib.pyplot as plt
import numpy as np
from imu_core.interface import IMUVisualizer

class MatplotlibIcosahedronVisualizer(IMUVisualizer):
    def __init__(self):
        self.fig = None
        self.ax = None

    def init_plot(self) -> None:
        """대화형 3D matplotlib 창을 기동"""
        plt.ion()
        self.fig = plt.figure(figsize=(8, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.show()

    def update_plot(self, collected_faces: set, normals: np.ndarray, new_point: np.ndarray = None, new_matched_idx: int = None, expected_face_idx: int = None) -> None:
        """실시간 20면체 거치 가이드를 드로잉 (이모지 배제)"""
        if self.fig is None or self.ax is None:
            return

        self.ax.clear()
        
        # 1. 3D 구면 그리드 (Sphere Wireframe) 시각화
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 10)
        x = np.outer(np.cos(u), np.sin(v))
        y = np.outer(np.sin(u), np.sin(v))
        z = np.outer(np.ones(np.size(u)), np.cos(v))
        self.ax.plot_wireframe(x, y, z, color='gray', alpha=0.1, linewidth=0.5)

        # 2. 20개 입체 안착 법선 그리기
        for idx, n in enumerate(normals):
            if idx in collected_faces:
                # 수집이 완료된 면 (Green)
                self.ax.quiver(0, 0, 0, n[0], n[1], n[2], color='forestgreen', alpha=0.8, length=0.9, arrow_length_ratio=0.15, linewidth=2.0)
                self.ax.text(n[0]*1.05, n[1]*1.05, n[2]*1.05, f"{idx}", color='darkgreen', fontsize=9, fontweight='bold')
            else:
                # 미수집 대상 면 (Red)
                self.ax.quiver(0, 0, 0, n[0], n[1], n[2], color='crimson', alpha=0.3, length=0.8, arrow_length_ratio=0.1, linewidth=1.0)
                self.ax.text(n[0]*1.05, n[1]*1.05, n[2]*1.05, f"{idx}", color='maroon', fontsize=8, alpha=0.6)

        # 3. 실시간 가속도 측정 벡터 가이드 (Yellow Star)
        if new_point is not None:
            norm_val = np.linalg.norm(new_point)
            p_unit = new_point / (norm_val if norm_val > 0 else 1.0)
            # 가속도 방향과 구면 반전 매칭 지점
            match_pt = -p_unit
            self.ax.scatter(match_pt[0], match_pt[1], match_pt[2], color='gold', s=150, marker='*', edgecolor='black', zorder=10, label='Live Accelerometer')
            self.ax.quiver(0, 0, 0, match_pt[0], match_pt[1], match_pt[2], color='gold', length=0.95, arrow_length_ratio=0.15, linewidth=2.5, zorder=9)

        title_str = f"Live 3D Icosahedron Guide (Collected: {len(collected_faces)}/20)"
        if expected_face_idx is not None:
            title_str += f" | Expected Target: #{expected_face_idx}"
        if new_matched_idx is not None:
            title_str += f" | Matched: #{new_matched_idx}"
            
        self.ax.set_title(title_str, fontsize=11, fontweight='bold')
        self.ax.set_xlim([-1.2, 1.2])
        self.ax.set_ylim([-1.2, 1.2])
        self.ax.set_zlim([-1.2, 1.2])
        self.ax.set_xlabel('X (Sensor)')
        self.ax.set_ylabel('Y (Sensor)')
        self.ax.set_zlabel('Z (Sensor)')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def plot_trajectory_comparison(time_seq: np.ndarray, integrated_yaw: np.ndarray, gt_yaw: np.ndarray, output_path: str) -> None:
    """1D 수평 회전판 자이로 적분 궤적 대조 플롯 저장 (이모지 배제)"""
    # 디렉토리 확인 및 생성
    dir_name = os.path.dirname(output_path)
    if dir_name and not os.path.exists(dir_name):
        os.makedirs(dir_name)

    plt.figure(figsize=(10, 5))
    plt.plot(time_seq, gt_yaw, 'r--', label='True Ground Truth (Protractor)', linewidth=2.0)
    plt.plot(time_seq, integrated_yaw, 'b-', label='Integrated Z-Gyro Trajectory (Calibrated)', linewidth=1.5)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.title("IMU Gyroscope 1D Integrator Trajectory Verification", fontsize=12, fontweight='bold')
    plt.xlabel("Time [seconds]")
    plt.ylabel("Yaw Angle [degrees]")
    plt.legend(loc='upper right')
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def plot_3d_quiver_comparison(
    normals_ideal: np.ndarray,
    normals_est: np.ndarray,
    mag_ideal: np.ndarray,
    mag_est: np.ndarray,
    angle_errors: np.ndarray,
    pure_sensor_errors: np.ndarray,
    q_rmse: float,
    q_pure_rmse: float,
    g_rmse: float,
    m_rmse: float,
    output_path: str
) -> None:
    """20 Pos 정적 벡터 정합 quiver 및 쿼터니언 오차 막대 차트 병렬 렌더링 (이모지 배제)"""
    dir_name = os.path.dirname(output_path)
    if dir_name and not os.path.exists(dir_name):
        os.makedirs(dir_name)

    fig = plt.figure(figsize=(16, 8))
    fig.suptitle("IMU Phase 3.1 3D Static Orientation & Vector Alignment Verification", fontsize=15, fontweight='bold')

    # Plot 1: 중력 벡터 대조
    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    ax1.set_title("Accelerometer Gravity Alignment (20 Pos)", fontsize=11)
    ax1.quiver(0, 0, 0, normals_ideal[:,0], normals_ideal[:,1], normals_ideal[:,2], 
               color='crimson', alpha=0.6, length=1.0, arrow_length_ratio=0.1, label='Ideal GT (Gravity)')
    ax1.quiver(0, 0, 0, normals_est[:,0], normals_est[:,1], normals_est[:,2], 
               color='royalblue', alpha=0.8, length=1.0, arrow_length_ratio=0.1, label='Calibrated (Gravity)')
    
    # 구면 와이어프레임
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

    # Plot 2: 지자기 벡터 대조
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    ax2.set_title("Magnetometer Magnetic Field Alignment", fontsize=11)
    ax2.quiver(0, 0, 0, mag_ideal[:,0], mag_ideal[:,1], mag_ideal[:,2], 
               color='crimson', alpha=0.6, length=1.0, arrow_length_ratio=0.1, label='Ideal GT (Mag Field)')
    ax2.quiver(0, 0, 0, mag_est[:,0], mag_est[:,1], mag_est[:,2], 
               color='royalblue', alpha=0.8, length=1.0, arrow_length_ratio=0.1, label='Calibrated (Mag Field)')
    ax2.plot_wireframe(x, y, z, color='gray', alpha=0.1, linewidth=0.5)
    
    ax2.set_xlim([-1.1, 1.1])
    ax2.set_ylim([-1.1, 1.1])
    ax2.set_zlim([-1.1, 1.1])
    ax2.set_xlabel('X (Sensor)')
    ax2.set_ylabel('Y (Sensor)')
    ax2.set_zlabel('Z (Sensor)')
    ax2.legend(loc='lower left')

    # Plot 3: 쿼터니언 자세 에러 바 차트
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
    
    # 텍스트 라벨 추가 (세로 표기)
    max_err = max(np.max(angle_errors), 1.0)
    for idx, (err, pure_err) in enumerate(zip(angle_errors, pure_sensor_errors)):
        ax3.text(idx - 0.2, err + (max_err * 0.02), f"{err:.1f}°", ha='center', va='bottom', fontsize=5.5, color='purple', rotation=90)
        ax3.text(idx + 0.2, pure_err + (max_err * 0.02), f"{pure_err:.1f}°", ha='center', va='bottom', fontsize=5.5, color='darkgreen', rotation=90)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
