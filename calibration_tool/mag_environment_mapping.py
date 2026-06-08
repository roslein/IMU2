"""
Real-world IMU Magnetometer Environment Mapping (mag_environment_mapping.py)
목적: 20 Positions 가속도/자력 데이터를 로드하고 보정 파라미터를 적용한 뒤,
      SVD 최소 회전(Shortest Arc) 기하 정합을 통해 현재 실험실 공간 고유의 
      3D 지구 지자기 레퍼런스 벡터(왜곡 복각 반영)를 역산하여 저장합니다.
"""

import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

# 로컬 모듈 탐색 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMU_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.append(IMU_ROOT)
sys.path.append(os.path.join(IMU_ROOT, '..', 'imu_simulation'))

import icosahedron
from utils.quaternion_math import q_mult, q_conj

def compute_theoretical_gt_quaternions(normals_jig, R_mount):
    R_sensor_to_ned_lut = []
    for i in range(20):
        n_jig = normals_jig[i]
        res_rot, _ = R_scipy.align_vectors(np.array([[0.0, 0.0, 1.0]]), np.array([n_jig]))
        R_jig_to_ned = res_rot.as_matrix()
        R_sensor_to_ned = R_jig_to_ned @ R_mount.T
        R_sensor_to_ned_lut.append(R_sensor_to_ned)
    return R_sensor_to_ned_lut

def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
        
    print("=" * 60)
    print(" 📡 Magnetometer Environment Mapping Solver")
    print("=" * 60)
    
    # 1. 데이터 로드 및 보정 적용
    raw_data_path = os.path.join(SCRIPT_DIR, "output", "collected_data.npz")
    acc_param_path = os.path.join(SCRIPT_DIR, "output", "acc_params.npz")
    mag_param_path = os.path.join(SCRIPT_DIR, "output", "mag_params.npz")
    
    if not (os.path.exists(raw_data_path) and os.path.exists(acc_param_path) and os.path.exists(mag_param_path)):
        print("❌ 매핑을 수행할 데이터 또는 보정 파라미터가 누락되었습니다.")
        sys.exit(1)
        
    raw_data = np.load(raw_data_path)
    acc_raw = raw_data["acc"]
    mag_raw = raw_data["mag"]
    
    acc_params = np.load(acc_param_path)
    W_acc = acc_params["W"]
    b_acc = acc_params["b"]
    
    mag_params = np.load(mag_param_path)
    W_mag = mag_params["W"]
    b_mag = mag_params["b"]
    
    # 보정 적용
    acc_cal = (W_acc @ (acc_raw - b_acc).T).T
    mag_cal = (W_mag @ (mag_raw - b_mag).T).T
    
    # 2. 기하 툴 파라미터 및 R_mount 로드
    normals_jig = icosahedron.get_icosahedron_normals()
    R_mount = icosahedron.get_jig_to_sensor_rotation()
    
    # 3. 20개 포지션별 GT 회전행렬 사전 도출
    R_sensor_to_ned_lut = compute_theoretical_gt_quaternions(normals_jig, R_mount)
    
    # 4. 가속도 부호 반전(-acc_cal)을 활용한 면 매칭 인덱스 도출
    rot_normals = icosahedron.get_rotated_normals()
    best_indices = []
    for i in range(20):
        best_idx, _ = icosahedron.match_face(-acc_cal[i], rot_normals)
        best_indices.append(best_idx)
        
    # 5. 실측 mag 데이터를 GT 회전으로 NED 프레임에 역투영하여 3D 자북 레퍼런스 평균 도출
    m_ned_list = []
    for i in range(20):
        best_idx = best_indices[i]
        m_ned = R_sensor_to_ned_lut[best_idx] @ mag_cal[i]
        m_ned_list.append(m_ned)
    m_ned_mean = np.mean(m_ned_list, axis=0)
    m_ned_ref = m_ned_mean / np.linalg.norm(m_ned_mean)
    
    # 복각 역산
    inclination = np.degrees(np.arcsin(m_ned_ref[2]))
    
    print("\n⚖️ [환경 지자기 레퍼런스 도출 성공]")
    print(f"   ↳ 3D 자북 레퍼런스 (m_ned_ref): {m_ned_ref}")
    print(f"   ↳ 계산된 고유 환경 복각 (Inclination): {inclination:.4f}°")
    
    # 6. 환경 파일 저장
    output_dir = os.path.join(SCRIPT_DIR, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    save_path = os.path.join(output_dir, "env_params.npz")
    np.savez(save_path, m_ned_ref=m_ned_ref, inclination=inclination)
    print(f"💾 환경 파라미터가 저장되었습니다: {save_path}\n")

if __name__ == "__main__":
    main()
