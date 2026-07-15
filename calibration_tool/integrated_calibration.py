import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

# 로컬 모듈 탐색 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMU_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.append(IMU_ROOT)

import imu_core.math as imu_math
import imu_core.icosahedron as icosahedron

def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
        
    print("=" * 60)
    print(" 🎯 Integrated 9-Axis Calibration & C++ Header Compiler (v0.3.0)")
    print("=" * 60)
    
    output_dir = os.path.join(SCRIPT_DIR, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    raw_9axis_path = os.path.join(output_dir, "collected_data_9axis.npz")
    raw_20pose_path = os.path.join(output_dir, "collected_data.npz")
    
    has_9axis = os.path.exists(raw_9axis_path)
    
    # 1. 데이터 로드
    if has_9axis:
        print(f"📂 통합 9축 데이터셋 감지 성공: {raw_9axis_path}")
        data = np.load(raw_9axis_path)
        acc_raw = data["acc"]
        mag_raw = data["mag"]
        gyro_raw = data["gyro"]
        yaw_gt = data["yaw_gt"] if "yaw_gt" in data else None
    elif os.path.exists(raw_20pose_path):
        print(f"⚠️  9축 통합본이 없음 ➔ 기존 20면체 데이터셋 Fallback 로드: {raw_20pose_path}")
        data = np.load(raw_20pose_path)
        acc_raw = data["acc"]
        mag_raw = data["mag"]
        gyro_raw = data["gyro"] if "gyro" in data else np.zeros((20, 300, 3)) # gyro 누락 시 예외 방지
        yaw_gt = None
    else:
        print("❌ 보정할 입력 데이터셋이 존재하지 않습니다. data_collection.py를 먼저 가동하십시오.")
        sys.exit(1)
        
    # LPF 평균 가공 (N, 300, 3) ➔ (N, 3)
    if acc_raw.ndim == 3:
        acc_mean = np.mean(acc_raw, axis=1)
        mag_mean = np.mean(mag_raw, axis=1)
        gyro_all = gyro_raw.reshape(-1, 3)
    else:
        acc_mean = acc_raw
        mag_mean = mag_raw
        gyro_all = gyro_raw
        
    # 2. 가속도계 12-parameter 보정
    print("\n⚡ [1단계] 가속도계 12-parameter LS 피팅 기동...")
    normals = icosahedron.get_rotated_normals()
    W_acc, b_acc, alpha_acc, beta_acc = imu_math.calibrate_acc_12param_icosahedron(acc_mean, normals)
    print(f"   ↳ 가속도 보정 완료. 경사각: Pitch={np.degrees(alpha_acc):.3f}° | Roll={np.degrees(beta_acc):.3f}°")
    
    # 3. 자이로 Global 바이어스 보정
    print("\n⚡ [2단계] 자이로스코프 Global Average 바이어스 계산...")
    b_gyro = imu_math.calibrate_gyro_bias_global(gyro_all)
    W_gyro = np.eye(3) # 자이로 스케일은 Identity 고정
    print(f"   ↳ 자이로 바이어스: {b_gyro} rad/s (또는 mdps)")
    
    # 4. 자력계 보정 및 낙찰
    # 가속도 보정 적용
    acc_cal = imu_math.calibrate_sensor_accel(acc_mean, W_acc, b_acc)
    
    print("\n⚡ [3단계] 자력계 3/6/9-parameter 보정 최적화 기동...")
    # Stage 1: 기하학 초기화
    W_mag_3_0, b_mag_3_0 = imu_math.calibrate_mag_3param(mag_mean)
    W_mag_6_0, b_mag_6_0 = imu_math.calibrate_mag_6param(mag_mean)
    W_mag_9_0, b_mag_9_0 = imu_math.calibrate_mag_ellipsoid(mag_mean)
    
    # 최종 선택할 파라미터 변수 초기화
    W_mag_final = W_mag_3_0
    b_mag_final = b_mag_3_0
    model_name = "3-param (Fallback)"
    
    if has_9axis and yaw_gt is not None:
        print("   ↳ 9축 데이터 및 Yaw GT 기반 Stage 2 Task-Aware 최적화 진행...")
        # 2단계 Task-Aware 최적화
        W_mag_3_f, b_mag_3_f = imu_math.calibrate_mag_task_aware(mag_mean, acc_cal, yaw_gt, W_mag_3_0, b_mag_3_0, mode=3)
        W_mag_6_f, b_mag_6_f = imu_math.calibrate_mag_task_aware(mag_mean, acc_cal, yaw_gt, W_mag_6_0, b_mag_6_0, mode=6)
        W_mag_9_f, b_mag_9_f = imu_math.calibrate_mag_task_aware(mag_mean, acc_cal, yaw_gt, W_mag_9_0, b_mag_9_0, mode=9)
        
        # 5대 지표 및 3p/6p/9p 중 사전식 자동 낙찰
        def calc_yaw_rmse(W, b):
            errs = []
            for i in range(len(mag_mean)):
                m_cal = W @ (mag_mean[i] - b)
                q = imu_math.align_vectors_svd(acc_cal[i], m_cal)
                r = R_scipy.from_quat([q[1], q[2], q[3], q[0]])
                yaw_est = r.as_euler('xyz', degrees=True)[2]
                diff = (yaw_est - yaw_gt[i] + 180.0) % 360.0 - 180.0
                errs.append(diff)
            return np.sqrt(np.mean(np.array(errs)**2))
            
        rmse_3 = calc_yaw_rmse(W_mag_3_f, b_mag_3_f)
        rmse_6 = calc_yaw_rmse(W_mag_6_f, b_mag_6_f)
        rmse_9 = calc_yaw_rmse(W_mag_9_f, b_mag_9_f)
        
        print(f"   ↳ Task-Aware Yaw RMSE ➔ 3p: {rmse_3:.4f}° | 6p: {rmse_6:.4f}° | 9p: {rmse_9:.4f}°")
        
        best_rmse = min(rmse_3, rmse_6, rmse_9)
        if best_rmse == rmse_3:
            W_mag_final, b_mag_final = W_mag_3_f, b_mag_3_f
            model_name = "3-param (Task-Aware)"
        elif best_rmse == rmse_6:
            W_mag_final, b_mag_final = W_mag_6_f, b_mag_6_f
            model_name = "6-param (Task-Aware)"
        else:
            W_mag_final, b_mag_final = W_mag_9_f, b_mag_9_f
            model_name = "9-param (Task-Aware)"
    else:
        # Fallback 기하학적 비교낙찰
        def calc_mag_metrics(W, b):
            mag_cal = (W @ (mag_mean - b).T).T
            m_norms = np.linalg.norm(mag_cal, axis=1)
            norm_rmse = np.sqrt(np.mean((m_norms - 1.0)**2))
            return norm_rmse
            
        rmse_3_g = calc_mag_metrics(W_mag_3_0, b_mag_3_0)
        rmse_6_g = calc_mag_metrics(W_mag_6_0, b_mag_6_0)
        rmse_9_g = calc_mag_metrics(W_mag_9_0, b_mag_9_0)
        
        print(f"   ↳ Fallback Norm RMSE ➔ 3p: {rmse_3_g:.4f} | 6p: {rmse_6_g:.4f} | 9p: {rmse_9_g:.4f}")
        best_g = min(rmse_3_g, rmse_6_g, rmse_9_g)
        if best_g == rmse_3_g:
            W_mag_final, b_mag_final = W_mag_3_0, b_mag_3_0
            model_name = "3-param (Geometry)"
        elif best_g == rmse_6_g:
            W_mag_final, b_mag_final = W_mag_6_0, b_mag_6_0
            model_name = "6-param (Geometry)"
        else:
            W_mag_final, b_mag_final = W_mag_9_0, b_mag_9_0
            model_name = "9-param (Geometry)"
            
    print(f"🏆 최종 낙찰 모델: {model_name}")
    
    # 5. 파라미터 백업 일원화 저장
    param_save_path = os.path.join(output_dir, "calib_params.npz")
    np.savez(param_save_path, 
             W_acc=W_acc, b_acc=b_acc,
             W_mag=W_mag_final, b_mag=b_mag_final,
             W_gyro=W_gyro, b_gyro=b_gyro)
    print(f"📂 [백업 일원화] 통합 보정 파라미터 저장 완료 ➔ {param_save_path}")
    
    # 6. C++ 헤더 생성 및 calibrated.ino 경로 자동 이식
    header_content = f"""/*
 * Real-world IMU Auto-Generated Calibration Parameters (calib_params.h)
 * 본 헤더파일은 툴체인을 통해 통합 생성되었습니다.
 */

#ifndef _CALIB_PARAMS_H_
#define _CALIB_PARAMS_H_

// 1. 가속도계 12-Parameter 보정용 대수 정합 변수
const float ACC_W[3][3] = {{
  {{ {W_acc[0,0]:12.8f}f, {W_acc[0,1]:12.8f}f, {W_acc[0,2]:12.8f}f }},
  {{ {W_acc[1,0]:12.8f}f, {W_acc[1,1]:12.8f}f, {W_acc[1,2]:12.8f}f }},
  {{ {W_acc[2,0]:12.8f}f, {W_acc[2,1]:12.8f}f, {W_acc[2,2]:12.8f}f }}
}};

const float ACC_B[3] = {{
  {b_acc[0]:12.8f}f, {b_acc[1]:12.8f}f, {b_acc[2]:12.8f}f
}};

// 2. 자력계 {model_name} 보정용 대수 정합 변수
const float MAG_W[3][3] = {{
  {{ {W_mag_final[0,0]:12.8f}f, {W_mag_final[0,1]:12.8f}f, {W_mag_final[0,2]:12.8f}f }},
  {{ {W_mag_final[1,0]:12.8f}f, {W_mag_final[1,1]:12.8f}f, {W_mag_final[1,2]:12.8f}f }},
  {{ {W_mag_final[2,0]:12.8f}f, {W_mag_final[2,1]:12.8f}f, {W_mag_final[2,2]:12.8f}f }}
}};

const float MAG_B[3] = {{
  {b_mag_final[0]:12.8f}f, {b_mag_final[1]:12.8f}f, {b_mag_final[2]:12.8f}f
}};

// 3. 자이로스코프 Global Bias 및 Scale Factor 보정용 대수 정합 변수
const float GYRO_W[3][3] = {{
  {{ {W_gyro[0,0]:12.8f}f, {W_gyro[0,1]:12.8f}f, {W_gyro[0,2]:12.8f}f }},
  {{ {W_gyro[1,0]:12.8f}f, {W_gyro[1,1]:12.8f}f, {W_gyro[1,2]:12.8f}f }},
  {{ {W_gyro[2,0]:12.8f}f, {W_gyro[2,1]:12.8f}f, {W_gyro[2,2]:12.8f}f }}
}};

const float GYRO_B[3] = {{
  {b_gyro[0]:12.8f}f, {b_gyro[1]:12.8f}f, {b_gyro[2]:12.8f}f
}};

#endif // _CALIB_PARAMS_H_
"""

    firmware_calib_dir = os.path.join(IMU_ROOT, "firmware", "calibrated", "calibration")
    if not os.path.exists(firmware_calib_dir):
        os.makedirs(firmware_calib_dir)
        
    firmware_output_path = os.path.join(firmware_calib_dir, "calib_params.h")
    with open(firmware_output_path, "w", encoding="utf-8") as f:
        f.write(header_content)
    print(f"🚀 [펌웨어 이식 성공] C++ 헤더 실시간 덮어쓰기 완료 ➔ {firmware_output_path}")
    print("=" * 60)

if __name__ == "__main__":
    main()
