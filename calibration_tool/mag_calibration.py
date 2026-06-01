"""
Real-world IMU Phase 2 Magnetometer Calibration (mag_calibration.py)
목적: 20면체 정지 평균화로 획득된 20개 노이즈 감쇄 지자기 데이터를 기반으로,
      Soft/Hard Iron 왜곡을 완벽 제거하는 9-parameter 대칭 타원체 피팅(Ellipsoid Fitting)을 구동하고,
      최종 가속도/자력 보정 파라미터를 펌웨어용 calib_params.h 파일로 빌드/저장합니다.
"""

import numpy as np
from scipy.optimize import least_squares
import os

def calibrate_mag_ellipsoid(mag_raw):
    """
    비선형 최소제곱법(Levenberg-Marquardt) 기반
    대칭 Soft-Iron 보정 행렬 W_mag (6개 변수) 및 Hard-Iron 오프셋 b_mag (3개 변수) 동시 피팅 솔버
    """
    n_points = len(mag_raw)
    
    # 목적 함수 정의: 보정된 지자기 벡터의 크기가 1.0(정규화)에 수렴하도록 유도
    def residuals(p, d):
        b = p[:3]
        # 6개의 변수만으로 대칭 행렬 W 강제 조립 (회전 오차 및 왜곡 축 분리)
        W = np.array([
            [p[3], p[4], p[5]],
            [p[4], p[6], p[7]],
            [p[5], p[7], p[8]]
        ])
        cal = (W @ (d - b).T).T
        return np.linalg.norm(cal, axis=1) - 1.0
        
    # 초기화 파라미터 구성: Hard-iron bias (3축 평균) + Soft-iron 대칭 성분 [1, 0, 0, 1, 0, 1]
    # 지자기 raw 데이터의 크기가 약 10,000 수준이므로, 초기 Scale Factor를 크기에 맞춰 축소
    mean_mag = np.mean(mag_raw, axis=0)
    mag_norms = np.linalg.norm(mag_raw - mean_mag, axis=1)
    avg_radius = np.mean(mag_norms) if np.mean(mag_norms) > 0 else 1.0
    init_scale = 1.0 / avg_radius
    
    p0 = np.concatenate([
        mean_mag, 
        [init_scale, 0.0, 0.0, init_scale, 0.0, init_scale]
    ])
    
    print("\n⚡ 자력계 9-Parameter 대칭 타구체 피팅(Ellipsoid Fitting) 솔버 구동...")
    res = least_squares(residuals, p0, args=(mag_raw,), method='lm')
    
    p = res.x
    b_est = p[:3]
    W_est = np.array([
        [p[3], p[4], p[5]],
        [p[4], p[6], p[7]],
        [p[5], p[7], p[8]]
    ])
    
    return W_est, b_est

def generate_cpp_header(acc_W, acc_b, mag_W, mag_b):
    """
    보정 완료된 12-parameter 가속도 및 9-parameter 자력계 파라미터를 
    펌웨어 프로젝트용 calib_params.h 헤더 파일로 컴파일 출력합니다.
    """
    header_content = f"""/*
 * Real-world IMU Auto-Generated Calibration Parameters (calib_params.h)
 * 생성일: 2026-06-01
 * 본 헤더파일을 복사하여 firmware/calibration/calib_params.h 경로에 이식하십시오.
 */

#ifndef _CALIB_PARAMS_H_
#define _CALIB_PARAMS_H_

// 1. 가속도계 12-Parameter 보정용 대수 정합 변수
const float ACC_W[3][3] = {{
  {{ {acc_W[0,0]:12.8f}f, {acc_W[0,1]:12.8f}f, {acc_W[0,2]:12.8f}f }},
  {{ {acc_W[1,0]:12.8f}f, {acc_W[1,1]:12.8f}f, {acc_W[1,2]:12.8f}f }},
  {{ {acc_W[2,0]:12.8f}f, {acc_W[2,1]:12.8f}f, {acc_W[2,2]:12.8f}f }}
}};

const float ACC_B[3] = {{
  {acc_b[0]:12.8f}f, {acc_b[1]:12.8f}f, {acc_b[2]:12.8f}f
}};

// 2. 자력계 9-Parameter 타원체 피팅(Soft/Hard Iron) 보정용 대수 정합 변수
const float MAG_W[3][3] = {{
  {{ {mag_W[0,0]:12.8f}f, {mag_W[0,1]:12.8f}f, {mag_W[0,2]:12.8f}f }},
  {{ {mag_W[1,0]:12.8f}f, {mag_W[1,1]:12.8f}f, {mag_W[1,2]:12.8f}f }},
  {{ {mag_W[2,0]:12.8f}f, {mag_W[2,1]:12.8f}f, {mag_W[2,2]:12.8f}f }}
}};

const float MAG_B[3] = {{
  {mag_b[0]:12.8f}f, {mag_b[1]:12.8f}f, {mag_b[2]:12.8f}f
}};

#endif // _CALIB_PARAMS_H_
"""
    output_dir = "calibration_tool/output"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    output_path = os.path.join(output_dir, "calib_params.h")
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(header_content)
        
    print(f"\n📂 [헤더 발행 성공] C++ 펌웨어용 파라미터 헤더 파일이 빌드되었습니다!")
    print(f"   ↳ 경로: {output_path}")

def main():
    print("=" * 60)
    print(" 🎯 Phase 2 Magnetometer Ellipsoid Fitting Solver")
    print("=" * 60)
    
    # 데이터 로드
    try:
        data = np.load("calibration_tool/collected_data.npz")
        mag_raw = data["mag"]
        print(f"📂 실측 데이터 세트 로드 완료 (Shape: {mag_raw.shape})")
    except Exception as e:
        print(f"❌ 데이터 로드 에러: {e}")
        print("data_collection.py를 먼저 가동해 주십시오.")
        return
        
    # 자력계 최적화 피팅 구동
    W_mag, b_mag = calibrate_mag_ellipsoid(mag_raw)
    
    # 보정 전/후 품질 RMSE 비교 대조
    # 보정 전 Norm 분산
    mean_mag_raw = np.mean(mag_raw, axis=0)
    norm_before = np.linalg.norm(mag_raw - mean_mag_raw, axis=1)
    radius_before = np.mean(norm_before)
    
    # 보정 공식 대입
    mag_cal = (W_mag @ (mag_raw - b_mag).T).T
    norm_after = np.linalg.norm(mag_cal, axis=1)
    
    rmse_before = np.sqrt(np.mean((norm_before - radius_before) ** 2))
    rmse_after = np.sqrt(np.mean((norm_after - 1.0) ** 2))
    
    print("\n" + "=" * 60)
    print(" 🎉 자력계 타원체(Soft/Hard Iron) 보정 완료 보고")
    print("=" * 60)
    print(f"📉 보정 전 자력계 Norm 구면 RMSE: {rmse_before:.2f} count")
    print(f"📈 보정 후 자력계 Norm 구면 RMSE: {rmse_after:.6f} unit (정규화 단위)")
    print("-" * 60)
    print("⚙️ 보정 행렬 W_mag [3x3]:")
    print(W_mag)
    print("\n⚙️ Hard-iron Bias b_mag [3x1]:")
    print(b_mag)
    print("=" * 60)
    
    # 가속도 보정 데이터도 있는지 확인 후 헤더파일 일괄 빌드
    if os.path.exists("calibration_tool/acc_params.npz"):
        acc_data = np.load("calibration_tool/acc_params.npz")
        W_acc = acc_data["W"]
        b_acc = acc_data["b"]
        generate_cpp_header(W_acc, b_acc, W_mag, b_mag)
    else:
        print("\n⚠️ 가속도계 보정 데이터(acc_params.npz)가 아직 생성되지 않았습니다.")
        print("   accel_calibration.py를 실행한 후 다시 mag_calibration.py를 구동하면")
        print("   본 툴이 최종 C++ 헤더파일(calib_params.h)을 일괄 빌드해 줍니다.")

if __name__ == "__main__":
    main()
