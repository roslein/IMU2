"""
Real-world IMU Phase 2.2 Magnetometer Calibration Multi-Method Comparison
목적: 다양한 자력계 보정 모델(3/6/9-sym/9-full/융합형 가중 패널티)과 
      데이터 입력 조건(20면 평균 vs 30,000점 전체 원시 데이터)의 9가지 조합을 교차 평가하고,
      Magnitude Norm RMSE 및 Dip Angle RMSE 기준으로 최적의 알고리즘을 규명합니다.
"""

import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy
from scipy.optimize import least_squares

# 로컬 모듈 탐색 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMU_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.append(IMU_ROOT)
sys.path.append(os.path.join(IMU_ROOT, 'calibration_tool'))

import icosahedron

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

# preconditioning 래퍼 함수 (수치 안정성 확보)
def preconditioned_solver(mag_raw, solver_func):
    mean_raw = np.mean(mag_raw, axis=0)
    dev_norms = np.linalg.norm(mag_raw - mean_raw, axis=1)
    scale_factor = np.mean(dev_norms) if np.mean(dev_norms) > 0 else 1.0
    
    mag_normed = (mag_raw - mean_raw) / scale_factor
    
    W_normed, b_normed = solver_func(mag_normed)
    
    W_est = W_normed / scale_factor
    b_est = mean_raw + scale_factor * b_normed
    
    return W_est, b_est

# 3-parameter 자력계 보정 솔버 (오프셋만 피팅, 스케일 W는 Identity 고정)
def calibrate_mag_3param(mag_raw):
    def solver(d):
        def residuals(b, x):
            return np.linalg.norm(x - b, axis=1) - 1.0
        p0 = np.array([0.0, 0.0, 0.0])
        res = least_squares(residuals, p0, args=(d,), method='lm')
        b_est = res.x
        W_est = np.eye(3)
        return W_est, b_est
    return preconditioned_solver(mag_raw, solver)

# Cholesky Parameterization helper for 100% SPD guarantee
def make_spd(p):
    L = np.array([
        [np.exp(p[0]), 0.0, 0.0],
        [p[1], np.exp(p[2]), 0.0],
        [p[3], p[4], np.exp(p[5])]
    ])
    return L @ L.T

# 6-parameter 자력계 보정 솔버 (대각 스케일 + 오프셋, 스케일 exp 처리로 양수 강제)
def calibrate_mag_6param(mag_raw):
    def solver(d):
        def residuals(p, x):
            b = p[:3]
            W = np.diag(np.exp(p[3:6]))
            cal = (W @ (x - b).T).T
            return np.linalg.norm(cal, axis=1) - 1.0
        p0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        res = least_squares(residuals, p0, args=(d,), method='lm')
        p = res.x
        b_est = p[:3]
        W_est = np.diag(np.exp(p[3:6]))
        return W_est, b_est
    return preconditioned_solver(mag_raw, solver)

# 9-parameter Symmetric 자력계 보정 솔버 (Cholesky SPD 제약 적용)
def calibrate_mag_9param_sym(mag_raw):
    def solver(d):
        def residuals(p, x):
            b = p[:3]
            W = make_spd(p[3:9])
            cal = (W @ (x - b).T).T
            return np.linalg.norm(cal, axis=1) - 1.0
        p0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        res = least_squares(residuals, p0, args=(d,), method='lm')
        p = res.x
        b_est = p[:3]
        W_est = make_spd(p[3:9])
        return W_est, b_est
    return preconditioned_solver(mag_raw, solver)

# 9-parameter Full 자력계 보정 솔버 (일반 3x3 자유 행렬 W, 12자유도)
def calibrate_mag_9param_full(mag_raw):
    def solver(d):
        def residuals(p, x):
            b = p[:3]
            W = p[3:12].reshape(3, 3)
            cal = (W @ (x - b).T).T
            return np.linalg.norm(cal, axis=1) - 1.0
        p0 = np.concatenate([
            [0.0, 0.0, 0.0],
            np.eye(3).flatten()
        ])
        res = least_squares(residuals, p0, args=(d,), method='lm')
        p = res.x
        b_est = p[:3]
        W_est = p[3:12].reshape(3, 3)
        return W_est, b_est
    return preconditioned_solver(mag_raw, solver)

# 융합형 가중 패널티 자력계 보정 솔버 (제4안, Cholesky SPD 제약 적용)
# J = mean(J_all) + lambda * mean(J_face)
def calibrate_mag_hybrid(mag_raw_all, mag_raw_face, lmbda=1.0):
    # preconditioning 스케일 팩터 산출
    mean_raw = np.mean(mag_raw_all, axis=0)
    dev_norms = np.linalg.norm(mag_raw_all - mean_raw, axis=1)
    scale_factor = np.mean(dev_norms) if np.mean(dev_norms) > 0 else 1.0
    
    # 정규화 도메인으로 데이터 변환
    d_all = (mag_raw_all - mean_raw) / scale_factor
    d_face = (mag_raw_face - mean_raw) / scale_factor
    
    def residuals(p, x_all, x_face):
        b = p[:3]
        W = make_spd(p[3:9])
        
        # 전체 데이터 잔차
        cal_all = (W @ (x_all - b).T).T
        res_all = np.linalg.norm(cal_all, axis=1) - 1.0
        
        # 평균 데이터 잔차
        cal_face = (W @ (x_face - b).T).T
        res_face = np.linalg.norm(cal_face, axis=1) - 1.0
        
        # 가중 선형 결합 (제곱합 형태 최적화를 위해 잔차 벡터에 sqrt(lambda)를 가산)
        # 1/N 팩터를 반영하여 균형 조정
        res_all_scaled = res_all / np.sqrt(len(x_all))
        res_face_scaled = res_face * np.sqrt(lmbda / len(x_face))
        
        return np.concatenate([res_all_scaled, res_face_scaled])
        
    p0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    res = least_squares(residuals, p0, args=(d_all, d_face), method='lm')
    p = res.x
    b_normed = p[:3]
    W_normed = make_spd(p[3:9])
    
    W_est = W_normed / scale_factor
    b_est = mean_raw + scale_factor * b_normed
    
    return W_est, b_est


def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
        
    print("=" * 70)
    print(" 🎯 Magnetometer Calibration Method Cross-Comparison (Phase 2.2)")
    print("=" * 70)
    
    # 1. 데이터 로드
    data_path = os.path.join(SCRIPT_DIR, "output", "collected_data_100s.npz")
    if not os.path.exists(data_path):
        print(f"❌ 데이터가 존재하지 않습니다: {data_path}")
        sys.exit(1)
        
    data = np.load(data_path)
    acc_100s = data["acc"]  # (20, N, 3)
    mag_100s = data["mag"]  # (20, N, 3)
    
    # 20개 포지션 평균 데이터 산출
    acc_mean = np.mean(acc_100s, axis=1)
    mag_mean = np.mean(mag_100s, axis=1)
    
    # 전체 원시 데이터 평탄화 (30,000점)
    acc_all = acc_100s.reshape(-1, 3)
    mag_all = mag_100s.reshape(-1, 3)
    
    # 2. 가속도계 12-parameter 정합 완료
    rot_normals = icosahedron.get_rotated_normals()
    W_acc, b_acc, _ = calibrate_acc_12param(acc_mean, rot_normals)
    acc_cal_mean = (W_acc @ (acc_mean - b_acc).T).T
    acc_unit_mean = acc_cal_mean / np.linalg.norm(acc_cal_mean, axis=1, keepdims=True)
    
    # 9가지 교차 비교 조합 구성
    results = []
    
    # 조합 리스트 정의
    # (조합 번호, 입력 데이터 설명, 알고리즘 이름, solver_func)
    combinations = [
        # 20면 평균 데이터 기반 피팅 (4가지)
        (1, "20면 평균", "3-param (Offset Only)", lambda: calibrate_mag_3param(mag_mean)),
        (2, "20면 평균", "6-param (Diagonal W)", lambda: calibrate_mag_6param(mag_mean)),
        (3, "20면 평균", "9-param Symmetric   ", lambda: calibrate_mag_9param_sym(mag_mean)),
        (4, "20면 평균", "9-param Full        ", lambda: calibrate_mag_9param_full(mag_mean)),
        
        # 전체 원시 데이터 기반 피팅 (5가지)
        (5, "전체 원시", "3-param (Offset Only)", lambda: calibrate_mag_3param(mag_all)),
        (6, "전체 원시", "6-param (Diagonal W)", lambda: calibrate_mag_6param(mag_all)),
        (7, "전체 원시", "9-param Symmetric   ", lambda: calibrate_mag_9param_sym(mag_all)),
        (8, "전체 원시", "9-param Full        ", lambda: calibrate_mag_9param_full(mag_all)),
        (9, "전체 원시", "융합형 가중 패널티  ", lambda: calibrate_mag_hybrid(mag_all, mag_mean, lmbda=1.0))
    ]
    
    print("🔄 9가지 교차 비교 피팅 및 정량 평가 루프 기동...")
    
    for idx, data_desc, algo_name, solver in combinations:
        try:
            # 캘리브레이션 실행
            W_mag, b_mag = solver()
            
            # 검증용 평균 데이터(20점) 보정
            mag_cal_mean = (W_mag @ (mag_mean - b_mag).T).T
            
            # 1. Magnitude Norm RMSE 산출
            m_norms = np.linalg.norm(mag_cal_mean, axis=1)
            norm_rmse = np.sqrt(np.mean((m_norms - 1.0)**2))
            
            # 2. 복각 (Dip Angle) 산출 및 편차 RMSE 산출
            mag_unit_mean = mag_cal_mean / np.linalg.norm(mag_cal_mean, axis=1, keepdims=True)
            dip_list = []
            for k in range(20):
                dot_val = np.clip(np.dot(acc_unit_mean[k], mag_unit_mean[k]), -1.0, 1.0)
                dip = 90.0 - np.degrees(np.arccos(dot_val))
                dip_list.append(dip)
            
            # 평균 복각 기반 내부 정밀도 RMSE
            dip_ref = np.mean(dip_list)
            dip_errors = np.array(dip_list) - dip_ref
            dip_rmse = np.sqrt(np.mean(dip_errors**2))
            
            # 인천 미추홀구 인하로 100 복각 참값 (-54.3 deg) 기반 절대 오차 RMSE
            DIP_TRUE = -54.3
            dip_errors_true = np.array(dip_list) - DIP_TRUE
            dip_rmse_true = np.sqrt(np.mean(dip_errors_true**2))
            
            results.append({
                "comb_idx": idx,
                "data_desc": data_desc,
                "algo_name": algo_name,
                "norm_rmse": norm_rmse,
                "dip_rmse": dip_rmse,
                "dip_rmse_true": dip_rmse_true,
                "dip_ref": dip_ref,
                "b_mag": b_mag,
                "W_mag": W_mag
            })
            print(f"   ↳ [완료] 조합 {idx:02d}: {data_desc} + {algo_name.strip()} ➔ Dip True RMSE: {dip_rmse_true:6.4f} deg")
        except Exception as e:
            print(f"   ↳ [실패] 조합 {idx:02d}: {data_desc} + {algo_name.strip()} ➔ 에러: {e}")
            
    # 3. 텍스트 보고서 생성 및 출력
    report_lines = []
    report_lines.append("=" * 110)
    report_lines.append(" 🎯 Magnetometer Calibration Cross-Comparison Quantitative Report ")
    report_lines.append("=" * 110)
    report_lines.append(f"Data Source: collected_data_100s.npz (N={len(acc_100s[0])} samples/face)")
    report_lines.append(f"True Dip Angle Reference for Incheon: -54.3000 deg")
    report_lines.append("-" * 110)
    report_lines.append(f"{'조합':<6}{'피팅 데이터':<14}{'알고리즘 모델':<24}{'Norm RMSE':<16}{'Dip RMSE(deg)':<16}{'Dip True RMSE(deg)':<20}{'평균 복각(deg)':<18}")
    report_lines.append("-" * 110)
    
    for r in results:
        line = f"조합 {r['comb_idx']:02d}  {r['data_desc']:<12}{r['algo_name']:<24}{r['norm_rmse']:<16.6f}{r['dip_rmse']:<16.4f}{r['dip_rmse_true']:<20.4f}{r['dip_ref']:<18.4f}"
        report_lines.append(line)
    report_lines.append("-" * 110)
    
    # 최적 조합 자동 판별 (Dip True RMSE 최소 기준)
    sorted_res = sorted(results, key=lambda x: x["dip_rmse_true"])
    best_res = sorted_res[0]
    report_lines.append(f"📡 최적 보정 설정: 조합 {best_res['comb_idx']:02d} ({best_res['data_desc']} + {best_res['algo_name'].strip()})")
    report_lines.append(f"   - 최소 절대 복각(True) RMSE: {best_res['dip_rmse_true']:.4f} deg")
    report_lines.append(f"   - 평균 복각 및 내부 RMSE: {best_res['dip_ref']:.4f} deg (내부 RMSE: {best_res['dip_rmse']:.4f} deg)")
    report_lines.append(f"   - 최적 W_mag (Scale):\n{best_res['W_mag']}")
    report_lines.append(f"   - 최적 b_mag (Offset): {best_res['b_mag']}")
    report_lines.append("=" * 110)

    
    report_text = "\n".join(report_lines)
    print("\n" + report_text)
    
    # 텍스트 파일 저장
    output_dir = os.path.join(SCRIPT_DIR, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    report_path = os.path.join(output_dir, "mag_calibration_compare_report_v0.2.2.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report_text)
    print(f"📂 [보고서 저장 완료] ➔ {report_path}")

if __name__ == "__main__":
    main()
