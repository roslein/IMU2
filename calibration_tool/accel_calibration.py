"""
Real-world IMU Phase 2 Accelerometer Calibration (accel_calibration.py)
목적: 수집된 20개 정적 가속도 데이터와 정20면체 법선 벡터 LUT 간
      NN 코사인 유사도 매칭을 실행하고, 방바닥 경사각(alpha, beta)을 역산하며
      수렴하는 12-parameter 경사보상 선형 최소제곱 솔버를 구동합니다.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R_scipy
import os
import icosahedron

def calibrate_acc_12param_icosahedron(d, normals, max_iter=50):
    """
    정20면체 12-parameter 가속도계 최적화 솔버
    d: 실측된 20개 정적 평균 가속도 벡터 (20, 3)
    normals: 정20면체의 이론적 20개 면 법선 벡터 (20, 3)
    """
    n_points = len(d)
    
    # 1단계: 실측 벡터 각각에 대해 가장 가까운 20면체 법선 벡터를 NN 매칭 매핑
    matched_normals = np.zeros_like(d)
    match_indices = []
    
    for i in range(n_points):
        best_idx, res = icosahedron.match_face(-d[i], normals)
        matched_normals[i] = normals[best_idx]
        match_indices.append(best_idx)
        print(f"   ↳ [NN 매칭] 실측 데이터 #{i+1:02d} ➔ 20면체 법선 #{best_idx:02d} 매칭 완료 (Residual: {res:.6f})")
        
    # 2단계: 12-Parameter 선형 최소제곱 반복 루프 기동
    W_est = np.eye(3)
    b_est = np.zeros(3)
    alpha, beta = 0.0, 0.0  # 초기 바닥 기울기 (Pitch, Roll)
    
    print("\n⚡ 12-Parameter 경사각 보상 Recursive 최소제곱 솔버 구동 시작...")
    
    for iteration in range(max_iter):
        # 방의 경사각(alpha, beta)이 반영된 3차원 기울기 회전 행렬 생성
        R_tilt = R_scipy.from_euler('yx', [beta, alpha]).as_matrix()
        
        # Upward 수직항력(+1g) 지향을 위해 기준 벡터 방향을 반전
        g_ref = -(R_tilt @ matched_normals.T).T
        
        # 선형 회귀 대수 행렬 조립 (A = [g_ref, 1])
        A = np.hstack([g_ref, np.ones((n_points, 1))])
        
        # 선형 최소제곱 해 구하기: A * M^T + b = d
        # sol shape: (4, 3) ➔ M_T (3x3), b_new (3x1)
        sol, residuals, rank, s = np.linalg.lstsq(A, d, rcond=None)
        
        M_T = sol[:3, :]
        b_new = sol[3, :]
        
        # 보정 행렬 W_new = inv(M) ➔ M = M_T.T 이므로
        W_new = np.linalg.inv(M_T.T)
        
        # 현재 추정된 W와 b를 기준으로 보정된 데이터 계산
        d_cal = (W_new @ (d - b_new).T).T
        
        # 각 포지션에서의 보정된 가속도 벡터 오차를 기반으로 방의 잔여 경사각 역산(arcsin)
        # alpha는 Y축 방향 회전성분, beta는 X축 방향 회전성분에 가깝게 수렴
        # 수치적 안정성을 위해 평균값 투영
        pitch_errs = []
        roll_errs = []
        for i in range(n_points):
            ref = matched_normals[i]
            cal = d_cal[i] / np.linalg.norm(d_cal[i])
            # 두 정규화된 벡터 간의 3차원 기울기 차이 역산
            # 기하학적 보정 수식 (Euler angle 역변환 근사)
            pitch_errs.append(cal[1] - ref[1])
            roll_errs.append(cal[0] - ref[0])
            
        alpha = np.arcsin(np.clip(np.mean(pitch_errs), -1.0, 1.0))
        beta = np.arcsin(np.clip(np.mean(roll_errs), -1.0, 1.0))
        
        # 수렴 판정 (오프셋 벡터 b의 변화량이 거의 0에 수렴할 때)
        if np.allclose(b_est, b_new, atol=1e-8):
            print(f"✅ 솔버 수렴 성공! (반복 횟수: {iteration+1}회)")
            W_est, b_est = W_new, b_new
            break
            
        W_est, b_est = W_new, b_new
    else:
        print("⚠️ 솔버가 최대 반복 횟수에 도달하여 강제 정지되었습니다.")
        
    return W_est, b_est, alpha, beta

def main():
    import sys
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
    print("=" * 60)
    print(" 🎯 Phase 2 Accelerometer 12-Parameter Calibration Solver")
    print("=" * 60)
    
    # 스크립트 실행 디렉토리 기준 절대 경로 확보
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(script_dir, "output", "collected_data.npz")
    
    # 데이터 로드
    try:
        data = np.load(data_path)
        acc_raw = data["acc"]
        print(f"📂 실측 데이터 세트 로드 완료 (Shape: {acc_raw.shape})")
    except Exception as e:
        print(f"❌ 데이터 로드 에러: {e}")
        print("data_collection.py를 먼저 가동하여 raw 데이터를 수집하십시오.")
        return
        
    normals = icosahedron.get_rotated_normals()
    
    # 캘리브레이션 솔버 실행
    W, b, alpha, beta = calibrate_acc_12param_icosahedron(acc_raw, normals)
    
    # 보정 전/후 품질 RMSE 비교 대조
    norm_before = np.linalg.norm(acc_raw, axis=1)
    
    # 보정 공식 대입
    acc_cal = (W @ (acc_raw - b).T).T
    norm_after = np.linalg.norm(acc_cal, axis=1)
    
    rmse_before = np.sqrt(np.mean((norm_before - 1000.0) ** 2)) # raw 카운트 기준 1g=1000 가정
    rmse_after = np.sqrt(np.mean((norm_after - 1.0) ** 2))      # 보정 후 정규화 1g=1.0 가정
    
    print("\n" + "=" * 60)
    print(" 🎉 가속도계 12-Parameter 최적화 완료 보고")
    print("=" * 60)
    print(f"📐 보정된 경사각 (방의 경사): Pitch={np.degrees(alpha):.3f}° | Roll={np.degrees(beta):.3f}°")
    print(f"📉 보정 전 가속도 Norm 잔차 RMSE: {rmse_before:.3f} raw cnt")
    print(f"📈 보정 후 가속도 Norm 잔차 RMSE: {rmse_after:.6f} g (중력가속도 단위)")
    print("-" * 60)
    print("⚙️ 보정 행렬 W [3x3]:")
    print(W)
    print("\n⚙️ 보정 오프셋 b [3x1]:")
    print(b)
    print("=" * 60)
    
    # 향후 헤더파일 생성을 위해 별도 임시 파일에 파라미터 및 마운팅 회전행렬 저장
    R_mount = icosahedron.get_jig_to_sensor_rotation().T
    param_path = os.path.join(script_dir, "output", "acc_params.npz")
    np.savez(param_path, W=W, b=b, R_mount=R_mount)

if __name__ == "__main__":
    main()
