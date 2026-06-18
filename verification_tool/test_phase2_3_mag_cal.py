"""
Real-world IMU Phase 2.3 Magnetometer Calibration Multi-Criteria Decision Solver
목적: 3가지 자력계 보정 모델(3-param, 6-param Cholesky, 9-param SPD)을
      전체 원시 데이터(collected_data_100s.npz) 기반으로 피팅하고,
      각도기(Protractor) 직접 대조 실시간 시리얼 데이터 수집 루프를 기동하여
      Roll, Pitch 및 틸트 보정(Tilt Compensation) 전후 Yaw 모니터링을 포함한
      Yaw Closed-loop 및 Increment RMSE를 실측 기반으로 사전식 정렬 평가 및 최종 낙찰합니다.
"""

import os
import sys
import struct
import time
import serial
import serial.tools.list_ports
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy
from scipy.optimize import least_squares

# 로컬 모듈 탐색 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMU_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.append(IMU_ROOT)
sys.path.append(os.path.join(IMU_ROOT, 'calibration_tool'))

import icosahedron

# 시리얼 통신용 상수 정의
PACKET_SIZE = 39
START_BYTE = 0xAA
END_BYTE = 0x55

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usb" in port.description.lower() or "arduino" in port.description.lower() or "ch340" in port.description.lower():
            return port.device
    if ports:
        return ports[0].device
    return None

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

# 9-parameter SPD 자력계 보정 솔버 (Cholesky SPD 제약 적용)
def calibrate_mag_9param_spd(mag_raw):
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

# 실시간 시리얼 데이터 수집 헬퍼 함수는 main 내부에서 non-blocking으로 통합 기동됨 (자이로 누적 적분 연동)


def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
        
    print("=" * 70)
    print(" 🎯 Magnetometer Calibration Protractor-Guided Decision Solver (Phase 2.3)")
    print("=" * 70)
    
    # 가속도계 보정 파라미터 로드
    acc_param_path = os.path.join(IMU_ROOT, "calibration_tool", "output", "acc_params.npz")
    if not os.path.exists(acc_param_path):
        print(f"❌ 가속도계 보정 파라미터가 유실되었습니다: {acc_param_path}")
        sys.exit(1)
    acc_params = np.load(acc_param_path)
    W_acc = acc_params["W"]
    b_acc = acc_params["b"]
    print("✅ 가속도계 보정 파라미터 로드 완료.")
    
    # 1. 데이터 로드 (collected_data_100s.npz 사용)
    data_path = os.path.join(SCRIPT_DIR, "output", "collected_data_100s.npz")
    if not os.path.exists(data_path):
        print(f"❌ 데이터가 존재하지 않습니다: {data_path}")
        sys.exit(1)
        
    data = np.load(data_path)
    acc_100s = data["acc"]
    mag_100s = data["mag"]
    
    # 자력계 Z축 데이터는 ino단에서 이미 반전 완료됨 (이중 반전 방지 위해 제거)
    
    acc_mean = np.mean(acc_100s, axis=1)
    mag_mean = np.mean(mag_100s, axis=1)
    mag_all = mag_100s.reshape(-1, 3)
    
    # 2. 가속도계 12-parameter 정합 완료
    rot_normals = icosahedron.get_rotated_normals()
    _, _, _ = calibrate_acc_12param(acc_mean, rot_normals) # 12-param 피팅 검증용 기하 도출
    acc_cal_mean = (W_acc @ (acc_mean - b_acc).T).T
    acc_unit_mean = acc_cal_mean / np.linalg.norm(acc_cal_mean, axis=1, keepdims=True)
    
    # 3. 3가지 보정 알고리즘 피팅 구동 (전체 원시 데이터 입력 기준)
    results = []
    
    combinations = [
        (1, "전체 원시", "3-param (Offset Only)", lambda: calibrate_mag_3param(mag_all)),
        (2, "전체 원시", "6-param (Diagonal W)", lambda: calibrate_mag_6param(mag_all)),
        (3, "전체 원시", "9-param SPD           ", lambda: calibrate_mag_9param_spd(mag_all))
    ]
    
    print("\n⚡ 3가지 자력계 보정 후보군 피팅 가동...")
    for idx, data_desc, algo_name, solver in combinations:
        try:
            W_mag, b_mag = solver()
            
            # 검증용 평균 데이터(20점) 보정
            mag_cal_mean = (W_mag @ (mag_mean - b_mag).T).T
            
            # Magnitude Norm RMSE 산출
            m_norms = np.linalg.norm(mag_cal_mean, axis=1)
            norm_rmse = np.sqrt(np.mean((m_norms - 1.0)**2))
            norm_std = np.std(m_norms)
            
            # 20개 포지션 내부 복각 (Dip Angle) 산출
            mag_unit_mean = mag_cal_mean / np.linalg.norm(mag_cal_mean, axis=1, keepdims=True)
            dip_list = []
            for k in range(20):
                dot_val = np.clip(np.dot(acc_unit_mean[k], mag_unit_mean[k]), -1.0, 1.0)
                dip = 90.0 - np.degrees(np.arccos(dot_val))
                dip_list.append(dip)
            
            dip_ref = np.mean(dip_list)
            dip_rmse = np.sqrt(np.mean((np.array(dip_list) - dip_ref)**2))
            
            results.append({
                "comb_idx": idx,
                "data_desc": data_desc,
                "algo_name": algo_name,
                "norm_rmse": norm_rmse,
                "norm_std": norm_std,
                "dip_rmse": dip_rmse,
                "dip_ref": dip_ref,
                "dip_list": dip_list,
                "b_mag": b_mag,
                "W_mag": W_mag
            })
            print(f"   ↳ [피팅 완료] {algo_name.strip()}")
        except Exception as e:
            print(f"   ↳ [피팅 실패] {algo_name.strip()} ➔ 에러: {e}")
            
    # 실시간 자이로 바이어스 초기화 (1단계 0도 안착 시 자동 갱신됨)
    gyro_bias = np.zeros(3)

    # 실시간 시리얼 포트 연결
    print("\n📶 시리얼 포트 연결 시도 중...")
    port = find_arduino_port()
    if not port:
        print("❌ 연결된 아두이노 장치(COM Port)를 찾을 수 없습니다.")
        sys.exit(1)
        
    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        time.sleep(2)
        print(f"✅ 포트 연결 성공: {port}")
    except Exception as e:
        print(f"❌ 포트 연결 실패: {e}")
        sys.exit(1)

    print("\n" + "=" * 60)
    print(" 🛠 각도기(Protractor Guided) + 자이로 실시간 병행 수집 개시")
    print("=" * 60)
    print("가이드에 따라 지정된 회전각 위치에 정렬 완료 후 [Space] 또는 [Enter]를 누르십시오.")
    print("-" * 60)
    
    target_angles = [0.0, 90.0, 180.0, 270.0, 360.0]
    live_raw_acc_samples = []
    live_raw_mag_samples = []
    live_gyro_integrated_angles = []
    
    R_tilt_fixed = None
    yaw_gyro_accum = 0.0
    
    import msvcrt
    ser.reset_input_buffer()
    byte_buffer = bytearray()
    
    try:
        for idx, target_angle in enumerate(target_angles):
            print(f"\n👉 [단계 {idx+1}/5] 센서를 각도기 기준 {target_angle:3.1f}도에 위치시키십시오.")
            print("   정렬 완료 후 [Enter] 또는 [Space]를 누르면 저장합니다...")
            
            acc_window = []
            mag_window = []
            gyro_window = []
            
            while True:
                try:
                    in_waiting = ser.in_waiting
                except Exception as e:
                    print(f"\n❌ 시리얼 포트 읽기 실패: {e}")
                    sys.exit(1)
                    
                if in_waiting > 0:
                    byte_buffer.extend(ser.read(in_waiting))
                    
                while len(byte_buffer) >= PACKET_SIZE:
                    if byte_buffer[0] != START_BYTE:
                        byte_buffer.pop(0)
                        continue
                        
                    packet = byte_buffer[:PACKET_SIZE]
                    if packet[-1] != END_BYTE:
                        byte_buffer.pop(0)
                        continue
                        
                    xor_sum = START_BYTE
                    for b in packet[1:37]:
                        xor_sum ^= b
                        
                    if xor_sum != packet[37]:
                        byte_buffer = byte_buffer[PACKET_SIZE:]
                        continue
                        
                    floats = struct.unpack('<9f', packet[1:37])
                    acc_raw = np.array(floats[0:3])
                    gyro_raw = np.array(floats[3:6])
                    mag_raw = np.array(floats[6:9])
                    
                    # 실시간 회전구간 자이로 Z축 적분 누적 (rad -> deg)
                    if idx > 0:
                        # 3축 자이로에서 바이어스 차감 후 틸트 보정 정합 행렬(R_tilt_fixed)을 곱해 지구 수직축(Z) 각속도 추출
                        gyro_zero = gyro_raw - gyro_bias
                        if R_tilt_fixed is not None:
                            gyro_cal = R_tilt_fixed @ gyro_zero
                        else:
                            gyro_cal = gyro_zero
                        gz_cal = gyro_cal[2]
                        yaw_gyro_accum += gz_cal * 0.01
                        
                    # 실시간 거치 물리 자세 계산
                    acc_cal_sample = W_acc @ (acc_raw - b_acc)
                    roll = np.degrees(np.arctan2(acc_cal_sample[1], acc_cal_sample[2]))
                    pitch = np.degrees(np.arctan2(-acc_cal_sample[0], np.sqrt(acc_cal_sample[1]**2 + acc_cal_sample[2]**2)))
                    
                    acc_window.append(acc_raw)
                    mag_window.append(mag_raw)
                    gyro_window.append(gyro_raw)
                    if len(acc_window) > 100:
                        acc_window.pop(0)
                        mag_window.pop(0)
                        gyro_window.pop(0)
                        
                    # 실시간 틸트 보정 모니터링 (3-param 기준)
                    if len(results) > 0 and R_tilt_fixed is not None:
                        W_mag_3p = results[0]["W_mag"]
                        b_mag_3p = results[0]["b_mag"]
                        mag_cal_sample = W_mag_3p @ (mag_raw - b_mag_3p)
                        yaw_raw_sample = np.degrees(np.arctan2(mag_cal_sample[1], mag_cal_sample[0]))
                        
                        mag_cal_ned_sample = R_tilt_fixed @ mag_cal_sample
                        yaw_tilt_sample = np.degrees(np.arctan2(mag_cal_ned_sample[1], mag_cal_ned_sample[0]))
                        
                        sys.stdout.write(f"\r📡 실시간 ➔ Roll: {roll:5.1f} | Pitch: {pitch:5.1f} | Yaw_tilt: {yaw_tilt_sample:6.1f} | Gyro_Yaw: {np.degrees(yaw_gyro_accum):6.1f}")
                        sys.stdout.flush()
                    else:
                        sys.stdout.write(f"\r📡 실시간 ➔ Roll: {roll:5.1f} | Pitch: {pitch:5.1f} | Gyro_Yaw: {np.degrees(yaw_gyro_accum):6.1f}")
                        sys.stdout.flush()
                        
                    byte_buffer = byte_buffer[PACKET_SIZE:]
                    
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key in [b'\r', b' ', b'\n']:
                        if len(acc_window) < 100:
                            print(f"\n⚠️  데이터 버퍼 부족 ({len(acc_window)}/100). 대기 후 다시 시도하십시오.")
                            continue
                        print(f"\n📥 [저장] 단계 {idx+1} 완료. (Gyro_Yaw: {np.degrees(yaw_gyro_accum):.2f} deg)")
                        break
                time.sleep(0.001)
                
            mean_acc_raw = np.mean(acc_window, axis=0)
            mean_mag_raw = np.mean(mag_window, axis=0)
            live_raw_acc_samples.append(mean_acc_raw)
            live_raw_mag_samples.append(mean_mag_raw)
            live_gyro_integrated_angles.append(np.degrees(yaw_gyro_accum))
            
            # 0도 첫 거치 시점에 단 한 번만 안착 면 매칭, R_tilt_fixed 계산 및 실시간 자이로 바이어스 갱신 후 고정 (downward 기준 [0,0,-1] 정렬)
            if R_tilt_fixed is None:
                gyro_bias = np.mean(gyro_window, axis=0)
                print(f"🔥 [자이로 바이어스 실시간 갱신 완료] gyro_bias (rad/s): {gyro_bias}")
                acc_cal_sample = W_acc @ (mean_acc_raw - b_acc)
                best_idx, _ = icosahedron.match_face(-acc_cal_sample, rot_normals)
                res_rot_fixed, _ = R_scipy.align_vectors(np.array([[0.0, 0.0, -1.0]]), np.array([rot_normals[best_idx]]))
                R_tilt_fixed = res_rot_fixed.as_matrix()
                print(f"🎯 [안착면 최초 매칭 완료] Face #{best_idx:02d} 법선 기준 틸트 보정 행렬 고정 완료.")
                
    finally:
        ser.close()
        print("\n🔌 시리얼 포트 연결을 안전하게 해제했습니다.")
        
    live_raw_acc_samples = np.array(live_raw_acc_samples)
    live_raw_mag_samples = np.array(live_raw_mag_samples)
    
    # 5. 각 보정 모델 대입 후 틸트 보정 반영 Yaw Increment RMSE 및 Closed-loop 오차 계산
    print("\n📊 수집된 실측 데이터 기반 최종 Yaw 기하학적 정합성(Tilt 보정 반영) 연산 중...")
    
    # 0번째 포지션 가속도 기준으로 안착 면 1회 판정 및 R_tilt_fixed 도출 (downward 기준 [0,0,-1] 정렬)
    acc_cal_0 = W_acc @ (live_raw_acc_samples[0] - b_acc)
    best_idx, _ = icosahedron.match_face(-acc_cal_0, rot_normals)
    res_rot_fixed, _ = R_scipy.align_vectors(np.array([[0.0, 0.0, -1.0]]), np.array([rot_normals[best_idx]]))
    R_tilt_fixed_eval = res_rot_fixed.as_matrix()
    print(f"📡 [최종 평가 정합] 최초 0도 위치의 Face #{best_idx:02d} 안착 법선 기준 고정 틸트 보정 적용.")
    
    for r in results:
        W_mag = r["W_mag"]
        b_mag = r["b_mag"]
        
        yaws_est_raw_list = []
        yaws_est_tilt_list = []
        
        for k in range(5):
            acc_cal_sample = W_acc @ (live_raw_acc_samples[k] - b_acc)
            mag_cal_sample = W_mag @ (live_raw_mag_samples[k] - b_mag)
            
            # 1. 틸트 보정 전 Yaw 계산 (Raw)
            yaw_raw = np.arctan2(mag_cal_sample[1], mag_cal_sample[0])
            yaws_est_raw_list.append(yaw_raw)
            
            # 2. 안착 면 법선 기준 고정 틸트 보정(Tilt Compensation) 적용 후 Yaw 계산
            mag_cal_ned = R_tilt_fixed_eval @ mag_cal_sample
            yaw_tilt = np.arctan2(mag_cal_ned[1], mag_cal_ned[0])
            yaws_est_tilt_list.append(yaw_tilt)
            
        # 0도 기준 unwrapped 정렬 적용
        # 틸트 보정 전 정렬
        yaws_est_raw_unwrapped = np.unwrap(yaws_est_raw_list)
        offset_raw = yaws_est_raw_unwrapped[0] - np.radians(target_angles[0])
        yaws_est_raw_aligned = np.degrees(yaws_est_raw_unwrapped - offset_raw)
        
        # 틸트 보정 후 정렬
        yaws_est_tilt_unwrapped = np.unwrap(yaws_est_tilt_list)
        offset_tilt = yaws_est_tilt_unwrapped[0] - np.radians(target_angles[0])
        yaws_est_tilt_aligned = np.degrees(yaws_est_tilt_unwrapped - offset_tilt)
        
        # 360도 Closed-loop 폐합 오차 계산 (틸트 보정 후 기준)
        closed_loop_err = np.abs(yaws_est_tilt_aligned[-1] - 360.0)
        
        # Yaw Increment RMSE 계산 (각도기 눈금 기준)
        yaw_errors = yaws_est_tilt_aligned - target_angles
        yaw_rmse = np.sqrt(np.mean(yaw_errors**2))
        
        # Yaw Increment RMSE 계산 (자이로 적분 궤적 기준)
        gyro_angles_aligned = np.array(live_gyro_integrated_angles) - live_gyro_integrated_angles[0]
        yaw_errors_gyro = yaws_est_tilt_aligned - gyro_angles_aligned
        yaw_rmse_gyro = np.sqrt(np.mean(yaw_errors_gyro**2))
        
        r["closed_loop_err"] = closed_loop_err
        r["yaw_rmse"] = yaw_rmse
        r["yaw_rmse_gyro"] = yaw_rmse_gyro
        r["yaws_est_aligned"] = yaws_est_tilt_aligned
        r["yaws_est_raw_aligned"] = yaws_est_raw_aligned
        r["gyro_angles_aligned"] = gyro_angles_aligned
        
    # 6. 복각 참값 입력 획득 및 20개 포지션 절대 오차 RMSE 계산 (기본 디폴트 54.3000으로 북반구 음수 부호 고정)
    print("\n" + "=" * 60)
    dip_input = input("👉 인천 미추홀구 복각 참값 입력 (디폴트: 54.3000): ").strip()
    dip_true = float(dip_input) if dip_input else 54.3000
    print(f"📡 복각 참값 {dip_true:.4f} deg 기준으로 절대 복각 RMSE 재산출 중...\n")
    
    for r in results:
        dip_errors_true = np.array(r["dip_list"]) - dip_true
        r["dip_rmse_true"] = np.sqrt(np.mean(dip_errors_true**2))
        
    # 7. 순수 사전식 순서(Lexicographic Order) 다차원 정렬로 최적 모델 최종 낙찰
    # 우선순위: 1. closed_loop_err -> 2. yaw_rmse_gyro -> 3. dip_rmse_true -> 4. norm_rmse
    sorted_res = sorted(results, key=lambda x: (x["closed_loop_err"], x["yaw_rmse_gyro"], x["dip_rmse_true"], x["norm_rmse"]))
    best_res = sorted_res[0]
    
    # 8. 보고서 텍스트 작성 및 출력
    report_lines = []
    report_lines.append("=" * 155)
    report_lines.append(" 🎯 Magnetometer Calibration Lexicographic Multi-Criteria Report (v0.2.3) ")
    report_lines.append("=" * 155)
    report_lines.append("Yaw Evaluation Mode: Protractor + Gyro-Integration (SVD 틸트 보정 반영)")
    report_lines.append("Data Source: collected_data_100s.npz (전체 원시 30,000점 피팅)")
    report_lines.append(f"True Dip Angle Reference: {dip_true:.4f} deg")
    report_lines.append("-" * 155)
    report_lines.append(f"{'알고리즘 모델':<28}{'Norm RMSE':<14}{'Norm Std':<12}{'Dip True RMSE':<18}{'Closed-loop':<16}{'Yaw RMSE(Prot)':<16}{'Yaw RMSE(Gyro)':<16}")
    report_lines.append("-" * 155)
    
    for r in results:
        line = f"{r['algo_name'].strip():<28}{r['norm_rmse']:<14.6f}{r['norm_std']:<12.6f}{r['dip_rmse_true']:<18.4f}{r['closed_loop_err']:<16.4f}{r['yaw_rmse']:<16.4f}{r['yaw_rmse_gyro']:<16.4f}"
        report_lines.append(line)
    report_lines.append("-" * 155)
    
    report_lines.append(f"📡 최적 보정 설정 최종 낙찰: {best_res['algo_name'].strip()}")
    report_lines.append(f"   - 실측 Closed-loop 폐합 오차: {best_res['closed_loop_err']:.4f} deg")
    report_lines.append(f"   - 실측 Yaw Increment RMSE (자이로 기준): {best_res['yaw_rmse_gyro']:.4f} deg")
    report_lines.append(f"   - 실측 Yaw Increment RMSE (각도기 기준): {best_res['yaw_rmse']:.4f} deg")
    report_lines.append(f"   - 20면 복각 절대 오차 RMSE:  {best_res['dip_rmse_true']:.4f} deg (평균 복각: {best_res['dip_ref']:.4f} deg)")
    report_lines.append(f"   - 20면 지자기 Norm RMSE:     {best_res['norm_rmse']:.6f} (표준편차: {best_res['norm_std']:.6f})")
    
    # 각도별 디테일 상세 출력 추가 (틸트 보정 전후 대조)
    report_lines.append("-" * 155)
    report_lines.append("🔍 각 보정 모델별 각도기 정렬 Yaw 계산 각도 (unwrapped, deg):")
    report_lines.append(f"   ↳ Target Protractor : {target_angles}")
    report_lines.append(f"   ↳ Gyro-Integration  : [{', '.join([f'{a:6.2f}' for a in best_res['gyro_angles_aligned']])}]")
    for r in results:
        angle_strs_raw = ", ".join([f"{a:6.2f}" for a in r["yaws_est_raw_aligned"]])
        angle_strs_tilt = ", ".join([f"{a:6.2f}" for a in r["yaws_est_aligned"]])
        report_lines.append(f"   ↳ {r['algo_name'].strip():<20} ➔ Raw : [{angle_strs_raw}]")
        report_lines.append(f"   ↳ {' ' * 20} ➔ Tilt: [{angle_strs_tilt}]")
        
    report_lines.append("-" * 155)
    report_lines.append(f"⚙️ 낙찰된 최적 W_mag:\n{best_res['W_mag']}")
    report_lines.append(f"⚙️ 낙찰된 최적 b_mag: {best_res['b_mag']}")
    report_lines.append("=" * 155)
    
    report_text = "\n".join(report_lines)
    print("\n" + report_text)
    
    # 텍스트 파일 저장
    output_dir = os.path.join(SCRIPT_DIR, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    report_path = os.path.join(output_dir, "mag_calibration_compare_report_v0.2.3.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report_text)
        
    # calibration_tool/output 에도 mag_params.npz 로 저장 (최종 파라미터 백업)
    calib_output_dir = os.path.join(IMU_ROOT, "calibration_tool", "output")
    if not os.path.exists(calib_output_dir):
        os.makedirs(calib_output_dir)
    npz_save_path = os.path.join(calib_output_dir, "mag_params.npz")
    np.savez(npz_save_path, W=best_res["W_mag"], b=best_res["b_mag"])
    
    print(f"📂 [보고서 저장 완료] ➔ {report_path}")
    print(f"💾 [최종 지자기 파라미터 백업 완료] ➔ {npz_save_path}")

if __name__ == "__main__":
    main()
