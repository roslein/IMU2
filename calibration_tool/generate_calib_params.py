import numpy as np
import os
import sys

def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
        
    print("=" * 60)
    print(" 🎯 Phase 2 IMU Integrated Calibration Parameter Compiler")
    print("=" * 60)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    
    # 디폴트 파라미터 구성 (보정 전 Identity 및 Zero 상태)
    acc_W = np.eye(3)
    acc_b = np.zeros(3)
    mag_W = np.eye(3)
    mag_b = np.zeros(3)
    gyro_W = np.eye(3)
    gyro_b = np.zeros(3)
    
    # 1. 가속도계 파라미터 로드
    acc_path = os.path.join(output_dir, "acc_params.npz")
    if os.path.exists(acc_path):
        acc_data = np.load(acc_path)
        acc_W = acc_data["W"]
        acc_b = acc_data["b"]
        print("    가속도계 보정 파라미터 로드 성공.")
    else:
        print("⚠️  가속도계 보정 파라미터(acc_params.npz) 유실 ➔ 미보정(Identity) 상태 컴파일.")
        
    # 2. 자력계 파라미터 로드
    mag_path = os.path.join(output_dir, "mag_params.npz")
    if os.path.exists(mag_path):
        mag_data = np.load(mag_path)
        mag_W = mag_data["W"]
        mag_b = mag_data["b"]
        print("    자력계 보정 파라미터 로드 성공.")
    else:
        print("⚠️  자력계 보정 파라미터(mag_params.npz) 유실 ➔ 미보정(Identity) 상태 컴파일.")
        
    # 3. 자이로스코프 파라미터 로드
    gyro_path = os.path.join(output_dir, "gyro_params.npz")
    if os.path.exists(gyro_path):
        gyro_data = np.load(gyro_path)
        # 만약 W 행렬이 저장되어 있지 않다면 Identity로 복원
        if "W_gyro" in gyro_data:
            gyro_W = gyro_data["W_gyro"]
        elif "W" in gyro_data:
            gyro_W = gyro_data["W"]
        else:
            gyro_W = np.eye(3)
            
        if "b_gyro" in gyro_data:
            gyro_b = gyro_data["b_gyro"]
        elif "b" in gyro_data:
            gyro_b = gyro_data["b"]
        else:
            gyro_b = np.zeros(3)
        print("    자이로스코프 보정 파라미터 로드 성공.")
    else:
        print("⚠️  자이로스코프 보정 파라미터(gyro_params.npz) 유실 ➔ 미보정(Identity) 상태 컴파일.")
        
    # 헤더파일 템플릿 제너레이션
    header_content = f"""/*
 * Real-world IMU Auto-Generated Calibration Parameters (calib_params.h)
 * 본 헤더파일은 툴체인을 통해 통합 생성되었습니다.
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

// 3. 자이로스코프 20-Positions Global Bias 및 Scale Factor 보정용 대수 정합 변수
const float GYRO_W[3][3] = {{
  {{ {gyro_W[0,0]:12.8f}f, {gyro_W[0,1]:12.8f}f, {gyro_W[0,2]:12.8f}f }},
  {{ {gyro_W[1,0]:12.8f}f, {gyro_W[1,1]:12.8f}f, {gyro_W[1,2]:12.8f}f }},
  {{ {gyro_W[2,0]:12.8f}f, {gyro_W[2,1]:12.8f}f, {gyro_W[2,2]:12.8f}f }}
}};

const float GYRO_B[3] = {{
  {gyro_b[0]:12.8f}f, {gyro_b[1]:12.8f}f, {gyro_b[2]:12.8f}f
}};

#endif // _CALIB_PARAMS_H_
"""

    # 1. 로컬 output 디렉토리에 백업 저장
    output_path = os.path.join(output_dir, "calib_params.h")
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(header_content)
    print(f"\n📂 [로컬 백업 성공] calib_params.h 저장 완료 ➔ {output_path}")

    # 2. 펌웨어 디렉토리에 이식
    imu_root = os.path.dirname(script_dir)
    firmware_calib_dir = os.path.join(imu_root, "firmware", "calibrated", "calibration")
    if not os.path.exists(firmware_calib_dir):
        os.makedirs(firmware_calib_dir)
        
    firmware_output_path = os.path.join(firmware_calib_dir, "calib_params.h")
    with open(firmware_output_path, "w", encoding="utf-8") as f:
        f.write(header_content)
    print(f"🚀 [펌웨어 이식 성공] C++ 헤더 실시간 덮어쓰기 완료 ➔ {firmware_output_path}")
    print("=" * 60)

if __name__ == "__main__":
    main()
