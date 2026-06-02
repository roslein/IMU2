"""
Real-world IMU Phase 3.1 Static Orientation Error Validator (test_phase3.py)
목적: calibrated.ino 보정 데이터 실시간 스트림으로부터 20 Positions 신규 방향 데이터를 수집하고,
      20 Positions ideal 거치 기하 Alignment로 계산된 이론적 GT 쿼터니언 LUT와 1대1 대조하여
      정적 회전 자세의 3D 각도 오차(Quaternion Angle Error) 및 전체 RMSE를 정량 검증하고 시각화합니다.
"""

import serial
import serial.tools.list_ports
import struct
import time
import sys
import os
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

def compute_theoretical_gt_quaternions(normals_jig, R_mount):
    """
    1단계: 정20면체 지그 ideal 법선 벡터와 마운팅 회전 행렬을 결합하여,
           20개 안착 포지션에 매핑되는 센서 기준 이론적 GT 쿼터니언 LUT를 생성합니다.
    R_mount: 지그 좌표계에서 센서 좌표계로의 회전 (3x3)
    """
    q_gt_lut = []
    
    for i in range(20):
        # 20면체 무게중심 안착 시, ideal 중력 가속도 역방향(Jig Down인 [0, 0, 1]에 매치)
        # Z축(안착 면 법선) 및 Y축 기저 정합을 통한 3차원 ideal 회전 정립
        n_jig = normals_jig[i]
        
        # n_jig 벡터와 수평 직교를 이루는 임의의 직교 축 y_jig 유도
        # Z축 n_jig와 외적하여 정규 직교 기저를 이룰 ideal Y 기저 정의
        y_axis = np.array([0.0, 1.0, 0.0])
        if abs(np.dot(n_jig, y_axis)) > 0.99:
            y_axis = np.array([1.0, 0.0, 0.0])
        
        x_jig = np.cross(y_axis, n_jig)
        x_jig /= np.linalg.norm(x_jig)
        y_jig = np.cross(n_jig, x_jig)
        
        # 지그 프레임에서 지구 고정 NED 프레임으로의 ideal 정합 회전
        # n_jig가 NED Down[0, 0, 1]을 향하고, x_jig가 NED North[1, 0, 0], y_jig가 NED East[0, 1, 0]를 향한다고 정합
        # R_jig_to_ned 기둥 벡터 조립
        R_jig_to_ned = np.column_stack((x_jig, y_jig, n_jig)).T
        
        # 센서에서 NED로의 회전: R_sensor_to_ned = R_jig_to_ned @ R_jig_to_sensor
        # R_mount는 지그에서 센서로의 회전이므로, R_jig_to_sensor = R_mount
        R_sensor_to_ned = R_jig_to_ned @ R_mount.T
        
        # 쿼터니언 복조
        r = R_scipy.from_matrix(R_sensor_to_ned)
        q_raw = r.as_quat() # [qx, qy, qz, qw]
        
        # [qw, qx, qy, qz] 형태로 정렬하여 LUT 저장
        q_gt = np.array([q_raw[3], q_raw[0], q_raw[1], q_raw[2]])
        q_gt_lut.append(q_gt)
        
    return np.array(q_gt_lut)

def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')

    print("=" * 60)
    print(" 🎯 Real-world IMU Phase 3.1 Static Orientation Error Validator")
    print("=" * 60)
    print("⚠️ [필독] 기동 전, EKF 트래킹용 펌웨어(calibrated.ino)가 보드에 업로드되어")
    print("⚠️ 실시간으로 보정 바이너리 텔레메트리를 송출하고 있는지 반드시 확인하십시오.")
    print("=" * 60)

    # 1. 기하학적 파라미터 및 R_mount 로드
    normals_jig = icosahedron.get_icosahedron_normals()
    R_mount = icosahedron.get_jig_to_sensor_rotation() # 지그 -> 센서 회전 (R_mount.T가 실제 탑재된 matrix)
    
    # 1단계: 20개 포지션별 이론적 GT 쿼터니언 LUT 사전 계산 완료
    q_gt_lut = compute_theoretical_gt_quaternions(normals_jig, R_mount)
    print("📊 20개 포지션별 이론적 GT 쿼터니언 LUT 사전 정의 성공.")

    # 2. 시리얼 연결
    port = find_arduino_port()
    if not port:
        print("❌ 사용 가능한 시리얼 포트를 감지하지 못했습니다.")
        return
        
    print(f"📡 시리얼 COM 포트 감지 성공: {port}")
    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        time.sleep(1.5)
    except Exception as e:
        print(f"❌ 포트 점유 또는 연결 실패: {e}")
        return

    # 3. 2단계: calibrated.ino 보정 데이터 기반 20 Positions 신규 수집 가동
    # 기존 data_collection.py 의 수집 기법을 그대로 차용하여 정밀 수집합니다.
    collected_acc_cal = []
    collected_mag_cal = []
    
    expected_samples = 500  # 최적 LPF 시간 5.0초 가이드라인 수렴
    
    print("\n👉 보정이 완료된 20개 방향 데이터 신규 수집을 시작합니다.")
    print("👉 20면체 지그 안내 3D 궤적에 따라 면을 안착하고 엔터를 쳐 주십시오.")
    
    active_idx = 0
    while active_idx < 20:
        print(f"\n👉 [현재 차례: {active_idx + 1:02d} / 20]")
        print(f"👉 20면체 법선의 #{active_idx:02d} 면을 바닥에 똑바로 정정 안착시켜 주십시오.")
        
        user_input = input(f"❔ #{active_idx:02d} 면 안착 완료 시 [Enter]를 누르십시오. (수집 취소/이전 면 재수집은 b 입력): ")
        if user_input.strip().lower() == 'b':
            if active_idx > 0:
                active_idx -= 1
                collected_acc_cal.pop()
                collected_mag_cal.pop()
                print(f"🔙 이전 #{active_idx:02d} 면 데이터셋을 제거하고 재수집 단계로 회항합니다.")
                continue
            else:
                print("⚠️ 첫 번째 포지션이므로 뒤로 돌아갈 수 없습니다.")
                continue
                
        # 5.0초 동안 초고속 바이너리 보정 데이터 수집 개시
        temp_acc = []
        temp_mag = []
        byte_buffer = bytearray()
        ser.reset_input_buffer()
        
        start_time = time.time()
        
        while len(temp_acc) < expected_samples:
            if time.time() - start_time > 8.0 and len(temp_acc) == 0:
                print("❌ 시리얼 스트리밍 패킷 수신 불통! calibrated.ino 구동 여부를 필히 확인하십시오.")
                ser.close()
                return
                
            try:
                in_waiting = ser.in_waiting
                if in_waiting > 0:
                    data = ser.read(in_waiting)
                    byte_buffer.extend(data)
            except Exception as e:
                print(f"❌ 데이터 수신 버스트 크래시: {e}")
                ser.close()
                return
                
            while len(byte_buffer) >= PACKET_SIZE:
                idx = byte_buffer.find(START_BYTE)
                if idx == -1:
                    byte_buffer.clear()
                    break
                if idx > 0:
                    byte_buffer = byte_buffer[idx:]
                    if len(byte_buffer) < PACKET_SIZE:
                        break
                        
                packet = byte_buffer[:PACKET_SIZE]
                if packet[-1] == END_BYTE:
                    xor_sum = START_BYTE
                    for b in packet[1:37]:
                        xor_sum ^= b
                    if xor_sum == packet[37]:
                        floats = struct.unpack('<9f', packet[1:37])
                        temp_acc.append(floats[0:3])
                        temp_mag.append(floats[6:9])
                        
                        progress = len(temp_acc)
                        percent = int((progress / expected_samples) * 100)
                        bar = '=' * int(progress / (expected_samples / 30))
                        sys.stdout.write(f"\r📥 LPF 데이터 수집: [{bar:<30}] {progress}/{expected_samples} 패킷 ({percent}%)")
                        sys.stdout.flush()
                        
                byte_buffer = byte_buffer[PACKET_SIZE:]
            time.sleep(0.002)
            
        # Box LPF 정량 평균 추출 완료
        acc_avg = np.mean(temp_acc, axis=0)
        mag_avg = np.mean(temp_mag, axis=0)
        
        collected_acc_cal.append(acc_avg)
        collected_mag_cal.append(mag_avg)
        
        print(f"\n✅ #{active_idx:02d} 면 신규 보정 데이터 평균 적립 완료!")
        print(f"   ↳ acc_avg: {acc_avg} | mag_avg: {mag_avg}")
        
        active_idx += 1

    ser.close()
    print("\n🎉 [대성공] 20개 방향 보정 데이터 실시간 신규 획득 및 적립 완수!")
    
    # 신규 수집된 calibrated data 영구 보존용 백업
    output_dir = os.path.join(SCRIPT_DIR, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    new_data_path = os.path.join(output_dir, "new_calib_collected.npz")
    np.savez(new_data_path, acc=collected_acc_cal, mag=collected_mag_cal)
    print(f"💾 보정 실측 신규 데이터셋 저장 위치: {new_data_path}")

    # 4. 3단계: TRIAD-NED 자세 역산 및 쿼터니언 각도 오차 RMSE 계산
    print("\n⚡ [3단계 검증] 20 Positions 3D 정적 자세 오리엔테이션 및 오차 검증 솔버 기동...")
    
    angle_errors = []
    
    for i in range(20):
        # imu_simulation 의 accel_mag_to_quaternion 대수 연산 인터페이스 호출
        q_est = accel_mag_to_quaternion(collected_acc_cal[i], collected_mag_cal[i])
        
        # 쿼터니언 오차 (Degree) 계산
        q_gt = q_gt_lut[i]
        err_deg = q_angle_error(q_gt, q_est)
        angle_errors.append(err_deg)
        
        # 결과 오일러각 복조
        euler_gt = np.degrees(quat_to_euler(q_gt))
        euler_est = np.degrees(quat_to_euler(q_est))
        
        print(f"📊 포지션 #{i:02d} 오차: {err_deg:8.4f}° | GT: {euler_gt} | EST: {euler_est}")

    angle_errors = np.array(angle_errors)
    rmse_error = np.sqrt(np.mean(angle_errors**2))
    max_error = np.max(angle_errors)
    min_error = np.min(angle_errors)

    print("\n" + "=" * 60)
    print(" 🎉 Phase 3 정적 3D 회전 자세 쿼터니언 오차 검증 결과 보고")
    print("=" * 60)
    print(f"📈 20개 꼭짓점 정위 거치 각도 오차 RMSE: {rmse_error:.6f}°")
    print(f"📈 최대 오리엔테이션 오차:           {max_error:.6f}° (포지션 #{np.argmax(angle_errors):02d})")
    print(f"📉 최소 오리엔테이션 오차:           {min_error:.6f}° (포지션 #{np.argmin(angle_errors):02d})")
    print("=" * 60)

    # 5. 결과 시각화 그래프 자동 빌드 저장
    plt.figure(figsize=(10, 6))
    faces_indices = np.arange(20)
    
    plt.bar(faces_indices, angle_errors, color='crimson', edgecolor='black', alpha=0.8, label='Orientation Angle Error')
    plt.axhline(y=rmse_error, color='darkblue', linestyle='--', linewidth=1.5, label=f'RMSE Error: {rmse_error:.4f}°')
    
    plt.title('IMU Phase 3 Static Orientation Validation: Quaternion Angle Error vs Position', fontsize=14)
    plt.xlabel('20-Icosahedron Face Index', fontsize=12)
    plt.ylabel('3D Rotation Angle Error [degrees]', fontsize=12)
    plt.xticks(faces_indices)
    plt.grid(True, which='both', linestyle=':', alpha=0.5)
    plt.legend(loc='upper right')
    
    # 각 Bar 위에 텍스트로 오차 각도 표시
    for idx, err in enumerate(angle_errors):
        plt.text(idx, err + (max_error * 0.01), f"{err:.2f}°", ha='center', va='bottom', fontsize=8, color='crimson')
        
    plt.ylim([0, max_error * 1.15])
    plt.tight_layout()
    
    result_img_path = os.path.join(SCRIPT_DIR, "test_phase3_result.png")
    plt.savefig(result_img_path, dpi=300)
    print(f"📊 [오차 분석 가시화 대조 플롯 저장 완료]")
    print(f"   ↳ 저장 경로: {result_img_path}")
    plt.show()

if __name__ == "__main__":
    main()
