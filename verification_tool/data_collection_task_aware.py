"""
Real-world IMU Phase 2.4 Task-Aware Calibration Data Collection (data_collection_task_aware.py)
목적: 대표 8개 안착 Pose 각각에서 수평 회전판의 30도 간격(12개 눈금) 데카르트 곱(총 96포인트)
      원시 데이터를 수동 조작 트리거(Enter)에 맞추어 수집 및 파일로 컴파일합니다.
      실시간 20면체 3D Matplotlib 그래픽 가이드 뷰어(GUI)를 탑재하여 거치 상태를 모니터링합니다.
"""

import serial
import serial.tools.list_ports
import struct
import time
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 로컬 모듈 탐색 경로 설정하여 calibration_tool 하위의 icosahedron.py 재사용
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMU_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.append(IMU_ROOT)
sys.path.append(os.path.join(IMU_ROOT, 'calibration_tool'))

import icosahedron

PACKET_SIZE = 39
START_BYTE = 0xAA
END_BYTE = 0x55

# X, Y, Z축 중력 반력을 균일하게 덮을 수 있는 20면체 대표 8개 면 인덱스 설정
# (20면체의 대표적인 법선 방향 다양성을 지닌 독립 8면 엄선)
REPRESENTATIVE_FACES = [0, 2, 5, 7, 10, 13, 16, 18]

# 30도 회전 간격 눈금 정의 (12개 포인트)
YAW_GT_STEPS = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]

# 3D Matplotlib 창 및 플롯 관리를 위한 전역 변수
fig = None
ax = None

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.lower()
        if "usb" in desc or "arduino" in desc or "ch340" in desc or "ftdi" in desc:
            return port.device
    if ports:
        return ports[0].device
    return None

def parse_packet(packet):
    """
    39바이트 패킷 검증 및 파싱하여 acc, gyro, mag 반환
    """
    if len(packet) != PACKET_SIZE:
        return None
    if packet[0] != START_BYTE or packet[-1] != END_BYTE:
        return None
        
    xor_sum = START_BYTE
    for b in packet[1:37]:
        xor_sum ^= b
    if xor_sum != packet[37]:
        return None
        
    floats = struct.unpack('<9f', packet[1:37])
    acc = np.array(floats[0:3])
    gyro = np.array(floats[3:6])
    mag = np.array(floats[6:9])
    return acc, gyro, mag

def collect_rolling_average(ser, duration_sec=1.5):
    """
    지정 시간 동안 들어오는 바이너리 스트림에서 패킷을 파싱하여 평균 물리 성분을 계산
    """
    ser.reset_input_buffer()
    byte_buffer = bytearray()
    collected_acc = []
    collected_gyro = []
    collected_mag = []
    
    start_time = time.time()
    
    while time.time() - start_time < duration_sec:
        try:
            in_waiting = ser.in_waiting
        except Exception:
            return None, None, None
            
        if in_waiting > 0:
            try:
                data = ser.read(in_waiting)
                byte_buffer.extend(data)
            except Exception:
                return None, None, None
                
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
                    parsed = parse_packet(packet)
                    if parsed is not None:
                        acc, gyro, mag = parsed
                        collected_acc.append(acc)
                        collected_gyro.append(gyro)
                        collected_mag.append(mag)
                byte_buffer = byte_buffer[1:]
        time.sleep(0.002)
        
    if len(collected_acc) == 0:
        return None, None, None
        
    avg_acc = np.mean(collected_acc, axis=0)
    avg_gyro = np.mean(collected_gyro, axis=0)
    avg_mag = np.mean(collected_mag, axis=0)
    return avg_acc, avg_gyro, avg_mag

def init_3d_plot():
    """
    3D Matplotlib 창을 대화형 모드(plt.ion)로 초기화합니다.
    """
    global fig, ax
    plt.ion()
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    plt.show()

def update_3d_plot(collected_faces, normals, new_point=None, new_matched_idx=None, expected_face_idx=None):
    """
    3D 공간 상에 이미 수집된 면(Green), 미수집 대상 면(Red), 실시간 측정 후보 벡터(Yellow 별표)를 맵핑합니다.
    """
    global fig, ax
    if fig is None or ax is None:
        return
        
    ax.clear()
    
    # 1. 3D 구면 그리드 (Sphere Wireframe) 시각화
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 10)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_wireframe(x, y, z, color='lightgray', alpha=0.3, linewidth=0.5)
    
    # 2. 대표 8개 면 법선 그리기
    for idx in REPRESENTATIVE_FACES:
        n = normals[idx]
        if idx in collected_faces:
            ax.scatter(n[0], n[1], n[2], color='green', s=100, marker='o', label='Collected' if idx == REPRESENTATIVE_FACES[0] else "")
            ax.text(n[0] * 1.15, n[1] * 1.15, n[2] * 1.15, f"#{idx:02d} (G)", color='darkgreen', fontsize=9, weight='bold')
            ax.plot([0, n[0]], [0, n[1]], [0, n[2]], color='green', alpha=0.5, linewidth=1.5)
        else:
            if idx == expected_face_idx:
                # 현재 맞춰야 하는 목표 대상 면은 특별한 표기 제공
                ax.scatter(n[0], n[1], n[2], color='orange', s=120, marker='s', edgecolors='black', label='Target Face')
                ax.text(n[0] * 1.15, n[1] * 1.15, n[2] * 1.15, f"#{idx:02d} (Target)", color='darkorange', fontsize=10, weight='bold')
                ax.plot([0, n[0]], [0, n[1]], [0, n[2]], color='orange', alpha=0.8, linewidth=2.0)
            else:
                ax.scatter(n[0], n[1], n[2], color='red', s=60, marker='x', label='Uncollected' if idx == REPRESENTATIVE_FACES[-1] else "")
                ax.text(n[0] * 1.15, n[1] * 1.15, n[2] * 1.15, f"#{idx:02d}", color='darkred', fontsize=9)
                ax.plot([0, n[0]], [0, n[1]], [0, n[2]], color='red', alpha=0.3, linestyle='--', linewidth=1.0)
                
    # 3. 새로운 포인트 후보 시각화 (실시간 중력 방향 매칭 벡터)
    if new_point is not None:
        norm_val = np.linalg.norm(new_point)
        if norm_val > 1e-3:
            # 중력 관성 반력 반전 정합
            new_unit = - (new_point / norm_val)
            ax.scatter(new_unit[0], new_unit[1], new_unit[2], color='gold', s=180, marker='*', edgecolors='black', linewidths=1.0, label='Current Pose')
            ax.plot([0, new_unit[0]], [0, new_unit[1]], [0, new_unit[2]], color='gold', linewidth=3.0)
            
            if new_matched_idx is not None:
                n_target = normals[new_matched_idx]
                ax.plot([new_unit[0], n_target[0]], [new_unit[1], n_target[1]], [new_unit[2], n_target[2]], color='orange', linestyle=':', linewidth=2.0)
                
    ax.set_title("📌 IMU Task-Aware 8-Pose Calibration Guidemap (3D)")
    ax.set_xlabel("X-Axis")
    ax.set_ylabel("Y-Axis")
    ax.set_zlabel("Z-Axis")
    ax.grid(True)
    
    # 범례 중복 방지
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper left')
    
    plt.draw()
    plt.pause(0.01)

def monitor_live_matching(ser, collected_faces, expected_face_idx):
    """
    사용자가 회전판을 조작하기 전, 현재 올려둔 면이 예상되는 대표 면과 기하학적으로 일치하는지
    실시간으로 가속도 벡터를 수집하여 20면체 매칭 진단 결과를 GUI 창과 화면에 피드백합니다.
    """
    # 20면체 센서 기준 법선 정의 로드
    normals = icosahedron.get_rotated_normals()
    print("⏳ 센서 안착 실시간 감지 작동 중 (1초마다 3D 창 및 텍스트 갱신)...")
    
    while True:
        # 0.3초 짧은 평균으로 실시간 중력 벡터 파악
        acc, _, _ = collect_rolling_average(ser, duration_sec=0.3)
        if acc is not None:
            norm_val = np.linalg.norm(acc)
            if norm_val > 1e-3:
                gravity_direction = - (acc / norm_val)
                matched_idx, similarity = icosahedron.match_face(gravity_direction, normals)
                
                # 3D 가이드 그래픽 갱신
                update_3d_plot(
                    collected_faces=collected_faces, 
                    normals=normals, 
                    new_point=acc, 
                    new_matched_idx=matched_idx,
                    expected_face_idx=expected_face_idx
                )
                
                status_str = f"🔍 감지된 현재 안착 면: #{matched_idx:02d} (유사도: {similarity:.4f})"
                
                if matched_idx == expected_face_idx:
                    print(f"\r{status_str} -> ✅ 목표 면 안착 일치! [Enter]를 입력하면 눈금 수집을 시작합니다.", end="")
                    break
                else:
                    print(f"\r{status_str} -> ⚠ 목표 면(#{expected_face_idx:02d})과 다릅니다. 올바른 포즈로 거치해 주십시오.", end="")
        time.sleep(0.1)
    print() # 개행

def main():
    print("=========================================================================")
    print(" 🎯 Task-Aware Calibration Data Collector (v0.3.0) ")
    print("=========================================================================")
    
    # 아두이노 시리얼 포트 스캔
    port = find_arduino_port()
    if not port:
        print("❌ 아두이노 장치를 감지하지 못했습니다. 시리얼 포트 연결을 점검하십시오.")
        sys.exit(1)
        
    print(f"📡 연결 포트 감지: {port} (Baudrate: 115200)")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        time.sleep(2.0)  # 아두이노 리부팅 딜레이
    except Exception as e:
        print(f"❌ 포트 개방 실패: {e}")
        sys.exit(1)
        
    # 저장할 데이터 셋 순번 선택 (반복능 검증용)
    print("\n[실험 설정]")
    try:
        dataset_num = int(input("수집할 데이터셋 번호 입력 (1~5 중 선택): "))
        if not (1 <= dataset_num <= 5):
            raise ValueError()
    except ValueError:
        print("⚠ 올바르지 않은 번호 입력. 기본값 '1'로 세팅합니다.")
        dataset_num = 1
        
    output_filename = f"pose_data_{dataset_num}.npz"
    print(f"💾 출력 파일명 예약: IMU/verification_tool/output/{output_filename}")
    
    # 0. 자이로 바이어스 측정을 위한 0도 거치 안착 트리거
    print("\n-------------------------------------------------------------------------")
    print("📌 [자이로 바이어스 영점 갱신]")
    print("지그를 수평 회전판에 완전히 고정하고 수동 0도 눈금에 정확히 맞춘 상태에서 멈춰주십시오.")
    input("정지 안착을 완료했다면 [Enter]를 누르십시오. 5.0초 동안 바이어스를 산출합니다...")
    
    print("자이로 바이어스 윈도우 수집 중 (500샘플)...")
    avg_acc_init, avg_gyro_init, avg_mag_init = collect_rolling_average(ser, duration_sec=5.0)
    if avg_gyro_init is None:
        print("❌ 초기 데이터 수집 실패. 스트림 수신 상태를 점검하십시오.")
        ser.close()
        sys.exit(1)
        
    # 자이로 raw는 mdps 단위이므로 dps 환산(/1000.0) 및 rad/s 변환(*pi/180) 필요
    dps_scale = 1.0 / 1000.0
    rad_scale = np.pi / 180.0
    gyro_bias_rad = avg_gyro_init * dps_scale * rad_scale
    
    print(f"✅ 초기 0도 안착 자이로 Raw 평균 (mdps): {avg_gyro_init}")
    print(f"✅ 환산된 자이로 영점 Bias (rad/s): {gyro_bias_rad}")
    
    # 3D 그래픽 엔진 기동
    init_3d_plot()
    
    # 수집용 저장 리스트
    acc_raw_list = []
    mag_raw_list = []
    gyro_raw_list = []
    yaw_gt_list = []
    pose_idx_list = []
    collected_faces = set()
    
    # 20면체 대표 8면 및 12개 회전각도 데카르트 곱 수집 시작
    print("\n-------------------------------------------------------------------------")
    print("📌 [데카르트 곱 수집 가동]")
    print(f"대표 {len(REPRESENTATIVE_FACES)}개 면 각각에 대해 수평판 12개 눈금을 차례로 조작하며 수집합니다.")
    print("[입력 명령 가이드] Enter: 현재 눈금 수집 | r: 이전 눈금 재수집(Undo) | q: 중단 및 저장")
    
    face_count = len(REPRESENTATIVE_FACES)
    yaw_count = len(YAW_GT_STEPS)
    
    f_idx = 0
    while f_idx < face_count:
        face = REPRESENTATIVE_FACES[f_idx]
        print(f"\n========================================================")
        print(f"👉 [면 설정 {f_idx + 1}/{face_count}] 20면체 #{face:02d}번 면을 수평판 바닥에 밀착하십시오.")
        print(f"========================================================")
        
        # 사용자가 올바른 면을 올렸는지 실시간 가속도 기반 SVD 법선 진단 및 감지 완료 대기 (3D 뷰어 동기화)
        monitor_live_matching(ser, collected_faces, face)
        
        y_idx = 0
        while y_idx < yaw_count:
            angle = YAW_GT_STEPS[y_idx]
            
            # 실시간 상태 모니터링 표시 (사용자가 수집 준비에 활용할 수 있도록)
            print(f"\n[안착 안내] #{face:02d}번 면 -> 수평 회전판 눈금 {angle:03d}도에 고정하십시오.")
            
            user_cmd = input(f"조작 완료 후 [Enter] 키 입력 (r=재시도, q=조기종료): ").strip().lower()
            
            if user_cmd == 'q':
                confirm = input("⚠ 정말로 수집을 조기 종료하고 파일에 저장할까요? (y/n): ").strip().lower()
                if confirm == 'y':
                    f_idx = face_count  # 전역 탈출 트리거
                    break
                else:
                    continue
                    
            if user_cmd == 'r':
                if len(acc_raw_list) > 0:
                    # 마지막으로 수집된 포인트 제거
                    acc_raw_list.pop()
                    mag_raw_list.pop()
                    gyro_raw_list.pop()
                    yaw_gt_list.pop()
                    pose_idx_list.pop()
                    
                    # 수집 인덱스 역산
                    if y_idx > 0:
                        y_idx -= 1
                    else:
                        if f_idx > 0:
                            f_idx -= 1
                            face = REPRESENTATIVE_FACES[f_idx]
                            collected_faces.discard(face)
                            y_idx = yaw_count - 1
                        else:
                            print("⚠ 이전에 수집된 데이터가 없습니다.")
                    print(f"↩ Undo 실행 완료: 이전 안착(#{face:02d}면, {YAW_GT_STEPS[y_idx]}도)으로 돌아갑니다.")
                else:
                    print("⚠ 취소할 이전 수집 데이터가 없습니다.")
                continue
                
            # 1.5초간 수집 롤링 평균 시작
            print(f"⌛ 1.5초간 패킷 데이터 로깅 및 롤링 평균 연산 중...")
            acc_val, gyro_val, mag_val = collect_rolling_average(ser, duration_sec=1.5)
            
            if acc_val is None:
                print("❌ 로깅 실패. 패킷 손실이 발견되었습니다. 이 눈금 조작을 다시 시도하십시오.")
                continue
                
            # 저장소에 추가
            acc_raw_list.append(acc_val)
            mag_raw_list.append(mag_val)
            gyro_raw_list.append(gyro_val)
            yaw_gt_list.append(angle)
            pose_idx_list.append(face)
            
            print(f"📊 수집 완료 (#{face:02d}면, {angle:03d}도) -> acc_raw: {acc_val}, mag_raw: {mag_val}")
            y_idx += 1
            
        # 해당 면 수집 완료 처리
        if user_cmd != 'q':
            collected_faces.add(face)
        f_idx += 1
        
    ser.close()
    plt.close('all') # 3D 창 종료
    
    # 수집 완료 후 폴더 검증 및 저장
    total_samples = len(acc_raw_list)
    if total_samples == 0:
        print("\n❌ 수집된 데이터 포인트가 존재하지 않아 저장을 생략합니다.")
        sys.exit(0)
        
    output_dir = os.path.join(IMU_ROOT, 'verification_tool', 'output')
    os.makedirs(output_dir, exist_ok=True)
    save_path = os.path.join(output_dir, output_filename)
    
    np.savez(
        save_path,
        acc_raw=np.array(acc_raw_list),
        mag_raw=np.array(mag_raw_list),
        gyro_raw=np.array(gyro_raw_list),
        yaw_gt=np.array(yaw_gt_list),
        pose_idx=np.array(pose_idx_list),
        gyro_bias_init=gyro_bias_rad
    )
    
    print("\n=========================================================================")
    print(f"🎉 데이터셋 수집 완료! 총 {total_samples}개 정적 포인트 적재 성공.")
    print(f"📂 파일 저장 경로: {save_path}")
    print("=========================================================================")

if __name__ == "__main__":
    main()
