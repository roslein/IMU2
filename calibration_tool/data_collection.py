"""
Real-world IMU Phase 2 Data Collection (data_collection.py)
목적: 사용자가 임의의 면을 거치하면 실시간으로 감지하고 중복을 확인하여 수집하며,
      각 면마다 13눈금(0~360도, 30도 간격)을 수집합니다.
      최종 완료 시, 보정용(240포인트, 12눈금) 데이터와 Closed-loop 평가용(260포인트, 13눈금 전체) 데이터를
      이원화하여 저장하는 Dual-Save 아키텍처를 구현합니다.
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

# 로컬 모듈 탐색 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMU_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.append(IMU_ROOT)

from imu_core import icosahedron

PACKET_SIZE = 39
START_BYTE = 0xAA
END_BYTE = 0x55

fig = None
ax = None

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usb" in port.description.lower() or "arduino" in port.description.lower() or "ch340" in port.description.lower():
            return port.device
    if ports:
        return ports[0].device
    return None

def init_3d_plot(normals):
    global fig, ax
    plt.ion()
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    plt.show()

def update_3d_plot(completed_faces, normals, new_point=None, new_matched_idx=None):
    global fig, ax
    if fig is None or ax is None:
        return
        
    ax.clear()
    
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 10)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_wireframe(x, y, z, color='lightgray', alpha=0.3, linewidth=0.5)
    
    for idx, n in enumerate(normals):
        if idx in completed_faces:
            ax.scatter(n[0], n[1], n[2], color='green', s=100, marker='o')
            ax.text(n[0] * 1.15, n[1] * 1.15, n[2] * 1.15, f"#{idx:02d} (G)", color='darkgreen', fontsize=9, weight='bold')
            ax.plot([0, n[0]], [0, n[1]], [0, n[2]], color='green', alpha=0.5, linewidth=1.5)
        else:
            ax.scatter(n[0], n[1], n[2], color='red', s=60, marker='x')
            ax.text(n[0] * 1.15, n[1] * 1.15, n[2] * 1.15, f"#{idx:02d}", color='darkred', fontsize=9)
            ax.plot([0, n[0]], [0, n[1]], [0, n[2]], color='red', alpha=0.3, linestyle='--', linewidth=1.0)
            
    if new_point is not None:
        norm_val = np.linalg.norm(new_point)
        if norm_val > 1e-3:
            new_unit = - (new_point / norm_val)
            ax.scatter(new_unit[0], new_unit[1], new_unit[2], color='gold', s=180, marker='*', edgecolors='black', linewidths=1.0)
            ax.plot([0, new_unit[0]], [0, new_unit[1]], [0, new_unit[2]], color='gold', linewidth=3.0)
            
            if new_matched_idx is not None:
                n_target = normals[new_matched_idx]
                ax.plot([new_unit[0], n_target[0]], [new_unit[1], n_target[1]], [new_unit[2], n_target[2]], color='orange', linestyle=':', linewidth=2.0)
                
    ax.set_title("[GUIDE] IMU 20-Position Calibration Guide View (3D)")
    ax.set_xlabel("X-Axis")
    ax.set_ylabel("Y-Axis")
    ax.set_zlabel("Z-Axis")
    ax.grid(True)
    
    plt.draw()
    plt.pause(0.01)

def get_latest_sample(ser):
    try:
        in_waiting = ser.in_waiting
    except Exception:
        return None, None
        
    if in_waiting > 1000:
        try:
            ser.read(in_waiting - 100)
        except Exception:
            return None, None
            
    byte_buffer = bytearray()
    start_time = time.time()
    while time.time() - start_time < 0.2:
        try:
            if ser.in_waiting > 0:
                byte_buffer.extend(ser.read(ser.in_waiting))
        except Exception:
            return None, None
            
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
                    return np.array(floats[0:3]), np.array(floats[6:9])
            byte_buffer = byte_buffer[1:]
        time.sleep(0.002)
    return None, None

def collect_static_samples(ser, sample_count=150):
    byte_buffer = bytearray()
    collected_acc = []
    collected_mag = []
    collected_gyro = []
    
    ser.reset_input_buffer()
    
    while len(collected_acc) < sample_count:
        try:
            in_waiting = ser.in_waiting
        except (serial.SerialException, OSError) as e:
            print(f"\n❌ 시리얼 포트 통신 장애: {e}")
            sys.exit(1)
            
        if in_waiting > 0:
            try:
                data = ser.read(in_waiting)
                byte_buffer.extend(data)
            except (serial.SerialException, OSError) as e:
                print(f"\n❌ 데이터 읽기 실패: {e}")
                sys.exit(1)
                
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
                
            data_payload = packet[1:37]
            floats = struct.unpack('<9f', data_payload)
            
            collected_acc.append(floats[0:3])
            collected_gyro.append(floats[3:6])
            collected_mag.append(floats[6:9])
            
            byte_buffer = byte_buffer[PACKET_SIZE:]
            
            progress = len(collected_acc)
            bar = "=" * (progress * 30 // sample_count) + " " * (30 - progress * 30 // sample_count)
            sys.stdout.write(f"\r📥 데이터 수집 중: [{bar}] {progress}/{sample_count} 패킷 완료")
            sys.stdout.flush()
            
        time.sleep(0.001)
        
    print()
    mean_acc = np.mean(collected_acc, axis=0)
    mean_gyro = np.mean(collected_gyro, axis=0)
    mean_mag = np.mean(collected_mag, axis=0)
    
    return mean_acc, mean_gyro, mean_mag

def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')
        
    print("=" * 60)
    print(" 🎯 Real-world IMU Phase 2 Autonomous 9-Axis Data Collector")
    print("=" * 60)
    
    port = find_arduino_port()
    if not port:
        print("❌ 연결된 시리얼 장치(COM Port)를 찾을 수 없습니다.")
        sys.exit(1)
        
    baudrate = 115200
    try:
        ser = serial.Serial(port, baudrate, timeout=1.0)
        time.sleep(2)
        print(f"✅ 포트 연결 성공: {port}")
    except Exception as e:
        print(f"❌ 포트 연결 실패: {e}")
        sys.exit(1)
        
    # (20, 13, 3) 3D 구조 데이터셋 초기화 (0~360도 총 13눈금)
    collected_acc = np.zeros((20, 13, 3))
    collected_mag = np.zeros((20, 13, 3))
    collected_gyro = np.zeros((20, 13, 3))
    collected_yaw_gt = np.zeros((20, 13))
    
    completed_faces = set()
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    final_save_path = os.path.join(output_dir, "collected_data_9axis.npz")
    closed_loop_save_path = os.path.join(output_dir, "collected_data_9axis_closed_loop.npz")
    
    # 백업 처리
    for path in [final_save_path, closed_loop_save_path]:
        if os.path.exists(path):
            try:
                mtime = os.path.getmtime(path)
                time_struct = time.localtime(mtime)
                timestamp_str = time.strftime("%Y%m%d_%H%M%S", time_struct)
                filename = os.path.basename(path)
                name, ext = os.path.splitext(filename)
                backup_name = f"{name}_backup_{timestamp_str}{ext}"
                backup_path = os.path.join(output_dir, backup_name)
                os.rename(path, backup_path)
                print(f"\n📁 [자동 백업] 기존 파일 감지 ➔ 백업 완료: output/{backup_name}")
            except Exception as e:
                print(f"\n⚠️ 기존 데이터 백업 실패: {e}")
            
    checkpoint_path = os.path.join(output_dir, "checkpoint_data_9axis.npz")
    if os.path.exists(checkpoint_path):
        print("\n⚠️ [임시 복원 데이터 감지] 수집 중이던 9축 체크포인트가 존재합니다.")
        ans = input("   👉 기존 데이터를 이어서 수집하시겠습니까? (Y/N): ").strip().lower()
        if ans in ['', 'y', 'yes']:
            try:
                with np.load(checkpoint_path) as data:
                    collected_acc = data['acc']
                    collected_mag = data['mag']
                    collected_gyro = data['gyro']
                    collected_yaw_gt = data['yaw_gt']
                    completed_faces = set(data['completed_faces'])
                print(f"   ✅ [복원 성공] 총 {len(completed_faces)}개 면 수집 완료 상태 복원됨.")
            except Exception as e:
                print(f"   ⚠️ 복원 실패: {e}. 새로 수집을 시작합니다.")
                completed_faces = set()
        else:
            print("   ➔ 새로 수집을 개시합니다.")
            
    normals = icosahedron.get_rotated_normals()
    
    init_3d_plot(normals)
    update_3d_plot(completed_faces, normals)
    
    print("\n💡 사용자가 임의로 거치한 면을 자동 식별하여 수집합니다.")
    print("💡 각 면마다 0도부터 360도까지 13개 눈금(총 260포인트)을 수집합니다.\n")
    
    try:
        import msvcrt
        while len(completed_faces) < 20:
            print(f"\n" + "=" * 70)
            print(f" 📂 [자율 안착 가이드] 미완료된 정20면체 면 중 하나를 바닥에 안착시키십시오.")
            print(f" (완료 현황: {len(completed_faces)} / 20 면 완료)")
            print(f"=============================================================")
            
            ser.reset_input_buffer()
            detected_face_idx = None
            last_plot_time = 0
            
            # 실시간 동적 면 감지 대기 루프
            while True:
                acc_raw, mag_raw = get_latest_sample(ser)
                if acc_raw is not None:
                    best_idx, res = icosahedron.match_face(acc_raw, normals)
                    match_percent = (1.0 - res) * 100.0
                    
                    if best_idx in completed_faces:
                        status_str = f"이미 완료된 면입니다 ❌ (면 #{best_idx:02d}) 다른 면을 거치해 주십시오."
                        is_ready = False
                    else:
                        status_str = f"미완료 면 감지! ✅ (면 #{best_idx:02d}) | 일치율: {match_percent:.1f}%"
                        is_ready = True
                        detected_face_idx = best_idx
                        
                    sys.stdout.write(f"\r📡 실시간 프리뷰 ➔ {status_str}")
                    sys.stdout.flush()
                    
                    if time.time() - last_plot_time > 0.15:
                        update_3d_plot(completed_faces, normals, new_point=acc_raw, new_matched_idx=best_idx)
                        last_plot_time = time.time()
                
                plt.pause(0.01)
                
                # 키 입력 대기 및 감지 완료 분기
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key in [b'\r', b' ', b'\n']:
                        if is_ready and detected_face_idx is not None:
                            print(f"\n👉 면 #{detected_face_idx:02d} 선택 완료! 13눈금(Yaw) 수집 시퀀스 진입.")
                            break
                        else:
                            print("\n⚠️ 수집할 수 없는 면(이미 완료됨)이거나 감지되지 않았습니다.")
                time.sleep(0.01)
                
            # 해당 면에 대해 13눈금(0~360도) 순차 수집 진행
            for yaw_idx in range(13):
                yaw_target_deg = yaw_idx * 30.0
                print(f"\n👉 [눈금 수집] 면 #{detected_face_idx:02d} | 회전판 눈금: {yaw_target_deg:.1f}°")
                print("📡 회전판 눈금을 정밀 조정하여 정지한 후 [Space] 또는 [Enter] 키를 누르십시오...")
                
                ser.reset_input_buffer()
                last_plot_time = 0
                while True:
                    acc_raw, mag_raw = get_latest_sample(ser)
                    if acc_raw is not None:
                        sys.stdout.write(f"\r📡 실시간 모니터링 ➔ 면 #{detected_face_idx:02d} 거치 상태 유지 중... (안정 정지 대기)")
                        sys.stdout.flush()
                        
                        if time.time() - last_plot_time > 0.15:
                            update_3d_plot(completed_faces, normals, new_point=acc_raw, new_matched_idx=detected_face_idx)
                            last_plot_time = time.time()
                            
                    plt.pause(0.01)
                    if msvcrt.kbhit():
                        key = msvcrt.getch()
                        if key in [b'\r', b' ', b'\n']:
                            print(f"\n🚀 [수집 기동] 면 #{detected_face_idx:02d} | 눈금 {yaw_target_deg:.1f}° 1.5초 수집 시작!")
                            break
                    time.sleep(0.01)
                    
                time.sleep(0.3)
                
                mean_acc, mean_gyro, mean_mag = collect_static_samples(ser, sample_count=150)
                
                # 3D 슬롯 구조에 정위치 대입
                collected_acc[detected_face_idx, yaw_idx] = mean_acc
                collected_mag[detected_face_idx, yaw_idx] = mean_mag
                collected_gyro[detected_face_idx, yaw_idx] = mean_gyro
                collected_yaw_gt[detected_face_idx, yaw_idx] = yaw_target_deg
                
                print(f"   ↳ ⚖️ [평균 획득] Acc: [{mean_acc[0]:.1f}, {mean_acc[1]:.1f}, {mean_acc[2]:.1f}] | Mag: [{mean_mag[0]:.1f}, {mean_mag[1]:.1f}, {mean_mag[2]:.1f}]")
                
                # 매 눈금마다 오토세이브 임시 저장
                try:
                    np.savez(checkpoint_path,
                             acc=collected_acc,
                             mag=collected_mag,
                             gyro=collected_gyro,
                             yaw_gt=collected_yaw_gt,
                             completed_faces=list(completed_faces))
                except Exception as e:
                    print(f"   ⚠️ 체크포인트 백업 에러: {e}")
                    
            # 13눈금 수집 완료 후 완료 셋에 추가 및 리렌더링
            completed_faces.add(detected_face_idx)
            update_3d_plot(completed_faces, normals)
            
            # 최종 면 단위 갱신 세이브
            try:
                np.savez(checkpoint_path,
                         acc=collected_acc,
                         mag=collected_mag,
                         gyro=collected_gyro,
                         yaw_gt=collected_yaw_gt,
                         completed_faces=list(completed_faces))
            except Exception as e:
                pass
                
        # 20개 면 완료 시 [Dual-Save 기동]
        # 1. 보정용 데이터 (13번째 360도 눈금 제외 ➔ 240포인트)
        acc_calib = collected_acc[:, :12, :].reshape(240, 3)
        mag_calib = collected_mag[:, :12, :].reshape(240, 3)
        gyro_calib = collected_gyro[:, :12, :].reshape(240, 3)
        yaw_gt_calib = collected_yaw_gt[:, :12].reshape(240)
        
        np.savez(final_save_path, 
                 acc=acc_calib, 
                 mag=mag_calib, 
                 gyro=gyro_calib, 
                 yaw_gt=yaw_gt_calib)
        
        # 2. Closed-loop 평가용 데이터 (13눈금 전체 포함 ➔ 260포인트)
        acc_cl = collected_acc.reshape(260, 3)
        mag_cl = collected_mag.reshape(260, 3)
        gyro_cl = collected_gyro.reshape(260, 3)
        yaw_gt_cl = collected_yaw_gt.reshape(260)
        
        np.savez(closed_loop_save_path,
                 acc=acc_cl,
                 mag=mag_cl,
                 gyro=gyro_cl,
                 yaw_gt=yaw_gt_cl)
        
        print("\n🎉 [대성공] 20면 x 13눈금 = 260개 9축 통합 데이터셋 자율 수집 완료!")
        print(f"📁 [저장완료] 보정용 240포인트 ➔ {final_save_path}")
        print(f"📁 [저장완료] 평가용 260포인트 ➔ {closed_loop_save_path}\n")
        
        if os.path.exists(checkpoint_path):
            try:
                os.remove(checkpoint_path)
            except Exception:
                pass
                
    except KeyboardInterrupt:
        print("\n\n🛑 사용자에 의해 데이터 수집 과정이 강제 중지되었습니다.")
    finally:
        ser.close()
        print("🔌 포트 연결을 해제하고 세션을 종료합니다.")

if __name__ == "__main__":
    main()
