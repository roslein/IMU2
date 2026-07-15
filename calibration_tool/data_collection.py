"""
Real-world IMU Phase 2 Data Collection (data_collection.py)
목적: 정20면체의 20개 면에 센서를 차례로 거치하고, 수평 회전판 12눈금(30도 간격)
      결합을 통해 총 240개 포인트(20면 x 12눈금)의 9축 원시 데이터를 수집합니다.
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

def update_3d_plot(collected_data, normals, new_point=None, new_matched_idx=None):
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
        if idx in collected_data:
            ax.scatter(n[0], n[1], n[2], color='green', s=100, marker='o', label='Collected' if idx == 0 else "")
            ax.text(n[0] * 1.15, n[1] * 1.15, n[2] * 1.15, f"#{idx:02d} (G)", color='darkgreen', fontsize=9, weight='bold')
            ax.plot([0, n[0]], [0, n[1]], [0, n[2]], color='green', alpha=0.5, linewidth=1.5)
        else:
            ax.scatter(n[0], n[1], n[2], color='red', s=60, marker='x', label='Uncollected' if idx == 0 else "")
            ax.text(n[0] * 1.15, n[1] * 1.15, n[2] * 1.15, f"#{idx:02d}", color='darkred', fontsize=9)
            ax.plot([0, n[0]], [0, n[1]], [0, n[2]], color='red', alpha=0.3, linestyle='--', linewidth=1.0)
            
    if new_point is not None:
        norm_val = np.linalg.norm(new_point)
        if norm_val > 1e-3:
            new_unit = - (new_point / norm_val)
            ax.scatter(new_unit[0], new_unit[1], new_unit[2], color='gold', s=180, marker='*', edgecolors='black', linewidths=1.0, label='New Position')
            ax.plot([0, new_unit[0]], [0, new_unit[1]], [0, new_unit[2]], color='gold', linewidth=3.0)
            
            if new_matched_idx is not None:
                n_target = normals[new_matched_idx]
                ax.plot([new_unit[0], n_target[0]], [new_unit[1], n_target[1]], [new_unit[2], n_target[2]], color='orange', linestyle=':', linewidth=2.0)
                
    ax.set_title("📌 IMU 20-Position Calibration Guide View (3D)")
    ax.set_xlabel("X-Axis")
    ax.set_ylabel("Y-Axis")
    ax.set_zlabel("Z-Axis")
    ax.grid(True)
    
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper left')
    
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
    print(" 🎯 Real-world IMU Phase 2 20-Position x 12-Yaw Data Collector (v0.3.0)")
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
        
    collected_acc = []
    collected_mag = []
    collected_gyro = []
    collected_yaw_gt = []
    
    start_face = 0
    start_yaw = 0
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    final_save_path = os.path.join(output_dir, "collected_data_9axis.npz")
    if os.path.exists(final_save_path):
        try:
            mtime = os.path.getmtime(final_save_path)
            time_struct = time.localtime(mtime)
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time_struct)
            backup_name = f"collected_data_9axis_backup_{timestamp_str}.npz"
            backup_path = os.path.join(output_dir, backup_name)
            os.rename(final_save_path, backup_path)
            print(f"\n📁 [자동 백업 완료] 기존 완성본을 감지하여 안전하게 백업했습니다.")
            print(f"   ➔ 백업 파일: output/{backup_name}")
        except Exception as e:
            print(f"\n⚠️ [백업 실패] 기존 데이터 백업 중 오류 발생: {e}")
            
    checkpoint_path = os.path.join(output_dir, "checkpoint_data_9axis.npz")
    if os.path.exists(checkpoint_path):
        print("\n⚠️ [임시 복원 데이터 감지] 수집 중이던 9축 데이터가 존재합니다.")
        ans = input("   👉 기존 데이터를 이어서 수집하시겠습니까? (Y/N): ").strip().lower()
        if ans in ['', 'y', 'yes']:
            try:
                with np.load(checkpoint_path) as data:
                    collected_acc = list(data['acc'])
                    collected_mag = list(data['mag'])
                    collected_gyro = list(data['gyro'])
                    collected_yaw_gt = list(data['yaw_gt'])
                    start_face = int(data['start_face'])
                    start_yaw = int(data['start_yaw'])
                print(f"   ✅ [복원 성공] {len(collected_acc)}개 포인트 데이터를 이어서 수집합니다! (면 #{start_face:02d}, 눈금 {start_yaw * 30.0:.1f}°부터 시작)")
            except Exception as e:
                print(f"   ⚠️ [복원 실패] 임시 데이터 로드 오류: {e}. 새로 수집을 개시합니다.")
                collected_acc = []
                collected_mag = []
                collected_gyro = []
                collected_yaw_gt = []
        else:
            print("   ➔ 임시 데이터를 무시하고 새로 수집을 개시합니다.")
            
    from imu_core import icosahedron
    normals = icosahedron.get_rotated_normals()
    
    gui_collected = {}
    for idx in range(len(collected_acc)):
        f_idx = idx // 12
        gui_collected[f_idx] = (collected_acc[idx], collected_mag[idx])
        
    init_3d_plot(normals)
    update_3d_plot(gui_collected, normals)
    
    print("\n💡 정20면체 20개 각 면의 안착과 회전판 12개 눈금(30도 간격) 결합 수집을 개시합니다.")
    print("💡 총 240개 포인트(20면 x 12눈금)가 수집될 예정입니다.\n")
    
    try:
        import msvcrt
        for face_idx in range(start_face, 20):
            print(f"\n" + "=" * 70)
            print(f" 📂 [면 안착 가이드] 정20면체 지그의 #{face_idx:02d}번 면을 수평 바닥에 안착시키십시오.")
            print(f"=============================================================")
            
            ser.reset_input_buffer()
            print("📡 실시간 법선 매칭 검증 중... 지그 안착 완료 후 [Enter] 또는 [Space] 키를 누르십시오.")
            last_plot_time = 0
            while True:
                acc_raw, mag_raw = get_latest_sample(ser)
                if acc_raw is not None:
                    best_idx, res = icosahedron.match_face(acc_raw, normals)
                    match_percent = (1.0 - res) * 100.0
                    
                    if best_idx == face_idx:
                        status_str = f"올바른 면 감지 성공!! ✅ (Face #{best_idx:02d})"
                    else:
                        status_str = f"잘못된 면 거치 ❌ (타겟: #{face_idx:02d} ➔ 실측 감지: #{best_idx:02d})"
                        
                    sys.stdout.write(f"\r📡 실시간 프리뷰 ➔ {status_str} | 일치율: {match_percent:.1f}%")
                    sys.stdout.flush()
                    
                    if time.time() - last_plot_time > 0.15:
                        update_3d_plot(gui_collected, normals, new_point=acc_raw, new_matched_idx=best_idx)
                        last_plot_time = time.time()
                
                plt.pause(0.01)
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key in [b'\r', b' ', b'\n']:
                        print("\n👉 면 안착 확인 완료! 회전판 12개 눈금 시퀀스를 개시합니다.")
                        break
                time.sleep(0.01)
                
            yaw_start_loop = start_yaw if face_idx == start_face else 0
            for yaw_idx in range(yaw_start_loop, 12):
                yaw_target_deg = yaw_idx * 30.0
                print(f"\n👉 [현재 수집 타겟] 면 #{face_idx:02d} | 회전판 눈금: {yaw_target_deg:.1f}°")
                print("📡 회전판 눈금을 정밀 조작하여 정지한 후 [Space] 또는 [Enter] 키를 누르십시오...")
                
                ser.reset_input_buffer()
                last_plot_time = 0
                while True:
                    acc_raw, mag_raw = get_latest_sample(ser)
                    if acc_raw is not None:
                        best_idx, res = icosahedron.match_face(acc_raw, normals)
                        sys.stdout.write(f"\r📡 실시간 모니터링 ➔ 면 #{best_idx:02d} 거치 중... (안정 정지 대기)")
                        sys.stdout.flush()
                        
                        if time.time() - last_plot_time > 0.15:
                            update_3d_plot(gui_collected, normals, new_point=acc_raw, new_matched_idx=best_idx)
                            last_plot_time = time.time()
                            
                    plt.pause(0.01)
                    if msvcrt.kbhit():
                        key = msvcrt.getch()
                        if key in [b'\r', b' ', b'\n']:
                            print(f"\n🚀 [수집 기동] 면 #{face_idx:02d} | 눈금 {yaw_target_deg:.1f}° 1.5초 수집 시작!")
                            break
                    time.sleep(0.01)
                    
                time.sleep(0.3)
                
                mean_acc, mean_gyro, mean_mag = collect_static_samples(ser, sample_count=150)
                
                gui_collected[face_idx] = (mean_acc, mean_mag)
                update_3d_plot(gui_collected, normals)
                
                collected_acc.append(mean_acc)
                collected_mag.append(mean_mag)
                collected_gyro.append(mean_gyro)
                collected_yaw_gt.append(yaw_target_deg)
                
                print(f"   ↳ ⚖️ [평균 획득] Acc: [{mean_acc[0]:.1f}, {mean_acc[1]:.1f}, {mean_acc[2]:.1f}] | Mag: [{mean_mag[0]:.1f}, {mean_mag[1]:.1f}, {mean_mag[2]:.1f}]")
                print(f"   ↳ [진행 상태] {len(collected_acc)} / 240 포인트 완료")
                
                try:
                    next_face = face_idx
                    next_yaw = yaw_idx + 1
                    if next_yaw == 12:
                        next_face += 1
                        next_yaw = 0
                        
                    np.savez(checkpoint_path,
                             acc=np.array(collected_acc),
                             mag=np.array(collected_mag),
                             gyro=np.array(collected_gyro),
                             yaw_gt=np.array(collected_yaw_gt),
                             start_face=next_face,
                             start_yaw=next_yaw)
                except Exception as e:
                    print(f"   ⚠️ [체크포인트 백업 에러]: {e}")
                    
        np.savez(final_save_path, 
                 acc=np.array(collected_acc), 
                 mag=np.array(collected_mag), 
                 gyro=np.array(collected_gyro), 
                 yaw_gt=np.array(collected_yaw_gt))
        
        print("\n🎉 [대성공] 20면 x 12눈금 = 240개 9축 통합 데이터셋 수집이 완전히 완료되었습니다!")
        print(f"📁 저장 경로: {final_save_path}\n")
        
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
