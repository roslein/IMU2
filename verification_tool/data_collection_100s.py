"""
Real-world IMU Phase 2 Multi-Second Data Collection (data_collection_100s.py)
목적: 정20면체의 20개 면에 센서를 차례로 거치하고, 사용자의 명시적인 입력 트리거에 맞춰
      100Hz 바이너리 스트림에서 지정된 시간(초) 동안 시계열 데이터를 누적 수집하여
      윈도우 분석용 20xNx3 3차원 데이터셋(collected_data_100s.npz)을 빌드합니다.
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
sys.path.append(os.path.join(IMU_ROOT, 'calibration_tool'))

# 🎯 수집 제어 상수 (이 시간만 수정하면 자동으로 모든 수집 샘플 개수가 연동됨)
COLLECT_TIME_SEC = 10.0
EXPECTED_SAMPLES = int(COLLECT_TIME_SEC * 100)  # 100Hz 기준 수집 샘플수

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
                
    ax.set_title(f"[GUIDE] IMU 20-Position {COLLECT_TIME_SEC}s Data Collection Guide (3D)")
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

def collect_100s_samples(ser, sample_count):
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
            sys.stdout.write(f"\r📥 {COLLECT_TIME_SEC}초 데이터 수집 중: [{bar}] {progress}/{sample_count} 패킷 완료 ({progress*100//sample_count}%)")
            sys.stdout.flush()
            
        time.sleep(0.001)
        
    print()
    return np.array(collected_acc), np.array(collected_gyro), np.array(collected_mag)

def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')

    print("=" * 60)
    print(f" 🎯 Real-world IMU Phase 2 {COLLECT_TIME_SEC}-Second Data Collector")
    print("=" * 60)
    print("⚠️ [치명적 경고] 이 스크립트는 캘리브레이션 윈도우 스캔용 원시 데이터를 수집합니다.")
    print("⚠️ 반드시 보드에 raw.ino 펌웨어가 업로드되어 스트리밍 중인지 확인하십시오.")
    print("⚠️ calibrated.ino(보정본) 상태로 수집 시, 이중 보정으로 스캔 결과가 파멸적으로 왜곡됩니다.")
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
        
        # 아두이노 초기 에러(I2C 연결 실패 등) 체크 가드 로직
        time.sleep(1.0)
        if ser.in_waiting > 0:
            test_data = ser.read(ser.in_waiting)
            try:
                test_str = test_data.decode('utf-8', errors='ignore')
                if "Error:" in test_str or "실패" in test_str:
                    print(f"\n❌ [아두이노 하드웨어 에러 감지]: {test_str.strip()}")
                    ser.close()
                    sys.exit(1)
            except Exception:
                pass
    except Exception as e:
        print(f"❌ 포트 연결 실패: {e}")
        sys.exit(1)
        
    collected_data = {}
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    checkpoint_path = os.path.join(output_dir, "checkpoint_data_100s.npz")
    
    if os.path.exists(checkpoint_path):
        print(f"\n⚠️ [임시 복원 데이터 감지] {COLLECT_TIME_SEC}초 수집 복원 파일이 존재합니다.")
        ans = input("   👉 기존 데이터를 이어서 수집하시겠습니까? (Y/N): ").strip().lower()
        if ans in ['', 'y', 'yes']:
            try:
                with np.load(checkpoint_path) as data:
                    faces = data['faces']
                    acc_data = data['acc']
                    mag_data = data['mag']
                    gyro_data = data['gyro']
                    
                    # 로드된 데이터와 현재 상수가 일치하는지 확인
                    if acc_data.shape[1] == EXPECTED_SAMPLES:
                        for idx, f in enumerate(faces):
                            collected_data[int(f)] = (acc_data[idx], mag_data[idx], gyro_data[idx])
                        print(f"   ✅ [복원 성공] 총 {len(collected_data)}개 면 데이터를 이어서 시작합니다!")
                    else:
                        print(f"   ⚠️ [복원 실패] 백업된 샘플 수({acc_data.shape[1]})와 현재 설정 상수({EXPECTED_SAMPLES})가 불일치함. 새로 시작합니다.")
            except Exception as e:
                print(f"   ⚠️ [복원 실패] 임시 데이터 로드 오류: {e}. 새로 시작합니다.")
        else:
            print("   ➔ 새로 수집을 개시합니다.")
            
    import icosahedron
    normals = icosahedron.get_rotated_normals()
    
    init_3d_plot(normals)
    update_3d_plot(collected_data, normals)
    
    print(f"\n💡 면당 {COLLECT_TIME_SEC}초({EXPECTED_SAMPLES}샘플) 정밀 거치 원시 데이터를 수집합니다.")
    print("중복/순서 상관없이 총 20개의 서로 다른 면을 안착시켜 주십시오.\n")
    
    try:
        while len(collected_data) < 20:
            remaining = [f for f in range(20) if f not in collected_data]
            print(f"\n👉 [남은 면 ({len(remaining)}개)]: {remaining}")
            print(f"👉 [현재 진행률: {len(collected_data)} / 20]")
            
            ser.reset_input_buffer()
            import msvcrt
            last_plot_time = 0
            while True:
                acc_raw, mag_raw = get_latest_sample(ser)
                if acc_raw is not None:
                    best_idx, res = icosahedron.match_face(acc_raw, normals)
                    match_percent = (1.0 - res) * 100.0
                    
                    if best_idx in collected_data:
                        status_str = f"이미 수집 완료 ❌ (Face #{best_idx:02d})"
                    else:
                        status_str = f"미수집 새 면!! ✅ (Face #{best_idx:02d})"
                        
                    sys.stdout.write(f"\r📡 실시간 프리뷰 ➔ {status_str} | 일치율: {match_percent:.1f}%")
                    sys.stdout.flush()
                    
                    if time.time() - last_plot_time > 0.15:
                        update_3d_plot(collected_data, normals, new_point=acc_raw, new_matched_idx=best_idx)
                        last_plot_time = time.time()
                
                plt.pause(0.01)
                
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key in [b'\r', b' ', b'\n']:
                        print(f"\n\n🚀 [트리거 감지] {COLLECT_TIME_SEC}초({EXPECTED_SAMPLES}패킷) 연속 수집을 시작합니다. 절대 건드리지 마십시오!")
                        break
                time.sleep(0.01)
                
            time.sleep(0.5)
            
            acc_series, gyro_series, mag_series = collect_100s_samples(ser, sample_count=EXPECTED_SAMPLES)
            
            mean_acc = np.mean(acc_series, axis=0)
            best_idx, res = icosahedron.match_face(mean_acc, normals)
            match_percent = (1.0 - res) * 100.0
            update_3d_plot(collected_data, normals, new_point=mean_acc, new_matched_idx=best_idx)
            
            print(f"   ↳ 🎯 [기하 매칭 자동 판정] 법선 #{best_idx:02d} 매칭됨 (일치율: {match_percent:.2f}%)")
            override_input = input(f"       * 매칭이 올바르면 [Enter], 다를 시 면 번호(0-19)를 입력하십시오: ").strip()
            
            final_idx = best_idx
            if override_input.isdigit():
                val = int(override_input)
                if 0 <= val <= 19:
                    final_idx = val
                    print(f"       ➔ 🛠 [수동 보정] Face #{final_idx:02d} 번으로 변경 등록합니다.")
            
            if final_idx in collected_data:
                print(f"   ⚠️ [경고] 이미 수집 완료된 면입니다! 다시 시도하십시오.")
            else:
                collected_data[final_idx] = (
                    acc_series[:EXPECTED_SAMPLES], 
                    mag_series[:EXPECTED_SAMPLES], 
                    gyro_series[:EXPECTED_SAMPLES]
                )
                print(f"   ✅ [수집 성공] Face #{final_idx:02d} {COLLECT_TIME_SEC}초 데이터 등록 완료!")
                
                try:
                    faces_to_save = list(collected_data.keys())
                    acc_to_save = [collected_data[f][0] for f in faces_to_save]
                    mag_to_save = [collected_data[f][1] for f in faces_to_save]
                    gyro_to_save = [collected_data[f][2] for f in faces_to_save]
                    np.savez(checkpoint_path, 
                             faces=np.array(faces_to_save), 
                             acc=np.array(acc_to_save), 
                             mag=np.array(mag_to_save),
                             gyro=np.array(gyro_to_save))
                except Exception as e:
                    print(f"   ⚠️ [체크포인트 실시간 백업 실패]: {e}")
                
            update_3d_plot(collected_data, normals)
            print("-" * 60)
            
        acc_samples = []
        mag_samples = []
        gyro_samples = []
        for i in range(20):
            acc_samples.append(collected_data[i][0])
            mag_samples.append(collected_data[i][1])
            gyro_samples.append(collected_data[i][2])
            
        acc_samples = np.array(acc_samples)
        mag_samples = np.array(mag_samples)
        gyro_samples = np.array(gyro_samples)
        
        final_save_path = os.path.join(output_dir, "collected_data_100s.npz")
        np.savez(final_save_path, acc=acc_samples, mag=mag_samples, gyro=gyro_samples)
        print(f"\n🎉 [대성공] 20개 포지션 {COLLECT_TIME_SEC}초 시계열 데이터 수집 완료!")
        print(f"📁 파일 경로: {final_save_path}\n")
        
        if os.path.exists(checkpoint_path):
            try:
                os.remove(checkpoint_path)
            except Exception:
                pass
        
    except KeyboardInterrupt:
        print("\n\n🛑 사용자에 의해 강제 중지되었습니다.")
    finally:
        ser.close()
        print("🔌 포트 연결을 해제하고 세션을 종료합니다.")

if __name__ == "__main__":
    main()
