"""
Real-world IMU Phase 2 Data Collection (data_collection.py)
목적: 정20면체의 20개 면에 센서를 차례로 거치하고, 사용자의 명시적인 입력 트리거에 맞춰
      100Hz 바이너리 스트림에서 3초(300개 패킷) 동안 데이터를 평균화하여
      최적화 솔버용 노이즈 프리 3축 가속도 & 자력 데이터 포인트를 20개 수집합니다.
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

# 🎯 3D Matplotlib 창 및 플롯 관리를 위한 전역 변수
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
    """
    3D Matplotlib 창을 초기화하고 정20면체 법선 벡터 위치 및 면 번호를 3D 공간에 가상 드로잉합니다.
    """
    global fig, ax
    plt.ion()  # 대화형 모드 활성화
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    plt.show()

def update_3d_plot(collected_data, normals, new_point=None, new_matched_idx=None):
    """
    3D 공간 상에 이미 수집된 벡터(Green), 미수집 법선(Red), 신규 측정 후보 벡터(Yellow)를 맵핑합니다.
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
    
    # 2. 20개 정20면체 법선 벡터 그리기
    for idx, n in enumerate(normals):
        if idx in collected_data:
            ax.scatter(n[0], n[1], n[2], color='green', s=100, marker='o', label='Collected' if idx == 0 else "")
            ax.text(n[0] * 1.15, n[1] * 1.15, n[2] * 1.15, f"#{idx:02d} (G)", color='darkgreen', fontsize=9, weight='bold')
            ax.plot([0, n[0]], [0, n[1]], [0, n[2]], color='green', alpha=0.5, linewidth=1.5)
        else:
            ax.scatter(n[0], n[1], n[2], color='red', s=60, marker='x', label='Uncollected' if idx == 0 else "")
            ax.text(n[0] * 1.15, n[1] * 1.15, n[2] * 1.15, f"#{idx:02d}", color='darkred', fontsize=9)
            ax.plot([0, n[0]], [0, n[1]], [0, n[2]], color='red', alpha=0.3, linestyle='--', linewidth=1.0)
            
    # 3. 새로운 포인트 후보 시각화 (중력 가속도 부호 반전하여 구면에 정합)
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
    
    # 범례 설정
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper left')
    
    plt.draw()
    plt.pause(0.01)

def get_latest_sample(ser):
    """
    시리얼 버퍼에서 가장 최근의 유효한 1개 패킷을 빠르게 추출하여 반환합니다.
    """
    try:
        in_waiting = ser.in_waiting
    except Exception:
        return None, None
        
    if in_waiting > 1000:
        try:
            ser.read(in_waiting - 100)  # 이전 쓰레기 데이터 버스트 버림
        except Exception:
            return None, None
            
    byte_buffer = bytearray()
    start_time = time.time()
    while time.time() - start_time < 0.2:  # 0.2초 타임아웃
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

def collect_static_samples(ser, sample_count=300):
    """
    시리얼 포트로부터 정확히 sample_count개의 유효한 39-Byte 패킷을 수집하여
    가속도 및 자력 데이터의 3축 원시 평균 벡터를 반환합니다.
    """
    byte_buffer = bytearray()
    collected_acc = []
    collected_mag = []
    collected_gyro = []
    
    # 입력 버퍼 비우기
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
                
            # XOR 체크섬
            xor_sum = START_BYTE
            for b in packet[1:37]:
                xor_sum ^= b
                
            if xor_sum != packet[37]:
                byte_buffer = byte_buffer[PACKET_SIZE:]
                continue
                
            # 복조
            data_payload = packet[1:37]
            floats = struct.unpack('<9f', data_payload)
            
            collected_acc.append(floats[0:3])
            collected_gyro.append(floats[3:6])
            collected_mag.append(floats[6:9])
            
            # 처리 패킷 비우기
            byte_buffer = byte_buffer[PACKET_SIZE:]
            
            # 수집 게이지 가시화
            progress = len(collected_acc)
            bar = "=" * (progress * 30 // sample_count) + " " * (30 - progress * 30 // sample_count)
            sys.stdout.write(f"\r📥 데이터 수집 중: [{bar}] {progress}/{sample_count} 패킷 완료")
            sys.stdout.flush()
            
        time.sleep(0.001)
        
    print()
    # 노이즈를 감쇄한 평균 벡터 계산
    mean_acc = np.mean(collected_acc, axis=0)
    mean_gyro = np.mean(collected_gyro, axis=0)
    mean_mag = np.mean(collected_mag, axis=0)
    
    return mean_acc, mean_gyro, mean_mag

def main():
    print("=" * 60)
    print(" 🎯 Real-world IMU Phase 2 20-Position Data Collector")
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
        
    collected_data = {}  # face_idx -> (mean_acc, mean_mag)
    
    # 🎯 실시간 임시 체크포인트 자동 복원 메커니즘
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    # 기존 완성 데이터가 있을 경우 자동 안전 백업
    final_save_path = os.path.join(output_dir, "collected_data.npz")
    if os.path.exists(final_save_path):
        try:
            mtime = os.path.getmtime(final_save_path)
            time_struct = time.localtime(mtime)
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time_struct)
            backup_name = f"collected_data_backup_{timestamp_str}.npz"
            backup_path = os.path.join(output_dir, backup_name)
            os.rename(final_save_path, backup_path)
            print(f"\n📁 [자동 백업 완료] 기존 완성본을 감지하여 안전하게 백업했습니다.")
            print(f"   ➔ 백업 파일: output/{backup_name}")
        except Exception as e:
            print(f"\n⚠️ [백업 실패] 기존 데이터 백업(이름 변경) 중 오류 발생: {e}")
            
    checkpoint_path = os.path.join(output_dir, "checkpoint_data.npz")
    if os.path.exists(checkpoint_path):
        print("\n⚠️ [임시 복원 데이터 감지] 이전에 수집 중이던 데이터가 파일로 보존되어 있습니다.")
        ans = input("   👉 기존 데이터를 이어서 수집하시겠습니까? (Y/N): ").strip().lower()
        if ans in ['', 'y', 'yes']:
            try:
                with np.load(checkpoint_path) as data:
                    faces = data['faces']
                    acc_data = data['acc']
                    mag_data = data['mag']
                    gyro_data = data['gyro'] if 'gyro' in data else None
                    for idx, f in enumerate(faces):
                        g_val = gyro_data[idx] if gyro_data is not None else np.zeros(3)
                        collected_data[int(f)] = (acc_data[idx], mag_data[idx], g_val)
                print(f"   ✅ [복원 성공] 총 {len(collected_data)}개 면 데이터를 이어서 시작합니다!")
            except Exception as e:
                print(f"   ⚠️ [복원 실패] 임시 데이터 로드 중 오류: {e}. 새로 수집을 개시합니다.")
        else:
            print("   ➔ 임시 데이터를 무시하고 새로 수집을 개시합니다.")
            
    # 🎯 정20면체 센서 기준 회전 법선 사전 로드
    import icosahedron
    normals = icosahedron.get_rotated_normals()
    
    # 🎯 3D 뷰어 창 초기화
    init_3d_plot(normals)
    update_3d_plot(collected_data, normals)
    
    print("\n💡 정20면체의 각 20개 면을 수평 바닥에 정적으로 안착시킨 후 수집을 시작합니다.")
    print("중복/순서 상관없이 총 20개의 서로 다른 면을 한 번씩 바닥에 수평 안착시키십시오.\n")
    
    try:
        while len(collected_data) < 20:
            remaining = [f for f in range(20) if f not in collected_data]
            print(f"\n👉 [남은 면 ({len(remaining)}개)]: {remaining}")
            print(f"👉 [현재 진행률: {len(collected_data)} / 20]")
            print("📡 다면체를 안착시키며 실시간 상태를 확인하십시오.")
            print("   - [미수집 새 면!! ✅]이 감지되면 [Space] 또는 [Enter] 키를 눌러 3초간 정밀 수집을 시작합니다.")
            print("   - [Ctrl+C]를 누르면 언제든지 수집이 강제 종료됩니다.")
            
            # 실시간 프리뷰 루프
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
                    
                    # 3D 뷰어 5Hz 갱신
                    if time.time() - last_plot_time > 0.15:
                        update_3d_plot(collected_data, normals, new_point=acc_raw, new_matched_idx=best_idx)
                        last_plot_time = time.time()
                
                # Matplotlib GUI 스레드 활성화 (회전 및 제어 지원)
                plt.pause(0.01)
                
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key in [b'\r', b' ', b'\n']: # Enter or Space
                        print("\n\n🚀 [트리거 감지] 정밀 3초 수집 기동!")
                        break
                time.sleep(0.01)
                
            # 안정화 딜레이
            time.sleep(0.5)
            
            mean_acc, mean_gyro, mean_mag = collect_static_samples(ser, sample_count=300)
            
            # 🎯 획득된 정밀 평균 데이터 기준으로 3D 가이드 업데이트
            best_idx, res = icosahedron.match_face(mean_acc, normals)
            match_percent = (1.0 - res) * 100.0
            update_3d_plot(collected_data, normals, new_point=mean_acc, new_matched_idx=best_idx)
            
            print(f"   ↳ ⚖️ [평균 획득] Acc: [{mean_acc[0]:.1f}, {mean_acc[1]:.1f}, {mean_acc[2]:.1f}] | Mag: [{mean_mag[0]:.1f}, {mean_mag[1]:.1f}, {mean_mag[2]:.1f}]")
            print(f"   ↳ 🎯 [기하 매칭] 자동 판정 ➔ 정20면체 법선 #{best_idx:02d} 매칭됨 (일치율: {match_percent:.2f}%)")
            
            # 🎯 수동 수치 보정 및 검증 입력 주입
            print("   ❔ [수동 보정] 3D 구면 안내도를 보시고 자동 판정이 올바른지 검증하십시오.")
            override_input = input(f"       * 매칭이 올바르면 [Enter], 왜곡되어 틀릴 시 올바른 면 번호(0-19)를 직접 입력하십시오: ").strip()
            
            final_idx = best_idx
            if override_input.isdigit():
                val = int(override_input)
                if 0 <= val <= 19:
                    final_idx = val
                    print(f"       ➔ 🛠 [수동 보정 적용] Face #{final_idx:02d} 번으로 변경 등록합니다.")
                else:
                    print(f"       ⚠️ [범위 초과] 0~19 외의 값이므로 자동 판정값(#{best_idx:02d})으로 등록합니다.")
            
            if final_idx in collected_data:
                print(f"   ⚠️ [경고] 등록하려는 면(Face #{final_idx:02d})은 이미 수집 완료된 상태입니다! 다른 면으로 다시 시도하십시오.")
            else:
                collected_data[final_idx] = (mean_acc, mean_mag, mean_gyro)
                print(f"   ✅ [수집 성공] Face #{final_idx:02d} 데이터로 등록 완료!")
                
                # 🎯 실시간 임시 체크포인트 백업
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
                
            # 최종 수집 및 갱신 완료 후 3D 플롯 다시 갱신
            update_3d_plot(collected_data, normals)
            print("-" * 60)
            
        # 수집 완료 후 인덱스 순서(0~19)대로 정렬 정렬하여 디스크 저장 (백업용)
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
        
        final_save_path = os.path.join(output_dir, "collected_data.npz")
        np.savez(final_save_path, acc=acc_samples, mag=mag_samples, gyro=gyro_samples)
        print("\n🎉 [대성공] 20개 포지션 데이터 수집이 완전히 끝났습니다!")
        print(f"📁 수집본 저장 완료: {final_save_path}\n")
        
        # 🎯 완수 후 임시 체크포인트 자동 제거
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
