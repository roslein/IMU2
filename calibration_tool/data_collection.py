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
import numpy as np

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
    
    # 🎯 정20면체 법선 및 매칭 엔진 사전 로드
    import icosahedron
    normals = icosahedron.get_icosahedron_normals()
    
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
                
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key in [b'\r', b' ', b'\n']: # Enter or Space
                        print("\n\n🚀 [트리거 감지] 정밀 3초 수집 기동!")
                        break
                time.sleep(0.05)
                
            # 안정화 딜레이
            time.sleep(0.5)
            
            mean_acc, mean_gyro, mean_mag = collect_static_samples(ser, sample_count=300)
            
            # 🎯 실시간 정20면체 기하 NN 매칭 검증 피드백 주입
            best_idx, res = icosahedron.match_face(mean_acc, normals)
            match_percent = (1.0 - res) * 100.0
            
            print(f"   ↳ ⚖️ [평균 획득] Acc: [{mean_acc[0]:.1f}, {mean_acc[1]:.1f}, {mean_acc[2]:.1f}] | Mag: [{mean_mag[0]:.1f}, {mean_mag[1]:.1f}, {mean_mag[2]:.1f}]")
            print(f"   ↳ 🎯 [기하 매칭] 최종 자세 ➔ 정20면체 법선 #{best_idx:02d} 매칭됨 (일치율: {match_percent:.2f}%)")
            
            if best_idx in collected_data:
                print(f"   ⚠️ [경고] 최종 수집 결과가 이미 완료된 면(Face #{best_idx:02d})으로 유입되었습니다! 다른 면으로 다시 시도하십시오.")
            else:
                collected_data[best_idx] = (mean_acc, mean_mag)
                print(f"   ✅ [수집 성공] Face #{best_idx:02d} 데이터로 등록 완료!")
                
            print("-" * 60)
            
        # 수집 완료 후 인덱스 순서(0~19)대로 정렬 정렬하여 디스크 저장 (백업용)
        acc_samples = []
        mag_samples = []
        for i in range(20):
            acc_samples.append(collected_data[i][0])
            mag_samples.append(collected_data[i][1])
            
        acc_samples = np.array(acc_samples)
        mag_samples = np.array(mag_samples)
        
        np.savez("calibration_tool/collected_data.npz", acc=acc_samples, mag=mag_samples)
        print("\n🎉 [대성공] 20개 포지션 데이터 수집이 완전히 끝났습니다!")
        print("📁 수집본 저장 완료: calibration_tool/collected_data.npz\n")
        
    except KeyboardInterrupt:
        print("\n\n🛑 사용자에 의해 데이터 수집 과정이 강제 중지되었습니다.")
    finally:
        ser.close()
        print("🔌 포트 연결을 해제하고 세션을 종료합니다.")

if __name__ == "__main__":
    main()
