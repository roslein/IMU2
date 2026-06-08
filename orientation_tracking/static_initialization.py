"""
Real-world IMU Phase 3 Static Orientation Estimation (static_initialization.py)
목적: 보정 완료된 센서 데이터(calibrated.ino)의 바이너리 텔레메트리 스트림을 수집하고,
      5.0초(500개 패킷) 동안 데이터를 누적하여 Box LPF 평균 연산을 수행한 뒤,
      TRIAD-NED 기법을 적용해 쿼터니언 기반 3D 정적 절대 자세(Roll, Pitch, Yaw)를 도출합니다.
"""

import serial
import serial.tools.list_ports
import struct
import time
import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

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

def main():
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8')

    print("=" * 60)
    print(" 🎯 Real-world IMU Phase 3 Static Orientation Estimator")
    print("=" * 60)
    print("⚠️ [필독] 기동 전, EKF 트래킹용 펌웨어(calibrated.ino)가 보드에 업로드되어")
    print("⚠️ 실시간으로 보정 바이너리 텔레메트리를 송출하고 있는지 반드시 확인하십시오.")
    print("=" * 60)

    # 1. 시리얼 포트 탐색 및 연결
    port = find_arduino_port()
    if not port:
        print("❌ 사용 가능한 시리얼 포트를 감지하지 못했습니다.")
        return
        
    print(f"📡 시리얼 COM 포트 감지 성공: {port}")
    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        time.sleep(1.5)  # ESP32 부팅 및 시리얼 연결 안정화 대기
    except Exception as e:
        print(f"❌ 포트 점유 또는 연결 실패: {e}")
        return

    # 2. 수집 시간 설정 (최적 윈도우 시간 가이드라인 반영: 5.0초)
    collect_time_sec = 5.0
    expected_samples = int(collect_time_sec * 100) # 100Hz 기준 500개 샘플
    
    print(f"💡 정적 윈도우 분석 최적 가이드라인에 따라 {collect_time_sec}초간 수집을 진행합니다.")
    print("💡 20면체 지그 혹은 센서를 완전히 정지 및 무회전 상태로 거치해 주십시오.")
    input("\n👉 방향 측정을 즉시 개시하려면 [Enter] 키를 누르십시오...")

    # 3. 데이터 스트리밍 누적 수집 개시
    collected_acc = []
    collected_mag = []
    
    byte_buffer = bytearray()
    ser.reset_input_buffer()
    
    print("\n🚀 [수집 기동] 3축 보정 가속도 및 자력 데이터 스트리밍 적립 중...")
    
    start_time = time.time()
    
    while len(collected_acc) < expected_samples:
        # 비접촉 타임아웃 감지 예외 처리
        if time.time() - start_time > collect_time_sec + 2.0 and len(collected_acc) == 0:
            print("❌ 시리얼 스트리밍 패킷 수신 타임아웃! 펌웨어가 calibrated.ino 인지 확인하십시오.")
            ser.close()
            return
            
        try:
            in_waiting = ser.in_waiting
            if in_waiting > 0:
                data = ser.read(in_waiting)
                byte_buffer.extend(data)
        except Exception as e:
            print(f"❌ 데이터 수신 버스트 에러: {e}")
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
                # XOR 체크섬 정밀 복조 검증
                xor_sum = START_BYTE
                for b in packet[1:37]:
                    xor_sum ^= b
                if xor_sum == packet[37]:
                    # 9개 float 추출 (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)
                    floats = struct.unpack('<9f', packet[1:37])
                    collected_acc.append(floats[0:3])
                    collected_mag.append(floats[6:9])
                    
                    # 텍스트 실시간 프로그레스 바 렌더링
                    progress = len(collected_acc)
                    percent = int((progress / expected_samples) * 100)
                    bar = '=' * int(progress / (expected_samples / 30))
                    sys.stdout.write(f"\r📥 데이터 수집 중: [{bar:<30}] {progress}/{expected_samples} 패킷 ({percent}%)")
                    sys.stdout.flush()
                    
            byte_buffer = byte_buffer[PACKET_SIZE:]
            
        time.sleep(0.002)
        
    ser.close()
    print("\n✅ 데이터 수집 완료! 시리얼 채널이 안전하게 종료되었습니다.")

    # 4. Box LPF 평균 연산 처리 (시간축 잡음 및 센서 유도 노이즈 극소화)
    acc_avg = np.mean(collected_acc, axis=0)
    mag_avg = np.mean(collected_mag, axis=0)
    
    print("\n⚖️ [Box LPF 노이즈 최소화 실측 평균]")
    print(f"   ↳ acc_avg [g]:   {acc_avg}")
    print(f"   ↳ mag_avg [norm]: {mag_avg}")

    # 5. SVD 기반 Wahba 문제 최소제곱 정밀 정합 (특이점/Sign flip 완전 회피)
    # 3D 자북 레퍼런스 (m_ned_ref) 로드
    calib_tool_dir = os.path.join(IMU_ROOT, "calibration_tool")
    env_param_path = os.path.join(calib_tool_dir, "output", "env_params.npz")
    
    if os.path.exists(env_param_path):
        env_params = np.load(env_param_path)
        m_ned_ref = env_params["m_ned_ref"]
        print(f"📡 로컬 환경 지자기 지도 로드 완료 (m_ned_ref: {m_ned_ref})")
    else:
        # Fallback: 표준 서울 복각(55도) 가정 레퍼런스 [cos(55), 0, sin(55)]
        m_ned_ref = np.array([0.573576, 0.0, 0.819152])
        print("⚠️  환경 지자기 지도(env_params.npz) 유실 ➔ [서울 표준 지자기 복각 55도 Fallback 적용]")
        print(f"📡 임시 지자기 레퍼런스 (m_ned_ref): {m_ned_ref}")
        
    # 가속도/자력 실측 벡터 정규화
    g_sensor = acc_avg / np.linalg.norm(acc_avg)
    m_sensor = mag_avg / np.linalg.norm(mag_avg)
    
    # 1 단계: SVD 기반 Wahba 정합을 통해 센서에서 지구 NED 프레임으로의 최적 회전 복조
    # (acc_avg는 중력 Down을 의미하므로 ideal Down [0,0,1]에 정합)
    v_sensor = np.array([g_sensor, m_sensor])
    v_ned = np.array([np.array([0.0, 0.0, 1.0]), m_ned_ref])
    
    res_rot, _ = R_scipy.align_vectors(v_ned, v_sensor)
    R_body_to_ned = res_rot.as_matrix()
    
    # 2 단계: 쿼터니언 qw, qx, qy, qz 상태 추출
    q_scipy = res_rot.as_quat() # [x, y, z, w]
    q_final = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]]) # [qw, qx, qy, qz]
    
    # 3 단계: 3축 오일러각 (XYZ 오더) 역산
    euler_deg = res_rot.as_euler('xyz', degrees=True)
    roll = euler_deg[0]
    pitch = euler_deg[1]
    yaw = euler_deg[2]

    print("\n" + "=" * 60)
    print(" 🎉 SVD-Wahba 정적 3D 자세 추정 완료 보고")
    print("=" * 60)
    print(f"📐 실측 절대 자세 오일러각 (Roll, Pitch, Yaw):")
    print(f"   ↳ Roll  (X축 회전각): {roll:10.4f}°")
    print(f"   ↳ Pitch (Y축 회전각): {pitch:10.4f}°")
    print(f"   ↳ Yaw   (Z축 회전각): {yaw:10.4f}°")
    print("-" * 60)
    print(f"⚙️ 복조된 4차원 쿼터니언 q [qw, qx, qy, qz]:")
    print(f"   ↳ {q_final}")
    print("=" * 60)

    # 6. 결과 파일 자동 영구 백업 보존
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    backup_path = os.path.join(output_dir, "static_orientation.npz")
    np.savez(backup_path, 
             acc_avg=acc_avg, 
             mag_avg=mag_avg, 
             R_body_to_ned=R_body_to_ned, 
             quaternion=q_final, 
             euler=euler_deg)
             
    print(f"📂 [최종 오리엔테이션 백업 성공] 정적 기하 데이터셋이 보존되었습니다.")
    print(f"   ↳ 경로: {backup_path}")

if __name__ == "__main__":
    main()
