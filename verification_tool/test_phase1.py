"""
Real-world IMU Phase 1 Test Script (test_phase1.py)
목표: 아두이노에서 전송하는 39-Byte Binary 패킷을 실시간 수신하여 체크섬을 검증하고,
      각 센서의 계측 데이터가 정상적으로 수신되는지 터미널에 가시성 높게 시각화합니다.
의존성: pip install pyserial
"""

import serial
import serial.tools.list_ports
import struct
import time
import sys

# 39-Byte 패킷 규격 정의
PACKET_SIZE = 39
START_BYTE = 0xAA
END_BYTE = 0x55

def find_arduino_port():
    """연결된 아두이노/ESP32 시리얼 포트 자동 감색"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # 아두이노나 USB Serial 장치 키워드 매칭
        if "usb" in port.description.lower() or "arduino" in port.description.lower() or "ch340" in port.description.lower():
            return port.device
    # 검색 실패 시 첫 포트 리턴
    if ports:
        return ports[0].device
    return None

def main():
    print("=" * 60)
    print(" 🎯 Real-world IMU Phase 1 Binary Stream Tester")
    print("=" * 60)
    
    port = find_arduino_port()
    if not port:
        print("❌ 연결된 시리얼 장치(COM Port)를 찾을 수 없습니다.")
        print("보드 USB 결선 상태 및 드라이버 설치 여부를 확인하십시오.")
        sys.exit(1)
        
    baudrate = 115200
    print(f"🔌 포트 발견: {port} (Baudrate: {baudrate}) 연결 시도 중...")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1.0)
        time.sleep(2) # 아두이노 자동 리셋 대기용 안전 마진
        ser.reset_input_buffer()
        print("✅ 포트 연결 성공. 바이너리 스트리밍 데이터 수신을 시작합니다...\n")
    except Exception as e:
        print(f"❌ 포트 개방 에러: {e}")
        sys.exit(1)
        
    # 패킷 파싱용 임시 바이트 버퍼
    byte_buffer = bytearray()
    
    packet_count = 0
    checksum_errors = 0
    start_time = time.time()
    
    try:
        while True:
            # 1. 시리얼 포트에서 사용 가능한 바이트 전체 읽기
            try:
                in_waiting = ser.in_waiting
            except (serial.SerialException, OSError) as e:
                print(f"\n❌ 시리얼 포트 에러 (장치 연결 유실): {e}")
                break

            if in_waiting > 0:
                try:
                    data = ser.read(in_waiting)
                    byte_buffer.extend(data)
                except (serial.SerialException, OSError) as e:
                    print(f"\n❌ 데이터 읽기 실패 (장치 분리됨): {e}")
                    break
                
            # 2. 버퍼 내에 패킷 규격 이상의 데이터가 쌓여 있을 때 슬라이딩 윈도우 파싱
            while len(byte_buffer) >= PACKET_SIZE:
                # 시작 바이트(0xAA) 탐색
                if byte_buffer[0] != START_BYTE:
                    # 시작 바이트가 아니면 1바이트 버림 (싱크 맞춤)
                    byte_buffer.pop(0)
                    continue
                    
                # 패킷 크기만큼 슬라이싱
                packet = byte_buffer[:PACKET_SIZE]
                
                # 종료 바이트(0x55) 확인
                if packet[-1] != END_BYTE:
                    # 꼬인 패킷이므로 맨 앞 0xAA만 버리고 다음 시작바이트 탐색
                    byte_buffer.pop(0)
                    continue
                    
                # 3. XOR 체크섬 유효성 검증
                xor_sum = START_BYTE
                for b in packet[1:37]: # float 9개 (36바이트) 데이터 영역 XOR
                    xor_sum ^= b
                    
                checksum_in_packet = packet[37]
                
                if xor_sum != checksum_in_packet:
                    checksum_errors += 1
                    # 체크섬 오염 패킷 버림
                    byte_buffer = byte_buffer[PACKET_SIZE:]
                    continue
                    
                # 4. 체크섬 통과 ➔ 데이터 9축 디코딩 (Float 변수 9개 추출)
                data_payload = packet[1:37]
                # '9f'는 9개의 32-bit float를 의미 (little-endian: '<9f')
                floats = struct.unpack('<9f', data_payload)
                
                ax, ay, az = floats[0:3]
                gx, gy, gz = floats[3:6]
                mx, my, mz = floats[6:9]
                
                packet_count += 1
                
                # 5. 콘솔 실시간 모니터링 출력
                elapsed = time.time() - start_time
                fps = packet_count / elapsed if elapsed > 0 else 0
                
                # \r\033[K : 캐리지 리턴 후 현재 행 끝까지 완벽 클리어하여 덮어쓰기 뭉개짐 방지
                sys.stdout.write(
                    f"\r\033[K[FPS: {fps:5.1f}Hz] "
                    f"Acc: {ax:9.4f}, {ay:9.4f}, {az:9.4f} | "
                    f"Gyro(rad): {gx:7.4f}, {gy:7.4f}, {gz:7.4f} | "
                    f"Mag: {mx:9.4f}, {my:9.4f}, {mz:9.4f} | "
                    f"Err: {checksum_errors}"
                )
                sys.stdout.flush()
                
                # 처리한 패킷만큼 버퍼에서 비우기
                byte_buffer = byte_buffer[PACKET_SIZE:]
                
            time.sleep(0.001) # CPU 과부하 방지용 짧은 휴식
            
    except KeyboardInterrupt:
        print("\n\n🛑 사용자에 의해 테스트 스크립트 동작이 강제 종료되었습니다.")
    finally:
        ser.close()
        print("🔌 포트 연결 해제 완료. 안전 종료.")

if __name__ == "__main__":
    main()
