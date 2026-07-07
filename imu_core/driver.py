import os
import sys
import time
import struct
import numpy as np
import serial
import serial.tools.list_ports
from imu_core.interface import SensorDriver
import imu_core.constants as const

class FileIMUDriver(SensorDriver):
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.data = None

    def connect(self) -> bool:
        if not os.path.exists(self.file_path):
            print(f"❌ 가상 데이터 파일이 유실되었습니다: {self.file_path}")
            return False
        try:
            self.data = np.load(self.file_path)
            # 필수 성분 유효성 검사
            if "acc" not in self.data or "mag" not in self.data:
                print("❌ npz 파일 내에 필수 성분(acc, mag)이 존재하지 않습니다.")
                return False
            return True
        except Exception as e:
            print(f"❌ 파일 데이터 로드 실패: {e}")
            return False

    def fetch_raw_data(self) -> dict:
        if self.data is None:
            raise RuntimeError("드라이버가 연결되지 않았습니다. 먼저 connect()를 호출하십시오.")
        result = {
            "acc": self.data["acc"],
            "mag": self.data["mag"]
        }
        if "gyro" in self.data:
            result["gyro"] = self.data["gyro"]
        return result

    def disconnect(self) -> None:
        self.data = None


class SerialIMUDriver(SensorDriver):
    def __init__(self, port: str = None, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.byte_buffer = bytearray()

    def _find_arduino_port(self) -> str:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            desc = port.description.lower()
            if "usb" in desc or "arduino" in desc or "ch340" in desc:
                return port.device
        if ports:
            return ports[0].device
        return None

    def connect(self) -> bool:
        if self.port is None:
            self.port = self._find_arduino_port()
        if self.port is None:
            print("❌ 사용 가능한 시리얼 포트를 감지하지 못했습니다.")
            return False

        try:
            print(f"🔌 포트 발견: {self.port} (Baudrate: {self.baudrate}) 연결 시도 중...")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            time.sleep(1.0) # 아두이노 부팅 대기
            
            # 아두이노 하드웨어 에러 감지 (I2C 선 단선 등)
            if self.ser.in_waiting > 0:
                startup_logs = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                if "error:" in startup_logs.lower() or "실패" in startup_logs:
                    print(f"\n❌ [아두이노 하드웨어 에러 감지]: {startup_logs.strip()}")
                    self.ser.close()
                    sys.exit(1)
            
            print("✅ 포트 연결 성공.")
            return True
        except Exception as e:
            print(f"❌ 시리얼 연결 에러: {e}")
            return False

    def _parse_packet(self, packet: bytearray) -> tuple:
        if len(packet) != const.PACKET_SIZE:
            return None
        # XOR 체크섬 검사
        xor_sum = const.START_BYTE
        for b in packet[1:37]:
            xor_sum ^= b
        if xor_sum != packet[37]:
            return None

        floats = struct.unpack('<9f', packet[1:37])
        acc = np.array(floats[0:3])
        gyro = np.array(floats[3:6])
        mag = np.array(floats[6:9])
        return acc, gyro, mag

    def fetch_raw_data(self) -> dict:
        """단일 실시간 패킷 읽기"""
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("드라이버가 연결되지 않았습니다.")
        
        start_time = time.time()
        while time.time() - start_time < 2.0:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.byte_buffer.extend(data)
                
                while len(self.byte_buffer) >= const.PACKET_SIZE:
                    if self.byte_buffer[0] != const.START_BYTE:
                        self.byte_buffer.pop(0)
                        continue
                    
                    packet = self.byte_buffer[:const.PACKET_SIZE]
                    if packet[-1] != const.END_BYTE:
                        self.byte_buffer.pop(0)
                        continue
                    
                    parsed = self._parse_packet(packet)
                    self.byte_buffer = self.byte_buffer[const.PACKET_SIZE:]
                    if parsed is not None:
                        acc, gyro, mag = parsed
                        return {"acc": acc, "gyro": gyro, "mag": mag}
            time.sleep(0.001)
        raise TimeoutError("시리얼 스트리밍 패킷 수신 타임아웃")

    def collect_samples(self, num_samples: int = 100) -> dict:
        """
        정해진 수의 샘플을 획득하여 딕셔너리로 반환 (정량 수집용)
        - Homogeneous shape 에러 방지를 위해 정확히 [:num_samples] 크기로 슬라이싱 보장
        """
        acc_list, gyro_list, mag_list = [], [], []
        self.ser.reset_input_buffer()
        self.byte_buffer.clear()
        
        while len(acc_list) < num_samples:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.byte_buffer.extend(data)
                
                while len(self.byte_buffer) >= const.PACKET_SIZE:
                    if self.byte_buffer[0] != const.START_BYTE:
                        self.byte_buffer.pop(0)
                        continue
                    
                    packet = self.byte_buffer[:const.PACKET_SIZE]
                    if packet[-1] != const.END_BYTE:
                        self.byte_buffer.pop(0)
                        continue
                    
                    parsed = self._parse_packet(packet)
                    self.byte_buffer = self.byte_buffer[const.PACKET_SIZE:]
                    if parsed is not None:
                        acc, gyro, mag = parsed
                        acc_list.append(acc)
                        gyro_list.append(gyro)
                        mag_list.append(mag)
            time.sleep(0.001)
            
        return {
            "acc": np.array(acc_list[:num_samples]),
            "gyro": np.array(gyro_list[:num_samples]),
            "mag": np.array(mag_list[:num_samples])
        }

    def collect_rolling_average(self, duration_sec: float = 1.5) -> tuple:
        """지정된 시간 동안 롤링 평균하여 반환"""
        self.ser.reset_input_buffer()
        self.byte_buffer.clear()
        collected_acc = []
        collected_gyro = []
        collected_mag = []
        
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.byte_buffer.extend(data)
                
                while len(self.byte_buffer) >= const.PACKET_SIZE:
                    if self.byte_buffer[0] != const.START_BYTE:
                        self.byte_buffer.pop(0)
                        continue
                    
                    packet = self.byte_buffer[:const.PACKET_SIZE]
                    if packet[-1] != const.END_BYTE:
                        self.byte_buffer.pop(0)
                        continue
                    
                    parsed = self._parse_packet(packet)
                    self.byte_buffer = self.byte_buffer[const.PACKET_SIZE:]
                    if parsed is not None:
                        acc, gyro, mag = parsed
                        collected_acc.append(acc)
                        collected_gyro.append(gyro)
                        collected_mag.append(mag)
            time.sleep(0.002)
            
        if len(collected_acc) == 0:
            return None, None, None
            
        return (
            np.mean(collected_acc, axis=0),
            np.mean(collected_gyro, axis=0),
            np.mean(collected_mag, axis=0)
        )

    def disconnect(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.byte_buffer.clear()
