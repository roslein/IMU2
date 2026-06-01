"""
Real-world IMU Phase 2 Gyroscope Bias Calibration (gyro_bias_calibration.py)
목적: 20면체 정적 안착 획득 데이터(collected_data.npz)에서 3축 자이로 데이터를 로드하여,
      20개 공간 포지션의 기하학적 대칭성을 활용한 Global Average 정밀 바이어스(오프셋)를 도출하고
      펌웨어 기동 시 정적 무회전 static initialization 영점 상수를 안전하게 제공합니다.
"""

import numpy as np
import os
import sys

# Windows CP949 콘솔 이모지 인코딩 충돌 방지 강제 설정
sys.stdout.reconfigure(encoding='utf-8')

def main():
    print("=" * 60)
    print(" 🎯 Phase 2 Gyroscope Global Average Bias Solver")
    print("=" * 60)
    
    # 스크립트 실행 디렉토리 기준 절대 경로 확보
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(script_dir, "output", "collected_data.npz")
    
    # 데이터 로드
    try:
        data = np.load(data_path)
        if "gyro" not in data:
            print("⚠️ [데이터 불일치] collected_data.npz 내에 자이로 데이터(gyro)가 누락되어 있습니다.")
            print("   👉 data_collection.py 최신 버전을 실행하여 가속도/자력/자이로가 병합 수집된")
            print("      새로운 20개 포지션 데이터셋을 완성한 뒤 본 툴을 구동해 주십시오.")
            return
            
        gyro_raw = data["gyro"]
        print(f"📂 실측 자이로 데이터 세트 로드 완료 (Shape: {gyro_raw.shape})")
    except Exception as e:
        print(f"❌ 데이터 로드 에러: {e}")
        print("data_collection.py를 가동하여 output/collected_data.npz 데이터를 먼저 확보하십시오.")
        return
        
    # 20개 포지션의 3축 자이로 원시 평균값을 대수적으로 다시 전수 총평균(Global Average)
    # 기하학적 대칭성에 의해 가압 요동 노이즈가 완벽하게 소쇄 연산됩니다.
    b_gyro = np.mean(gyro_raw, axis=0)
    
    print("\n⚡ Global Average 자이로스코프 Bias 오프셋 계산 완료...")
    print("=" * 60)
    print(" 🎉 자이로스코프 정적 Bias (오프셋) 보정 결과 보고")
    print("=" * 60)
    print(f"⚙️ 도출된 Gyro Bias b_gyro_x: {b_gyro[0]:12.8f} dps")
    print(f"⚙️ 도출된 Gyro Bias b_gyro_y: {b_gyro[1]:12.8f} dps")
    print(f"⚙️ 도출된 Gyro Bias b_gyro_z: {b_gyro[2]:12.8f} dps")
    print("-" * 60)
    print("💡 [펌웨어 이식 가이드] 아래의 초기화 상수를 펌웨어 HAL 초기 구동단(static initialization)")
    print("   또는 calib_params.h 헤더 파일 내부 자이로 영점 오프셋 상수로 복사해 넣으십시오.")
    print(f"   const float GYRO_BIAS[3] = {{ {b_gyro[0]:.8f}f, {b_gyro[1]:.8f}f, {b_gyro[2]:.8f}f }};")
    print("=" * 60)
    
    # 파라미터 백업 파일 저장
    param_path = os.path.join(script_dir, "output", "gyro_params.npz")
    np.savez(param_path, b_gyro=b_gyro)
    print(f"📂 자이로 바이어스 파라미터 백업 성공: {param_path}")

if __name__ == "__main__":
    main()
