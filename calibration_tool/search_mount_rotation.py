import numpy as np
import icosahedron
import sys

sys.stdout.reconfigure(encoding='utf-8')

# 1. 15개 실측 데이터 정의
raw_data = [
    [593.2, -545.9, -571.0],
    [18.6, -336.9, -928.7],
    [-5.0, 382.5, -923.9],
    [-583.4, 582.4, -563.3],
    [-373.3, 942.8, 12.4],
    [-578.0, 575.2, 593.1],
    [6.4, 369.5, 950.0],
    [605.0, -551.0, 579.5],
    [390.7, -910.9, 1.6],
    [-546.5, -588.2, 582.4],
    [561.7, 623.3, -563.6],
    [935.1, 53.2, -344.7],
    [-541.0, -590.6, -567.3],
    [-918.0, -18.5, 361.6],
    [562.4, 610.8, 597.0]
]

# 단위 벡터화 및 중력 방향 변환 (-g)
g_meas = []
for acc in raw_data:
    v = np.array(acc)
    g_meas.append(- (v / np.linalg.norm(v)))
g_meas = np.array(g_meas)

# 2. 정20면체 법선 로드
normals = icosahedron.get_icosahedron_normals()

# 3. 3D 회전 격자 탐색 (Euler angles Z-Y-X)
from scipy.spatial.transform import Rotation as R_scipy

best_R = None
best_score = 0.0
best_euler = None

print("🔄 전역 회전 탐색 시작 (마운팅 회전 행렬 R_mount 추정)...")

# Y축 기준 29.05도 근방 + 90도 단위 회전 탐색
# 또는 전체 3D 공간 격자 검색 (10도 간격)
for yaw in np.arange(-180, 180, 10):
    for pitch in np.arange(-180, 180, 10):
        for roll in np.arange(-180, 180, 10):
            r = R_scipy.from_euler('zyx', [yaw, pitch, roll], degrees=True)
            R = r.as_matrix()
            
            # 회전된 법선 벡터
            rotated_normals = (R.T @ normals.T).T
            
            # 매칭 인덱스 및 고유성 확인
            matched_indices = []
            similarities = []
            for g in g_meas:
                dots = rotated_normals @ g
                best_idx = np.argmax(dots)
                matched_indices.append(best_idx)
                similarities.append(dots[best_idx])
                
            # 평가 기준: 1) 매칭된 면의 개수가 최대한 다양해야 함 (중복 최소화), 2) 평균 유사도가 높아야 함
            unique_count = len(set(matched_indices))
            avg_similarity = np.mean(similarities)
            
            # 가중 점수 (중복 없는 고유 면 개수가 가장 중요!)
            score = unique_count * 100.0 + avg_similarity
            
            if score > best_score:
                best_score = score
                best_R = R
                best_euler = [yaw, pitch, roll]
                best_matches = matched_indices
                best_sims = similarities

print(f"\n🎉 [탐색 완료] 최적 마운팅 회전 찾음!")
print(f"오일러각 (Z-Y-X 순, deg): Z={best_euler[0]:.1f}°, Y={best_euler[1]:.1f}°, X={best_euler[2]:.1f}°")
print(f"고유 매칭 면 개수: {len(set(best_matches))} / 15")
print(f"평균 유사도: {np.mean(best_sims)*100.0:.2f}%")

print("\n📊 15개 실측 포지션별 1:1 매칭 맵핑 결과:")
for idx, g in enumerate(g_meas):
    print(f"실측 #{idx+1:02d} ➔ 매칭 정20면체 면 #{best_matches[idx]:02d} (유사도: {best_sims[idx]*100.0:.2f}%)")
