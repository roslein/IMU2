import numpy as np
import icosahedron
import sys

sys.stdout.reconfigure(encoding='utf-8')

# 1. 15개 실측 데이터와 정확히 매칭된 Face ID 정의
raw_data = [
    {"registered": 6, "acc": [593.2, -545.9, -571.0]},
    {"registered": 5, "acc": [18.6, -336.9, -928.7]},
    {"registered": 14, "acc": [-5.0, 382.5, -923.9]},
    {"registered": 17, "acc": [-583.4, 582.4, -563.3]},
    {"registered": 11, "acc": [-373.3, 942.8, 12.4]},
    {"registered": 0, "acc": [-578.0, 575.2, 593.1]},
    {"registered": 1, "acc": [6.4, 369.5, 950.0]},
    {"registered": 18, "acc": [605.0, -551.0, 579.5]},
    {"registered": 9, "acc": [390.7, -910.9, 1.6]},
    {"registered": 0.0, "acc": [-546.5, -588.2, 582.4]}, # 0.0? Wait! In the SVD printout:
    # 실측 #10 ➔ 매칭 정20면체 면 #02 (유사도: 99.97%)
    # Let's write them down exactly from SVD results:
]

# Let's map them exactly:
mapping = [
    (0, 6),   # 실측 #01 -> Face #06
    (1, 5),   # 실측 #02 -> Face #05
    (2, 14),  # 실측 #03 -> Face #14
    (3, 17),  # 실측 #04 -> Face #17
    (4, 11),  # 실측 #05 -> Face #11
    (5, 0),   # 실측 #06 -> Face #00
    (6, 1),   # 실측 #07 -> Face #01
    (7, 18),  # 실측 #08 -> Face #18
    (8, 9),   # 실측 #09 -> Face #09
    (9, 2),   # 실측 #10 -> Face #02
    (10, 13), # 실측 #11 -> Face #13
    (11, 15), # 실측 #12 -> Face #15
    (12, 8),  # 실측 #13 -> Face #08
    (13, 3),  # 실측 #14 -> Face #03
    (14, 12)  # 실측 #15 -> Face #12
]

acc_values = [
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

normals = icosahedron.get_icosahedron_normals()

A_pts = []
B_pts = []

for acc_idx, face_idx in mapping:
    ideal_n = normals[face_idx]
    acc = np.array(acc_values[acc_idx])
    g_meas_unit = - (acc / np.linalg.norm(acc))
    
    A_pts.append(ideal_n)
    B_pts.append(g_meas_unit)

A_pts = np.array(A_pts)
B_pts = np.array(B_pts)

# Kabsch SVD
H = B_pts.T @ A_pts
U, S, Vt = np.linalg.svd(H)
R_mount = Vt.T @ U.T

if np.linalg.det(R_mount) < 0:
    Vt[2, :] *= -1
    R_mount = Vt.T @ U.T

print("⚡ [정밀 SVD 결과] 최적 마운팅 회전 행렬 R_mount:")
print(np.array2string(R_mount, separator=', '))
print(f"Determinant: {np.linalg.det(R_mount):.6f}")

from scipy.spatial.transform import Rotation as R_scipy
r = R_scipy.from_matrix(R_mount)
euler = r.as_euler('zyx', degrees=True)
print(f"오일러각 (Z-Y-X 순, deg): Z={euler[0]:.2f}°, Y={euler[1]:.2f}°, X={euler[2]:.2f}°")

# 유사도 재검증
rotated_normals = (R_mount.T @ normals.T).T
total_sim = 0.0
for acc_idx, face_idx in mapping:
    acc = np.array(acc_values[acc_idx])
    g_meas_unit = - (acc / np.linalg.norm(acc))
    sim = np.dot(rotated_normals[face_idx], g_meas_unit)
    print(f"실측 #{acc_idx+1:02d} ➔ 진짜 면 #{face_idx:02d} | 일치율: {sim*100.0:.2f}%")
    total_sim += sim
print(f"➔ 정밀 평균 일치율: {total_sim/len(mapping)*100.0:.2f}%")
