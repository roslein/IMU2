import struct
import numpy as np

def parse_binary_stl(file_path):
    with open(file_path, 'rb') as f:
        header = f.read(80)
        num_triangles_bytes = f.read(4)
        if len(num_triangles_bytes) < 4:
            return []
        num_triangles = struct.unpack('<I', num_triangles_bytes)[0]
        
        triangles = []
        for _ in range(num_triangles):
            data = f.read(50)
            if len(data) < 50:
                break
            floats = struct.unpack('<12f', data[:48])
            normal = floats[0:3]
            v1 = floats[3:6]
            v2 = floats[6:9]
            v3 = floats[9:12]
            triangles.append({
                'normal': np.array(normal),
                'v1': np.array(v1),
                'v2': np.array(v2),
                'v3': np.array(v3)
            })
        return triangles

triangles = parse_binary_stl('E:/USB Drive/sensor_20_v1.stl')
normals = np.array([t['normal'] for t in triangles])
# norm
norms = np.linalg.norm(normals, axis=1)
valid_idx = norms > 1e-3
normals_unit = normals[valid_idx] / norms[valid_idx][:, np.newaxis]

print("Total valid normals:", len(normals_unit))
print("Min normal values:", np.min(normals_unit, axis=0))
print("Max normal values:", np.max(normals_unit, axis=0))

# 군집화 (점들 사이의 거리가 0.1 이상인 것들만 고유 점으로 인정)
unique_normals = []
for n in normals_unit:
    found = False
    for un in unique_normals:
        if np.dot(un, n) > 0.9: # 25도
            found = True
            break
    if not found:
        unique_normals.append(n)

print(f"\nUnique Normals with 0.9 threshold ({len(unique_normals)}):")
for idx, un in enumerate(unique_normals):
    print(f"#{idx:02d}: [{un[0]:.4f}, {un[1]:.4f}, {un[2]:.4f}]")
