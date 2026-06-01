import numpy as np
import icosahedron

normals = icosahedron.get_icosahedron_normals()
plate_normal = np.array([0.8742, -0.0003, -0.4855])

print("Dot products of plate normal with icosahedron normals:")
for idx, n in enumerate(normals):
    dot = np.dot(plate_normal, n)
    print(f"Face #{idx:02d}: dot = {dot:.4f}, angle = {np.arccos(np.clip(dot, -1, 1))*180/np.pi:.1f}°")
