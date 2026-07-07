import numpy as np

def get_icosahedron_normals() -> np.ndarray:
    """
    황금비(phi) 공식을 기하학적으로 활용하여, 12개 꼭짓점의 거리 대조를 통해
    20개 면의 정밀 법선 단위 벡터(20x3)를 실시간 자동 탐색 및 생성합니다.
    """
    phi = (1.0 + np.sqrt(5.0)) / 2.0  # 황금비
    
    # 1. 12개 꼭짓점 좌표 생성 및 정규화
    vertices = []
    # [0, +/-1, +/-phi]
    for y in [-1.0, 1.0]:
        for z in [-phi, phi]:
            vertices.append([0.0, y, z])
    # [+/-1, +/-phi, 0]
    for x in [-1.0, 1.0]:
        for y in [-phi, phi]:
            vertices.append([x, y, 0.0])
    # [+/-phi, 0, +/-1]
    for x in [-phi, phi]:
        for z in [-1.0, 1.0]:
            vertices.append([x, 0.0, z])
            
    vertices = np.array(vertices, dtype=float)
    vertices /= np.linalg.norm(vertices[0])
    
    # 2. 정20면체의 한 변의 길이(L) 수학적 탐색
    diff = vertices[:, np.newaxis, :] - vertices[np.newaxis, :, :]
    dist_matrix = np.linalg.norm(diff, axis=-1)
    
    L = np.min(dist_matrix[dist_matrix > 1e-5])
    
    # 3. 세 변의 길이가 모두 L인 3개 꼭짓점 조합(삼각형 면) 완전 탐색
    faces = []
    n_vertices = 12
    for i in range(n_vertices):
        for j in range(i + 1, n_vertices):
            if abs(dist_matrix[i, j] - L) > 1e-4:
                continue
            for k in range(j + 1, n_vertices):
                if abs(dist_matrix[i, k] - L) > 1e-4 or abs(dist_matrix[j, k] - L) > 1e-4:
                    continue
                faces.append([i, j, k])
                
    faces = np.array(faces)
    
    # 4. 무게중심 정규화 법선 획득
    normals = []
    for face in faces:
        centroid = np.mean(vertices[face], axis=0)
        norm_vec = centroid / np.linalg.norm(centroid)
        normals.append(norm_vec)
        
    return np.array(normals, dtype=float)

def get_jig_to_sensor_rotation() -> np.ndarray:
    """
    STL 모델 기하 분석 및 실측 SVD 정합을 통해 도출된
    정20면체 지그 좌표계에서 센서 좌표계로의 최적 마운팅 회전 행렬(R_mount.T) 반환
    """
    R_mount = np.array([
        [-0.81882323,  0.28767469, -0.49676129],
        [-0.47402805, -0.82692424,  0.30247928],
        [-0.32376832,  0.48315585,  0.81347065]
    ])
    return R_mount.T

def get_rotated_normals() -> np.ndarray:
    """지그의 ideal 법선 벡터(20x3)를 R_mount.T로 센서 좌표계 기준 회전 변환하여 반환"""
    normals = get_icosahedron_normals()
    R_jig_to_sensor = get_jig_to_sensor_rotation()
    return (R_jig_to_sensor @ normals.T).T

def match_face(acc_meas: np.ndarray, normals: np.ndarray) -> tuple:
    """실측 중력 가속도 데이터와 20면체 법선 벡터 간 Cosine 유사도 최접점 매칭"""
    norm_val = np.linalg.norm(acc_meas)
    a_unit = acc_meas / (norm_val if norm_val > 0 else 1.0)
    
    # 수직항력 가속도(Upward) 지향 매칭 (ad-hoc 음수 부호 소거)
    dots = normals @ a_unit
    best_idx = int(np.argmax(dots))
    residual = float(1.0 - dots[best_idx])
    return best_idx, residual
