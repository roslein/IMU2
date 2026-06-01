import numpy as np

def get_icosahedron_normals():
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
    # 정규화
    vertices /= np.linalg.norm(vertices[0])
    
    # 2. 정20면체의 한 변의 길이(L) 수학적 탐색
    # 모든 꼭짓점 간의 거리 행렬 계산
    diff = vertices[:, np.newaxis, :] - vertices[np.newaxis, :, :]
    dist_matrix = np.linalg.norm(diff, axis=-1)
    
    # 자기 자신(0.0)을 제외한 가장 가까운 거리가 한 엣지의 길이임
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
                
    faces = np.array(faces) # 정확히 20개의 삼각형 면 검출
    
    # 4. 각 면의 무게중심을 정규화하여 20개 법선 벡터 획득
    normals = []
    for face in faces:
        centroid = np.mean(vertices[face], axis=0)
        norm_vec = centroid / np.linalg.norm(centroid)
        normals.append(norm_vec)
        
    return np.array(normals, dtype=float)

def match_face(g_meas, normals):
    """
    실측 정적 중력 데이터와 20면체 법선 벡터 간 Cosine Similarity 최접점 매칭
    g_meas: raw 가속도 실측 평균 벡터 (3,)
    normals: 20개 면의 정규 법선 벡터 (20, 3)
    반환: 최접점 인덱스, 잔차(Residual, 0에 수렴할수록 완벽)
    """
    g_unit = g_meas / np.linalg.norm(g_meas)
    
    # 물리적 방향성: 지구 중력은 중심축 아래 방향으로 당김 ➔ 법선 벡터의 음의 방향(-n)과 매칭
    dots = normals @ (-g_unit)
    best_idx = np.argmax(dots)
    residual = 1.0 - dots[best_idx]
    
    return best_idx, residual

if __name__ == "__main__":
    # 테스트 구동
    normals = get_icosahedron_normals()
    print(f" 정20면체 법선 벡터 추출 완료! (Shape: {normals.shape})")
    for idx, norm in enumerate(normals):
        print(f"Face #{idx:02d}: [{norm[0]:6.3f}, {norm[1]:6.3f}, {norm[2]:6.3f}]")
    
    # 테스트 매칭
    test_g = np.array([0.1, 0.2, -0.95])
    idx, res = match_face(test_g, normals)
    print(f"\n🔍 테스트 매칭 결과 ➔ Best Face Index: #{idx}, Cos Residual: {res:.6f}")
