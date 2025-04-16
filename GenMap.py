import numpy as np

def generate_plane_map(x_left, y_top, z_height, width, depth, num_vertices, save_path="plane_map.npy"):
    """
    生成指定参数的平面地图并保存为npy文件。

    参数:
    x_left (float): 左上角X坐标
    y_top (float): 左上角Y坐标
    z_height (float): 平面的Z坐标（高度）
    width (float): X轴方向的宽度（向右延伸）
    depth (float): Y轴方向的深度（向下延伸）
    num_vertices (int): 每个方向的顶点数（网格数+1）
    save_path (str): 保存路径，默认为"plane_map.npy"
    
    返回:
    np.ndarray: 生成的三角形顶点数据，形状为(num_triangles, 3, 3)
    """
    dx = width / (num_vertices - 1)
    dy = depth / (num_vertices - 1)
    
    # 生成顶点坐标
    vertices = []
    for j in range(num_vertices):
        y = y_top - j * dy
        for i in range(num_vertices):
            x = x_left + i * dx
            vertices.append([x, y, z_height])
    vertices = np.array(vertices, dtype=np.float32)
    
    # 生成三角形索引
    triangles = []
    for j in range(num_vertices - 1):
        for i in range(num_vertices - 1):
            A = j * num_vertices + i
            B = j * num_vertices + (i + 1)
            C = (j + 1) * num_vertices + (i + 1)
            D = (j + 1) * num_vertices + i
            # 逆时针顺序确保法线朝上
            triangles.append([C, B, A])
            triangles.append([D, C, A])
    triangles = np.array(triangles, dtype=np.int32)
    
    # 转换为三角形顶点数据
    triangle_vertices = vertices[triangles]
    
    # 保存为npy文件
    np.save(save_path, triangle_vertices)
    return triangle_vertices

if __name__ == '__main__':
    # 示例用法：生成左上角在(x,y,z)的宽为w高为h的地图
    generate_plane_map(0, 0, 0, 1080, 1920, 200, "data/plane_map.npy")
