if __name__ == '__main__':
    # 读取OBJ文件
    triangles_path = "/root/workspace/xjy/ObjToNpy/data/plane_map.npy"  # 确保这个路径与read_mesh中的save_path一致
    _ = read_mesh(args.mesh_path, triangles_path)
    try:
        # 直接从npy文件读取三角形网格数据
        mesh = np.load(triangles_path)
        print("Mesh shape: ", mesh.shape)
        print("First 3x3 matrix (first triangle):\n", mesh[0])
        print("Last 3x3 matrix (last triangle):\n", mesh[-1])
    except FileNotFoundError:
        # 如果npy文件不存在，则从OBJ文件读取并保存
        mesh = read_mesh(args.mesh_path, triangles_path)
        print("No load from npy file successfully")

