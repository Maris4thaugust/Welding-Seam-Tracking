import open3d as o3d
import numpy as np

# Tạo đám mây điểm
points = np.random.rand(100, 3)  # Tạo 100 điểm ngẫu nhiên
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Tạo các đường nối có hướng giữa các điểm
lines = []
for i in range(len(points)-1):
    line = [i, i+1]
    lines.append(line)

# Tạo thông tin về hướng cho các đường nối
colors = [[1, 0, 0] for _ in range(len(lines))]  # Màu đỏ cho các đường nối
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(colors)

# Tạo đối tượng Visualizer và thêm đám mây điểm và đường nối vào đó
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
vis.add_geometry(line_set)

vis.run()
vis.destroy_window()