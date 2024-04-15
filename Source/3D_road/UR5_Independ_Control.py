from urx.robot import Robot
import numpy as np
import time 
import os

import UR5_necessaryMethod as ur5m

import open3d as o3d
from scipy.interpolate import interp1d

##################################################################################################

pose_m = []
with open('zList_Points.txt', 'r') as file:
    lines = file.readlines()
for line in lines:
    pose_m.append(line.replace('[', "").replace("]", ""))

lp = []
for i in range(len(pose_m)):
    print(pose_m[i])
    t = np.array([float((pose_m[i].split())[0]), float((pose_m[i].split())[1]), float((pose_m[i].split())[2]), 1.9585, 1.9615, -0.5977])
    lp.append(np.array([float((pose_m[i].split())[0]), float((pose_m[i].split())[1]), float((pose_m[i].split())[2])]))
lp.append(lp[0])
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(lp)

lines = []
for i in range(len(lp)-1):
    line = [i, i+1]
    lines.append(line)

# Tạo thông tin về hướng cho các đường nối
colors = [[1, 0, 0] for _ in range(len(lines))]  # Màu đỏ cho các đường nối
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(lp)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(colors)

# Nội suy đường cong từ vòng kín
num_points_interpolated = 100*23  # Số điểm nội suy
line_points = np.asarray(line_set.points)
t = np.linspace(0, 23, num_points_interpolated)
interp = interp1d(np.arange(line_points.shape[0]), line_points, axis=0, kind='cubic')
interpolated_points = interp(t)

spline = o3d.geometry.LineSet()
spline.points = o3d.utility.Vector3dVector(interpolated_points)

# Tạo các đường nối cho đường cong nội suy
num_lines = num_points_interpolated - 1
lines = [[i, i + 1] for i in range(num_lines)]
lines.append([num_lines, 0])
spline.lines = o3d.utility.Vector2iVector(lines)

##################################################################################################

# Tạo đối tượng Visualizer và thêm đám mây điểm và đường nối vào đó
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
vis.add_geometry(spline)
vis.add_geometry(line_set)
vis.run()
vis.destroy_window()