import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import copy

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
print(f'The center of the coordinate 0: {mesh.get_center()}')

mesh_tx = copy.deepcopy(mesh).translate((1.3, 0, 0))
print(f'The center of the coordinate 0: {mesh_tx.get_center()}')


o3d.visualization.draw_geometries([mesh,mesh_tx])

