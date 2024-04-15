import numpy as np  
import open3d as o3d
import CustomizeModule as cm

pcd = cm.pclread("coc_4.ply")
o3d.visualization.draw_geometries_with_editing([pcd])
