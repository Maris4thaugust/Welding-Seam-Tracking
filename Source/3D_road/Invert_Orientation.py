import open3d as o3d
import numpy as np

# Load the point clouds
small_pcd_o = o3d.io.read_point_cloud("./merged_cloud_v4.ply")

init_st_matrix_rotZ = np.asarray([[-1, 0, 0, 0],
                                  [0, -1, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])    
small_pcd_o.transform(init_st_matrix_rotZ)
o3d.io.write_point_cloud("./merged_cloud_v4.ply", small_pcd_o)