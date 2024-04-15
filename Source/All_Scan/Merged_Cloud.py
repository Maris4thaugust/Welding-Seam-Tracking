import open3d as o3d
import numpy as np
import necessery_method as nm
import time 
import copy

vis = o3d.visualization.Visualizer()
vis.create_window()

for i in range(1, 36):
    if i == 1:
        large_pcd_o = o3d.io.read_point_cloud(f"data_Glove_v1/{i-1}.ply")
        small_pcd_o = o3d.io.read_point_cloud(f"data_Glove_v1/{i}.ply")
    else:
        large_pcd_o = o3d.io.read_point_cloud(f"merged_cloud_v1.ply")
        small_pcd_o = o3d.io.read_point_cloud(f"data_Glove_v1/{i}.ply")
    small_pcd = copy.deepcopy(small_pcd_o)
    large_pcd = copy.deepcopy(large_pcd_o)
    small_pcd.paint_uniform_color([1, 0.706, 0])
    large_pcd.paint_uniform_color([0, 0.651, 0.929])

    if i>34:
        small_pcd = nm.rotateX(small_pcd, np.pi/2)
        small_pcd_o = nm.rotateX(small_pcd_o, np.pi/2)
        small_pcd = nm.move2Ori(small_pcd, 0)
        large_pcd = nm.move2Ori(large_pcd, 0)
        small_pcd_o = nm.move2Ori(small_pcd_o, 0)
        large_pcd_o = nm.move2Ori(large_pcd_o, 0)
    else:   
        small_pcd = nm.move2Ori(small_pcd, i)
        large_pcd = nm.move2Ori(large_pcd, 0)
        small_pcd_o = nm.move2Ori(small_pcd_o, i)
        large_pcd_o = nm.move2Ori(large_pcd_o, 0)

    ptp = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    nb_ptp = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6,max_iteration=1000)
    threshold = 3
    
    if i <= 13:
        trans_init = np.asarray([[1, 0, 0, 0],
                                [0, 1, 0, 0], 
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
    elif 13 < i <= 24:
        trans_init = np.asarray([[1, 0, 0, 10],
                                [0, 1, 0, 0], 
                                [0, 0, 1, 25],
                                [0, 0, 0, 1]])
    elif 25 < i <= 27:
        trans_init = np.asarray([[1, 0, 0, 30],
                                [0, 1, 0, 0], 
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
    elif 28 < i <= 32:
        trans_init = np.asarray([[1, 0, 0, 40],
                                [0, 1, 0, 0], 
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
    elif 32 < i <= 34:
        trans_init = np.asarray([[1, 0, 0, 20],
                                [0, 1, 0, 0], 
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
    else:
        trans_init = np.asarray([[1, 0, 0, 0],
                                [0, 1, 0, 100], 
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

    reg_p2l = o3d.pipelines.registration.registration_icp(small_pcd, large_pcd, threshold, trans_init, ptp, nb_ptp)

    print(reg_p2l)
    print("Do khop mau: ", reg_p2l.fitness)
    # print(reg_p2l.inlier_rmse)
    # print("Transformation is:\n", reg_p2l.transformation)
    if reg_p2l.fitness > 0.7:
        small_pcd_o.transform(reg_p2l.transformation)
        small_pcd.transform(reg_p2l.transformation)

    # Save the merged point cloud in a PLY file 
        merged_cloud = small_pcd_o + large_pcd_o
        merged_cloud = merged_cloud.voxel_down_sample(voxel_size=1)
        o3d.io.write_point_cloud('merged_cloud_v3.ply', merged_cloud)
    print(f"===> Complete {round(i*100/36)}%")
       
    vis.add_geometry(merged_cloud)
    vis.update_geometry(merged_cloud)
    vis.poll_events()
    vis.update_renderer()

# Visualize the aligned template point cloud
# small_pcd.transform(trans_init)
vis.destroy_window()
o3d.visualization.draw_geometries([merged_cloud])

