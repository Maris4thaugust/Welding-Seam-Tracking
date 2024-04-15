import open3d as o3d
import numpy as np
import copy
import necessery_method as nm
import time 

vis = o3d.visualization.Visualizer()
vis.create_window()

for i in range(1, 11):
    # i = 50
    # if i == 1:
    #     large_pcd_o = o3d.io.read_point_cloud(f"./data_Shoe_v4/{i-1}.ply")
    #     small_pcd_o = o3d.io.read_point_cloud(f"./data_Shoe_v4/{i}.ply")
    # else:
    large_pcd_o = o3d.io.read_point_cloud(f"merged_cloud_v4.ply")
    small_pcd_o = o3d.io.read_point_cloud(f"./data_Shoe_v4_1/{i}.ply")

    small_pcd = copy.deepcopy(small_pcd_o)
    large_pcd = copy.deepcopy(large_pcd_o)
    small_pcd.paint_uniform_color([1, 0.706, 0])
    large_pcd.paint_uniform_color([0, 0.651, 0.929])

    small_pcd = small_pcd.voxel_down_sample(voxel_size=1)
    large_pcd = large_pcd.voxel_down_sample(voxel_size=1)

    small_pcd = nm.move2Ori(small_pcd, -i)
    large_pcd = nm.move2Ori(large_pcd, 0)
    small_pcd_o = nm.move2Ori(small_pcd_o, -i)
    large_pcd_o = nm.move2Ori(large_pcd_o, 0)

    sm_bbox = small_pcd.get_axis_aligned_bounding_box()
    min_bound_sm = sm_bbox.get_min_bound()
    max_bound_sm = sm_bbox.get_max_bound()
    la_bbox = large_pcd.get_axis_aligned_bounding_box()
    min_bound_la = la_bbox.get_min_bound()
    max_bound_la = la_bbox.get_max_bound()
   
    ptp = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    nb_ptp = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6,max_iteration=1000)
    threshold = 10

    if i == 1:
        trans_init = np.asarray([[1, 0, 0, 0],
                                [0, 1, 0, 0], 
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]]) 
    elif 1 < i:
        trans_init = np.asarray([[1, 0, 0, 0],
                                [0, 1, 0, (min_bound_la[1] - min_bound_sm[1])], 
                                [0, 0, 1, (min_bound_la[2] - min_bound_sm[2])/1.5],
                                [0, 0, 0, 1]]) 


    reg_p2l = o3d.pipelines.registration.registration_icp(small_pcd, large_pcd, threshold, trans_init, ptp, nb_ptp)

    print(reg_p2l)
    print("Do khop mau: ", reg_p2l.fitness)
    print(reg_p2l.inlier_rmse)
    print("Transformation is:\n", reg_p2l.transformation)

    if reg_p2l.fitness > 0.75:
        print("Number: ", i)
        small_pcd_o.transform(reg_p2l.transformation)
        small_pcd.transform(reg_p2l.transformation)

    # Save the merged point cloud in a PLY file 
        merged_cloud = small_pcd_o + large_pcd_o
        merged_cloud = merged_cloud.voxel_down_sample(voxel_size=2)
        o3d.io.write_point_cloud('merged_cloud_v4.ply', merged_cloud)
        time.sleep(0.1)
        vis.add_geometry(merged_cloud)
        vis.update_geometry(merged_cloud)
        vis.poll_events()
        vis.update_renderer()

    elif reg_p2l.fitness < 0.65:
        print("Number: ", i)
        continue

vis.destroy_window()
o3d.visualization.draw_geometries([merged_cloud])


