import open3d as o3d
import numpy as np
import copy
import necessery_method as nm

small_pcd = o3d.io.read_point_cloud("merged_cloud_v5_1.ply")
large_pcd = o3d.io.read_point_cloud("Testing.ply")

small_pcd = nm.rotateZ(small_pcd, -np.pi/2)

small_pcd = copy.deepcopy(small_pcd)
large_pcd = copy.deepcopy(large_pcd)
small_pcd.paint_uniform_color([1, 0.706, 0])
large_pcd.paint_uniform_color([0, 0.651, 0.929])

sm_bbox = small_pcd.get_axis_aligned_bounding_box()
min_bound_sm = sm_bbox.get_min_bound()
la_bbox = large_pcd.get_axis_aligned_bounding_box()
min_bound_la = la_bbox.get_min_bound()
max_bound_la = la_bbox.get_max_bound()
ofs = max_bound_la[2] - min_bound_la[2]

small_pcd = small_pcd.voxel_down_sample(voxel_size=0.1)
large_pcd = large_pcd.voxel_down_sample(voxel_size=0.1)
trans_init = np.asarray([[1, 0, 0,(min_bound_la[0] - min_bound_sm[0])],
                        [0, 1, 0, (min_bound_la[1] - min_bound_sm[1])], 
                        [0, 0, 1, (min_bound_la[2] - min_bound_sm[2])],
                        [0, 0, 0, 1]])

small_pcd.transform(trans_init)
o3d.visualization.draw_geometries([small_pcd, large_pcd])

ptp = o3d.pipelines.registration.TransformationEstimationPointToPoint()
nb_ptp = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000)
threshold = 20
trans_init = np.asarray([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0], 
                         [0, 0, 0, 1]])
reg_p2l = o3d.pipelines.registration.registration_icp(small_pcd, large_pcd, threshold, trans_init, ptp, nb_ptp)

print(reg_p2l)
print("Do khop mau: ", reg_p2l.fitness)
print(reg_p2l.inlier_rmse)
print("Transformation is:\n", reg_p2l.transformation)

small_pcd.transform(reg_p2l.transformation)
merCloud = small_pcd + large_pcd
o3d.io.write_point_cloud("./merged_cloud_v4_1.ply", merCloud)


# Visualize kết quả
sm_bbox = small_pcd.get_axis_aligned_bounding_box()
min_bound_sm = sm_bbox.get_min_bound()
la_bbox = large_pcd.get_axis_aligned_bounding_box()
min_bound_la = la_bbox.get_min_bound()

coord_frame_m = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
coord_frame_m.translate(min_bound_sm)

coord_frame_l = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
coord_frame_l.translate(min_bound_la)

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.get_render_option().background_color = [0.8, 0.5, 0.8] 
vis.add_geometry(small_pcd)
vis.add_geometry(large_pcd)
vis.add_geometry(sm_bbox)
vis.add_geometry(la_bbox)
vis.add_geometry(coord_frame_m)
vis.add_geometry(coord_frame_l)
vis.run()
vis.destroy_window()
