import numpy as np  
import open3d as o3d
import copy
import CustomizeModule as cm
import matplotlib.pyplot as plt

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

PcdTarget = cm.pclread("Assets/CroppedCupSample/cropped_1.ply")
PcdSource = cm.pclread("Assets/CroppedCupSample/cropped_2.ply")

InitialTransformation = np.eye(4)

PcdTarget.voxel_down_sample(0.5)
PcdSource.voxel_down_sample(0.5)

PcdTarget.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

Evaluation = o3d.pipelines.registration.evaluate_registration(PcdSource,PcdTarget,0.02, InitialTransformation)
print(PcdSource)
print(PcdTarget)
print(f"Fitness = {Evaluation.fitness}")
print(f"Rmse = {Evaluation.inlier_rmse}")
print(f"CorrespondenceSet = {Evaluation.correspondence_set}")



def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target],
                                      zoom=0.5,
                                      front=[-0.2458, -0.8088, 0.5342],
                                      lookat=[1.7745, 2.2305, 0.9787],
                                      up=[0.3109, -0.5878, -0.7468])
    
print("1. Load two point clouds and show initial pose")
current_transformation = np.identity(4)
print("2. Point-to-plane ICP registration is applied on original point")
print("   clouds to refine the alignment. Distance threshold 0.02.")
result_icp = o3d.pipelines.registration.registration_icp(
    PcdSource, PcdTarget, 0.02, current_transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
print(result_icp)
draw_registration_result_original_color(PcdSource, PcdTarget,
                                        result_icp.transformation)

# draw initial alignment
current_transformation = np.identity(4)
draw_registration_result_original_color(PcdSource, PcdTarget, current_transformation)
voxel_radius = [0.04, 0.02, 0.01]
max_iter = [50, 30, 14]
current_transformation = np.identity(4)
print("3. Colored point cloud registration")
for scale in range(3):
    iter = max_iter[scale]
    radius = voxel_radius[scale]
    print([iter, radius, scale])

    print("3-1. Downsample with a voxel size %.2f" % radius)
    source_down = PcdSource.voxel_down_sample(radius)
    target_down = PcdTarget.voxel_down_sample(radius)

    print("3-2. Estimate normal.")
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

    print("3-3. Applying colored point cloud registration")
    result_icp = o3d.pipelines.registration.registration_colored_icp(
        source_down, target_down, radius, current_transformation,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                          relative_rmse=1e-6,
                                                          max_iteration=iter))
    current_transformation = result_icp.transformation
    print(result_icp)
draw_registration_result_original_color(PcdSource, PcdTarget,
                                        result_icp.transformation)

# OverlapIndice = [Coord[0] for Coord in Evaluation.correspondence_set ]
# OverlapPcd = PcdSource.select_by_index(OverlapIndice)
# OverlapPcd.paint_uniform_color([0,0,0.5])
# cm.pcdshow([PcdSource.select_by_index(OverlapIndice, invert=True),OverlapPcd,PcdTarget])



