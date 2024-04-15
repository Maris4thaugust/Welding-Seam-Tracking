import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import CustomizeModule as cm
import copy
import os 

def DeleteFile(Path):
    try:
        os.remove(Path)
        print(f"Remove {Path} sucessfully.")
        return True
    except OSError as e:
        print(f"Error: {e.strerror}")
        return False
def DrawCurve():
    # Create some sample points to define the curve
    num_points = 100
    t = np.linspace(0, 2*np.pi, num_points)
    x = np.sin(t)
    y = np.cos(t)
    z = np.zeros_like(t)
    points = np.column_stack((x, y, z))

    # Create a LineSet geometry object
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)

    # Define the lines by specifying the start and end point indices
    lines = []
    for i in range(num_points - 1):
        lines.append([i, i+1])
    line_set.lines = o3d.utility.Vector2iVector(lines)

    # Create visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Add the LineSet to the visualization
    vis.add_geometry(line_set)

    # Set up camera and rendering options
    vis.get_render_option().load_from_json("render_options.json")

    # Run visualization
    vis.run()
    vis.destroy_window()
def checkPolygon():
    # Coordinates of the polygon
    polygon_coords = [
        [2.6509309513852526, 1.6834473132326844],
        [2.5786428246917148, 1.6892074266735244],
        [2.4625790337552154, 1.6665777078297999],
        [2.2228544982251655, 1.6168160446813649],
        [2.166993206001413, 1.6115495157201662],
        [2.1167895865303286, 1.6257706054969348],
        [2.0634657721747383, 1.623021658624539],
        [2.0568612343437236, 1.5853892911207643],
        [2.1605399001237027, 0.96228993255083017],
        [2.1956669387205228, 0.95572746049785073],
        [2.2191318790575583, 0.88734449982108754],
        [2.2484881847925919, 0.87042807267013633],
        [2.6891234157295827, 0.94140677988967603],
        [2.7328692490470647, 0.98775740674840251],
        [2.7129337547575547, 1.0398850034649203],
        [2.7592174072415405, 1.0692940558509485],
        [2.7689216419453428, 1.0953914441371593],
        [2.6851455625455669, 1.6307334122162018],
        [2.6714776099981239, 1.675524657088997],
        [2.6579576128816544, 1.6819127849749496]
    ]

    # Extract X and Z coordinates
    x_coords = [coord[0] for coord in polygon_coords]
    z_coords = [coord[1] for coord in polygon_coords]

    # Plot the polygon
    plt.figure(figsize=(8, 6))
    plt.plot(x_coords, z_coords, marker='o', linestyle='-')
    plt.xlabel('X-coordinate')
    plt.ylabel('Z-coordinate')
    plt.title('2D Polygon Plot')
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
def draw_registration_result(source, target, transformation):
    source.transform(transformation)
    o3d.visualization.draw_geometries([source, target])
def PreprocessPointCloud(pcd, voxel_size):
    RadiusNormal = voxel_size * 2
    RadiusFeature = voxel_size * 5
    PcdDown = pcd.voxel_down_sample(voxel_size)
    PcdDown.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=RadiusNormal, max_nn=30))
    KdPoints = o3d.geometry.KDTreeSearchParamHybrid(radius=RadiusFeature, max_nn=100)
    PcdFpfh = o3d.pipelines.registration.compute_fpfh_feature( PcdDown,KdPoints)
    return PcdDown, PcdFpfh

delta_x = 12.38117981
delta_y = -2.67799377
delta_z = -21.09512329

TranslationMatrix = np.array([
    [1, 0, 0, delta_x],
    [0, 1, 0, delta_y],
    [0, 0, 1, delta_z],
    [0, 0, 0, 1]
])

# Load point cloud
pcd = cm.pclread("Assets/CroppedCupSample/cropped_1.ply")
pcd_2 = cm.pclread("Assets/CroppedCupSample/cropped_2.ply")
print(pcd)
print(pcd_2)
PcdPoints = np.asarray(pcd.points)
PcdPoints2 = np.asarray(pcd.points)

#Preprocess point cloud 
pcd_copy = copy.deepcopy(pcd)
pcd_2_copy = copy.deepcopy(pcd)
pcdDown, PcdFpfh = PreprocessPointCloud(pcd,1)
pcdDown2, PcdFpfh2 = PreprocessPointCloud(pcd_2,1)
print(len(PcdFpfh.data[0]))

# transformation point cloud
pcd_2.translate((delta_x,delta_y,delta_z))

# Get min bound
MinBound = pcd.get_min_bound()
# print(f"The min bound coordinate of cropped_1: {MinBound}")
MinBound2 = pcd_2.get_min_bound()
# print(f"The min bound coordinate of cropped_2: {MinBound2}")

# Down sampling
DownPcd1 = pcd.voxel_down_sample(1)
DownPcd2 = pcd_2.voxel_down_sample(1)


# visualize specific point cloud
CenterPointCoordinate = DownPcd1.get_center()
SpecPoints = o3d.geometry.PointCloud()
# Add the point's coordinates
SpecPoints.points = o3d.utility.Vector3dVector([CenterPointCoordinate,MinBound,MinBound2])  
# print(SpecPoints)
SpecPoints.paint_uniform_color([1,1,0])

aabb = pcd.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)
aabb2 = pcd_2.get_axis_aligned_bounding_box()
aabb2.color = (0, 1, 0)

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=6, origin=MinBound)
mesh_frame_2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=6, origin=MinBound2)

# Examine point feature




# InitialTransformation = np.eye(4)
# print("Apply point-to-point ICP")
# reg_p2p = o3d.pipelines.registration.registration_icp(
#     pcd_2, pcd, 0.1, InitialTransformation,
#     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
# print(f"Fitness = {reg_p2p.fitness}")
# print(f"Rmse = {reg_p2p.inlier_rmse}")
# print(f"CorrespondenceSet = {reg_p2p.correspondence_set}")
# draw_registration_result(pcd_2, pcd, reg_p2p.transformation)



# def display_inlier_outlier(pcd, index):
#     InlinerPoint = pcd.select_by_index(index)
#     OutlinerPoint = pcd.select_by_index(index, invert=True)
#     OutlinerPoint.paint_uniform_color([1, 0, 0])
#     InlinerPoint.paint_uniform_color([0.8, 0.8, 0.8])
#     pcdshow([InlinerPoint, OutlinerPoint])
# cl, ind = DownPcd1.remove_statistical_outlier(nb_neighbors=50,
#                                                     std_ratio=0.5)
# display_inlier_outlier(DownPcd1, ind)


# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     labels = np.array(
#         DownPcd1.cluster_dbscan(eps=5, min_points=10, print_progress=True))

# max_label = labels.max()
# print(f"point cloud has {max_label + 1} clusters")
# colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
# colors[labels < 0] = 0
# DownPcd1.colors = o3d.utility.Vector3dVector(colors[:, :3])









