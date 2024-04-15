import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
mesh_box = o3d.geometry.TriangleMesh.create_box(width=1.0,
                                                height=1.0,
                                                depth=1.0)
# mesh_box.translate((2,0,0))

# points = [
#     (0,0,0),
#     (1,0,0),
#     (1,1,0),
#     (0,1,0),
#     (0,0,1),
#     (1,0,1),
#     (1,1,1),
#     (0,1,1)
# ]
# lines = [
#     (0,1),
#     (1,2),
#     (2,3),
#     (3,0),
#     (4,5),
#     (5,6),
#     (6,7),
#     (4,7),
#     (0,4),
#     (1,5),
#     (2,6),
#     (3,7)
# ]

# colors = [[1, 0, 0] for i in range(len(lines))]
# LineSet = o3d.geometry.LineSet(
#     points = o3d.utility.Vector3dVector(points),
#     lines = o3d.utility.Vector2iVector(lines)
# )
# LineSet.colors = o3d.utility.Vector3dVector(colors)
# x = LineSet.get_center()
# print(x)
# y = LineSet.get_rotation_matrix_from_axis_angle
# o3d.visualization.draw_geometries([coordinate_frame,mesh_box,LineSet])

def prepare_data():
    pcd_data = o3d.data.DemoICPPointClouds()
    source_raw = o3d.io.read_point_cloud(pcd_data.paths[0])
    target_raw = o3d.io.read_point_cloud(pcd_data.paths[1])
    source = source_raw.voxel_down_sample(voxel_size=0.02)
    target = target_raw.voxel_down_sample(voxel_size=0.02)

    trans = [[0.862, 0.011, -0.507, 0.0], [-0.139, 0.967, -0.215, 0.7],
             [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]]
    source.transform(trans)
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    source.transform(flip_transform)
    target.transform(flip_transform)
    return source, target


def demo_non_blocking_visualization():
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    source, target = prepare_data()
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(source)
    vis.add_geometry(target)
    threshold = 0.05
    icp_iteration = 100
    save_image = False

    for i in range(icp_iteration):
        reg_p2l = o3d.pipelines.registration.registration_icp(
            source, target, threshold, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
        source.transform(reg_p2l.transformation)
        vis.update_geometry(source)
        vis.poll_events()
        vis.update_renderer()
        if save_image:
            vis.capture_screen_image("temp_%04d.jpg" % i)
    vis.destroy_window()

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)


if __name__ == '__main__':
    demo_non_blocking_visualization()
