import open3d as o3d
import numpy as np
import necessery_method as nm
from urx.robot import Robot
import time
import keyboard
import os

# Vị trí (m) và hướng (Rad) mốc đồ gá với htd Robot 
Ori_00 = np.array([-0.38740, -0.51182, 0.21777, 1.9585, 1.9615, -0.5977]) 
# Vị trí (mm) của mốc đồ gá trên htd Scanner
Ori_00c = np.array([9.43, -3.44, 742.86]) 

sample = o3d.io.read_point_cloud(f"merged_cloud_v4_1.ply")
sample.paint_uniform_color([1, 0.706, 0])

cou = 1
while (1):
    print("Press G to get points")
    while (1):
        if keyboard.is_pressed('g'):
            if cou > 1:
                saData = o3d.io.read_point_cloud(f"merged_cloud_v5.ply")
            else: 
                saData = o3d.io.read_point_cloud(f"merged_cloud_v4_1.ply")
            try:
                robot = Robot("192.168.1.45")
                robot.set_tcp((0, 0.07, 0.284, 0, 0, 0))
                robot.set_payload(1, (0, 0, 0.1))
                time.sleep(0.1)

                pose = np.array(robot.getl()[0:3]) 
                pose = Ori_00c + (pose - Ori_00[0:3])*1000*[1, -1, -1]
                pose_f = pose.copy()
                pose_f[2] -= 10
                robot.close()
                os.system('cls')
                break
            except:
                print("Cannot detect the connection! Try pressing G agian!")
                time.sleep(1.5)
                continue
        else:
            continue   

    print(pose)
    sample_tree = o3d.geometry.KDTreeFlann(sample)
    [k1, sample_idx, _] = sample_tree.search_knn_vector_3d(pose_f, 1) 
    [k0, sample_idx_0, _] = sample_tree.search_knn_vector_3d(pose, 1) 

    with open('zList_idx.txt', 'a') as file:
        file.write(str(sample_idx[0]) + '\n')
    print(sample.points[sample_idx[0]])
    print(sample.normals[sample_idx[0]])

    small_cs = o3d.geometry.TriangleMesh.create_coordinate_frame(size=15)
    small_cs.translate(sample.points[sample_idx[0]])

    # c2r = sample.points[sample_idx[0]] + 10*np.array(sample.normals[sample_idx[0]])
    c2r = sample.points[sample_idx_0[0]] - 10*np.array([0, 0, 1])

    print("RobotPoint in Scanner Coodinate: ", c2r)
    c2r = (c2r - Ori_00c + Ori_00[0:3]*1000*[1, -1, -1])*[1, -1, -1]
    print("RobotPoint: ", c2r)
    print("Press W to write data point/Press Q to quit process")

    while (1):
        if keyboard.is_pressed('w'):
            with open('zList_Points.txt', 'a') as file:
                file.write(str(c2r/1000) + '\n')
                print(f"Finish writing ({cou}) points --> zList_Points.txt")
                o3d.visualization.draw_geometries([sample, small_cs], point_show_normal=False)
                file.close()
            break
        if keyboard.is_pressed('q'):
            exit()   
    cou += 1

    small_cs = small_cs.sample_points_uniformly(500)
    saData += small_cs
    o3d.io.write_point_cloud("./merged_cloud_v5.ply", saData)
    

# vis = o3d.visualization.Visualizer()
# vis.create_window()
# vis.get_render_option().background_color = [0.8, 0.5, 0.8] 
# vis.add_geometry(sample)
# vis.add_geometry(small_cs)
# vis.run()
# vis.destroy_window()
