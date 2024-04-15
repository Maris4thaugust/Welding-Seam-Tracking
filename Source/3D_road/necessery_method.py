import math
import numpy as np

def plane_Oxy_angle(a1, b1, c1):
    # Phương trình mặt phẳng 1: a1x + b1y + c1z + d1 = 0
    # Phương trình mặt phẳng Oxy: z = 0
    a2, b2, c2 = 0, 0, 1
    # Vector pháp tuyến của mặt phẳng thứ nhất và thứ 2
    n1 = [a1, b1, c1]
    n2 = [a2, b2, c2]
    # Tính tích vô hướng của 2 vector pháp tuyến
    dot_product = sum([n1[i] * n2[i] for i in range(3)])
    # Tính độ dài của vector pháp tuyến mỗi mặt phẳng
    mag1 = math.sqrt(sum([n1[i] ** 2 for i in range(3)]))
    mag2 = math.sqrt(sum([n2[i] ** 2 for i in range(3)]))
    # Tính cos của góc giữa 2 mặt phẳng
    cos_angle = dot_product / (mag1 * mag2)
    return math.acos(cos_angle)

def find_plane(pcd):
    # Tính toán mặt phẳng với thuật toán RANSAC
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                            ransac_n=3,
                                            num_iterations=1000)
    # inlier_points = pcd.select_by_index(inliers).voxel_down_sample(voxel_size=0.05)
    # outlier_points = pcd.select_by_index(inliers, invert=True).voxel_down_sample(voxel_size=0.05)
    # Lấy mặt phẳng từ mô hình
    [a, b, c, d] = plane_model
    # print("Mặt phẳng tìm được: {}x + {}y + {}z + {} = 0".format(a, b, c, d))
    return a, b, c

def rotateY(pcd, angle):
    c = np.cos(angle)
    s = np.sin(angle)
    init_st_matrix_rotY = np.asarray([[c, 0, s, 0],
                                      [0, 1, 0, 0],
                                      [-s, 0, c, 0],
                                      [0, 0, 0, 1]])
    pcd.transform(init_st_matrix_rotY)
    return pcd

def rotateX(pcd, angle):
    c = np.cos(angle)
    s = np.sin(angle)
    init_st_matrix_rotX = np.asarray([[1, 0, 0, 0],
                                      [0, c, -s, 0],
                                      [0, s, c, 0],
                                      [0, 0, 0, 1]])
    pcd.transform(init_st_matrix_rotX)
    return pcd

def rotateZ180(pcd):
    c = np.cos(np.pi)
    s = np.sin(np.pi)
    init_st_matrix_rotZ180 = np.asarray([[-1, 0, 0, 0],
                                        [0, -1, 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
    pcd.transform(init_st_matrix_rotZ180)
    return pcd

def rotateZ(pcd, angle):
    c = np.cos(angle)
    s = np.sin(angle)
    init_st_matrix_rotZ = np.asarray([[c, -s, 0, 0],
                                     [s, c, 0, 0],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])
    pcd.transform(init_st_matrix_rotZ)
    return pcd

def bbox(pcd):
    bbox = pcd.get_axis_aligned_bounding_box()
    min_bound = bbox.get_min_bound()
    max_bound = bbox.get_max_bound()
    return pcd, min_bound, max_bound

def move2Ori(pcd, i):
    angl = i*np.pi/36
    c = np.cos(angl)
    s = np.sin(angl)
    init_st_matrix_rotX = np.asarray([[1, 0, 0, 0],
                                      [0, c, -s, 0],
                                      [0, s, c, 0],
                                      [0, 0, 0, 1]])
    pcd.transform(init_st_matrix_rotX)
    bbox = pcd.get_axis_aligned_bounding_box()
    min_bound = bbox.get_min_bound()
    max_bound = bbox.get_max_bound()
    init_st_matrix_trans = np.asarray([[1, 0, 0, -max_bound[0]],
                                       [0, 1, 0, -max_bound[1]],
                                       [0, 0, 1, -max_bound[2]],
                                       [0, 0, 0, 1]])
    pcd.transform(init_st_matrix_trans)
    return pcd