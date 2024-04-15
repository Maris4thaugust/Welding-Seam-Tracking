import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import copy
import os
import glob

def pclread(path):
    if os.path.exists(path):
        pcd = o3d.io.read_point_cloud(path)
    else:
        print("Cannot find directory")
    return pcd
def pcdshow(Pcds):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for pcd in Pcds:      
        vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

def PreProcess(pcd,VoxelSize):
    RadiusNormal = VoxelSize*2
    RadiusFeature = VoxelSize*5

    PcdDown = pcd.voxel_down_sample(VoxelSize)
    PcdDown.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=RadiusNormal, max_nn=30))
    PcdFpfh = o3d.pipelines.registration.compute_fpfh_feature(
        PcdDown,
        o3d.geometry.KDTreeSearchParamHybrid(radius=RadiusFeature, max_nn=100))
        
    return PcdDown, PcdFpfh

def AxisDefine():
    
    return

def CropROI(pcd):
    CropPcd = pcd.crop
    return CropPcd


