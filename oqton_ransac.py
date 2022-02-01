#!/usr/bin/evn python3
"""
This script was made to the test the RANSAC plane fitting into the RPD model.
Created by: Vice, 01.02.2022
"""
# Import dependencies
import open3d as o3d
import numpy as np
from oqton_utils import customDrawGeometry
from oqton_params import *

if __name__ == "__main__":

    # Input the .stl files
    RPD_model = o3d.io.read_triangle_mesh(rpd_model)
    RPD_data = o3d.io.read_triangle_mesh(rpd_data)

    # Create point clouds from .stl files - (simple)
    RPD_model_PCD = RPD_model.sample_points_uniformly(rpd_sampling)
    RPD_data_PCD = RPD_data.sample_points_uniformly(rpd_sampling)

    # Offset one of the clouds away to test the ICP method
    RPD_data_PCD.translate([50000, 20000, 20000])
    RPD_data_PCD.translate([0, 20000, 20000])
    RPD_data_PCD.rotate(RPD_data_PCD.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4)))

    # Visualize initial state
    #customDrawGeometry(RPD_model_PCD, RPD_data_PCD, 3, "Initial state", None)

    # Segment a plane in one point cloud
    plane_model, inliers = RPD_data_PCD.segment_plane(distance_threshold = 300,
                                            ransac_n = 500,
                                            num_iterations = 2000)

    inlier_cloud = RPD_data_PCD.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = RPD_data_PCD.select_by_index(inliers, invert=True)
    outlier_cloud.paint_uniform_color([0, 0, 1.0])

    print(inlier_cloud)
    print(outlier_cloud)

    # Visualize the first cloud with the segmented plane
    customDrawGeometry(inlier_cloud, outlier_cloud, 3, "Cloud and a plane", None)

    