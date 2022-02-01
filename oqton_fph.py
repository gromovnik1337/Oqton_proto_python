#!/usr/bin/evn python3
"""
This script was made to the test the FPFH approach for the global registration of the RPD models.
Created by: Vice, 01.02.2022
"""
# Import dependencies
import open3d as o3d
import numpy as np
import copy
from oqton_utils import customDrawGeometry
from oqton_params import *

if __name__ == "__main__":

    # Input the .stl files
    RPD_model = o3d.io.read_triangle_mesh(rpd_model)
    RPD_data = o3d.io.read_triangle_mesh(rpd_data)

    # Create point clouds from .stl files - (simple)
    RPD_model_PCD = RPD_model.sample_points_uniformly(rpd_sampling)
    RPD_data_PCD = RPD_data.sample_points_uniformly(rpd_sampling)

    # Offset one of the clouds away to test the registration method
    RPD_data_PCD.translate([50000, 20000, 20000])
    RPD_data_PCD.translate([0, 20000, 20000])
    RPD_data_PCD.rotate(RPD_data_PCD.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4)))

    # Do the pre-processing (voxel downsampling + FPFH)
    # TODO Refactor!
    # ----------------------------------------------------------------

    voxel_size = 2000

    RPD_model_PCD = RPD_model_PCD.voxel_down_sample(voxel_size)
    RPD_data_PCD = RPD_data_PCD.voxel_down_sample(voxel_size)

    # Visualize 
    #customDrawGeometry(RPD_model_PCD, RPD_data_PCD, 3, "Voxel downsampling", None)

    radius_normal = voxel_size * 2

    RPD_model_PCD.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius = radius_normal, max_nn = 30)) # Check
    RPD_data_PCD.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius = radius_normal, max_nn = 30))

    radius_feature = 50 # Radius of the K-neighbourhood
    max_nn_fpfh = 100 # Maximum number of neighbouhrs that is to be searched 

    RPD_model_PCD_fpfh = o3d.pipelines.registration.compute_fpfh_feature(RPD_model_PCD, o3d.geometry.KDTreeSearchParamHybrid(radius = radius_feature, max_nn = max_nn_fpfh))
    RPD_data_PCD_fpfh = o3d.pipelines.registration.compute_fpfh_feature(RPD_data_PCD, o3d.geometry.KDTreeSearchParamHybrid(radius = radius_feature, max_nn = max_nn_fpfh))

    # Do the global registration
    # ----------------------------------------------------------------

    # Center to center translation
    translation_vector = RPD_model_PCD.get_center() - RPD_data_PCD.get_center()
    RPD_data_PCD.translate(translation_vector)

    # http://www.open3d.org/docs/latest/python_api/open3d.pipelines.registration.registration_ransac_based_on_feature_matching.html
    distance_threshold = 10000

    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        RPD_model_PCD, RPD_data_PCD, RPD_model_PCD_fpfh, RPD_data_PCD_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance = distance_threshold,
            decrease_mu = False,
            iteration_number = 10000,
            use_absolute_scale = False
            )
        )

    print(result)
    customDrawGeometry(RPD_model_PCD, RPD_data_PCD, 5, "Global alignment", result.transformation)

    