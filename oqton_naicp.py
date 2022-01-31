#!/usr/bin/evn python3
#TODO Create a virtual environment for the Python development of this pipeline, if it would require more larger dependancies,
# other than Open3D --> possible conflicts.
"""
This script was made to the test the NAICP idea for improving the local minima convergence of the two RPD models.
It is based on a simple assumption that two RPD models will only differentiate in rotation after one of them had been translated to 
the center of the other. After initial ICP alignment, the aligned model is rotated around the X axis in discrete steps, and every time
a new ICP alignment is done. If the fitting improves, the result is updated. 
Created by: Vice, 26.01.2022
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

    # Offset one of the clouds away to test the ICP method
    RPD_data_PCD.translate([50000, 20000, 20000])
    RPD_data_PCD.translate([0, 20000, 20000])
    RPD_data_PCD.rotate(RPD_data_PCD.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4)))

    # Visualize initial state
    customDrawGeometry(RPD_model_PCD, RPD_data_PCD, 3, "Initial state", None)

    # NAICP (proto)
    # ----------------------------------------------------------------

    # Center to center translation
    translation_vector = RPD_model_PCD.get_center() - RPD_data_PCD.get_center()
    RPD_data_PCD.translate(translation_vector)
    customDrawGeometry(RPD_model_PCD, RPD_data_PCD, 3, "Center to center transformation", None)                          

    # 1st ICP
    print("Applying 1st point-to-point ICP")
    ICP_registration = o3d.pipelines.registration.registration_icp(
        RPD_model_PCD, RPD_data_PCD, threshold_icp, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = max_iteration)
        )

    print(ICP_registration)
    print("Transformation is:")
    ICP_transform = ICP_registration.transformation
    print(ICP_transform)

    customDrawGeometry(RPD_model_PCD, RPD_data_PCD, 3, "Point to point ICP - 1st", ICP_transform)

    # Transform the data towards the model
    ICP_transform_best = ICP_transform
    RPD_data_PCD.transform(ICP_transform)
    RPD_data_PCD_best = copy.deepcopy(RPD_data_PCD)

    for i in range(8):
        
        # Rotate the data
        RPD_data_PCD.rotate(RPD_data_PCD.get_rotation_matrix_from_xyz((np.pi / 8, 0, 0)))
        #customDrawGeometry(RPD_model_PCD, RPD_data_PCD, 3, "Rotation", ICP_transform)

        # ICP
        print("Applying point-to-point ICP in {0}th loop.".format(i+1))
        ICP_registration = o3d.pipelines.registration.registration_icp(
            RPD_model_PCD, RPD_data_PCD, threshold_icp, ICP_transform_best,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = max_iteration)
            )

        ICP_transform = ICP_registration.transformation
        print("Current fitness: ", ICP_registration.fitness)

        if i == 0:
            fitness_best = ICP_registration.fitness
            ICP_transform_best = ICP_transform

        # If better fit is achieved:
        # 1. Update the values
        # 2. Transform the data towards the model
        # 3. Use the last transformation function as the input for the next ICP, otherwise, only rotate the cloud
        if ICP_registration.fitness >= fitness_best:
            fitness_best = ICP_registration.fitness

            # Save the position of the best cloud position and its transformations
            ICP_transform_best = ICP_transform 
            RPD_data_PCD.transform(ICP_transform)
            RPD_data_PCD_best = copy.deepcopy(RPD_data_PCD)

        print("Best fitness: ", fitness_best)

    print("Final best fitness: ", fitness_best)
    print("Final best transformation: ", ICP_transform_best)
    customDrawGeometry(RPD_model_PCD, RPD_data_PCD_best, 3, "Point to point ICP - final", None)
