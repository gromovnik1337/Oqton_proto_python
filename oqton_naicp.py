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

    # Transform the data towards the model according to the 1st ICP results
    RPD_data_PCD.transform(ICP_transform)
    # Save the position of the best cloud position, its transformation and the step where it was achieved
    ICP_transform_best = ICP_transform
    fitness_best = ICP_registration.fitness
    RPD_data_PCD_best = copy.deepcopy(RPD_data_PCD)
    best_fit_step = 0

    # Create hosting structure for saving multiple transformation matrices and ICP fitness results
    ICP_transform_XYZ = np.zeros((4, 4, 3)) # 3D array to append the transformation matrices for the ICP results after rotating around X, y and Z axis
    ICP_fitness_XYZ = np.zeros(3)

    # "Wiggle algorithm" - rotations + ICP attempts to find the best fit
    # ----------------------------------------------------------------
    for i in range(wiggle_steps):

        print("Wiggle algorithm: rotation + ICP, {0}th step.".format(i+1))
        
        # Rotate the data - X axis
        # ----------------------------------------------------------------
        RPD_data_PCD_temp_X = copy.deepcopy(RPD_data_PCD)
        RPD_data_PCD_temp_X.rotate(RPD_data_PCD_temp_X.get_rotation_matrix_from_xyz((rotation_step, 0, 0)))
        axis_current = "X"

        # ICP
        ICP_registration = o3d.pipelines.registration.registration_icp(
            RPD_model_PCD, RPD_data_PCD_temp_X, threshold_icp, ICP_transform_best,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = max_iteration)
            )

        ICP_transform_XYZ[:, :, 0] = ICP_registration.transformation
        ICP_fitness_XYZ[0] = ICP_registration.fitness
        
        print("Rotation around: {0} axis, ICP fitness: {1}, RMSE inliers: {2}.".format(axis_current, ICP_registration.fitness, round(ICP_registration.inlier_rmse, 2)))
        #customDrawGeometry(RPD_model_PCD, RPD_data_PCD_temp_X, 5, "Current status", ICP_registration.transformation)

        # Rotate the data - Y axis
        # ----------------------------------------------------------------
        RPD_data_PCD_temp_Y = copy.deepcopy(RPD_data_PCD)
        RPD_data_PCD_temp_Y.rotate(RPD_data_PCD_temp_Y.get_rotation_matrix_from_xyz((0, rotation_step, 0)))
        axis_current = "Y"
        
        # ICP
        ICP_registration = o3d.pipelines.registration.registration_icp(
            RPD_model_PCD, RPD_data_PCD_temp_Y, threshold_icp, ICP_transform_best,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = max_iteration)
            )

        ICP_transform_XYZ[:, :, 1] = ICP_registration.transformation
        ICP_fitness_XYZ[1] = ICP_registration.fitness

        print("Rotation around: {0} axis, ICP fitness: {1}, RMSE inliers: {2}.".format(axis_current, ICP_registration.fitness, round(ICP_registration.inlier_rmse, 2)))
        #customDrawGeometry(RPD_model_PCD, RPD_data_PCD_temp_Y, 5, "Current status", ICP_registration.transformation)

        # Rotate the data - Z axis
        # ----------------------------------------------------------------
        RPD_data_PCD_temp_Z = copy.deepcopy(RPD_data_PCD)
        RPD_data_PCD_temp_Z.rotate(RPD_data_PCD_temp_Z.get_rotation_matrix_from_xyz((0, 0, rotation_step)))
        axis_current = "Z"

        # ICP
        ICP_registration = o3d.pipelines.registration.registration_icp(
            RPD_model_PCD, RPD_data_PCD_temp_Z, threshold_icp, ICP_transform_best,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = max_iteration)
            )

        ICP_transform_XYZ[:, :, 2] = ICP_registration.transformation
        ICP_fitness_XYZ[2] = ICP_registration.fitness

        print("Rotation around: {0} axis, ICP fitness: {1}, RMSE inliers: {2}.".format(axis_current, ICP_registration.fitness, round(ICP_registration.inlier_rmse, 2)))
        #customDrawGeometry(RPD_model_PCD, RPD_data_PCD_temp_Z, 5, "Current status", ICP_registration.transformation)

        # Chose the best achieved fit (if it happened) for all conducted rotations:
        # 1. Update the values
        # 2. Transform the data towards the model
        # 3. Use the last transformation function as the input for the next ICP, otherwise, only rotate the cloud
        # 4. If no better fit achieved, rotate the data around the axis whose rotation achieved the best fit and do the center to center transformation again (to correct for possible ICP flukes)
        no_better_fit = False

        for j in range(ICP_fitness_XYZ.size):

            if ICP_fitness_XYZ[j] >= fitness_best:

                RPD_data_PCD.transform(ICP_transform_XYZ[:, :, j])
                ICP_transform_best = ICP_transform_XYZ[:, :, j] 
                fitness_best = ICP_fitness_XYZ[j]           
                RPD_data_PCD_best = copy.deepcopy(RPD_data_PCD) # Update the final transformation
                best_fit_step = i+1
                no_better_fit = True

                print("Better fit achieved after rotation around {0} axis (0 = X, 1 = Y, Z = 2).".format(ICP_fitness_XYZ[j]))

        if no_better_fit == False:

            best_fit_axis_index = np.argmax(ICP_fitness_XYZ)
            print("No better fit achieved, rotating around {0} axis (0 = X, 1 = Y, Z = 2).".format(best_fit_axis_index))

            if best_fit_axis_index == 0: 

                RPD_data_PCD.rotate(RPD_data_PCD.get_rotation_matrix_from_xyz((rotation_step, 0, 0)))

            if best_fit_axis_index == 1: 

                RPD_data_PCD.rotate(RPD_data_PCD.get_rotation_matrix_from_xyz((0, rotation_step, 0)))

            if best_fit_axis_index == 2: 

                RPD_data_PCD.rotate(RPD_data_PCD.get_rotation_matrix_from_xyz((0, 0, rotation_step)))

            # Center to center translation
            translation_vector = RPD_model_PCD.get_center() - RPD_data_PCD.get_center()
            RPD_data_PCD.translate(translation_vector)

        print("Best fitness: ", fitness_best)
        print("\n")

        # Visualize the current status - if required
        #customDrawGeometry(RPD_model_PCD, RPD_data_PCD, 5, "Current status", None)

    print("Final best fitness is {0}, achieved in step number {1}.".format(fitness_best, best_fit_step))
    print("Final best transformation:")
    print(ICP_transform_best)
    customDrawGeometry(RPD_model_PCD, RPD_data_PCD_best, 5, "Point to point ICP - final", None)
