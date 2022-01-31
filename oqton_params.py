#!/usr/bin/evn python3
"""
This script contains only the parameters for the NAICP and Octree low density Oqton prototypes.
Created by: Vice, 31.10.2022
"""

import numpy as np

# General parameters
# ----------------------------------------------------------------

# File names
rpd_model = "./data/RPD_model.stl"
rpd_data = "./data/RPD_data.stl"

# Point cloud sampling - determines the ammmount of points in the cloud converted from the .stl file
rpd_sampling = 5000

# NAICP parameters
# ----------------------------------------------------------------

# Basic ICP parameters
trans_init = np.array([[1., 0., 0., 0],
                        [0., 1., 0., 0],
                        [0., 0., 1., 0],
                        [0., 0., 0., 1.]])
threshold_icp = 2000
max_iteration = 15000

# "Wiggle" algorithm parameters
wiggle_steps = 20
rotation_step = np.pi / (wiggle_steps/5)

# Octree parammeters
# ----------------------------------------------------------------

max_depth = 3
threshold_octree = 150