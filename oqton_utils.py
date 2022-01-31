#!/usr/bin/evn python3
"""
This script contains utilites for the NAICP and Octree prototype.
Created by: Vice, 26.01.2022
"""
# Import dependencies
import open3d as o3d
import numpy as np

def customDrawGeometry(model, data, point_size, window_name, transformation):
    """ Custom visualizer function for the NAICP prototype.
    Args:
        model ([open3d geom object: pcd, triangle mesh, image): Referent pointset
        data ([open3d geom object: pcd, triangle mesh, image]): Pointset to be aligned
        point_size ([int]): Point size in visualization
        window_name ([string]): Title for the visualization window
        transformation ([array]): Transformation array for the data pointset
    """
    # Initialize the main Visualizer class
    vis = o3d.visualization.Visualizer() 
    vis.create_window(
        window_name = window_name, 
        width = 720, 
        height = 720, 
        left = 25,
        top = 25 
        )

    # Add the geometry to the scene and create corresponding shaders
    vis.add_geometry(model)
    vis.add_geometry(data)

    # Do the transformation, if required
    if transformation is not None:
        data.transform(transformation)

    # Rendering options
    opt = vis.get_render_option()
    opt.show_coordinate_frame = True
    opt.point_size = point_size
    opt.light_on = True
    # Paint two clouds
    model.paint_uniform_color([1, 0.706, 0])
    data.paint_uniform_color([0, 0.651, 0.929]) 
    # opt.mesh_color_option = Color
    opt.mesh_show_wireframe = True
    opt. mesh_show_back_face = True

    # Change the view
    view = vis.get_view_control()
    view.rotate(250, 250)

    # Run the visualization
    vis.run()
    vis.destroy_window()

def customDrawOctree(data, window_name):
    """Custom Octree data structure visualizer.

    Args:
        data ([open3d geom point cloud]): Input data for the Octree
        window_name ([string]): Title for the visualization window
    """

    # Initialize the main Visualizer class
    vis = o3d.visualization.Visualizer() 
    vis.create_window(
        window_name = window_name, 
        width = 720, 
        height = 720, 
        left = 25,
        top = 25 
        )

    # Add the geometry to the scene and create corresponding shaders
    vis.add_geometry(data)

    # Rendering options
    opt = vis.get_render_option()
    opt.show_coordinate_frame = True
    opt.light_on = True

    # Change the view
    view = vis.get_view_control()
    view.rotate(250, 250)

    # Run the visualization
    vis.run()
    vis.destroy_window()

def customDrawSingle(data, point_size, window_name):
    """ Custom visualizer function for the single point cloud.
    Args:
        data ([open3d geom object: pcd, triangle mesh, image]): Input point cloud.
        point_size ([int]): Point size in visualization
        window_name ([string]): Title for the visualization window

    """
    # Initialize the main Visualizer class
    vis = o3d.visualization.Visualizer() 
    vis.create_window(
        window_name = window_name, 
        width = 720, 
        height = 720, 
        left = 25,
        top = 25 
        )

    # Add the geometry to the scene and create corresponding shaders
    vis.add_geometry(data)

    # Rendering options
    opt = vis.get_render_option()
    opt.show_coordinate_frame = True
    opt.point_size = point_size
    opt.light_on = True
    # Paint two clouds
    data.paint_uniform_color([0, 0.651, 0.929]) 

    # Change the view
    view = vis.get_view_control()
    view.rotate(250, 250)

    # Run the visualization
    vis.run()
    vis.destroy_window()

