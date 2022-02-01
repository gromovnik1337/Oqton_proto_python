#!/usr/bin/evn python3
"""
This script casts the the RPD point cloud into the Octree data shape and subsequently
traverses through it and extrapolates only points in the nodes of low density, segmenting the point cloud.
Created by: Vice, 26.01.2022
"""
# Import dependencies
import open3d as o3d
import numpy as np
from oqton_utils import customDrawOctree, customDrawSingle
from oqton_params import *

if __name__ == "__main__":

    # Input the .stl files
    RPD_model = o3d.io.read_triangle_mesh(rpd_model)

    # Create point clouds from .stl files - (simple)
    RPD_model_PCD = RPD_model.sample_points_uniformly(rpd_sampling)
    RPD_model_PCD_numpy = np.asarray(RPD_model_PCD.points)

    octree = o3d.geometry.Octree(max_depth)
    octree.convert_from_point_cloud(RPD_model_PCD, size_expand = 0.01)

    customDrawOctree(octree, "Octree data structure of the RPD point cloud")

    class traverser:

        critical_indices = []

        def traverseCallback(self, node, node_info):
            """Traversal callback function, modified from: http://www.open3d.org/docs/latest/tutorial/geometry/octree.html

            Args:
                node ([pen3d.cpu.pybind.geometry.OctreeNode]): Node (internal or leaf) in the Octree structure.
                node_info ([open3d.cpu.pybind.geometry.OctreeNodeInfo]): Info data attached to the specific node.

            Raises:
                NotImplementedError: Intrinsic to Open3D Octree data structure
            """

            # Save indices of the points inside the nodes that have less than "threshold_octree_octree points"
            # Internal nodes
            if isinstance(node, o3d.geometry.OctreeInternalNode):

                if isinstance(node, o3d.geometry.OctreeInternalPointNode):
                    n = 0
                    for child in node.children:
                        if child is not None:
                            n += 1
                    print("{} {}: Internal node at depth {} has {} children and {} points"
                    .format('    ' * node_info.depth,
                    node_info.child_index, node_info.depth, n,
                    len(node.indices)))
                    if len(node.indices) < threshold_octree:
                        self.critical_indices.extend(node.indices)

            # Leaf nodes
            elif isinstance(node, o3d.geometry.OctreeLeafNode):

                if isinstance(node, o3d.geometry.OctreePointColorLeafNode):

                    print("{}{}: Leaf node at depth {} has {} points"
                    .format('    ' * node_info.depth, node_info.child_index,
                                node_info.depth, len(node.indices)))
            else:
                raise NotImplementedError('Node type not recognized!')

    # Create the traverser object and iterate through the Octree data structure
    look_through_nodes = traverser()
    octree.traverse(look_through_nodes.traverseCallback)

    # Create a point cloud made only of points that are in the Octree nodes with less than "threshold_octree points"
    RPD_model_segmented = o3d.geometry.PointCloud()
    RPD_model_segmented.points = o3d.utility.Vector3dVector(RPD_model_PCD_numpy[look_through_nodes.critical_indices])

    customDrawSingle(RPD_model_PCD, 3, "Full point cloud")
    customDrawSingle(RPD_model_segmented, 3, "Segmented regions")

