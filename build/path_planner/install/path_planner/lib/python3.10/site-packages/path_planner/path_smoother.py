#!/usr/bin/env python3

import copy

from geometry_msgs.msg import Point

from.graph import is_occluded

class PathSmoother():
    def __init__(self, parent_node, graph, alpha, beta):
        self.parent_node_ = parent_node
        self.graph_ = graph
        
        self.alpha_ = alpha
        self.beta_ = beta

        self.path_smooth_ = []

    def smooth_path(self, path_nodes):
        """Smooth the path to remove sharp corners resulting from the grid-based planning"""

        self.parent_node_.get_logger().info('Smoothing path...')

        # Convert into into a geometry_msgs.Point[]
        path = []

        for node in path_nodes:
            p = Point()
            p.x = float(node.x)
            p.y = float(node.y)
            path.append(p)

        # Initialise the smooth path
        path_smooth = copy.deepcopy(path)

        # Loop until the smoothing converges
        # In each iteration, update every waypoint except the first and last waypoint

        ####################
        ## YOUR CODE HERE ##
        ## Task 5         ##
        ####################
        




















        self.path_smooth_ = path_smooth