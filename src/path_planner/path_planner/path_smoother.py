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

    def smoothing_formula(self, i, path, path_smooth, coord):
        """Compute the smoothed coordinate (x or y) for waypoint i."""
        return ((1 - self.alpha_ - 2 * self.beta_) * getattr(path_smooth[i], coord)
                + self.alpha_ * getattr(path[i], coord)
                + self.beta_ * getattr(path_smooth[i - 1], coord)
                + self.beta_ * getattr(path_smooth[i + 1], coord))

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

        e = 0.001
        change = 999999

        while change >= e:
            change = 0.0

                # Loop over all points except start and goal
            for i in range(1, len(path) - 1):
                old_x, old_y = path_smooth[i].x, path_smooth[i].y

                new_x = self.smoothing_formula(i, path, path_smooth, "x")
                new_y = self.smoothing_formula(i, path, path_smooth, "y")

                if not is_occluded(self.graph_.map_.obstacle_map_,
                                [path_smooth[i - 1].x, path_smooth[i - 1].y],
                                [new_x, new_y]) and \
                not is_occluded(self.graph_.map_.obstacle_map_,
                                [new_x, new_y],
                                [path_smooth[i + 1].x, path_smooth[i + 1].y]):
                    path_smooth[i].x = new_x
                    path_smooth[i].y = new_y

                change += (path_smooth[i].x - old_x) ** 2 + (path_smooth[i].y - old_y) ** 2



        self.path_smooth_ = path_smooth