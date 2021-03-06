##################################################
# Project: Robot Topological Mapping
# Author: Adam Kovacs
# Version: 1.0.0
# Maintainer: Adam Kovacs
# E-mail: kovadam19@gmail.com
# Released: 13 June 2021
##################################################

# Generic/Built-in imports
import numpy as np


class PathNode:
    """Class to represent a node on the path for autonomous navigation"""

    def __init__(self, cluster=None, parent_node=None, position=None):
        """Initialize the node class"""
        # Cluster, Parent node, position
        self.cluster = cluster
        self.parent_node = parent_node
        self.position = position

        # Cost from parent node to this node
        self.g = np.inf

        # Estimated cost from this node to the end node
        self.h = np.inf

        # Total estimated cost
        self.f = np.inf

    def __eq__(self, other):
        """Checks if this node is equal to the other"""
        return self.position == other.position
