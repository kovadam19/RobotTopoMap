##################################################
# Project: Robot Topological Mapping
# Author: Adam Kovacs
# Version: 1.0.0
# Maintainer: Adam Kovacs
# E-mail: kovadam19@gmail.com
# Released: 13 June 2021
##################################################

# Generic/Built-in imports
import random
from scipy.spatial import ConvexHull
import numpy as np


class Cluster:
    """A class to manage convex clusters on the robot's map"""

    def __init__(self, center_x, center_y):
        """Create a cluster on to the robot's map"""
        # Center coordinates
        self.centerX = center_x
        self.centerY = center_y

        # Outermost X coordinates
        self.x_min = center_x
        self.x_max = center_x

        # Color
        red = int(random.random() * 255)
        green = int(random.random() * 255)
        blue = int(random.random() * 255)
        self.color = (red, green, blue)

        # Cells
        self.cells = set()
        self.cells.add((self.centerX, self.centerY))

        # Slices
        self.slices = {self.centerX: set()}
        self.slices[self.centerX].add(self.centerY)

        # Neighbouring clusters
        self.neighbours = set()

        # Navigation node positions
        self.navigation_node_positions = set()

    def _update_outermost_x_coords(self, new_cells):
        """Updates the coordinates of outermost cells in X direction"""
        for cell in new_cells:
            if cell[0] > self.x_max:
                self.x_max = cell[0]
            elif cell[0] < self.x_min:
                self.x_min = cell[0]

    def _update_cells(self, new_cells):
        """Updates the cells"""
        for cell in new_cells:
            self.cells.add(cell)

    def _update_slices(self, new_cells):
        """Updates the slices"""
        for cell in new_cells:
            if cell[0] in self.slices:
                self.slices[cell[0]].add(cell[1])
            else:
                self.slices[cell[0]] = set()
                self.slices[cell[0]].add(cell[1])

    def add_cells(self, new_cells):
        """Adds new cells to the cluster"""
        self._update_outermost_x_coords(new_cells)
        self._update_cells(new_cells)
        self._update_slices(new_cells)

    def get_perimeter_points(self, outermost=True, safety_zone=0):
        """Collects the outermost perimeter points or perimeter points within a safety zone"""
        perimeter_points = set()
        # Check if we need to collect the outermost points
        if outermost:
            # Go though the slices
            for x in self.slices:
                # Check if the current slice is one of the outermost slices
                if x == self.x_min or x == self.x_max:
                    # Collect all the points from the outermost slice
                    for y in self.slices[x]:
                        perimeter_points.add((x, y))
                else:
                    # Collect the points with min and max y coordinates from the slice
                    perimeter_points.add((x, min(self.slices[x])))
                    perimeter_points.add((x, max(self.slices[x])))
            return perimeter_points
        else:
            # Calculate the outermost slices with respect to the safety zone
            x_min = self.x_min + safety_zone
            x_max = self.x_max - safety_zone
            # Go through the slices
            for x in self.slices:
                # Check if the current slice is one of the outermost slices
                if x == x_min or x == x_max:
                    # Calculate the min and max y coordinates by taking into the safety zone account
                    y_min = min(self.slices[x]) + safety_zone
                    y_max = max(self.slices[x]) - safety_zone
                    # Go through the y coordinates in the slice
                    for y in self.slices[x]:
                        # Check if the y coordinate falls between the min and max ones
                        if y_min <= y <= y_max:
                            perimeter_points.add((x, y))
                elif x_min < x < x_max:
                    # Get the cells with the max and min coordinates then apply the safety zone on them
                    perimeter_points.add((x, min(self.slices[x]) + safety_zone))
                    perimeter_points.add((x, max(self.slices[x]) - safety_zone))
            return perimeter_points

    def update_center_point(self):
        """Updates the center point of the cluster with the mean of coordinates in each direction"""
        # Collecting all the x & y coordinates
        x_coords = []
        y_coords = []
        for cell in self.cells:
            x_coords.append(cell[0])
            y_coords.append(cell[1])
        # Calculating the mean of the coordinates
        self.centerX = np.mean(x_coords)
        self.centerY = np.mean(y_coords)

    def point_in_cluster(self, point, tolerance=1e-12):
        """
            Checks if a point falls into a given convex hull of the cluster. Returns True or False.
            Implemented from: https://stackoverflow.com/a/42165596
        """
        # Calculate the convex hull for the cluster
        hull = ConvexHull(list(self.cells))
        # Check if the point is in the hull
        return all((np.dot(eq[:-1], point) + eq[-1] <= tolerance) for eq in hull.equations)






