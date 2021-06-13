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


class Obstacle:
    """A class for rectangle obstacles on the robot's map"""

    def __init__(self, x, y, size_x, size_y):
        """Initializes the obstacle"""
        # Position on the map
        self.x = x
        self.y = y

        # Size of the rectangle
        self.sizeX = size_x
        self.sizeY = size_y

        # Color of the obstacle
        red = int(random.random() * 255)
        green = int(random.random() * 255)
        blue = int(random.random() * 255)
        self.color = (red, green, blue)
