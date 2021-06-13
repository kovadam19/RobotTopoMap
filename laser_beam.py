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
import pygame
from pygame.sprite import Sprite


class Beam(Sprite):
    """A class to manage laser beams emitted from the robot's LIDAR"""

    def __init__(self, simulation, orientation):
        """Create a beam emitted from the robot's LIDAR"""
        # Initialize the sprite and get simulation screen and settings
        super().__init__()
        self.orientation = orientation
        self.screen = simulation.screen
        self.settings = simulation.settings
        self.color = self.settings.laser_color

        # Initial position based on the robot's orientation
        self.init_x = simulation.robot.x - np.sin(simulation.robot.o) * self.settings.lidar_position
        self.init_y = simulation.robot.y - np.cos(simulation.robot.o) * self.settings.lidar_position

        # Precise position
        self.x = self.init_x.copy()
        self.y = self.init_y.copy()

        # Create a beam rect at the lidar's emitter
        self.rect = pygame.Rect(self.x, self.y, self.settings.laser_width, self.settings.laser_height)

        # Free to move flag
        self.free_to_move = True

    def update(self):
        """Move the beam into the orientation"""
        if self.free_to_move:
            self.x = self.x - self.settings.laser_speed * np.sin(self.orientation)
            self.y = self.y - self.settings.laser_speed * np.cos(self.orientation)

            self.rect.center = (self.x, self.y)

    def draw_beam(self):
        """Draw beam to the screen"""
        pygame.draw.circle(self.screen, self.color, (self.x, self.y), self.settings.laser_width / 2)
