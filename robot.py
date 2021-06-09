import numpy as np
import copy
import pygame
from random import random
from pygame.sprite import Sprite


class Robot(Sprite):
    """Class for the robot"""

    def __init__(self, simulation, init_x, init_y):
        """Initialize the robot and set its initial location and orientation"""
        super().__init__()

        # Get the properties of the simulation
        self.screen = simulation.screen
        self.settings = simulation.settings
        self.screen_rect = simulation.screen.get_rect()

        # Load the robot and get its rect
        self.original_image = pygame.image.load("images/Robot_000.bmp")
        self.image = self.original_image
        self.rect = self.image.get_rect()

        # Robot size
        self.size = int(np.sqrt(self.rect.width**2 + self.rect.height**2))

        # Initialize precise global position and local orientation
        self.init_x = init_x
        self.init_y = init_y
        self.init_o = random() * 2.0 * np.pi

        # Actual global position and local orientation
        self.x = copy.deepcopy(self.init_x)
        self.y = copy.deepcopy(self.init_y)
        self.o = copy.deepcopy(self.init_o)
        self._rotate_image()

        # Odometry (local position and orientation)
        self.odo_x = 0.0
        self.odo_y = 0.0
        self.odo_o = 0.0
        self.odo_unit_x = np.cos(self.odo_o)
        self.odo_unit_y = np.sin(self.odo_o)

        # Target point
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_reached = True

        # Center rect to the random position
        self.rect.center = (self.x, self.y)

        # Movement flag
        self.moving_forward = False
        self.moving_backward = False

        # Sensing flag
        self.sensing = True
        self.sensing_counter = 0
        self.set_sensing_counter()

    def set_sensing_counter(self):
        """Sets the sensing counter of the robot"""
        self.sensing_counter = self.settings.lidar_sensing_counter

    def rotate_by_increment(self, increment):
        """Rotate the robot around its center"""
        self.o = (self.o + increment) % (2 * np.pi)

        if self.o < 0:
            self.o += 2 * np.pi

        self._rotate_image()

    def _rotate_image(self):
        """Rotating the robot image to fit to the orientation"""
        degree = self.settings.robot_rotation_increment * round(self.o / self.settings.robot_rotation_increment)

        degree = int(np.degrees(degree))

        if 0 <= degree <= 90:
            self.image = pygame.image.load("images/Robot_{:0>3d}.bmp".format(degree))
        elif 90 < degree <= 180:
            self.image = pygame.image.load("images/Robot_{:0>3d}.bmp".format(degree - ((degree - 90) * 2)))
            self.image = pygame.transform.flip(self.image, False, True)
        elif 180 < degree <= 270:
            self.image = pygame.image.load("images/Robot_{:0>3d}.bmp".format(degree - 180))
            self.image = pygame.transform.flip(self.image, True, True)
        elif 270 < degree <= 360:
            self.image = pygame.image.load("images/Robot_{:0>3d}.bmp".format(degree - 180 - ((degree - 270) * 2)))
            self.image = pygame.transform.flip(self.image, True, False)

        self.rect = self.image.get_rect()
        self.rect.center = (self.x, self.y)

    def _calc_odometry(self):
        """Calculates the odometry"""
        # Actual local orientation
        self.odo_o = self.o - self.init_o

        # Calculate the unit vector
        self.odo_unit_x = np.cos(self.odo_o)
        self.odo_unit_y = -1 * np.sin(self.odo_o)

        # Global movement vector
        x = self.x - self.init_x
        y = self.y - self.init_y

        # Actual local movement vector
        theta = self.init_o + (np.pi / 2)
        self.odo_x = x * np.cos(theta) - y * np.sin(theta)
        self.odo_y = x * np.sin(theta) + y * np.cos(theta)

    def update(self):
        """Updates the robot's position and odometry"""
        self.rect.center = (self.x, self.y)
        self._calc_odometry()

    def blitme(self):
        """Draw the robot onto the screen"""
        self.screen.blit(self.image, self.rect)
