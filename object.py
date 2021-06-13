##################################################
# Project: Robot Topological Mapping
# Author: Adam Kovacs
# Version: 1.0.0
# Maintainer: Adam Kovacs
# E-mail: kovadam19@gmail.com
# Released: 13 June 2021
##################################################

# Generic/Built-in imports
import pygame
from pygame.sprite import Sprite


class Object(Sprite):
    """Claas to represent an object"""

    def __init__(self, simulation, init_x, init_y):
        """Initialize the object"""
        super().__init__()

        # Get the properties of the simulation
        self.screen = simulation.screen
        self.settings = simulation.settings
        self.screen_rect = simulation.screen.get_rect()

        # Load the object and get its rect
        self.image = pygame.image.load("images/Object.bmp")
        self.rect = self.image.get_rect()

        # Center rect to the random position
        self.rect.center = (init_x, init_y)

    def blitme(self):
        """Draw the object onto the screen"""
        self.screen.blit(self.image, self.rect)
