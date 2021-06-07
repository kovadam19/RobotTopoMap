import pygame

from pygame.sprite import Sprite


class GridCell(Sprite):
    """A class to manage grid cells on the robot's map"""

    def __init__(self, x, y, color):
        """Create a grid cell on to the robot's map"""
        # Initialize the sprite
        super().__init__()

        # Real position
        self.x = x
        self.y = y

        # Color
        self.color = color

        # Create a rect for the grid cell
        self.rect = pygame.Rect(x, y, 8, 8)
