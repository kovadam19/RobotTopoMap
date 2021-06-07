import random

class Obstacle:
    """A class for rectangle obstacles on the robot's map"""
    def __init__(self, x, y, sizeX, sizeY):
        """Initializes the obstacle"""
        # Position on the map
        self.x = x
        self.y = y

        # Size of the rectangle
        self.sizeX = sizeX
        self.sizeY = sizeY

        # Color of the obstacle
        red = int(random.random() * 255)
        green = int(random.random() * 255)
        blue = int(random.random() * 255)
        self.color = (red, green, blue)