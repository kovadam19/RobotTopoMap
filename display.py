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


class Display:
    """A class to report information"""

    def __init__(self, simulation):
        """Initializing the display"""
        self.simulation = simulation
        self.screen = simulation.screen
        self.screen_rect = self.screen.get_rect()
        self.settings = simulation.settings
        self.map = simulation.map
        self.robot = simulation.robot

        # Font settings
        self.text_color = self.settings.display_text_color
        self.font = pygame.font.SysFont(None, self.settings.display_font_size)
        self.line_gap = self.settings.display_line_gap

        # Background
        self.bg_color = self.settings.display_bg_color

        # Prepare the initial display
        self.prep_robot_state()
        self.prep_robot_action()
        self.prep_robot_location()
        self.prep_robot_target()

    def prep_robot_state(self):
        """Prepares the state of the robot"""
        if self.settings.manual_control:
            state_str = "State: Manual Control"
        elif self.settings.autonomous_exploration:
            state_str = f"State: Autonomous Exploration ({self.simulation.exploration_steps})"
        elif self.settings.autonomous_navigation:
            state_str = "State: Autonomous Navigation"
        elif self.settings.layout_design:
            state_str = "State: Layout design"
        else:
            state_str = "State: Waiting for command"
        # Prepare the image and positions it on the screen
        self.state_image = self.font.render(state_str, True, self.text_color, self.bg_color)
        self.state_rect = self.state_image.get_rect()
        self.state_rect.left = self.settings.display_position_x
        self.state_rect.top = self.settings.display_position_y

    def prep_robot_action(self):
        """Prepares the action of the robot"""
        if self.robot.sensing:
            action_str = "Action: Sensing..."
        elif self.robot.moving_forward or self.robot.moving_backward:
            action_str = "Action: Moving..."
        else:
            action_str = "Action: Target reached..."
        # Prepare the image and positions it on the screen
        self.action_image = self.font.render(action_str, True, self.text_color, self.bg_color)
        self.action_rect = self.action_image.get_rect()
        self.action_rect.left = self.state_rect.left
        self.action_rect.top = self.state_rect.bottom + self.line_gap

    def prep_robot_location(self):
        """Prepares the robot's location and orientation"""
        x = int(self.robot.odo_x)
        y = int(self.robot.odo_y)
        o = round(self.robot.odo_o, 2)
        location_str = f"Location (X,Y,O): {str(x)}, {str(y)}, {str(o)}"
        # Prepare the image and positions it on the screen
        self.location_image = self.font.render(location_str, True, self.text_color, self.bg_color)
        self.location_rect = self.location_image.get_rect()
        self.location_rect.left = self.action_rect.left
        self.location_rect.top = self.action_rect.bottom + self.line_gap

    def prep_robot_target(self):
        """Prepares the robot's target"""
        x = int(self.robot.target_x)
        y = int(self.robot.target_y)
        target_str = f"Target (X,Y): {str(x)}, {str(y)}"
        # Prepare the image and positions it on the screen
        self.target_image = self.font.render(target_str, True, self.text_color, self.bg_color)
        self.target_rect = self.target_image.get_rect()
        self.target_rect.left = self.location_rect.left
        self.target_rect.top = self.location_rect.bottom + self.line_gap

    def show_display(self):
        """Shows the display"""
        self.screen.blit(self.state_image, self.state_rect)
        self.screen.blit(self.action_image, self.action_rect)
        self.screen.blit(self.location_image, self.location_rect)
        self.screen.blit(self.target_image, self.target_rect)
