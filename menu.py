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


class Menu:
    """A class to show the menu"""

    def __init__(self, simulation):
        """Initializing the menu"""
        self.simulation = simulation
        self.screen = simulation.screen
        self.screen_rect = self.screen.get_rect()
        self.settings = simulation.settings
        self.map = simulation.map
        self.robot = simulation.robot

        # Font settings
        self.text_color_basic = self.settings.menu_text_color_basic
        self.text_color_selected = self.settings.menu_text_color_selected
        self.font = pygame.font.SysFont(None, self.settings.menu_font_size)
        self.line_gap = self.settings.menu_line_gap

        # Background
        self.bg_color = self.settings.menu_bg_color

        # Prepare the initial menu
        self.prep_menu_line()
        self.prep_layout_design_menu()
        self.prep_manual_control()
        self.prep_autonomous_exp()
        self.prep_autonomous_navigation()
        self.prep_visualization()

    def prep_menu_line(self):
        """Prepares the menu line"""
        menu_str = "MENU & CONTROLS (F1)"
        # Prepare the image and positions it on the screen
        self.menu_image = self.font.render(menu_str, True, self.text_color_basic, self.bg_color)
        self.menu_rect = self.menu_image.get_rect()
        self.menu_rect.left = self.settings.menu_position_x
        self.menu_rect.top = self.settings.menu_position_y

    def prep_layout_design_menu(self):
        """Prepares the menu for the layout design"""
        layout_str = "Layout design (L)"
        if self.settings.layout_design:
            layout_color = self.text_color_selected
        else:
            layout_color = self.text_color_basic
        # Prepare the image and positions it on the screen
        self.layout_image = self.font.render(layout_str, True, layout_color, self.bg_color)
        self.layout_rect = self.layout_image.get_rect()
        self.layout_rect.left = self.menu_rect.left
        self.layout_rect.top = self.menu_rect.bottom + self.line_gap

        if self.settings.layout_grids:
            grid_str = "Grids (G): ON"
        else:
            grid_str = "Grids (G): OFF"
        # Prepare the image and positions it on the screen
        self.grid_image = self.font.render(grid_str, True, self.text_color_basic, self.bg_color)
        self.grid_rect = self.grid_image.get_rect()
        self.grid_rect.left = self.layout_rect.left + 3 * self.line_gap
        self.grid_rect.top = self.layout_rect.bottom + self.line_gap

        load_str = "Loading from file (F)"
        # Prepare the image and positions it on the screen
        self.load_image = self.font.render(load_str, True, self.text_color_basic, self.bg_color)
        self.load_rect = self.load_image.get_rect()
        self.load_rect.left = self.grid_rect.left
        self.load_rect.top = self.grid_rect.bottom + self.line_gap

        mouse_str = "Mouse buttons: left (object), middle (target), right (robot)"
        # Prepare the image and positions it on the screen
        self.mouse_image = self.font.render(mouse_str, True, self.text_color_basic, self.bg_color)
        self.mouse_rect = self.mouse_image.get_rect()
        self.mouse_rect.left = self.load_rect.left
        self.mouse_rect.top = self.load_rect.bottom + self.line_gap

    def prep_manual_control(self):
        """Prepares menu for manual control"""
        manual_str = "Manual control (M)"
        if self.settings.manual_control:
            manual_color = self.text_color_selected
        else:
            manual_color = self.text_color_basic
        # Prepare the image and positions it on the screen
        self.manual_image = self.font.render(manual_str, True, manual_color, self.bg_color)
        self.manual_rect = self.manual_image.get_rect()
        self.manual_rect.left = self.mouse_rect.left - 3 * self.line_gap
        self.manual_rect.top = self.mouse_rect.bottom + self.line_gap

        movement_str = "Forward (UP), Backward (DOWN)"
        # Prepare the image and positions it on the screen
        self.movement_image = self.font.render(movement_str, True, self.text_color_basic, self.bg_color)
        self.movement_rect = self.movement_image.get_rect()
        self.movement_rect.left = self.manual_rect.left + 3 * self.line_gap
        self.movement_rect.top = self.manual_rect.bottom + self.line_gap

        turn_str = "Turn left (LEFT), Turn right (RIGHT)"
        # Prepare the image and positions it on the screen
        self.turn_image = self.font.render(turn_str, True, self.text_color_basic, self.bg_color)
        self.turn_rect = self.turn_image.get_rect()
        self.turn_rect.left = self.movement_rect.left
        self.turn_rect.top = self.movement_rect.bottom + self.line_gap

        laser_str = "Emit laser beams (SPACE)"
        # Prepare the image and positions it on the screen
        self.laser_image = self.font.render(laser_str, True, self.text_color_basic, self.bg_color)
        self.laser_rect = self.laser_image.get_rect()
        self.laser_rect.left = self.turn_rect.left
        self.laser_rect.top = self.turn_rect.bottom + self.line_gap

        cluster_str = "Grow cluster (C)"
        # Prepare the image and positions it on the screen
        self.cluster_image = self.font.render(cluster_str, True, self.text_color_basic, self.bg_color)
        self.cluster_rect = self.cluster_image.get_rect()
        self.cluster_rect.left = self.laser_rect.left
        self.cluster_rect.top = self.laser_rect.bottom + self.line_gap

        obstacle_str = "Detect obstacles (O)"
        # Prepare the image and positions it on the screen
        self.obstacle_image = self.font.render(obstacle_str, True, self.text_color_basic, self.bg_color)
        self.obstacle_rect = self.obstacle_image.get_rect()
        self.obstacle_rect.left = self.cluster_rect.left
        self.obstacle_rect.top = self.cluster_rect.bottom + self.line_gap

    def prep_autonomous_exp(self):
        """Prepares the menu for autonomous exploration"""
        a_exp_str = "Autonomous Exploration (E)"
        if self.settings.autonomous_exploration:
            a_exp_color = self.text_color_selected
        else:
            a_exp_color = self.text_color_basic
        # Prepare the image and positions it on the screen
        self.exp_image = self.font.render(a_exp_str, True, a_exp_color, self.bg_color)
        self.exp_rect = self.exp_image.get_rect()
        self.exp_rect.left = self.obstacle_rect.left - 3 * self.line_gap
        self.exp_rect.top = self.obstacle_rect.bottom + self.line_gap

    def prep_autonomous_navigation(self):
        """Prepares the menu for autonomous navigation"""
        a_nav_str = "Autonomous Navigation (A)"
        if self.settings.autonomous_navigation:
            a_nav_color = self.text_color_selected
        else:
            a_nav_color = self.text_color_basic
        # Prepare the image and positions it on the screen
        self.nav_image = self.font.render(a_nav_str, True, a_nav_color, self.bg_color)
        self.nav_rect = self.nav_image.get_rect()
        self.nav_rect.left = self.exp_rect.left
        self.nav_rect.top = self.exp_rect.bottom + self.line_gap

    def prep_visualization(self):
        """Prepares the menu for the visualization options"""
        vis_str = "Visualization Options"
        # Prepare the image and positions it on the screen
        self.vis_image = self.font.render(vis_str, True, self.text_color_basic, self.bg_color)
        self.vis_rect = self.nav_image.get_rect()
        self.vis_rect.left = self.nav_rect.left
        self.vis_rect.top = self.nav_rect.bottom + self.line_gap

        if self.settings.map_draw_lidar_points:
            lidar_str = "Lidar Points (F2): ON"
        else:
            lidar_str = "Lidar Points (F2): OFF"
        # Prepare the image and positions it on the screen
        self.lidar_image = self.font.render(lidar_str, True, self.text_color_basic, self.bg_color)
        self.lidar_rect = self.lidar_image.get_rect()
        self.lidar_rect.left = self.vis_rect.left + 3 * self.line_gap
        self.lidar_rect.top = self.vis_rect.bottom + self.line_gap

        if self.settings.map_draw_obstacles:
            obs_str = "Obstacles (F3): ON"
        else:
            obs_str = "Obstacles (F3): OFF"
        # Prepare the image and positions it on the screen
        self.obs_image = self.font.render(obs_str, True, self.text_color_basic, self.bg_color)
        self.obs_rect = self.obs_image.get_rect()
        self.obs_rect.left = self.lidar_rect.left
        self.obs_rect.top = self.lidar_rect.bottom + self.line_gap

        if self.settings.map_draw_all_clusters:
            all_cluster_str = "All Cluster (F4): ON"
        else:
            all_cluster_str = "All Cluster (F4): OFF"
        # Prepare the image and positions it on the screen
        self.all_cluster_image = self.font.render(all_cluster_str, True, self.text_color_basic, self.bg_color)
        self.all_cluster_rect = self.all_cluster_image.get_rect()
        self.all_cluster_rect.left = self.obs_rect.left
        self.all_cluster_rect.top = self.obs_rect.bottom + self.line_gap

        if self.settings.map_draw_last_cluster:
            last_cluster_str = "Last Cluster (F5): ON"
        else:
            last_cluster_str = "Last Cluster (F5): OFF"
        # Prepare the image and positions it on the screen
        self.last_cluster_image = self.font.render(last_cluster_str, True, self.text_color_basic, self.bg_color)
        self.last_cluster_rect = self.last_cluster_image.get_rect()
        self.last_cluster_rect.left = self.all_cluster_rect.left
        self.last_cluster_rect.top = self.all_cluster_rect.bottom + self.line_gap

        if self.settings.map_draw_exploration_points:
            exp_point_str = "Exploration Points (F6): ON"
        else:
            exp_point_str = "Exploration Points (F6): OFF"
        # Prepare the image and positions it on the screen
        self.exp_point_image = self.font.render(exp_point_str, True, self.text_color_basic, self.bg_color)
        self.exp_point_rect = self.exp_point_image.get_rect()
        self.exp_point_rect.left = self.last_cluster_rect.left
        self.exp_point_rect.top = self.last_cluster_rect.bottom + self.line_gap

    def show_menu(self):
        """Shows the menu"""
        self.screen.blit(self.menu_image, self.menu_rect)
        if self.settings.menu_show:
            self.screen.blit(self.layout_image, self.layout_rect)
            self.screen.blit(self.grid_image, self.grid_rect)
            self.screen.blit(self.load_image, self.load_rect)
            self.screen.blit(self.mouse_image, self.mouse_rect)
            self.screen.blit(self.manual_image, self.manual_rect)
            self.screen.blit(self.movement_image, self.movement_rect)
            self.screen.blit(self.turn_image, self.turn_rect)
            self.screen.blit(self.laser_image, self.laser_rect)
            self.screen.blit(self.cluster_image, self.cluster_rect)
            self.screen.blit(self.obstacle_image, self.obstacle_rect)
            self.screen.blit(self.exp_image, self.exp_rect)
            self.screen.blit(self.nav_image, self.nav_rect)
            self.screen.blit(self.vis_image, self.vis_rect)
            self.screen.blit(self.lidar_image, self.lidar_rect)
            self.screen.blit(self.obs_image, self.obs_rect)
            self.screen.blit(self.all_cluster_image, self.all_cluster_rect)
            self.screen.blit(self.last_cluster_image, self.last_cluster_rect)
            self.screen.blit(self.exp_point_image, self.exp_point_rect)
