import numpy as np


class Settings:
    """Class to manage the settings"""

    def __init__(self):
        """Initialize the settings"""

        # Screen settings
        self.screen_width = 1600
        self.screen_height = 800
        self.bg_color = (255, 255, 255)

        # Layout settings
        self.layout_width = self.screen_width / 2
        self.layout_height = self.screen_height
        self.layout_object_size = 38
        self.layout_robot_size = 76
        self.layout_grids = False
        self.layout = {"Objects": set(),# (300, 300), (500, 300), (500, 500), (300, 500)
                       "Robots": set()}

        # Robot settings
        self.robot_rotation_increment = 0.0872665  # radians
        self.robot_speed = 0.1
        self.robot_target_distance_tolerance = 0.1
        self.robot_orientation_tolerance = 0.0001

        # Laser settings
        self.laser_color = (255, 0, 0)
        self.laser_width = 4
        self.laser_height = 4
        self.laser_speed = 1.0
        self.laser_number = 100

        # LIDAR settings
        self.lidar_position = 33
        self.lidar_view_angle = 2 * (np.pi / 3)
        self.lidar_sensing_distance = 200
        self.lidar_sensing_counter = 6

        # Map settings
        self.map_color = (0, 0, 0)
        self.map_lidar_color = (255, 255, 255)
        self.map_cell_size = 20
        self.map_font_size = 16
        self.map_text_color = (0, 255, 0)
        self.map_target_size = 8
        self.map_coverage_threshold = 0.9
        self.map_draw_lidar_points = False
        self.map_draw_all_clusters = False
        self.map_draw_trajectory_points = True

        # Display settings
        self.display_bg_color = (0, 0, 0)
        self.display_font_size = 16
        self.display_text_color = (0, 255, 0)
        self.display_position_x = 805
        self.display_position_y = 5
        self.display_line_gap = 4

        # Menu settings
        self.menu_show = True
        self.menu_bg_color = (255, 255, 255)
        self.menu_font_size = 16
        self.menu_text_color_basic = (0, 0, 0)
        self.menu_text_color_selected = (255, 0, 0)
        self.menu_position_x = 5
        self.menu_position_y = 5
        self.menu_line_gap = 4

        # Control settings
        self.layout_design = False
        self.manual_control = False
        self.autonomous_exploration = False
        self.autonomous_navigation = False

        # Autonomous exploration settings
        self.ae_exploration_steps = 10
        self.ae_safety_zone_multiplier = 0.6  # Size of the robot is multiplied by this
        self.ae_counter_limit = 100  # Limit of tries with one setting of requirements (angle, distance)
        self.ae_initial_angle = 0.349066  # 20 deg (0.349066 rad)
        self.ae_angle_increment = 0.0872665  # 5 deg (0.0872665 rad)
        self.ae_min_distance = 40  # Minimum distance that the robot has to move
        self.ae_distance_increment = 1
