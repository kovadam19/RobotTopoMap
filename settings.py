import numpy as np


class Settings:
    """Class to manage the settings"""

    def __init__(self):
        """Initialize the settings"""

        # Screen settings
        self.screen_width = 1600  # Do not modify!
        self.screen_height = 800  # Do not modify!
        self.bg_color = (255, 255, 255)

        # Layout settings
        self.layout_object_filename = "Layout_Objects.txt"
        self.layout_robot_filename = "Layout_Robots.txt"
        self.layout_width = self.screen_width / 2   # Do not modify!
        self.layout_height = self.screen_height   # Do not modify!
        self.layout_object_size = 38  # Do not modify!
        self.layout_robot_size = 76  # Do not modify!
        self.layout_grids = False   # Do not modify!
        self.layout = {"Objects": set(),
                       "Robots": set(),
                       "Targets": []}

        # Robot settings
        self.robot_rotation_increment = 0.0872665  # Radians, do not change!
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
        self.lidar_position = 33  # Do not modify!
        self.lidar_view_angle = 2 * (np.pi / 3)
        self.lidar_sensing_distance = 200
        self.lidar_sensing_counter = 6

        # Map settings
        self.map_cell_size = 20
        self.map_cluster_aspect_ratio = 1.0
        self.map_font_size = 16
        self.map_target_size = 8
        self.map_coverage_threshold = 0.95
        self.map_navigation_rect_size = 12
        self.map_color = (0, 0, 0)
        self.map_lidar_color = (255, 255, 255)
        self.map_robot_position_color = (0, 255, 0)
        self.map_old_exp_point_color = (0, 0, 255)
        self.map_new_exp_point_color = (255, 255, 0)
        self.map_navigation_node_color = (255, 0, 0)
        self.map_navigation_path_color = (255, 255, 255)
        self.map_navigation_target_color = (0, 255, 0)
        self.map_text_color = (0, 255, 0)
        self.map_x_axis_color = (255, 0, 0)
        self.map_y_axis_color = (0, 255, 0)
        self.map_draw_lidar_points = False  # Do not modify!
        self.map_draw_obstacles = True  # Do not modify!
        self.map_draw_all_clusters = False  # Do not modify!
        self.map_draw_last_cluster = True  # Do not modify!
        self.map_draw_exploration_points = True  # Do not modify!

        # Display settings
        self.display_bg_color = (0, 0, 0)
        self.display_font_size = 16
        self.display_text_color = (0, 255, 0)
        self.display_position_x = 805
        self.display_position_y = 5
        self.display_line_gap = 4

        # Menu settings
        self.menu_show = True  # Do not modify!
        self.menu_bg_color = (255, 255, 255)
        self.menu_font_size = 16
        self.menu_text_color_basic = (0, 0, 0)
        self.menu_text_color_selected = (255, 0, 0)
        self.menu_position_x = 5
        self.menu_position_y = 5
        self.menu_line_gap = 4

        # Control settings
        self.layout_design = False  # Do not modify!
        self.manual_control = False  # Do not modify!
        self.autonomous_exploration = False  # Do not modify!
        self.autonomous_navigation = False  # Do not modify!

        # Autonomous exploration settings
        self.ae_exploration_steps = 35
        self.ae_safety_zone_multiplier = 0.6  # Size of the robot is multiplied by this
        self.ae_counter_limit = 100  # Limit of tries with one setting of requirements (angle, distance)
        self.ae_initial_angle = 0.349066  # 20 deg (0.349066 rad)
        self.ae_angle_increment = 0.0872665  # 5 deg (0.0872665 rad)
        self.ae_min_distance = 40  # Minimum distance that the robot has to move
        self.ae_distance_increment = 1

        # Save settings
        self.save_folder = "/Test_1/"
        self.save_image_name = "image"
        self.save_file_extension = ".jpg"
        self.save_interval = 100
