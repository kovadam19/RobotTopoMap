import sys
import pygame
from time import sleep
import numpy as np
import random

from settings import Settings
from robot import Robot
from laser_beam import Beam
from object import Object
from map import Map
from display import Display
from menu import Menu


class Simulation:
    """Overall class to manage the robot localization simulation"""

    def __init__(self):
        """Initialize the simulation"""
        # Initialize the game module
        pygame.init()

        # Get settings
        self.settings = Settings()

        # Display and caption
        self.screen = pygame.display.set_mode((self.settings.screen_width, self.settings.screen_height))
        pygame.display.set_caption("Robot TopoMap Simulation by Adam")

        # Autonomous exploration
        self.exploration_steps = self.settings.ae_exploration_steps

        # Initialize simulation components and layout
        self._initialize_components()

    def _initialize_components(self):
        """Initializes simulation components and layout"""
        # Initialize the robot
        self.robots = pygame.sprite.Group()
        if self.settings.layout["Robots"]:
            for coords in self.settings.layout["Robots"]:
                self.robot = Robot(self, *coords)
                self.robots.add(self.robot)
        else:
            self.robot = Robot(self, 400, 400)
            self.robots.add(self.robot)

        # Initialize the objects
        self.objects = pygame.sprite.Group()
        for coords in self.settings.layout["Objects"]:
            new_object = Object(self, *coords)
            self.objects.add(new_object)

        # Initialize the laser beams
        self.laser_beams = pygame.sprite.Group()

        # Initialize the map
        self.map = Map(self)

        # Initialize the display
        self.display = Display(self)

        # Initialize the menu
        self.menu = Menu(self)

    def run_simulation(self):
        """Run the main loop for the simulation"""
        while True:
            # Check events
            self._check_events()

            # Update robot
            self._update_robot()

            # Check the position of the laser beams
            self._check_laser_beam_distance()

            # Check laser beam collisions
            self._check_laser_beam_collision()

            # Update laser beams
            self._update_laser_beams()

            # Update map
            self._update_map()

            # Prepare the display
            self.display.prep_robot_state()
            self.display.prep_robot_action()
            self.display.prep_robot_location()
            self.display.prep_robot_target()

            # Prepare the menu
            if self.settings.menu_show:
                self.menu.prep_layout_design_menu()
                self.menu.prep_manual_control()
                self.menu.prep_autonomous_exp()


            # Update the screen
            self._update_screen()

    def _check_events(self):
        """Respond to key and mouse events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_F1:
                    self.settings.menu_show = not self.settings.menu_show
                elif event.key == pygame.K_LEFT and self.settings.manual_control:
                    self.robot.rotate_by_increment(0.174533)
                elif event.key == pygame.K_RIGHT and self.settings.manual_control:
                    self.robot.rotate_by_increment(-0.174533)
                elif event.key == pygame.K_UP and self.settings.manual_control:
                    self.robot.moving_forward = True
                elif event.key == pygame.K_DOWN and self.settings.manual_control:
                    self.robot.moving_backward = True
                elif event.key == pygame.K_SPACE and self.settings.manual_control:
                    self._emit_laser_beams()
                elif event.key == pygame.K_c and self.settings.manual_control:
                    self.map.flag_cluster_growing = True
                    self.map.growing_cluster((self.robot.odo_x, self.robot.odo_y))
                elif event.key == pygame.K_o and self.settings.manual_control:
                    self.map.flag_obstacle_detection = True
                    self.map.detecting_obstacles()
                elif event.key == pygame.K_m:
                    self.settings.layout_design = False
                    self.settings.autonomous_exploration = False
                    self.settings.autonomous_navigation = False
                    self.settings.manual_control = True
                elif event.key == pygame.K_e:
                    self.settings.layout_design = False
                    self.settings.autonomous_navigation = False
                    self.settings.manual_control = False
                    self.settings.autonomous_exploration = True
                elif event.key == pygame.K_l:
                    self.settings.autonomous_navigation = False
                    self.settings.manual_control = False
                    self.settings.autonomous_exploration = False
                    self.settings.layout_design = True
                elif event.key == pygame.K_g and self.settings.layout_design:
                    self.settings.layout_grids = not self.settings.layout_grids
                elif event.key == pygame.K_f and self.settings.layout_design:
                    self._load_layout_from_file()
            elif event.type == pygame.KEYUP and self.settings.manual_control:
                if event.key == pygame.K_UP:
                    self.robot.moving_forward = False
                if event.key == pygame.K_DOWN:
                    self.robot.moving_backward = False
            elif event.type == pygame.MOUSEBUTTONDOWN and self.settings.layout_design:
                # Left mouse click
                if event.button == 1:
                    x, y = pygame.mouse.get_pos()
                    if self.settings.layout_grids:
                        x = (self.settings.layout_object_size / 2) + (int(x / self.settings.layout_object_size) * self.settings.layout_object_size)
                        y = (self.settings.layout_object_size / 2) + (int(y / self.settings.layout_object_size) * self.settings.layout_object_size)
                    x_in_range = 0 + (self.settings.layout_object_size / 2) <= x <= self.settings.layout_width - (self.settings.layout_object_size / 2)
                    y_in_range = 0 + (self.settings.layout_object_size / 2) <= y <= self.settings.layout_height - (self.settings.layout_object_size / 2)
                    if x_in_range and y_in_range:
                        self.settings.layout["Objects"].add((x, y))
                        self.objects = pygame.sprite.Group()
                        for coords in self.settings.layout["Objects"]:
                            new_object = Object(self, *coords)
                            self.objects.add(new_object)
                # Right mouse click
                if event.button == 3:
                    x, y = pygame.mouse.get_pos()
                    x_in_range = 0 + self.settings.layout_robot_size <= x <= self.settings.layout_width - self.settings.layout_robot_size
                    y_in_range = 0 + self.settings.layout_robot_size <= y <= self.settings.layout_height - self.settings.layout_robot_size
                    if x_in_range and y_in_range:
                        self.settings.layout["Robots"].clear()
                        self.settings.layout["Robots"].add((x, y))
                        self.robots = pygame.sprite.Group()
                        for coords in self.settings.layout["Robots"]:
                            self.robot = Robot(self, *coords)
                            self.robots.add(self.robot)

    def _load_layout_from_file(self):
        """Loads the layout from file"""
        # Loading the objects
        with open("Layout_Objects.txt", "r") as file:
            lines = file.readlines()
            for line in lines:
                x, y = line.split()
                x = int(x)
                y = int(y)
                self.settings.layout["Objects"].add((x, y))
                self.objects = pygame.sprite.Group()
                for coords in self.settings.layout["Objects"]:
                    new_object = Object(self, *coords)
                    self.objects.add(new_object)

        # Loading the robots
        with open("Layout_Robots.txt", "r") as file:
            lines = file.readlines()
            for line in lines:
                x, y = line.split()
                x = int(x)
                y = int(y)
                self.settings.layout["Robots"].clear()
                self.settings.layout["Robots"].add((x, y))
                self.robots = pygame.sprite.Group()
                for coords in self.settings.layout["Robots"]:
                    self.robot = Robot(self, *coords)
                    self.robots.add(self.robot)

    def _update_robot(self):
        """Updates the robot"""
        # Check if autonomous exploration is active
        if self.settings.autonomous_exploration and self.exploration_steps >= 1:
            # Check if robot reached the target, it is in sensing action and there are no active laser beams
            if self.robot.target_reached and self.robot.sensing and not self.laser_beams:
                # Sense the environment
                self._autonomous_sensing()
            # Check if the robot is not sensing, there are no active laser beams and robot reached the target
            if self.robot.target_reached and self.map.flag_obstacle_detection and self.map.flag_cluster_growing:
                # Decrease the number of exploration steps
                self.exploration_steps -= 1
                # Check if this was not the last step
                if self.exploration_steps != 0:
                    # Detect obstacle on the map
                    if self.map.detecting_obstacles():
                        # Grow a cluster at the location of the robot
                        if self.map.growing_cluster((self.robot.target_x, self.robot.target_y),
                                                    aspect_ratio_multiplier=1.0,
                                                    max_distance=self.settings.lidar_sensing_distance):
                            # Calculate a new exploration target
                            self._calculate_new_exploration_target()
                            # Turn off the map obstacle detection and cluster growing
                            self.map.flag_obstacle_detection = False
                            self.map.flag_cluster_growing = False
                else:
                    # Detect obstacle on the map
                    self.map.detecting_obstacles()
                    # Grow the final cluster
                    self.map.growing_cluster((self.robot.target_x, self.robot.target_y),
                                             aspect_ratio_multiplier=1.0,
                                             max_distance=self.settings.lidar_sensing_distance)
                    # Merging intersecting clusters
                    self.map.merging_clusters()
                    # Removing highly covered clusters
                    self.map.remove_covered_clusters()
                    # Finding the neighbours for each cluster
                    self.map.find_cluster_neighbours()
                    # Create navigation nodes
                    self.map.create_navigation_nodes()
                    # Turn on cluster drawing all the clusters
                    self.settings.map_draw_all_clusters = True
                    # Turn off the autonomous exploration
                    self.settings.autonomous_exploration = False
                    # Turn on the autonomous navigation
                    self.settings.autonomous_navigation = True
        elif self.settings.autonomous_navigation and self.robot.target_reached:
            if self.settings.layout["Targets"]:
                self.map.path_planner((self.robot.odo_x, self.robot.odo_y), self.settings.layout["Targets"].pop())


        # Check if the robot is not sensing and the target is not reached
        if not self.robot.target_reached and not self.robot.sensing:
            # Calculate the autonomous movement for the robot
            self._calculate_autonomous_movement()

        # Calculate the robot's position and update the sprite
        self._calculate_robot_position()
        self.robots.update()

    def _autonomous_sensing(self):
        """Controls the robot's LIDAR sensor in autonomous mode"""
        # Check if the sensing counter is larger than zero
        if self.robot.sensing_counter > 0:
            # Rotate the robot to the sensing position
            self.robot.rotate_by_increment((2 * np.pi) / self.settings.lidar_sensing_counter)
            # Emit the laser beams and decrease the sensing counter
            self._emit_laser_beams()
            self.robot.sensing_counter -= 1
        else:
            # The robot is done with sensing
            self.robot.sensing = False
            # Resetting the sensing counter
            self.robot.set_sensing_counter()
            # Indicate obstacle detection and cluster growing
            self.map.flag_obstacle_detection = True
            self.map.flag_cluster_growing = True

    def _calculate_new_exploration_target(self):
        """Calculates new target for the robot"""
        # Get the last cluster from the clusters
        last_cluster = self.map.clusters[-1]
        # Calculate the size of the safety zone (region from the outermost points in the cluster)
        safety_zone = int((self.robot.size * self.settings.ae_safety_zone_multiplier) / self.settings.map_cell_size) * self.settings.map_cell_size
        # Get the perimeter points (possible target points for the robot)
        possible_targets = last_cluster.get_perimeter_points(outermost=False, safety_zone=safety_zone)
        # If there is any possible target point
        if possible_targets:
            # Set the initial variables
            counter = 0
            angle_limit = self.settings.ae_initial_angle # Angle between the robot current orientation vector and the target vector
            min_distance = self.settings.ae_min_distance  # Minimum distance that the robot has to move
            # Select points until they fulfill the requirements of a target point
            while True:
                # Increase the counter
                counter += 1
                # Check if the counter reached a limit of tries with one setting of requirements (angle, distance)
                if counter == self.settings.ae_counter_limit:
                    # Zero the counter
                    counter = 0
                    # Check if the angle is smaller than PI
                    if angle_limit < np.pi:
                        # Increase the angle by increment
                        angle_limit += self.settings.ae_angle_increment
                    # Check if the length is larger than zero
                    if min_distance > 0:
                        # Decrease the length by increment
                        min_distance -= self.settings.ae_distance_increment
                # Choose a random point from the possible targets
                target = random.choice(list(possible_targets))
                # Calculate the local vector from the robot to the chosen target point
                vec_x = target[0] - self.robot.odo_x
                vec_y = target[1] - self.robot.odo_y
                # Calculate the distance between the robot and the chosen target point
                length = np.sqrt(vec_x**2 + vec_y**2)
                # Check if the length of the vector is larger then zero
                if length > 0.001:
                    # Calculate the unit vector
                    unit_vec_x = vec_x / length
                    unit_vec_y = vec_y / length
                    # Calculate the angle between the robot's orientation and the target vector
                    angle = np.arccos(self.robot.odo_unit_x * unit_vec_x + self.robot.odo_unit_y * unit_vec_y)
                    # Check if the angle is lower than the current limit and the length is larger than the current min
                    if angle <= angle_limit and length >= min_distance:
                        break
            # At this point we have the target point, so we have to send it to the robot and the map
            self.robot.target_x = target[0]
            self.robot.target_y = target[1]
            self.robot.target_reached = False
            self.map.trajectory_points.append((target[0], target[1]))

    def _calculate_autonomous_movement(self):
        """
        Calculates the autonomous movement of the robot
        Returns True if the robot is moving
        Returns False if the robot is not moving
        """
        # Get the robot unit orientation, robot position, target position
        robot_unit = np.array([self.robot.odo_unit_x, self.robot.odo_unit_y])
        robot_pos = np.array([self.robot.odo_x, self.robot.odo_y])
        target_pos = np.array([self.robot.target_x, self.robot.target_y])

        # Calculate the distance vector and distance
        distance_vector = target_pos - robot_pos
        distance = np.linalg.norm(distance_vector)

        # Check if the distance is grater than the tolerance
        if distance > self.settings.robot_target_distance_tolerance:
            # Calculate the delta orientation
            if -1.0 <= np.dot(robot_unit, distance_vector) / distance <= 1.0:
                delta_orientation = np.arccos(np.dot(robot_unit, distance_vector) / distance)
            else:
                delta_orientation = 0.0
            # Check if the delta orientation is grater than the tolerance
            if delta_orientation > self.settings.robot_orientation_tolerance:
                # Check if the direction is correct
                # Calculate the actual future distance between the target position and robot with the actual orientation
                actual_future_robot_pos = robot_unit * distance
                actual_future_distance_vector = target_pos - actual_future_robot_pos
                actual_future_distance = np.linalg.norm(actual_future_distance_vector)
                # Calculate the new future distance between the target position and robot with a new orientation
                new_orientation = (self.robot.odo_o + delta_orientation) % (2 * np.pi)
                new_future_robot_unit = np.array([np.cos(new_orientation), -1 * np.sin(new_orientation)])
                new_future_robot_pos = new_future_robot_unit * distance
                new_future_distance_vector = target_pos - new_future_robot_pos
                new_future_distance = np.linalg.norm(new_future_distance_vector)
                # If the new future distance is smaller than the actual future distance
                if new_future_distance < actual_future_distance:
                    # Rotate the robot by the increment
                    self.robot.rotate_by_increment(delta_orientation)
                elif new_future_distance > actual_future_distance:
                    # Rotate the robot by the negative increment
                    self.robot.rotate_by_increment(-delta_orientation)
                else:
                    # Rotate the robot a little bit to get out of equilibrium
                    self.robot.rotate_by_increment(0.0174533)
            # The robot is oriented into the right direction, it has to move forward
            else:
                self.robot.moving_forward = True
                return True
        # The robot reached the target position it has to stop to sense
        else:
            # Turn the moving off and indicate that the target is reached
            self.robot.moving_forward = False
            self.robot.target_reached = True
            # Check if autonomous exploration is active
            if self.settings.autonomous_exploration:
                # Turn the sensing on and clear the clusters
                self.robot.sensing = True
                #self.map.clusters.clear()
            return False

    def _calculate_robot_position(self):
        """Calculates the next position of the robots"""
        for robot in self.robots.sprites():
            # Calculate the new x, y position
            if robot.moving_forward:
                x = robot.x - robot.settings.robot_speed * np.sin(robot.o)
                y = robot.y - robot.settings.robot_speed * np.cos(robot.o)
                self._check_robot_movement(robot, x, y)
            elif robot.moving_backward:
                x = robot.x + robot.settings.robot_speed * np.sin(robot.o)
                y = robot.y + robot.settings.robot_speed * np.cos(robot.o)
                self._check_robot_movement(robot, x, y)

    def _check_robot_movement(self, robot, x, y):
        """Checks if the robot can move to the given position without going trough a wall or an object"""
        # Calculate the future edges of the robot's rect
        left = x - (robot.rect.width / 2)
        right = x + (robot.rect.width / 2)
        top = y - (robot.rect.height / 2)
        bottom = y + (robot.rect.height / 2)

        # Check if the robot would go through the edge of the world
        if top < 0 or right > self.settings.layout_width or bottom > self.screen.get_rect().bottom or left < 0:
            return False

        # Calculate robot and object collisions
        collisions = pygame.sprite.groupcollide(self.robots, self.objects, False, False)

        # Check if the robot would go through any objects
        if robot in collisions.keys():
            for ob in collisions[robot]:
                current_distance = np.sqrt((ob.rect.x - robot.x) ** 2 + (ob.rect.y - robot.y) ** 2)
                new_distance = np.sqrt((ob.rect.x - x) ** 2 + (ob.rect.y - y) ** 2)
                if new_distance < current_distance:
                    return False

        # The robot can move to position
        robot.x = x
        robot.y = y
        return True

    def _emit_laser_beams(self):
        """Emits a laser beam from the lidar"""
        for orientation in np.linspace(-1 * self.settings.lidar_view_angle / 2, self.settings.lidar_view_angle / 2,
                                       self.settings.laser_number):
            new_beam = Beam(self, self.robot.o + orientation)
            self.laser_beams.add(new_beam)

    def _update_laser_beams(self):
        """Updates the position of the laser beams"""
        self.laser_beams.update()

    def _check_laser_beam_distance(self):
        """Checks distance from the emitting position"""
        for beam in self.laser_beams.sprites():
            # Calculate the distance
            distance = np.sqrt((beam.x - beam.init_x) ** 2 + (beam.y - beam.init_y) ** 2)

            # If the distance is larger then the lidar's sensing distance
            if distance > self.settings.lidar_sensing_distance:
                # Remove the beam
                self.laser_beams.remove(beam)
            else:
                # Change the color according to distance (the further the lighter)
                color_code = int((distance / self.settings.lidar_sensing_distance) * 255)
                beam.color = (255, color_code, color_code)

    def _check_laser_beam_collision(self):
        """Checks if the beam hit an object or wall"""
        # Object collision
        collisions = pygame.sprite.groupcollide(self.objects, self.laser_beams, False, False)

        for ob in collisions.values():
            for beam in ob:
                # Stop moving the laser beam if it hit an object
                beam.free_to_move = False

        # Wall collision
        for beam in self.laser_beams.sprites():
            if beam.rect.top < 0 or beam.rect.bottom > self.screen.get_rect().bottom:
                beam.free_to_move = False
            elif beam.rect.right > self.settings.layout_width or beam.rect.left < 0:
                beam.free_to_move = False

    def _update_map(self):
        """Updates the map"""
        # Check if any of the laser beams hit an object
        for beam in self.laser_beams.sprites():
            if not beam.free_to_move:
                # Calculate the coordinates based on the odometry of the robot
                # Transfer the coordinates into the origin of the robot's coordinate system
                loc_x = beam.x - self.robot.x
                loc_y = beam.y - self.robot.y

                # Rotate the coordinates in the robot's coordinate system
                theta = self.robot.init_o + (np.pi / 2)
                rot_x = loc_x * np.cos(theta) - loc_y * np.sin(theta)
                rot_y = loc_x * np.sin(theta) + loc_y * np.cos(theta)

                # Transfer the rotated coordinates based on the robot's odometry in the robot's coordinate system
                map_x = rot_x + self.robot.odo_x
                map_y = rot_y + self.robot.odo_y

                # Add the coordinates to the map
                self.map.add_lidar_detection_point(map_x, map_y)

                # Remove the laser beam
                self.laser_beams.remove(beam)

    def _update_screen(self):
        """Update images on the screen and flip to the new screen"""
        # Fill the screen
        self.screen.fill(self.settings.bg_color)

        # Draw the robot
        for robot in self.robots.sprites():
            robot.blitme()

        # Draw laser beams
        for beam in self.laser_beams.sprites():
            beam.draw_beam()

        # Draw objects
        for ob in self.objects.sprites():
            ob.blitme()

        # Draw the robot's map
        self.map.draw_map()

        # Draw the display
        self.display.show_display()

        # Draw the menu
        self.menu.show_menu()

        # Make the most recent drawn screen visible
        pygame.display.flip()


if __name__ == '__main__':
    # Make a simulation instance and run it
    simulation = Simulation()
    simulation.run_simulation()
