import pygame
import numpy as np
import copy
from sklearn.decomposition import PCA
import random
from scipy.spatial import ConvexHull

from cluster import Cluster
from obstacle import Obstacle
from path_node import PathNode


class Map:
    """A class to represent the robot's map"""

    def __init__(self, simulation):
        """Initialize the map"""
        self.screen = simulation.screen
        self.settings = simulation.settings
        self.robot = simulation.robot

        # Global map position
        self.x = self.settings.layout_width
        self.y = 0

        # Center map position
        self.centerX = self.settings.layout_width + 400
        self.centerY = self.settings.screen_height / 2

        # Map scale
        self.max_coordinate = 0.0
        self.scale = 1.0

        # Map cell size
        self.cell_size = self.settings.map_cell_size

        # LIDAR detection points
        self.lidar_detection_points = set()

        # Obstacles
        self.obstacles = []

        # Clusters
        self.clusters = []

        # Explored trajectory points
        self.trajectory_points = [(0, 0)]

        # Path points for autonomous navigation
        self.path = []

        # Create the rect for the map background
        self.map_screen_rect = pygame.Rect(self.x, self.y, 800, 800)

        # Font settings
        self.text_color = self.settings.map_text_color
        self.font = pygame.font.SysFont(None, self.settings.map_font_size)

        # Flag for detecting obstacles
        self.flag_obstacle_detection = False

        # Flag growing clusters
        self.flag_cluster_growing = False

        # Flag topological map is completed
        self.flag_topological_map_completed = False

    def add_lidar_detection_point(self, x, y):
        """Adds a lidar detection point"""
        # Calculate the coordinates of the lidar detection point
        grid_x = int(x)
        grid_y = int(y)

        # Create a new detection point
        previous_length = len(self.lidar_detection_points)
        detection_point = (grid_x, grid_y)
        self.lidar_detection_points.add(detection_point)

        # Check if the detection point was added to the set
        if len(self.lidar_detection_points) > previous_length:
            # Check if any of new coordinates are larger then the actual maximum
            actual_max = max(abs(grid_x), abs(grid_y))
            if actual_max > self.max_coordinate:
                # Update the scale
                self.max_coordinate = actual_max
                if (350 / actual_max) < self.scale:
                    self.scale = (350 / actual_max)

    def detecting_obstacles(self):
        """Detects grid cells that form a rectangular obstacle"""
        if self.flag_obstacle_detection:
            # Clear the list of obstacles
            self.obstacles.clear()
            # Get the list of the occupied grid cells
            occupied_grid_cells = list(copy.deepcopy(self.lidar_detection_points))
            # Run the algorithm while there are grid cells on the list
            while occupied_grid_cells:
                # Initialize a new obstacle by using the last element on the occupied grid cell list
                new_obstacle = [occupied_grid_cells.pop()]
                # Go through all the obstacle and occupied cells
                for obstacle_cell in new_obstacle:
                    for occupied_cell in occupied_grid_cells:
                        # Calculate the distance in X and Y directions
                        distanceX = abs(occupied_cell[0] - obstacle_cell[0])
                        distanceY = abs(occupied_cell[1] - obstacle_cell[1])
                        # If the distance is smaller then two times the cell size in X and Y direction and the occupied cell is not assigned to the new obstacle
                        if distanceX < (self.cell_size * 2) and distanceY < (self.cell_size * 2) and occupied_cell not in new_obstacle:
                            # Add the cell to the new obstacle and remove it from the occupied grid cell list
                            new_obstacle.append(occupied_cell)
                            occupied_grid_cells.remove(occupied_cell)
                # A set for the obstacle cells
                obstacle_cells = set()
                # Color of the obstacle
                red = int(random.random() * 255)
                green = int(random.random() * 255)
                blue = int(random.random() * 255)
                color = (red, green, blue)
                # Go through the points
                while new_obstacle:
                    # Get the points one by one
                    point = new_obstacle.pop()
                    # Calculate the coordinate of the occupied grid cell
                    x = point[0]
                    y = point[1]
                    half_cell = self.cell_size / 2
                    x = int((x - half_cell) / half_cell) * half_cell
                    y = int((y - half_cell) / half_cell) * half_cell
                    # Add the coordinates to the obstacle cells
                    obstacle_cells.add((x, y))
                # Go through the points and create obstacles
                for cell in obstacle_cells:
                    obstacle = Obstacle(cell[0], cell[1], self.cell_size, self.cell_size)
                    obstacle.color = color
                    self.obstacles.append(obstacle)
            return True
        else:
            return False

    def growing_cluster(self, center_point, aspect_ratio_multiplier=1.0, max_distance=100):
        """Grows convex cluster"""
        if self.flag_cluster_growing:
            # Create a new a cluster at the center point
            new_cluster = Cluster(center_point[0], center_point[1])
            # Grows a cluster while there are new candidate cells added
            while True:
                # Find all directly adjacent non-occupied cells for the current cell cluster
                adjacent_cells = set()
                # Go through all the perimeter points of the current cluster shape
                for perimeter_point in new_cluster.get_perimeter_points():
                    # Create all the possible neighbouring cells
                    for x_mult in [-1, 0, 1]:
                        for y_mult in [-1, 0, 1]:
                            new_cell = (perimeter_point[0] + x_mult * self.cell_size,
                                        perimeter_point[1] + y_mult * self.cell_size)
                            # Check if the neighbouring cell is not already present in the cluster and
                            # it does not fall into an obstacle
                            if (new_cell not in new_cluster.cells) and not self._cell_in_obstacle(new_cell):
                                # Add the cell to the adjacent cells
                                adjacent_cells.add(new_cell)

                # Collect the cells from the neighbouring cells that form a compact cluster with the core cells
                compact_cells = set()
                # Perform principal component analysis (PCA) if there are enough cells
                if len(new_cluster.cells) > 2:
                    # Principal component analysis
                    pca = PCA(n_components=2)
                    pca.fit(list(new_cluster.cells))
                    # Calculate the length of the principal axes
                    length_1 = 3 * np.sqrt(pca.explained_variance_[0])
                    length_2 = 3 * np.sqrt(pca.explained_variance_[1])
                    # Calculate the aspect ratio with respect to the shorter axis
                    aspect_ratio = min(length_1, length_2) * aspect_ratio_multiplier
                    # Go through the adjacent cells
                    for cell in adjacent_cells:
                        # Calculate the distance from the center of the cluster
                        distance = np.sqrt((cell[0] - new_cluster.centerX) ** 2 + (cell[1] - new_cluster.centerY) ** 2)
                        # Check if the candidate cell is not too far from the center and it forms a compact cluster
                        if (distance <= max_distance) and (distance <= aspect_ratio):
                            compact_cells.add(cell)
                # Otherwise add the adjacent cells to the compact cells
                else:
                    compact_cells = adjacent_cells

                # Collect cells that form a convex cluster with the core cells
                convex_cells = set()
                for actual_candidate_cell in compact_cells:
                    # Create rays between the actual candidate cell and cluster cells
                    rays = set()
                    for cluster_cell in new_cluster.get_perimeter_points():
                        rays = rays.union(self._calculate_rays(actual_candidate_cell, cluster_cell))
                    # Calculates if there is intersection between the rays and any of the obstacles
                    intersections = []
                    for ray in rays:
                        for obstacle in self.obstacles:
                            # Calculate the obstacle boundaries
                            obstacle_boundaries = (obstacle.x,
                                                   obstacle.y,
                                                   obstacle.x + obstacle.sizeX,
                                                   obstacle.y + obstacle.sizeY)
                            # Check if there is intersection
                            if self._line_rectangle_intersection(ray[0],
                                                                 ray[1],
                                                                 ray[2],
                                                                 ray[3],
                                                                 obstacle_boundaries[0],
                                                                 obstacle_boundaries[1],
                                                                 obstacle_boundaries[2],
                                                                 obstacle_boundaries[3]):
                                intersections.append(True)
                    # If the rays have no intersection with any of the obstacles
                    if True not in intersections:
                        # Then add the actual candidate cell to the convex cells
                        convex_cells.add(actual_candidate_cell)

                # Check if there are new candidate cells
                if convex_cells:
                    # Add the new candidate cells to the cluster
                    new_cluster.add_cells(convex_cells)
                else:
                    # Stop growing the cluster
                    break
            # Add the new cluster to the clusters
            self.clusters.append(new_cluster)
            return True
        else:
            return False

    def _cell_in_obstacle(self, point):
        """Checks if a cell intersects with an obstacle"""
        # Get the cell size
        cell_size = self.cell_size
        # Calculate all the corner points of the cell
        cell_points = []
        for x_mult in [0, 1]:
            for y_mult in [0, 1]:
                cell_points.append((point[0] + x_mult * cell_size, point[1] + y_mult * cell_size))
        # Go through the obstacles
        for obstacle in self.obstacles:
            # Calculate the top-left and bottom-right corners of the obstacle
            O1_X = obstacle.x
            O1_Y = obstacle.y
            O2_X = obstacle.x + obstacle.sizeX
            O2_Y = obstacle.y + obstacle.sizeY
            # Check if any of the cell points lays inside the obstacle
            for cell_point in cell_points:
                if O1_X <= cell_point[0] <= O2_X and O1_Y <= cell_point[1] <= O2_Y:
                    return True
        # If we reach this point then there is no intersection
        return False

    def _calculate_rays(self, actual_cell, other_cell):
        """Calculates the rays between the actual candidate cell and an other cell"""
        # Create a set for the new rays
        new_rays = set()
        # Calculate the local vector components from the actual cell to the other cell
        delta_x = other_cell[0] - actual_cell[0]
        delta_y = other_cell[1] - actual_cell[1]
        # Check if the deltas are not zeros
        if delta_x != 0 and delta_y != 0:
            # Calculate the rays when the other cell falls into the first or third quarter of the coordinate system
            if (delta_x >= 0 and delta_y <= 0) or (delta_x < 0 and delta_y > 0):
                ray_1 = (actual_cell[0], actual_cell[1], delta_x, delta_y)
                ray_2 = (actual_cell[0] + self.cell_size, actual_cell[1] + self.cell_size, delta_x, delta_y)
                new_rays.add(ray_1)
                new_rays.add(ray_2)
            # Calculate the rays when the other cell fall into the second or fourth quarter of the coordinate system
            elif (delta_x >= 0 and delta_y > 0) or (delta_x < 0 and delta_y <= 0):
                ray_1 = (actual_cell[0] + self.cell_size, actual_cell[1], delta_x, delta_y)
                ray_2 = (actual_cell[0], actual_cell[1] + self.cell_size, delta_x, delta_y)
                new_rays.add(ray_1)
                new_rays.add(ray_2)
        # Returns the set
        return new_rays

    def _line_rectangle_intersection(self, x_zero, y_zero, delta_x, delta_y, x_min, y_min, x_max, y_max, collision_point=False):
        """Checks if a line intersects with a rectangle by the Liang-Barsky algorithm.
        Implemented from https://gist.github.com/ChickenProp/3194723"""
        # Initialization
        p = [-delta_x, delta_x, -delta_y, delta_y]
        q = [x_zero - x_min, x_max - x_zero, y_zero - y_min, y_max - y_zero]
        u1 = float("-inf")
        u2 = float("inf")
        # Calculations
        for i in range(4):
            if p[i] == 0:
                if q[i] < 0:
                    return False
            else:
                t = q[i] / p[i]
                if p[i] < 0 and u1 < t:
                    u1 = t
                elif p[i] > 0 and u2 > t:
                    u2 = t
        if (u1 > u2) or (u1 > 1) or (u1 < 0):
            return False
        # Collision point
        if collision_point:
            collision_x = x_zero + u1 * delta_x
            collision_y = y_zero + u1 * delta_y
            return collision_x, collision_y
        else:
            return True

    def merging_clusters(self):
        """Merges clusters that have intersection"""
        # Run iterations until there is no more clusters to merge
        while True:
            # Get the initial length of clusters
            len_initial_clusters = len(self.clusters)
            # Make a list with the cluster indices
            initial_cluster_indices = [i for i in range(len(self.clusters))]
            # Start a new list of merged clusters
            merged_clusters = []

            # Find candidate pairs
            candidate_pairs = set()
            for i in range(len(self.clusters)):
                for j in range(i + 1, len(self.clusters)):
                    # Get the cells from the two clusters
                    actual_cluster_cells = self.clusters[i].cells
                    next_cluster_cells = self.clusters[j].cells
                    # Check if there is intersection between the two clusters
                    if actual_cluster_cells.intersection(next_cluster_cells):
                        candidate_pairs.add((i, j))

            # If there are candidate pairs
            if candidate_pairs:
                # Go through the pairs one by one
                for i, j in candidate_pairs:
                    # Check if the clusters are on the initial list
                    if i in initial_cluster_indices and j in initial_cluster_indices:
                        # Get the clusters
                        cluster_i = self.clusters[i]
                        cluster_j = self.clusters[j]
                        # Get the perimeter points of the two clusters
                        perimeter_i = cluster_i.get_perimeter_points()
                        perimeter_j = cluster_j.get_perimeter_points()
                        # Merge the perimeter points and calculate the convex hull of the two clusters
                        merged_perimeter = perimeter_i.union(perimeter_j)
                        hull = ConvexHull(list(merged_perimeter))

                        # Check if any of the obstacle points are inside the convex hull
                        obstacle_in_hull = False
                        for obstacle in self.obstacles:
                            point = (obstacle.x, obstacle.y)
                            if self._point_in_hull(point, hull):
                                obstacle_in_hull = True
                                break
                        # If there is no obstacles in the convex hull of the two clusters
                        if not obstacle_in_hull:
                            # Calculate the center of the convex hull
                            centerX = np.mean(hull.points[hull.vertices, 0])
                            centerY = np.mean(hull.points[hull.vertices, 1])
                            # Create a new cluster
                            new_cluster = Cluster(centerX, centerY)
                            # Calculate the union of the two clusters
                            new_cells = cluster_i.cells.union(cluster_j.cells)
                            # Add the cells to the new cluster
                            new_cluster.add_cells(new_cells)
                            # Add the merged cluster to the list of merged clusters
                            merged_clusters.append(new_cluster)
                            # Remove the merged clusters from the initial list
                            initial_cluster_indices.remove(i)
                            initial_cluster_indices.remove(j)

            # Add the remaining clusters to the merged cluster list
            for index in initial_cluster_indices:
                merged_clusters.append(self.clusters[index])

            # Replace the clusters with the merged clusters
            self.clusters = merged_clusters

            # Check if the length of clusters did not change over the iteration
            if len(self.clusters) == len_initial_clusters:
                break

    def _point_in_hull(self, point, hull, tolerance=1e-12):
        """Checks if a point falls into a given convex hull. Returns True or False.
        Implemented from: https://stackoverflow.com/a/42165596"""
        return all((np.dot(eq[:-1], point) + eq[-1] <= tolerance) for eq in hull.equations)

    def remove_covered_clusters(self):
        """Removes clusters that are highly covered by other clusters"""
        # Run iterations until there are no more highly covered cluster
        while True:
            # Create a list for the highly covered clusters
            highly_covered_clusters = []

            # Go through all the clusters
            for i in range(len(self.clusters)):
                # Create a set for the intersecting cells in the current cluster
                intersecting_cells = set()
                for j in range(len(self.clusters)):
                    # Check if the clusters are not identical
                    if i != j:
                        # Calculate the intersecting cells
                        current_overlapping_cells = self.clusters[i].cells.intersection(self.clusters[j].cells)
                        # Add the current intersecting cells to the rest
                        intersecting_cells = intersecting_cells.union(current_overlapping_cells)
                # Check if the current cluster is highly covered
                if len(intersecting_cells) >= (self.settings.map_coverage_threshold * len(self.clusters[i].cells)):
                    # Add the cluster to the list of highly covered clusters
                    highly_covered_clusters.append(i)
                    break

            # Check if there is any highly covered cluster
            if highly_covered_clusters:
                # Remove the highly covered clusters
                del self.clusters[highly_covered_clusters[0]]
            else:
                break

    def find_cluster_neighbours(self):
        """Finds the neighbouring clusters"""
        # Go through all the clusters
        for i in range(len(self.clusters)):
            for j in range(len(self.clusters)):
                # Check if the cluster is not itself
                if i != j:
                    # Check if there is an intersection with the other cluster
                    if self.clusters[i].cells.intersection(self.clusters[j].cells):
                        # Add the other cluster to the neighbouring clusters
                        self.clusters[i].neighbours.add(j)

    def create_navigation_nodes(self):
        """Creates navigation nodes within the clusters"""
        # Go through all the clusters
        for cluster in self.clusters:
            # Update the center point
            cluster.update_center_point()
            # Add the center point to the navigation points
            cluster.navigation_node_positions.add((cluster.centerX, cluster.centerY))
            # Go through all the neighbouring clusters
            for neighbour_id in cluster.neighbours:
                # Calculate the intersecting cells with the neighbour
                intersection = cluster.cells.intersection(self.clusters[neighbour_id].cells)
                # Collect the x & y coordinates of the intersecting cells
                x_coords = [point[0] for point in intersection]
                y_coords = [point[1] for point in intersection]
                # Calculate the mean of the coordinates
                mean_x = np.average(x_coords)
                mean_y = np.average(y_coords)
                # Add the portal point to the navigation points
                cluster.navigation_node_positions.add((mean_x, mean_y))

    def path_planner(self, start, end):
        """
            Plans a path from the current location to the actual target point
            Implementation of A* search algorithm
            Reference1: https://brilliant.org/wiki/a-star-search/
            Reference2: https://towardsdatascience.com/a-star-a-search-algorithm-eb495fb156bb
        """
        # Find the cluster that contains the starting and end points
        start_cluster = None
        end_cluster = None
        for i in range(len(self.clusters)):
            if self.clusters[i].point_in_cluster(start) and start_cluster is None:
                start_cluster = i
            if self.clusters[i].point_in_cluster(end) and end_cluster is None:
                end_cluster = i

        # Check if the start and end point fall into the same cluster
        if start_cluster == end_cluster:
            self.path.append(end)
        else:
            # Add the start and end nodes to the cluster temporarily
            self.clusters[start_cluster].navigation_node_positions.add(start)
            self.clusters[end_cluster].navigation_node_positions.add(end)

            # Start an open and closed list
            open_list = []
            closed_list = []

            # Create starting and end nodes
            starting_node = PathNode(start_cluster, None, start)
            end_node = PathNode(end_cluster, None, end)

            # Configure the starting node
            starting_node.g = 0
            starting_node.h = np.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)
            starting_node.f = starting_node.g + starting_node.h

            # Add starting node to the open list
            open_list.append(starting_node)

            # Iterate until we reach the end node
            while open_list:
                # Find the lowest f score in the open list
                f_scores = [node.f for node in open_list]
                min_f_score = min(f_scores)
                for node in open_list:
                    if node.f == min_f_score:
                        current_node = node

                # Check if this is the end node
                if current_node == end_node:
                    # Extract the path from the nodes
                    parent = current_node
                    while parent is not None:
                        self.path.append(parent.position)
                        parent = parent.parent_node
                    self.path.reverse()
                    break

                # Remove the current node from the open list
                open_list.remove(current_node)

                # Put the current node into the closed list
                closed_list.append(current_node)

                # Search for neighbouring clusters involving the current cluster
                neighbour_clusters = [current_node.cluster, *self.clusters[current_node.cluster].neighbours]
                # Search neighbouring nodes within the neighbouring clusters
                neighbour_nodes = []
                for neighbour_cluster in neighbour_clusters:
                    # Check if the current node belongs to the neighbouring cluster too (it is a portal node between the two clusters)
                    if current_node.position in self.clusters[neighbour_cluster].navigation_node_positions:
                        # Go through the neighbouring nodes
                        for node_position in self.clusters[neighbour_cluster].navigation_node_positions:
                            # Check if the node of interest is not the current node
                            if node_position != current_node.position:
                                # Create a new node
                                new_node = PathNode(neighbour_cluster, current_node, node_position)
                                # Calculate the cost from the parent node to the new node
                                local_x = new_node.position[0] - current_node.position[0]
                                local_y = new_node.position[1] - current_node.position[1]
                                new_node.g = np.sqrt(local_x ** 2 + local_y ** 2) + current_node.g
                                # Calculate the cost from the new node to the end node (heuristics - Euclidean Distance)
                                local_x = end_node.position[0] - new_node.position[0]
                                local_y = end_node.position[1] - new_node.position[1]
                                new_node.h = np.sqrt(local_x ** 2 + local_y ** 2)
                                # Calculate the total estimated cost
                                new_node.f = new_node.g + new_node.h
                                # Add the node to the neighbouring nodes
                                neighbour_nodes.append(new_node)

                # Go through all the neighbour nodes
                for node in neighbour_nodes:
                    # Check if the node is in the closed list
                    if node in closed_list:
                        continue
                    # Check if the node is in the open list
                    elif node in open_list:
                        index = open_list.index(node)
                        open_node = open_list[index]
                        # Check if the current neighbour has a lower g value than the one on the list
                        if node.g < open_node.g:
                            # Update the node in the open list
                            open_node.g = node.g
                            open_node.parent_node = current_node
                    # If the node is not on the lists it is added to the open list
                    elif node not in closed_list and node not in open_list:
                        open_list.append(node)

            # Remove the start and end nodes from the clusters
            self.clusters[start_cluster].navigation_node_positions.remove(start)
            self.clusters[end_cluster].navigation_node_positions.remove(end)

    def draw_map(self):
        """Draws the map to the screen"""
        # Draw the background
        pygame.draw.rect(self.screen, self.settings.map_color, self.map_screen_rect)

        # Draw obstacles
        for obstacle in self.obstacles:
            x = (obstacle.x * self.scale) + self.centerX
            y = (obstacle.y * self.scale) + self.centerY
            pygame.draw.rect(self.screen, obstacle.color, pygame.Rect(x, y, obstacle.sizeX, obstacle.sizeY))

        # Draw the lidar detection points
        if self.settings.map_draw_lidar_points:
            for cell in self.lidar_detection_points:
                x = (cell[0] * self.scale) + self.centerX
                y = (cell[1] * self.scale) + self.centerY
                pygame.draw.circle(self.screen, self.settings.map_lidar_color, (x, y), 1)

        # Draw the clusters
        if self.clusters:
            if self.settings.map_draw_all_clusters:
                for cluster in self.clusters:
                    for cell in cluster.cells:
                        x = (cell[0] * self.scale) + self.centerX
                        y = (cell[1] * self.scale) + self.centerY
                        pygame.draw.rect(self.screen, cluster.color, pygame.Rect(x, y, self.cell_size, self.cell_size))
            else:
                cluster = self.clusters[-1]
                for cell in cluster.cells:
                    x = (cell[0] * self.scale) + self.centerX
                    y = (cell[1] * self.scale) + self.centerY
                    pygame.draw.rect(self.screen, cluster.color, pygame.Rect(x, y, self.cell_size, self.cell_size))

        # Draw the X-axis of the robot's local coordinate system
        pygame.draw.line(self.screen, (255, 0, 0), (self.centerX, self.centerY), (self.centerX + 50, self.centerY), 2)
        pygame.draw.line(self.screen, (255, 0, 0), (self.centerX + 50, self.centerY), (self.centerX + 40, self.centerY - 5), 2)
        pygame.draw.line(self.screen, (255, 0, 0), (self.centerX + 50, self.centerY), (self.centerX + 40, self.centerY + 5), 2)
        self.x_image = self.font.render("X", True, self.text_color, self.settings.map_color)
        self.x_rect = self.x_image.get_rect()
        self.x_rect.midbottom = (self.centerX + 45, self.centerY - 10)
        self.screen.blit(self.x_image, self.x_rect)

        # Draw the Y-axis of the robot's local coordinate system
        pygame.draw.line(self.screen, (0, 255, 0), (self.centerX, self.centerY), (self.centerX, self.centerY + 50), 2)
        pygame.draw.line(self.screen, (0, 255, 0), (self.centerX, self.centerY + 50), (self.centerX - 5, self.centerY + 40), 2)
        pygame.draw.line(self.screen, (0, 255, 0), (self.centerX, self.centerY + 50), (self.centerX + 5, self.centerY + 40), 2)
        self.y_image = self.font.render("Y", True, self.text_color, self.settings.map_color)
        self.y_rect = self.y_image.get_rect()
        self.y_rect.midright = (self.centerX - 10, self.centerY + 45)
        self.screen.blit(self.y_image, self.y_rect)

        # Draw the trajectory points already explored by the robot
        if self.settings.map_draw_trajectory_points:
            for point in self.trajectory_points[0:-1]:
                x = (point[0] * self.scale) + self.centerX
                y = (point[1] * self.scale) + self.centerY
                pygame.draw.circle(self.screen, (0, 0, 255), (x, y), self.settings.map_target_size)
                # Drawing the index of the point on the map
                index_str = str(self.trajectory_points.index(point))
                self.index_image = self.font.render(index_str, True, self.text_color, self.settings.map_color)
                self.index_rect = self.index_image.get_rect()
                self.index_rect.midbottom = (x, y - 10)
                self.screen.blit(self.index_image, self.index_rect)

            # Draw the last trajectory point and its index on the map
            x = (self.trajectory_points[-1][0] * self.scale) + self.centerX
            y = (self.trajectory_points[-1][1] * self.scale) + self.centerY
            pygame.draw.circle(self.screen, (255, 255, 0), (x, y), self.settings.map_target_size)
            index_str = str(len(self.trajectory_points) - 1)
            self.index_image = self.font.render(index_str, True, self.text_color, self.settings.map_color)
            self.index_rect = self.index_image.get_rect()
            self.index_rect.midbottom = (x, y - 10)
            self.screen.blit(self.index_image, self.index_rect)

        # Draw navigation nodes
        for cluster in self.clusters:
            for node in cluster.navigation_node_positions:
                x = (node[0] * self.scale) + self.centerX
                y = (node[1] * self.scale) + self.centerY
                pygame.draw.circle(self.screen, (255, 0, 0), (x, y), self.settings.map_target_size)
                pygame.draw.circle(self.screen, cluster.color, (x, y), self.settings.map_target_size - 2)

        # Draw the robot's current location
        x = (self.robot.odo_x * self.scale) + self.centerX
        y = (self.robot.odo_y * self.scale) + self.centerY
        pygame.draw.circle(self.screen, (0, 255, 0), (x, y), self.settings.map_target_size)

        # Draw targets for autonomous navigation
        for x, y in self.settings.layout["Targets"]:
            x = (x * self.scale) + self.centerX
            y = (y * self.scale) + self.centerY
            pygame.draw.rect(self.screen, (255, 0, 0), pygame.Rect(x - self.settings.map_target_size / 2, y - self.settings.map_target_size / 2, self.settings.map_target_size, self.settings.map_target_size))

        # Draw navigation path
        if self.path:
            for x, y in self.path:
                x = (x * self.scale) + self.centerX
                y = (y * self.scale) + self.centerY
                pygame.draw.rect(self.screen, (255, 255, 255), pygame.Rect(x - self.settings.map_target_size / 2,
                                                                           y - self.settings.map_target_size / 2,
                                                                           self.settings.map_target_size,
                                                                           self.settings.map_target_size))
