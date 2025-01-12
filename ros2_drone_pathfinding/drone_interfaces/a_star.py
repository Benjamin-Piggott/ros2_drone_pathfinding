# Create at ros2_drone_pathfinding/drone_interfaces/a_star.py

import math
import heapq
from typing import List, Tuple, Optional
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Path, OccupancyGrid

class Cell:
    def __init__(self):
        self.parent_i: float = 0.0
        self.parent_j: float = 0.0
        self.f: float = float("inf")
        self.g: float = float("inf")
        self.h: float = 0.0

class PathFinder:
    def __init__(self, grid_msg: OccupancyGrid):
        """Initialise PathFinder with ROS2 OccupancyGrid message."""
        self.resolution = grid_msg.info.resolution
        self.width = grid_msg.info.width
        self.height = grid_msg.info.height
        
        # Convert flat OccupancyGrid data to 2D grid
        # In ROS2, occupied cells have values > 50
        self.grid = [[1 if grid_msg.data[i * self.width + j] < 50 else 0 
                     for j in range(self.width)]
                     for i in range(self.height)]

    def world_to_grid(self, point: Point) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        grid_x = int(point.x / self.resolution)
        grid_y = int(point.y / self.resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x: int, grid_y: int) -> Point:
        """Convert grid coordinates to world coordinates."""
        world_point = Point()
        world_point.x = (grid_x + 0.5) * self.resolution
        world_point.y = (grid_y + 0.5) * self.resolution
        world_point.z = 0.0
        return world_point

    def is_valid(self, row: int, col: int) -> bool:
        """Check if coordinates are within grid boundaries."""
        return 0 <= row < self.height and 0 <= col < self.width

    def is_unblocked(self, row: int, col: int) -> bool:
        """Check if cell is passable."""
        return self.grid[row][col] == 1

    def calculate_h_value(self, row: int, col: int, dest: Tuple[int, int]) -> float:
        """Calculate heuristic value using Euclidean distance."""
        return math.sqrt((row - dest[0]) ** 2 + (col - dest[1]) ** 2)

    def create_path_msg(self, path: List[Tuple[int, int]], frame_id: str = 'map') -> Path:
        """Convert grid path to ROS2 Path message."""
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        
        for grid_x, grid_y in path:
            pose = Pose()
            world_point = self.grid_to_world(grid_x, grid_y)
            pose.position = world_point
            path_msg.poses.append(pose)
            
        return path_msg

    def find_path(self, start: Point, goal: Point) -> Tuple[Optional[Path], List[str]]:
        """Find path between start and goal points using A* algorithm."""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)
        
        # Initialise visualisation steps
        steps = []

        # Check validity of start and goal positions
        if not all(self.is_valid(coord[0], coord[1]) for coord in [start_grid, goal_grid]):
            return None, ["Invalid start or goal position"]
        if not all(self.is_unblocked(coord[0], coord[1]) for coord in [start_grid, goal_grid]):
            return None, ["Start or goal position is blocked"]

        # Initialise data structures
        closed_list = [[False for _ in range(self.width)] for _ in range(self.height)]
        cell_details = [[Cell() for _ in range(self.width)] for _ in range(self.height)]
        
        # Initialise start cell
        i, j = start_grid
        cell_details[i][j].f = 0.0
        cell_details[i][j].g = 0.0
        cell_details[i][j].h = 0.0
        cell_details[i][j].parent_i = i
        cell_details[i][j].parent_j = j

        # Initialise open list with start cell
        open_list = [(0.0, i, j)]
        
        # Define movement directions (8-directional movement)
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                     (1, 1), (1, -1), (-1, 1), (-1, -1)]

        found_path = False
        
        while open_list:
            f, i, j = heapq.heappop(open_list)
            closed_list[i][j] = True

            for di, dj in directions:
                new_i, new_j = i + di, j + dj
                
                if not self.is_valid(new_i, new_j):
                    continue
                if not self.is_unblocked(new_i, new_j):
                    continue
                if closed_list[new_i][new_j]:
                    continue

                # Check if destination reached
                if (new_i, new_j) == goal_grid:
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    found_path = True
                    break

                # Calculate new path costs
                g_new = cell_details[i][j].g + math.sqrt(di*di + dj*dj)
                h_new = self.calculate_h_value(new_i, new_j, goal_grid)
                f_new = g_new + h_new

                # Update if better path found
                if cell_details[new_i][new_j].f > f_new:
                    heapq.heappush(open_list, (f_new, new_i, new_j))
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j

            if found_path:
                break

        if not found_path:
            return None, ["No path found"]

        # Reconstruct path
        path = []
        curr = goal_grid
        while curr != start_grid:
            path.append(curr)
            curr_i, curr_j = curr
            curr = (int(cell_details[curr_i][curr_j].parent_i),
                   int(cell_details[curr_i][curr_j].parent_j))
        path.append(start_grid)
        path.reverse()

        # Create ROS2 Path message
        path_msg = self.create_path_msg(path)
        
        return path_msg, steps