# Create at ros2_drone_pathfinding/drone_interfaces/grid_utils.py
"""
Grid Utilities Module for Drone Pathfinding System

This module provides utility functions for handling and visualising grid-based environments
used in drone pathfinding operations. It includes tools for converting between ROS2
occupancy grids and internal grid representations, as well as functions for generating
visual representations of the pathfinding process.

The module supports:
    - Converting ROS2 OccupancyGrid messages to 2D grid arrays
    - Generating ASCII-based visualisations of the pathfinding environment
    - Creating grids with path visualisation for debugging and monitoring

Key Features:
    - Grid format conversion between ROS2 and internal representations
    - ASCII-based grid visualisation with multiple cell states
    - Support for visualising open/closed lists during pathfinding
    - Path visualisation utilities for debugging
"""

from typing import List, Tuple, Any
from nav_msgs.msg import OccupancyGrid

def generate_grid_string(grid: List[List[int]], 
                        open_list: List[Any], 
                        closed_list: List[List[bool]], 
                        start: Tuple[int, int], 
                        dest: Tuple[int, int]) -> str:
    """
    Generate a string representation of the grid for visualisation.
    
    Args:
        grid: 2D grid representing the environment
        open_list: List of cells to be explored
        closed_list: 2D list of explored cells
        start: Starting position coordinates
        dest: Destination position coordinates
    
    Returns:
        String representation of the grid with:
        - 'S': Start position
        - 'D': Destination
        - '*': Path
        - '█': Obstacle
        - 'o': Cells in open list
        - '.': Explored cells
        - ' ': Free space
    """
    grid_str = []
    rows, cols = len(grid), len(grid[0])
    
    for i in range(rows):
        row_str = []
        for j in range(cols):
            if (i, j) == start:
                row_str.append('S')
            elif (i, j) == dest:
                row_str.append('D')
            elif isinstance(grid[i][j], str) and grid[i][j] == '*':
                row_str.append('*')
            elif grid[i][j] == 0:
                row_str.append('█')
            elif (i, j) in [(x[1], x[2]) for x in open_list]:
                row_str.append('o')
            elif closed_list[i][j]:
                row_str.append('.')
            else:
                row_str.append(' ')
        grid_str.append(''.join(row_str))
    return '\n'.join(grid_str)

def convert_occupancy_grid(grid_msg: OccupancyGrid) -> List[List[int]]:
    """
    Convert ROS2 OccupancyGrid message to 2D grid.
    
    Args:
        grid_msg: ROS2 OccupancyGrid message
        
    Returns:
        2D list where 1 represents free space and 0 represents obstacles
    """
    width = grid_msg.info.width
    height = grid_msg.info.height
    
    # Convert flat array to 2D grid
    # In ROS2, occupancy values > 50 are considered obstacles
    grid = [[1 if grid_msg.data[i * width + j] < 50 else 0 
             for j in range(width)]
            for i in range(height)]
    
    return grid

def create_visualisation_grid(path: List[Tuple[int, int]], 
                            grid: List[List[int]]) -> List[List[Any]]:
    """
    Create a grid with path visualisation.
    
    Args:
        path: List of coordinates representing the path
        grid: Original 2D grid
        
    Returns:
        Grid with path marked using '*' characters
    """
    # Create a deep copy of the grid
    vis_grid = [row.copy() for row in grid]
    
    # Mark path cells with '*'
    for x, y in path:
        if vis_grid[x][y] != 0:  # Don't mark obstacles
            vis_grid[x][y] = '*'
    
    return vis_grid