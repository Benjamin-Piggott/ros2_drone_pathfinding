"""
Test suite for the drone pathfinding system components.

This module provides comprehensive testing for various components of the drone
pathfinding system including path planning, obstacle detection, and motor coordination.
It uses pytest framework for test organisation and execution.

Key Test Areas:
    - Path planning and coordinate transformations
    - Obstacle detection and grid updates
    - Motor coordination and navigation
    - End-to-end system integration
"""

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseArray
from ros2_drone_pathfinding.drone_interfaces.a_star import PathFinder
from ros2_drone_pathfinding.motor_coordinator import MotorCoordinator
from ros2_drone_pathfinding.sensors.obstacle_detection import ObstacleDetection

# Fixtures for test setup and teardown
@pytest.fixture(scope="session")
def ros_context():
    """
    Initialise and tear down ROS context for the test session.
    
    Yields:
        None: Provides ROS context for the duration of the test session
    """
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def test_grid():
    """
    Create a sample occupancy grid for testing.
    
    Returns:
        OccupancyGrid: Sample 5x5 grid with known obstacles
        Grid values: 0 = free space, 100 = obstacle
    """
    grid_msg = OccupancyGrid()
    grid_msg.info.width = 5
    grid_msg.info.height = 5
    grid_msg.info.resolution = 0.1  # 10cm per grid cell
    grid_msg.data = [
        0, 0, 0, 0, 0,      # Row 1: All free space
        0, 100, 0, 100, 0,  # Row 2: Obstacles at (1,1) and (1,3)
        0, 0, 0, 0, 0,      # Row 3: All free space
        0, 100, 0, 100, 0,  # Row 4: Obstacles at (3,1) and (3,3)
        0, 0, 0, 0, 0       # Row 5: All free space
    ]
    return grid_msg

@pytest.fixture
def pathfinder(test_grid):
    """
    Create a PathFinder instance with the test grid.
    
    Args:
        test_grid: Pytest fixture providing the sample occupancy grid
    
    Returns:
        PathFinder: Configured pathfinding instance
    """
    return PathFinder(test_grid)

@pytest.fixture
def obstacle_detection(ros_context):
    """
    Create and clean up ObstacleDetection node instance.
    
    Args:
        ros_context: Pytest fixture providing ROS context
    
    Yields:
        ObstacleDetection: Active obstacle detection node
    """
    node = ObstacleDetection()
    yield node
    node.destroy_node()

@pytest.fixture
def motor_coordinator(ros_context):
    """
    Create and clean up MotorCoordinator node instance.
    
    Args:
        ros_context: Pytest fixture providing ROS context
    
    Yields:
        MotorCoordinator: Active motor coordinator node
    """
    node = MotorCoordinator()
    yield node
    node.destroy_node()

# Coordinate transformation tests
@pytest.mark.parametrise("world_point,expected_grid", [
    ((0.15, 0.25), (1, 2)),  # Centre of grid cell
    ((0.35, 0.45), (3, 4)),  # Near edge of grid cell
    ((0.05, 0.05), (0, 0)),  # Corner case
])
def test_world_to_grid(pathfinder, world_point, expected_grid):
    """
    Test conversion from world coordinates to grid coordinates.
    
    Args:
        pathfinder: Pytest fixture providing PathFinder instance
        world_point (tuple): Input world coordinates (x, y)
        expected_grid (tuple): Expected grid coordinates (x, y)
    """
    point = Point()
    point.x, point.y = world_point
    grid_x, grid_y = pathfinder.world_to_grid(point)
    assert (grid_x, grid_y) == expected_grid

@pytest.mark.parametrise("grid_point,expected_world", [
    ((1, 2), (0.15, 0.25)),  # Standard case
    ((3, 4), (0.35, 0.45)),  # Edge of grid
    ((0, 0), (0.05, 0.05)),  # Origin case
])
def test_grid_to_world(pathfinder, grid_point, expected_world):
    """
    Test conversion from grid coordinates to world coordinates.
    
    Args:
        pathfinder: Pytest fixture providing PathFinder instance
        grid_point (tuple): Input grid coordinates (x, y)
        expected_world (tuple): Expected world coordinates (x, y)
    """
    world_point = pathfinder.grid_to_world(grid_point[0], grid_point[1])
    assert pytest.approx(world_point.x, 0.01) == expected_world[0]
    assert pytest.approx(world_point.y, 0.01) == expected_world[1]

# Path finding tests
@pytest.mark.parametrise("start,goal,expected_success", [
    ((0.05, 0.05), (0.45, 0.45), True),   # Valid path through free space
    ((0.05, 0.05), (0.15, 0.15), False),  # Path blocked by obstacle
    ((0.05, 0.05), (0.35, 0.35), True),   # Path requiring navigation around obstacle
])
def test_path_finding(pathfinder, start, goal, expected_success):
    """
    Test path finding functionality with various scenarios.
    
    Args:
        pathfinder: Pytest fixture providing PathFinder instance
        start (tuple): Starting position in world coordinates
        goal (tuple): Goal position in world coordinates
        expected_success (bool): Whether path finding should succeed
    """
    start_point = Point(x=start[0], y=start[1])
    goal_point = Point(x=goal[0], y=goal[1])
    
    path, steps = pathfinder.find_path(start_point, goal_point)
    
    if expected_success:
        assert isinstance(path, PoseArray)
        assert len(path.poses) > 0
    else:
        assert path is None

# Integration tests
def test_end_to_end_navigation(pathfinder, obstacle_detection, motor_coordinator):
    """
    Test complete navigation pipeline from path planning to execution.
    
    Args:
        pathfinder: Pytest fixture providing PathFinder instance
        obstacle_detection: Pytest fixture providing ObstacleDetection node
        motor_coordinator: Pytest fixture providing MotorCoordinator node
    """
    start = Point(x=0.1, y=0.1)
    goal = Point(x=0.9, y=0.9)
    
    # Test path planning
    path, steps = pathfinder.find_path(start, goal)
    assert path is not None
    
    # Test motor coordination response to path
    motor_coordinator.path_callback(path)
    
    # Test obstacle detection updates
    obstacle_detection.publish_grid()

def test_obstacle_avoidance(pathfinder, obstacle_detection):
    """
    Test system's ability to detect and avoid obstacles.
    
    Args:
        pathfinder: Pytest fixture providing PathFinder instance
        obstacle_detection: Pytest fixture providing ObstacleDetection node
    """
    start = Point(x=0.1, y=0.1)
    goal = Point(x=0.4, y=0.4)
    
    # Update environment with obstacle data
    obstacle_detection.publish_grid()
    
    # Verify path planning accounts for obstacles
    path, steps = pathfinder.find_path(start, goal)
    assert path is not None