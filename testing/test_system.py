import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseArray
from ros2_drone_pathfinding.drone_interfaces.a_star import PathFinder
from ros2_drone_pathfinding.motor_coordinator import MotorCoordinator
from ros2_drone_pathfinding.sensors.obstacle_detection import ObstacleDetection

# Fixtures for common test setup
@pytest.fixture(scope="session")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def test_grid():
    grid_msg = OccupancyGrid()
    grid_msg.info.width = 5
    grid_msg.info.height = 5
    grid_msg.info.resolution = 0.1
    grid_msg.data = [
        0, 0, 0, 0, 0,
        0, 100, 0, 100, 0,
        0, 0, 0, 0, 0,
        0, 100, 0, 100, 0,
        0, 0, 0, 0, 0
    ]
    return grid_msg

@pytest.fixture
def pathfinder(test_grid):
    return PathFinder(test_grid)

@pytest.fixture
def obstacle_detection(ros_context):
    node = ObstacleDetection()
    yield node
    node.destroy_node()

@pytest.fixture
def motor_coordinator(ros_context):
    node = MotorCoordinator()
    yield node
    node.destroy_node()

# Pathfinding tests
@pytest.mark.parametrise("world_point,expected_grid", [
    ((0.15, 0.25), (1, 2)),
    ((0.35, 0.45), (3, 4)),
    ((0.05, 0.05), (0, 0)),
])
def test_world_to_grid(pathfinder, world_point, expected_grid):
    point = Point()
    point.x, point.y = world_point
    grid_x, grid_y = pathfinder.world_to_grid(point)
    assert (grid_x, grid_y) == expected_grid

@pytest.mark.parametrise("grid_point,expected_world", [
    ((1, 2), (0.15, 0.25)),
    ((3, 4), (0.35, 0.45)),
    ((0, 0), (0.05, 0.05)),
])
def test_grid_to_world(pathfinder, grid_point, expected_world):
    world_point = pathfinder.grid_to_world(grid_point[0], grid_point[1])
    assert pytest.approx(world_point.x, 0.01) == expected_world[0]
    assert pytest.approx(world_point.y, 0.01) == expected_world[1]

@pytest.mark.parametrise("start,goal,expected_success", [
    ((0.05, 0.05), (0.45, 0.45), True),   # Valid path
    ((0.05, 0.05), (0.15, 0.15), False),  # Blocked path
    ((0.05, 0.05), (0.35, 0.35), True),   # Alternative path
])
def test_path_finding(pathfinder, start, goal, expected_success):
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
    # Test complete navigation pipeline
    start = Point(x=0.1, y=0.1)
    goal = Point(x=0.9, y=0.9)
    
    # Find path
    path, steps = pathfinder.find_path(start, goal)
    assert path is not None
    
    # Test motor coordination
    motor_coordinator.path_callback(path)
    # Add specific motor command assertions here
    
    # Test obstacle detection
    obstacle_detection.publish_grid()
    # Add specific grid update assertions here

def test_obstacle_avoidance(pathfinder, obstacle_detection):
    # Initialise with known obstacles
    start = Point(x=0.1, y=0.1)
    goal = Point(x=0.4, y=0.4)
    
    # Simulate obstacle detection
    obstacle_detection.publish_grid()
    
    # Verify path avoids obstacles
    path, steps = pathfinder.find_path(start, goal)
    assert path is not None
    # Add assertions to verify path avoids known obstacles