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
    - WebSocket visualisation
    - Motor controller responses
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose
from std_msgs.msg import Float32
from ros2_drone_pathfinding.drone_interfaces.a_star import PathFinder
from ros2_drone_pathfinding.motor_coordinator import MotorCoordinator
from ros2_drone_pathfinding.sensors.obstacle_detection import ObstacleDetection
import asyncio
import websockets
import json
import time

class TestNode(Node):
    """Helper node for testing ROS 2 communications."""
    def __init__(self):
        super().__init__('test_node')
        self.received_messages = []
        
    def create_test_subscription(self, msg_type, topic, queue_size=10):
        """Create a subscription that stores received messages."""
        return self.create_subscription(
            msg_type,
            topic,
            lambda msg: self.received_messages.append(msg),
            queue_size
        )

@pytest.fixture(scope="session")
def ros_context():
    """Initialise and tear down ROS context for the test session."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def executor():
    """Provide a single-threaded executor for running nodes."""
    executor = SingleThreadedExecutor()
    yield executor
    executor.shutdown()

@pytest.fixture
def test_grid():
    """Create a sample occupancy grid for testing."""
    grid_msg = OccupancyGrid()
    grid_msg.header.frame_id = "map"
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
    """Create a PathFinder instance with the test grid."""
    return PathFinder(test_grid)

@pytest.fixture
def test_node(ros_context):
    """Create and clean up test node instance."""
    node = TestNode()
    yield node
    node.destroy_node()

@pytest.fixture
def obstacle_detection(ros_context, executor):
    """Create and clean up ObstacleDetection node instance."""
    node = ObstacleDetection()
    executor.add_node(node)
    yield node
    executor.remove_node(node)
    node.destroy_node()

@pytest.fixture
def motor_coordinator(ros_context, executor):
    """Create and clean up MotorCoordinator node instance."""
    node = MotorCoordinator()
    executor.add_node(node)
    yield node
    executor.remove_node(node)
    node.destroy_node()

# Coordinate transformation tests
@pytest.mark.parametrize("world_point,expected_grid", [
    ((0.15, 0.25), (1, 2)),  # Centre of grid cell
    ((0.35, 0.45), (3, 4)),  # Near edge of grid cell
    ((0.05, 0.05), (0, 0)),  # Corner case
])
def test_world_to_grid(pathfinder, world_point, expected_grid):
    """Test conversion from world coordinates to grid coordinates."""
    point = Point()
    point.x, point.y = world_point
    grid_x, grid_y = pathfinder.world_to_grid(point)
    assert (grid_x, grid_y) == expected_grid

@pytest.mark.parametrize("grid_point,expected_world", [
    ((1, 2), (0.15, 0.25)),  # Standard case
    ((3, 4), (0.35, 0.45)),  # Edge of grid
    ((0, 0), (0.05, 0.05)),  # Origin case
])
def test_grid_to_world(pathfinder, grid_point, expected_world):
    """Test conversion from grid coordinates to world coordinates."""
    world_point = pathfinder.grid_to_world(grid_point[0], grid_point[1])
    assert pytest.approx(world_point.x, 0.01) == expected_world[0]
    assert pytest.approx(world_point.y, 0.01) == expected_world[1]

# Path finding tests
@pytest.mark.parametrize("start,goal,expected_success", [
    ((0.05, 0.05), (0.45, 0.45), True),   # Valid path through free space
    ((0.05, 0.05), (0.15, 0.15), False),  # Path blocked by obstacle
    ((0.05, 0.05), (0.35, 0.35), True),   # Path requiring navigation around obstacle
])
def test_path_finding(pathfinder, start, goal, expected_success):
    """Test path finding functionality with various scenarios."""
    start_point = Point()
    start_point.x, start_point.y = start
    goal_point = Point()
    goal_point.x, goal_point.y = goal
    
    path, steps = pathfinder.find_path(start_point, goal_point)
    
    if expected_success:
        assert path is not None
        assert len(path.poses) > 0
    else:
        assert path is None

# Motor controller tests
@pytest.mark.asyncio
async def test_motor_responses(ros_context, executor, test_node):
    """Test motor controller responses to commands."""
    # Set up subscriptions for all motor status topics
    motors = ['fl', 'fr', 'rl', 'rr']
    subscribers = {
        motor: test_node.create_test_subscription(
            Float32,
            f'motor_{motor}_status'
        ) for motor in motors
    }
    
    # Create publishers for motor commands
    publishers = {
        motor: test_node.create_publisher(
            Float32,
            f'motor_{motor}_command',
            10
        ) for motor in motors
    }
    
    executor.add_node(test_node)
    
    # Test each motor
    for motor in motors:
        # Send test command
        msg = Float32()
        msg.data = 0.5
        publishers[motor].publish(msg)
        
        # Allow time for processing
        await asyncio.sleep(0.1)
        executor.spin_once()
        
        # Verify response
        assert len(test_node.received_messages) > 0
        latest_msg = test_node.received_messages[-1]
        assert pytest.approx(latest_msg.data, 0.01) == 0.5

# WebSocket visualisation tests
@pytest.mark.asyncio
async def test_websocket_visualisation():
    """Test WebSocket server functionality for visualisation."""
    # Connect to WebSocket server
    async with websockets.connect('ws://localhost:8765') as websocket:
        # Create test path data
        path_data = {
            'type': 'path_update',
            'path': [
                {'x': 0.0, 'y': 0.0},
                {'x': 1.0, 'y': 1.0}
            ]
        }
        
        # Send test data
        await websocket.send(json.dumps(path_data))
        
        # Verify response
        response = await websocket.recv()
        assert response is not None

# Integration tests
def test_end_to_end_navigation(pathfinder, obstacle_detection, motor_coordinator, executor):
    """Test complete navigation pipeline from path planning to execution."""
    # Create test point
    start = Point()
    start.x, start.y = 0.1, 0.1
    goal = Point()
    goal.x, goal.y = 0.9, 0.9
    
    # Test path planning
    path, steps = pathfinder.find_path(start, goal)
    assert path is not None
    
    # Test motor coordination response to path
    motor_coordinator.path_callback(path)
    executor.spin_once()
    
    # Test obstacle detection updates
    obstacle_detection.publish_grid()
    executor.spin_once()

@pytest.mark.asyncio
async def test_obstacle_avoidance(pathfinder, obstacle_detection, executor, test_node):
    """Test system's ability to detect and avoid obstacles."""
    # Subscribe to obstacle grid updates
    grid_sub = test_node.create_test_subscription(
        OccupancyGrid,
        'obstacle_grid'
    )
    
    executor.add_node(test_node)
    
    # Create test points
    start = Point()
    start.x, start.y = 0.1, 0.1
    goal = Point()
    goal.x, goal.y = 0.4, 0.4
    
    # Update environment with obstacle data
    obstacle_detection.publish_grid()
    await asyncio.sleep(0.1)
    executor.spin_once()
    
    # Verify grid update
    assert len(test_node.received_messages) > 0
    latest_grid = test_node.received_messages[-1]
    assert isinstance(latest_grid, OccupancyGrid)
    
    # Verify path planning accounts for obstacles
    path, steps = pathfinder.find_path(start, goal)
    assert path is not None
    
    # Verify path avoids obstacles
    for pose in path.poses:
        grid_x, grid_y = pathfinder.world_to_grid(pose.pose.position)
        assert latest_grid.data[grid_y * latest_grid.info.width + grid_x] < 50

if __name__ == '__main__':
    pytest.main(['-v', __file__])