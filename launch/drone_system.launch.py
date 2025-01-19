"""
Launch Configuration for Drone Pathfinding System

This launch file configures and initialises all necessary nodes for the complete
drone pathfinding system. It sets up the core pathfinding functionality, sensor
systems, motor controllers, and visualisation components.

System Components:
    Core Systems:
        - Pathfinding server with configurable grid parameters
        - Position sensing and obstacle detection
        - Motor control system with individual motor controllers
        - Visualisation bridge for system monitoring

Configuration Parameters:
    Pathfinding:
        - grid_size: 40 (grid dimensions)
        - cell_size: 0.25 metres (resolution)
    
    Obstacle Detection:
        - update_rate: 10.0 Hz
        - sensor_range: 5.0 metres
    
    Visualisation:
        - websocket_port: 8765 (monitoring interface)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Core system nodes
        Node(
            package='ros2_drone_pathfinding',
            executable='pathfinding_server',
            name='pathfinding_server',
            output='screen',
            parameters=[{
                'grid_size': 40,
                'cell_size': 0.25  # meters
            }]
        ),
        
        # Sensor nodes
        Node(
            package='ros2_drone_pathfinding',
            executable='position_sensor',
            name='position_sensor',
            output='screen'
        ),
        Node(
            package='ros2_drone_pathfinding',
            executable='obstacle_detection',
            name='obstacle_detection',
            output='screen',
            parameters=[{
                'update_rate': 10.0,  # Hz
                'sensor_range': 5.0   # meters
            }]
        ),
        
        # Motor control nodes
        Node(
            package='ros2_drone_pathfinding',
            executable='motor_coordinator',
            name='motor_coordinator',
            output='screen'
        ),
        Node(
            package='ros2_drone_pathfinding',
            executable='motor_controller_fl',
            name='front_left_motor',
            output='screen'
        ),
        Node(
            package='ros2_drone_pathfinding',
            executable='motor_controller_fr',
            name='front_right_motor',
            output='screen'
        ),
        Node(
            package='ros2_drone_pathfinding',
            executable='motor_controller_rl',
            name='rear_left_motor',
            output='screen'
        ),
        Node(
            package='ros2_drone_pathfinding',
            executable='motor_controller_rr',
            name='rear_right_motor',
            output='screen'
        ),
        
        # Visualisation bridge
        Node(
            package='ros2_drone_pathfinding',
            executable='visualisation_bridge',
            name='visualisation_bridge',
            output='screen',
            parameters=[{
                'websocket_port': 8765
            }]
        ),
    ])