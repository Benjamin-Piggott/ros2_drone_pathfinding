"""Launch file for the complete drone pathfinding system."""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_drone_pathfinding',
            executable='pathfinding_server',
            name='pathfinding_server',
            output='screen'
        ),
        Node(
            package='ros2_drone_pathfinding',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),
        Node(
            package='ros2_drone_pathfinding',
            executable='visualisation_bridge',
            name='visualisation_bridge',
            output='screen'
        )
    ])
