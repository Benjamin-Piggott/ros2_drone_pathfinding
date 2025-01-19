#!/usr/bin/env python3
"""
Drone Position Sensing System

This module implements a ROS 2 node for precise drone position tracking and
broadcasting. It provides real-time position data essential for pathfinding
and obstacle detection subsystems.

Key Features:
    - High-frequency position updates (10Hz)
    - Best-effort QoS profile for real-time performance
    - 3D position tracking (x, y, z coordinates)
    - ROS 2 time synchronisation
    - Map frame reference system

The system broadcasts position data using the geometry_msgs/PoseStamped message type,
allowing other nodes to track the drone's location in real-time.

Dependencies:
    - ROS 2 rclpy
    - geometry_msgs
    - rclpy.qos utilities
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PositionSensor(Node):
    def __init__(self):
        """
        Initialise the position sensor node with publishers and timer.
        
        Sets up:
            - Position publisher with best-effort QoS profile
            - 10Hz update timer for position broadcasting
            - Initial position tracking variables
        """

        super().__init__('position_sensor')
        
        # Create QoS profile for sensor data
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Publisher for current position
        self.position_publisher = self.create_publisher(
            PoseStamped,
            'drone_position',
            qos_profile
        )
        
        # Timer for position updates
        self.create_timer(0.1, self.publish_position)  # 10Hz updates
        
        # Initialise position tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.get_logger().info('Position sensor is ready')
        
    def publish_position(self):
        """
        Publish the current drone position.
        
        Creates and publishes a PoseStamped message containing:
            - Current timestamp
            - Map frame reference
            - 3D position coordinates (x, y, z)
            
        Note:
            This implementation currently simulates position data.
            In a production environment, this would interface with
            actual positioning hardware.
        """
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # In a real implementation, this would get data from actual sensors
        pose_msg.pose.position.x = self.current_x
        pose_msg.pose.position.y = self.current_y
        pose_msg.pose.position.z = self.current_z
        
        self.position_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionSensor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()