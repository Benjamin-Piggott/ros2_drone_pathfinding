#!/usr/bin/env python3
"""
ROS 2 node for drone position sensing.
Provides position data for A* pathfinding and obstacle detection.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PositionSensor(Node):
    def __init__(self):
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
        """Publish current position (simulated for now)."""
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