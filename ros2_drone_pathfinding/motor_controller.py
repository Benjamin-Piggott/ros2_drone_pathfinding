#!/usr/bin/env python3
"""
ROS 2 node that controls individual drone motors based on the planned path.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Subscribe to planned path
        self.path_subscription = self.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )
        
        # Publisher for motor commands
        self.motor_publisher = self.create_publisher(
            Float32MultiArray,
            'motor_commands',
            10
        )
        
        self.get_logger().info('Motor controller is ready')

    def path_callback(self, path_msg):
        """Convert path points to motor commands."""
        if not path_msg.poses:
            return
            
        for i in range(len(path_msg.poses) - 1):
            current = path_msg.poses[i].pose.position
            next_point = path_msg.poses[i + 1].pose.position
            
            # Calculate direction and movement
            dx = next_point.x - current.x
            dy = next_point.y - current.y
            
            # Convert movement to motor commands (simplified example)
            motor_speeds = self.calculate_motor_speeds(dx, dy)
            
            # Publish motor commands
            msg = Float32MultiArray()
            msg.data = motor_speeds
            self.motor_publisher.publish(msg)
            
    def calculate_motor_speeds(self, dx, dy):
        """Calculate individual motor speeds based on desired movement."""
        # Simplified motor control logic
        # In real implementation, this would include proper drone dynamics
        front_left = float(dx + dy)
        front_right = float(dx - dy)
        rear_left = float(dx - dy)
        rear_right = float(dx + dy)
        
        return [front_left, front_right, rear_left, rear_right]

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
