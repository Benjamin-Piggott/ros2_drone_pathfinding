"""
ROS 2 node that coordinates motor commands based on the planned path.
Receives A* pathfinding results from the pathfinding_server and
converts them into individualised motor commands.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import math

class MotorCoordinator(Node):
    def __init__(self):
        super().__init__('motor_coordinator')
        
        # Subscribe to planned path from A* algorithm
        self.path_subscription = self.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )
        
        # Publishers for individualised motor commands
        self.fl_publisher = self.create_publisher(Float32, 'motor_fl_command', 10)
        self.fr_publisher = self.create_publisher(Float32, 'motor_fr_command', 10)
        self.rl_publisher = self.create_publisher(Float32, 'motor_rl_command', 10)
        self.rr_publisher = self.create_publisher(Float32, 'motor_rr_command', 10)
        
        self.get_logger().info('Motor coordinator is ready')

    def path_callback(self, path_msg):
        """Convert A* path points to individualised motor commands."""
        if not path_msg.poses:
            return
            
        for i in range(len(path_msg.poses) - 1):
            current = path_msg.poses[i].pose.position
            next_point = path_msg.poses[i + 1].pose.position
            
            # Calculate direction and movement
            dx = next_point.x - current.x
            dy = next_point.y - current.y
            
            # Calculate motor speeds and publish to individual motors
            self.publish_motor_commands(dx, dy)
            
    def publish_motor_commands(self, dx, dy):
        """Calculate and publish commands for each motor."""
        # Calculate individualised motor speeds
        fl_speed = Float32()
        fr_speed = Float32()
        rl_speed = Float32()
        rr_speed = Float32()
        
        # Simple differential drive calculations
        fl_speed.data = float(dx + dy)
        fr_speed.data = float(dx - dy)
        rl_speed.data = float(dx - dy)
        rr_speed.data = float(dx + dy)
        
        # Publish commands to each motor
        self.fl_publisher.publish(fl_speed)
        self.fr_publisher.publish(fr_speed)
        self.rl_publisher.publish(rl_speed)
        self.rr_publisher.publish(rr_speed)

def main(args=None):
    rclpy.init(args=args)
    node = MotorCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()