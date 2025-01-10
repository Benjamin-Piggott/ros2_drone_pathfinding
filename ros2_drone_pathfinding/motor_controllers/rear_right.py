"""
ROS 2 node for controlling the rear right motor.
Receives commands derived from A* pathfinding results.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class RearRightMotorController(Node):
    def __init__(self):
        super().__init__('rear_right_motor')
        
        self.command_subscription = self.create_subscription(
            Float32,
            'motor_rr_command',
            self.command_callback,
            10
        )
        
        self.status_publisher = self.create_publisher(
            Float32,
            'motor_rr_status',
            10
        )
        
        self.get_logger().info('Rear right motor controller is ready')
        
    def command_callback(self, msg):
        """Handle incoming motor commands."""
        self.status_publisher.publish(msg)
        self.get_logger().debug(f'Rear right motor speed: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RearRightMotorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()