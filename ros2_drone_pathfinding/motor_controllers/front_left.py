"""
ROS 2 node for controlling the front left motor.
Receives commands derived from A* pathfinding results.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class FrontLeftMotorController(Node):
    def __init__(self):
        super().__init__('front_left_motor')
        
        # Subscribe to motor commands
        self.command_subscription = self.create_subscription(
            Float32,
            'motor_fl_command',
            self.command_callback,
            10
        )
        
        # Publisher for motor status
        self.status_publisher = self.create_publisher(
            Float32,
            'motor_fl_status',
            10
        )
        
        self.get_logger().info('Front left motor controller is ready')
        
    def command_callback(self, msg):
        """Handle incoming motor commands."""
        # Here you would implement actual motor control logic
        # For simulation, we'll simply echo the command as status
        self.status_publisher.publish(msg)
        self.get_logger().debug(f'Front left motor speed: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = FrontLeftMotorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
