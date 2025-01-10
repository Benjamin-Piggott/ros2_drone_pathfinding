#!/usr/bin/env python3
"""
ROS 2 node for obstacle detection and grid updates.
Updates the grid used by the A* pathfinding algorithm.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detection')
        
        # Create QoS profile for sensor data
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to drone position
        self.position_subscription = self.create_subscription(
            PoseStamped,
            'drone_position',
            self.position_callback,
            qos_profile
        )
        
        # Publisher for occupancy grid updates
        self.grid_publisher = self.create_publisher(
            OccupancyGrid,
            'obstacle_grid',
            10
        )
        
        # Initialise grid
        self.grid_width = 100  # cells
        self.grid_height = 100  # cells
        self.grid_resolution = 0.1  # metres per cell
        self.grid = [0] * (self.grid_width * self.grid_height)
        
        # Timer for grid updates
        self.create_timer(1.0, self.publish_grid)  # 1Hz updates
        
        self.get_logger().info('Obstacle detection is ready')
        
    def position_callback(self, msg):
        """Update grid based on current position and simulated sensor data."""
        # In a real implementation, this would process actual sensor data
        x = int(msg.pose.position.x / self.grid_resolution)
        y = int(msg.pose.position.y / self.grid_resolution)
        
        # Update nearby cells (simulated sensor range)
        self.update_nearby_cells(x, y, 5)  # 5 cell radius
        
    def update_nearby_cells(self, centre_x, centre_y, radius):
        """Update grid cells within radius of centre position."""
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                x = centre_x + dx
                y = centre_y + dy
                
                if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                    # Simulate obstacle detection
                    # In real implementation, this would use actual sensor data
                    cell_idx = y * self.grid_width + x
                    if abs(dx) == radius or abs(dy) == radius:
                        self.grid[cell_idx] = 100  # Occupied
                    
    def publish_grid(self):
        """Publish current occupancy grid."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.grid_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        
        msg.data = self.grid
        
        self.grid_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()