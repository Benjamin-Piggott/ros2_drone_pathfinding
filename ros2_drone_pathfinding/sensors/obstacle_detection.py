#!/usr/bin/env python3
"""
Obstacle Detection and Grid Mapping System

This module implements a ROS 2 node for real-time obstacle detection and
grid map maintenance. It maintains an occupancy grid representation of the
environment for use in pathfinding and navigation.

Key Features:
    - Real-time occupancy grid updates
    - Position-based obstacle detection
    - Configurable grid resolution
    - Best-effort QoS for sensor data
    - Simulated sensor range implementation

The system maintains a 2D grid representation of the environment, marking
cells as free or occupied based on sensor data and drone position.

Dependencies:
    - ROS 2 rclpy
    - nav_msgs for occupancy grid
    - geometry_msgs for position data
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ObstacleDetection(Node):
    def __init__(self):
        """
        Initialise the obstacle detection node with grid management systems.
        
        Sets up:
            - Position subscription with best-effort QoS
            - Occupancy grid publisher
            - Grid parameters and storage
            - Update timer for grid publishing
        """
        
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
        """
        Update grid based on current drone position and sensor data.
        
        Processes incoming position updates to maintain an accurate
        representation of obstacles in the environment.
        
        Args:
            msg: Current drone position as PoseStamped message
        
        Note:
            Current implementation simulates sensor data. In production,
            this would process actual sensor readings.
        """

        # In a real implementation, this would process actual sensor data
        x = int(msg.pose.position.x / self.grid_resolution)
        y = int(msg.pose.position.y / self.grid_resolution)
        
        # Update nearby cells (simulated sensor range)
        self.update_nearby_cells(x, y, 5)  # 5 cell radius
        
    def update_nearby_cells(self, centre_x, centre_y, radius):
        """
        Update grid cells within specified radius of current position.
        
        Simulates sensor readings by updating occupancy values for cells
        within the specified detection radius.
        
        Args:
            centre_x: Grid x-coordinate of current position
            centre_y: Grid y-coordinate of current position
            radius: Number of cells to update in each direction
            
        Note:
            Uses a simple radius-based update model. Production systems
            would use actual sensor data for more accurate updates.
        """

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
        """
        Publish current occupancy grid state.
        
        Creates and publishes an OccupancyGrid message containing:
            - Current timestamp
            - Map frame reference
            - Grid resolution and dimensions
            - Cell occupancy data
            
        Note:
            Published at 1Hz to balance update frequency with system load.
        """

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