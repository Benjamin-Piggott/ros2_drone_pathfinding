#!/usr/bin/env python3
"""
ROS 2 node that provides pathfinding services for the drone system.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from .drone_interfaces.a_star import PathFinder
from .drone_interfaces.grid_utils import convert_occupancy_grid, generate_grid_string

class PathfindingServer(Node):
    def __init__(self):
        super().__init__('pathfinding_server')
        
        # Initialise the grid
        self.grid = [
            [1, 1, 1, 1, 1],
            [1, 0, 1, 1, 1],
            [1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1],
            [1, 1, 1, 1, 1]
        ]
        
        self.rows = len(self.grid)
        self.cols = len(self.grid[0])
        
        # Initialise pathfinder
        self.pathfinder = PathFinder(self.grid, self.rows, self.cols)
        
        # Create publisher for path visualisation
        self.path_publisher = self.create_publisher(
            Path,
            'planned_path',
            10
        )
        
        # Create service for pathfinding requests
        self.path_service = self.create_service(
            FindPath,
            'find_path',
            self.handle_find_path
        )
        
        self.get_logger().info('Pathfinding server is ready')

    def handle_find_path(self, request, response):
        """Handle incoming pathfinding requests."""
        start = (request.start.x, request.start.y)
        goal = (request.goal.x, request.goal.y)
        
        # Find path and get visualisation steps
        path_result = self.pathfinder.a_star_search(start, goal)
        
        if path_result[0]:
            path, steps = path_result
            # Convert path to ROS Path message
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for point in path:
                pose = PoseStamped()
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
            
            # Publish path for visualisation
            self.path_publisher.publish(path_msg)
            
            response.success = True
            response.path = path_msg
            response.visualisation_steps = steps
        else:
            response.success = False
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PathfindingServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
