#!/usr/bin/env python3
"""
ROS 2 node that provides pathfinding services for the drone system.

This server implements A* pathfinding algorithm to find optimal paths for drone navigation
whilst avoiding obstacles. It maintains a grid representation of the environment and
provides both service-based pathfinding and path visualisation capabilities.

Key Features:
    - A* pathfinding implementation
    - Grid-based environment representation
    - Path visualisation via ROS topics
    - Service-based path requests
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from .drone_interfaces.a_star import PathFinder
from .drone_interfaces.grid_utils import convert_occupancy_grid, generate_grid_string

class PathfindingServer(Node):
    """
    A ROS 2 node that handles pathfinding requests for drone navigation.
    
    Attributes:
        grid (List[List[int]]): 2D grid representation of the environment where:
            - 0 represents obstacles
            - 1 represents free space
        rows (int): Number of rows in the grid
        cols (int): Number of columns in the grid
        pathfinder (PathFinder): Instance of A* pathfinding algorithm
        path_publisher (Publisher): Publishes visualisation of planned paths
        path_service (Service): Handles incoming pathfinding requests
    """
    
    def __init__(self):
        """Initialise the pathfinding server with grid setup and ROS interfaces."""
        super().__init__('pathfinding_server')
        
        # Initialise the environment grid
        # Note: This is a sample grid - in production, this should be loaded from
        # configuration or generated from sensor data
        self.grid = [
            [1, 1, 1, 1, 1],  # 1 represents traversable space
            [1, 0, 1, 1, 1],  # 0 represents obstacles
            [1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1],
            [1, 1, 1, 1, 1]
        ]
        
        # Store grid dimensions for easy access
        self.rows = len(self.grid)
        self.cols = len(self.grid[0])
        
        # Create pathfinder instance with current grid configuration
        self.pathfinder = PathFinder(self.grid, self.rows, self.cols)
        
        # Set up ROS publisher for path visualisation
        self.path_publisher = self.create_publisher(
            Path,
            'planned_path',  # Topic name for visualisation
            10  # Queue size
        )
        
        # Set up ROS service for handling pathfinding requests
        self.path_service = self.create_service(
            FindPath,  # Service message type
            'find_path',  # Service name
            self.handle_find_path  # Callback function
        )
        
        self.get_logger().info('Pathfinding server is ready')

    def handle_find_path(self, request, response):
        """
        Process incoming pathfinding service requests.
        
        Args:
            request: Service request containing start and goal positions
            response: Service response object to be filled
        
        Returns:
            response: Filled service response containing:
                - success: Boolean indicating if path was found
                - path: ROS Path message if successful
                - visualisation_steps: List of steps taken during pathfinding
        """
        # Extract start and goal positions from request
        start = (request.start.x, request.start.y)
        goal = (request.goal.x, request.goal.y)
        
        # Attempt to find path and get visualisation steps
        path_result = self.pathfinder.a_star_search(start, goal)
        
        if path_result[0]:  # If path found
            path, steps = path_result
            # Create ROS Path message for visualisation
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Convert path points to PoseStamped messages
            for point in path:
                pose = PoseStamped()
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = 0.0  # Assuming 2D planning
                path_msg.poses.append(pose)
            
            # Publish path for visualisation purposes
            self.path_publisher.publish(path_msg)
            
            # Fill response with success data
            response.success = True
            response.path = path_msg
            response.visualisation_steps = steps
        else:  # If no path found
            response.success = False
            
        return response

def main(args=None):
    """Main function to initialise and run the pathfinding server."""
    rclpy.init(args=args)
    node = PathfindingServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()