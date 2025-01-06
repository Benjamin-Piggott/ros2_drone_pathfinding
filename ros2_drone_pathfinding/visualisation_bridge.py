#!/usr/bin/env python3
"""
ROS 2 node that bridges between ROS topics and WebSocket for visualisation.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import json
import asyncio
import websockets
from .websocket.websocket_server import WebSocketServer

class VisualisationBridge(Node):
    def __init__(self):
        super().__init__('visualisation_bridge')
        
        # Subscribe to path visualisation
        self.path_subscription = self.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )
        
        # Initialise WebSocket server
        self.websocket_server = WebSocketServer()
        
        self.get_logger().info('Visualisation bridge is ready')
        
    def path_callback(self, path_msg):
        """Handle incoming path messages and forward to WebSocket."""
        # Convert path message to visualisation format
        path_data = {
            'type': 'path_update',
            'path': [
                {
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y
                }
                for pose in path_msg.poses
            ]
        }
        
        # Send to WebSocket clients
        asyncio.run(self.websocket_server.broadcast(json.dumps(path_data)))

def main(args=None):
    rclpy.init(args=args)
    node = VisualisationBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
