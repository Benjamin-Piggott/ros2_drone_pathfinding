#!/usr/bin/env python3
"""
ROS 2 node that bridges between ROS topics and WebSocket for visualisation.
Combines position tracking and path visualisation capabilities.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import json
import asyncio
import time
from math import atan2, degrees
from .websocket.websocket_server import WebSocketServer

class VisualisationBridge(Node):
    def __init__(self):
        super().__init__('visualisation_bridge')
        
        # Load configuration parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 30.0),
                ('position_history_size', 100)
            ]
        )
        
        # Initialise position tracking
        self.position_history = []
        self.max_history = self.get_parameter('position_history_size').value
        self.current_path = None
        
        # Subscribe to position updates
        self.position_subscription = self.create_subscription(
            PoseStamped,
            'drone_position',
            self.position_callback,
            10
        )
        
        # Subscribe to planned path
        self.path_subscription = self.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )
        
        # Initialise WebSocket server
        self.websocket_server = WebSocketServer()
        
        # Create timer for regular updates
        update_rate = self.get_parameter('update_rate').value
        self.create_timer(1.0/update_rate, self.publish_visualisation)
        
        self.get_logger().info('Enhanced visualisation bridge is ready')
    
    def position_callback(self, msg: PoseStamped):
        """Handle incoming position updates with trajectory tracking."""
        position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'timestamp': time.time()
        }
        
        # Calculate orientation
        if len(self.position_history) > 0:
            last_pos = self.position_history[-1]
            dx = position['x'] - last_pos['x']
            dy = position['y'] - last_pos['y']
            position['heading'] = degrees(atan2(dy, dx))
        else:
            position['heading'] = 0.0
        
        # Update position history
        self.position_history.append(position)
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
    
    def path_callback(self, msg: Path):
        """Handle incoming path messages."""
        self.current_path = msg
    
    def calculate_velocity(self):
        """Calculate current velocity from position history."""
        if len(self.position_history) < 2:
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        p1 = self.position_history[-2]
        p2 = self.position_history[-1]
        dt = p2['timestamp'] - p1['timestamp']
        
        return {
            'x': (p2['x'] - p1['x']) / dt,
            'y': (p2['y'] - p1['y']) / dt,
            'z': (p2['z'] - p1['z']) / dt
        }
    
    def calculate_acceleration(self):
        """Calculate current acceleration from velocity history."""
        if len(self.position_history) < 3:
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        v1 = self.calculate_velocity()  # Current velocity
        
        # Calculate previous velocity
        p1 = self.position_history[-3]
        p2 = self.position_history[-2]
        dt1 = p2['timestamp'] - p1['timestamp']
        prev_velocity = {
            'x': (p2['x'] - p1['x']) / dt1,
            'y': (p2['y'] - p1['y']) / dt1,
            'z': (p2['z'] - p1['z']) / dt1
        }
        
        dt = self.position_history[-1]['timestamp'] - p2['timestamp']
        
        return {
            'x': (v1['x'] - prev_velocity['x']) / dt,
            'y': (v1['y'] - prev_velocity['y']) / dt,
            'z': (v1['z'] - prev_velocity['z']) / dt
        }
    
    async def publish_visualisation(self):
        """Publish comprehensive visualisation data through WebSocket."""
        if not self.position_history:
            return
        
        # Create visualisation data structure
        vis_data = {
            'type': 'drone_status',
            'current_position': self.position_history[-1],
            'trajectory': self.position_history,
            'planned_path': None if not self.current_path else [
                {'x': pose.pose.position.x, 
                 'y': pose.pose.position.y}
                for pose in self.current_path.poses
            ],
            'timestamp': time.time()
        }
        
        # Calculate additional metrics
        if len(self.position_history) > 1:
            vis_data['velocity'] = self.calculate_velocity()
            vis_data['acceleration'] = self.calculate_acceleration()
        
        # Send to WebSocket clients
        await self.websocket_server.broadcast(json.dumps(vis_data))

def main(args=None):
    rclpy.init(args=args)
    node = VisualisationBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()