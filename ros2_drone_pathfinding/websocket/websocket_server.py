#!/usr/bin/env python3
"""WebSocket server for drone visualisation."""

import asyncio
import websockets
import json
from typing import Set, Dict, Any

class WebSocketServer:
    def __init__(self, host: str = 'localhost', port: int = 8765):
        """Initialize the WebSocket server.
        
        Args:
            host: Server host address
            port: Server port number
        """
        self.host = host
        self.port = port
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.server = None
    
    async def register(self, websocket: websockets.WebSocketServerProtocol):
        """Register a new client connection."""
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.remove(websocket)
    
    async def broadcast(self, message: str):
        """Broadcast message to all connected clients.
        
        Args:
            message: JSON-formatted string to broadcast
        """
        if self.clients:
            await asyncio.gather(
                *[client.send(message) for client in self.clients]
            )
    
    async def start_server(self):
        """Start the WebSocket server."""
        async with websockets.serve(self.register, self.host, self.port):
            await asyncio.Future()  # run forever
    
    def run(self):
        """Run the server in the current thread."""
        asyncio.run(self.start_server())

# Example usage
if __name__ == "__main__":
    server = WebSocketServer()
    server.run()