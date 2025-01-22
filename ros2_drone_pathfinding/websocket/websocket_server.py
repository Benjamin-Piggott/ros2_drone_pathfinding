"""WebSocket server with robust error handling and reconnection logic."""

import asyncio
import websockets
import json
from websockets.exceptions import ConnectionClosed
import logging


class EnhancedWebSocketServer:
    def __init__(self, host: str = 'localhost', port: int = 8765):
        """Initialise the WebSocket server with enhanced error handling.

        Args:
            host: Server host address
            port: Server port number
        """
        self.host = host
        self.port = port
        self.clients = set()  # Store active WebSocket connections
        self.logger = logging.getLogger(__name__)

        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        )

    async def register(self, websocket):
        """Register a new client connection with error handling."""
        self.clients.add(websocket)
        self.logger.info(f"New client connected. Total clients: {len(self.clients)}")
        try:
            async for _ in websocket:  # Keep the connection alive
                pass
        except ConnectionClosed as e:
            self.logger.warning(f"Client connection closed: {e}")
        finally:
            self.clients.remove(websocket)
            self.logger.info(f"Client disconnected. Remaining clients: {len(self.clients)}")

    async def broadcast(self, message: str):
        """Broadcast message to all connected clients with error handling.

        Args:
            message: JSON-formatted string to broadcast
        """
        if not self.clients:
            return

        disconnected_clients = set()

        for client in self.clients:
            try:
                await client.send(message)
            except ConnectionClosed:
                disconnected_clients.add(client)
            except Exception as e:
                self.logger.error(f"Error broadcasting to client: {e}")
                disconnected_clients.add(client)

        # Clean up disconnected clients
        self.clients -= disconnected_clients

    async def start_server(self):
        """Start the WebSocket server with error handling and reconnection."""
        async def handler(websocket, _):
            await self.register(websocket)

        self.logger.info(f"Starting WebSocket server on ws://{self.host}:{self.port}")
        async with websockets.serve(handler, self.host, self.port):
            await asyncio.Future()  # Run forever

    def run(self):
        """Run the server in the current thread with error handling."""
        try:
            asyncio.run(self.start_server())
        except KeyboardInterrupt:
            self.logger.info("Server shutdown requested")
        except Exception as e:
            self.logger.error(f"Fatal server error: {e}")
            raise
