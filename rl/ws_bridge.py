"""
rl/ws_bridge.py — WebSocket bridge between Python PPO trainer and browser simulator

Starts a WebSocket server that connects to the browser's RL environment.
Protocol:
  Server → Browser: { type: 'reset', seed: N } or { type: 'step', action: {...} }
  Browser → Server: { type: 'obs', observation: [...], reward: N, done: bool, ... }
"""

import asyncio
import json
import numpy as np
import websockets


class SimulatorBridge:
    """WebSocket bridge to the browser-based V2V simulator."""

    def __init__(self, host='localhost', port=8765):
        self.host = host
        self.port = port
        self.ws = None
        self.response_queue = asyncio.Queue()
        self.connected = asyncio.Event()
        self.env_info = None

    async def start_server(self):
        """Start WebSocket server and wait for browser connection."""
        self.server = await websockets.serve(
            self._handler, self.host, self.port
        )
        print(f'[Bridge] WebSocket server started on ws://{self.host}:{self.port}')
        print(f'[Bridge] Waiting for browser connection...')
        await self.connected.wait()
        print(f'[Bridge] Browser connected!')

    async def _handler(self, websocket, path=None):
        """Handle incoming WebSocket connection."""
        self.ws = websocket
        self.connected.set()
        try:
            async for message in websocket:
                data = json.loads(message)
                if data.get('type') == 'env_info':
                    self.env_info = data
                    print(f'[Bridge] Env info: obs={data.get("observationSize")}, '
                          f'act={data.get("actionSize")}')
                else:
                    await self.response_queue.put(data)
        except websockets.exceptions.ConnectionClosed:
            print('[Bridge] Browser disconnected')
            self.connected.clear()
            self.ws = None

    async def reset(self, seed=None):
        """Send reset command and wait for initial observation."""
        msg = {'type': 'reset'}
        if seed is not None:
            msg['seed'] = int(seed)
        await self.ws.send(json.dumps(msg))
        response = await self.response_queue.get()
        return np.array(response['observation'], dtype=np.float32)

    async def step(self, action):
        """Send action and wait for step result."""
        msg = {
            'type': 'step',
            'action': {
                'steering': float(action[0]),
                'throttle': float(action[1]),
                'brake': float(action[2])
            }
        }
        await self.ws.send(json.dumps(msg))
        response = await self.response_queue.get()
        return (
            np.array(response['observation'], dtype=np.float32),
            float(response['reward']),
            bool(response['done']),
            bool(response['truncated']),
            response.get('info', {})
        )

    async def set_curriculum(self, stage):
        """Set curriculum stage."""
        await self.ws.send(json.dumps({
            'type': 'set_curriculum',
            'stage': int(stage)
        }))
        await self.response_queue.get()  # wait for ack

    async def set_v2v(self, enabled):
        """Enable/disable V2V for comparison experiments."""
        await self.ws.send(json.dumps({
            'type': 'set_v2v',
            'enabled': bool(enabled)
        }))
        await self.response_queue.get()  # wait for ack

    async def get_metrics(self):
        """Get episode metrics from simulator."""
        await self.ws.send(json.dumps({'type': 'get_metrics'}))
        response = await self.response_queue.get()
        return response.get('data', {})

    async def close(self):
        """Close the server."""
        if self.server:
            self.server.close()
            await self.server.wait_closed()
