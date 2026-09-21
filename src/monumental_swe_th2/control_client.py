import json
import websockets

class CommandClient:
    def __init__(self, uri):
        self.uri = uri
        self.ws = None

    async def connect(self):
        self.ws = await websockets.connect(self.uri)

        # Read the first message sent by the server
        message = await self.ws.recv()
        print("received:", message)

    async def send_wheel_velocity(self, ws, v_left, v_right):
        message = {
            "v_left": float(v_left),
            "v_right": float(v_right),
        }

        await ws.send(json.dumps(message))

    async def close(self):
        if self.ws is not None:
            await self.ws.close()