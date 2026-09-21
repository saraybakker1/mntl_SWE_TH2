import json
import websockets


class CommandClient:
    def __init__(self, uri):
        self.uri = uri
        self.ws = None

    async def connect(self):
        self.ws = await websockets.connect(self.uri)

    async def send_wheel_velocity(self, v_left, v_right):
        message = {
            "v_left": float(v_left),
            "v_right": float(v_right),
        }
        print("message: ", message)
        await self.ws.send(json.dumps(message))

    async def close(self):
        if self.ws is not None:
            await self.ws.close()