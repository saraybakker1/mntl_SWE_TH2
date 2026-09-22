import asyncio
import json
from dataclasses import dataclass
from datetime import datetime
import numpy as np

@dataclass
class RobotState:
    position: np.ndarray | None = None          # [x, y] [m]
    orientation: float | None = None        # theta [rad]: estimated
    velocity: np.ndarray | None = None          # [v_x, v_y]: estimated
    angular_velocity: float | None = None  # [angular velocity (gyro)] [rad/s]
    acceleration: np.ndarray | None = None      # [ax, ay] [m/s2]
    timestamp: str | None = None
    current_time: float | None = None       # timestamp - start-time


class SensorClient:
    """
    Read out sensor measurements:
    - Position from GPS
    - Angular velocity from gyro
    - Acceleration from accelerometer (local)
    """
    def __init__(self, uri):
        self.uri = uri
        self.state = RobotState()
        self._task = None
        self._theta = 0.0
        self._start_timestamp = None
        self._last_timestamp = None

    async def start(self, ws):
        self._task = asyncio.create_task(self.receive(ws))

    async def receive(self, ws):
        async for message in ws:
            self._update(json.loads(message))

    def _update(self, message):
        if message.get("message_type") != "sensors":
            return

        for sensor in message["sensors"]:

            name = sensor["name"]
            data = np.asarray(sensor["data"], dtype=float)
            timestamp = sensor["timestamp"]

            if name == "gps":
                self.state.position = data

            elif name == "accelerometer":
                self.state.acceleration = data

            elif name == "gyro":
                self.state.angular_velocity = float(data[0])

            if self._start_timestamp is None:
                self._start_timestamp = datetime.fromisoformat(timestamp)

            self.state.timestamp = timestamp
            self._last_timestamp = timestamp
            timestamp_sec = datetime.fromisoformat(timestamp)
            self.state.current_time = (timestamp_sec - self._start_timestamp).total_seconds()


    def get_state(self):
        return self.state

    async def stop(self):
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None