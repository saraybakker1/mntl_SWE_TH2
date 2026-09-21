from monumental_swe_th2.sensor_client import RobotState
import numpy as np
from collections import deque

class StateEstimator:
    def __init__(self, max_vel=2.0, max_acc=1.0, filter_alpha = 0.7, filter_window=100):
        self._last_position = None
        self._last_timestamp = None
        self._max_vel = max_vel
        self._max_acc = max_acc
        self._filter_alpha = filter_alpha
        self._velocity_history = deque(maxlen=filter_window)

    def world_to_robot(self, value, theta):
        # value can be velocity or acceleration (in world frame)
        ax, ay = value

        c = np.cos(theta)
        s = np.sin(theta)

        ax_robot = c * ax + s * ay
        ay_robot = -s * ax + c * ay

        return np.array([ax_robot, ay_robot])

    def warnings_limits(self, velocity, acceleration, orientation):
        velocity_wheels = self.world_to_robot(velocity, orientation)

        # warnings when outside velocity margin:
        velocity_valid = np.all(
            (velocity_wheels >= -self._max_vel) &
            (velocity_wheels <= self._max_vel)
        )
        if velocity_valid is False:
            # todo: make proper warning and logging, also don't log absolute, but wheel velocity:
            print("Warning: velocity above threshold, current velocity (x, y):", velocity, ", threshold: ", [-self._max_vel, self._max_vel])

        if acceleration is not None:
            acceleration_wheels = self.world_to_robot(acceleration, orientation)
            acceleration_valid = np.all(
                (acceleration_wheels >= -self._max_acc) &
                (acceleration_wheels <= self._max_acc)
            )

            # warnings when outside velocity margin:
            if acceleration_valid is False:
                # todo: make proper warning and logging, also don't log absolute, but wheel acceleration:
                print("Warning: velocity above threshold, current velocity (x, y):", acceleration_wheels, ", threshold: ",
                      [-self._max_acc, self._max_acc])

    def update(self, state: RobotState):
        if state.position is None or state.timestamp is None:
            return state

        position = state.position
        acceleration = state.acceleration
        timestamp = np.datetime64(state.timestamp)

        # first step:
        if self._last_position is None:
            self._last_position = position.copy()
            self._last_timestamp = timestamp
            return state

        dt = (
            timestamp - self._last_timestamp
        ) / np.timedelta64(1, "s")

        if dt <= 0:
            return state

        # Raw velocity estimate
        velocity_raw = (position - self._last_position) / dt

        # Store raw velocity
        self._velocity_history.append(velocity_raw)

        # Exponential smoothing
        if state.velocity is None:
            velocity = velocity_raw
        else:
            velocity = (
                self._filter_alpha * velocity_raw
                + (1.0 - self._filter_alpha) * state.velocity
            )

        state.velocity = velocity

        # Estimate orientation from direction of motion
        state.orientation = np.arctan2(
                velocity[1],
                velocity[0],
            )

        state.timestamp = str(timestamp)

        self._last_position = position.copy()
        self._last_timestamp = timestamp

        # checks for velocity and acceleration over the limits:
        self.warnings_limits(velocity, acceleration, state.orientation)

        return state