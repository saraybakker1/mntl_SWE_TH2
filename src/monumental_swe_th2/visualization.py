import matplotlib.pyplot as plt
import numpy as np
from collections import deque

from monumental_swe_th2.sensor_client import RobotState
from monumental_swe_th2.path import Path_LoG
import matplotlib
matplotlib.use("QtAgg")
print(matplotlib.get_backend())

class StatePlotter:
    def __init__(self, max_points=500, path_resolution=2000,):
        self.x = deque(maxlen=max_points)
        self.y = deque(maxlen=max_points)
        self.orientation = deque(maxlen=max_points)

        self.velocity_time = deque(maxlen=max_points)
        self.speed = deque(maxlen=max_points)

        self.acceleration_time = deque(maxlen=max_points)
        self.acceleration = deque(maxlen=max_points)

        self.start_time = None

        plt.ion()

        self.fig, self.axes = plt.subplots(
            1, 4,
            figsize=(12, 4),
        )

        self.position_ax = self.axes[0]
        self.orientation_ax = self.axes[1]
        self.velocity_ax = self.axes[2]
        self.acceleration_ax = self.axes[3]

        self.path_times = np.linspace(0.0, 20.0, path_resolution)
        path = Path_LoG(0)
        path_positions = np.array([
            path.get(t)[0]
            for t in self.path_times
        ])
        # Reference path
        self.path_x = path_positions[:, 0]
        self.path_y = path_positions[:, 1]

    def update(self, state: RobotState, target: np.ndarray):
        if state.position is None:
            return

        if target is None:
            return

        self.target = target

        # Position
        self.x.append(state.position[0])
        self.y.append(state.position[1])

        # orientation:
        self.orientation.append(state.orientation)

        if state.timestamp is None:
            return

        timestamp = np.datetime64(state.timestamp)

        if self.start_time is None:
            self.start_time = timestamp

        t = float(
            (timestamp - self.start_time)
            / np.timedelta64(1, "s")
        )

        # Velocity
        if state.velocity is not None:
            self.velocity_time.append(t)
            self.speed.append(
                np.linalg.norm(state.velocity)
            )

        # Acceleration
        if state.acceleration is not None:
            self.acceleration_time.append(t)
            self.acceleration.append(
                np.linalg.norm(state.acceleration)
            )

        self._draw()

    def _draw(self):
        # Position
        self.position_ax.clear()
        self.position_ax.plot(self.x, self.y)

        self.position_ax.set_title("Position")
        self.position_ax.set_xlabel("x [m]")
        self.position_ax.set_ylabel("y [m]")
        self.position_ax.grid(True)

        # reference:
        self.position_ax.plot(
            self.path_x,
            self.path_y,
            "--",
            label="Reference path",
            color="black"
        )

        # reference point:
        self.position_ax.scatter(
            self.target[0],
            self.target[1],
            color="red",
            s=100,
        )

        # Robot heading
        if (
                len(self.x) > 0
                and self.orientation is not None
        ):
            x = self.x[-1]
            y = self.y[-1]

            arrow_length = 0.5

            dx = arrow_length * np.cos(self.orientation)
            dy = arrow_length * np.sin(self.orientation)

            self.position_ax.quiver(
                x,
                y,
                dx,
                dy,
                angles="xy",
                scale_units="xy",
                scale=1,
                color="red",
                width=0.005,
            )

        # Orientation
        self.orientation_ax.clear()
        self.orientation_ax.plot(self.velocity_time, self.orientation)

        self.orientation_ax.set_title("Orientation")
        self.orientation_ax.set_xlabel("x [m]")
        self.orientation_ax.set_ylabel("y [m]")
        self.orientation_ax.grid(True)

        # Velocity
        self.velocity_ax.clear()
        self.velocity_ax.plot(
            self.velocity_time,
            self.speed,
        )

        self.velocity_ax.set_title("Speed")
        self.velocity_ax.set_xlabel("Time [s]")
        self.velocity_ax.set_ylabel("m/s")
        self.velocity_ax.grid(True)

        # Acceleration
        self.acceleration_ax.clear()
        self.acceleration_ax.plot(
            self.acceleration_time,
            self.acceleration,
        )

        self.acceleration_ax.set_title("Acceleration")
        self.acceleration_ax.set_xlabel("Time [s]")
        self.acceleration_ax.set_ylabel("m/s²")
        self.acceleration_ax.grid(True)

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def is_open(self):
        return plt.fignum_exists(self.fig.number)

    def close(self):
        plt.ioff()
        plt.close(self.fig)