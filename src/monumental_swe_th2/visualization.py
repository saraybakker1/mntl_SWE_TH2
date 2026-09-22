import matplotlib.pyplot as plt
import numpy as np
from collections import deque

from monumental_swe_th2.sensor_client import RobotState
from monumental_swe_th2.path import Path_LoG
import matplotlib
matplotlib.use("QtAgg")

class StatePlotter:
    """
    Visualization interface, should create a pop-up window.
    """
    def __init__(self, max_points=500, path_resolution=2000, max_wheel_velocity=2.0):
        """
        :param max_points: max visible points in the plot
        :param path_resolution: number of points to represent the path
        :param max_wheel_velocity: limits of the wheel velocity for boundary plotting.
        """
        # History:
        self.x = deque(maxlen=max_points)
        self.y = deque(maxlen=max_points)
        self.orientation = deque(maxlen=max_points)

        self.velocity_time = deque(maxlen=max_points)
        self.speed = deque(maxlen=max_points)
        self.velocities = deque(maxlen=max_points)

        self.acceleration_time = deque(maxlen=max_points)
        self.acceleration = deque(maxlen=max_points)

        self.action_time = deque(maxlen=max_points)
        self.actions = deque(maxlen=max_points)

        self.start_time = None
        self.max_wheel_velocity = max_wheel_velocity


        # Figure initialization:
        plt.ion()

        self.fig, self.axes = plt.subplots(
            1, 5,
            figsize=(16, 4),
        )

        self.position_ax = self.axes[0]
        self.orientation_ax = self.axes[1]
        self.velocity_ax = self.axes[2]
        self.acceleration_ax = self.axes[3]
        self.actions_ax = self.axes[4]

        # define path:
        self.path_times = np.linspace(0.0, 20.0, path_resolution)
        path = Path_LoG(0)
        path_positions = np.array([
            path.get(t)[0]
            for t in self.path_times
        ])
        # Reference path
        self.path_x = path_positions[:, 0]
        self.path_y = path_positions[:, 1]

    def update(self, state: RobotState, target: np.ndarray, action: np.ndarray):
        if state.position is None:
            return

        if target is None:
            return

        if action is None:
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
            self.velocities.append(
                state.velocity
            )

        # Acceleration
        if state.acceleration is not None:
            self.acceleration_time.append(t)
            self.acceleration.append(
                state.acceleration
            )

        # Acceleration
        if action is not None:
            self.action_time.append(t)
            self.actions.append(
                action
            )

        self._draw()

    def _draw(self):
        # Position
        self.position_ax.clear()
        self.position_ax.scatter(self.x, self.y, label="robot")

        self.position_ax.set_title("Position")
        self.position_ax.set_xlabel("x [m]")
        self.position_ax.set_ylabel("y [m]")
        self.position_ax.grid(True)

        # reference:
        self.position_ax.plot(
            self.path_x,
            self.path_y,
            "--",
            label="reference",
            color="black"
        )

        # reference point:
        self.position_ax.scatter(
            self.target[0],
            self.target[1],
            color="red",
            label="target",
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

            dx = arrow_length * np.cos(self.orientation[-1])
            dy = arrow_length * np.sin(self.orientation[-1])

            self.position_ax.quiver(
                x,
                y,
                dx,
                dy,
                angles="xy",
                scale_units="xy",
                scale=1,
                color="blue",
                label="heading",
                width=0.01,
            )
        self.position_ax.legend(loc="upper left")

        # Orientation
        self.orientation_ax.clear()
        self.orientation_ax.plot(self.velocity_time, self.orientation)

        self.orientation_ax.set_title("Orientation")
        self.orientation_ax.set_xlabel("Time [s]")
        self.orientation_ax.set_ylabel("Orientation [rad]")
        self.orientation_ax.grid(True)

        # Velocity
        self.velocity_ax.clear()
        self.velocity_ax.plot(
            self.velocity_time,
            self.speed,
            color = "green",
            label="speed"
        )
        self.velocity_ax.plot(
            self.velocity_time,
            self.velocities,
            "--",
            label=["$v_x$", "$v_y$"],
        )

        self.velocity_ax.set_title("Velocity (Global frame)")
        self.velocity_ax.set_xlabel("Time [s]")
        self.velocity_ax.set_ylabel("m/s")
        self.velocity_ax.grid(True)
        self.velocity_ax.legend(loc="upper left")

        # Acceleration
        self.acceleration_ax.clear()
        self.acceleration_ax.plot(
            self.acceleration_time,
            self.acceleration,
            label = ["$a_x$", "$a_y$"]
        )

        self.acceleration_ax.set_title("Acceleration (Robot frame)")
        self.acceleration_ax.set_xlabel("Time [s]")
        self.acceleration_ax.set_ylabel("m/s²")
        self.acceleration_ax.grid(True)
        self.acceleration_ax.legend(loc="upper left")

        # Control action = Wheel velocity
        self.actions_ax.clear()
        self.actions_ax.plot(
            self.action_time,
            self.actions,
            label=["$v_{left}$", "$v_{right}$"]
        )
        self.actions_ax.plot(
            [self.acceleration_time[0], self.acceleration_time[-1]],
            [self.max_wheel_velocity, self.max_wheel_velocity],
            "--",
            label="limits",
            color = "red"
        )
        self.actions_ax.plot(
            [self.acceleration_time[0], self.acceleration_time[-1]],
            [-self.max_wheel_velocity, -self.max_wheel_velocity],
            "--",
            color = "red"
        )

        self.actions_ax.set_title("Action: Wheel velocity")
        self.actions_ax.set_xlabel("Time [s]")
        self.actions_ax.set_ylabel("Velocity [rad/s]")
        self.actions_ax.grid(True)
        self.actions_ax.legend(loc="upper left")

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def is_open(self):
        return plt.fignum_exists(self.fig.number)

    def close(self):
        plt.ioff()
        plt.close(self.fig)