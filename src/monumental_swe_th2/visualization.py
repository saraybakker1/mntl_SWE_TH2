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
    def __init__(self, max_points=500, path_resolution=2000, max_wheel_velocity=2.0, visuals_update_full=False):
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

        self.visuals_update_full = visuals_update_full

        # Figure initialization:
        plt.ion()

        self.fig, self.axes = plt.subplots(
            1, 5,
            figsize=(16, 4),
        )

        self.position_ax = self.axes[0]

        if visuals_update_full:
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

        self.robot_point = self.position_ax.scatter([], [], label="robot")

        self.reference_line, = self.position_ax.plot(
            self.path_x,
            self.path_y,
            "--",
            color="black",
            label="reference"
        )

        self.target_point = self.position_ax.scatter(
            [],
            [],
            color="red",
            label="target",
            s=100
        )

        self.heading_arrow = self.position_ax.quiver(
            [0], [0],  # initial x, y
            [0.5], [0],  # initial dx, dy
            angles="xy",
            scale_units="xy",
            scale=1,
            color="blue",
            width=0.01,
            label="heading",
        )

        if visuals_update_full:
            self.orientation_line, = self.orientation_ax.plot([], [])

            self.speed_line, = self.velocity_ax.plot(
                [], [], color="green", label="speed"
            )

            self.vx_line, = self.velocity_ax.plot(
                [], [], "--", label="$v_x$"
            )

            self.vy_line, = self.velocity_ax.plot(
                [], [], "--", label="$v_y$"
            )

            # Acceleration
            self.ax_line, = self.acceleration_ax.plot(
                [], [], label="$a_x$"
            )

            self.ay_line, = self.acceleration_ax.plot(
                [], [], label="$a_y$"
            )

            # Wheel velocity / action
            self.left_wheel_line, = self.actions_ax.plot(
                [], [], label="$v_{left}$"
            )

            self.right_wheel_line, = self.actions_ax.plot(
                [], [], label="$v_{right}$"
            )

            # Wheel velocity limits
            self.max_wheel_velocity_line, = self.actions_ax.plot(
                [],
                [],
                "--",
                color="red",
                label="limits",
            )

            self.min_wheel_velocity_line, = self.actions_ax.plot(
                [],
                [],
                "--",
                color="red",
            )

        self.position_ax.set_title("Position")
        self.position_ax.set_xlabel("x [m]")
        self.position_ax.set_ylabel("y [m]")
        self.position_ax.grid(True)
        self.position_ax.legend(loc="upper left")

        if visuals_update_full:
            self.orientation_ax.set_title("Orientation")
            self.orientation_ax.set_xlabel("Time [s]")
            self.orientation_ax.set_ylabel("Orientation [rad]")
            self.orientation_ax.grid(True)

            self.velocity_ax.set_title("Velocity (Global frame)")
            self.velocity_ax.set_xlabel("Time [s]")
            self.velocity_ax.set_ylabel("m/s")
            self.velocity_ax.grid(True)
            self.velocity_ax.legend(loc="upper left")

            self.acceleration_ax.set_title("Acceleration (Robot frame)")
            self.acceleration_ax.set_xlabel("Time [s]")
            self.acceleration_ax.set_ylabel("m/s²")
            self.acceleration_ax.grid(True)
            self.acceleration_ax.legend(loc="upper left")

            self.actions_ax.set_title("Action: Wheel velocity")
            self.actions_ax.set_xlabel("Time [s]")
            self.actions_ax.set_ylabel("Velocity [rad/s]")
            self.actions_ax.grid(True)
            self.actions_ax.legend(loc="upper left")

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

        self.fig.tight_layout()
        # self._draw()

    def _draw(self):
        # update data instead of clear() + plot()

        self.robot_point.set_offsets([[self.x[-1], self.y[-1]]])
        self.robot_point.set_offsets(
            np.column_stack((self.x, self.y))
        )

        self.target_point.set_offsets([[self.target[0], self.target[1]]])

        if len(self.x) > 0 and self.orientation is not None:
            x = self.x[-1]
            y = self.y[-1]

            arrow_length = 0.5
            theta = self.orientation[-1]

            dx = arrow_length * np.cos(theta)
            dy = arrow_length * np.sin(theta)

            self.heading_arrow.set_offsets(
                np.array([[x, y]])
            )

            self.heading_arrow.set_UVC(
                np.array([dx]),
                np.array([dy])
            )

        self.acceleration_ax.relim()
        self.acceleration_ax.autoscale_view()

        if self.visuals_update_full:
            orientation = np.asarray(self.orientation).flatten()

            self.orientation_line.set_data(
                self.velocity_time,
                orientation
            )

            self.orientation_ax.relim()
            self.orientation_ax.autoscale_view()

            # Velocity
            self.speed_line.set_data(
                self.velocity_time,
                self.speed
            )

            velocities = np.asarray(self.velocities)

            self.vx_line.set_data(
                self.velocity_time,
                velocities[:, 0]
            )

            self.vy_line.set_data(
                self.velocity_time,
                velocities[:, 1]
            )

            self.velocity_ax.relim()
            self.velocity_ax.autoscale_view()

            # Acceleration
            acceleration = np.asarray(self.acceleration)

            self.ax_line.set_data(
                self.acceleration_time,
                acceleration[:, 0]
            )

            self.ay_line.set_data(
                self.acceleration_time,
                acceleration[:, 1]
            )

            self.acceleration_ax.relim()
            self.acceleration_ax.autoscale_view()

            # Control action
            actions = np.asarray(self.actions)

            self.left_wheel_line.set_data(
                self.action_time,
                actions[:, 0]
            )

            self.right_wheel_line.set_data(
                self.action_time,
                actions[:, 1]
            )

            self.actions_ax.relim()
            self.actions_ax.autoscale_view()

            if len(self.action_time) > 0:
                t0 = self.action_time[0]
                t1 = self.action_time[-1]

                self.max_wheel_velocity_line.set_data(
                    [t0, t1],
                    [self.max_wheel_velocity, self.max_wheel_velocity],
                )

                self.min_wheel_velocity_line.set_data(
                    [t0, t1],
                    [-self.max_wheel_velocity, -self.max_wheel_velocity],
                )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def is_open(self):
        return plt.fignum_exists(self.fig.number)

    def close(self):
        plt.ioff()
        plt.close(self.fig)