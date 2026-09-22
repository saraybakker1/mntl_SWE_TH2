import numpy as np
from collections import deque


class StateEstimator:
    """
    Kalman-filter state estimator for a 2D robot.

    States to be estimated/updated:
        [x, y, theta, vx, vy]

    Inputs:
        - GPS position
        - acceleration (local)
        - angular velocity

    GPS messages may arrive at a high frequency while the actual
    GPS value remains unchanged. A GPS update is therefore only
    performed when the position changes.
    """

    def __init__(
        self,
        max_vel=2.0,
        max_acc=1.0,
        initial_orientation=0.0,
        gps_position_std=0.1,
        acceleration_std=0.02,
        orientation_std=0.015,
        filter_window=100,
        gps_heading_min_distance = 0.3
    ):
        self._max_vel = max_vel
        self._max_acc = max_acc
        self._initial_orientation = initial_orientation

        # State: [x, y, vx, vy, theta]
        self._x = np.zeros(5)
        self._x[4] = initial_orientation

        # State covariance
        self._P = np.diag([
            gps_position_std**2,
            gps_position_std**2,
            1.0**2,
            1.0**2,
            orientation_std**2,
        ])

        self._gps_position_std = gps_position_std
        self._acceleration_std = acceleration_std
        self._orientation_std = orientation_std

        # GPS measures [x, y]
        self._R_gps = np.diag([
            gps_position_std**2,
            gps_position_std**2,
        ])

        self._last_timestamp = None
        self._last_gps_position = None
        self._initialized = False
        self.orientation_gps = 0.

        self._gps_history = deque(maxlen=filter_window)
        self._velocity_history = deque(maxlen=filter_window)
        self._gps_heading_min_distance = gps_heading_min_distance

    def world_to_robot(self, value, theta):
        ax, ay = value
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            c * ax + s * ay,
            -s * ax + c * ay,
        ])

    def robot_to_world(self, value, theta):
        ax, ay = value
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            c * ax - s * ay,
            s * ax + c * ay,
        ])

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def predict(self, dt, acceleration, angular_velocity):
        if dt <= 0:
            return

        acceleration = (
            np.zeros(2)
            if acceleration is None
            else np.asarray(acceleration, dtype=float)
        )

        angular_velocity = (
            0.0
            if angular_velocity is None
            else float(angular_velocity)
        )

        # Update orientation first
        theta = self.wrap_angle(
            self._x[4] + angular_velocity * dt
        )

        # Convert body-frame acceleration to world frame
        acceleration_world = self.robot_to_world(
            acceleration,
            theta,
        )

        # Integrate position and velocity
        self._x[0:2] += (
                self._x[2:4] * dt
                + 0.5 * acceleration_world * dt ** 2
        )

        self._x[2:4] += acceleration_world * dt

        self._x[4] = theta

        # Covariance prediction
        F = np.array([
            [1, 0, dt, 0, 0],
            [0, 1, 0, dt, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1],
        ], dtype=float)

        acc_var = self._acceleration_std ** 2

        Q = np.diag([
            0.25 * dt ** 4 * acc_var,
            0.25 * dt ** 4 * acc_var,
            dt ** 2 * acc_var,
            dt ** 2 * acc_var,
            self._orientation_std ** 2 * dt,
        ])

        self._P = F @ self._P @ F.T + Q

    def update_gps(self, gps_position):
        gps_position = np.asarray(
            gps_position,
            dtype=float,
        )

        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
        ], dtype=float)

        innovation = gps_position - H @ self._x
        S = H @ self._P @ H.T + self._R_gps
        K = self._P @ H.T @ np.linalg.inv(S)

        self._x += K @ innovation
        self._P = (np.eye(5) - K @ H) @ self._P
        self._x[4] = self.wrap_angle(self._x[4])

    def gps_has_changed(self, position):
        if position is None:
            return False

        position = np.asarray(position, dtype=float)

        if self._last_gps_position is None:
            return True

        return not np.array_equal(
            position,
            self._last_gps_position,
        )

    def update_gps_heading_from_positions(
            self,
            previous_position,
            current_position,
    ):
        displacement = (
                current_position - previous_position
        )

        distance = np.linalg.norm(displacement)

        if distance < self._gps_heading_min_distance:
            return

        gps_heading = np.arctan2(
            displacement[1],
            displacement[0],
        )

        return gps_heading

    def update_gps_heading(
            self,
            gps_heading,
            gps_heading_std,
    ):
        H = np.array([
            [0, 0, 0, 0, 1]
        ], dtype=float)

        R = np.array([
            [gps_heading_std ** 2]
        ])

        innovation = self.wrap_angle(
            gps_heading - self._x[4]
        )

        S = H @ self._P @ H.T + R

        K = self._P @ H.T @ np.linalg.inv(S)

        self._x += (
                K[:, 0] * innovation
        )

        self._P = (
                          np.eye(5) - K @ H
                  ) @ self._P

        self._x[4] = self.wrap_angle(
            self._x[4]
        )
        return self._x[4]

    def warnings_limits(
        self,
        velocity,
        acceleration,
        orientation,
    ):
        velocity_robot = self.world_to_robot(
            velocity,
            orientation,
        )

        velocity_valid = np.all(
            (velocity_robot >= -self._max_vel)
            & (velocity_robot <= self._max_vel)
        )

        if not velocity_valid:
            print(
                "Warning: velocity above threshold:",
                velocity_robot,
            )

        if acceleration is not None:
            acceleration_robot = self.world_to_robot(
                acceleration,
                orientation,
            )

            acceleration_valid = np.all(
                (acceleration_robot >= -self._max_acc)
                & (acceleration_robot <= self._max_acc)
            )

            if not acceleration_valid:
                print(
                    "Warning: acceleration above threshold:",
                    acceleration_robot,
                )

    def update(self, state):
        if state.timestamp is None:
            return state

        timestamp = np.datetime64(state.timestamp)

        # Initialize
        if not self._initialized:
            if state.position is not None:
                position = np.asarray(
                    state.position,
                    dtype=float,
                )
                self._x[0:2] = position
                self._last_gps_position = position.copy()

            if state.velocity is not None:
                self._x[2:4] = np.asarray(
                    state.velocity,
                    dtype=float,
                )

            self._x[4] = (
                state.orientation
                if state.orientation is not None
                else self._initial_orientation
            )

            self._x[4] = self.wrap_angle(self._x[4])
            self._last_timestamp = timestamp
            self._initialized = True

            state.position = self._x[0:2].copy()
            state.velocity = self._x[2:4].copy()
            state.orientation = self._x[4]

            self.orientation_gps = state.orientation
            return state

        # Time step
        dt = (
            timestamp - self._last_timestamp
        ) / np.timedelta64(1, "s")

        if dt <= 0:
            return state

        self._last_timestamp = timestamp

        # Prediction
        self.predict(
            dt,
            state.acceleration,
            state.angular_velocity,
        )

        # GPS correction only when GPS value changes
        if state.position is not None:
            gps_position = np.asarray(
                state.position,
                dtype=float,
            )

            if self.gps_has_changed(gps_position):
                self.update_gps(gps_position)
                self.orientation_gps = self.update_gps_heading_from_positions(
                    self._last_gps_position, state.position)
                self._last_gps_position = (gps_position.copy())
                if self.orientation_gps is not None:
                    state.orientation = self.update_gps_heading(gps_heading=self.orientation_gps, gps_heading_std=self._gps_position_std)
                else:
                    state.orientation = self.wrap_angle(self._x[4])
            else:
                state.orientation = self.wrap_angle(self._x[4])

        # Output estimated state
        state.position = self._x[0:2].copy()
        state.velocity = self._x[2:4].copy()

        self._velocity_history.append(
            state.velocity.copy()
        )

        self.warnings_limits(
            state.velocity,
            state.acceleration,
            state.orientation,
        )

        return state
