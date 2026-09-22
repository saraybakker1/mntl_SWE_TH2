import numpy as np
from monumental_swe_th2.sensor_client import RobotState
from monumental_swe_th2.path import Path_LoG
from monumental_swe_th2.control_feedforward import GroundSpeedFeedforward

class Controller:
    """
    Controller: Taking in (estimated) current state, and providing the velocity action of the wheels to be send to the robot.
    """
    def __init__(
        self,
        wheel_base=0.5,
        max_wheel_velocity=2.0,
        lookahead_distance=0.3,
        max_velocity=2.0,
        max_wheel_acceleration=1.0,
        speed_curvature_gain=10.0,
        path_resolution=2000,
        dt = 0.02,
        goal_tolerance = 0.5,
        WITH_OPTION2 = False,
    ):
        self.wheel_base = wheel_base
        self.max_wheel_velocity = max_wheel_velocity
        self.max_wheel_acceleration = max_wheel_acceleration
        self.lookahead_distance = lookahead_distance
        self.max_velocity = max_velocity #todo: remove
        self.speed_curvature_gain = speed_curvature_gain
        self.path_resolution = path_resolution

        self.path_times = np.linspace(0.0, 20.0, path_resolution)
        self.path = Path_LoG(0)
        self.path_positions = np.array([self.path.get(t)[0] for t in self.path_times])
        self._nearest_index = 0
        self._previous_left = 0.0
        self._previous_right = 0.0
        self.dt = dt
        self.goal_tolerance = goal_tolerance
        self.stopped = False
        self.WITH_OPTION2 = WITH_OPTION2

        self.feedforward = GroundSpeedFeedforward(path=self.path)

    def _find_nearest_index(self, position, current_time):
        # Only search forward from the previous point.
        # This prevents jumping to the other branch at
        # the self-intersection of the lemniscate.
        max_index = self._nearest_index + int(7*(self.path_resolution/20))
        distances = np.linalg.norm(
            self.path_positions[self._nearest_index:max_index] - position,
            axis=1,
        )

        index = self._nearest_index + np.argmin(distances)

        self._nearest_index = index

        return index

    def _find_lookahead_index(self, nearest_index):
        distances = np.linalg.norm(
            self.path_positions[nearest_index:]
            - self.path_positions[nearest_index],
            axis=1,
        )

        indices = np.where(
            distances >= self.lookahead_distance
        )[0]

        if len(indices) == 0:
            return len(self.path_positions) - 1
        return nearest_index + indices[0]

    @staticmethod
    def _world_to_robot(vector, theta):
        x, y = vector

        c = np.cos(theta)
        s = np.sin(theta)

        return np.array([
            c * x + s * y,
            -s * x + c * y,
        ])

    def _pure_pursuit(self, target, state):
        target = np.asarray(target, dtype=float)

        relative_position = target - state.position
        target_robot = self._world_to_robot(
            relative_position,
            state.orientation,
        )

        x, y = target_robot
        distance_squared = x ** 2 + y ** 2

        if distance_squared < 1e-8:
            return 0.0, 0.0

        curvature = 2.0 * y / distance_squared

        velocity = (
                self.max_velocity
                / (1.0 + self.speed_curvature_gain * abs(curvature))
        )

        if x < 0:
            velocity = -velocity
            angular_vel = -velocity * curvature
        else:
            angular_vel = velocity * curvature

        return velocity, angular_vel


    def _velocity_to_wheels(self, velocity, angular_vel):
        half_wheel_base = self.wheel_base / 2.0

        left = velocity - angular_vel * half_wheel_base
        right = velocity + angular_vel * half_wheel_base

        # Respect wheel velocity limits.
        max_velocity = max(
            abs(left),
            abs(right),
        )

        if max_velocity > self.max_wheel_velocity:
            scale = self.max_wheel_velocity / max_velocity
            left *= scale
            right *= scale

        return left, right

    def _limit_wheel_acceleration(
            self,
            desired_left,
            desired_right,
            dt,
    ):
        max_acc = self.max_wheel_acceleration

        max_delta = max_acc * dt

        left_delta = desired_left - self._previous_left
        right_delta = desired_right - self._previous_right

        max_delta = self.max_wheel_acceleration * dt

        # Find the largest wheel delta
        max_requested_delta = max(
            abs(left_delta),
            abs(right_delta),
        )

        # Scale both deltas equally
        if max_requested_delta > max_delta:
            scale = max_delta / max_requested_delta
        else:
            scale = 1.0

        left_delta *= scale
        right_delta *= scale

        left = self._previous_left + left_delta
        right = self._previous_right + right_delta

        self._previous_left = left
        self._previous_right = right

        return left, right

    def update(self, state: RobotState):
        if (
            state.position is None
            or state.orientation is None
        ):
            return None, None

        current_time = state.current_time

        nearest_index = self._find_nearest_index(
            state.position,
            current_time
        )

        lookahead_index = self._find_lookahead_index(
            nearest_index
        )

        target = self.path_positions[lookahead_index]

        velocity, angular_vel = self._pure_pursuit(
            target,
            state,
        )

        # option 2: combination of feedforward and feedback term:
        if self.WITH_OPTION2:
            target_position, velocity, angular_vel = self.feedforward.update(current_time, state)
            target = target_position

        desired_left, desired_right = self._velocity_to_wheels(
            velocity,
            angular_vel,
        )

        left, right = self._limit_wheel_acceleration(
            desired_left,
            desired_right,
            self.dt,
        )

        # stopping criteria:
        dist_to_goal = np.linalg.norm(np.asarray(state.position) - np.asarray(self.path_positions[-1]))
        if ((lookahead_index == self.path_resolution - 1) and dist_to_goal < self.goal_tolerance)\
                or self.stopped:
            left, right = 0., 0.
            self.stopped = True

        return np.array([left, right]), np.array(target)