import numpy as np
from monumental_swe_th2.sensor_client import RobotState
from monumental_swe_th2.path import Path_LoG

class PathController:
    def __init__(
        self,
        wheel_base=0.5,
        max_wheel_velocity=2.0,
        lookahead_distance=0.3,
        max_velocity=2.0,
        speed_curvature_gain=1.0,
        path_resolution=2000,
    ):
        self.wheel_base = wheel_base
        self.max_wheel_velocity = max_wheel_velocity
        self.lookahead_distance = lookahead_distance
        self.max_velocity = max_velocity
        self.speed_curvature_gain = speed_curvature_gain

        self.path_times = np.linspace(0.0, 20.0, path_resolution)
        self.path = Path_LoG(0)
        self.path_positions = np.array([self.path.get(t)[0] for t in self.path_times])
        self._nearest_index = 0

    def _find_nearest_index(self, position):
        # Only search forward from the previous point.
        # This prevents jumping to the other branch at
        # the self-intersection of the lemniscate.
        distances = np.linalg.norm(
            self.path_positions[self._nearest_index:] - position,
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
        relative_position = target - state.position

        target_robot = self._world_to_robot(
            relative_position,
            state.orientation,
        )

        x = target_robot[0]
        y = target_robot[1]

        distance_squared = x**2 + y**2

        if distance_squared < 1e-8:
            return 0.0, 0.0

        curvature = 2.0 * y / distance_squared

        # Slow down when curvature is high.
        velocity = (
            self.max_velocity
            / (1.0 + self.speed_curvature_gain * abs(curvature))
        )

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

    def update(self, state: RobotState):
        if (
            state.position is None
            or state.orientation is None
        ):
            return None, None

        current_time = state.current_time
        target_timewise, _ = self.path.get(state.current_time)
        nearest_index = self._find_nearest_index(
            state.position
        )

        lookahead_index = self._find_lookahead_index(
            nearest_index
        )
        print("lookahead_index:", lookahead_index)

        target = self.path_positions[lookahead_index]

        target = target_timewise

        velocity, angular_vel = self._pure_pursuit(
            target,
            state,
        )

        left, right = self._velocity_to_wheels(
            velocity,
            angular_vel,
        )

        return np.array([10*left, 10*right]), np.array(target)