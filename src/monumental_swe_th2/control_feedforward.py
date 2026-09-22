import numpy as np

class GroundSpeedFeedforward:
    """
    Option 2 controller: Combines a feedforward term (using a time-dependent reference) with a feedback term.
    """

    def __init__(self, path, position_gain = 1.0, heading_gain=1.0):
        self.path = path
        self.position_gain = position_gain
        self.heading_gain = heading_gain

    def update(self, t, state):
        position, velocity, acceleration = self.path.get(t)
        velocity_speed, angular_velocity = self._cartesian_to_unicycle(position, velocity, acceleration, state)
        return position, velocity_speed, angular_velocity
    # def get_path(self, t):
    #     position, velocity, acceleration = self.path.get(t)
    #
    #     vx = velocity[0]
    #     vy = velocity[1]
    #
    #     ground_speed = np.hypot(vx, vy)
    #
    #     return position, ground_speed

    def _wrap_to_pi(self, angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def _cartesian_to_unicycle(
            self,
            target_position,
            target_velocity,
            target_acceleration,
            state,
    ):
        position_error = target_position - state.position

        vx, vy = target_velocity
        ax, ay = target_acceleration

        speed_squared = vx ** 2 + vy ** 2

        if speed_squared > 1e-8:
            # --------------------------------
            # Moving reference
            # --------------------------------

            velocity = np.sqrt(speed_squared)

            # Feedforward angular velocity from
            # the curvature of the reference trajectory.
            angular_vel_ff = (
                                     vx * ay - vy * ax
                             ) / speed_squared

            # Desired direction of travel.
            target_heading = np.arctan2(vy, vx)

            # Heading error.
            heading_error = self._wrap_to_pi(
                target_heading - state.orientation
            )

            # Feedback correction.
            angular_vel_fb = (
                    self.heading_gain * heading_error
            )

            angular_vel = (
                    angular_vel_ff
                    + angular_vel_fb
            )

        else:
            # --------------------------------
            # Stationary reference
            # --------------------------------

            distance = np.linalg.norm(position_error)

            if distance > 1e-3:
                target_heading = np.arctan2(
                    position_error[1],
                    position_error[0],
                )

                heading_error = self._wrap_to_pi(
                    target_heading - state.orientation
                )

                velocity = self.position_gain * distance

                angular_vel = (
                        self.heading_gain * heading_error
                )

            else:
                velocity = 0.0
                angular_vel = 0.0

        return velocity, angular_vel