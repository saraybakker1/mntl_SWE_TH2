import numpy as np


class Path_LoG:
    """
    Compute path position, and velocity of the path
    """
    def __init__(self, start_time):
        self.start_time = start_time

    def get(self, t):
        if t < 20.0:
            k = np.pi * t / 10.0 - np.pi / 2
            dk_dt = np.pi / 10.0

            x = -2 * np.sin(k) * np.cos(k)
            y = 2 * (np.sin(k) + 1)

            vx = -2 * np.cos(2 * k) * dk_dt
            vy = 2 * np.cos(k) * dk_dt

            ax = 4 * np.sin(2 * k) * dk_dt ** 2
            ay = -2 * np.sin(k) * dk_dt ** 2

            return (
                np.array([x, y]),
                np.array([vx, vy]),
                np.array([ax, ay]),
            )

        # Path stops at t=20
        k = 3 * np.pi / 2

        x = -2 * np.sin(k) * np.cos(k)
        y = 2 * (np.sin(k) + 1)

        return np.array([x, y]), np.zeros(2), np.zeros(2)