from .trajectory_smoothing_impl import smooth
import numpy as np
import math


class TrajectorySmoother:
    def __init__(
        self,
        dof: int,
        max_acceleration: np.array,
        max_velocity: np.array,
        max_deviation: float,
    ):
        self.dof = dof
        self.max_acceleration = max_acceleration.astype(dtype=np.float64)
        self.max_velocity = max_velocity.astype(dtype=np.float64)
        self.max_deviation = max_deviation

    def smooth_interpolate(self, trajectory: np.ndarray, dt: float = 0.1):
        try:
            p, v = self._smooth_interpolate(trajectory, dt)
            return p, v
        except:
            return None, None
        

    def _smooth_interpolate(self, trajectory: np.ndarray, dt: float = 0.1):
        traj = trajectory.astype(dtype=np.float64)
        out = smooth(
            self.dof,
            dt,
            self.max_deviation,
            self.max_velocity,
            self.max_acceleration,
            traj,
        )
        if out.shape[0] == 1:
            return None, None

        max_length = math.ceil(out.shape[0] / 2)
        traj_positions = out[:max_length, :]
        traj_velocities = out[max_length:, :]
        return traj_positions.astype(np.float32), traj_velocities.astype(np.float32)
