import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .trajectory_smoothing_impl import smooth, test


@dataclass
class SmoothResult:
    success: bool
    length: Optional[int] = None
    interpolation_dt: Optional[float] = None
    position: Optional[np.ndarray] = None
    velocity: Optional[np.ndarray] = None
    acceleration: Optional[np.ndarray] = None
    jerk: Optional[np.ndarray] = None
    solve_time: float = 0.0

    def __post_init__(self):
        if self.success:
            self.interpolation_dt = float(self.interpolation_dt)
            self.position = self.position.astype(dtype=np.float32)
            self.velocity = self.velocity.astype(dtype=np.float32)
            self.acceleration = self.acceleration.astype(dtype=np.float32)
            self.jerk = self.jerk.astype(dtype=np.float32)


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

    def smooth_interpolate(
        self,
        trajectory: np.ndarray,
        traj_dt: float = 0.05,
        interpolation_dt: float = 0.01,
        max_tsteps: int = -1,
    ) -> SmoothResult:
        st_time = time.time()
        traj = trajectory.astype(dtype=np.float64)

        (out0, out1, out2, out3, out4, out5, out6) = smooth(
            self.dof,
            interpolation_dt,
            self.max_deviation,
            self.max_velocity,
            self.max_acceleration,
            traj,
            traj_dt,
            max_tsteps,
        )
        if not out0:
            return SmoothResult(False, solve_time=time.time() - st_time)

        result = SmoothResult(
            out0,
            out1,
            out2,
            out3,
            out4,
            out5,
            out6,
            time.time() - st_time,
        )

        return result

    def test(self):
        return test()
