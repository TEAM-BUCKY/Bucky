"""Physics backend interface.

Future C++ backend: implement PhysicsBackend via pybind11, expose as
`from rl.physics.cpp_backend import CppBackend`. Drop-in replacement.
"""
from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


@dataclass
class PhysicsState:
    robot_pos: np.ndarray       # (2,) world frame, meters
    robot_vel: np.ndarray       # (2,) world frame, m/s
    robot_heading: float        # radians, 0 = +x axis (field forward)
    robot_omega: float          # rad/s, positive = CCW
    ball_pos: np.ndarray        # (2,) world frame, meters
    ball_vel: np.ndarray        # (2,) world frame, m/s


class PhysicsBackend(ABC):
    """2D physics environment interface.

    Coordinate system: origin at field center, +x = right, +y = up (field view).
    Field: 2.4 m × 1.8 m. Goals on ±x axis.
    """

    @abstractmethod
    def reset(self, seed: int | None = None) -> PhysicsState: ...

    @abstractmethod
    def step(self, vx_body: float, vy_body: float, omega: float) -> tuple[PhysicsState, dict]:
        """Apply body-frame velocity command for one timestep (0.02 s).

        Returns updated state and info dict (e.g. {'goal_scored': bool, 'out_of_bounds': bool}).
        """
        ...

    @property
    @abstractmethod
    def dt(self) -> float: ...
