"""
Tipos y utilidades articulares del brazo de 7 GDL.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np
from numpy.typing import NDArray


@dataclass
class JointState:
    """Estado articular del robot (dp en mm, thetas en radianes)."""

    dp: float
    theta1: float
    theta2: float
    theta3: float
    theta4: float
    theta5: float
    theta6: float

    def as_array(self) -> NDArray[np.float64]:
        return np.array(
            [
                self.dp,
                self.theta1,
                self.theta2,
                self.theta3,
                self.theta4,
                self.theta5,
                self.theta6,
            ],
            dtype=float,
        )

    @classmethod
    def from_array(cls, q: Iterable[float]) -> "JointState":
        values = list(q)
        if len(values) != 7:
            raise ValueError("Se esperaban 7 variables articulares.")
        return cls(*values)

    def as_degrees(self) -> dict[str, float]:
        return {
            "dp_mm": self.dp,
            "theta1_deg": math.degrees(self.theta1),
            "theta2_deg": math.degrees(self.theta2),
            "theta3_deg": math.degrees(self.theta3),
            "theta4_deg": math.degrees(self.theta4),
            "theta5_deg": math.degrees(self.theta5),
            "theta6_deg": math.degrees(self.theta6),
        }


@dataclass
class IKResult:
    success: bool
    joints: JointState
    position_error_mm: float
    message: str


def joints_to_motor_steps(
    joints: JointState,
    *,
    steps_per_mm: float = 100.0,
    steps_per_degree: float = 80.0,
    home_joints: JointState | None = None,
) -> list[int]:
    """Convierte juntas a pasos relativos al home mecanico."""
    home = home_joints or JointState(
        dp=0.0, theta1=0.0, theta2=0.0, theta3=0.0, theta4=0.0, theta5=0.0, theta6=0.0
    )
    delta = joints.as_array() - home.as_array()

    steps = [int(round(delta[0] * steps_per_mm))]
    for angle in delta[1:]:
        steps.append(int(round(math.degrees(angle) * steps_per_degree)))
    return steps
