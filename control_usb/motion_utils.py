"""Utilidades de conversion juntas -> pasos (sin dependencia serial)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_kinematics import JointState  # noqa: E402

import config  # noqa: E402


def home_joint_state() -> JointState:
    h = config.HOME_JOINTS
    return JointState(
        dp=h["dp"],
        theta1=h["theta1"],
        theta2=h["theta2"],
        theta3=h["theta3"],
        theta4=h["theta4"],
        theta5=h["theta5"],
        theta6=h["theta6"],
    )


def joint_delta_to_steps(
    desde: JointState,
    hasta: JointState,
    *,
    steps_per_mm: float | None = None,
    steps_per_degree: float | None = None,
) -> list[int]:
    """Pasos relativos entre dos configuraciones articulares."""
    spm = steps_per_mm if steps_per_mm is not None else config.STEPS_PER_MM
    spd = steps_per_degree if steps_per_degree is not None else config.STEPS_PER_DEGREE

    delta = hasta.as_array() - desde.as_array()
    pasos = [int(round(delta[0] * spm))]
    for angulo in delta[1:]:
        pasos.append(int(round(math.degrees(angulo) * spd)))
    return pasos
