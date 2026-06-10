"""
Cinematica en marco CoppeliaSim: FK e IK sobre coppelia_kinematics.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import minimize

from coppelia_kinematics import L0, forward_kinematics_coppelia, forward_kinematics_from_joints
from robot_kinematics import IKResult, JointState

JOINT_LIMITS = {
    "dp": (0.0, L0),
    "theta1": (-math.pi, math.pi),
    "theta2": (-math.pi, math.pi),
    "theta3": (-math.pi, math.pi),
    "theta4": (-math.pi, math.pi),
    "theta5": (-math.pi, math.pi),
    "theta6": (-math.pi, math.pi),
}


def joint_home() -> JointState:
    return JointState(
        dp=0.0,
        theta1=0.0,
        theta2=0.0,
        theta3=0.0,
        theta4=0.0,
        theta5=0.0,
        theta6=0.0,
    )


def position_coppelia(joints: JointState) -> tuple[float, float, float]:
    return forward_kinematics_from_joints(joints).posicion


def home_cartesian_coppelia() -> tuple[float, float, float]:
    return position_coppelia(joint_home())


def _position_from_q(q: NDArray[np.float64]) -> NDArray[np.float64]:
    fk = forward_kinematics_coppelia(
        float(q[0]),
        math.degrees(float(q[1])),
        math.degrees(float(q[2])),
        math.degrees(float(q[3])),
        math.degrees(float(q[4])),
        math.degrees(float(q[5])),
        math.degrees(float(q[6])),
    )
    return np.array([fk.x, fk.y, fk.z], dtype=float)


def _pack_bounds() -> list[tuple[float, float]]:
    keys = ["dp", "theta1", "theta2", "theta3", "theta4", "theta5", "theta6"]
    return [JOINT_LIMITS[key] for key in keys]


def _initial_guesses(
    target: NDArray[np.float64],
    preferred: NDArray[np.float64],
) -> list[NDArray[np.float64]]:
    x, y, z = target
    seeds = [
        preferred.copy(),
        np.array([preferred[0], math.atan2(y, x), 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([z * 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([preferred[0], 0.0, -0.5, 0.8, -0.3, 0.0, 0.0]),
        np.array([preferred[0], math.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ]

    unique: list[NDArray[np.float64]] = []
    for seed in seeds:
        seed[0] = float(np.clip(seed[0], 0.0, L0))
        if not any(np.allclose(seed, existing, atol=1e-3) for existing in unique):
            unique.append(seed)
    return unique


def inverse_kinematics_coppelia(
    target_xyz: Iterable[float],
    *,
    preferred_pose: NDArray[np.float64] | None = None,
    tolerance_mm: float = 3.0,
    redundancy_weight: float = 0.02,
) -> IKResult:
    """Resuelve IK posicional (x, y, z) con el modelo CoppeliaSim."""
    target = np.asarray(target_xyz, dtype=float)
    if target.shape != (3,):
        raise ValueError("target_xyz debe ser [x, y, z]")

    preferred = joint_home().as_array() if preferred_pose is None else np.asarray(preferred_pose, dtype=float)
    bounds = _pack_bounds()

    best_q = preferred.copy()
    best_error = float("inf")
    best_message = "Sin convergencia"

    def objective(q: NDArray[np.float64]) -> float:
        pos = _position_from_q(q)
        pos_error = float(np.sum((pos - target) ** 2))
        reg = redundancy_weight * float(np.sum((q - preferred) ** 2))
        return pos_error + reg

    for q0 in _initial_guesses(target, preferred):
        result = minimize(
            objective,
            q0,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 400, "ftol": 1e-12},
        )
        error = float(np.linalg.norm(_position_from_q(result.x) - target))
        if error < best_error:
            best_error = error
            best_q = result.x.copy()
            best_message = result.message

    success = best_error <= tolerance_mm
    return IKResult(
        success=success,
        joints=JointState.from_array(best_q),
        position_error_mm=best_error,
        message=best_message if success else f"No se alcanzo la tolerancia ({best_error:.2f} mm): {best_message}",
    )
