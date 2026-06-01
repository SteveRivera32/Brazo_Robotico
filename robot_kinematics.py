"""
Cinemática directa e inversa del brazo de 7 GDL (1 prismática + 6 rotacionales).
Convención DH estándar según la tabla del proyecto.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np
from numpy.typing import NDArray

try:
    from scipy.optimize import minimize
except ImportError:  # pragma: no cover
    minimize = None


# Longitudes en mm
L0 = 500.0  # recorrido máximo prismática
L1 = 235.0
L2 = 154.0
L3 = 145.0
L4 = 224.0
L5 = 145.0
L6 = 224.0
L7 = 52.0

# Tabla DH: [a, alpha, d, theta_variable_index]
# theta_variable_index: -1 = fijo (0), 0 = dp, 1..6 = theta1..theta6
DH_TABLE = [
    (0.0, math.pi / 2, "dp", None),
    (0.0, math.pi / 2, L1, 1),
    (L2, -math.pi / 2, 0.0, 2),
    (L3, math.pi / 2, 0.0, 3),
    (L4, -math.pi / 2, 0.0, 4),
    (L5, math.pi / 2, 0.0, 5),
    (L6, -math.pi / 2, 0.0, 6),
    (L7, 0.0, 0.0, None),
]

DEFAULT_JOINT_LIMITS = {
    "dp": (0.0, L0),
    "theta1": (-math.pi, math.pi),
    "theta2": (-math.pi, math.pi),
    "theta3": (-math.pi, math.pi),
    "theta4": (-math.pi, math.pi),
    "theta5": (-math.pi, math.pi),
    "theta6": (-math.pi, math.pi),
}

# Pose preferida para resolver la redundancia del brazo (7 GDL, solo posición)
DEFAULT_PREFERRED_POSE = np.array([200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


@dataclass
class JointState:
    """Estado articular del robot."""

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


def _dh_matrix(a: float, alpha: float, d: float, theta: float) -> NDArray[np.float64]:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def forward_kinematics(joints: JointState | NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    Devuelve (posición xyz, matriz T 4x4) del efector final.
    """
    if isinstance(joints, JointState):
        q = joints.as_array()
    else:
        q = np.asarray(joints, dtype=float)

    dp = q[0]
    thetas = {i + 1: q[i + 1] for i in range(6)}

    t = np.eye(4)
    for a, alpha, d_param, theta_idx in DH_TABLE:
        d = dp if d_param == "dp" else float(d_param)
        theta = thetas[theta_idx] if theta_idx is not None else 0.0
        t = t @ _dh_matrix(a, alpha, d, theta)

    return t[:3, 3].copy(), t


def position_from_joints(joints: JointState | NDArray[np.float64]) -> NDArray[np.float64]:
    return forward_kinematics(joints)[0]


def _pack_bounds(limits: dict[str, tuple[float, float]] | None = None) -> list[tuple[float, float]]:
    limits = limits or DEFAULT_JOINT_LIMITS
    keys = ["dp", "theta1", "theta2", "theta3", "theta4", "theta5", "theta6"]
    return [limits[key] for key in keys]


def _initial_guesses(
    target: NDArray[np.float64],
    preferred: NDArray[np.float64],
) -> list[NDArray[np.float64]]:
    x, y, z = target
    r_xy = math.hypot(x, y)
    theta1_seed = math.atan2(z - preferred[0], r_xy) if r_xy > 1e-6 else 0.0

    seeds = [
        preferred.copy(),
        np.array([z, theta1_seed, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([preferred[0], math.atan2(y, x), 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([z, 0.0, -0.5, 0.8, -0.3, 0.0, 0.0]),
        np.array([z, math.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([preferred[0], 0.0, 0.6, -0.9, 0.4, 0.0, 0.0]),
    ]

    unique: list[NDArray[np.float64]] = []
    for seed in seeds:
        seed[0] = float(np.clip(seed[0], 0.0, L0))
        if not any(np.allclose(seed, existing, atol=1e-3) for existing in unique):
            unique.append(seed)
    return unique


def inverse_kinematics_position(
    target_xyz: Iterable[float],
    *,
    preferred_pose: NDArray[np.float64] | None = None,
    joint_limits: dict[str, tuple[float, float]] | None = None,
    tolerance_mm: float = 1.0,
    redundancy_weight: float = 0.02,
) -> IKResult:
    """
    Calcula una solución articular que alcanza (x, y, z).
    Con 7 GDL y solo posición hay infinitas soluciones; se elige la más cercana
    a `preferred_pose` dentro de los límites articulares.
    """
    if minimize is None:
        raise ImportError("Se requiere scipy para la cinemática inversa. Instala: pip install scipy")

    target = np.asarray(target_xyz, dtype=float)
    if target.shape != (3,):
        raise ValueError("target_xyz debe ser [x, y, z]")

    preferred = DEFAULT_PREFERRED_POSE if preferred_pose is None else np.asarray(preferred_pose, dtype=float)
    bounds = _pack_bounds(joint_limits)

    best_q = preferred.copy()
    best_error = float("inf")
    best_message = "Sin convergencia"

    def objective(q: NDArray[np.float64]) -> float:
        pos = position_from_joints(q)
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
        pos = position_from_joints(result.x)
        error = float(np.linalg.norm(pos - target))
        if error < best_error:
            best_error = error
            best_q = result.x.copy()
            best_message = result.message

    success = best_error <= tolerance_mm
    return IKResult(
        success=success,
        joints=JointState.from_array(best_q),
        position_error_mm=best_error,
        message=best_message if success else f"No se alcanzó la tolerancia ({best_error:.2f} mm): {best_message}",
    )


def joints_to_motor_steps(
    joints: JointState,
    *,
    steps_per_mm: float = 100.0,
    steps_per_degree: float = 80.0,
    home_joints: JointState | None = None,
) -> list[int]:
    """
    Convierte desplazamiento/ángulos a pasos relativos al 'home'.
    Ajusta steps_per_mm y steps_per_degree según la calibración real.
    """
    home = home_joints or JointState(dp=0.0, theta1=0.0, theta2=0.0, theta3=0.0, theta4=0.0, theta5=0.0, theta6=0.0)
    delta = joints.as_array() - home.as_array()

    steps = [int(round(delta[0] * steps_per_mm))]
    for angle in delta[1:]:
        steps.append(int(round(math.degrees(angle) * steps_per_degree)))
    return steps
