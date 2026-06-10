"""
Cinematica directa del brazo en marco CoppeliaSim.

Modelo: prismática en Y + eslabones con traslaciones/rotaciones alternadas
segun la geometria del robot fisico.

La evaluacion numerica usa NumPy (rapida). El mismo modelo se puede derivar
con SymPy para ecuaciones simbolicas.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from robot_kinematics import JointState

# ============================
# MEDIDAS DEL ROBOT EN mm
# ============================

L0 = 420.0
L1 = 576.4
L2 = 149.72
L3 = 144.92
L4 = 223.95
L5 = 146.59
L6 = 223.95
L7 = 54.75


def _Tx(a: float) -> np.ndarray:
    return np.array([
        [1.0, 0.0, 0.0, a],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _Ty(a: float) -> np.ndarray:
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, a],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _Tz(a: float) -> np.ndarray:
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, a],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _Rz(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _Ry(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _matriz_a_euler_coppelia(R: np.ndarray) -> tuple[float, float, float]:
    """
    Convierte una matriz de rotacion 3x3 a angulos tipo CoppeliaSim.
    R = Rx(alpha) * Ry(beta) * Rz(gamma)
    """
    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
    r23, r33 = R[1, 2], R[2, 2]

    beta = math.asin(float(np.clip(r13, -1.0, 1.0)))
    alpha = math.atan2(-r23, r33)
    gamma = math.atan2(-r12, r11)
    return alpha, beta, gamma


def _transform_matrix(
    dp_mm: float,
    theta1_rad: float,
    theta2_rad: float,
    theta3_rad: float,
    theta4_rad: float,
    theta5_rad: float,
    theta6_rad: float,
) -> np.ndarray:
    T = np.eye(4)
    T = T @ _Ty(dp_mm)
    T = T @ _Tz(L1)
    T = T @ _Rz(theta1_rad)
    T = T @ _Tz(L2)
    T = T @ _Ry(theta2_rad)
    T = T @ _Ty(L3)
    T = T @ _Rz(theta3_rad)
    T = T @ _Tz(L4)
    T = T @ _Ry(theta4_rad)
    T = T @ _Ty(L5)
    T = T @ _Rz(theta5_rad)
    T = T @ _Tz(L6)
    T = T @ _Ry(theta6_rad)
    T = T @ _Ty(L7)
    return T


@dataclass
class CoppeliaFKResult:
    x: float
    y: float
    z: float
    alpha_deg: float
    beta_deg: float
    gamma_deg: float

    @property
    def posicion(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def orientacion(self) -> tuple[float, float, float]:
        return (self.alpha_deg, self.beta_deg, self.gamma_deg)


def forward_kinematics_coppelia(
    dp_mm: float,
    theta1_deg: float,
    theta2_deg: float,
    theta3_deg: float,
    theta4_deg: float,
    theta5_deg: float,
    theta6_deg: float,
) -> CoppeliaFKResult:
    """Cinematica directa numerica (dp en mm, angulos en grados)."""
    thetas = [math.radians(t) for t in (theta1_deg, theta2_deg, theta3_deg, theta4_deg, theta5_deg, theta6_deg)]
    T = _transform_matrix(dp_mm, *thetas)

    x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
    alpha, beta, gamma = _matriz_a_euler_coppelia(T[:3, :3])

    return CoppeliaFKResult(
        x=x,
        y=y,
        z=z,
        alpha_deg=math.degrees(alpha),
        beta_deg=math.degrees(beta),
        gamma_deg=math.degrees(gamma),
    )


def forward_kinematics_from_joints(joints: JointState) -> CoppeliaFKResult:
    deg = joints.as_degrees()
    return forward_kinematics_coppelia(
        deg["dp_mm"],
        deg["theta1_deg"],
        deg["theta2_deg"],
        deg["theta3_deg"],
        deg["theta4_deg"],
        deg["theta5_deg"],
        deg["theta6_deg"],
    )


if __name__ == "__main__":
    import time

    resultado = forward_kinematics_coppelia(78, 67, 0, 89, 23, 0, 0)
    x, y, z = resultado.posicion
    alpha, beta, gamma = resultado.orientacion

    t0 = time.perf_counter()
    for _ in range(1000):
        forward_kinematics_coppelia(78, 67, 0, 89, 23, 0, 0)
    elapsed_ms = (time.perf_counter() - t0) / 1000 * 1000

    print("================================")
    print("POSICION DEL EFECTOR FINAL")
    print("================================")
    print(f"X = {x:+.3f} mm")
    print(f"Y = {y:+.3f} mm")
    print(f"Z = {z:+.3f} mm")
    print("\nEn centimetros:")
    print(f"X = {x/10:+.3f} cm")
    print(f"Y = {y/10:+.3f} cm")
    print(f"Z = {z/10:+.3f} cm")
    print("\n================================")
    print("ORIENTACION DEL ULTIMO ESLABON")
    print("FORMATO TIPO COPPELIA")
    print("================================")
    print(f"a / alpha = {alpha:+.3f}°")
    print(f"b / beta  = {beta:+.3f}°")
    print(f"g / gamma = {gamma:+.3f}°")
    print(f"\n1000 calculos: {elapsed_ms:.2f} ms total ({elapsed_ms:.3f} ms/calculo)")
