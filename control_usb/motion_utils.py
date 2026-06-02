"""Utilidades de conversion juntas -> pasos (sin dependencia serial)."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_kinematics import JointState, position_from_joints  # noqa: E402

import config  # noqa: E402


@dataclass
class SimulacionMovimiento:
    pose_inicial: JointState
    pose_final: JointState
    delta_dp_mm: float
    delta_thetas_deg: tuple[float, float, float, float, float, float]
    posicion_xyz: tuple[float, float, float]
    pasos_motor: list[int]


def pose_inicial_sim() -> JointState:
    p = config.POSE_INICIAL_SIM
    return JointState(
        dp=p["dp"],
        theta1=p["theta1"],
        theta2=p["theta2"],
        theta3=p["theta3"],
        theta4=p["theta4"],
        theta5=p["theta5"],
        theta6=p["theta6"],
    )


def simular_movimiento_relativo(
    delta_dp_mm: float,
    delta_thetas_deg: tuple[float, ...],
    *,
    pose_inicial: JointState | None = None,
) -> SimulacionMovimiento:
    """
    Aplica desplazamientos relativos a cada motor y calcula la posicion final (FK).
    Motor 0: mm. Motores 1-6: grados.
    """
    if len(delta_thetas_deg) != 6:
        raise ValueError("Se esperaban 6 angulos (theta1..theta6).")

    inicial = pose_inicial or pose_inicial_sim()
    deltas_rad = [math.radians(float(d)) for d in delta_thetas_deg]
    final = JointState(
        dp=inicial.dp + delta_dp_mm,
        theta1=inicial.theta1 + deltas_rad[0],
        theta2=inicial.theta2 + deltas_rad[1],
        theta3=inicial.theta3 + deltas_rad[2],
        theta4=inicial.theta4 + deltas_rad[3],
        theta5=inicial.theta5 + deltas_rad[4],
        theta6=inicial.theta6 + deltas_rad[5],
    )
    xyz = tuple(float(v) for v in position_from_joints(final))
    pasos = joint_delta_to_steps(inicial, final)
    return SimulacionMovimiento(
        pose_inicial=inicial,
        pose_final=final,
        delta_dp_mm=delta_dp_mm,
        delta_thetas_deg=tuple(float(d) for d in delta_thetas_deg),  # type: ignore[assignment]
        posicion_xyz=xyz,
        pasos_motor=pasos,
    )


def formatear_simulacion(resultado: SimulacionMovimiento, *, ejecutado: bool = False) -> str:
    ini = resultado.pose_inicial.as_degrees()
    fin = resultado.pose_final.as_degrees()
    x, y, z = resultado.posicion_xyz
    d = resultado.delta_thetas_deg
    titulo = (
        "=== MOVIMIENTO ARTICULAR (FK + ROBOT REAL) ==="
        if ejecutado
        else "=== SIMULACION DE MOVIMIENTO (FK) ==="
    )
    lineas = [
        titulo,
        "",
        "Movimiento solicitado:",
        f"  dp     : {resultado.delta_dp_mm:+.1f} mm",
        f"  theta1 : {d[0]:+.1f} deg",
        f"  theta2 : {d[1]:+.1f} deg",
        f"  theta3 : {d[2]:+.1f} deg",
        f"  theta4 : {d[3]:+.1f} deg",
        f"  theta5 : {d[4]:+.1f} deg",
        f"  theta6 : {d[5]:+.1f} deg",
        "",
        "Pose inicial:",
        f"  dp={ini['dp_mm']:.1f} mm | "
        f"t1={ini['theta1_deg']:.1f} t2={ini['theta2_deg']:.1f} t3={ini['theta3_deg']:.1f} | "
        f"t4={ini['theta4_deg']:.1f} t5={ini['theta5_deg']:.1f} t6={ini['theta6_deg']:.1f} (deg)",
        "",
        "Pose final:",
        f"  dp={fin['dp_mm']:.1f} mm | "
        f"t1={fin['theta1_deg']:.1f} t2={fin['theta2_deg']:.1f} t3={fin['theta3_deg']:.1f} | "
        f"t4={fin['theta4_deg']:.1f} t5={fin['theta5_deg']:.1f} t6={fin['theta6_deg']:.1f} (deg)",
        "",
        f"Posicion del efector: x={x:.2f} mm  y={y:.2f} mm  z={z:.2f} mm",
        f"Pasos de motor (referencia): {resultado.pasos_motor}",
    ]
    return "\n".join(lineas)


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
