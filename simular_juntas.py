"""
Simulacion de movimiento relativo por juntas (solo calculo FK).

Usa el modelo CoppeliaSim (SymPy) de coppelia_kinematics.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from coppelia_kinematics import CoppeliaFKResult, forward_kinematics_from_joints
from robot_kinematics import JointState


@dataclass
class SimulacionJuntasResult:
    pose_inicial: JointState
    pose_final: JointState
    deltas_mm_deg: dict[str, float]
    fk_inicial: CoppeliaFKResult
    fk_final: CoppeliaFKResult

    @property
    def posicion_inicial(self) -> tuple[float, float, float]:
        return self.fk_inicial.posicion

    @property
    def posicion_final(self) -> tuple[float, float, float]:
        return self.fk_final.posicion

    def resumen(self) -> str:
        ini = self.pose_inicial.as_degrees()
        fin = self.pose_final.as_degrees()
        d = self.deltas_mm_deg
        pi = self.fk_inicial
        pf = self.fk_final

        lineas = [
            "=== SIMULACION DE MOVIMIENTO (solo calculo) ===",
            "Modelo: CoppeliaSim (SymPy)",
            "",
            "Movimiento aplicado:",
            f"  dp     : {d['dp_mm']:7.1f} mm  (posicion absoluta, 0-420)",
            f"  theta1 : {d['theta1_deg']:+7.1f} deg",
            f"  theta2 : {d['theta2_deg']:+7.1f} deg",
            f"  theta3 : {d['theta3_deg']:+7.1f} deg",
            f"  theta4 : {d['theta4_deg']:+7.1f} deg",
            f"  theta5 : {d['theta5_deg']:+7.1f} deg",
            f"  theta6 : {d['theta6_deg']:+7.1f} deg",
            "",
            "Pose inicial (juntas):",
            f"  dp={ini['dp_mm']:.1f} mm | "
            f"t1={ini['theta1_deg']:.1f} t2={ini['theta2_deg']:.1f} t3={ini['theta3_deg']:.1f} | "
            f"t4={ini['theta4_deg']:.1f} t5={ini['theta5_deg']:.1f} t6={ini['theta6_deg']:.1f} (deg)",
            "",
            "Posicion inicial:",
            f"  X = {pi.x:+.3f} mm   Y = {pi.y:+.3f} mm   Z = {pi.z:+.3f} mm",
            f"  Orientacion (alpha, beta, gamma): "
            f"{pi.alpha_deg:+.3f}°, {pi.beta_deg:+.3f}°, {pi.gamma_deg:+.3f}°",
            "",
            "Pose final (juntas):",
            f"  dp={fin['dp_mm']:.1f} mm | "
            f"t1={fin['theta1_deg']:.1f} t2={fin['theta2_deg']:.1f} t3={fin['theta3_deg']:.1f} | "
            f"t4={fin['theta4_deg']:.1f} t5={fin['theta5_deg']:.1f} t6={fin['theta6_deg']:.1f} (deg)",
            "",
            "Posicion final:",
            f"  X = {pf.x:+.3f} mm   Y = {pf.y:+.3f} mm   Z = {pf.z:+.3f} mm",
            f"  Orientacion (alpha, beta, gamma): "
            f"{pf.alpha_deg:+.3f}°, {pf.beta_deg:+.3f}°, {pf.gamma_deg:+.3f}°",
            "",
            "Desplazamiento cartesiano:",
            f"  dX = {pf.x - pi.x:+.3f} mm   dY = {pf.y - pi.y:+.3f} mm   dZ = {pf.z - pi.z:+.3f} mm",
        ]
        return "\n".join(lineas) + "\n"


def pose_home() -> JointState:
    return JointState(
        dp=0.0,
        theta1=0.0,
        theta2=0.0,
        theta3=0.0,
        theta4=0.0,
        theta5=0.0,
        theta6=0.0,
    )


def simular_movimiento_relativo(
    *,
    dp_mm: float = 0.0,
    theta1_deg: float = 0.0,
    theta2_deg: float = 0.0,
    theta3_deg: float = 0.0,
    theta4_deg: float = 0.0,
    theta5_deg: float = 0.0,
    theta6_deg: float = 0.0,
    pose_inicial: JointState | None = None,
) -> SimulacionJuntasResult:
    """Calcula pose y posicion final tras mover cada motor la cantidad indicada."""
    inicial = pose_inicial or pose_home()
    deltas = {
        "dp_mm": dp_mm,
        "theta1_deg": theta1_deg,
        "theta2_deg": theta2_deg,
        "theta3_deg": theta3_deg,
        "theta4_deg": theta4_deg,
        "theta5_deg": theta5_deg,
        "theta6_deg": theta6_deg,
    }
    final = JointState(
        dp=dp_mm,
        theta1=inicial.theta1 + math.radians(theta1_deg),
        theta2=inicial.theta2 + math.radians(theta2_deg),
        theta3=inicial.theta3 + math.radians(theta3_deg),
        theta4=inicial.theta4 + math.radians(theta4_deg),
        theta5=inicial.theta5 + math.radians(theta5_deg),
        theta6=inicial.theta6 + math.radians(theta6_deg),
    )
    fk_i = forward_kinematics_from_joints(inicial)
    fk_f = forward_kinematics_from_joints(final)
    return SimulacionJuntasResult(
        pose_inicial=inicial,
        pose_final=final,
        deltas_mm_deg=deltas,
        fk_inicial=fk_i,
        fk_final=fk_f,
    )
