"""
Simulación de trayectoria pick and place en espacio cartesiano.

Para cada waypoint (x, y, z) se resuelve cinemática inversa y se valida
la posición resultante con cinemática directa.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from robot_kinematics import IKResult, JointState
from robot_frame import (
    home_cartesian_coppelia,
    inverse_kinematics_coppelia,
    joint_home,
    position_coppelia,
)


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    z: float


@dataclass
class PlannedMove:
    waypoint: Waypoint
    ik: IKResult
    verified_xyz: tuple[float, float, float]


def _interpolate_waypoints(start: Waypoint, end: Waypoint, steps: int) -> list[Waypoint]:
    if steps < 2:
        return [end]

    points: list[Waypoint] = []
    for i in range(1, steps + 1):
        t = i / steps
        points.append(
            Waypoint(
                name=f"{start.name}->{end.name} [{i}/{steps}]",
                x=start.x + t * (end.x - start.x),
                y=start.y + t * (end.y - start.y),
                z=start.z + t * (end.z - start.z),
            )
        )
    return points


def plan_pick_and_place(
    pick_xyz: tuple[float, float, float],
    place_xyz: tuple[float, float, float],
    *,
    approach_offset_mm: float = 80.0,
    transit_z: float | None = None,
    segments_per_move: int = 5,
    preferred_pose: np.ndarray | None = None,
) -> list[PlannedMove]:
    """
    Genera una secuencia pick and place:
    home -> aproximar pick -> pick -> elevar -> aproximar place -> place -> elevar

    Coordenadas pick/place en marco CoppeliaSim (mm).
    """
    pick_x, pick_y, pick_z = pick_xyz
    place_x, place_y, place_z = place_xyz
    safe_z = transit_z if transit_z is not None else max(pick_z, place_z) + approach_offset_mm
    hx, hy, hz = home_cartesian_coppelia()

    keypoints = [
        Waypoint("home", hx, hy, hz),
        Waypoint("aproximar_pick", pick_x, pick_y, pick_z + approach_offset_mm),
        Waypoint("pick", pick_x, pick_y, pick_z),
        Waypoint("elevar_pick", pick_x, pick_y, safe_z),
        Waypoint("aproximar_place", place_x, place_y, safe_z),
        Waypoint("aproximar_place_baja", place_x, place_y, place_z + approach_offset_mm),
        Waypoint("place", place_x, place_y, place_z),
        Waypoint("elevar_final", place_x, place_y, safe_z),
    ]

    expanded: list[Waypoint] = [keypoints[0]]
    for start, end in zip(keypoints, keypoints[1:]):
        expanded.extend(_interpolate_waypoints(start, end, segments_per_move))

    plan: list[PlannedMove] = []
    last_successful = joint_home()

    for waypoint in expanded:
        seed = last_successful.as_array() if preferred_pose is None else preferred_pose
        ik = inverse_kinematics_coppelia(
            (waypoint.x, waypoint.y, waypoint.z),
            preferred_pose=seed,
        )
        if ik.success:
            last_successful = ik.joints

        verified = position_coppelia(ik.joints)
        plan.append(PlannedMove(waypoint=waypoint, ik=ik, verified_xyz=verified))

    return plan


def print_plan(plan: list[PlannedMove]) -> None:
    print("\n=== PLAN PICK AND PLACE ===")
    for i, move in enumerate(plan, start=1):
        wp = move.waypoint
        joints = move.ik.joints.as_degrees()
        status = "OK" if move.ik.success else "FALLÓ"
        print(f"\n[{i:02d}] {wp.name} ({status})")
        print(f"     Objetivo : x={wp.x:8.2f}  y={wp.y:8.2f}  z={wp.z:8.2f}")
        print(f"     Verificado: x={move.verified_xyz[0]:8.2f}  y={move.verified_xyz[1]:8.2f}  z={move.verified_xyz[2]:8.2f}")
        print(f"     Error IK  : {move.ik.position_error_mm:.3f} mm")
        print(
            "     Juntas    : "
            f"dp={joints['dp_mm']:6.1f} mm | "
            f"t1={joints['theta1_deg']:7.2f} t2={joints['theta2_deg']:7.2f} t3={joints['theta3_deg']:7.2f} | "
            f"t4={joints['theta4_deg']:7.2f} t5={joints['theta5_deg']:7.2f} t6={joints['theta6_deg']:7.2f} (deg)"
        )


if __name__ == "__main__":
    from robot_config import PICK_PLACE_DEFAULTS

    d = PICK_PLACE_DEFAULTS
    pick = (d["pick_x"], d["pick_y"], d["pick_z"])
    place = (d["place_x"], d["place_y"], d["place_z"])

    print(f"Pick  : {pick}")
    print(f"Place : {place}")

    trajectory = plan_pick_and_place(pick, place)
    print_plan(trajectory)

    failed = [m for m in trajectory if not m.ik.success]
    if failed:
        print(f"\nAdvertencia: {len(failed)} waypoint(s) no convergieron dentro de la tolerancia.")
    else:
        print("\nTodos los waypoints convergieron correctamente.")
