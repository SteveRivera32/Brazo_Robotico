"""
Punto de entrada del brazo robótico.

- Cinemática directa (FK): juntas -> posición (x, y, z)
- Cinemática inversa (IK): posición (x, y, z) -> juntas
- Simulación pick and place

Uso:
  python main.py fk --dp 200 --theta1 12 --theta2 14 --theta3 22 --theta4 12 --theta5 10 --theta6 22
  python main.py ik --x 650 --y -300 --z 120
  python main.py pickplace --pick-x 650 --pick-y -300 --pick-z 120 --place-x 450 --place-y -500 --place-z 120
  python main.py symbolic   # genera las ecuaciones simbólicas (SymPy)
"""

from __future__ import annotations

import argparse
import math

from pick_and_place import plan_pick_and_place, print_plan
from robot_kinematics import JointState, inverse_kinematics_position, joints_to_motor_steps, position_from_joints


def _parse_fk(args: argparse.Namespace) -> None:
    joints = JointState(
        dp=args.dp,
        theta1=math.radians(args.theta1),
        theta2=math.radians(args.theta2),
        theta3=math.radians(args.theta3),
        theta4=math.radians(args.theta4),
        theta5=math.radians(args.theta5),
        theta6=math.radians(args.theta6),
    )
    pos = position_from_joints(joints)
    print("\n=== CINEMÁTICA DIRECTA ===")
    print(f"Entrada (deg): dp={args.dp} mm, t1={args.theta1}, t2={args.theta2}, t3={args.theta3}, "
          f"t4={args.theta4}, t5={args.theta5}, t6={args.theta6}")
    print(f"Posición   : x={pos[0]:.3f} mm, y={pos[1]:.3f} mm, z={pos[2]:.3f} mm")


def _parse_ik(args: argparse.Namespace) -> None:
    result = inverse_kinematics_position((args.x, args.y, args.z))
    joints = result.joints.as_degrees()
    verified = position_from_joints(result.joints)
    steps = joints_to_motor_steps(result.joints)

    print("\n=== CINEMÁTICA INVERSA ===")
    print(f"Objetivo   : x={args.x:.3f} mm, y={args.y:.3f} mm, z={args.z:.3f} mm")
    print(f"Estado     : {'OK' if result.success else 'FALLÓ'} ({result.message})")
    print(f"Error pos. : {result.position_error_mm:.3f} mm")
    print(
        "Juntas     : "
        f"dp={joints['dp_mm']:.2f} mm | "
        f"t1={joints['theta1_deg']:.2f} t2={joints['theta2_deg']:.2f} t3={joints['theta3_deg']:.2f} | "
        f"t4={joints['theta4_deg']:.2f} t5={joints['theta5_deg']:.2f} t6={joints['theta6_deg']:.2f} (deg)"
    )
    print(f"Verificado : x={verified[0]:.3f} mm, y={verified[1]:.3f} mm, z={verified[2]:.3f} mm")
    print(f"Pasos mot. : {steps}  (calibrar steps_per_mm / steps_per_degree en robot_kinematics.py)")


def _parse_pickplace(args: argparse.Namespace) -> None:
    pick = (args.pick_x, args.pick_y, args.pick_z)
    place = (args.place_x, args.place_y, args.place_z)
    plan = plan_pick_and_place(
        pick,
        place,
        approach_offset_mm=args.approach_offset,
        segments_per_move=args.segments,
    )
    print_plan(plan)


def _parse_symbolic() -> None:
    import sympy as sp

    dp = sp.symbols("d_p")
    theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols(
        "theta1 theta2 theta3 theta4 theta5 theta6"
    )
    L1, L2, L3, L4, L5, L6, L7 = 235, 154, 145, 224, 145, 224, 52

    def dh(a, alpha, d, theta):
        return sp.Matrix(
            [
                [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
                [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
                [0, sp.sin(alpha), sp.cos(alpha), d],
                [0, 0, 0, 1],
            ]
        )

    tabla_dh = [
        [0, sp.pi / 2, dp, 0],
        [0, sp.pi / 2, L1, theta1],
        [L2, -sp.pi / 2, 0, theta2],
        [L3, sp.pi / 2, 0, theta3],
        [L4, -sp.pi / 2, 0, theta4],
        [L5, sp.pi / 2, 0, theta5],
        [L6, -sp.pi / 2, 0, theta6],
        [L7, 0, 0, 0],
    ]

    t = sp.eye(4)
    for fila in tabla_dh:
        t = t * dh(*fila)

    posicion = sp.simplify(t[0:3, 3])
    print("\n=== ECUACIONES SIMBÓLICAS (FK) ===")
    print("Posición del efector final [x, y, z]:")
    sp.pprint(posicion)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Cinemática y simulación del brazo de 7 GDL")
    sub = parser.add_subparsers(dest="command", required=True)

    fk = sub.add_parser("fk", help="Cinemática directa: juntas -> posición")
    fk.add_argument("--dp", type=float, default=200.0)
    fk.add_argument("--theta1", type=float, default=0.0)
    fk.add_argument("--theta2", type=float, default=0.0)
    fk.add_argument("--theta3", type=float, default=0.0)
    fk.add_argument("--theta4", type=float, default=0.0)
    fk.add_argument("--theta5", type=float, default=0.0)
    fk.add_argument("--theta6", type=float, default=0.0)
    fk.set_defaults(func=_parse_fk)

    ik = sub.add_parser("ik", help="Cinemática inversa: posición -> juntas")
    ik.add_argument("--x", type=float, required=True)
    ik.add_argument("--y", type=float, required=True)
    ik.add_argument("--z", type=float, required=True)
    ik.set_defaults(func=_parse_ik)

    pp = sub.add_parser("pickplace", help="Simular trayectoria pick and place")
    pp.add_argument("--pick-x", type=float, required=True)
    pp.add_argument("--pick-y", type=float, required=True)
    pp.add_argument("--pick-z", type=float, required=True)
    pp.add_argument("--place-x", type=float, required=True)
    pp.add_argument("--place-y", type=float, required=True)
    pp.add_argument("--place-z", type=float, required=True)
    pp.add_argument("--approach-offset", type=float, default=80.0)
    pp.add_argument("--segments", type=int, default=5)
    pp.set_defaults(func=_parse_pickplace)

    sym = sub.add_parser("symbolic", help="Mostrar ecuaciones simbólicas con SymPy")
    sym.set_defaults(func=lambda _args: _parse_symbolic())

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
