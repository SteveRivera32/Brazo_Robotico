"""
Ejecuta pick-and-place en el robot real via USB.

Planifica la trayectoria en Python, calcula IK y envia pasos al firmware.

Uso (desde la carpeta control_usb):
  python ejecutar_pickplace.py --port COM3 --pick-x 650 --pick-y -300 --pick-z 120 --place-x 450 --place-y -500 --place-z 120

Prueba sin mover motores:
  python ejecutar_pickplace.py --dry-run --pick-x 650 --pick-y -300 --pick-z 120 --place-x 450 --place-y -500 --place-z 120
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from pick_and_place import plan_pick_and_place  # noqa: E402
from robot_kinematics import JointState  # noqa: E402

import config  # noqa: E402
from motion_utils import joint_delta_to_steps  # noqa: E402


def ejecutar_plan(
    plan,
    *,
    port: str | None,
    dry_run: bool = False,
    segments: int = 3,
    pause: float | None = None,
    log=print,
    force_on_ik_fail: bool = False,
) -> None:
    if not plan:
        log("Plan vacio.")
        return

    fallidos = [m for m in plan if not m.ik.success]
    if fallidos:
        log(f"Advertencia: {len(fallidos)} waypoint(s) no convergieron en IK.")
        if not force_on_ik_fail:
            respuesta = input("Continuar de todos modos? [s/N]: ").strip().lower()
            if respuesta != "s":
                log("Cancelado.")
                return

    cliente = None
    if not dry_run:
        from robot_client import RobotClient

        log(f"Conectando a {port or config.PUERTO_SERIAL}...")
        cliente = RobotClient(port=port)
        if not cliente.ping():
            raise RuntimeError("El robot no respondio PONG. Verifica firmware y puerto.")
        log("Conexion OK (PONG)\n")

    posicion_actual = JointState(dp=200.0, theta1=0.0, theta2=0.0, theta3=0.0, theta4=0.0, theta5=0.0, theta6=0.0)
    pausa = pause if pause is not None else config.PAUSA_ENTRE_PUNTOS

    log(f"Ejecutando {len(plan)} waypoints ({'simulacion' if dry_run else 'robot real'})...\n")

    for i, movimiento in enumerate(plan, start=1):
        wp = movimiento.waypoint
        objetivo = movimiento.ik.joints
        pasos = joint_delta_to_steps(posicion_actual, objetivo)
        posicion_actual = objetivo

        log(f"[{i:02d}/{len(plan)}] {wp.name}")
        log(f"       xyz objetivo: ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})")
        log(f"       pasos delta : {pasos}")

        if dry_run:
            continue

        assert cliente is not None
        cliente.move_steps(pasos, verbose=False)
        time.sleep(pausa)

    if cliente is not None:
        cliente.close()

    log("\nPick-and-place finalizado.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Ejecutar pick-and-place en el robot via USB")
    parser.add_argument("--port", type=str, default=None, help="Puerto COM (ej. COM3)")
    parser.add_argument("--pick-x", type=float, required=True)
    parser.add_argument("--pick-y", type=float, required=True)
    parser.add_argument("--pick-z", type=float, required=True)
    parser.add_argument("--place-x", type=float, required=True)
    parser.add_argument("--place-y", type=float, required=True)
    parser.add_argument("--place-z", type=float, required=True)
    parser.add_argument("--approach-offset", type=float, default=80.0)
    parser.add_argument("--segments", type=int, default=3, help="Puntos intermedios por tramo (3 = mas rapido)")
    parser.add_argument("--dry-run", action="store_true", help="Solo muestra pasos, no mueve el robot")
    parser.add_argument("--pause", type=float, default=None, help="Pausa entre puntos (segundos)")
    args = parser.parse_args()

    pick = (args.pick_x, args.pick_y, args.pick_z)
    place = (args.place_x, args.place_y, args.place_z)

    print("Calculando trayectoria...")
    print(f"  Pick  : {pick}")
    print(f"  Place : {place}")

    plan = plan_pick_and_place(
        pick,
        place,
        approach_offset_mm=args.approach_offset,
        segments_per_move=args.segments,
    )

    ejecutar_plan(
        plan,
        port=args.port,
        dry_run=args.dry_run,
        segments=args.segments,
        pause=args.pause,
    )


if __name__ == "__main__":
    main()
