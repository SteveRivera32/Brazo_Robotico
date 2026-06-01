"""
Prueba de un solo motor (util para calibrar pasos/mm y pasos/grado).

Uso:
  python test_motor.py --port COM3 --motor 1 --pasos 500
  python test_motor.py --port COM3 --motor 0 --pasos -200
"""

from __future__ import annotations

import argparse
import sys

import config
from robot_client import RobotClient, RobotSerialError


def main() -> None:
    parser = argparse.ArgumentParser(description="Mover un motor una cantidad de pasos")
    parser.add_argument("--port", type=str, default=None)
    parser.add_argument("--motor", type=int, required=True, help="Indice 0..6 (0=prismatica)")
    parser.add_argument("--pasos", type=int, required=True, help="Pasos (+ o -)")
    args = parser.parse_args()

    if args.motor < 0 or args.motor > 6:
        print("Motor debe ser 0..6")
        sys.exit(1)

    puerto = args.port or config.PUERTO_SERIAL
    print(f"Moviendo motor {args.motor} -> {args.pasos} pasos")

    try:
        with RobotClient(port=puerto) as robot:
            if not robot.ping():
                print("FALLO PING")
                sys.exit(1)
            robot.move_motor(args.motor, args.pasos)
            print("LISTO")
    except RobotSerialError as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)


if __name__ == "__main__":
    main()
