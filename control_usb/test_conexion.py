"""
Prueba rapida de conexion USB con el robot.

Uso:
  python test_conexion.py --port COM3
"""

from __future__ import annotations

import argparse
import sys

import serial

import config
from robot_client import RobotClient, RobotSerialError


def main() -> None:
    parser = argparse.ArgumentParser(description="Probar conexion serial con el robot")
    parser.add_argument("--port", type=str, default=None, help="Puerto COM (ej. COM3)")
    args = parser.parse_args()

    puerto = args.port or config.PUERTO_SERIAL
    print(f"Puerto: {puerto} @ {config.BAUDRATE} baud")

    try:
        with RobotClient(port=puerto) as robot:
            if robot.ping():
                print("OK - El robot respondio PONG")
                print("Firmware listo para recibir comandos G / M")
            else:
                print("FALLO - Respuesta inesperada al PING")
                sys.exit(1)
    except RobotSerialError as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)
    except serial.SerialException as exc:
        print(f"ERROR serial: {exc}")
        print("Verifica el puerto COM y que el Monitor Serial del Arduino IDE este cerrado.")
        sys.exit(1)


if __name__ == "__main__":
    main()
