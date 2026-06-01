"""
Cliente serial para el brazo robotico.
Envia comandos al firmware en control_usb/firmware/brazo_robotico_serial/
"""

from __future__ import annotations

import math
import sys
import time

try:
    import serial
except ImportError as exc:
    raise ImportError("Instala pyserial: pip install pyserial") from exc

import config  # noqa: E402
from motion_utils import joint_delta_to_steps  # noqa: E402


class RobotSerialError(RuntimeError):
    pass


class RobotClient:
    def __init__(
        self,
        port: str | None = None,
        baudrate: int | None = None,
        timeout: float = 2.0,
    ) -> None:
        self.port = port or config.PUERTO_SERIAL
        self.baudrate = baudrate or config.BAUDRATE
        self._ser = serial.Serial(self.port, self.baudrate, timeout=timeout)
        time.sleep(2.0)
        self._vaciar_buffer()

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> "RobotClient":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def _vaciar_buffer(self) -> None:
        while self._ser.in_waiting:
            self._ser.readline()

    def _leer_linea(self, timeout: float) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            linea = raw.decode("utf-8", errors="ignore").strip()
            if linea:
                return linea
        raise RobotSerialError(f"Timeout esperando respuesta del robot ({timeout}s)")

    def ping(self) -> bool:
        self._ser.write(b"PING\n")
        respuesta = self._leer_linea(timeout=5.0)
        return respuesta == "PONG"

    def stop(self) -> None:
        self._ser.write(b"STOP\n")
        self._leer_linea(timeout=5.0)

    def move_steps(
        self,
        steps: list[int],
        *,
        timeout: float | None = None,
        verbose: bool = False,
    ) -> None:
        if len(steps) != 7:
            raise ValueError("Se requieren exactamente 7 valores de pasos.")

        comando = "G " + " ".join(str(s) for s in steps) + "\n"
        if verbose:
            print(f"  -> {comando.strip()}")

        self._ser.write(comando.encode("utf-8"))
        self._esperar_listo(timeout=timeout or config.TIMEOUT_MOVIMIENTO, verbose=verbose)

    def move_motor(self, motor: int, steps: int, *, timeout: float | None = None) -> None:
        if motor < 0 or motor > 6:
            raise ValueError("Motor debe estar entre 0 y 6.")
        comando = f"M {motor} {steps}\n"
        self._ser.write(comando.encode("utf-8"))
        self._esperar_listo(timeout=timeout or config.TIMEOUT_MOVIMIENTO)

    def _esperar_listo(self, timeout: float, verbose: bool = False) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            linea = self._leer_linea(timeout=max(0.1, deadline - time.monotonic()))
            if verbose and linea not in ("LISTO", "PONG"):
                print(f"  <- {linea}")
            if linea == "LISTO":
                return
            if linea.startswith("ERR"):
                raise RobotSerialError(linea)
        raise RobotSerialError(f"El robot no respondio LISTO en {timeout}s")
