"""
Cliente serial para el brazo robotico.
Envia comandos al firmware en control_usb/firmware/brazo_robotico_serial/
"""

from __future__ import annotations

import concurrent.futures
import time

try:
    import serial
    from serial.serialutil import SerialException
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
        timeout: float = 1.0,
        *,
        configure_speed: bool = True,
    ) -> None:
        self.port = port or config.PUERTO_SERIAL
        self.baudrate = baudrate or config.BAUDRATE
        self._ser = self._abrir_puerto(timeout)
        time.sleep(getattr(config, "ESPERA_PUERTO", 0.3))
        self._vaciar_buffer()
        if configure_speed:
            factor = getattr(config, "VELOCIDAD_FACTOR", None)
            if factor is not None:
                self.set_speed_factor(int(factor))

    def _abrir_puerto(self, timeout: float) -> serial.Serial:
        open_timeout = getattr(config, "TIMEOUT_APERTURA", 3.0)
        pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        future = pool.submit(
            serial.Serial,
            self.port,
            self.baudrate,
            timeout=timeout,
        )
        try:
            return future.result(timeout=open_timeout)
        except concurrent.futures.TimeoutError as exc:
            raise RobotSerialError(
                f"No se pudo abrir {self.port} en {open_timeout:.0f}s. "
                "Verifica que el robot este conectado y el puerto COM sea correcto."
            ) from exc
        except SerialException as exc:
            raise RobotSerialError(
                f"No se pudo abrir {self.port}: {exc}. "
                "Verifica conexion USB, puerto COM y que el Monitor Serial del Arduino este cerrado."
            ) from exc
        except OSError as exc:
            raise RobotSerialError(
                f"Error al acceder a {self.port}: {exc}"
            ) from exc
        finally:
            pool.shutdown(wait=False, cancel_futures=True)

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> "RobotClient":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def _vaciar_buffer(self, max_ms: float = 300) -> None:
        deadline = time.monotonic() + max_ms / 1000.0
        old_timeout = self._ser.timeout
        try:
            while time.monotonic() < deadline:
                if not self._ser.in_waiting:
                    break
                restante = deadline - time.monotonic()
                self._ser.timeout = max(0.05, min(restante, 0.1))
                self._ser.readline()
        finally:
            self._ser.timeout = old_timeout

    def _leer_linea(self, timeout: float) -> str:
        deadline = time.monotonic() + timeout
        old_timeout = self._ser.timeout
        try:
            while time.monotonic() < deadline:
                restante = deadline - time.monotonic()
                self._ser.timeout = max(0.05, min(restante, 1.0))
                raw = self._ser.readline()
                if not raw:
                    continue
                linea = raw.decode("utf-8", errors="ignore").strip()
                if linea:
                    return linea
        finally:
            self._ser.timeout = old_timeout
        raise RobotSerialError(f"Timeout esperando respuesta del robot ({timeout:.0f}s)")

    def set_speed_factor(self, factor: int) -> bool:
        """Ajusta velocidad de giro en la placa (F factor). Mayor = mas lento."""
        if factor < 1:
            factor = 1
        self._ser.write(f"F {factor}\n".encode("utf-8"))
        try:
            return self._leer_linea(timeout=1.0) == "OK FACTOR"
        except RobotSerialError:
            return False

    def ping(self) -> bool:
        self._vaciar_buffer(max_ms=100)
        self._ser.write(b"PING\n")
        try:
            respuesta = self._leer_linea(timeout=getattr(config, "TIMEOUT_PING", 3.0))
        except RobotSerialError:
            return False
        return respuesta == "PONG"

    def stop(self) -> None:
        if not self._ser.is_open:
            return
        self._ser.write(b"STOP\n")
        try:
            self._leer_linea(timeout=2.0)
        except RobotSerialError:
            pass

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
            restante = max(0.1, deadline - time.monotonic())
            linea = self._leer_linea(timeout=restante)
            if verbose and linea not in ("LISTO", "PONG"):
                print(f"  <- {linea}")
            if linea == "LISTO":
                return
            if linea.startswith("ERR"):
                raise RobotSerialError(linea)
        raise RobotSerialError(f"El robot no respondio LISTO en {timeout:.0f}s")
