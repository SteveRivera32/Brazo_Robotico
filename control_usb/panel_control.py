"""
Panel interactivo para probar conexion, motores y pick-and-place.

Uso (desde control_usb):
  python panel_control.py
"""

from __future__ import annotations

import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from ejecutar_pickplace import ejecutar_plan
from motion_utils import formatear_simulacion, pose_inicial_sim, simular_movimiento_relativo
from pick_and_place import plan_pick_and_place
from robot_client import RobotClient, RobotSerialError

MOTOR_LABELS = [
    "0 — Prismatica (dp)",
    "1 — Theta1",
    "2 — Theta2",
    "3 — Theta3",
    "4 — Theta4",
    "5 — Theta5",
    "6 — Theta6",
]


class PanelControl(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Brazo Robotico — Panel de control")
        self.minsize(760, 620)
        self._busy = False
        self._robot: RobotClient | None = None
        self._sim_slider_vars: list[tk.DoubleVar] = []
        self._sim_value_labels: list[ttk.Label] = []

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        main = ttk.Frame(self, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        notebook = ttk.Notebook(main)
        notebook.pack(fill=tk.BOTH, expand=True)

        tab_ops = ttk.Frame(notebook, padding=4)
        tab_sim = ttk.Frame(notebook, padding=4)
        notebook.add(tab_ops, text="Operaciones")
        notebook.add(tab_sim, text="Simular posicion")

        self._build_tab_operaciones(tab_ops)
        self._build_tab_simulacion(tab_sim)

        log_frame = ttk.LabelFrame(main, text="Salida", padding=4)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.log = scrolledtext.ScrolledText(log_frame, height=12, state=tk.DISABLED, wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)

        self._log("Panel listo. Configura el puerto COM y usa los botones para probar.\n")

    def _build_tab_operaciones(self, parent: ttk.Frame) -> None:
        conn = ttk.LabelFrame(parent, text="Conexion USB", padding=8)
        conn.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(conn, text="Puerto COM:").grid(row=0, column=0, sticky=tk.W)
        self.var_puerto = tk.StringVar(value=config.PUERTO_SERIAL)
        ttk.Entry(conn, textvariable=self.var_puerto, width=12).grid(row=0, column=1, padx=6, sticky=tk.W)
        self.btn_test = ttk.Button(conn, text="Probar conexion (PING)", command=self._run_test_conexion)
        self.btn_test.grid(row=0, column=2, padx=4)
        self.btn_stop = ttk.Button(conn, text="Paro de emergencia (STOP)", command=self._run_stop)
        self.btn_stop.grid(row=0, column=3, padx=4)

        motor = ttk.LabelFrame(parent, text="Prueba de motor", padding=8)
        motor.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(motor, text="Motor:").grid(row=0, column=0, sticky=tk.W)
        self.var_motor = tk.StringVar(value=MOTOR_LABELS[1])
        ttk.Combobox(
            motor,
            textvariable=self.var_motor,
            values=MOTOR_LABELS,
            state="readonly",
            width=22,
        ).grid(row=0, column=1, padx=6, sticky=tk.W)

        ttk.Label(motor, text="Pasos (+/-):").grid(row=0, column=2, sticky=tk.W, padx=(12, 0))
        self.var_pasos = tk.StringVar(value="200")
        ttk.Entry(motor, textvariable=self.var_pasos, width=10).grid(row=0, column=3, padx=6, sticky=tk.W)

        self.btn_motor = ttk.Button(motor, text="Mover motor", command=self._run_test_motor)
        self.btn_motor.grid(row=0, column=4, padx=4)

        pp = ttk.LabelFrame(parent, text="Pick and place", padding=8)
        pp.pack(fill=tk.X, pady=(0, 8))

        mode_row = ttk.Frame(pp)
        mode_row.pack(fill=tk.X, pady=(0, 6))
        self.var_modo_pp = tk.StringVar(value="simulacion")
        ttk.Radiobutton(
            mode_row,
            text="Simulacion (solo muestra pasos)",
            variable=self.var_modo_pp,
            value="simulacion",
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            mode_row,
            text="Ejecutar en robot real",
            variable=self.var_modo_pp,
            value="real",
        ).pack(side=tk.LEFT, padx=(16, 0))

        ttk.Button(mode_row, text="Cargar valores de prueba", command=self._cargar_defaults_pp).pack(
            side=tk.RIGHT
        )

        coords = ttk.Frame(pp)
        coords.pack(fill=tk.X)

        self._pp_fields: dict[str, tk.StringVar] = {}
        defaults = config.PICK_PLACE_DEFAULTS
        fields = [
            ("Pick X", "pick_x"),
            ("Pick Y", "pick_y"),
            ("Pick Z", "pick_z"),
            ("Place X", "place_x"),
            ("Place Y", "place_y"),
            ("Place Z", "place_z"),
            ("Aprox. (mm)", "approach_offset"),
            ("Segmentos", "segments"),
        ]

        for i, (label, key) in enumerate(fields):
            row, col = divmod(i, 4)
            cell = ttk.Frame(coords)
            cell.grid(row=row, column=col, padx=4, pady=4, sticky=tk.W)
            ttk.Label(cell, text=label).pack(anchor=tk.W)
            var = tk.StringVar(value=str(defaults[key]))
            self._pp_fields[key] = var
            ttk.Entry(cell, textvariable=var, width=10).pack()

        self.btn_pp = ttk.Button(pp, text="Ejecutar pick and place", command=self._run_pickplace)
        self.btn_pp.pack(anchor=tk.W, pady=(8, 0))

    def _build_tab_simulacion(self, parent: ttk.Frame) -> None:
        info = ttk.LabelFrame(parent, text="Movimiento relativo por motor", padding=8)
        info.pack(fill=tk.X, pady=(0, 8))

        inicial = pose_inicial_sim().as_degrees()
        ttk.Label(
            info,
            text=(
                "Ajusta cuanto se movera cada motor desde la pose inicial. "
                "En simulacion solo calcula la posicion; en robot real envia los pasos por USB."
            ),
            wraplength=700,
        ).pack(anchor=tk.W)
        ttk.Label(
            info,
            text=(
                f"Pose inicial: dp={inicial['dp_mm']:.0f} mm, "
                f"theta1..6 = {inicial['theta1_deg']:.0f}, {inicial['theta2_deg']:.0f}, "
                f"{inicial['theta3_deg']:.0f}, {inicial['theta4_deg']:.0f}, "
                f"{inicial['theta5_deg']:.0f}, {inicial['theta6_deg']:.0f} deg"
            ),
        ).pack(anchor=tk.W, pady=(4, 0))

        mode_row = ttk.Frame(info)
        mode_row.pack(fill=tk.X, pady=(8, 0))
        self.var_modo_sim = tk.StringVar(value="simulacion")
        ttk.Radiobutton(
            mode_row,
            text="Simulacion (solo calculo FK)",
            variable=self.var_modo_sim,
            value="simulacion",
            command=self._actualizar_modo_sim,
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            mode_row,
            text="Ejecutar en robot real",
            variable=self.var_modo_sim,
            value="real",
            command=self._actualizar_modo_sim,
        ).pack(side=tk.LEFT, padx=(16, 0))

        sliders = ttk.LabelFrame(parent, text="Desplazamiento por motor", padding=8)
        sliders.pack(fill=tk.BOTH, expand=True)

        dp_min, dp_max = config.SIM_SLIDER_RANGOS["dp_mm"]
        th_min, th_max = config.SIM_SLIDER_RANGOS["theta_deg"]

        slider_specs = [
            ("Prismatica (dp)", "mm", dp_min, dp_max, 1.0),
            ("Theta1", "deg", th_min, th_max, 1.0),
            ("Theta2", "deg", th_min, th_max, 1.0),
            ("Theta3", "deg", th_min, th_max, 1.0),
            ("Theta4", "deg", th_min, th_max, 1.0),
            ("Theta5", "deg", th_min, th_max, 1.0),
            ("Theta6", "deg", th_min, th_max, 1.0),
        ]

        self._sim_slider_vars.clear()
        self._sim_value_labels.clear()

        for i, (nombre, unidad, vmin, vmax, res) in enumerate(slider_specs):
            row = ttk.Frame(sliders)
            row.pack(fill=tk.X, pady=3)

            ttk.Label(row, text=nombre, width=16).pack(side=tk.LEFT)
            var = tk.DoubleVar(value=0.0)
            self._sim_slider_vars.append(var)

            def make_callback(idx: int, unit: str) -> callable:
                def on_change(_value: str) -> None:
                    val = self._sim_slider_vars[idx].get()
                    self._sim_value_labels[idx].configure(text=f"{val:+.0f} {unit}")

                return on_change

            scale = ttk.Scale(
                row,
                from_=vmin,
                to=vmax,
                orient=tk.HORIZONTAL,
                variable=var,
                command=make_callback(i, unidad),
            )
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

            val_label = ttk.Label(row, text=f"+0 {unidad}", width=12)
            val_label.pack(side=tk.RIGHT)
            self._sim_value_labels.append(val_label)

        actions = ttk.Frame(parent)
        actions.pack(fill=tk.X, pady=(8, 0))
        self.btn_sim_reset = ttk.Button(actions, text="Restablecer barras", command=self._reset_sim_sliders)
        self.btn_sim_reset.pack(side=tk.LEFT)
        self.btn_sim = ttk.Button(actions, text="Calcular posicion final", command=self._run_simulacion_posicion)
        self.btn_sim.pack(side=tk.LEFT, padx=(8, 0))

        self.lbl_sim_resultado = ttk.Label(
            parent,
            text="Posicion final: —",
            font=("Segoe UI", 10, "bold"),
            wraplength=700,
        )
        self.lbl_sim_resultado.pack(anchor=tk.W, pady=(10, 0))

    def _actualizar_modo_sim(self) -> None:
        if self.var_modo_sim.get() == "real":
            self.btn_sim.configure(text="Ejecutar en robot")
        else:
            self.btn_sim.configure(text="Calcular posicion final")

    def _cargar_defaults_pp(self) -> None:
        for key, var in self._pp_fields.items():
            var.set(str(config.PICK_PLACE_DEFAULTS[key]))
        self._log("Valores de prueba cargados.\n")

    def _log(self, text: str) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, text)
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def _set_busy(self, busy: bool) -> None:
        self._busy = busy
        state = tk.DISABLED if busy else tk.NORMAL
        for btn in (self.btn_test, self.btn_motor, self.btn_pp, self.btn_stop, self.btn_sim):
            btn.configure(state=state)

    def _run_in_thread(self, target) -> None:
        if self._busy:
            return

        def wrapper() -> None:
            self._set_busy(True)
            try:
                target()
            except Exception as exc:
                self.after(0, lambda: self._log(f"\nERROR: {exc}\n"))
                self.after(0, lambda: messagebox.showerror("Error", str(exc)))
            finally:
                self.after(0, lambda: self._set_busy(False))

        threading.Thread(target=wrapper, daemon=True).start()

    def _puerto(self) -> str:
        return self.var_puerto.get().strip() or config.PUERTO_SERIAL

    def _run_test_conexion(self) -> None:
        def task() -> None:
            puerto = self._puerto()
            self.after(0, lambda: self._log(f"\n--- Test conexion ({puerto}) ---\n"))
            with RobotClient(port=puerto) as robot:
                if robot.ping():
                    self.after(0, lambda: self._log("OK — El robot respondio PONG\n"))
                else:
                    raise RobotSerialError("Respuesta inesperada al PING")

        self._run_in_thread(task)

    def _motor_index(self) -> int:
        label = self.var_motor.get()
        for i, name in enumerate(MOTOR_LABELS):
            if name == label:
                return i
        return int(label.split("—")[0].strip())

    def _run_test_motor(self) -> None:
        def task() -> None:
            puerto = self._puerto()
            motor = self._motor_index()
            pasos = int(self.var_pasos.get().strip())
            self.after(
                0,
                lambda: self._log(f"\n--- Motor {motor} → {pasos} pasos ({puerto}) ---\n"),
            )
            with RobotClient(port=puerto) as robot:
                if not robot.ping():
                    raise RobotSerialError("FALLO PING")
                robot.move_motor(motor, pasos)
            self.after(0, lambda: self._log("LISTO\n"))

        try:
            int(self.var_pasos.get().strip())
        except ValueError:
            messagebox.showerror("Datos invalidos", "Los pasos deben ser un numero entero.")
            return
        self._run_in_thread(task)

    def _leer_pp_coords(self) -> tuple[tuple[float, float, float], tuple[float, float, float], float, int]:
        try:
            pick = (
                float(self._pp_fields["pick_x"].get()),
                float(self._pp_fields["pick_y"].get()),
                float(self._pp_fields["pick_z"].get()),
            )
            place = (
                float(self._pp_fields["place_x"].get()),
                float(self._pp_fields["place_y"].get()),
                float(self._pp_fields["place_z"].get()),
            )
            approach = float(self._pp_fields["approach_offset"].get())
            segments = int(self._pp_fields["segments"].get())
        except ValueError as exc:
            raise ValueError("Revisa las coordenadas: deben ser numeros validos.") from exc
        if segments < 1:
            raise ValueError("Segmentos debe ser al menos 1.")
        return pick, place, approach, segments

    def _run_pickplace(self) -> None:
        try:
            pick, place, approach, segments = self._leer_pp_coords()
        except ValueError as exc:
            messagebox.showerror("Datos invalidos", str(exc))
            return

        dry_run = self.var_modo_pp.get() == "simulacion"
        puerto = self._puerto()

        def task() -> None:
            modo = "simulacion" if dry_run else "robot real"
            self.after(0, lambda: self._log(f"\n--- Pick and place ({modo}) ---\n"))
            self.after(0, lambda: self._log(f"  Pick  : {pick}\n"))
            self.after(0, lambda: self._log(f"  Place : {place}\n"))

            plan = plan_pick_and_place(
                pick,
                place,
                approach_offset_mm=approach,
                segments_per_move=segments,
            )

            fallidos = [m for m in plan if not m.ik.success]
            force = False
            if fallidos:
                event = threading.Event()
                choice: list[bool] = [False]

                def preguntar() -> None:
                    choice[0] = messagebox.askyesno(
                        "Advertencia IK",
                        f"{len(fallidos)} waypoint(s) no convergieron.\n"
                        "¿Continuar de todos modos?",
                    )
                    event.set()

                self.after(0, preguntar)
                event.wait()
                if not choice[0]:
                    self.after(0, lambda: self._log("Cancelado por el usuario.\n"))
                    return
                force = True

            def log_threadsafe(msg: str) -> None:
                self.after(0, lambda m=msg: self._log(m if m.endswith("\n") else m + "\n"))

            ejecutar_plan(
                plan,
                port=puerto,
                dry_run=dry_run,
                segments=segments,
                log=log_threadsafe,
                force_on_ik_fail=force,
            )

        self._run_in_thread(task)

    def _reset_sim_sliders(self) -> None:
        unidades = ["mm", "deg", "deg", "deg", "deg", "deg", "deg"]
        for i, var in enumerate(self._sim_slider_vars):
            var.set(0.0)
            self._sim_value_labels[i].configure(text=f"+0 {unidades[i]}")
        self.lbl_sim_resultado.configure(text="Posicion final: —")
        self._log("Barras de simulacion restablecidas a cero.\n")

    def _run_simulacion_posicion(self) -> None:
        delta_dp = self._sim_slider_vars[0].get()
        delta_thetas = tuple(var.get() for var in self._sim_slider_vars[1:])
        ejecutar_real = self.var_modo_sim.get() == "real"

        def task() -> None:
            try:
                resultado = simular_movimiento_relativo(delta_dp, delta_thetas)
            except Exception as exc:
                self.after(0, lambda: messagebox.showerror("Error de simulacion", str(exc)))
                return

            x, y, z = resultado.posicion_xyz
            ejecutado = False

            if ejecutar_real:
                if all(p == 0 for p in resultado.pasos_motor):
                    self.after(
                        0,
                        lambda: messagebox.showinfo(
                            "Sin movimiento",
                            "Todos los desplazamientos estan en cero. No hay nada que enviar al robot.",
                        ),
                    )
                else:
                    event = threading.Event()
                    confirmar: list[bool] = [False]

                    def preguntar() -> None:
                        confirmar[0] = messagebox.askyesno(
                            "Confirmar movimiento",
                            "Se enviaran estos pasos al robot:\n"
                            f"{resultado.pasos_motor}\n\n"
                            "¿Continuar?",
                        )
                        event.set()

                    self.after(0, preguntar)
                    event.wait()
                    if confirmar[0]:
                        puerto = self._puerto()
                        self.after(
                            0,
                            lambda: self._log(f"\n--- Movimiento articulado ({puerto}) ---\n"),
                        )
                        with RobotClient(port=puerto) as robot:
                            if not robot.ping():
                                raise RobotSerialError("FALLO PING — verifica conexion y firmware.")
                            self.after(
                                0,
                                lambda: self._log(f"Pasos enviados: {resultado.pasos_motor}\n"),
                            )
                            robot.move_steps(resultado.pasos_motor)
                        ejecutado = True
                        self.after(0, lambda: self._log("LISTO — movimiento completado.\n"))
                    else:
                        self.after(0, lambda: self._log("Ejecucion cancelada por el usuario.\n"))

            texto = formatear_simulacion(resultado, ejecutado=ejecutado)
            self.after(
                0,
                lambda: self.lbl_sim_resultado.configure(
                    text=f"Posicion final: x={x:.1f} mm  |  y={y:.1f} mm  |  z={z:.1f} mm"
                ),
            )
            self.after(0, lambda: self._log(f"\n{texto}\n"))

        self._run_in_thread(task)

    def _run_stop(self) -> None:
        def task() -> None:
            puerto = self._puerto()
            self.after(0, lambda: self._log(f"\n--- Paro de emergencia ({puerto}) ---\n"))
            with RobotClient(port=puerto) as robot:
                robot.stop()
            self.after(0, lambda: self._log("STOP enviado.\n"))

        self._run_in_thread(task)

    def _on_close(self) -> None:
        if self._robot is not None:
            self._robot.close()
        self.destroy()


def main() -> None:
    app = PanelControl()
    app.mainloop()


if __name__ == "__main__":
    main()
