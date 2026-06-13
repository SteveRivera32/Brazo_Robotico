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
from pick_and_place import plan_pick_and_place
from motion_utils import joint_delta_to_steps
from robot_client import RobotClient, RobotSerialError
from simular_juntas import simular_movimiento_relativo

MOTOR_LABELS = [
    "0 — Prismatica (dp)",
    "1 — Theta1",
    "2 — Theta2",
    "3 — Theta3",
    "4 — Theta4",
    "5 — Theta5",
    "6 — Theta6",
]

SIM_JOINT_SPECS = [
    ("dp", "Prismatica (dp, 0-420 mm)", "mm", "dp_mm"),
    ("theta1", "Theta1", "deg", "theta_delta_deg"),
    ("theta2", "Theta2", "deg", "theta_delta_deg"),
    ("theta3", "Theta3", "deg", "theta_delta_deg"),
    ("theta4", "Theta4", "deg", "theta_delta_deg"),
    ("theta5", "Theta5", "deg", "theta_delta_deg"),
    ("theta6", "Theta6", "deg", "theta_delta_deg"),
]


class PanelControl(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Brazo Robotico — Panel de control")
        self.minsize(980, 620)
        self._busy = False
        self._robot: RobotClient | None = None
        self._sim_scales: dict[str, tk.Scale] = {}
        self._sim_vars: dict[str, tk.IntVar] = {}
        self._sim_limits: dict[str, tuple[int, int]] = {}
        self._sim_snap: dict[str, int] = {}
        self._sim_scale_ancho = 560

        self._build_ui()
        self._centrar_ventana()
        self.bind("<Configure>", self._resize_sim_scales)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _centrar_ventana(self) -> None:
        self.update_idletasks()
        ancho = max(self.winfo_width(), self.minsize()[0])
        alto = max(self.winfo_height(), self.minsize()[1])
        x = (self.winfo_screenwidth() - ancho) // 2
        y = (self.winfo_screenheight() - alto) // 2
        self.geometry(f"{ancho}x{alto}+{x}+{y}")

    def _build_ui(self) -> None:
        main = ttk.Frame(self, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        notebook = ttk.Notebook(main)
        notebook.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        tab_control = ttk.Frame(notebook, padding=4)
        tab_sim = ttk.Frame(notebook, padding=4)
        notebook.add(tab_control, text="Control USB")
        notebook.add(tab_sim, text="Simular juntas")

        self._build_tab_control(tab_control)
        self._build_tab_simular(tab_sim)

        log_frame = ttk.LabelFrame(main, text="Salida", padding=4)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log = scrolledtext.ScrolledText(log_frame, height=14, state=tk.DISABLED, wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)

        self._log("Panel listo. Configura el puerto COM y usa las pestanas para probar.\n")

    def _build_tab_control(self, parent: ttk.Frame) -> None:
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
        pp.pack(fill=tk.X)

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

    def _build_tab_simular(self, parent: ttk.Frame) -> None:
        intro = ttk.Label(
            parent,
            text=(
                "Ajusta la prismática (dp) de 0 a 420 mm y el movimiento relativo "
                "de cada eje rotacional desde la pose home. La barra encaja en cada "
                "número entero; también puedes escribir el valor a mano o usar la rueda "
                "del ratón. Usa «Calcular posicion final» para simular o "
                "«Mover robot» para ejecutar el movimiento en el brazo real."
            ),
            wraplength=900,
        )
        intro.pack(anchor=tk.W, pady=(0, 8))

        sliders = ttk.LabelFrame(parent, text="Movimiento relativo por motor", padding=8)
        sliders.pack(fill=tk.BOTH, expand=True)

        limits = config.SIMULACION_JUNTAS_LIMITS

        snap_cfg = getattr(config, "SIM_SNAP_BARRA", {"dp_mm": 10, "theta_delta_deg": 5})

        for row, (key, label, unit, limit_key) in enumerate(SIM_JOINT_SPECS):
            lo, hi = (int(limits[limit_key][0]), int(limits[limit_key][1]))
            snap = int(snap_cfg.get(limit_key, 5))
            self._sim_limits[key] = (lo, hi)
            self._sim_snap[key] = snap
            var = tk.IntVar(value=0)
            self._sim_vars[key] = var

            row_frame = ttk.Frame(sliders)
            row_frame.pack(fill=tk.X, pady=4)

            ttk.Label(row_frame, text=label, width=16).pack(side=tk.LEFT)

            scale = tk.Scale(
                row_frame,
                from_=lo,
                to=hi,
                orient=tk.HORIZONTAL,
                resolution=snap,
                showvalue=False,
                length=560,
                sliderlength=36,
                tickinterval=50 if key == "dp" else 45,
                bigincrement=10 if key == "dp" else 15,
                variable=var,
                command=lambda _val, k=key: self._snap_sim_var(k),
            )
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 4))
            scale.bind("<MouseWheel>", lambda e, k=key: self._on_sim_wheel(e, k))
            scale.bind("<ButtonRelease-1>", lambda _e, k=key: self._snap_sim_var(k))
            self._sim_scales[key] = scale

            spin = ttk.Spinbox(
                row_frame,
                from_=lo,
                to=hi,
                increment=snap,
                textvariable=var,
                width=7,
                justify=tk.RIGHT,
            )
            spin.pack(side=tk.LEFT, padx=(0, 2))
            spin.bind("<Return>", lambda _e, k=key: self._clamp_sim_var(k))
            spin.bind("<FocusOut>", lambda _e, k=key: self._clamp_sim_var(k))
            ttk.Label(row_frame, text=unit, width=4).pack(side=tk.LEFT)

        btn_row = ttk.Frame(parent)
        btn_row.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(btn_row, text="Restablecer barras", command=self._reset_sim_sliders).pack(side=tk.LEFT)
        self.btn_simular = ttk.Button(
            btn_row,
            text="Calcular posicion final",
            command=self._run_simular_juntas,
        )
        self.btn_simular.pack(side=tk.LEFT, padx=(8, 0))
        self.btn_mover_sim = ttk.Button(
            btn_row,
            text="Mover robot",
            command=self._run_mover_sim_juntas,
        )
        self.btn_mover_sim.pack(side=tk.LEFT, padx=(8, 0))

        self.var_sim_resultado = tk.StringVar(value="Posicion final: —")
        ttk.Label(
            parent,
            textvariable=self.var_sim_resultado,
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor=tk.W, pady=(10, 0))
        self.after_idle(self._resize_sim_scales)

    def _resize_sim_scales(self, event: tk.Event | None = None) -> None:
        if event is not None and event.widget is not self:
            return
        nuevo = min(max(self.winfo_width() - 280, 480), 720)
        if nuevo == self._sim_scale_ancho:
            return
        self._sim_scale_ancho = nuevo
        for scale in self._sim_scales.values():
            scale.configure(length=nuevo)

    def _clamp_sim_var(self, key: str) -> None:
        lo, hi = self._sim_limits[key]
        var = self._sim_vars[key]
        try:
            valor = int(round(float(var.get())))
        except (tk.TclError, ValueError):
            valor = 0
        var.set(max(lo, min(hi, valor)))

    def _snap_sim_var(self, key: str) -> None:
        lo, hi = self._sim_limits[key]
        snap = self._sim_snap[key]
        var = self._sim_vars[key]
        try:
            valor = int(var.get())
        except (tk.TclError, ValueError):
            valor = 0
        encajado = int(round(valor / snap) * snap)
        var.set(max(lo, min(hi, encajado)))

    def _on_sim_wheel(self, event: tk.Event, key: str) -> None:
        lo, hi = self._sim_limits[key]
        snap = self._sim_snap[key]
        paso = snap if event.delta > 0 else -snap
        var = self._sim_vars[key]
        var.set(max(lo, min(hi, int(var.get()) + paso)))

    def _reset_sim_sliders(self) -> None:
        for key in self._sim_vars:
            self._sim_vars[key].set(0)

    def _leer_deltas_sim(self) -> dict[str, float]:
        for key in self._sim_vars:
            self._clamp_sim_var(key)
        return {
            "dp_mm": float(self._sim_vars["dp"].get()),
            "theta1_deg": float(self._sim_vars["theta1"].get()),
            "theta2_deg": float(self._sim_vars["theta2"].get()),
            "theta3_deg": float(self._sim_vars["theta3"].get()),
            "theta4_deg": float(self._sim_vars["theta4"].get()),
            "theta5_deg": float(self._sim_vars["theta5"].get()),
            "theta6_deg": float(self._sim_vars["theta6"].get()),
        }

    def _run_simular_juntas(self) -> None:
        d = self._leer_deltas_sim()
        resultado = simular_movimiento_relativo(**d)
        pf = resultado.fk_final
        self.var_sim_resultado.set(
            f"Posicion final: X={pf.x:+.1f}  Y={pf.y:+.1f}  Z={pf.z:+.1f} mm  |  "
            f"a={pf.alpha_deg:+.1f}°  b={pf.beta_deg:+.1f}°  g={pf.gamma_deg:+.1f}°"
        )
        self._log(f"\n{resultado.resumen()}")

    def _run_mover_sim_juntas(self) -> None:
        d = self._leer_deltas_sim()
        resultado = simular_movimiento_relativo(**d)
        pasos = joint_delta_to_steps(resultado.pose_inicial, resultado.pose_final)

        if not any(pasos):
            messagebox.showinfo(
                "Sin movimiento",
                "Todos los valores estan en cero; no hay nada que mover.",
            )
            return

        resumen = (
            f"dp={d['dp_mm']:.0f} mm | "
            f"t1={d['theta1_deg']:+.0f} t2={d['theta2_deg']:+.0f} t3={d['theta3_deg']:+.0f} | "
            f"t4={d['theta4_deg']:+.0f} t5={d['theta5_deg']:+.0f} t6={d['theta6_deg']:+.0f} (deg)\n\n"
            f"Pasos a enviar: {pasos}\n\n"
            "¿Mover el robot con estos valores?"
        )
        if not messagebox.askyesno("Confirmar movimiento", resumen):
            return

        puerto = self._puerto()

        def task() -> None:
            self.after(0, lambda: self._log(f"\n--- Mover juntas ({puerto}) ---\n"))
            self.after(0, lambda: self._log(f"  Deltas : {d}\n"))
            self.after(0, lambda: self._log(f"  Pasos  : {pasos}\n"))
            with RobotClient(port=puerto) as robot:
                if not robot.ping():
                    raise RobotSerialError("FALLO PING")
                robot.move_steps(pasos)
            pf = resultado.fk_final
            self.after(
                0,
                lambda: self.var_sim_resultado.set(
                    f"Posicion final: X={pf.x:+.1f}  Y={pf.y:+.1f}  Z={pf.z:+.1f} mm  |  "
                    f"a={pf.alpha_deg:+.1f}°  b={pf.beta_deg:+.1f}°  g={pf.gamma_deg:+.1f}°"
                ),
            )
            self.after(0, lambda: self._log("LISTO\n"))

        self._run_in_thread(task)

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
        for btn in (self.btn_test, self.btn_motor, self.btn_pp, self.btn_simular, self.btn_mover_sim):
            btn.configure(state=state)
        # STOP siempre disponible aunque otra operacion este en curso

    def _report_error(self, msg: str) -> None:
        self._log(f"\nERROR: {msg}\n")
        messagebox.showerror("Error", msg)

    def _clear_busy(self) -> None:
        self._set_busy(False)

    def _run_in_thread(self, target, *, allow_while_busy: bool = False) -> None:
        if self._busy and not allow_while_busy:
            return

        def wrapper() -> None:
            if not allow_while_busy:
                self._set_busy(True)
            try:
                target()
            except Exception as exc:
                msg = str(exc)
                self.after(0, lambda m=msg: self._report_error(m))
            finally:
                if not allow_while_busy:
                    self.after(0, self._clear_busy)

        threading.Thread(target=wrapper, daemon=True).start()

    def _puerto(self) -> str:
        return self.var_puerto.get().strip() or config.PUERTO_SERIAL

    def _run_test_conexion(self) -> None:
        def task() -> None:
            puerto = self._puerto()
            self.after(0, lambda: self._log(f"\n--- Test conexion ({puerto}) ---\n"))
            self.after(0, lambda: self._log("Abriendo puerto...\n"))
            with RobotClient(port=puerto, configure_speed=False) as robot:
                self.after(0, lambda: self._log("Enviando PING...\n"))
                if robot.ping():
                    self.after(0, lambda: self._log("OK — El robot respondio PONG\n"))
                else:
                    msg = (
                        f"Sin respuesta en {config.TIMEOUT_PING:.0f}s. "
                        "El puerto abrio pero el robot no respondio PONG. "
                        "Revisa firmware, alimentacion y que sea el puerto correcto."
                    )
                    self.after(0, lambda m=msg: self._log(f"FALLO — {m}\n"))
                    raise RobotSerialError(msg)

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

    def _run_stop(self) -> None:
        def task() -> None:
            puerto = self._puerto()
            self.after(0, lambda: self._log(f"\n--- Paro de emergencia ({puerto}) ---\n"))
            with RobotClient(port=puerto, configure_speed=False) as robot:
                robot.stop()
            self.after(0, lambda: self._log("STOP enviado.\n"))

        self._run_in_thread(task, allow_while_busy=True)

    def _on_close(self) -> None:
        if self._robot is not None:
            self._robot.close()
        self.destroy()


def main() -> None:
    app = PanelControl()
    app.mainloop()


if __name__ == "__main__":
    main()
