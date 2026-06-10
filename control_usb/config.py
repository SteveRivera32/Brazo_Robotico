"""
Configuracion del enlace USB PC <-> robot.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_config import PICK_PLACE_DEFAULTS  # noqa: E402

# Puerto serial (Windows: COM3, COM4... | Linux: /dev/ttyUSB0)
PUERTO_SERIAL = "COM4"
BAUDRATE = 115200

# Tiempo de espera maximo por movimiento (segundos)
TIMEOUT_MOVIMIENTO = 120.0

# Timeouts de conexion (segundos)
TIMEOUT_APERTURA = 3.0
TIMEOUT_PING = 3.0
ESPERA_PUERTO = 0.3

# Calibracion de motores (AJUSTAR en banco de pruebas)
STEPS_PER_MM = 100.0
STEPS_PER_DEGREE = 80.0

# Pose 'home' fisica del robot (juntas en cero mecanico)
HOME_JOINTS = {
    "dp": 0.0,
    "theta1": 0.0,
    "theta2": 0.0,
    "theta3": 0.0,
    "theta4": 0.0,
    "theta5": 0.0,
    "theta6": 0.0,
}

# Velocidad de giro de motores (solo intervalo entre pulsos STEP en la placa).
# 1 = rapido (base del firmware), 2 = mitad de rapidez, 4 = un cuarto, etc.
VELOCIDAD_FACTOR = 4

# Pausa entre waypoints del pick-and-place (NO cambia la velocidad del giro)
PAUSA_ENTRE_PUNTOS = 0.3

# Limites de las barras en la simulacion de movimiento por juntas
SIMULACION_JUNTAS_LIMITS = {
    "dp_mm": (0, 420),
    "theta_delta_deg": (-180, 180),
}

# Paso de la barra al arrastrar (1 = encaje en cada entero: 0, 1, 2, 3...)
SIM_SNAP_BARRA = {
    "dp_mm": 1,
    "theta_delta_deg": 1,
}
