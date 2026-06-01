"""
Configuracion del enlace USB PC <-> robot.
Edita estos valores segun tu setup real.
"""

# Puerto serial (Windows: COM3, COM4... | Linux: /dev/ttyUSB0)
PUERTO_SERIAL = "COM3"
BAUDRATE = 115200

# Tiempo de espera maximo por movimiento (segundos)
TIMEOUT_MOVIMIENTO = 120.0

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

# Pausa entre waypoints del pick-and-place (segundos)
PAUSA_ENTRE_PUNTOS = 0.3
