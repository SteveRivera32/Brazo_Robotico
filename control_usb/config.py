"""
Configuracion del enlace USB PC <-> robot.
"""

# Puerto serial (Windows: COM3, COM4... | Linux: /dev/ttyUSB0)
PUERTO_SERIAL = "COM4"
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

# Pose de partida para simular movimientos relativos (mm + rad en theta*)
POSE_INICIAL_SIM = {
    "dp": 200.0,
    "theta1": 0.0,
    "theta2": 0.0,
    "theta3": 0.0,
    "theta4": 0.0,
    "theta5": 0.0,
    "theta6": 0.0,
}

# Rangos de las barras deslizantes en el simulador de posicion
SIM_SLIDER_RANGOS = {
    "dp_mm": (-100.0, 100.0),
    "theta_deg": (-180.0, 180.0),
}

# Velocidad de giro de motores (solo intervalo entre pulsos STEP en la placa).
# 1 = rapido (base del firmware), 2 = mitad de rapidez, 4 = un cuarto, etc.
# Requiere firmware con comando F (brazo_robotico_serial.ino actualizado).
VELOCIDAD_FACTOR = 4

# Pausa entre waypoints del pick-and-place (NO cambia la velocidad del giro)
PAUSA_ENTRE_PUNTOS = 0.3

# Valores predeterminados para prueba rapida de pick-and-place (mm)
PICK_PLACE_DEFAULTS = {
    "pick_x": 650.0,
    "pick_y": -300.0,
    "pick_z": 120.0,
    "place_x": 450.0,
    "place_y": -500.0,
    "place_z": 120.0,
    "approach_offset": 80.0,
    "segments": 3,
}
