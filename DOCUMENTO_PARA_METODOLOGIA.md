# Brief del proyecto — Brazo robótico (para redactar metodología)

> Copia este documento completo en ChatGPT y pide: *"Redacta la sección de Metodología de un informe académico en español, basándote en este brief. Incluye modelado DH, cinemática, planificación de trayectorias y arquitectura software/hardware."*

---

## 1. Título y objetivo

**Proyecto:** Brazo robótico serial de **7 grados de libertad (7 GDL)**:
- **1 junta prismática** (`dp`): desplazamiento lineal en mm (recorrido máximo 500 mm).
- **6 juntas rotacionales** (`theta1` … `theta6`): ángulos en radianes/grados.

**Objetivo general:** Modelar matemáticamente el brazo, calcular posiciones del efector final (cinemática directa e inversa) y ejecutar tareas de **pick and place** (recoger y colocar objetos) mediante trayectorias planificadas en el espacio cartesiano, con ejecución en hardware vía control de motores paso a paso.

**Alcance del modelado:** Se controla la **posición** del efector `(x, y, z)` en milímetros; no se modela de forma completa la orientación 6D del tool (solo posición cartesiana).

---

## 2. Hardware

- **7 motores paso a paso** (drivers y pines definidos en firmware Arduino/ESP32).
- **Placa:** ESP32 o compatible (firmware en `control_usb/firmware/brazo_robotico_serial/`).
- **Comunicación:** USB serial, 115200 baud.
- **Firmware:** Recibe comandos de pasos por motor; no implementa cinemática (la cinemática corre en la PC).
- **Firmware legado:** `prograActual.cpp` — control manual por teclado serial (sin modelado); reemplazado por el firmware serial cuando se usa pick-and-place automático.

**Geometría del brazo (longitudes de eslabón en mm):**

| Parámetro | Valor (mm) | Descripción |
|-----------|------------|-------------|
| L0 | 500 | Recorrido máximo eje prismático |
| L1 | 235 | Eslabón 1 |
| L2 | 154 | Eslabón 2 |
| L3 | 145 | Eslabón 3 |
| L4 | 224 | Eslabón 4 |
| L5 | 145 | Eslabón 5 |
| L6 | 224 | Eslabón 6 |
| L7 | 52 | Offset/herramienta al efector |

---

## 3. Modelado matemático — Denavit-Hartenberg (DH)

Se usa la **convención DH estándar**. Cada eslabón tiene parámetros `(a, α, d, θ)` y una matriz de transformación homogénea 4×4. La pose del efector es el producto de matrices: **T = A₁ · A₂ · … · A₈**.

**Tabla DH del proyecto (8 filas = 7 juntas + eslabón fijo final):**

| # | a (mm) | α (rad) | d (mm) | θ (variable) |
|---|--------|---------|--------|----------------|
| 1 | 0 | π/2 | d_p (prismática) | 0 (fijo) |
| 2 | 0 | π/2 | L1 = 235 | θ₁ |
| 3 | L2 = 154 | −π/2 | 0 | θ₂ |
| 4 | L3 = 145 | π/2 | 0 | θ₃ |
| 5 | L4 = 224 | −π/2 | 0 | θ₄ |
| 6 | L5 = 145 | π/2 | 0 | θ₅ |
| 7 | L6 = 224 | −π/2 | 0 | θ₆ |
| 8 | L7 = 52 | 0 | 0 | 0 (fijo) |

**Vector de juntas:** `q = [d_p, θ₁, θ₂, θ₃, θ₄, θ₅, θ₆]`.

**Implementación:** `robot_kinematics.py` — constante `DH_TABLE`, función `_dh_matrix()`, `forward_kinematics()`.

**Ecuaciones simbólicas:** `python main.py symbolic` genera con SymPy la expresión simbólica de `[x, y, z]` del efector a partir de la misma tabla DH.

---

## 4. Cinemática

### 4.1 Cinemática directa (FK)
- **Entrada:** valores de juntas `q`.
- **Salida:** posición `(x, y, z)` y matriz de transformación 4×4 del efector.
- **Método:** multiplicación secuencial de matrices DH.

### 4.2 Cinemática inversa (IK)
- **Entrada:** posición objetivo `(x, y, z)`.
- **Salida:** vector de juntas `q` que aproxima el objetivo.
- **Método:** optimización numérica con **SciPy** (`scipy.optimize.minimize`, método L-BFGS-B), minimizando error de posición + término de regularización hacia una **pose preferida** (el brazo es redundante: 7 GDL para 3 ecuaciones de posición → infinitas soluciones).
- **Tolerancia:** ~1 mm de error de posición.
- **Límites articulares:** definidos en `DEFAULT_JOINT_LIMITS` (dp entre 0 y 500 mm; ángulos entre −π y π).

### 4.3 Conversión a motores
- Los ángulos/desplazamientos se convierten a **pasos de motor** con factores calibrables: `STEPS_PER_MM` y `STEPS_PER_DEGREE` (en `config.py` y `robot_kinematics.py`).

---

## 5. Planificación de trayectorias

**Tarea principal:** Pick and place (recoger en un punto, transportar, soltar en otro).

**Archivo:** `pick_and_place.py`

**Secuencia de waypoints (espacio cartesiano, mm):**
1. Home  
2. Aproximar al punto de recogida (offset vertical, p. ej. +80 mm en Z)  
3. Pick (posición de recogida)  
4. Elevar con pieza  
5. Tránsito a zona de colocación (altura segura)  
6. Aproximar colocación  
7. Place  
8. Elevar final  

**Interpolación:** entre waypoints consecutivos se generan puntos intermedios por **interpolación lineal** en X, Y, Z (`segments_per_move` segmentos por tramo).

**Por cada waypoint:**
1. Resolver IK → juntas.  
2. Verificar con FK que la posición alcanzada coincide (control de error).  
3. (En ejecución real) convertir delta de juntas a pasos y enviar al robot.

**No implementado (mencionar como limitación en metodología):** perfiles de velocidad/aceleración en el tiempo, splines cúbicos, planificación de orientación del efector, evitación de obstáculos.

---

## 6. Arquitectura del software

```
┌─────────────────────────────────────────────────────────┐
│  PC — Python 3                                           │
│  main.py              → CLI: FK, IK, simulación pickplace │
│  robot_kinematics.py  → DH, FK, IK, pasos de motor        │
│  pick_and_place.py    → planificación de trayectoria    │
│  control_usb/                                            │
│    ejecutar_pickplace.py → plan + envío al robot          │
│    robot_client.py       → protocolo serial               │
│    config.py             → puerto COM, calibración        │
└──────────────────────────┬──────────────────────────────┘
                           │ USB serial 115200
                           ▼
┌─────────────────────────────────────────────────────────┐
│  Robot — C++ (Arduino/ESP32)                             │
│  brazo_robotico_serial.ino → ejecuta pasos, responde     │
│    PING/PONG, G s0..s6, STOP, LISTO                      │
└─────────────────────────────────────────────────────────┘
```

**Flujo pick-and-place real:**
1. Usuario define coordenadas pick y place.  
2. Python planifica waypoints e interpola.  
3. IK en cada punto → juntas.  
4. Delta respecto a pose anterior → pasos por motor.  
5. Comando serial `G paso0 paso1 ... paso6` al firmware.  
6. Firmware mueve motores en paralelo hasta responder `LISTO`.

---

## 7. Herramientas y tecnologías

| Herramienta | Uso en el proyecto |
|-------------|-------------------|
| **Python 3** | Lenguaje principal: cinemática, trayectorias, cliente serial |
| **NumPy** | Matrices DH, álgebra lineal |
| **SciPy** | Cinemática inversa (optimización) |
| **SymPy** | Derivación simbólica de ecuaciones FK (`main.py symbolic`) |
| **pyserial** | Comunicación USB PC ↔ robot |
| **argparse** | Interfaz de línea de comandos (`main.py`) |
| **Arduino IDE** | Carga de firmware al microcontrolador |
| **C++ (Arduino)** | Control de motores paso a paso en tiempo real |

**Dependencias:** `requirements.txt` (numpy, scipy, sympy, pyserial).

---

## 8. Archivos principales del repositorio

| Archivo | Función |
|---------|---------|
| `robot_kinematics.py` | Tabla DH, FK, IK, conversión a pasos |
| `pick_and_place.py` | Trayectoria pick-and-place |
| `main.py` | Entrada CLI (fk, ik, pickplace, symbolic) |
| `control_usb/ejecutar_pickplace.py` | Ejecución en robot real o `--dry-run` |
| `control_usb/robot_client.py` | Cliente del protocolo serial |
| `control_usb/config.py` | Puerto COM, calibración |
| `control_usb/firmware/.../brazo_robotico_serial.ino` | Firmware del robot |
| `prograActual.cpp` | Firmware antiguo (control manual, sin cinemática) |

---

## 9. Comandos de uso (referencia)

```bash
pip install -r requirements.txt

# Cinemática directa
python main.py fk --dp 200 --theta1 0 --theta2 0 ...

# Cinemática inversa
python main.py ik --x 650 --y -300 --z 120

# Simular trayectoria
python main.py pickplace --pick-x 650 --pick-y -300 --pick-z 120 --place-x 450 --place-y -500 --place-z 120

# Ecuaciones simbólicas DH
python main.py symbolic

# Ejecución real (desde control_usb/)
python ejecutar_pickplace.py --port COM3 --pick-x 650 ...
python ejecutar_pickplace.py --dry-run ...   # sin mover motores
```

---

## 10. Limitaciones y supuestos (para la metodología)

- Modelo **cinemático** (no dinámico): no se modelan masas, inercias ni pares.
- IK **numérica**, no analítica cerrada.
- Redundancia 7 GDL resuelta eligiendo la solución más cercana a una pose preferida.
- Trayectorias por **waypoints + interpolación lineal**, sin perfil temporal óptimo.
- Calibración mecánica necesaria (`STEPS_PER_MM`, `STEPS_PER_DEGREE`).
- El firmware **no** calcula cinemática; toda la inteligencia de movimiento está en Python.

---

## 11. Instrucción sugerida para ChatGPT

Redacta la **Metodología** de un informe o tesis en español formal, con subsecciones como mínimo:

1. Enfoque general del proyecto  
2. Descripción del sistema (hardware y software)  
3. Modelado del robot con parámetros Denavit-Hartenberg (incluir tabla DH)  
4. Cinemática directa e inversa  
5. Planificación de trayectorias (pick and place)  
6. Implementación computacional y herramientas  
7. Procedimiento de validación (simulación FK tras IK, dry-run, calibración)  
8. Limitaciones del enfoque  

Usa tono académico, tercera persona o voz pasiva, y diagramas de flujo en texto si ayudan. No inventes hardware o software que no esté en este documento.
