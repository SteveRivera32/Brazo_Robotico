# Control USB — PC (Python) + Robot (firmware serial)

Esta carpeta conecta el pick-and-place con el brazo fisico por USB.

## Como funciona

```
PC (Python)                         Robot (firmware)
-------------                       ----------------
panel_control.py       --USB-->     brazo_robotico_serial.ino
  - simular juntas (FK)                - recibe pasos
  - pick-and-place (IK+FK)             - mueve motores
  - prueba motor / PING / STOP         - responde LISTO
```

## Paso 1 — Subir el firmware

1. Abre **Arduino IDE**
2. Abre `control_usb/firmware/brazo_robotico_serial/brazo_robotico_serial.ino`
3. Sube el programa y cierra el Monitor Serial

## Paso 2 — Instalar dependencias

Desde la raiz del proyecto:

```powershell
pip install -r requirements.txt
```

## Paso 3 — Configurar puerto COM

Edita `control_usb/config.py`:

```python
PUERTO_SERIAL = "COM4"
STEPS_PER_MM = 100.0
STEPS_PER_DEGREE = 80.0
```

## Paso 4 — Panel de control (recomendado)

```powershell
cd control_usb
python panel_control.py
```

El panel incluye:

| Funcion | Descripcion |
|---------|-------------|
| **Probar conexion** | PING / PONG al robot |
| **Paro de emergencia** | Envia STOP |
| **Probar motor** | Mueve un motor por pasos |
| **Pick and place** | Simulacion o ejecucion real |
| **Simular juntas** | FK CoppeliaSim con barras deslizantes |

## Pick and place por consola (opcional)

```powershell
cd control_usb
python ejecutar_pickplace.py --dry-run
python ejecutar_pickplace.py --port COM4 --pick-x 37 --pick-y 138 --pick-z 1136 --place-x -163 --place-y -62 --place-z 1136
```

Coordenadas en marco **CoppeliaSim** (mm). Valores de prueba en `robot_config.py`.

## Archivos principales

| Archivo | Funcion |
|---------|---------|
| `panel_control.py` | Interfaz grafica |
| `ejecutar_pickplace.py` | Pick-and-place por consola |
| `robot_client.py` | Cliente serial |
| `config.py` | Puerto COM y calibracion |
| `../coppelia_kinematics.py` | FK CoppeliaSim (NumPy) |
| `../robot_frame.py` | IK CoppeliaSim |
| `../pick_and_place.py` | Planificacion de trayectoria |
| `../simular_juntas.py` | Simulacion por juntas |

## Protocolo serial (115200 baud)

| Comando | Descripcion |
|---------|-------------|
| `PING` | Prueba de vida → `PONG` |
| `G s0..s6` | Mueve 7 motores en paralelo |
| `M n pasos` | Un motor (n = 0..6) |
| `F factor` | Velocidad de giro |
| `S` / `STOP` | Paro de emergencia |
| (fin movimiento) | `LISTO` |

## Notas de seguridad

- Ten listo el boton **STOP** del panel
- Empieza con pocos pasos al probar motores
- Usa simulacion antes de la primera ejecucion real
