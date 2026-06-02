# Control USB — PC (Python) + Robot (C++)

Esta carpeta conecta el pick-and-place de Python con el brazo fisico por USB.

## Como funciona

```
PC (Python)                         Robot (C++)
-------------                       -------------
ejecutar_pickplace.py  --USB-->     brazo_robotico_serial.ino
  - calcula IK                         - recibe pasos
  - planifica trayectoria              - mueve motores
  - envia: G 100 -50 0 ...             - responde: LISTO
```

## Paso 1 — Subir el firmware al robot

1. Abre **Arduino IDE**
2. Abre el archivo:
   `control_usb/firmware/brazo_robotico_serial/brazo_robotico_serial.ino`
3. Selecciona tu placa (ESP32 o la que usen) y el puerto COM
4. Sube el programa (**Subir** / Upload)
5. Cierra el **Monitor Serial** del Arduino IDE (solo un programa puede usar el puerto a la vez)

> El firmware original `prograActual.cpp` queda reemplazado por este cuando quieras control por posicion.

## Paso 2 — Instalar dependencias en la PC

Desde la raiz del proyecto:

```powershell
pip install -r requirements.txt
pip install -r control_usb/requirements.txt
```

## Paso 3 — Configurar el puerto COM

Edita `control_usb/config.py`:

```python
PUERTO_SERIAL = "COM3"   # cambia por tu puerto
STEPS_PER_MM = 100.0     # calibrar
STEPS_PER_DEGREE = 80.0  # calibrar
```

Para ver tu puerto: **Administrador de dispositivos** → Puertos (COM y LPT).

## Paso 4 — Probar conexion

```powershell
cd control_usb
python test_conexion.py --port COM3
```

Debe mostrar: `OK - El robot respondio PONG`

## Paso 5 — Probar un motor

```powershell
python test_motor.py --port COM3 --motor 1 --pasos 200
```

- `--motor 0` = prismática (eje lineal)
- `--motor 1` a `6` = rotacionales

## Paso 6 — Pick and place

**Simulacion (no mueve el robot, solo muestra pasos):**

```powershell
python ejecutar_pickplace.py --dry-run --pick-x 650 --pick-y -300 --pick-z 120 --place-x 450 --place-y -500 --place-z 120
```

**Ejecucion real:**

```powershell
python ejecutar_pickplace.py --port COM3 --pick-x 650 --pick-y -300 --pick-z 120 --place-x 450 --place-y -500 --place-z 120
```

## Protocolo serial (115200 baud)

| Comando | Descripcion |
|---------|-------------|
| `PING` | Prueba de vida → responde `PONG` |
| `G s0 s1 s2 s3 s4 s5 s6` | Mueve los 7 motores en paralelo (pasos +/-) |
| `M n pasos` | Mueve un motor (n = 0..6) |
| `S` o `STOP` | Paro de emergencia |
| (fin de movimiento) | Responde `LISTO` |

Ejemplo:
```
G 100 -50 0 200 0 0 0
```

## Velocidad de giro (pulsos STEP)

La velocidad del giro **no** se controla con `PAUSA_ENTRE_PUNTOS`. Esa pausa solo espera entre waypoints del pick-and-place.

La velocidad real es el **tiempo entre pulsos** en el firmware:

1. **`VELOCIDAD_FACTOR` en `config.py`** (recomendado): `4` = un cuarto de rapidez. Al conectar, Python envia `F 4` a la placa. Sube el numero para ir mas lento (`6`, `8`...).
2. **`velocidades[]` en `brazo_robotico_serial.ino`**: microsegundos base por motor (mayor = mas lento).
3. **Monitor serial (prueba manual):** `F 6` → responde `OK FACTOR`.

Tras cambiar el `.ino`, **subir de nuevo** el firmware con Arduino IDE (al menos una vez para tener el comando `F`).

## Calibracion

Los valores `STEPS_PER_MM` y `STEPS_PER_DEGREE` en `config.py` deben medirse en el robot real:

1. Mueve un motor con `test_motor.py`
2. Mide cuanto se movio (mm o grados)
3. Ajusta los valores hasta que la posicion calculada coincida con la real

## Archivos

| Archivo | Funcion |
|---------|---------|
| `firmware/brazo_robotico_serial/brazo_robotico_serial.ino` | Programa del robot |
| `config.py` | Puerto COM y calibracion |
| `robot_client.py` | Cliente serial Python |
| `ejecutar_pickplace.py` | Pick-and-place completo |
| `test_conexion.py` | Prueba PING/PONG |
| `test_motor.py` | Prueba de un motor |

## Notas de seguridad

- Ten listo el paro: enviar `STOP` o desconectar alimentacion
- Empieza con pocos pasos en `test_motor.py`
- Usa `--dry-run` antes de la primera ejecucion real
- `--segments 3` reduce puntos y hace la secuencia mas rapida que la simulacion original
