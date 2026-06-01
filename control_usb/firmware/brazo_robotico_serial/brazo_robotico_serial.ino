/*
 * Brazo robotico 7 GDL - control por pasos via USB Serial
 *
 * Protocolo (115200 baud, lineas terminadas en \\n):
 *   PING              -> responde PONG
 *   G s0 s1 s2 s3 s4 s5 s6   -> mueve los 7 motores en paralelo (pasos relativos, +/-)
 *   M n pasos         -> mueve un solo motor (n = 0..6)
 *   S o STOP          -> paro de emergencia
 *
 * Al terminar un movimiento responde: LISTO
 *
 * Subir este archivo con Arduino IDE (abrir la carpeta brazo_robotico_serial).
 */

const int NUM_MOTORES = 7;

int dirPins[NUM_MOTORES]  = {15, 4, 17, 18, 22, 26, 33};
int stepPins[NUM_MOTORES] = {2, 16, 5, 19, 23, 27, 25};

// Tiempos entre pulsos en microsegundos (menor = mas rapido)
int velocidades[NUM_MOTORES] = {400, 600, 500, 600, 600, 700, 1200};

long pasosPendientes[NUM_MOTORES] = {0, 0, 0, 0, 0, 0, 0};
bool direccionMotor[NUM_MOTORES] = {true, true, true, true, true, true, true};
unsigned long previoMicros[NUM_MOTORES] = {0, 0, 0, 0, 0, 0, 0};
bool estadoStep[NUM_MOTORES] = {false, false, false, false, false, false, false};

bool esperandoListo = false;
String bufferSerial = "";

void aplicarDireccion(int i, bool adelante) {
  direccionMotor[i] = adelante;
  // Motor 5 (indice 4): logica de direccion invertida como en prograActual.cpp
  bool dirReal = (i == 4) ? !direccionMotor[i] : direccionMotor[i];
  digitalWrite(dirPins[i], dirReal);
}

void paroEmergencia() {
  for (int i = 0; i < NUM_MOTORES; i++) {
    pasosPendientes[i] = 0;
    estadoStep[i] = false;
    digitalWrite(stepPins[i], LOW);
  }
  esperandoListo = false;
  Serial.println("PARO");
}

bool hayMovimientoPendiente() {
  for (int i = 0; i < NUM_MOTORES; i++) {
    if (pasosPendientes[i] != 0) {
      return true;
    }
  }
  return false;
}

void configurarMotores() {
  for (int i = 0; i < NUM_MOTORES; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    aplicarDireccion(i, true);
    digitalWrite(stepPins[i], LOW);
  }
}

bool iniciarMovimientoMotor(int indice, long pasos) {
  if (indice < 0 || indice >= NUM_MOTORES) {
    Serial.println("ERR MOTOR");
    return false;
  }
  if (pasos == 0) {
    return true;
  }

  aplicarDireccion(indice, pasos > 0);
  pasosPendientes[indice] = pasos;
  esperandoListo = true;
  return true;
}

bool iniciarMovimientoGrupo(long pasos[NUM_MOTORES]) {
  bool alguno = false;
  for (int i = 0; i < NUM_MOTORES; i++) {
    if (pasos[i] != 0) {
      aplicarDireccion(i, pasos[i] > 0);
      pasosPendientes[i] = pasos[i];
      alguno = true;
    } else {
      pasosPendientes[i] = 0;
    }
  }
  if (alguno) {
    esperandoListo = true;
  }
  return true;
}

void generarPulsos() {
  unsigned long ahora = micros();

  for (int i = 0; i < NUM_MOTORES; i++) {
    if (pasosPendientes[i] == 0) {
      continue;
    }

    if (ahora - previoMicros[i] >= (unsigned long)velocidades[i]) {
      previoMicros[i] = ahora;
      estadoStep[i] = !estadoStep[i];
      digitalWrite(stepPins[i], estadoStep[i]);

      if (estadoStep[i]) {
        if (pasosPendientes[i] > 0) {
          pasosPendientes[i]--;
        } else {
          pasosPendientes[i]++;
        }
      }
    }
  }

  if (esperandoListo && !hayMovimientoPendiente()) {
    esperandoListo = false;
    Serial.println("LISTO");
  }
}

void procesarLinea(String linea) {
  linea.trim();
  if (linea.length() == 0) {
    return;
  }

  if (linea == "PING") {
    Serial.println("PONG");
    return;
  }

  if (linea == "S" || linea == "STOP") {
    paroEmergencia();
    return;
  }

  if (linea.startsWith("M ")) {
    int indice = 0;
    long pasos = 0;
    if (sscanf(linea.c_str(), "M %d %ld", &indice, &pasos) == 2) {
      if (iniciarMovimientoMotor(indice, pasos)) {
        if (!hayMovimientoPendiente()) {
          Serial.println("LISTO");
        }
      }
    } else {
      Serial.println("ERR FORMATO");
    }
    return;
  }

  if (linea.startsWith("G ")) {
    long pasos[NUM_MOTORES] = {0, 0, 0, 0, 0, 0, 0};
    int leidos = sscanf(
      linea.c_str(),
      "G %ld %ld %ld %ld %ld %ld %ld",
      &pasos[0], &pasos[1], &pasos[2], &pasos[3], &pasos[4], &pasos[5], &pasos[6]
    );

    if (leidos == NUM_MOTORES) {
      iniciarMovimientoGrupo(pasos);
      if (!hayMovimientoPendiente()) {
        Serial.println("LISTO");
      }
    } else {
      Serial.println("ERR FORMATO");
    }
    return;
  }

  Serial.println("ERR COMANDO");
}

void setup() {
  Serial.begin(115200);
  configurarMotores();
  Serial.println("SISTEMA SERIAL OK");
  Serial.println("Comandos: PING | G s0..s6 | M n pasos | S");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferSerial.length() > 0) {
        procesarLinea(bufferSerial);
        bufferSerial = "";
      }
    } else {
      bufferSerial += c;
      if (bufferSerial.length() > 120) {
        bufferSerial = "";
        Serial.println("ERR BUFFER");
      }
    }
  }

  generarPulsos();
}
