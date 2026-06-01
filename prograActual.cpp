int dirPins[7]  = {15, 4, 17, 18, 22, 26, 33}; // Motor 6: DIR 26 | Motor 7: DIR 33
int stepPins[7] = {2, 16, 5, 19, 23, 27, 25};  // Motor 6: PUL 27 | Motor 7: PUL 25

int velocidades[7] = {400, 600, 500, 600, 600, 700, 1200}; // Tiempos en microsegundos

// Variables de estado por motor
bool motorEncendido[7] = {false, false, false, false, false, false, false};
bool direccionMotor[7] = {true, true, true, true, true, true, true};
unsigned long previoMicros[7] = {0, 0, 0, 0, 0, 0, 0};
bool estadoStep[7] = {false, false, false, false, false, false, false};

void setup() {
  Serial.begin(115200);
  
  for (int i = 0; i < 7; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    digitalWrite(dirPins[i], direccionMotor[i]);
    digitalWrite(stepPins[i], LOW);
  }
  
  Serial.println("SISTEMA INDEPENDIENTE OK");
  Serial.println("Controles: 1-7 para Iniciar/Cambiar Giro | 's' para Paro Global");
}

void loop() {
  // Manejo de comandos seriales
  if (Serial.available() > 0) {
    char comando = Serial.read();

    if (comando >= '1' && comando <= '7') {
      int i = comando - '1'; // Convertir char a indice 0-6

      if (!motorEncendido[i]) { 
        motorEncendido[i] = true;
        Serial.print("Motor "); Serial.print(i + 1); Serial.println(" INICIADO");
      } else {
        direccionMotor[i] = !direccionMotor[i];
        // Inversion logica especial para el motor 5 si es necesario
        bool dirReal = (i == 4) ? !direccionMotor[i] : direccionMotor[i];
        digitalWrite(dirPins[i], dirReal);
        Serial.print("Motor "); Serial.print(i + 1); Serial.println(" CAMBIO GIRO");
      }
    } 
    else if (comando == 's' || comando == 'S') {
      for (int i = 0; i < 7; i++) {
        motorEncendido[i] = false;
        digitalWrite(stepPins[i], LOW);
      }
      Serial.println("PARO GENERAL EJECUTADO");
    }
  }

  // Generacion asincrona de pulsos
  unsigned long ahora = micros();

  for (int i = 0; i < 7; i++) {
    if (motorEncendido[i]) {
      if (ahora - previoMicros[i] >= velocidades[i]) {
        previoMicros[i] = ahora;
        estadoStep[i] = !estadoStep[i];
        digitalWrite(stepPins[i], estadoStep[i]);
      }
    }
  }
}