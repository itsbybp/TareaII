/*
  Capitulo 6 de Arduino desde cero en Español
  Control de servo con tres zonas: -180°, 0°, 180°
  Autor: bitwiseAr  | Mod: ChatGPT
*/

#include <Servo.h>

Servo servo1;

const uint8_t PINSERVO = 5;    // <-- Asegúrate de cablear al pin 5 (o cambia este valor)
const int PULSOMIN = 544;      // µs (ajusta a tu servo)
const int PULSOMAX = 2400;     // µs (ajusta a tu servo)

const unsigned long T_ESPERA = 1000; // ms entre zonas

// Convierte ángulo lógico [-180..180] a [0..180] real del servo
void moverServoDeg180(int angulo_logico) {
  angulo_logico = constrain(angulo_logico, -180, 180);
  // Si tu servo golpea tope, usa 10..170 en lugar de 0..180
  int angulo_servo = map(angulo_logico, -180, 180, 0, 180);
  servo1.write(angulo_servo);
}

// Zonas
void IrZona1() { moverServoDeg180(0); }
void IrZona2() { moverServoDeg180(180); }
void IrZona3() { moverServoDeg180(-180); }

void setup() {
  Serial.begin(9600);
  servo1.attach(PINSERVO, PULSOMIN, PULSOMAX); // ¡No hacer detach aquí!
  IrZona1();
}

void loop() {
  IrZona1(); delay(T_ESPERA);
  IrZona2(); delay(T_ESPERA);
  IrZona3(); delay(T_ESPERA);
}
