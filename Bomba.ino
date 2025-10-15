// --- Configuración de pines ---
const int pinBomba = 53;   // Pin conectado al Gate del MOSFET

void setup() {
  Serial.begin(9600);          // Comunicación serial para control manual
  pinMode(pinBomba, OUTPUT);   // El pin controla el MOSFET
  digitalWrite(pinBomba, LOW); // Asegura que la bomba empiece apagada

  Serial.println("Control de bomba listo.");
  Serial.println("Comandos disponibles:");
  Serial.println("  ON  - Enciende la bomba");
  Serial.println("  OFF - Apaga la bomba");
  Serial.println("  AUTO - Activa modo automático (enciende 3 seg, apaga 5 seg)");
}

void loop() {
  // --- Control manual desde el monitor serie ---
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim(); // Elimina espacios o saltos de línea

    if (comando.equalsIgnoreCase("ON")) {
      digitalWrite(pinBomba, HIGH);
      Serial.println("💧 Bomba ENCENDIDA");
    } 
    else if (comando.equalsIgnoreCase("OFF")) {
      digitalWrite(pinBomba, LOW);
      Serial.println("💤 Bomba APAGADA");
    } 
    else if (comando.equalsIgnoreCase("AUTO")) {
      Serial.println("⏱ Modo automático (3s encendida, 5s apagada)");
      for (int i = 0; i < 3; i++) { // Repite 3 ciclos
        digitalWrite(pinBomba, HIGH);
        Serial.println("💧 Bomba ENCENDIDA");
        delay(3000); // 3 segundos encendida
        digitalWrite(pinBomba, LOW);
        Serial.println("💤 Bomba APAGADA");
        delay(5000); // 5 segundos apagada
      }
      Serial.println("Modo automático finalizado.");
    } 
    else {
      Serial.println("Comando no reconocido. Usa: ON / OFF / AUTO");
    }
  }
}
