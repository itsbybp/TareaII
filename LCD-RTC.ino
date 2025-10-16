/*
  Sistema de Riego Inteligente - Versión POO Simplificada
  
  Funcionalidades:
  - 3 zonas de riego independientes
  - Control por humedad mínima
  - Programación por horario y días
  - Bloqueo por probabilidad de lluvia ≥ 70%
  - Servo para dirigir el riego
  - Configuración interactiva por Serial
  
  Hardware:
  - Arduino Mega
  - LCD 16x2
  - DS1302 (RTC)
  - 3x DHT11 (sensores humedad)
  - 3x Relés
  - 1x Servo
  - 1x Bomba de agua
  
  COMANDOS SERIAL:
  - Presiona 'c' o 'C' para entrar al menú de configuración
*/

// ===== LIBRERÍAS =====
#include <LiquidCrystal.h>
#include <virtuabotixRTC.h>
#include <DHT.h>
#include <Servo.h>

constexpr float SCHEDULE_SKIP_HUM = 70.0f; // si humedad > 70% NO riega por horario

// ===== CONFIGURACIÓN DE HARDWARE =====
const uint8_t PIN_BOMBA = 53;
const uint8_t PIN_RELAY[3] = {40, 42, 44};
const uint8_t PIN_DHT[3] = {2, 3, 4};
const uint8_t PIN_SERVO = 9;

// Configuración LCD (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(32, 30, 28, 26, 24, 22);

// Configuración RTC (CLK, DAT, RST)
virtuabotixRTC myRTC(6, 7, 8);

// ===== CLASE: SENSOR DE HUMEDAD =====
class HumiditySensor {
private:
  DHT dht;
  float lastReading;
  
public:
  HumiditySensor(uint8_t pin) : dht(pin, DHT11), lastReading(NAN) {}
  
  void begin() {
    dht.begin();
  }
  
  float read() {
    float h = dht.readHumidity();
    if (!isnan(h) && h >= 0 && h <= 100) {
      lastReading = h;      // actualiza solo si es válida
    }
    return lastReading;     // devuelve la última válida
  }

  
  float getLast() const {
    return lastReading;
  }
  
  bool isValid() const {
    return !isnan(lastReading);
  }
};

// ===== CLASE: ZONA DE RIEGO =====
class IrrigationZone {
private:
  uint16_t lastScheduleDay = 0;   // último día que regó por HORARIO (no confundir con lastWateringDay)
  bool startedBySchedule = false; // true si este riego inició en ventana

  uint8_t id;
  char label;
  uint8_t relayPin;
  int servoAngle;
  HumiditySensor* sensor;
  
  // Configuración de riego
  uint8_t minHumidity;
  uint8_t maxHumidity;
  uint8_t intervalDays;
  uint16_t startTimeMin;
  uint16_t endTimeMin;
  uint16_t maxDurationMin;
  
  // Estado de riego
  bool isWatering;
  uint32_t wateringStartMillis;
  uint16_t lastWateringDay;
  
public:
  IrrigationZone(uint8_t zoneId, char zoneLbl, uint8_t relay, int servo, uint8_t dhtPin)
    : id(zoneId), label(zoneLbl), relayPin(relay), servoAngle(servo),
      minHumidity(40), maxHumidity(80), intervalDays(5),
      startTimeMin(7 * 60), endTimeMin(7 * 60 + 30), maxDurationMin(30),
      isWatering(false), wateringStartMillis(0), lastWateringDay(0) {
    
    sensor = new HumiditySensor(dhtPin);
    pinMode(relayPin, OUTPUT);
    stopRelay();
  }
  
  ~IrrigationZone() {
    delete sensor;
  }
  
  void begin() {
    sensor->begin();
  }
  
  // ===== CONFIGURACIÓN =====
  void setHumidityLimits(uint8_t min, uint8_t max) {
    minHumidity = constrain(min, 0, 100);
    maxHumidity = constrain(max, 0, 100);
  }
  
  void setSchedule(uint8_t days, uint16_t startMin, uint16_t endMin) {
    intervalDays = days;
    startTimeMin = startMin;
    endTimeMin = endMin;
  }
  
  void setMaxDuration(uint16_t minutes) {
    maxDurationMin = minutes;
  }
  
  // ===== GETTERS =====
  uint8_t getId() const { return id; }
  char getLabel() const { return label; }
  int getServoAngle() const { return servoAngle; }
  bool getWateringState() const { return isWatering; }
  float getHumidity() const { return sensor->getLast(); }
  uint8_t getMinHumidity() const { return minHumidity; }
  uint8_t getMaxHumidity() const { return maxHumidity; }
  uint8_t getIntervalDays() const { return intervalDays; }
  uint16_t getStartTimeMin() const { return startTimeMin; }
  uint16_t getEndTimeMin() const { return endTimeMin; }
  uint16_t getMaxDuration() const { return maxDurationMin; }
  
  // ===== CONTROL DE RELÉ =====
  void startRelay() {
    digitalWrite(relayPin, LOW);
  }
  
  void stopRelay() {
    digitalWrite(relayPin, HIGH);
  }
  
  void updateSensor() {
    sensor->read();
  }
  
  // ===== VERIFICAR SI DEBE REGAR =====
  bool shouldWater(uint16_t currentDayOfYear, uint16_t currentTimeMin) {
    if (isWatering) return false;

    // 1) ¿Estamos en ventana? (soporta medianoche)
    bool inWindow =
      (startTimeMin <= endTimeMin)
        ? (currentTimeMin >= startTimeMin && currentTimeMin < endTimeMin)
        : (currentTimeMin >= startTimeMin ||  currentTimeMin < endTimeMin);

    // 2) HORARIO: arranca según días SIN mirar humedad
    if (intervalDays > 0) {
      int d = (int)currentDayOfYear - (int)lastScheduleDay; // <- ¡OJO! usa lastScheduleDay
      if (d < 0) d += 365;
      if (d >= intervalDays && inWindow) {
        return true;  // en ventana: arranca siempre
      }
    }

    // 3) FUERA de horario: riego por mínima (requiere lectura válida)
    if (!inWindow && sensor->isValid() && sensor->getLast() < minHumidity) {
      return true;
    }

    return false;
  }

  
  void startWatering(uint16_t currentDayOfYear) {
    if (!isWatering) {
      isWatering = true;
      wateringStartMillis = millis();
      lastWateringDay = currentDayOfYear;
      uint16_t t = (uint16_t)myRTC.hours * 60u + (uint16_t)myRTC.minutes;
      bool inWindow = (startTimeMin <= endTimeMin)
                        ? (t >= startTimeMin && t < endTimeMin)
                        : (t >= startTimeMin ||  t < endTimeMin);

      startedBySchedule = inWindow;           // true si inició en ventana
      if (startedBySchedule) {
        lastScheduleDay = currentDayOfYear;   // registra SOLO riegos por horario
      }

      // (mantén tu lastWateringDay = currentDayOfYear si ya lo haces)

      startRelay();
      
      Serial.print(F(">> Zona "));
      Serial.print(label);
      Serial.print(F(" ("));
      Serial.print(id + 1);
      Serial.println(F(") INICIANDO riego"));
    }
  }
  
  void stopWatering(const char* reason) {
    startedBySchedule = false;

    if (isWatering) {
      isWatering = false;
      stopRelay();
      
      Serial.print(F(">> Zona "));
      Serial.print(label);
      Serial.print(F(" ("));
      Serial.print(id + 1);
      Serial.print(F(") DETENIDA - "));
      Serial.println(reason);
    }
  }
  
  void checkStopConditions(uint8_t rainProb) {
    if (!isWatering) return;
    if (rainProb >= 70) { stopWatering("probabilidad de lluvia alta"); return; }
    // Hora actual
    uint16_t t = (uint16_t)myRTC.hours * 60u + (uint16_t)myRTC.minutes;
    bool inWindow =
      (startTimeMin <= endTimeMin)
        ? (t >= startTimeMin && t < endTimeMin)
        : (t >= startTimeMin ||  t < endTimeMin);

    bool outOfWindow =
      (startTimeMin <= endTimeMin)
        ? (t < startTimeMin || t >= endTimeMin)
        : (t >= endTimeMin && t < startTimeMin);

    // --- Si empezó por HORARIO, manda la ventana ---
    if (startedBySchedule) {
      if (outOfWindow) {                 // se acabó el horario
        stopWatering("fin de ventana horaria");
        startedBySchedule = false;
        return;
      }
      // En ventana: IGNORA duración y humedad
      return;
    }

    // --- Si NO empezó por horario (mínima/manual): aplica reglas normales ---
    sensor->read();

    // Duración máxima (solo fuera de horario)
    uint32_t elapsedMin = (millis() - wateringStartMillis) / 60000UL;
    if (elapsedMin >= maxDurationMin) {
      stopWatering("tiempo maximo alcanzado");
      return;
    }

    if (sensor->isValid()) {
      float h = sensor->getLast();
      if (h >= maxHumidity) {
        stopWatering("humedad maxima alcanzada");
        return;
      }
      if (h >= minHumidity) {
        stopWatering("humedad minima alcanzada");
        return;
      }
    }
  }


  
  void printConfig() const {
    Serial.print(F("  Zona "));
    Serial.print(id + 1);
    Serial.print(F(" ("));
    Serial.print(label);
    Serial.println(F(")"));
    
    Serial.print(F("    Humedad: min="));
    Serial.print(minHumidity);
    Serial.print(F("%, max="));
    Serial.print(maxHumidity);
    Serial.println(F("%"));
    
    Serial.print(F("    Regar cada: "));
    Serial.print(intervalDays);
    Serial.println(F(" dias"));
    
    Serial.print(F("    Horario: "));
    Serial.print(startTimeMin / 60);
    Serial.print(F(":"));
    if (startTimeMin % 60 < 10) Serial.print(F("0"));
    Serial.print(startTimeMin % 60);
    Serial.print(F(" - "));
    Serial.print(endTimeMin / 60);
    Serial.print(F(":"));
    if (endTimeMin % 60 < 10) Serial.print(F("0"));
    Serial.println(endTimeMin % 60);
    
    Serial.print(F("    Duracion max: "));
    Serial.print(maxDurationMin);
    Serial.println(F(" min"));
  }
};

// ===== CLASE: CONTROLADOR DE SERVO =====
class ServoController {
private:
  Servo servo;
  uint8_t pin;
  int currentAngle;
  
public:
  ServoController(uint8_t servoPin) : pin(servoPin), currentAngle(90) {}
  
  void begin() {
    servo.attach(pin);
    servo.write(currentAngle);
    delay(500);
  }
  
  void moveTo(int targetAngle) {
    targetAngle = constrain(targetAngle, 0, 180);
    int step = (targetAngle > currentAngle) ? 1 : -1;
    
    for (int angle = currentAngle; angle != targetAngle; angle += step) {
      servo.write(angle);
      delay(10);
    }
    
    servo.write(targetAngle);
    currentAngle = targetAngle;
    delay(500);
  }
};

// ===== CLASE: BOMBA DE AGUA =====
class WaterPump {
private:
  uint8_t pin;
  bool state;
  
public:
  WaterPump(uint8_t pumpPin) : pin(pumpPin), state(false) {
    pinMode(pin, OUTPUT);
    off();
  }
  
  void on() {
    if (!state) {
      digitalWrite(pin, HIGH);
      state = true;
      Serial.println(F(">> BOMBA ENCENDIDA"));
    }
  }
  
  void off() {
    if (state) {
      digitalWrite(pin, LOW);
      state = false;
      Serial.println(F(">> BOMBA APAGADA"));
    }
  }
  
  bool isOn() const {
    return state;
  }
};

// ===== CLASE: SISTEMA DE RIEGO =====
class IrrigationSystem {
private:
  IrrigationZone* zones[3];
  ServoController* servo;
  WaterPump* pump;
  uint8_t rainProbability;
  uint32_t lastSensorUpdate;
  uint32_t lastLcdUpdate;
  
public:
  IrrigationSystem() : rainProbability(0), lastSensorUpdate(0), lastLcdUpdate(0) {
    zones[0] = new IrrigationZone(0, 'P', PIN_RELAY[0], 30, PIN_DHT[0]);
    zones[1] = new IrrigationZone(1, 'J', PIN_RELAY[1], 90, PIN_DHT[1]);
    zones[2] = new IrrigationZone(2, 'H', PIN_RELAY[2], 150, PIN_DHT[2]);
    
    servo = new ServoController(PIN_SERVO);
    pump = new WaterPump(PIN_BOMBA);
  }
  
  ~IrrigationSystem() {
    for (int i = 0; i < 3; i++) {
      delete zones[i];
    }
    delete servo;
    delete pump;
  }
  
  void begin() {
    Serial.begin(115200);
    lcd.begin(16, 2);
    
    for (int i = 0; i < 3; i++) {
      zones[i]->begin();
    }
    
    servo->begin();
    
    Serial.println(F("\n╔════════════════════════════════════╗"));
    Serial.println(F("║  SISTEMA DE RIEGO INTELIGENTE      ║"));
    Serial.println(F("╚════════════════════════════════════╝"));
    Serial.println(F("\nPresiona 'c' para CONFIGURAR"));
    Serial.println(F("Presiona 'v' para VER estado actual\n"));
  }
  
  void configureZone(uint8_t zoneId, uint8_t minHum, uint8_t maxHum, 
                     uint8_t days, uint16_t startMin, uint16_t endMin, uint16_t maxDur) {
    if (zoneId < 3) {
      zones[zoneId]->setHumidityLimits(minHum, maxHum);
      zones[zoneId]->setSchedule(days, startMin, endMin);
      zones[zoneId]->setMaxDuration(maxDur);
    }
  }
  
  void setRainProbability(uint8_t prob) {
    rainProbability = constrain(prob, 0, 100);
  }
  
  bool isRainBlocking() const {
    return rainProbability >= 70;
  }
  
  uint16_t getDayOfYear(int year, int month, int day) {
    static const uint16_t cumDays[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    bool leap = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
    uint16_t doy = cumDays[month - 1] + day;
    if (leap && month > 2) doy++;
    return doy;
  }
  
  void updateSensors() {
    uint32_t now = millis();
    if (now - lastSensorUpdate >= 3000) {    // cada 3 s está bien para DHT11
      lastSensorUpdate = now;
      for (int i = 0; i < 3; i++) {
        zones[i]->updateSensor();            // lectura rápida, sin bloquear
      }
    }
  }

  
  void updateLCD() {
    uint32_t now = millis();
    
    if (now - lastLcdUpdate >= 1000) {
      lastLcdUpdate = now;
      myRTC.updateTime();
      
      lcd.setCursor(0, 0);
      char line0[17];
      snprintf(line0, sizeof(line0), "%02d:%02d:%02d R:%3d%%",
               myRTC.hours, myRTC.minutes, myRTC.seconds, rainProbability);
      lcd.print(line0);
      
      lcd.setCursor(0, 1);
      for (int i = 0; i < 3; i++) {
        lcd.print(zones[i]->getLabel());
        lcd.print(':');
        
        float hum = zones[i]->getHumidity();
        if (isnan(hum)) {
          lcd.print("--");
        } else {
          int val = (int)(hum + 0.5);
          if (val < 10) lcd.print('0');
          lcd.print(val);
        }
        
        if (i < 2) lcd.print(' ');
      }
    }
  }
  
  void update() {
    updateSensors();
    updateLCD();
    
    if (isRainBlocking()) {
      pump->off();
      for (int i = 0; i < 3; i++) {
        zones[i]->stopWatering("probabilidad de lluvia alta");
      }
      return;
    }
    
    myRTC.updateTime();
    uint16_t currentDayOfYear = getDayOfYear(myRTC.year, myRTC.month, myRTC.dayofmonth);
    uint16_t currentTimeMin = myRTC.hours * 60 + myRTC.minutes;
    
    // Verificar si alguna zona está regando
    int wateringZone = -1;
    for (int i = 0; i < 3; i++) {
      if (zones[i]->getWateringState()) {
        wateringZone = i;
        break;
      }
    }
    
    // Si hay una zona regando, verificar si debe detenerse
    if (wateringZone >= 0) {
      zones[wateringZone]->checkStopConditions(rainProbability);
      
      // Si se detuvo, apagar bomba y volver servo a home
      if (!zones[wateringZone]->getWateringState()) {
        pump->off();
        servo->moveTo(90);
        
        Serial.println(F("\n>> Sistema en REPOSO - Esperando siguiente ciclo\n"));
      }
      
      return;  // No evaluar otras zonas mientras una está regando
    }
    
    // Nadie está regando: evaluar zonas en orden de prioridad (1 → 2 → 3)
    for (int i = 0; i < 3; i++) {
      if (zones[i]->shouldWater(currentDayOfYear, currentTimeMin)) {
        // Mover servo a la zona
        servo->moveTo(zones[i]->getServoAngle());
        
        // Encender bomba y zona
        pump->on();
        zones[i]->startWatering(currentDayOfYear);
        
        return;  // Solo una zona a la vez
      }
    }
    
    // Si nadie está regando, asegurar que bomba esté apagada
    pump->off();
  }
  
  // ===== MOSTRAR ESTADO ACTUAL =====
  void showStatus() {
    Serial.println(F("\n╔════════════════════════════════════╗"));
    Serial.println(F("║       ESTADO ACTUAL DEL SISTEMA    ║"));
    Serial.println(F("╚════════════════════════════════════╝"));
    
    myRTC.updateTime();
    Serial.print(F("\nFecha: "));
    Serial.print(myRTC.dayofmonth);
    Serial.print(F("/"));
    Serial.print(myRTC.month);
    Serial.print(F("/"));
    Serial.println(myRTC.year);
    
    Serial.print(F("Hora: "));
    Serial.print(myRTC.hours);
    Serial.print(F(":"));
    if (myRTC.minutes < 10) Serial.print(F("0"));
    Serial.print(myRTC.minutes);
    Serial.print(F(":"));
    if (myRTC.seconds < 10) Serial.print(F("0"));
    Serial.println(myRTC.seconds);
    
    Serial.print(F("\nProbabilidad de lluvia: "));
    Serial.print(rainProbability);
    Serial.print(F("%"));
    if (rainProbability >= 70) {
      Serial.println(F(" [RIEGO BLOQUEADO]"));
    } else {
      Serial.println();
    }
    
    Serial.println(F("\n--- HUMEDADES ACTUALES ---"));
    for (int i = 0; i < 3; i++) {
      Serial.print(F("  Zona "));
      Serial.print(zones[i]->getLabel());
      Serial.print(F(": "));
      
      float hum = zones[i]->getHumidity();
      if (isnan(hum)) {
        Serial.print(F("--"));
      } else {
        Serial.print((int)(hum + 0.5));
        Serial.print(F("%"));
      }
      
      if (zones[i]->getWateringState()) {
        Serial.print(F(" [REGANDO]"));
      }
      Serial.println();
    }
    
    Serial.println(F("\n--- CONFIGURACION ---"));
    for (int i = 0; i < 3; i++) {
      zones[i]->printConfig();
      Serial.println();
    }
    Serial.println(F("═══════════════════════════════════════\n"));
  }
  
  // ===== MENÚ DE CONFIGURACIÓN =====
  void configurationMenu() {
    Serial.println(F("\n╔════════════════════════════════════╗"));
    Serial.println(F("║      MENU DE CONFIGURACION         ║"));
    Serial.println(F("╚════════════════════════════════════╝"));
    Serial.println(F("\n1. Configurar Zona 1 (Patio)"));
    Serial.println(F("2. Configurar Zona 2 (Jardin)"));
    Serial.println(F("3. Configurar Zona 3 (Huerto)"));
    Serial.println(F("4. Configurar probabilidad de lluvia"));
    Serial.println(F("5. Ver estado actual"));
    Serial.println(F("6. Salir"));
    Serial.println(F("\nSelecciona una opcion (1-6): "));
    
    while (!Serial.available()) {
      updateLCD(); // Mantener LCD actualizado
      delay(50);
    }
    
    char option = Serial.read();
    while (Serial.available()) Serial.read(); // Limpiar buffer
    
    Serial.println(option);
    
    if (option >= '1' && option <= '3') {
      configureSpecificZone(option - '1');
    } else if (option == '4') {
      configureRain();
    } else if (option == '5') {
      showStatus();
      delay(3000);
      configurationMenu();
    } else if (option == '6') {
      Serial.println(F("\nSaliendo del menu...\n"));
      return;
    } else {
      Serial.println(F("\nOpcion invalida. Intenta de nuevo."));
      delay(1000);
      configurationMenu();
    }
  }
  
  // ===== CONFIGURAR ZONA ESPECÍFICA =====
  void configureSpecificZone(uint8_t zoneId) {
    if (zoneId >= 3) return;
    
    const char* zoneNames[] = {"Patio", "Jardin", "Huerto"};
    
    Serial.println(F("\n╔════════════════════════════════════╗"));
    Serial.print(F("║   CONFIGURANDO ZONA "));
    Serial.print(zoneId + 1);
    Serial.print(F(" ("));
    Serial.print(zoneNames[zoneId]);
    Serial.println(F(")     ║"));
    Serial.println(F("╚════════════════════════════════════╝"));
    
    // HUMEDAD MÍNIMA
    Serial.println(F("\n>> HUMEDAD MINIMA (0-100%):"));
    Serial.print(F("   Actual: "));
    Serial.print(zones[zoneId]->getMinHumidity());
    Serial.println(F("%"));
    Serial.print(F("   Nueva: "));
    uint8_t minHum = readInt(0, 100);
    
    // HUMEDAD MÁXIMA
    Serial.println(F("\n>> HUMEDAD MAXIMA (0-100%):"));
    Serial.print(F("   Actual: "));
    Serial.print(zones[zoneId]->getMaxHumidity());
    Serial.println(F("%"));
    Serial.print(F("   Nueva: "));
    uint8_t maxHum = readInt(0, 100);
    
    // Validar que max > min
    if (maxHum <= minHum) {
      Serial.println(F("\n[ADVERTENCIA] La humedad maxima debe ser MAYOR a la minima."));
      Serial.println(F("Ajustando automaticamente..."));
      maxHum = minHum + 10;
      Serial.print(F("Nueva humedad maxima: "));
      Serial.println(maxHum);
      delay(2000);
    }
    
    // CADA CUÁNTOS DÍAS REGAR
    Serial.println(F("\n>> REGAR CADA CUANTOS DIAS (1-30):"));
    Serial.print(F("   Actual: "));
    Serial.print(zones[zoneId]->getIntervalDays());
    Serial.println(F(" dias"));
    Serial.print(F("   Nuevo: "));
    uint8_t days = readInt(1, 30);
    
    // HORA DE INICIO
    Serial.println(F("\n>> HORA DE INICIO DEL RIEGO (HH:MM):"));
    Serial.print(F("   Actual: "));
    printTime(zones[zoneId]->getStartTimeMin());
    Serial.print(F("   Nueva: "));
    uint16_t startMin = readTime();
    
    // HORA DE FIN
    Serial.println(F("\n>> HORA DE FIN DEL RIEGO (HH:MM):"));
    Serial.print(F("   Actual: "));
    printTime(zones[zoneId]->getEndTimeMin());
    Serial.print(F("   Nueva: "));
    uint16_t endMin = readTime();
    
    // Validar que fin > inicio
    if (endMin <= startMin) {
      Serial.println(F("\n[ADVERTENCIA] La hora de fin debe ser DESPUES de la hora de inicio."));
      Serial.println(F("Ajustando automaticamente..."));
      endMin = startMin + 30; // Agregar 30 minutos
      Serial.print(F("Nueva hora de fin: "));
      printTime(endMin);
      delay(2000);
    }
    
    // DURACIÓN MÁXIMA
    Serial.println(F("\n>> DURACION MAXIMA POR SESION (1-240 min):"));
    Serial.print(F("   Actual: "));
    Serial.print(zones[zoneId]->getMaxDuration());
    Serial.println(F(" min"));
    Serial.print(F("   Nueva: "));
    uint16_t maxDur = readInt(1, 240);
    
    // APLICAR CONFIGURACIÓN
    zones[zoneId]->setHumidityLimits(minHum, maxHum);
    zones[zoneId]->setSchedule(days, startMin, endMin);
    zones[zoneId]->setMaxDuration(maxDur);
    
    Serial.println(F("\n╔════════════════════════════════════╗"));
    Serial.println(F("║  CONFIGURACION GUARDADA CON EXITO  ║"));
    Serial.println(F("╚════════════════════════════════════╝"));
    
    delay(2000);
    configurationMenu();
  }
  
  // ===== CONFIGURAR PROBABILIDAD DE LLUVIA =====
  void configureRain() {
    Serial.println(F("\n╔════════════════════════════════════╗"));
    Serial.println(F("║   PROBABILIDAD DE LLUVIA           ║"));
    Serial.println(F("╚════════════════════════════════════╝"));
    
    Serial.print(F("\nProbabilidad actual: "));
    Serial.print(rainProbability);
    Serial.println(F("%"));
    
    Serial.print(F("\nNueva probabilidad (0-100%): "));
    uint8_t newProb = readInt(0, 100);
    
    rainProbability = newProb;
    
    Serial.print(F("\nProbabilidad actualizada: "));
    Serial.print(rainProbability);
    Serial.print(F("%"));
    
    if (rainProbability >= 70) {
      Serial.println(F(" [RIEGO BLOQUEADO]"));
    } else {
      Serial.println();
    }
    
    delay(2000);
    configurationMenu();
  }
  
private:
  // ===== FUNCIONES AUXILIARES =====
  
  // Leer un número entero desde Serial
  int readInt(int minVal, int maxVal) {
    while (true) {
      while (!Serial.available()) {
        updateLCD();
        delay(50);
      }
      
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      Serial.println(input);
      
      int value = input.toInt();
      
      if (value >= minVal && value <= maxVal) {
        return value;
      } else {
        Serial.print(F("[ERROR] Valor debe estar entre "));
        Serial.print(minVal);
        Serial.print(F(" y "));
        Serial.println(maxVal);
        Serial.print(F("Intenta de nuevo: "));
      }
    }
  }
  
  // Leer hora en formato HH:MM
  uint16_t readTime() {
    while (true) {
      while (!Serial.available()) {
        updateLCD();
        delay(50);
      }
      
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      Serial.println(input);
      
      int colonPos = input.indexOf(':');
      if (colonPos == -1) {
        Serial.println(F("[ERROR] Formato incorrecto. Use HH:MM (ejemplo: 07:30)"));
        Serial.print(F("Intenta de nuevo: "));
        continue;
      }
      
      int hours = input.substring(0, colonPos).toInt();
      int minutes = input.substring(colonPos + 1).toInt();
      
      if (hours >= 0 && hours <= 23 && minutes >= 0 && minutes <= 59) {
        return (uint16_t)(hours * 60 + minutes);
      } else {
        Serial.println(F("[ERROR] Hora invalida. Horas: 0-23, Minutos: 0-59"));
        Serial.print(F("Intenta de nuevo: "));
      }
    }
  }
  
  // Imprimir tiempo en formato HH:MM
  void printTime(uint16_t timeMin) {
    int hh = timeMin / 60;
    int mm = timeMin % 60;
    
    if (hh < 10) Serial.print(F("0"));
    Serial.print(hh);
    Serial.print(F(":"));
    if (mm < 10) Serial.print(F("0"));
    Serial.print(mm);
    Serial.println();
  }
};

// ===== INSTANCIA GLOBAL =====
IrrigationSystem* irrigationSystem = nullptr;

// ===== SETUP =====
void setup() {
  irrigationSystem = new IrrigationSystem();
  irrigationSystem->begin();
  
  // ===== CONFIGURACIÓN INICIAL POR DEFECTO =====
  // Formato: configureZone(id, minHum, maxHum, días, horaInicio, horaFin, duraciónMáx)
  
  // Zona 1 (Patio): Regar cada 5 días, de 7:00 a 7:30, humedad 30-70%
  irrigationSystem->configureZone(0, 30, 70, 5, 7*60, 7*60+30, 30);
  
  // Zona 2 (Jardín): Regar cada 3 días, de 6:00 a 6:45, humedad 35-75%
  irrigationSystem->configureZone(1, 35, 75, 3, 6*60, 6*60+45, 45);
  
  // Zona 3 (Huerto): Regar cada 2 días, de 7:30 a 8:00, humedad 40-80%
  irrigationSystem->configureZone(2, 40, 80, 2, 7*60+30, 8*60, 30);
  
  // Probabilidad de lluvia inicial
  irrigationSystem->setRainProbability(0);  // 0% = sin lluvia
  
  Serial.println(F("\n✓ Sistema listo y operando"));
  Serial.println(F("\nPresiona 'c' para CONFIGURAR"));
  Serial.println(F("Presiona 'v' para VER estado\n"));
}

// ===== LOOP =====
void loop() {
  // Verificar si hay comandos por Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Limpiar buffer
    while (Serial.available()) {
      Serial.read();
    }
    
    if (cmd == 'c' || cmd == 'C') {
      irrigationSystem->configurationMenu();
    } else if (cmd == 'v' || cmd == 'V') {
      irrigationSystem->showStatus();
    }
  }
  
  // Actualizar sistema de riego
  irrigationSystem->update();
  delay(100);
}