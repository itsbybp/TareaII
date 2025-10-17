/*
  ═══════════════════════════════════════════════════════════════════════
  SISTEMA DE RIEGO INTELIGENTE CON WIFI
  ═══════════════════════════════════════════════════════════════════════
  
  Funcionalidades:
  - 3 zonas de riego con control individual
  - Riego programado por horario (cada N días)
  - Riego por humedad mínima (fuera de horario)
  - Bloqueo automático si probabilidad de lluvia ≥ 70%
  - Actualización WiFi cada 10 minutos
  
  Hardware:
  - Arduino Mega 2560
  - LCD 16x2 (muestra hora y humedades)
  - DS1302 RTC (reloj en tiempo real)
  - 3x DHT11 (sensores de humedad)
  - 3x Relés (control de zonas)
  - 1x Servo (dirección del riego)
  - 1x Bomba de agua
  - ESP8266 (WiFi)
  
  Comandos:
  - 'c' o 'C' = Menú de configuración
  - 'v' o 'V' = Ver estado actual
*/

// ═══════════════════════════════════════════════════════════════════════
// LIBRERÍAS
// ═══════════════════════════════════════════════════════════════════════
#include <LiquidCrystal.h>
#include <virtuabotixRTC.h>
#include <DHT.h>
#include <Servo.h>

// ═══════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN WIFI
// ═══════════════════════════════════════════════════════════════════════
const char* WIFI_SSID = "Casa - Tec Arriba 2.4";
const char* WIFI_PASS = "Eslaclave2";
const String LAT = "9.8626";  // Cartago, Costa Rica
const String LON = "-83.9195";

// ═══════════════════════════════════════════════════════════════════════
// PINES DE HARDWARE
// ═══════════════════════════════════════════════════════════════════════
const uint8_t PIN_BOMBA = 53;
const uint8_t PIN_RELAY[3] = {40, 42, 44};  // Relés de zonas 1, 2, 3
const uint8_t PIN_DHT[3] = {2, 3, 4};        // Sensores DHT11 zonas 1, 2, 3
const uint8_t PIN_SERVO = 9;

// LCD: RS=32, E=30, D4=28, D5=26, D6=24, D7=22
LiquidCrystal lcd(32, 30, 28, 26, 24, 22);

// RTC: CLK=6, DAT=7, RST=8
virtuabotixRTC myRTC(6, 7, 8);

// ═══════════════════════════════════════════════════════════════════════
// VARIABLES GLOBALES DE LLUVIA
// ═══════════════════════════════════════════════════════════════════════
// ►►► AQUÍ PUEDES CAMBIAR LA PROBABILIDAD MANUALMENTE ◄◄◄
// Si quieres forzar un valor sin usar WiFi, cambia esta línea:
// int rainProbability = 80;  // Ejemplo: forzar 80%
int rainProbability = 80;  // -1 = sin datos aún

bool b = rainProbability<0;

unsigned long ultimaActualizacionLluvia = 0;
const unsigned long INTERVALO_LLUVIA = 600000;  // 10 minutos en milisegundos

// ═══════════════════════════════════════════════════════════════════════
// CLASE: SENSOR DE HUMEDAD (DHT11)
// ═══════════════════════════════════════════════════════════════════════
class HumiditySensor {
private:
  DHT dht;
  float lastReading;  // Última lectura válida
  
public:
  HumiditySensor(uint8_t pin) : dht(pin, DHT11), lastReading(NAN) {}
  
  void begin() {
    dht.begin();
  }
  
  // Lee humedad y guarda solo si es válida
  float read() {
    float h = dht.readHumidity();
    if (!isnan(h) && h >= 0 && h <= 100) {
      lastReading = h;
    }
    return lastReading;
  }
  
  float getLast() const { return lastReading; }
  bool isValid() const { return !isnan(lastReading); }
};

// ═══════════════════════════════════════════════════════════════════════
// CLASE: ZONA DE RIEGO
// ═══════════════════════════════════════════════════════════════════════
class IrrigationZone {
private:
  // Identificación
  uint8_t id;           // 0, 1, 2
  char label;           // 'P', 'J', 'H'
  uint8_t relayPin;     // Pin del relé
  int servoAngle;       // Ángulo del servo para esta zona
  HumiditySensor* sensor;
  
  // Configuración de riego
  uint8_t minHumidity;      // Humedad mínima (para riego fuera de horario)
  uint8_t maxHumidity;      // Humedad máxima (para detener riego)
  uint8_t intervalDays;     // Cada cuántos días regar
  uint16_t startTimeMin;    // Hora inicio ventana (en minutos desde medianoche)
  uint16_t endTimeMin;      // Hora fin ventana (en minutos)
  uint16_t maxDurationMin;  // Duración máxima de riego
  
  // Estado actual
  bool isWatering;              // ¿Está regando ahora?
  uint32_t wateringStartMillis; // Momento en que empezó a regar
  uint16_t lastScheduleDay;     // Último día que regó por horario
  bool startedBySchedule;       // ¿Este riego inició por horario?
  
public:
  IrrigationZone(uint8_t zoneId, char zoneLbl, uint8_t relay, int servo, uint8_t dhtPin)
    : id(zoneId), label(zoneLbl), relayPin(relay), servoAngle(servo),
      minHumidity(40), maxHumidity(80), intervalDays(5),
      startTimeMin(7 * 60), endTimeMin(7 * 60 + 30), maxDurationMin(30),
      isWatering(false), wateringStartMillis(0), lastScheduleDay(0),
      startedBySchedule(false) {
    
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
  
  // ─── CONFIGURACIÓN ───
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
  
  // ─── GETTERS ───
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
  
  // ─── CONTROL DE RELÉ ───
  void startRelay() { digitalWrite(relayPin, LOW); }   // Relés activos en LOW
  void stopRelay() { digitalWrite(relayPin, HIGH); }
  
  void updateSensor() {
    sensor->read();
  }
  
  // ─── DETERMINAR SI DEBE REGAR ───
  bool shouldWater(uint16_t currentDayOfYear, uint16_t currentTimeMin) {
    if (isWatering) return false;

    // Verificar si estamos dentro de la ventana horaria
    bool inWindow = (startTimeMin <= endTimeMin)
      ? (currentTimeMin >= startTimeMin && currentTimeMin < endTimeMin)
      : (currentTimeMin >= startTimeMin || currentTimeMin < endTimeMin);

    // REGLA 1: RIEGO POR HORARIO (cada N días, dentro de ventana)
    if (intervalDays > 0) {
      int daysSinceLastWatering = (int)currentDayOfYear - (int)lastScheduleDay;
      if (daysSinceLastWatering < 0) daysSinceLastWatering += 365;  // Wrap de año
      
      if (daysSinceLastWatering >= intervalDays && inWindow) {
        return true;  // Regar por horario
      }
    }

    // REGLA 2: RIEGO POR HUMEDAD BAJA (fuera de ventana)
    if (!inWindow && sensor->isValid() && sensor->getLast() < minHumidity) {
      return true;  // Regar por humedad
    }

    return false;
  }
  
  // ─── INICIAR RIEGO ───
  void startWatering(uint16_t currentDayOfYear) {
    isWatering = true;
    wateringStartMillis = millis();
    startRelay();

    // Determinar si inició por horario o por humedad
    uint16_t t = (uint16_t)myRTC.hours * 60u + (uint16_t)myRTC.minutes;
    bool inWindow = (startTimeMin <= endTimeMin)
      ? (t >= startTimeMin && t < endTimeMin)
      : (t >= startTimeMin || t < endTimeMin);

    startedBySchedule = inWindow;
    
    // Solo actualizar calendario si fue por horario
    if (startedBySchedule) {
      lastScheduleDay = currentDayOfYear;
    }
    
    Serial.print(F(">> Zona "));
    Serial.print(label);
    Serial.print(F(" INICIANDO - "));
    Serial.println(startedBySchedule ? F("por horario") : F("por humedad"));
  }
  
  // ─── DETENER RIEGO ───
  void stopWatering(const char* reason) {
    if (isWatering) {
      isWatering = false;
      startedBySchedule = false;
      stopRelay();
      
      Serial.print(F(">> Zona "));
      Serial.print(label);
      Serial.print(F(" DETENIDA - "));
      Serial.println(reason);
    }
  }
  
  // ─── VERIFICAR CONDICIONES DE PARO ───
  void checkStopConditions() {
    if (!isWatering) return;

    uint16_t t = (uint16_t)myRTC.hours * 60u + (uint16_t)myRTC.minutes;
    
    // Si inició por horario: solo detener al salir de ventana
    if (startedBySchedule) {
      bool outOfWindow = (startTimeMin <= endTimeMin)
        ? (t < startTimeMin || t >= endTimeMin)
        : (t >= endTimeMin && t < startTimeMin);
      
      if (outOfWindow) {
        stopWatering("fin de ventana horaria");
      }
      return;  // En ventana: ignorar humedad y duración
    }

    // Si NO inició por horario: aplicar límites
    sensor->read();

    // Límite de duración máxima
    uint32_t elapsedMin = (millis() - wateringStartMillis) / 60000UL;
    if (elapsedMin >= maxDurationMin) {
      stopWatering("tiempo maximo alcanzado");
      return;
    }

    // Límite de humedad
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
  
  // ─── IMPRIMIR CONFIGURACIÓN ───
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

// ═══════════════════════════════════════════════════════════════════════
// CLASE: SERVO (Dirección del riego)
// ═══════════════════════════════════════════════════════════════════════
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
  
  // Mover suavemente al ángulo objetivo
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

// ═══════════════════════════════════════════════════════════════════════
// CLASE: BOMBA DE AGUA
// ═══════════════════════════════════════════════════════════════════════
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
  
  bool isOn() const { return state; }
};

// ═══════════════════════════════════════════════════════════════════════
// FUNCIONES WIFI (ESP8266)
// ═══════════════════════════════════════════════════════════════════════

// Enviar comando AT y esperar respuesta
String enviarComandoESP(String cmd, int espera) {
  while(Serial1.available()) Serial1.read();  // Limpiar buffer
  
  Serial1.println(cmd);
  String respuesta = "";
  unsigned long inicio = millis();
  
  while (millis() - inicio < espera) {
    while (Serial1.available()) {
      respuesta += (char)Serial1.read();
    }
  }
  
  return respuesta;
}

// Conectar a WiFi
bool conectarWiFi() {
  Serial.println(F("Conectando WiFi..."));
  enviarComandoESP("AT", 1000);
  enviarComandoESP("AT+CWMODE=1", 2000);
  
  String cmd = "AT+CWJAP=\"" + String(WIFI_SSID) + "\",\"" + String(WIFI_PASS) + "\"";
  String resp = enviarComandoESP(cmd, 15000);
  
  bool connected = (resp.indexOf("OK") >= 0 || resp.indexOf("ALREADY") >= 0);
  
  if (connected) {
    Serial.println(F("WiFi conectado!"));
  } else {
    Serial.println(F("Error al conectar WiFi"));
  }
  
  return connected;
}

// Obtener probabilidad de lluvia desde API
void obtenerProbabilidadLluvia() {
  String servidor = "api.open-meteo.com";
  String ruta = "/v1/forecast?latitude=" + LAT + 
                "&longitude=" + LON + 
                "&current=precipitation_probability" +
                "&timezone=America/Costa_Rica";
  
  // 1. Cerrar conexión previa
  enviarComandoESP("AT+CIPCLOSE", 500);
  delay(1000);
  
  // 2. Conectar al servidor
  Serial1.println("AT+CIPSTART=\"TCP\",\"" + servidor + "\",80");
  delay(6000);
  while(Serial1.available()) Serial1.read();
  
  // 3. Construir petición HTTP GET
  String peticion = "GET " + ruta + " HTTP/1.1\r\n";
  peticion += "Host: " + servidor + "\r\n";
  peticion += "Connection: close\r\n\r\n";
  
  // 4. Indicar tamaño de datos a enviar
  Serial1.println("AT+CIPSEND=" + String(peticion.length()));
  delay(3000);
  while(Serial1.available()) Serial1.read();
  
  // 5. Enviar petición HTTP
  Serial1.print(peticion);
  
  // 6. Leer respuesta (byte a byte, con paciencia)
  String respuesta = "";
  unsigned long inicioLectura = millis();
  unsigned long ultimoDato = millis();
  
  while (millis() - inicioLectura < 40000) {  // Timeout 40 segundos
    while (Serial1.available()) {
      char c = Serial1.read();
      respuesta += c;
      ultimoDato = millis();
    }
    
    // Si no hay datos por 8 segundos, terminar
    if (millis() - ultimoDato > 8000) break;
    
    // Si vemos CLOSED, terminar
    if (respuesta.indexOf("CLOSED") > 0) break;
    
    delay(5);
  }
  
  // 7. Cerrar conexión
  enviarComandoESP("AT+CIPCLOSE", 500);
  
  // 8. PARSEAR JSON - Buscar la sección "current"
  int posCurrentStart = respuesta.indexOf("\"current\":{");
  
  if (posCurrentStart >= 0) {
    int pos = respuesta.indexOf("precipitation_probability", posCurrentStart);
    if (pos > 0) {
      int posDosPuntos = respuesta.indexOf(":", pos);
      if (posDosPuntos > 0) {
        String numStr = "";
        
        // Saltar espacios, comillas, etc.
        int startPos = posDosPuntos + 1;
        while (startPos < respuesta.length() && 
               (respuesta[startPos] == ' ' || 
                respuesta[startPos] == '"' || 
                respuesta[startPos] == '\n' || 
                respuesta[startPos] == '\r')) {
          startPos++;
        }
        
        // Leer dígitos del número
        for (int i = startPos; i < respuesta.length() && i < startPos + 10; i++) {
          char c = respuesta[i];
          if (c >= '0' && c <= '9') {
            numStr += c;
          } else if (numStr.length() > 0) {
            break;
          }
        }
        
        if (b) {
          // ►►► AQUÍ SE ACTUALIZA LA PROBABILIDAD DESDE LA API ◄◄◄
          if (numStr.length() > 0) {
            rainProbability = numStr.toInt();
            
            Serial.print(F(">> Probabilidad de lluvia actualizada: "));
            Serial.print(rainProbability);
            Serial.println(F("%"));
          }
        }
      }
    }
  }
  
  if (rainProbability < 0) {
    Serial.println(F(">> Error al obtener probabilidad de lluvia"));
  }
}

// ═══════════════════════════════════════════════════════════════════════
// CLASE: SISTEMA DE RIEGO (Controlador principal)
// ═══════════════════════════════════════════════════════════════════════
class IrrigationSystem {
private:
  IrrigationZone* zones[3];
  ServoController* servo;
  WaterPump* pump;
  uint32_t lastSensorUpdate;
  uint32_t lastLcdUpdate;
  
public:
  IrrigationSystem() : lastSensorUpdate(0), lastLcdUpdate(0) {
    // Crear las 3 zonas: Patio (30°), Jardín (90°), Huerto (150°)
    zones[0] = new IrrigationZone(0, 'P', PIN_RELAY[0], 30, PIN_DHT[0]);
    zones[1] = new IrrigationZone(1, 'J', PIN_RELAY[1], 90, PIN_DHT[1]);
    zones[2] = new IrrigationZone(2, 'H', PIN_RELAY[2], 150, PIN_DHT[2]);
    
    servo = new ServoController(PIN_SERVO);
    pump = new WaterPump(PIN_BOMBA);
  }
  
  ~IrrigationSystem() {
    for (int i = 0; i < 3; i++) delete zones[i];
    delete servo;
    delete pump;
  }
  
  // ─── INICIALIZACIÓN ───
  void begin() {
    Serial.begin(9600);
    Serial1.begin(9600);   // Serial1 para ESP8266
    Serial1.setTimeout(30000);
    
    lcd.begin(16, 2);
    
    for (int i = 0; i < 3; i++) {
      zones[i]->begin();
    }
    
    servo->begin();
    
    Serial.println(F("\n╔═══════════════════════════════════╗"));
    Serial.println(F("║  SISTEMA DE RIEGO INTELIGENTE     ║"));
    Serial.println(F("╚═══════════════════════════════════╝"));
    
    // Inicializar WiFi y obtener primera lectura
    delay(2000);
    enviarComandoESP("AT+CIPMUX=0", 1000);
    
    if (conectarWiFi()) {
      Serial.println(F("\n>> Obteniendo primera actualización de lluvia..."));
      obtenerProbabilidadLluvia();
      ultimaActualizacionLluvia = millis();
    } else {
      // ►►► SI NO HAY WIFI, PUEDES FORZAR UN VALOR AQUÍ ◄◄◄
      rainProbability = 0;  // Asumir 0% si no hay WiFi
      Serial.println(F(">> Sin WiFi - Usando probabilidad de lluvia: 0%"));
    }
    
    Serial.println(F("\nPresiona 'c' para CONFIGURAR"));
    Serial.println(F("Presiona 'v' para VER estado actual\n"));
  }
  
  // ─── CONFIGURAR ZONA ───
  void configureZone(uint8_t zoneId, uint8_t minHum, uint8_t maxHum, 
                     uint8_t days, uint16_t startMin, uint16_t endMin, uint16_t maxDur) {
    if (zoneId < 3) {
      zones[zoneId]->setHumidityLimits(minHum, maxHum);
      zones[zoneId]->setSchedule(days, startMin, endMin);
      zones[zoneId]->setMaxDuration(maxDur);
    }
  }
  
  // ─── VERIFICAR SI LLUVIA BLOQUEA RIEGO ───
  bool isRainBlocking() const {
    return rainProbability >= 70;
  }
  
  // ─── CALCULAR DÍA DEL AÑO ───
  uint16_t getDayOfYear(int year, int month, int day) {
    static const uint16_t cumDays[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    bool leap = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
    uint16_t doy = cumDays[month - 1] + day;
    if (leap && month > 2) doy++;
    return doy;
  }
  
  // ─── ACTUALIZAR SENSORES (cada 3 segundos) ───
  void updateSensors() {
    uint32_t now = millis();
    if (now - lastSensorUpdate >= 3000) {
      lastSensorUpdate = now;
      for (int i = 0; i < 3; i++) {
        zones[i]->updateSensor();
      }
    }
  }
  
  // ─── ACTUALIZAR LCD (cada 1 segundo) ───
  void updateLCD() {
    uint32_t now = millis();
    
    if (now - lastLcdUpdate >= 1000) {
      lastLcdUpdate = now;
      myRTC.updateTime();
      
      // Línea 0: HH:MM:SS R:XXX%
      lcd.setCursor(0, 0);
      char line0[17];
      int rainDisplay = (rainProbability < 0) ? 0 : rainProbability;
      snprintf(line0, sizeof(line0), "%02d:%02d:%02d R:%3d%%",
               myRTC.hours, myRTC.minutes, myRTC.seconds, rainDisplay);
      lcd.print(line0);
      
      // Línea 1: P:XX J:XX H:XX (humedades)
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
  
  // ─── ACTUALIZAR PROBABILIDAD DE LLUVIA (cada 10 minutos) ───
  void updateRainProbability() {
    if (millis() - ultimaActualizacionLluvia >= INTERVALO_LLUVIA) {
      Serial.println(F("\n>> Actualización automática de probabilidad de lluvia..."));
      obtenerProbabilidadLluvia();
      ultimaActualizacionLluvia = millis();
    }
  }
  
  // ─── BUCLE PRINCIPAL DE ACTUALIZACIÓN ───
  void update() {
    updateSensors();
    updateLCD();
    updateRainProbability();  // ◄── ACTUALIZACIÓN AUTOMÁTICA WIFI
    
    // BLOQUEO POR LLUVIA: Si probabilidad ≥ 70%, detener todo
    if (isRainBlocking()) {
      pump->off();
      for (int i = 0; i < 3; i++) {
        zones[i]->stopWatering("probabilidad de lluvia alta");
      }
      return;
    }
    
    // Obtener hora actual
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
      zones[wateringZone]->checkStopConditions();
      
      // Si se detuvo, apagar bomba y volver servo a home
      if (!zones[wateringZone]->getWateringState()) {
        pump->off();
        servo->moveTo(90);
        Serial.println(F("\n>> Sistema en REPOSO\n"));
      }
      
      return;  // No evaluar otras zonas mientras una está regando
    }
    
    // Nadie está regando: evaluar zonas por prioridad (1 → 2 → 3)
    for (int i = 0; i < 3; i++) {
      if (zones[i]->shouldWater(currentDayOfYear, currentTimeMin)) {
        servo->moveTo(zones[i]->getServoAngle());
        pump->on();
        zones[i]->startWatering(currentDayOfYear);
        return;  // Solo una zona a la vez
      }
    }
    
    // Si nadie está regando, asegurar que bomba esté apagada
    pump->off();
  }
  
  // ─── MOSTRAR ESTADO ACTUAL ───
  void showStatus() {
    Serial.println(F("\n╔═══════════════════════════════════╗"));
    Serial.println(F("║       ESTADO ACTUAL DEL SISTEMA   ║"));
    Serial.println(F("╚═══════════════════════════════════╝"));
    
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
    if (rainProbability < 0) {
      Serial.println(F("-- (sin datos WiFi)"));
    } else {
      Serial.print(rainProbability);
      Serial.print(F("%"));
      if (rainProbability >= 70) {
        Serial.println(F(" [RIEGO BLOQUEADO]"));
      } else {
        Serial.println();
      }
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
    Serial.println(F("═══════════════════════════════════\n"));
  }
  
  // ─── MENÚ DE CONFIGURACIÓN ───
  void configurationMenu() {
    Serial.println(F("\n╔═══════════════════════════════════╗"));
    Serial.println(F("║      MENU DE CONFIGURACION        ║"));
    Serial.println(F("╚═══════════════════════════════════╝"));
    Serial.println(F("\n1. Configurar Zona 1 (Patio)"));
    Serial.println(F("2. Configurar Zona 2 (Jardin)"));
    Serial.println(F("3. Configurar Zona 3 (Huerto)"));
    Serial.println(F("4. Actualizar probabilidad de lluvia AHORA"));
    Serial.println(F("5. Ver estado actual"));
    Serial.println(F("6. Salir"));
    Serial.println(F("\nSelecciona una opcion (1-6): "));
    
    while (!Serial.available()) {
      updateLCD();
      delay(50);
    }
    
    char option = Serial.read();
    while (Serial.available()) Serial.read();
    
    Serial.println(option);
    
    if (option >= '1' && option <= '3') {
      configureSpecificZone(option - '1');
    } else if (option == '4') {
      Serial.println(F("\n>> Actualizando probabilidad de lluvia..."));
      obtenerProbabilidadLluvia();
      ultimaActualizacionLluvia = millis();
      delay(2000);
      configurationMenu();
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
  
  // ─── CONFIGURAR ZONA ESPECÍFICA ───
  void configureSpecificZone(uint8_t zoneId) {
    if (zoneId >= 3) return;
    
    const char* zoneNames[] = {"Patio", "Jardin", "Huerto"};
    
    Serial.println(F("\n╔═══════════════════════════════════╗"));
    Serial.print(F("║   CONFIGURANDO ZONA "));
    Serial.print(zoneId + 1);
    Serial.print(F(" ("));
    Serial.print(zoneNames[zoneId]);
    Serial.println(F(")     ║"));
    Serial.println(F("╚═══════════════════════════════════╝"));
    
    Serial.println(F("\n>> HUMEDAD MINIMA (0-100%):"));
    Serial.print(F("   Actual: "));
    Serial.print(zones[zoneId]->getMinHumidity());
    Serial.println(F("%"));
    Serial.print(F("   Nueva: "));
    uint8_t minHum = readInt(0, 100);
    
    Serial.println(F("\n>> HUMEDAD MAXIMA (0-100%):"));
    Serial.print(F("   Actual: "));
    Serial.print(zones[zoneId]->getMaxHumidity());
    Serial.println(F("%"));
    Serial.print(F("   Nueva: "));
    uint8_t maxHum = readInt(0, 100);
    
    if (maxHum <= minHum) {
      Serial.println(F("\n[ADVERTENCIA] La humedad maxima debe ser MAYOR a la minima."));
      Serial.println(F("Ajustando automaticamente..."));
      maxHum = minHum + 10;
      Serial.print(F("Nueva humedad maxima: "));
      Serial.println(maxHum);
      delay(2000);
    }
    
    Serial.println(F("\n>> REGAR CADA CUANTOS DIAS (1-30):"));
    Serial.print(F("   Actual: "));
    Serial.print(zones[zoneId]->getIntervalDays());
    Serial.println(F(" dias"));
    Serial.print(F("   Nuevo: "));
    uint8_t days = readInt(1, 30);
    
    Serial.println(F("\n>> HORA DE INICIO DEL RIEGO (HH:MM):"));
    Serial.print(F("   Actual: "));
    printTime(zones[zoneId]->getStartTimeMin());
    Serial.print(F("   Nueva: "));
    uint16_t startMin = readTime();
    
    Serial.println(F("\n>> HORA DE FIN DEL RIEGO (HH:MM):"));
    Serial.print(F("   Actual: "));
    printTime(zones[zoneId]->getEndTimeMin());
    Serial.print(F("   Nueva: "));
    uint16_t endMin = readTime();
    
    if (endMin <= startMin) {
      Serial.println(F("\n[ADVERTENCIA] La hora de fin debe ser DESPUES de la hora de inicio."));
      Serial.println(F("Ajustando automaticamente..."));
      endMin = startMin + 30;
      Serial.print(F("Nueva hora de fin: "));
      printTime(endMin);
      delay(2000);
    }
    
    Serial.println(F("\n>> DURACION MAXIMA POR SESION (1-240 min):"));
    Serial.print(F("   Actual: "));
    Serial.print(zones[zoneId]->getMaxDuration());
    Serial.println(F(" min"));
    Serial.print(F("   Nueva: "));
    uint16_t maxDur = readInt(1, 240);
    
    zones[zoneId]->setHumidityLimits(minHum, maxHum);
    zones[zoneId]->setSchedule(days, startMin, endMin);
    zones[zoneId]->setMaxDuration(maxDur);
    
    Serial.println(F("\n╔═══════════════════════════════════╗"));
    Serial.println(F("║  CONFIGURACION GUARDADA CON EXITO ║"));
    Serial.println(F("╚═══════════════════════════════════╝"));
    
    delay(2000);
    configurationMenu();
  }
  
private:
  // ─── LEER NÚMERO ENTERO DESDE SERIAL ───
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
  
  // ─── LEER HORA EN FORMATO HH:MM ───
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
  
  // ─── IMPRIMIR TIEMPO EN FORMATO HH:MM ───
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

// ═══════════════════════════════════════════════════════════════════════
// PROGRAMA PRINCIPAL
// ═══════════════════════════════════════════════════════════════════════

IrrigationSystem* irrigationSystem = nullptr;

void setup() {
  irrigationSystem = new IrrigationSystem();
  irrigationSystem->begin();
  
  // ─── CONFIGURACIÓN INICIAL POR DEFECTO ───
  // Formato: configureZone(id, minHum, maxHum, días, horaInicio, horaFin, duraciónMáx)
  
  // Zona 1 (Patio): Regar cada 5 días, de 7:00 a 7:30, humedad 30-70%
  irrigationSystem->configureZone(0, 30, 70, 5, 7*60, 7*60+30, 30);
  
  // Zona 2 (Jardín): Regar cada 3 días, de 6:00 a 6:45, humedad 35-75%
  irrigationSystem->configureZone(1, 35, 75, 3, 6*60, 6*60+45, 45);
  
  // Zona 3 (Huerto): Regar cada 2 días, de 7:30 a 8:00, humedad 40-80%
  irrigationSystem->configureZone(2, 40, 80, 2, 7*60+30, 8*60, 30);
  
  Serial.println(F("\n✓ Sistema listo y operando"));
  Serial.println(F("✓ Actualización automática de lluvia cada 10 minutos"));
  Serial.println(F("\nPresiona 'c' para CONFIGURAR"));
  Serial.println(F("Presiona 'v' para VER estado\n"));
}

void loop() {
  // Verificar comandos por Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();  // Limpiar buffer
    
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

// ═══════════════════════════════════════════════════════════════════════
// NOTAS IMPORTANTES:
// ═══════════════════════════════════════════════════════════════════════
/*
  ►►► CÓMO CAMBIAR LA PROBABILIDAD DE LLUVIA MANUALMENTE ◄◄◄
  
  Hay 3 lugares donde puedes modificar rainProbability:
  
  1. LÍNEA 60 (Variable global inicial):
     int rainProbability = -1;  // Cambiar a: int rainProbability = 80;
  
  2. LÍNEA 546 (Si no hay WiFi al iniciar):
     rainProbability = 0;  // Cambiar a: rainProbability = 80;
  
  3. LÍNEA 467 (Después de obtener de API):
     rainProbability = numStr.toInt();  // Cambiar a: rainProbability = 80;
  
  Si quieres DESHABILITAR completamente el WiFi y usar valor manual:
  - Comenta la línea que llama a obtenerProbabilidadLluvia() en begin()
  - Comenta updateRainProbability() en update()
  - Establece rainProbability = TU_VALOR en setup()
  
  ═══════════════════════════════════════════════════════════════════════
  LÓGICA DE RIEGO:
  ═══════════════════════════════════════════════════════════════════════
  
  1. RIEGO POR HORARIO (dentro de ventana):
     - Cada N días (configurable)
     - Solo entre hora inicio y hora fin
     - Ignora humedad
     - Se detiene al salir de ventana
  
  2. RIEGO POR HUMEDAD (fuera de ventana):
     - Si humedad < mínima
     - Se detiene cuando alcanza mínima o máxima
     - Respeta duración máxima
  
  3. BLOQUEO POR LLUVIA:
     - Si probabilidad ≥ 70%, NO riega
     - Detiene cualquier riego en curso
  
  ═══════════════════════════════════════════════════════════════════════
*/
