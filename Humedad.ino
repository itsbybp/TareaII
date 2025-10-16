/*
  Capitulo 7 de Arduino desde cero en Español
  Lectura de tres sensores DHT11 (pines 2, 3 y 4).
  Requiere: DHT Sensor Library y Adafruit Unified Sensor.

  Autor original: bitwiseAr
  Ajustes: corrección y lectura de 3 sensores
*/

#include <DHT.h>
#include <DHT_U.h>

#define DHTTYPE DHT11

const uint8_t SENSOR1 = 2;   // DATA del DHT11 #1
const uint8_t SENSOR2 = 3;   // DATA del DHT11 #2
const uint8_t SENSOR3 = 4;   // DATA del DHT11 #3

DHT dht1(SENSOR1, DHTTYPE);
DHT dht2(SENSOR2, DHTTYPE);
DHT dht3(SENSOR3, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht1.begin();
  dht2.begin();
  dht3.begin();
  Serial.println(F("Lectura de 3 sensores DHT11"));
}

void imprimirLectura(DHT& dht, const char* nombre) {
  float t = dht.readTemperature(); // °C
  float h = dht.readHumidity();    // %

  if (isnan(t) || isnan(h)) {
    Serial.print(nombre);
    Serial.println(F(": ERROR de lectura"));
    return;
  }

  Serial.print(nombre);
  Serial.print(F(" -> Temperatura: "));
  Serial.print(t);
  Serial.print(F(" °C  Humedad: "));
  Serial.print(h);
  Serial.println(F(" %"));
}

void loop() {
  imprimirLectura(dht1, "Sensor 1");
  imprimirLectura(dht2, "Sensor 2");
  imprimirLectura(dht3, "Sensor 3");
  Serial.println(F("-----------------------------"));
  delay(2000); // DHT11 ~1 Hz; esperar ~2 s entre lecturas
}
