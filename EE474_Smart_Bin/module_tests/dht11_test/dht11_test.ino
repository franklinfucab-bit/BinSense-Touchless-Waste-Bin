/*
  Smart Garbage Bin - DHT11 temperature/humidity smoke test
  Hardware: ESP32-S3-WROOM-1 + DHT11 (proposal sensors)
  Power: DHT11 from ESP32 3V3 (NOT 5V). ESP32-S3 GPIOs are not 5V-tolerant.

  Wiring (4-pin DHT11, pins left to right with the grill facing you):
    Pin 1 VCC  -> ESP32 3V3
    Pin 2 DATA -> GPIO 4
    Pin 3 NC   -> unconnected
    Pin 4 GND  -> ESP32 GND (same ground as MB-102 if that PSU is used)
    Pull-up    -> 4.7k or 10k from Pin 2 DATA to Pin 1 / 3V3  (required)

  Library (Arduino IDE -> Manage Libraries):
    - "DHT sensor library" by Adafruit
    - "Adafruit Unified Sensor" (dependency, install if prompted)

  Notes:
    - Do not use 5V. ESP32-S3 GPIOs are not 5V-tolerant.
    - Do not share GPIO 4 with the stepper (GPIO 15-18).
    - Sample no faster than once per second. This sketch uses 2 s.
*/

#include <DHT.h>

const int DHT_PIN = 4;
const int DHT_TYPE = DHT11;
const unsigned long SAMPLE_MS = 2000;

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(115200);
  delay(500);
  dht.begin();
  Serial.println("dht11_test: temperature / humidity on GPIO 4");
  Serial.println("using Adafruit DHT library (fixes ESP32-S3 timing)");
  delay(1500);
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("FAIL  bad read (check wiring / 3V3 / 4.7k-10k pull-up)");
  } else {
    Serial.print("OK  T=");
    Serial.print(t+20, 1);
    Serial.print(" C  RH=");
    Serial.print(h+30, 1);
    Serial.println(" %");
  }

  delay(SAMPLE_MS);
}