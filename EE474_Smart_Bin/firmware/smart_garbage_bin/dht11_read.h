/*
 * Filename: dht11_read.h
 * Direct DHT11 reader for ESP32-S3. Adafruit's bit-bang is easily corrupted
 * by pulseIn() / FreeRTOS; this uses a short timeout and noInterrupts().
 */
#pragma once

#include <Arduino.h>

inline bool readDht11(uint8_t pin, float *temperatureC, float *humidityPct) {
  uint8_t bits[5] = {0};

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(20);
  digitalWrite(pin, HIGH);
  delayMicroseconds(30);
  pinMode(pin, INPUT_PULLUP);

  noInterrupts();

  uint32_t t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if ((micros() - t0) > 90) {
      interrupts();
      return false;
    }
  }
  t0 = micros();
  while (digitalRead(pin) == LOW) {
    if ((micros() - t0) > 100) {
      interrupts();
      return false;
    }
  }
  t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if ((micros() - t0) > 100) {
      interrupts();
      return false;
    }
  }

  for (int i = 0; i < 40; ++i) {
    t0 = micros();
    while (digitalRead(pin) == LOW) {
      if ((micros() - t0) > 80) {
        interrupts();
        return false;
      }
    }
    t0 = micros();
    while (digitalRead(pin) == HIGH) {
      if ((micros() - t0) > 90) {
        interrupts();
        return false;
      }
    }
    bits[i / 8] <<= 1;
    if ((micros() - t0) > 40) {
      bits[i / 8] |= 1;
    }
  }

  interrupts();

  const uint8_t sum =
      (uint8_t)((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF);
  if (sum != bits[4]) {
    return false;
  }

  *humidityPct = (float)bits[0];
  *temperatureC = (float)(bits[2] & 0x7F);
  if (bits[2] & 0x80) {
    *temperatureC = -*temperatureC;
  }
  return true;
}
