/*
 * Filename: dht_diag.ino
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 21, 2026
 * Description: Isolates the DHT11 from the rest of the Smart Garbage Bin.
 *              Phase 1 emits a slow square wave so the scope probe can be
 *              confirmed on the right pin. Phase 2 reads the sensor and
 *              reports which step of the handshake failed, with measured
 *              pulse widths.
 *
 * Set DHT_PIN below to whichever pin you are testing.
 * Board: ESP32S3 Dev Module, Arduino-ESP32 3.3.10
 */

#include <Arduino.h>

constexpr uint8_t DHT_PIN = 10;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint8_t IDENTITY_CYCLES = 10;

enum class DhtStep : uint8_t {
  OK,
  NO_ANSWER,
  PRESENCE_LOW_STUCK,
  PRESENCE_HIGH_STUCK,
  BIT_LOW_TIMEOUT,
  BIT_HIGH_TIMEOUT,
  BAD_CHECKSUM
};

struct DhtResult {
  DhtStep step;
  uint8_t bitIndex;
  uint32_t presenceLowUs;
  uint32_t presenceHighUs;
  uint32_t firstBitHighUs;
  uint8_t raw[5];
};

const char *stepName(DhtStep step) {
  switch (step) {
    case DhtStep::OK:
      return "OK";
    case DhtStep::NO_ANSWER:
      return "NO_ANSWER - sensor never pulled the line low";
    case DhtStep::PRESENCE_LOW_STUCK:
      return "PRESENCE_LOW_STUCK - line held low too long";
    case DhtStep::PRESENCE_HIGH_STUCK:
      return "PRESENCE_HIGH_STUCK - line held high too long";
    case DhtStep::BIT_LOW_TIMEOUT:
      return "BIT_LOW_TIMEOUT - frame stopped partway";
    case DhtStep::BIT_HIGH_TIMEOUT:
      return "BIT_HIGH_TIMEOUT - frame stopped partway";
    default:
      return "BAD_CHECKSUM - bits arrived but did not add up";
  }
}

// Waits while the pin stays at `level`. Returns false on timeout.
static bool waitWhile(uint8_t pin, int level, uint32_t timeoutUs,
                      uint32_t *elapsedUs) {
  const uint32_t start = micros();
  while (digitalRead(pin) == level) {
    if ((micros() - start) > timeoutUs) {
      *elapsedUs = micros() - start;
      return false;
    }
  }
  *elapsedUs = micros() - start;
  return true;
}

DhtResult readDiag(uint8_t pin) {
  DhtResult result = {};
  uint32_t width = 0;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(20);
  digitalWrite(pin, HIGH);
  delayMicroseconds(30);
  pinMode(pin, INPUT_PULLUP);

  noInterrupts();

  if (!waitWhile(pin, HIGH, 200, &width)) {
    interrupts();
    result.step = DhtStep::NO_ANSWER;
    return result;
  }

  if (!waitWhile(pin, LOW, 300, &result.presenceLowUs)) {
    interrupts();
    result.step = DhtStep::PRESENCE_LOW_STUCK;
    return result;
  }

  if (!waitWhile(pin, HIGH, 300, &result.presenceHighUs)) {
    interrupts();
    result.step = DhtStep::PRESENCE_HIGH_STUCK;
    return result;
  }

  for (uint8_t i = 0; i < 40; ++i) {
    if (!waitWhile(pin, LOW, 200, &width)) {
      interrupts();
      result.step = DhtStep::BIT_LOW_TIMEOUT;
      result.bitIndex = i;
      return result;
    }

    if (!waitWhile(pin, HIGH, 200, &width)) {
      interrupts();
      result.step = DhtStep::BIT_HIGH_TIMEOUT;
      result.bitIndex = i;
      return result;
    }

    if (i == 0) {
      result.firstBitHighUs = width;
    }

    result.raw[i / 8] <<= 1;
    if (width > 40) {
      result.raw[i / 8] |= 1;
    }
  }

  interrupts();

  const uint8_t sum = (uint8_t)((result.raw[0] + result.raw[1] + result.raw[2] +
                                 result.raw[3]) & 0xFF);
  result.step = (sum == result.raw[4]) ? DhtStep::OK : DhtStep::BAD_CHECKSUM;
  return result;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.printf("DHT diagnostic on GPIO %u\n", DHT_PIN);
  Serial.println("Phase 1: square wave, 500 ms high / 500 ms low, 10 cycles.");
  Serial.println("Put the scope on the DHT data pin now. If you do not see a");
  Serial.println("1 Hz square wave, the probe is on the wrong pin.");

  pinMode(DHT_PIN, OUTPUT);
  for (uint8_t i = 0; i < IDENTITY_CYCLES; ++i) {
    digitalWrite(DHT_PIN, HIGH);
    delay(500);
    digitalWrite(DHT_PIN, LOW);
    delay(500);
  }
  pinMode(DHT_PIN, INPUT_PULLUP);

  Serial.println("Phase 1 done.");
  Serial.println("Phase 2: one DHT11 read every 2 s.");
  delay(1500);
}

void loop() {
  const DhtResult result = readDiag(DHT_PIN);

  Serial.printf("%-52s", stepName(result.step));

  if (result.step == DhtStep::NO_ANSWER) {
    Serial.println("  (check 3V3, GND, pull-up, and the pin)");
  } else {
    Serial.printf("  presence %lu/%lu us  bit0 %lu us  raw %02X %02X %02X %02X %02X",
                  (unsigned long)result.presenceLowUs,
                  (unsigned long)result.presenceHighUs,
                  (unsigned long)result.firstBitHighUs, result.raw[0],
                  result.raw[1], result.raw[2], result.raw[3], result.raw[4]);

    if (result.step == DhtStep::OK) {
      Serial.printf("  ->  T=%u C  RH=%u %%", result.raw[2] & 0x7F, result.raw[0]);
    } else if (result.step == DhtStep::BIT_LOW_TIMEOUT ||
               result.step == DhtStep::BIT_HIGH_TIMEOUT) {
      Serial.printf("  (stopped at bit %u of 40)", result.bitIndex);
    }
    Serial.println();
  }

  delay(2000);
}
