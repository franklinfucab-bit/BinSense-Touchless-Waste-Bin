/*
 * Filename: am2320_diag.ino
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 21, 2026
 * Description: I2C diagnostic for the ASAIR AM2320 temperature/humidity
 *              sensor, which is what the project actually has - it is NOT a
 *              DHT11. Scans the bus, then performs a full AM2320 read with the
 *              required wake-up sequence and reports exactly which step failed.
 *
 *              Replaces firmware/dht_diag for this part. Not in the flashing
 *              sequence; this is a bench tool.
 *
 * WIRING - this is the whole point, AM2320 is NOT pin-compatible with DHT11:
 *              AM2320 pin 1 = VDD  -> 3V3
 *              AM2320 pin 2 = SDA  -> GPIO 8   (shared with the LCD)
 *              AM2320 pin 3 = GND  -> GND
 *              AM2320 pin 4 = SCL  -> GPIO 9   (shared with the LCD)
 *              DHT11 by contrast is VDD / DATA / NC / GND, so pins 3 and 4
 *              swap roles. Wiring it as a DHT11 grounds SCL and leaves the
 *              real ground floating, which is silent failure.
 *              Verify against the label on your part before powering up.
 *
 * Libraries: none beyond Wire
 * Board: ESP32S3 Dev Module, Arduino-ESP32 3.3.10
 */

#include <Arduino.h>
#include <Wire.h>
#include "pins.h"

constexpr uint8_t AM2320_ADDR = 0x5C;
constexpr uint8_t AM2320_READ_FUNC = 0x03;
constexpr uint8_t AM2320_REG_START = 0x00;
constexpr uint8_t AM2320_REG_COUNT = 0x04;
constexpr uint32_t POLL_INTERVAL_MS = 2000;

// Modbus CRC16, low byte transmitted first.
uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void scanBus() {
  Serial.println();
  Serial.println("--- I2C scan ---");
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X", addr);
      if (addr == 0x27) {
        Serial.print("   <- the 1602 LCD");
      }
      if (addr == AM2320_ADDR) {
        Serial.print("   <- AM2320");
      }
      Serial.println();
      ++found;
    }
    delay(2);
  }
  Serial.printf("  %u device(s) responded\n", found);
  Serial.println("NOTE: the AM2320 sleeps and usually does NOT answer a scan.");
  Serial.println("Not seeing 0x5C here is normal and proves nothing. The read");
  Serial.println("test below is the real check.");
}

// Returns true on success. Reports the failing step otherwise.
bool readAm2320(float *temperatureC, float *humidityPct) {
  // Step 1: wake the sensor. It is asleep and will NOT acknowledge this, so a
  // NACK here is expected and must not be treated as an error.
  Wire.beginTransmission(AM2320_ADDR);
  Wire.endTransmission();
  delay(2);  // datasheet wants 0.8-3 ms before the real request

  // Step 2: ask for 4 registers starting at 0x00.
  Wire.beginTransmission(AM2320_ADDR);
  Wire.write(AM2320_READ_FUNC);
  Wire.write(AM2320_REG_START);
  Wire.write(AM2320_REG_COUNT);
  const uint8_t txError = Wire.endTransmission();
  if (txError != 0) {
    Serial.printf("  STEP2 FAIL: command not acknowledged (Wire error %u)\n",
                  txError);
    Serial.println("  -> sensor is not answering at 0x5C at all.");
    Serial.println("     Check VDD and GND first. Remember pin 3 is GND and");
    Serial.println("     pin 4 is SCL, which is the opposite of a DHT11.");
    return false;
  }
  delay(2);  // datasheet wants >1.5 ms before reading the answer

  // Step 3: read 8 bytes back.
  uint8_t buf[8] = {0};
  const uint8_t got = Wire.requestFrom((int)AM2320_ADDR, 8);
  if (got != 8) {
    Serial.printf("  STEP3 FAIL: wanted 8 bytes, got %u\n", got);
    return false;
  }
  for (uint8_t i = 0; i < 8; ++i) {
    buf[i] = Wire.read();
  }

  Serial.print("  raw:");
  for (uint8_t i = 0; i < 8; ++i) {
    Serial.printf(" %02X", buf[i]);
  }
  Serial.println();

  if (buf[0] != AM2320_READ_FUNC || buf[1] != AM2320_REG_COUNT) {
    Serial.printf("  STEP4 FAIL: bad header, expected 03 04 got %02X %02X\n",
                  buf[0], buf[1]);
    return false;
  }

  const uint16_t crcGot = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
  const uint16_t crcCalc = crc16(buf, 6);
  if (crcGot != crcCalc) {
    Serial.printf("  STEP5 FAIL: CRC got %04X, calculated %04X\n", crcGot,
                  crcCalc);
    Serial.println("  -> sensor IS talking, only the decode/wiring noise is off.");
    return false;
  }

  // 16-bit values, tenths of a unit. This is DHT22-style, not DHT11-style.
  const uint16_t rawHum = ((uint16_t)buf[2] << 8) | buf[3];
  uint16_t rawTemp = ((uint16_t)buf[4] << 8) | buf[5];
  float tempC;
  if (rawTemp & 0x8000) {
    rawTemp &= 0x7FFF;  // sign lives in the top bit, not two's complement
    tempC = -((float)rawTemp) / 10.0F;
  } else {
    tempC = ((float)rawTemp) / 10.0F;
  }

  *humidityPct = ((float)rawHum) / 10.0F;
  *temperatureC = tempC;
  return true;
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500);

  Serial.println();
  Serial.println("AM2320 I2C diagnostic (the part is NOT a DHT11)");
  Serial.printf("SDA=GPIO %u  SCL=GPIO %u  expected address 0x%02X\n",
                PIN_I2C_SDA, PIN_I2C_SCL, AM2320_ADDR);

  Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(100000);  // AM2320 tops out at 100 kHz, do not go faster

  scanBus();
  Serial.println();
  Serial.println("--- read attempts every 2 s ---");
}

void loop() {
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if ((now - lastMs) < POLL_INTERVAL_MS) {
    return;
  }
  lastMs = now;

  float tempC = 0.0F;
  float humPct = 0.0F;
  if (readAm2320(&tempC, &humPct)) {
    Serial.printf("  OK  T=%.1f C  RH=%.1f %%\n", tempC, humPct);
  }
}
