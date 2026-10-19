/*
 * Filename: am2320_read.h
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 21, 2026
 * Description: I2C reader for the ASAIR AM2320 (sometimes marked 2320).
 *              The part is not a DHT11: pinout is VDD / SDA / GND / SCL and
 *              the protocol is I2C at 0x5C with a wake-up before every read.
 *
 * Requires Wire already begun on SDA/SCL at <= 100 kHz.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "pins.h"

constexpr uint8_t AM2320_READ_FUNC = 0x03;
constexpr uint8_t AM2320_REG_START = 0x00;
constexpr uint8_t AM2320_REG_COUNT = 0x04;

inline uint16_t am2320Crc16(const uint8_t *data, uint8_t len) {
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

inline bool readAm2320(float *temperatureC, float *humidityPct) {
  // Sensor sleeps between reads. This wake-up is expected to NACK.
  Wire.beginTransmission(AM2320_I2C_ADDRESS);
  Wire.endTransmission();
  delay(2);

  Wire.beginTransmission(AM2320_I2C_ADDRESS);
  Wire.write(AM2320_READ_FUNC);
  Wire.write(AM2320_REG_START);
  Wire.write(AM2320_REG_COUNT);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  delay(2);

  uint8_t buf[8] = {0};
  if (Wire.requestFrom((int)AM2320_I2C_ADDRESS, 8) != 8) {
    return false;
  }
  for (uint8_t i = 0; i < 8; ++i) {
    buf[i] = Wire.read();
  }

  if (buf[0] != AM2320_READ_FUNC || buf[1] != AM2320_REG_COUNT) {
    return false;
  }

  const uint16_t crcGot = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
  if (crcGot != am2320Crc16(buf, 6)) {
    return false;
  }

  const uint16_t rawHum = ((uint16_t)buf[2] << 8) | buf[3];
  uint16_t rawTemp = ((uint16_t)buf[4] << 8) | buf[5];
  if (rawTemp & 0x8000) {
    *temperatureC = -((float)(rawTemp & 0x7FFF)) / 10.0F;
  } else {
    *temperatureC = ((float)rawTemp) / 10.0F;
  }
  *humidityPct = ((float)rawHum) / 10.0F;
  return true;
}
