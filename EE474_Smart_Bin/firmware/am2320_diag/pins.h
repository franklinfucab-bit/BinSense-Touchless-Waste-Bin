/*
 * Filename: pins.h
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 21, 2026
 * Description: Unified ESP32-S3 GPIO map for the Smart Garbage Bin.
 *              Temperature/humidity is an ASAIR AM2320 on I2C (SDA 8, SCL 9),
 *              sharing the bus with the 1602 LCD. GPIO 4 and GPIO 10 are unused.
 */

#pragma once

#include <stdint.h>

constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// AM2320 I2C temperature/humidity (the part is not a DHT11).
#define AM2320_ENABLED 1

// --- Sensors ---
constexpr uint8_t PIN_LDR = 1;        // analog lid-closure feedback
constexpr uint8_t PIN_PIR = 5;        // HC-SR501 digital output
constexpr uint32_t PIR_HOLD_OPEN_MS = 10000;  // close only after 10 s with no motion
constexpr uint8_t PIN_SR04_ECHO = 6;  // HC-SR04 echo (3.3 V or resistor divider)
constexpr uint8_t PIN_SR04_TRIG = 7;  // HC-SR04 trigger

// --- I2C: 1602 LCD + ASAIR AM2320 ---
constexpr uint8_t PIN_I2C_SDA = 8;
constexpr uint8_t PIN_I2C_SCL = 9;
constexpr uint8_t LCD_I2C_ADDRESS = 0x27;
constexpr uint8_t LCD_COLUMNS = 16;
constexpr uint8_t LCD_ROWS = 2;
constexpr uint8_t AM2320_I2C_ADDRESS = 0x5C;
// AM2320's absolute maximum is 100 kHz. Running at 50 kHz on purpose: the bus
// is shared, unshielded on a breadboard, and sits next to the stepper wiring.
// Slower edges tolerate the motor's switching noise much better, and the
// sensor is only read every 2 s so there is nothing to lose.
constexpr uint32_t I2C_CLOCK_HZ = 50000;

// --- Outputs ---
constexpr uint8_t PIN_BUZZER = 2;        // passive buzzer via LEDC
constexpr uint8_t PIN_STEPPER_IN1 = 15;  // ULN2003 IN1
constexpr uint8_t PIN_STEPPER_IN2 = 16;  // ULN2003 IN2
constexpr uint8_t PIN_STEPPER_IN3 = 17;  // ULN2003 IN3
constexpr uint8_t PIN_STEPPER_IN4 = 18;  // ULN2003 IN4

// --- Stepper (same coil order as stepper_90_test.ino) ---
constexpr int STEPS_PER_REV = 2048;
constexpr int STEPS_90 = STEPS_PER_REV / 4;  // 512
constexpr uint8_t LID_SPEED_RPM = 8;

// Arduino Stepper(steps, IN1, IN3, IN2, IN4) pin order.
constexpr uint8_t STEPPER_LIBRARY_PIN1 = PIN_STEPPER_IN1;
constexpr uint8_t STEPPER_LIBRARY_PIN2 = PIN_STEPPER_IN3;
constexpr uint8_t STEPPER_LIBRARY_PIN3 = PIN_STEPPER_IN2;
constexpr uint8_t STEPPER_LIBRARY_PIN4 = PIN_STEPPER_IN4;
