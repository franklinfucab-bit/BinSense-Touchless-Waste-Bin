/*
 * Filename: stage2_pir_lid.ino
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 21, 2026
 * Description: Stage 2 pairwise PASS — HC-SR501 on GPIO5 opens the lid 90 deg
 *              on a rising edge and closes it 10 s after the last PIR HIGH.
 *              GPIO4 is reserved for DHT11 and is not used as a PIR LED.
 *
 * Board: ESP32S3 Dev Module, Arduino-ESP32 3.3.10
 */

#include <Arduino.h>
#include <Stepper.h>
#include "pins.h"

Stepper lid(STEPS_PER_REV, STEPPER_LIBRARY_PIN1, STEPPER_LIBRARY_PIN2,
            STEPPER_LIBRARY_PIN3, STEPPER_LIBRARY_PIN4);

bool lidIsOpen = false;
uint32_t lastMotionMs = 0;

void openLid() {
  if (lidIsOpen) {
    return;
  }
  Serial.println("Opening lid (90 deg)...");
  lid.step(STEPS_90);
  lidIsOpen = true;
  Serial.println("Lid open.");
}

void closeLid() {
  if (!lidIsOpen) {
    return;
  }
  Serial.println("Closing lid (90 deg)...");
  lid.step(-STEPS_90);
  lidIsOpen = false;
  Serial.println("Lid closed.");
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(PIN_PIR, INPUT);
  lid.setSpeed(LID_SPEED_RPM);

  Serial.println("Stage 2: PIR + lid (GPIO4 LED removed).");
  Serial.println("Allow the PIR sensor ~30 s to stabilize after power-up.");
  Serial.println("Wave in front of the sensor to open the lid.");
}

void loop() {
  const uint32_t now = millis();
  const int motionState = digitalRead(PIN_PIR);

  if (motionState == HIGH) {
    lastMotionMs = now;
    if (!lidIsOpen) {
      Serial.println("Motion detected - opening now");
      openLid();
    }
  } else if (lidIsOpen && (now - lastMotionMs) >= PIR_HOLD_OPEN_MS) {
    Serial.println("No motion for 10 s - closing");
    closeLid();
  }
}
