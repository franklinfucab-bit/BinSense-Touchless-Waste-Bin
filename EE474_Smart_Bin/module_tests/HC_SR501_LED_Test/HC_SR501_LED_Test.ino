/*
 * Filename: HC_SR501_LED_Test.ino
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 20, 2026
 * Description: Tests an HC-SR501 PIR motion sensor on GPIO5 by lighting an
 *              external LED on GPIO4 whenever the sensor output is HIGH.
 *
 * OpenAI Codex was used to help clarify the test design and create this
 * component-test sketch.
 */

// ---------------------------------------------------------------------------
// Includes
// ---------------------------------------------------------------------------

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

constexpr uint8_t PIR_SIGNAL_PIN = 5;
constexpr uint8_t LED_PIN = 4;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

int previousMotionState = LOW;

// ---------------------------------------------------------------------------
// Arduino Entry Points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(PIR_SIGNAL_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("HC-SR501 LED test started.");
  Serial.println("Allow the PIR sensor time to stabilize after power-up.");
}

void loop() {
  const int motionState = digitalRead(PIR_SIGNAL_PIN);

  // The HC-SR501 output remains HIGH for its configured hold time after motion.
  digitalWrite(LED_PIN, motionState == HIGH ? HIGH : LOW);

  // Print only when the output changes to keep the Serial Monitor readable.
  if (motionState != previousMotionState) {
    if (motionState == HIGH) {
      Serial.println("Motion detected - LED ON");
    } else {
      Serial.println("No motion - LED OFF");
    }

    previousMotionState = motionState;
  }
}
