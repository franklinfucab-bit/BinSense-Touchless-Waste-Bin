/*
 * Filename: pir_lid_integration_test.ino
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 20, 2026
 * Description: Integration test - HC-SR501 PIR on GPIO5 triggers the 28BYJ-48
 *              stepper (GPIO 15-18) to open the lid 90 deg on motion and close it
 *              when the PIR output returns LOW. GPIO4 LED mirrors PIR state.
 *
 * Combines: Shawn/HC_SR501_LED_Test.ino + stepper_90_test/stepper_90_test.ino
 */

#include <Arduino.h>
#include <Stepper.h>

constexpr uint8_t PIR_SIGNAL_PIN = 5;
constexpr uint8_t LED_PIN = 4;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

constexpr int STEPS_PER_REV = 2048;
constexpr int STEPS_90 = STEPS_PER_REV / 4;
constexpr uint8_t IN1 = 15;
constexpr uint8_t IN2 = 16;
constexpr uint8_t IN3 = 17;
constexpr uint8_t IN4 = 18;
constexpr uint8_t LID_SPEED = 8;

Stepper lid(STEPS_PER_REV, IN1, IN3, IN2, IN4);

int previousMotionState = LOW;
bool lidIsOpen = false;

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

  pinMode(PIR_SIGNAL_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lid.setSpeed(LID_SPEED);

  Serial.println("PIR + lid integration test started.");
  Serial.println("Allow the PIR sensor ~30 s to stabilize after power-up.");
  Serial.println("Wave in front of the sensor to open the lid.");
}

void loop() {
  const int motionState = digitalRead(PIR_SIGNAL_PIN);

  digitalWrite(LED_PIN, motionState == HIGH ? HIGH : LOW);

  if (motionState != previousMotionState) {
    if (motionState == HIGH) {
      Serial.println("Motion detected - LED ON");
      openLid();
    } else {
      Serial.println("No motion - LED OFF");
      closeLid();
    }

    previousMotionState = motionState;
  }
}