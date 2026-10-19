/*
 * Filename: HC_SR04_Distance_Test.ino
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 20, 2026
 * Description: Tests an HC-SR04 ultrasonic sensor by measuring distance,
 *              displaying it on the Serial Monitor, and sounding a passive
 *              buzzer after an object remains closer than 10 cm for 0.2 seconds.
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

constexpr uint8_t ECHO_PIN = 6;
constexpr uint8_t TRIGGER_PIN = 7;
constexpr uint8_t BUZZER_PIN = 2;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

constexpr uint32_t TRIGGER_PULSE_MICROSECONDS = 10;
constexpr uint32_t ECHO_TIMEOUT_MICROSECONDS = 30000;
constexpr uint32_t MEASUREMENT_INTERVAL_MILLISECONDS = 100;
constexpr uint8_t REQUIRED_NEAR_DISTANCE_SAMPLES = 3;

constexpr float ALARM_START_DISTANCE_CENTIMETERS = 10.0F;
constexpr float ALARM_STOP_DISTANCE_CENTIMETERS = 12.0F;

constexpr uint32_t BUZZER_FREQUENCY_HZ = 523;  // C5 is approximately 523.25 Hz.
constexpr uint8_t BUZZER_PWM_RESOLUTION_BITS = 8;

// The echo pulse covers the trip to the object and back, so divide by two.
constexpr float SOUND_SPEED_CENTIMETERS_PER_MICROSECOND = 0.0343F;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

bool buzzerReady = false;
bool buzzerActive = false;
uint8_t consecutiveNearDistanceSamples = 0;

// ---------------------------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------------------------

void setBuzzerActive(bool shouldBeActive);
void updateDistanceAlarm(bool measurementIsValid, float distanceCentimeters);

// ---------------------------------------------------------------------------
// Function Implementations
// ---------------------------------------------------------------------------

/**
 * Starts or stops the passive-buzzer tone.
 *
 * @param shouldBeActive True to play C5, or false to silence the buzzer.
 * @return Nothing.
 */
void setBuzzerActive(bool shouldBeActive) {
  if (shouldBeActive == buzzerActive) {
    return;
  }

  if (shouldBeActive) {
    Serial.println("SHOULD BUZZ NOW - requesting C5 tone on GPIO2");

    if (!buzzerReady) {
      Serial.println("BUZZER FAILED - LEDC was not attached to GPIO2");
      return;
    }

    const uint32_t actualFrequency = ledcWriteTone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
    if (actualFrequency == 0) {
      Serial.println("BUZZER FAILED - LEDC could not start the tone");
      return;
    }

    Serial.print("BUZZER ON - actual frequency: ");
    Serial.print(actualFrequency);
    Serial.println(" Hz");
  } else {
    ledcWriteTone(BUZZER_PIN, 0);
    Serial.println("DISTANCE RECOVERED - buzzer OFF");
  }

  buzzerActive = shouldBeActive;
}

/**
 * Applies three-sample confirmation and 10/12 cm distance hysteresis. With a
 * 100 ms measurement interval, the first-to-third sample span is about 200 ms.
 * An invalid measurement cannot start the alarm and does not count as recovery
 * from an alarm that is already active.
 *
 * @param measurementIsValid True when a returning echo was measured.
 * @param distanceCentimeters Latest valid distance, in centimeters.
 * @return Nothing.
 */
void updateDistanceAlarm(bool measurementIsValid, float distanceCentimeters) {
  if (!measurementIsValid) {
    if (!buzzerActive) {
      consecutiveNearDistanceSamples = 0;
    }
    return;
  }

  if (buzzerActive) {
    if (distanceCentimeters > ALARM_STOP_DISTANCE_CENTIMETERS) {
      setBuzzerActive(false);
    }
    return;
  }

  if (distanceCentimeters < ALARM_START_DISTANCE_CENTIMETERS) {
    if (consecutiveNearDistanceSamples < REQUIRED_NEAR_DISTANCE_SAMPLES) {
      ++consecutiveNearDistanceSamples;
    }

    Serial.print("NEAR SAMPLE ");
    Serial.print(consecutiveNearDistanceSamples);
    Serial.print("/");
    Serial.println(REQUIRED_NEAR_DISTANCE_SAMPLES);

    if (consecutiveNearDistanceSamples >= REQUIRED_NEAR_DISTANCE_SAMPLES) {
      consecutiveNearDistanceSamples = 0;
      setBuzzerActive(true);
    }
  } else {
    consecutiveNearDistanceSamples = 0;
  }
}

// ---------------------------------------------------------------------------
// Arduino Entry Points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  buzzerReady =
      ledcAttach(BUZZER_PIN, BUZZER_FREQUENCY_HZ, BUZZER_PWM_RESOLUTION_BITS);
  if (buzzerReady) {
    ledcWriteTone(BUZZER_PIN, 0);
  } else {
    Serial.println("Buzzer error: LEDC could not attach to GPIO2");
  }

  Serial.println("HC-SR04 distance test started.");
}

void loop() {
  // Generate the HC-SR04 trigger pulse.
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(TRIGGER_PULSE_MICROSECONDS);
  digitalWrite(TRIGGER_PIN, LOW);

  // Measure the duration of the returning echo pulse.
  const uint32_t echoDurationMicroseconds =
      pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_MICROSECONDS);

  if (echoDurationMicroseconds == 0) {
    Serial.println("No echo received");
    updateDistanceAlarm(false, 0.0F);
  } else {
    const float distanceCentimeters =
        echoDurationMicroseconds * SOUND_SPEED_CENTIMETERS_PER_MICROSECOND / 2.0F;

    Serial.print("Distance: ");
    Serial.print(distanceCentimeters, 1);
    Serial.println(" cm");

    updateDistanceAlarm(true, distanceCentimeters);
  }

  delay(MEASUREMENT_INTERVAL_MILLISECONDS);
}
