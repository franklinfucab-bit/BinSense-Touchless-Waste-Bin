/*
 * Filename: stage3_sensors_ui.ino
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 21, 2026
 * Description: Stages 3-5 — PIR lid plus HC-SR04 fill hysteresis/buzzer,
 *              ASAIR AM2320 on I2C (SDA 8, SCL 9, 0x5C) plus 1602 LCD 0x27.
 *              Blocking stepper moves are acceptable for this PASS.
 *
 * Libraries: LiquidCrystal_I2C (AM2320 uses local am2320_read.h)
 * Board: ESP32S3 Dev Module, Arduino-ESP32 3.3.10
 */

#include <Arduino.h>
#include <Stepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "pins.h"
#include "am2320_read.h"

constexpr uint32_t TRIGGER_PULSE_US = 10;
constexpr uint32_t ECHO_TIMEOUT_US = 30000;
constexpr uint32_t DISTANCE_INTERVAL_MS = 100;
constexpr uint32_t LCD_REINIT_MS = 5000;
constexpr uint32_t DHT_INTERVAL_MS = 2000;
constexpr uint8_t REQUIRED_NEAR_SAMPLES = 3;
constexpr float ALARM_START_CM = 10.0F;
constexpr float ALARM_STOP_CM = 12.0F;
constexpr float EMPTY_DISTANCE_CM = 30.0F;
constexpr float SOUND_CM_PER_US = 0.0343F;
constexpr uint32_t BUZZER_FREQ_HZ = 523;
constexpr uint8_t BUZZER_PWM_BITS = 8;

Stepper lid(STEPS_PER_REV, STEPPER_LIBRARY_PIN1, STEPPER_LIBRARY_PIN2,
            STEPPER_LIBRARY_PIN3, STEPPER_LIBRARY_PIN4);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

bool lidIsOpen = false;
uint32_t lastMotionMs = 0;
bool binFull = false;
bool buzzerReady = false;
bool buzzerActive = false;
uint8_t nearSamples = 0;

float lastDistanceCm = 0.0F;
bool lastDistanceValid = false;
float lastTempC = NAN;
float lastHumPct = NAN;
bool lastDhtValid = false;

uint32_t lastDistanceMs = 0;
uint32_t lastDhtMs = 0;

const char *uiState() {
  if (binFull) {
    return "FULL";
  }
  if (lidIsOpen) {
    return "OPEN";
  }
  return "CLOSED";
}

float fillPercent(float distanceCm) {
  const float span = EMPTY_DISTANCE_CM - ALARM_START_CM;
  if (span <= 0.0F) {
    return 0.0F;
  }
  float pct = 100.0F * (EMPTY_DISTANCE_CM - distanceCm) / span;
  if (pct < 0.0F) {
    pct = 0.0F;
  }
  if (pct > 100.0F) {
    pct = 100.0F;
  }
  return pct;
}

void setBuzzerActive(bool shouldBeActive) {
  if (shouldBeActive == buzzerActive) {
    return;
  }
  if (!buzzerReady) {
    return;
  }
  if (shouldBeActive) {
    ledcWriteTone(PIN_BUZZER, BUZZER_FREQ_HZ);
  } else {
    ledcWriteTone(PIN_BUZZER, 0);
  }
  buzzerActive = shouldBeActive;
}

void updateLcd() {
  char line0[17];
  char line1[17];
  const float fill = lastDistanceValid ? fillPercent(lastDistanceCm) : 0.0F;

  snprintf(line0, sizeof(line0), "%-7s %3.0f%%    ", uiState(), fill);
#if AM2320_ENABLED
  if (lastDhtValid) {
    snprintf(line1, sizeof(line1), "T=%4.1fC H=%02.0f    ", lastTempC,
             lastHumPct);
  } else {
    snprintf(line1, sizeof(line1), "T=--.-C H=--    ");
  }
#else
  snprintf(line1, sizeof(line1), "AM2320 off      ");
#endif

  // Same 4-bit phase problem as the full firmware: one lost nibble garbles the
  // display permanently, so re-seat it periodically instead of staying broken.
  static uint32_t lastLcdInitMs = 0;
  const uint32_t lcdNowMs = millis();
  if ((lcdNowMs - lastLcdInitMs) >= LCD_REINIT_MS) {
    lastLcdInitMs = lcdNowMs;
    lcd.init();
    lcd.backlight();
  }

  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

void openLid() {
  if (lidIsOpen) {
    return;
  }
  if (binFull) {
    Serial.println("FULL - PIR open allowed so the bin can be emptied");
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

void updateFillAlarm(bool measurementIsValid, float distanceCm) {
  if (!measurementIsValid) {
    if (!binFull) {
      nearSamples = 0;
    }
    return;
  }

  if (binFull) {
    if (distanceCm > ALARM_STOP_CM) {
      binFull = false;
      nearSamples = 0;
      setBuzzerActive(false);
      Serial.println("Fill recovered");
    }
    return;
  }

  if (distanceCm < ALARM_START_CM) {
    if (nearSamples < REQUIRED_NEAR_SAMPLES) {
      ++nearSamples;
    }
    if (nearSamples >= REQUIRED_NEAR_SAMPLES) {
      binFull = true;
      setBuzzerActive(true);
      Serial.println("FULL - buzzer ON");
    }
  } else {
    nearSamples = 0;
  }
}

void measureDistance() {
  digitalWrite(PIN_SR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SR04_TRIG, HIGH);
  delayMicroseconds(TRIGGER_PULSE_US);
  digitalWrite(PIN_SR04_TRIG, LOW);

  const uint32_t echoUs = pulseIn(PIN_SR04_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (echoUs == 0) {
    lastDistanceValid = false;
    updateFillAlarm(false, 0.0F);
    return;
  }

  lastDistanceCm = echoUs * SOUND_CM_PER_US / 2.0F;
  lastDistanceValid = true;
  updateFillAlarm(true, lastDistanceCm);
}

#if AM2320_ENABLED
void sampleAm2320() {
  float t = 0.0F;
  float h = 0.0F;
  if (!readAm2320(&t, &h)) {
    Serial.println("AM2320 bad read (keep last LCD value if any)");
    return;
  }

  lastHumPct = h;
  lastTempC = t;
  lastDhtValid = true;
  Serial.print("OK  T=");
  Serial.print(t, 1);
  Serial.print(" C  RH=");
  Serial.print(h, 1);
  Serial.println(" %");
}
#endif

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_SR04_TRIG, OUTPUT);
  pinMode(PIN_SR04_ECHO, INPUT);
  digitalWrite(PIN_SR04_TRIG, LOW);

  lid.setSpeed(LID_SPEED_RPM);
  delay(2000);

  Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  buzzerReady = ledcAttach(PIN_BUZZER, BUZZER_FREQ_HZ, BUZZER_PWM_BITS);
  if (buzzerReady) {
    ledcWriteTone(PIN_BUZZER, 0);
  }

  Serial.println("Stages 3-5: PIR lid + fill/buzzer + AM2320 + LCD");
  Serial.println("AM2320 on I2C: SDA=GPIO8 SCL=GPIO9 addr=0x5C (not DHT11).");
  Serial.println("Allow PIR ~30 s to stabilize. Wave to open the lid.");
  updateLcd();
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

#if AM2320_ENABLED
  if ((now - lastDhtMs) >= DHT_INTERVAL_MS) {
    lastDhtMs = now;
    sampleAm2320();
  }
#endif

  if (!lidIsOpen && (now - lastDistanceMs) >= DISTANCE_INTERVAL_MS) {
    lastDistanceMs = now;
    measureDistance();
  }

  static uint32_t lastLcdMs = 0;
  if ((now - lastLcdMs) >= 250) {
    lastLcdMs = now;
    updateLcd();
  }
}
