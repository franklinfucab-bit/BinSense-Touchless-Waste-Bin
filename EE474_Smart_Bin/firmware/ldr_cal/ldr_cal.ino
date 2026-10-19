/*
 * Filename: ldr_cal.ino
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 21, 2026
 * Description: LDR calibration tool for the lid-closure sensor on GPIO 1.
 *              Captures the ADC distribution with the lid closed and with the
 *              lid open, then recommends LDR_CLOSED_SETPOINT, LDR_HYSTERESIS,
 *              and LDR_CLOSED_MEANS_DARK for smart_garbage_bin.ino. Also
 *              replays the firmware's exact detection logic so the numbers can
 *              be trusted before they are pasted in.
 *
 *              Not part of the flashing sequence. This is a bench tool.
 *
 *              ADC is left at the core defaults (12-bit, 11 dB) because
 *              smart_garbage_bin.ino calls analogRead() without configuring it.
 *              If you ever configure the ADC in one, do it in both or the
 *              calibration will not transfer.
 *
 * Libraries: none beyond the bundled Stepper
 * Board: ESP32S3 Dev Module, Arduino-ESP32 3.3.10
 */

#include <Arduino.h>
#include <Stepper.h>
#include "pins.h"

// Must match smart_garbage_bin.ino so the replay below is faithful.
constexpr int LDR_SMA_WINDOW = 5;
constexpr uint8_t LDR_CONFIRM_SAMPLES = 5;

constexpr uint32_t SAMPLE_INTERVAL_MS = 10;   // 100 Hz, same as the control loop
constexpr uint16_t CAPTURE_SAMPLES = 200;     // ~2 s per capture
constexpr int MIN_HYSTERESIS = 30;            // never recommend less than this
constexpr int ADC_MAX = 4095;

Stepper lid(STEPS_PER_REV, STEPPER_LIBRARY_PIN1, STEPPER_LIBRARY_PIN2,
            STEPPER_LIBRARY_PIN3, STEPPER_LIBRARY_PIN4);

struct Stats {
  uint32_t n;
  double sum;
  double sumSq;
  int min;
  int max;
};

Stats closedStats;
Stats openStats;
bool streaming = false;

// Replay state for the 't' test mode.
int testSetpoint = 0;
int testHysteresis = 0;
bool testDarkMeansClosed = true;
bool testActive = false;

void statsReset(Stats *s) {
  s->n = 0;
  s->sum = 0.0;
  s->sumSq = 0.0;
  s->min = ADC_MAX + 1;
  s->max = -1;
}

void statsAdd(Stats *s, int sample) {
  ++s->n;
  s->sum += (double)sample;
  s->sumSq += (double)sample * (double)sample;
  if (sample < s->min) {
    s->min = sample;
  }
  if (sample > s->max) {
    s->max = sample;
  }
}

double statsMean(const Stats *s) {
  return (s->n == 0) ? 0.0 : (s->sum / (double)s->n);
}

double statsStdDev(const Stats *s) {
  if (s->n < 2) {
    return 0.0;
  }
  const double mean = statsMean(s);
  double var = (s->sumSq / (double)s->n) - (mean * mean);
  if (var < 0.0) {
    var = 0.0;  // floating point noise near zero spread
  }
  return sqrt(var);
}

void printStats(const char *label, const Stats *s) {
  if (s->n == 0) {
    Serial.printf("  %-6s not captured yet\n", label);
    return;
  }
  Serial.printf("  %-6s n=%lu  mean=%.1f  sd=%.1f  min=%d  max=%d\n", label,
                (unsigned long)s->n, statsMean(s), statsStdDev(s), s->min,
                s->max);
}

int readLdrFiltered(int sample) {
  static int window[LDR_SMA_WINDOW] = {0};
  static int count = 0;
  static int index = 0;

  window[index] = sample;
  index = (index + 1) % LDR_SMA_WINDOW;
  if (count < LDR_SMA_WINDOW) {
    ++count;
  }
  long sum = 0;
  for (int i = 0; i < count; ++i) {
    sum += window[i];
  }
  return (int)(sum / count);
}

// Same band logic as updateLdrClosed() in smart_garbage_bin.ino.
void replayDetector(int filtered) {
  static uint8_t confirm = 0;
  static bool closed = false;
  static bool lastClosed = false;

  const int closedBand = testSetpoint - testHysteresis;
  const int openBand = testSetpoint + testHysteresis;
  const bool inClosedBand =
      testDarkMeansClosed ? (filtered <= closedBand) : (filtered >= openBand);
  const bool inOpenBand =
      testDarkMeansClosed ? (filtered >= openBand) : (filtered <= closedBand);

  const char *zone = "HOLD";
  if (inClosedBand) {
    zone = "closed-band";
    if (confirm < LDR_CONFIRM_SAMPLES) {
      ++confirm;
    }
    if (confirm >= LDR_CONFIRM_SAMPLES) {
      closed = true;
    }
  } else if (inOpenBand) {
    zone = "open-band";
    confirm = 0;
    closed = false;
  }

  if (closed != lastClosed) {
    lastClosed = closed;
    Serial.printf("  -> declared %s (filtered=%d, %s)\n",
                  closed ? "CLOSED" : "OPEN", filtered, zone);
  }
}

void capture(Stats *target, const char *label) {
  statsReset(target);
  Serial.printf("Capturing %s for %u samples...\n", label, CAPTURE_SAMPLES);
  for (uint16_t i = 0; i < CAPTURE_SAMPLES; ++i) {
    statsAdd(target, analogRead(PIN_LDR));
    delay(SAMPLE_INTERVAL_MS);
  }
  printStats(label, target);
}

void recommend() {
  Serial.println();
  Serial.println("--- captured distributions ---");
  printStats("CLOSED", &closedStats);
  printStats("OPEN", &openStats);

  if (closedStats.n == 0 || openStats.n == 0) {
    Serial.println("Need both captures first: close the lid and press k, then");
    Serial.println("open the lid and press n.");
    return;
  }

  const double closedMean = statsMean(&closedStats);
  const double openMean = statsMean(&openStats);
  const double separation = fabs(closedMean - openMean);
  const double worstSd = max(statsStdDev(&closedStats), statsStdDev(&openStats));
  const bool darkMeansClosed = closedMean < openMean;

  Serial.println();
  Serial.printf("separation = %.1f counts, worst sd = %.1f\n", separation,
                worstSd);

  // Do the two states actually overlap? If so no threshold can separate them.
  const bool overlap = darkMeansClosed ? (closedStats.max >= openStats.min)
                                       : (openStats.max >= closedStats.min);
  if (overlap) {
    Serial.println("WARNING: the closed and open ranges OVERLAP.");
    Serial.println("No setpoint can separate them. Fix the optics first:");
    Serial.println("  - shade the LDR from room light");
    Serial.println("  - move it closer to the lid edge");
    Serial.println("  - check the divider resistor value");
  }
  if (separation < 4.0 * worstSd || separation < 100.0) {
    Serial.println("WARNING: weak separation. Homing will be unreliable.");
  }

  int setpoint = (int)lround((closedMean + openMean) / 2.0);
  int hysteresis = (int)lround(separation / 4.0);
  if (hysteresis < MIN_HYSTERESIS) {
    hysteresis = MIN_HYSTERESIS;
  }
  // Keep both bands reachable. The setpoint sits midway between the two means,
  // so each mean is separation/2 away from it; a hysteresis wider than that
  // pushes the band edge past the mean and the state can never update.
  int maxHysteresis = (int)(separation / 2.0) - 1;
  if (maxHysteresis < 1) {
    maxHysteresis = 1;
  }
  if (hysteresis > maxHysteresis) {
    hysteresis = maxHysteresis;
  }

  // The firmware is parameterised by the two measured endpoints plus a target
  // percentage along the span, so report exactly that. Polarity is implied by
  // which endpoint is larger and needs no separate flag.
  const int closeTargetPct =
      (int)lround(100.0 * fabs((double)setpoint - hysteresis - closedMean) /
                  separation);

  Serial.println();
  Serial.println("--- paste into smart_garbage_bin.ino ---");
  Serial.printf("constexpr int LDR_RAW_CLOSED = %d;\n", (int)lround(closedMean));
  Serial.printf("constexpr int LDR_RAW_OPEN = %d;\n", (int)lround(openMean));
  Serial.printf("constexpr int LDR_CLOSE_TARGET_PCT = %d;\n",
                closeTargetPct < 1 ? 1 : closeTargetPct);
  Serial.printf("constexpr int LDR_HYSTERESIS = %d;\n", hysteresis);
  Serial.println("----------------------------------------");
  Serial.printf("polarity: closed reads %s than open, so darker = %s\n",
                darkMeansClosed ? "LOWER" : "HIGHER",
                darkMeansClosed ? "closed" : "open");
  Serial.println("LDR_CLOSE_TARGET_PCT decides where the lid stops:");
  Serial.println("  lower  = drives the lid further shut");
  Serial.println("  higher = stops earlier, further from shut");
  Serial.printf("closed band: filtered %s %d\n", darkMeansClosed ? "<=" : ">=",
                darkMeansClosed ? setpoint - hysteresis : setpoint + hysteresis);
  Serial.printf("open band:   filtered %s %d\n", darkMeansClosed ? ">=" : "<=",
                darkMeansClosed ? setpoint + hysteresis : setpoint - hysteresis);
  Serial.println("Between the bands the state holds, which is intended.");

  testSetpoint = setpoint;
  testHysteresis = hysteresis;
  testDarkMeansClosed = darkMeansClosed;
  Serial.println();
  Serial.println("Press t to live-test these values, moving the lid by hand.");
}

void printHelp() {
  Serial.println();
  Serial.println("=== LDR calibration ===");
  Serial.println("  k  capture with lid CLOSED");
  Serial.println("  n  capture with lid OPEN");
  Serial.println("  r  recommend setpoint / hysteresis / polarity");
  Serial.println("  t  toggle live test of the recommended values");
  Serial.println("  s  toggle raw sample streaming");
  Serial.println("  o  step the lid 90 deg open");
  Serial.println("  c  step the lid 90 deg closed");
  Serial.println("  x  clear both captures");
  Serial.println("  ?  this help");
  Serial.println();
  Serial.println("Typical run: close lid -> k, open lid -> n, then r, then t.");
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500);

  pinMode(PIN_LDR, INPUT);
  lid.setSpeed(LID_SPEED_RPM);

  statsReset(&closedStats);
  statsReset(&openStats);

  Serial.println();
  Serial.println("LDR calibration tool. GPIO 1, 12-bit ADC, core defaults.");
  Serial.printf("Current LDR reading: %d\n", analogRead(PIN_LDR));
  printHelp();
}

void loop() {
  static uint32_t lastSampleMs = 0;

  if (Serial.available()) {
    const int key = Serial.read();
    switch (key) {
      case 'k':
        capture(&closedStats, "CLOSED");
        break;
      case 'n':
        capture(&openStats, "OPEN");
        break;
      case 'r':
        recommend();
        break;
      case 't':
        if (testSetpoint == 0) {
          Serial.println("Run r first so there is something to test.");
        } else {
          testActive = !testActive;
          Serial.printf("Live test %s (setpoint=%d hyst=%d dark=%s)\n",
                        testActive ? "ON" : "OFF", testSetpoint, testHysteresis,
                        testDarkMeansClosed ? "true" : "false");
        }
        break;
      case 's':
        streaming = !streaming;
        Serial.printf("Streaming %s\n", streaming ? "ON" : "OFF");
        break;
      case 'o':
        Serial.println("Opening 90 deg...");
        lid.step(STEPS_90);
        Serial.println("done");
        break;
      case 'c':
        Serial.println("Closing 90 deg...");
        lid.step(-STEPS_90);
        Serial.println("done");
        break;
      case 'x':
        statsReset(&closedStats);
        statsReset(&openStats);
        Serial.println("Captures cleared.");
        break;
      case '?':
        printHelp();
        break;
      default:
        break;  // ignore newlines and stray characters
    }
  }

  const uint32_t now = millis();
  if ((now - lastSampleMs) < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs = now;

  const int raw = analogRead(PIN_LDR);
  const int filtered = readLdrFiltered(raw);

  if (testActive) {
    replayDetector(filtered);
  }

  if (streaming) {
    static uint8_t decimate = 0;
    if (++decimate >= 10) {  // print at 10 Hz, sample at 100 Hz
      decimate = 0;
      Serial.printf("raw=%4d filtered=%4d\n", raw, filtered);
    }
  }
}
