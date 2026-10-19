/*
 * Filename: smart_garbage_bin.ino
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 21, 2026
 * Description: Baseline Smart Garbage Bin firmware. Dual-core FreeRTOS with a
 *              100 Hz hardware-timer lid task, 50 Hz safety state machine,
 *              three queues plus a timer semaphore, LDR closed-loop homing,
 *              ASAIR AM2320 / HC-SR04 / PIR / LCD / buzzer, and LittleFS logging.
 *              BLE and IR are not enabled (optional after core V&V).
 *
 * Libraries: LiquidCrystal_I2C (AM2320 uses local am2320_read.h)
 * Board: ESP32S3 Dev Module, Arduino-ESP32 3.3.10
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <LittleFS.h>
#include <Preferences.h>
#include "pins.h"
#include "types.h"
#include "am2320_read.h"

// ---------------------------------------------------------------------------
// Timing and calibration
// ---------------------------------------------------------------------------

constexpr uint32_t CONTROL_HZ = 100;
constexpr uint32_t CONTROL_PERIOD_US = 1000000 / CONTROL_HZ;
constexpr uint32_t SAFETY_PERIOD_MS = 20;

// Second hardware timer, deliberately 64 Hz. 1000000/64 divides exactly into
// 15625 us. A FreeRTOS task delay cannot hit 32/64/128 Hz at all, because the
// tick is 1 ms and those periods are 31.25 / 15.625 / 7.8125 ms, so this rate
// has to come from a hardware timer.
constexpr uint32_t INPUT_HZ = 64;
constexpr uint32_t INPUT_PERIOD_US = 1000000 / INPUT_HZ;
// Consecutive agreeing samples before a PIR change is accepted.
// 3 samples at 64 Hz is about 47 ms, short enough to still feel instant.
constexpr uint8_t PIR_CONFIRM_SAMPLES = 3;
constexpr uint32_t INPUT_BUDGET_US = 2000;
constexpr uint32_t SENSOR_PIR_MS = 20;
constexpr uint32_t SENSOR_DISTANCE_MS = 100;
constexpr uint32_t SENSOR_DHT_MS = 2000;
constexpr uint32_t UI_PERIOD_MS = 100;
// The 1602 is an HD44780 behind a PCF8574 in 4-bit mode, so a single corrupted
// nibble leaves it permanently out of phase and every later character is
// garbage. Re-initialising on a slow cycle guarantees it heals. See section 3
// of DEVLOG.md.
constexpr uint32_t LCD_REINIT_MS = 5000;
constexpr uint32_t LOG_SNAPSHOT_MS = 10000;
constexpr uint32_t HOLD_AFTER_PIR_LOW_MS = PIR_HOLD_OPEN_MS;
constexpr uint32_t HOME_TIMEOUT_MS = 15000;
constexpr uint32_t CLOSE_TIMEOUT_MS = 10000;
constexpr uint32_t DHT_STALE_MS = 10000;
constexpr uint32_t DEADLINE_BUDGET_US = 8000;
constexpr uint8_t DEADLINE_FAULT_STREAK = 10;
constexpr int STEPS_PER_TICK = 3;
constexpr int DIR_OPEN = 1;
constexpr int DIR_CLOSE = -1;
constexpr int HOME_MAX_STEPS = STEPS_90 * 2;
constexpr int CLOSE_EXTRA_STEPS = 64;

constexpr int LDR_SMA_WINDOW = 5;

// Calibrated endpoints, measured on hardware August 21 with firmware/ldr_cal,
// 200 samples per state: closed mean 1053.7 (sd 7.8, range 1041-1068), open
// mean 3276.7 (sd 1.4, range 3273-3283). See DEVLOG.md section 7b.
constexpr int LDR_RAW_CLOSED = 1054;  // lid shut, sensor shaded
constexpr int LDR_RAW_OPEN = 3277;    // lid open, sensor lit
constexpr int LDR_SPAN = LDR_RAW_OPEN - LDR_RAW_CLOSED;

// Where along that span the lid is declared shut, as a percentage of the
// closed-to-open distance. This is the closed-loop target: the control task
// stops stepping once the filtered reading crosses it.
//   0%   = demand the fully-shut reading exactly, no margin, will never latch
//   100% = accept the fully-open reading, would latch immediately
// LOWER this to drive the lid further closed, RAISE it to stop earlier.
constexpr int LDR_CLOSE_TARGET_PCT = 45;

constexpr int LDR_CLOSED_THRESHOLD =
    LDR_RAW_CLOSED + (LDR_SPAN * LDR_CLOSE_TARGET_PCT) / 100;
constexpr int LDR_HYSTERESIS = 556;
constexpr int LDR_OPEN_THRESHOLD =
    LDR_CLOSED_THRESHOLD + ((LDR_SPAN >= 0) ? LDR_HYSTERESIS : -LDR_HYSTERESIS);

// Polarity is derived from the endpoints instead of being a separate flag that
// could contradict them. "Darker" means a lower ADC count in this wiring.
constexpr bool LDR_CLOSED_IS_DARKER = (LDR_RAW_CLOSED < LDR_RAW_OPEN);

constexpr uint8_t LDR_CONFIRM_SAMPLES = 5;

// Both bands must stay inside the measured range or the state can never update.
static_assert(LDR_CLOSE_TARGET_PCT > 0 && LDR_CLOSE_TARGET_PCT < 100,
              "LDR_CLOSE_TARGET_PCT must be a percentage strictly inside 0..100");
static_assert(LDR_CLOSED_IS_DARKER ? (LDR_OPEN_THRESHOLD < LDR_RAW_OPEN)
                                   : (LDR_OPEN_THRESHOLD > LDR_RAW_OPEN),
              "open band unreachable: lower LDR_CLOSE_TARGET_PCT or LDR_HYSTERESIS");
static_assert(LDR_CLOSED_IS_DARKER ? (LDR_CLOSED_THRESHOLD > LDR_RAW_CLOSED)
                                   : (LDR_CLOSED_THRESHOLD < LDR_RAW_CLOSED),
              "closed band unreachable: raise LDR_CLOSE_TARGET_PCT");

constexpr uint32_t TRIGGER_PULSE_US = 10;
constexpr uint32_t ECHO_TIMEOUT_US = 30000;
constexpr uint8_t REQUIRED_NEAR_SAMPLES = 3;
constexpr float ALARM_START_CM = 10.0F;
constexpr float ALARM_STOP_CM = 12.0F;
constexpr float EMPTY_DISTANCE_CM = 30.0F;
constexpr float SOUND_CM_PER_US = 0.0343F;
constexpr uint32_t BUZZER_FREQ_HZ = 523;
constexpr uint8_t BUZZER_PWM_BITS = 8;

constexpr uint32_t QUEUE_WAIT_MS = 10;
constexpr uint32_t STACK_LID = 4096;
constexpr uint32_t STACK_SAFETY = 4096;
constexpr uint32_t STACK_SENSOR = 4096;
constexpr uint32_t STACK_UI = 6144;
constexpr uint32_t STACK_DHT = 4096;
constexpr uint32_t STACK_INPUT = 3072;
constexpr uint32_t STACK_LOAD = 2048;
constexpr uint32_t LOAD_SPIN_US = 8000;  // spin 8 ms, then yield 1 tick

// ---------------------------------------------------------------------------
// RTOS objects
// ---------------------------------------------------------------------------

QueueHandle_t sensorDataQueue = nullptr;
QueueHandle_t controlCommandQueue = nullptr;
QueueHandle_t systemEventQueue = nullptr;
SemaphoreHandle_t controlTimerSemaphore = nullptr;
SemaphoreHandle_t inputTimerSemaphore = nullptr;
SemaphoreHandle_t sharedMutex = nullptr;
SemaphoreHandle_t i2cMutex = nullptr;
hw_timer_t *controlTimer = nullptr;  // core 1, 100 Hz, attached in setup()
hw_timer_t *inputTimer = nullptr;    // core 0, 64 Hz, attached in inputTask()

// ---------------------------------------------------------------------------
// Drivers
// ---------------------------------------------------------------------------

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);
Preferences prefs;

// ---------------------------------------------------------------------------
// Timing evidence
//
// Deliberately lock-free. Each instance has exactly one writer task and readers
// only ever read, and aligned 32-bit accesses are atomic on the ESP32-S3.
// Taking a mutex inside the measured window would add contention to the very
// interval being measured, so the numbers would describe the instrumentation
// rather than the system.
//
// `periodUs` is measured start-of-work to start-of-work, so it captures timer
// jitter and release latency together - that is what "jitter" means here.
// `execUs` is the work itself, i.e. worst-case execution time.
// ---------------------------------------------------------------------------

struct TimingStats {
  volatile uint32_t samples;
  volatile uint32_t periodMinUs;
  volatile uint32_t periodMaxUs;
  volatile uint32_t jitterMaxUs;
  volatile uint32_t execMaxUs;
  volatile uint32_t execAvgUs;
  volatile uint32_t overrunCount;
};

TimingStats gCtrlTiming = {};
TimingStats gInputTiming = {};
volatile bool gLoadEnabled = false;

void timingRecord(TimingStats *s, uint32_t periodUs, uint32_t nominalUs,
                  uint32_t execUs, uint32_t budgetUs) {
  if (s->samples == 0) {
    s->periodMinUs = periodUs;
    s->periodMaxUs = periodUs;
    s->execAvgUs = execUs;
  } else {
    if (periodUs < s->periodMinUs) {
      s->periodMinUs = periodUs;
    }
    if (periodUs > s->periodMaxUs) {
      s->periodMaxUs = periodUs;
    }
    const int32_t diff = (int32_t)execUs - (int32_t)s->execAvgUs;
    s->execAvgUs = (uint32_t)((int32_t)s->execAvgUs + diff / 16);
  }

  const uint32_t jitterUs = (periodUs > nominalUs) ? (periodUs - nominalUs)
                                                  : (nominalUs - periodUs);
  if (jitterUs > s->jitterMaxUs) {
    s->jitterMaxUs = jitterUs;
  }
  if (execUs > s->execMaxUs) {
    s->execMaxUs = execUs;
  }
  if (execUs > budgetUs) {
    ++s->overrunCount;
  }
  ++s->samples;
}

void timingClear(TimingStats *s) {
  s->samples = 0;
  s->periodMinUs = 0;
  s->periodMaxUs = 0;
  s->jitterMaxUs = 0;
  s->execMaxUs = 0;
  s->execAvgUs = 0;
  s->overrunCount = 0;
}

// ---------------------------------------------------------------------------
// Shared state (Core 1 mutex)
// ---------------------------------------------------------------------------

SystemState gState = SystemState::CLOSED;
FaultCode gFault = FaultCode::NONE;
int gPositionSteps = 0;
int gStepPhase = 0;
int gLdrRaw = 0;
int gLdrFiltered = 0;
bool gLdrClosed = false;
bool gPirStable = false;  // debounced PIR, owned by inputTask
bool gMotionComplete = false;
bool gLimitHit = false;
uint32_t gStateEnteredMs = 0;
uint32_t gOpeningCount = 0;
uint32_t gTimeoutCount = 0;
uint32_t gDeadlineMissCount = 0;
uint8_t gDeadlineStreak = 0;
SensorData gLatestSensors = {};
float gDhtTempC = 0.0F;
float gDhtHumPct = 0.0F;
bool gDhtOk = false;

// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------

const char *stateName(SystemState state);
void publishEvent(SystemEventType type, SystemState state, FaultCode fault);
void stepperWritePhase(int phase);
void stepperIdle();
void stepperAdvance(int direction);
int updateLdrClosed(int sample);
void enterState(SystemState next, FaultCode fault);
void IRAM_ATTR onControlTimer();
void IRAM_ATTR onInputTimer();
void lidControlTask(void *arg);
void safetyTask(void *arg);
void sensorTask(void *arg);
void inputTask(void *arg);
void loadTask(void *arg);
void printStats();
void printHelp();
#if AM2320_ENABLED
void am2320Task(void *arg);
#endif
void uiTask(void *arg);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

const char *stateName(SystemState state) {
  switch (state) {
    case SystemState::HOMING:
      return "HOMING";
    case SystemState::CLOSED:
      return "CLOSED";
    case SystemState::OPENING:
      return "OPENING";
    case SystemState::HOLD_OPEN:
      return "HOLD";
    case SystemState::CLOSING:
      return "CLOSING";
    case SystemState::FULL:
      return "FULL";
    case SystemState::FAULT_SAFE:
      return "FAULT";
    default:
      return "?";
  }
}

void publishEvent(SystemEventType type, SystemState state, FaultCode fault) {
  SystemEvent event = {};
  event.type = type;
  event.state = state;
  event.faultCode = fault;
  event.timestampMs = millis();
  xQueueSend(systemEventQueue, &event, 0);
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

void persistCounters() {
  prefs.putUInt("opens", gOpeningCount);
  prefs.putUInt("timeouts", gTimeoutCount);
  prefs.putUInt("deadlines", gDeadlineMissCount);
}

void enterState(SystemState next, FaultCode fault) {
  if (gState == next && gFault == fault) {
    return;
  }
  gState = next;
  gFault = fault;
  gStateEnteredMs = millis();
  gMotionComplete = false;
  gLimitHit = false;
  if (next == SystemState::FAULT_SAFE) {
    stepperIdle();
  }
  publishEvent(SystemEventType::STATE_CHANGE, next, fault);
  if (next == SystemState::FAULT_SAFE) {
    publishEvent(SystemEventType::FAULT, next, fault);
  }
}

// Matches Arduino Stepper 4-wire stepMotor() with pins (IN1, IN3, IN2, IN4).
void stepperWritePhase(int phase) {
  phase = ((phase % 4) + 4) % 4;
  switch (phase) {
    case 0:
      digitalWrite(PIN_STEPPER_IN1, HIGH);
      digitalWrite(PIN_STEPPER_IN3, LOW);
      digitalWrite(PIN_STEPPER_IN2, HIGH);
      digitalWrite(PIN_STEPPER_IN4, LOW);
      break;
    case 1:
      digitalWrite(PIN_STEPPER_IN1, LOW);
      digitalWrite(PIN_STEPPER_IN3, HIGH);
      digitalWrite(PIN_STEPPER_IN2, HIGH);
      digitalWrite(PIN_STEPPER_IN4, LOW);
      break;
    case 2:
      digitalWrite(PIN_STEPPER_IN1, LOW);
      digitalWrite(PIN_STEPPER_IN3, HIGH);
      digitalWrite(PIN_STEPPER_IN2, LOW);
      digitalWrite(PIN_STEPPER_IN4, HIGH);
      break;
    default:
      digitalWrite(PIN_STEPPER_IN1, HIGH);
      digitalWrite(PIN_STEPPER_IN3, LOW);
      digitalWrite(PIN_STEPPER_IN2, LOW);
      digitalWrite(PIN_STEPPER_IN4, HIGH);
      break;
  }
}

void stepperIdle() {
  digitalWrite(PIN_STEPPER_IN1, LOW);
  digitalWrite(PIN_STEPPER_IN2, LOW);
  digitalWrite(PIN_STEPPER_IN3, LOW);
  digitalWrite(PIN_STEPPER_IN4, LOW);
}

void stepperAdvance(int direction) {
  gStepPhase += direction;
  stepperWritePhase(gStepPhase);
  gPositionSteps += direction;
}

int updateLdrClosed(int sample) {
  static int window[LDR_SMA_WINDOW] = {0};
  static int count = 0;
  static int index = 0;
  static uint8_t confirm = 0;

  window[index] = sample;
  index = (index + 1) % LDR_SMA_WINDOW;
  if (count < LDR_SMA_WINDOW) {
    ++count;
  }

  long sum = 0;
  for (int i = 0; i < count; ++i) {
    sum += window[i];
  }
  const int filtered = (int)(sum / count);
  gLdrRaw = sample;
  gLdrFiltered = filtered;

  bool inClosedBand = LDR_CLOSED_IS_DARKER
                          ? (filtered <= LDR_CLOSED_THRESHOLD)
                          : (filtered >= LDR_CLOSED_THRESHOLD);
  bool inOpenBand = LDR_CLOSED_IS_DARKER ? (filtered >= LDR_OPEN_THRESHOLD)
                                         : (filtered <= LDR_OPEN_THRESHOLD);

  if (inClosedBand) {
    if (confirm < LDR_CONFIRM_SAMPLES) {
      ++confirm;
    }
    if (confirm >= LDR_CONFIRM_SAMPLES) {
      gLdrClosed = true;
    }
  } else if (inOpenBand) {
    confirm = 0;
    gLdrClosed = false;
  }

  return filtered;
}

void IRAM_ATTR onControlTimer() {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(controlTimerSemaphore, &woken);
  portYIELD_FROM_ISR(woken);
}

// ---------------------------------------------------------------------------
// Core 1: 100 Hz lid control
// ---------------------------------------------------------------------------

void lidControlTask(void *arg) {
  (void)arg;
  int homeSteps = 0;
  int closeSteps = 0;
  uint32_t lastStartUs = 0;
  bool havePrevious = false;

  while (true) {
    if (xSemaphoreTake(controlTimerSemaphore, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    const uint32_t startUs = micros();
    const int ldrSample = analogRead(PIN_LDR);

    if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(QUEUE_WAIT_MS)) != pdTRUE) {
      continue;
    }

    updateLdrClosed(ldrSample);
    const SystemState state = gState;

    if (state == SystemState::HOMING) {
      if (gLdrClosed || homeSteps >= STEPS_90) {
        gPositionSteps = 0;
        gMotionComplete = true;
        stepperIdle();
        homeSteps = 0;
      } else {
        for (int i = 0; i < STEPS_PER_TICK; ++i) {
          stepperAdvance(DIR_CLOSE);
          ++homeSteps;
        }
      }
    } else if (state == SystemState::OPENING) {
      homeSteps = 0;
      closeSteps = 0;
      if (gPositionSteps >= STEPS_90) {
        gMotionComplete = true;
        stepperIdle();
      } else {
        for (int i = 0; i < STEPS_PER_TICK && gPositionSteps < STEPS_90; ++i) {
          stepperAdvance(DIR_OPEN);
        }
      }
    } else if (state == SystemState::CLOSING) {
      if (gLdrClosed || closeSteps >= STEPS_90) {
        gPositionSteps = 0;
        gMotionComplete = true;
        stepperIdle();
        closeSteps = 0;
      } else if (closeSteps >= (STEPS_90 + CLOSE_EXTRA_STEPS)) {
        gLimitHit = true;
        stepperIdle();
      } else {
        for (int i = 0; i < STEPS_PER_TICK; ++i) {
          stepperAdvance(DIR_CLOSE);
          ++closeSteps;
        }
      }
    } else {
      stepperIdle();
      homeSteps = 0;
      closeSteps = 0;
    }

    const uint32_t elapsedUs = micros() - startUs;
    if (elapsedUs > DEADLINE_BUDGET_US) {
      ++gDeadlineMissCount;
      ++gDeadlineStreak;
      if (gDeadlineStreak == 1) {
        publishEvent(SystemEventType::DEADLINE_MISS, state, FaultCode::NONE);
      }
    } else {
      gDeadlineStreak = 0;
    }

    // First pass has no previous timestamp, so there is no period to report.
    if (havePrevious) {
      timingRecord(&gCtrlTiming, startUs - lastStartUs, CONTROL_PERIOD_US,
                   elapsedUs, DEADLINE_BUDGET_US);
    }
    lastStartUs = startUs;
    havePrevious = true;

    xSemaphoreGive(sharedMutex);
  }
}

// ---------------------------------------------------------------------------
// Core 1: 50 Hz safety / state machine
// ---------------------------------------------------------------------------

void safetyTask(void *arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  bool pirHigh = false;
  bool binFull = false;
#if AM2320_ENABLED
  bool sawValidDht = false;
  uint32_t lastDhtOkMs = millis();
#endif

  while (true) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    const uint32_t now = millis();

    ControlCommand command = {};
    while (xQueueReceive(controlCommandQueue, &command, 0) == pdTRUE) {
      if (command.type == ControlCommandType::RESET) {
        if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(QUEUE_WAIT_MS)) == pdTRUE) {
          enterState(SystemState::HOMING, FaultCode::NONE);
          xSemaphoreGive(sharedMutex);
        }
      }
    }

    SensorData sensors = {};
    xQueuePeek(sensorDataQueue, &sensors, 0);
    pirHigh = sensors.pirHigh;
    binFull = sensors.binFull;
#if AM2320_ENABLED
    if (sensors.dhtValid) {
      sawValidDht = true;
      lastDhtOkMs = sensors.timestampMs;
    }
#endif

    if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(QUEUE_WAIT_MS)) != pdTRUE) {
      continue;
    }

    gLatestSensors = sensors;
    const uint32_t inStateMs = now - gStateEnteredMs;

    if (gDeadlineStreak >= DEADLINE_FAULT_STREAK &&
        gState != SystemState::FAULT_SAFE) {
      enterState(SystemState::FAULT_SAFE, FaultCode::DEADLINE);
    }

#if AM2320_ENABLED
    if (sawValidDht && gState != SystemState::FAULT_SAFE &&
        gState != SystemState::HOMING && (now - lastDhtOkMs) > DHT_STALE_MS) {
      enterState(SystemState::FAULT_SAFE, FaultCode::STALE_SENSOR);
    }
#endif

    switch (gState) {
      case SystemState::HOMING:
        if (pirHigh) {
          ++gOpeningCount;
          gPositionSteps = 0;
          publishEvent(SystemEventType::MOTION, gState, FaultCode::NONE);
          enterState(SystemState::OPENING, FaultCode::NONE);
        } else if (gMotionComplete) {
          enterState(SystemState::CLOSED, FaultCode::NONE);
        } else if (gLimitHit || inStateMs > HOME_TIMEOUT_MS) {
          gPositionSteps = 0;
          enterState(SystemState::CLOSED, FaultCode::NONE);
        }
        break;

      case SystemState::CLOSED:
        if (pirHigh) {
          ++gOpeningCount;
          publishEvent(SystemEventType::MOTION, gState, FaultCode::NONE);
          enterState(SystemState::OPENING, FaultCode::NONE);
        } else if (binFull) {
          publishEvent(SystemEventType::FULL_BIN, gState, FaultCode::NONE);
          enterState(SystemState::FULL, FaultCode::NONE);
        }
        break;

      case SystemState::OPENING:
        if (gMotionComplete || gPositionSteps >= STEPS_90) {
          enterState(SystemState::HOLD_OPEN, FaultCode::NONE);
        }
        break;

      case SystemState::HOLD_OPEN:
        if (!pirHigh && inStateMs >= HOLD_AFTER_PIR_LOW_MS) {
          enterState(SystemState::CLOSING, FaultCode::NONE);
        } else if (pirHigh) {
          gStateEnteredMs = now;
        }
        break;

      case SystemState::CLOSING:
        if (pirHigh) {
          publishEvent(SystemEventType::MOTION, gState, FaultCode::NONE);
          enterState(SystemState::OPENING, FaultCode::NONE);
        } else if (gMotionComplete) {
          enterState(binFull ? SystemState::FULL : SystemState::CLOSED,
                     FaultCode::NONE);
        } else if (gLimitHit || inStateMs > CLOSE_TIMEOUT_MS) {
          ++gTimeoutCount;
          publishEvent(SystemEventType::TIMEOUT, gState, FaultCode::CLOSE_TIMEOUT);
          enterState(SystemState::FAULT_SAFE, FaultCode::CLOSE_TIMEOUT);
        }
        break;

      case SystemState::FULL:
        if (pirHigh) {
          ++gOpeningCount;
          publishEvent(SystemEventType::MOTION, gState, FaultCode::NONE);
          enterState(SystemState::OPENING, FaultCode::NONE);
        } else if (!binFull) {
          enterState(SystemState::CLOSED, FaultCode::NONE);
        }
        break;

      case SystemState::FAULT_SAFE:
        stepperIdle();
        break;
    }

    xSemaphoreGive(sharedMutex);
  }
}

// ---------------------------------------------------------------------------
// Core 0: sensors
// ---------------------------------------------------------------------------

void IRAM_ATTR onInputTimer() {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(inputTimerSemaphore, &woken);
  portYIELD_FROM_ISR(woken);
}

// Core 0, exactly 64 Hz, released by the second hardware timer. Owns every
// physical push-input on the board: today the PIR, and any button added later
// belongs here too rather than in a task with its own ad-hoc timing.
void inputTask(void *arg) {
  (void)arg;
  pinMode(PIN_PIR, INPUT);

  // The second timer is started HERE and not in setup() on purpose. An ESP32
  // timer interrupt is installed on whichever core calls
  // timerAttachInterrupt(), and this task is pinned to core 0. Moving these
  // three lines into setup() would silently put the ISR on core 1 next to the
  // lid control timer and collapse the per-core split.
  inputTimer = timerBegin(1000000);
  timerAttachInterrupt(inputTimer, &onInputTimer);
  timerAlarm(inputTimer, INPUT_PERIOD_US, true, 0);

  bool stable = false;
  uint8_t disagree = 0;
  uint32_t lastStartUs = 0;
  bool havePrevious = false;

  while (true) {
    if (xSemaphoreTake(inputTimerSemaphore, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    const uint32_t startUs = micros();

    // Single code path to the bottom of the loop, so every release is timed.
    const bool raw = (digitalRead(PIN_PIR) == HIGH);
    if (raw == stable) {
      disagree = 0;  // nothing pending, any earlier blip is discarded
    } else if (disagree < PIR_CONFIRM_SAMPLES) {
      ++disagree;
    }

    if (raw != stable && disagree >= PIR_CONFIRM_SAMPLES) {
      disagree = 0;
      stable = raw;

      if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(QUEUE_WAIT_MS)) == pdTRUE) {
        gPirStable = stable;
        xSemaphoreGive(sharedMutex);
      }

      ControlCommand command = {};
      command.type =
          stable ? ControlCommandType::OPEN : ControlCommandType::CLOSE;
      xQueueSend(controlCommandQueue, &command, 0);
    }

    const uint32_t execUs = micros() - startUs;
    if (havePrevious) {
      timingRecord(&gInputTiming, startUs - lastStartUs, INPUT_PERIOD_US, execUs,
                   INPUT_BUDGET_US);
    }
    lastStartUs = startUs;
    havePrevious = true;
  }
}

// Synthetic CPU load, for the "measured under CPU load" evidence the proposal
// promises. Pinned to core 1 at priority 1, below the 100 Hz control task and
// the 50 Hz safety task, so it consumes only whatever CPU they leave behind and
// cannot starve them. It yields one tick after each spin because a pure busy
// loop would block core 1's idle task and trip the FreeRTOS task watchdog,
// producing a crash report instead of a measurement.
void loadTask(void *arg) {
  (void)arg;
  while (true) {
    if (!gLoadEnabled) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    const uint32_t spinStartUs = micros();
    volatile uint32_t sink = 0;
    while ((micros() - spinStartUs) < LOAD_SPIN_US) {
      sink = sink + 1;
    }
    vTaskDelay(1);
  }
}

void sensorTask(void *arg) {
  (void)arg;
  uint8_t nearSamples = 0;
  bool binFull = false;
  uint32_t lastDistanceMs = 0;
  SensorData data = {};

  // PIN_PIR is owned by inputTask, which samples and debounces it at 64 Hz.
  pinMode(PIN_SR04_TRIG, OUTPUT);
  pinMode(PIN_SR04_ECHO, INPUT);
  digitalWrite(PIN_SR04_TRIG, LOW);
  analogReadResolution(12);

  while (true) {
    const uint32_t now = millis();
    data.timestampMs = now;

    SystemState stateCopy = SystemState::CLOSED;
    if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(QUEUE_WAIT_MS)) == pdTRUE) {
      stateCopy = gState;
      data.pirHigh = gPirStable;
      data.ldrRaw = gLdrRaw;
      data.ldrFiltered = gLdrFiltered;
      data.temperatureC = gDhtTempC;
      data.humidityPct = gDhtHumPct;
      data.dhtValid = gDhtOk;
      xSemaphoreGive(sharedMutex);
    }

    data.binFull = binFull;

    const bool lidClosed = (stateCopy == SystemState::CLOSED ||
                            stateCopy == SystemState::FULL ||
                            stateCopy == SystemState::HOMING);

    if (lidClosed && (now - lastDistanceMs) >= SENSOR_DISTANCE_MS) {
      lastDistanceMs = now;
      digitalWrite(PIN_SR04_TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(PIN_SR04_TRIG, HIGH);
      delayMicroseconds(TRIGGER_PULSE_US);
      digitalWrite(PIN_SR04_TRIG, LOW);
      const uint32_t echoUs = pulseIn(PIN_SR04_ECHO, HIGH, ECHO_TIMEOUT_US);
      if (echoUs == 0) {
        data.distanceValid = false;
        if (!binFull) {
          nearSamples = 0;
        }
      } else {
        data.distanceCm = echoUs * SOUND_CM_PER_US / 2.0F;
        data.distanceValid = true;
        data.fillPct = fillPercent(data.distanceCm);
        if (binFull) {
          if (data.distanceCm > ALARM_STOP_CM) {
            binFull = false;
            nearSamples = 0;
          }
        } else if (data.distanceCm < ALARM_START_CM) {
          if (nearSamples < REQUIRED_NEAR_SAMPLES) {
            ++nearSamples;
          }
          if (nearSamples >= REQUIRED_NEAR_SAMPLES) {
            binFull = true;
          }
        } else {
          nearSamples = 0;
        }
      }
    }

    data.binFull = binFull;

    xQueueOverwrite(sensorDataQueue, &data);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_PIR_MS));
  }
}

#if AM2320_ENABLED
void am2320Task(void *arg) {
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(2000));

  while (true) {
    float temperatureC = 0.0F;
    float humidityPct = 0.0F;
    bool ok = false;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      ok = readAm2320(&temperatureC, &humidityPct);
      xSemaphoreGive(i2cMutex);
    }

    if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(QUEUE_WAIT_MS)) == pdTRUE) {
      if (ok) {
        gDhtTempC = temperatureC;
        gDhtHumPct = humidityPct;
        gDhtOk = true;
      }
      xSemaphoreGive(sharedMutex);
    }

    if (ok) {
      Serial.printf("OK  T=%.1f C  RH=%.1f %%\n", temperatureC, humidityPct);
    } else {
      Serial.println("AM2320 bad read (keep last LCD value if any)");
    }

    vTaskDelay(pdMS_TO_TICKS(SENSOR_DHT_MS));
  }
}
#endif

// ---------------------------------------------------------------------------
// Core 0: LCD, buzzer, flash log, Serial
// ---------------------------------------------------------------------------

void appendLog(const SensorData &sensors, SystemState state, FaultCode fault,
               const char *eventName) {
  File file = LittleFS.open("/bin_log.csv", FILE_APPEND);
  if (!file) {
    return;
  }
  file.printf("%lu,%s,%u,%.1f,%.1f,%.0f,%d,%s,%lu,%lu,%lu\n",
              (unsigned long)millis(), stateName(state), (unsigned)fault,
              sensors.dhtValid ? sensors.temperatureC : NAN,
              sensors.dhtValid ? sensors.humidityPct : NAN,
              sensors.distanceValid ? sensors.fillPct : NAN, sensors.ldrFiltered,
              eventName, (unsigned long)gOpeningCount,
              (unsigned long)gTimeoutCount, (unsigned long)gDeadlineMissCount);
  file.close();
}

void uiTask(void *arg) {
  (void)arg;
  bool buzzerReady =
      ledcAttach(PIN_BUZZER, BUZZER_FREQ_HZ, BUZZER_PWM_BITS);
  if (buzzerReady) {
    ledcWriteTone(PIN_BUZZER, 0);
  }

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    xSemaphoreGive(i2cMutex);
  }

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  } else if (!LittleFS.exists("/bin_log.csv")) {
    File file = LittleFS.open("/bin_log.csv", FILE_WRITE);
    if (file) {
      file.println(
          "ms,state,fault,temp_c,rh,fill_pct,ldr,event,opens,timeouts,deadline");
      file.close();
    }
  }

  uint32_t lastSnapshotMs = 0;
  uint32_t lastLcdInitMs = millis();
  uint32_t lastFaultBeepMs = 0;
  bool faultToneOn = false;
  SystemState shownState = SystemState::HOMING;
  FaultCode shownFault = FaultCode::NONE;

  while (true) {
    SystemEvent event = {};
    while (xQueueReceive(systemEventQueue, &event, 0) == pdTRUE) {
      SensorData sensors = {};
      SystemState state = event.state;
      FaultCode fault = event.faultCode;
      uint32_t opens = 0;
      uint32_t timeouts = 0;
      uint32_t deadlines = 0;

      if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(QUEUE_WAIT_MS)) == pdTRUE) {
        sensors = gLatestSensors;
        state = gState;
        fault = gFault;
        opens = gOpeningCount;
        timeouts = gTimeoutCount;
        deadlines = gDeadlineMissCount;
        xSemaphoreGive(sharedMutex);
      }

      const char *eventName = "state";
      switch (event.type) {
        case SystemEventType::MOTION:
          eventName = "motion";
          break;
        case SystemEventType::FULL_BIN:
          eventName = "full";
          break;
        case SystemEventType::TIMEOUT:
          eventName = "timeout";
          persistCounters();
          break;
        case SystemEventType::FAULT:
          eventName = "fault";
          persistCounters();
          break;
        case SystemEventType::DEADLINE_MISS:
          eventName = "deadline";
          persistCounters();
          break;
        default:
          if (event.state == SystemState::CLOSED ||
              event.state == SystemState::HOLD_OPEN) {
            persistCounters();
          }
          break;
      }

      Serial.printf("EVENT %s state=%s fault=E%02u fill=%.0f ldr=%d opens=%lu\n",
                    eventName, stateName(state), (unsigned)fault,
                    sensors.fillPct, sensors.ldrFiltered,
                    (unsigned long)opens);
      appendLog(sensors, state, fault, eventName);
      (void)timeouts;
      (void)deadlines;
    }

    SensorData sensors = {};
    SystemState state = SystemState::HOMING;
    FaultCode fault = FaultCode::NONE;
    if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(QUEUE_WAIT_MS)) == pdTRUE) {
      sensors = gLatestSensors;
      state = gState;
      fault = gFault;
      xSemaphoreGive(sharedMutex);
    }
    xQueuePeek(sensorDataQueue, &sensors, 0);

    char line0[17];
    char line1[17];
    // Every string below is padded to the full 16 columns on purpose. A short
    // write leaves whatever was in the trailing columns untouched, which after
    // a corruption event means leftover garbage sits there indefinitely.
    if (state == SystemState::FAULT_SAFE) {
      snprintf(line0, sizeof(line0), "FAULT E%02u       ", (unsigned)fault);
    } else {
      snprintf(line0, sizeof(line0), "%-7s %3.0f%%    ", stateName(state),
               sensors.distanceValid ? sensors.fillPct : 0.0F);
    }
#if AM2320_ENABLED
    if (sensors.dhtValid) {
      snprintf(line1, sizeof(line1), "T=%4.1fC H=%02.0f    ",
               sensors.temperatureC, sensors.humidityPct);
    } else {
      snprintf(line1, sizeof(line1), "T=--.-C H=--    ");
    }
#else
    snprintf(line1, sizeof(line1), "AM2320 off      ");
#endif
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      // A stateless sensor recovers from a bus glitch on its own because the
      // next read starts fresh. This display cannot: it holds 4-bit phase, so
      // one lost nibble corrupts everything afterwards. Re-seat it periodically
      // rather than leaving it garbled until someone power-cycles the board.
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
      xSemaphoreGive(i2cMutex);
    }

    if (buzzerReady) {
      if (state == SystemState::FAULT_SAFE) {
        const uint32_t now = millis();
        if ((now - lastFaultBeepMs) >= 250) {
          lastFaultBeepMs = now;
          faultToneOn = !faultToneOn;
          ledcWriteTone(PIN_BUZZER, faultToneOn ? BUZZER_FREQ_HZ : 0);
        }
      } else if (state == SystemState::FULL) {
        ledcWriteTone(PIN_BUZZER, BUZZER_FREQ_HZ);
        faultToneOn = false;
      } else {
        ledcWriteTone(PIN_BUZZER, 0);
        faultToneOn = false;
      }
    }

    const uint32_t now = millis();
    if ((now - lastSnapshotMs) >= LOG_SNAPSHOT_MS || state != shownState ||
        fault != shownFault) {
      lastSnapshotMs = now;
      shownState = state;
      shownFault = fault;
      appendLog(sensors, state, fault, "snapshot");
    }

    vTaskDelay(pdMS_TO_TICKS(UI_PERIOD_MS));
  }
}

// ---------------------------------------------------------------------------
// Arduino entry
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(200);

  pinMode(PIN_STEPPER_IN1, OUTPUT);
  pinMode(PIN_STEPPER_IN2, OUTPUT);
  pinMode(PIN_STEPPER_IN3, OUTPUT);
  pinMode(PIN_STEPPER_IN4, OUTPUT);
  stepperIdle();

  prefs.begin("smartbin", false);
  gOpeningCount = prefs.getUInt("opens", 0);
  gTimeoutCount = prefs.getUInt("timeouts", 0);
  gDeadlineMissCount = prefs.getUInt("deadlines", 0);

  sensorDataQueue = xQueueCreate(1, sizeof(SensorData));
  controlCommandQueue = xQueueCreate(8, sizeof(ControlCommand));
  systemEventQueue = xQueueCreate(16, sizeof(SystemEvent));
  controlTimerSemaphore = xSemaphoreCreateBinary();
  inputTimerSemaphore = xSemaphoreCreateBinary();
  sharedMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();

  Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  gStateEnteredMs = millis();

  xTaskCreatePinnedToCore(lidControlTask, "lid", STACK_LID, nullptr, 5, nullptr,
                          1);
  xTaskCreatePinnedToCore(safetyTask, "safety", STACK_SAFETY, nullptr, 4,
                          nullptr, 1);
  // inputTask starts its own hardware timer so the ISR lands on core 0.
  xTaskCreatePinnedToCore(inputTask, "input", STACK_INPUT, nullptr, 4, nullptr,
                          0);
  xTaskCreatePinnedToCore(sensorTask, "sensor", STACK_SENSOR, nullptr, 3,
                          nullptr, 0);
#if AM2320_ENABLED
  xTaskCreatePinnedToCore(am2320Task, "am2320", STACK_DHT, nullptr, 2, nullptr,
                          0);
#endif
  xTaskCreatePinnedToCore(uiTask, "ui", STACK_UI, nullptr, 2, nullptr, 0);
  // Idle unless "load on" is typed. Priority 1 keeps it below control/safety.
  xTaskCreatePinnedToCore(loadTask, "load", STACK_LOAD, nullptr, 1, nullptr, 1);

  controlTimer = timerBegin(1000000);
  timerAttachInterrupt(controlTimer, &onControlTimer);
  timerAlarm(controlTimer, CONTROL_PERIOD_US, true, 0);

  Serial.println("Smart Garbage Bin baseline started.");
  Serial.printf("Timer 1: core 1, %lu Hz lid control. "
                "Timer 2: core 0, %lu Hz input/debounce.\n",
                (unsigned long)CONTROL_HZ, (unsigned long)INPUT_HZ);
  Serial.println("AM2320 on I2C 0x5C (SDA=GPIO8 SCL=GPIO9), not a DHT11.");
#if !AM2320_ENABLED
  Serial.println("AM2320 skipped. No T/RH, no E03.");
#endif
  Serial.println("BLE/IR are disabled until core V&V PASSES.");
  printHelp();
}

void printTimingRow(const char *label, const TimingStats *s,
                    uint32_t nominalUs) {
  if (s->samples == 0) {
    Serial.printf("  %-6s no samples yet\n", label);
    return;
  }
  // Copy first: the owning task keeps writing while we format.
  const uint32_t n = s->samples;
  const uint32_t pMin = s->periodMinUs;
  const uint32_t pMax = s->periodMaxUs;
  const uint32_t jMax = s->jitterMaxUs;
  const uint32_t eMax = s->execMaxUs;
  const uint32_t eAvg = s->execAvgUs;
  const uint32_t over = s->overrunCount;
  Serial.printf("  %-6s n=%-9lu nominal=%luus\n", label, (unsigned long)n,
                (unsigned long)nominalUs);
  Serial.printf("         period min/max = %lu / %lu us   jitter max = %lu us\n",
                (unsigned long)pMin, (unsigned long)pMax, (unsigned long)jMax);
  Serial.printf("         exec avg/max   = %lu / %lu us   overruns = %lu\n",
                (unsigned long)eAvg, (unsigned long)eMax, (unsigned long)over);
}

void printStats() {
  Serial.println();
  Serial.println("=== timing evidence ===");
  Serial.printf("synthetic CPU load on core 1: %s\n",
                gLoadEnabled ? "ON" : "off");
  printTimingRow("ctrl", &gCtrlTiming, CONTROL_PERIOD_US);
  printTimingRow("input", &gInputTiming, INPUT_PERIOD_US);
  Serial.printf("  budgets: ctrl %lu us, input %lu us\n",
                (unsigned long)DEADLINE_BUDGET_US,
                (unsigned long)INPUT_BUDGET_US);
  Serial.printf("  lifetime counters: opens=%lu timeouts=%lu deadlineMiss=%lu\n",
                (unsigned long)gOpeningCount, (unsigned long)gTimeoutCount,
                (unsigned long)gDeadlineMissCount);
  Serial.println("(period is start-of-work to start-of-work, so it includes");
  Serial.println(" timer jitter and release latency. exec is the work itself.)");
}

void printHelp() {
  Serial.println();
  Serial.println("commands: reset | stats | statsreset | load on | load off | help");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.toLowerCase();
    if (line == "reset") {
      ControlCommand command = {};
      command.type = ControlCommandType::RESET;
      xQueueSend(controlCommandQueue, &command, 0);
      Serial.println("RESET queued - re-homing");
    } else if (line == "stats") {
      printStats();
    } else if (line == "statsreset") {
      timingClear(&gCtrlTiming);
      timingClear(&gInputTiming);
      Serial.println("timing stats cleared");
    } else if (line == "load on") {
      gLoadEnabled = true;
      Serial.println("synthetic load ON (core 1, priority 1)");
      Serial.println("run for a while, then type stats");
    } else if (line == "load off") {
      gLoadEnabled = false;
      Serial.println("synthetic load off");
    } else if (line == "help") {
      printHelp();
    } else if (line.length() > 0) {
      Serial.println("unknown command, type help");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}
