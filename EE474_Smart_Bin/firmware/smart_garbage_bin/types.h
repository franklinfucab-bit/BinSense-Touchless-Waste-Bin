/*
 * Filename: types.h
 * Authors: Shawn Yin and Franklin Fu
 * Date: August 21, 2026
 * Description: Shared types for queues, lid states, and fault codes.
 */

#pragma once

#include <stdint.h>

enum class SystemState : uint8_t {
  HOMING = 0,
  CLOSED,
  OPENING,
  HOLD_OPEN,
  CLOSING,
  FULL,
  FAULT_SAFE
};

enum class ControlCommandType : uint8_t {
  NONE = 0,
  OPEN,
  CLOSE,
  RESET
};

enum class SystemEventType : uint8_t {
  MOTION = 0,
  FULL_BIN,
  TIMEOUT,
  FAULT,
  STATE_CHANGE,
  DEADLINE_MISS
};

enum class FaultCode : uint8_t {
  NONE = 0,
  HOME_TIMEOUT = 1,
  CLOSE_TIMEOUT = 2,
  STALE_SENSOR = 3,
  DEADLINE = 4
};

struct SensorData {
  float temperatureC;
  float humidityPct;
  float distanceCm;
  float fillPct;
  int ldrRaw;
  int ldrFiltered;
  bool dhtValid;
  bool distanceValid;
  bool pirHigh;
  bool binFull;
  uint32_t timestampMs;
};

struct ControlCommand {
  ControlCommandType type;
};

struct SystemEvent {
  SystemEventType type;
  SystemState state;
  FaultCode faultCode;
  uint32_t timestampMs;
};
