# Speed Limiter Firmware

ESP32-based speed limiter firmware with modular tasks for CAN speed input, pedal sensing, limiter logic, and analog/PWM output generation.

This README documents the overall system behavior and configuration knobs used by the current codebase.

## System Overview

- Hardware target: ESP32-WROVER (`esp-wrover-kit` in PlatformIO)
- Runtime architecture: FreeRTOS tasks + shared state
- Primary modes:
  - Automatic limiter (default)
  - Manual relay mode (`MANUAL=1`)
  - Logger firmware (`BUILD_TEST_LOGGER=1`)
- Local AP web UI for status/configuration (disabled in logger build)

## Runtime Architecture

Main entry is `src/main.cpp`.

In normal automatic firmware, startup flow is:

1. Initialize persistent speed limit storage (NVS)
2. Initialize modules:
   - `AdcModule_Begin()`
   - `PwmModule_Begin()`
   - `CanModule_Begin()`
   - `LogicModule_Begin()`
3. Start tasks:
   - `CanModule_StartTask()`
   - `AdcModule_StartTask()`
   - `PwmModule_StartTask()`
   - `LogicModule_StartTask()`
4. Start web server AP/UI

All modules communicate through `shared_state` globals + accessors.

## Module Behavior

### CAN Module (`src/can_module.cpp`)

- Reads vehicle speed and RPM from CAN.
- Default source is OBD PID polling (`SPEED_SOURCE_OBD=1`):
  - Speed PID `0x0D`
  - RPM PID `0x0C`
  - Request period: `OBD_REQ_INTERVAL_MS` (default `40ms`, ~25Hz)
- Alternative source (`SPEED_SOURCE_OBD=0`): broadcast frame parsing using `SPEED_FRAME_ID` and `SPEED_BYTE_INDEX`.
- Publishes:
  - `g_speed_kmh`, `g_speed_last_update_ms`
  - `g_rpm`, `g_rpm_last_update_ms`

### ADC Module (`src/adc_module.cpp`)

- Samples APS channels:
  - APS1: GPIO39 / ADC1_CH3
  - APS2: GPIO36 / ADC1_CH0
- Converts ADC voltage to ECU-level using `PEDAL_SCALE`.
- Publishes APS values to shared state continuously.

### PWM/DAC Output Module (`src/pwm_module.cpp`)

- Reads desired ECU-level output pair from shared state.
- Applies output divider compensation (`OUTPUT_DIVIDER_GAIN`) before driving pins.
- Output mode controlled by `USE_DAC_OUTPUT`:
  - `0` (default): PWM on GPIO21/GPIO22 at 5kHz, 10-bit
  - `1`: DAC on GPIO25/GPIO26
- Uses safe defaults at boot:
  - `DEFAULT_SIGNAL_S1_V` (0.350V)
  - `DEFAULT_SIGNAL_S2_V` (0.700V)

### Logic Module (Limiter)

Selected by build flag:

- `USE_ALGORITHM_MODULE=1` -> `src/algorithm.cpp` (legacy algorithm)
- `USE_ALGORITHM_MODULE=0` -> `src/new_algorithm.cpp` (current default)

`src/new_algorithm.cpp` behavior summary:

- State machine:
  - `PASS_THROUGH`
  - `OVERSHOOT_CONTROL`
  - `LIMIT_ACTIVE`
  - `FAULT`
- Relay control is anti-chatter protected using `RELAY_MIN_CHANGE_INTERVAL_MS`.
- Limiter activates when speed enters configured band and pedal is not idle.
- APPS table cap is selected from `include/new_algorithm.h` (with interpolation).
- Decay behavior:
  - Decay starts near limit (`DECAY_START_BEFORE_SL_KMH`)
  - Decay rate ramps with speed and gets harsher when overspeed
  - Decay is rate-limited in time (`DECAY_APPLY_INTERVAL_MS`)
  - Hold value is not allowed below dynamic floor:
    - `apps_floor = apps_table - (apps_table - prev_point) * DECAY_FLOOR_FROM_PREV_DELTA_FACTOR`
    - Then clamped to safety defaults (`DEFAULT_SIGNAL_S1_V`, `DEFAULT_SIGNAL_S2_V`)
- Fail-safe:
  - If speed is invalid (`SPEED_TIMEOUT_MS`) or speed limit is `0`, relay is forced OFF and output passes through safely.

### Speed Limit Store (`src/speed_limit_store.cpp`)

- Persists speed limit in NVS (`Preferences`):
  - Namespace: `speedLimiter`
  - Key: `sl`
- Value is clamped to `0..250`.
- `0` means limiter disabled.

### Web Server Module (`src/web_server_module.cpp`)

- Starts ESP32 AP:
  - SSID format: `SpeedLimiterXXXX` (MAC suffix)
  - Password: `P@ssw0rd`
- Routes:
  - `GET /` status page
  - `GET /config` config page
  - `GET /api/status` JSON status
  - `POST /set` set speed limit (`sl=0..250`)
  - `POST /reset` reset to default speed limit
  - Manual mode only:
    - `POST /override`
    - `POST /override_relay`

## Build Modes

Configured in `platformio.ini`:

- `esp32-wrover-n4r8-pwm` (default)
  - `USE_DAC_OUTPUT=0`
  - `USE_ALGORITHM_MODULE=0`
- `esp32-wrover-n4r8-dac`
  - `USE_DAC_OUTPUT=1`
- `esp32-wrover-n4r8-logger`
  - `BUILD_TEST_LOGGER=1`
- `esp32-wrover-n4r8-manual`
  - `MANUAL=1`
- `esp32-wrover-n4r8-pwm-debug`
  - Debug build

## Key Configuration Reference

Main configuration file: `include/sl_config.h`

### Global Build/Feature Flags

- `BUILD_TEST_LOGGER` (default `0`)
- `USE_ALGORITHM_MODULE` (default `0`)
- `USE_DAC_OUTPUT` (default `0`)
- `SPEED_SOURCE_OBD` (default `1`)
- `MANUAL` (set from PlatformIO environment)

### Safety and Limits

- `SPEED_TIMEOUT_MS = 500` (speed validity timeout)
- `DEFAULT_SIGNAL_S1_V = 0.350`
- `DEFAULT_SIGNAL_S2_V = 0.700`
- `RELAY_MIN_CHANGE_INTERVAL_MS = 2000`
- `SPEED_LIMIT_DEFAULT_KMH = 50`
- `SPEED_LIMIT_ACTIVATION_OFFSET_KMH = 10`
- `SPEED_LIMIT_DEACTIVATION_HYSTERESIS_KMH = 10`

### CAN and OBD

- `CAN_BAUD = 500000`
- `OBD_REQ_ID = 0x7DF`
- `OBD_PID_SPEED = 0x0D`
- `OBD_PID_RPM = 0x0C`
- `OBD_REQ_INTERVAL_MS = 40`

### Pins

- Relay: GPIO23
- CAN RX/TX: GPIO32 / GPIO33
- APS input: GPIO39 / GPIO36
- Output pins:
  - PWM mode: GPIO21 / GPIO22
  - DAC mode: GPIO25 / GPIO26

### Task Intervals

- `ADC_SAMPLE_INTERVAL_MS = 1`
- `PWM_UPDATE_INTERVAL_MS = 1`
- `LOGIC_LOOP_INTERVAL_MS = 1` (legacy constant family)

## Build and Flash

Common commands:

```bash
pio run
pio run --target upload
pio run --target monitor
```

Specific environment:

```bash
pio run -e esp32-wrover-n4r8-manual --target upload
```

## Behavior Notes

- Limiter state depends on both speed validity and configured limit.
- If speed updates stop, system goes fail-safe (relay OFF).
- Setting speed limit to `0` disables limiting.
- Web UI and logic both read/write the same shared speed-limit state.
- In manual mode with override enabled, relay follows web override and ignores automatic speed logic.

## Related Docs

- Build/environment details: `README_PlatformIO.md`
- Requirement/spec notes: `Software_Requirements_Document.txt`
- Test procedure: `NewFirmware_Test_Procedure_Detailed.txt`
