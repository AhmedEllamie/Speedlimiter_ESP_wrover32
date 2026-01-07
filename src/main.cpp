/*
================================================================================
Speed Limiter System - NEW FIRMWARE (SRD COMPLIANT)
================================================================================
Target: ESP32-WROVER-E-N4R8 (Arduino framework + ESP-IDF drivers)
Old firmware reference: ESP32_ADC2_IDF_G2.ino

This firmware is updated to match:
  - Software_Requirements_Document.txt (new SRD)
  - SRD_Comparison_Differences.txt (fixes gaps: Mode1, RS232, watchdog, etc.)

Key changes vs old firmware:
  - Implements Mode 1 (Manual) + Mode 2 (Automatic), selected by hardware switch
  - Pinout updated to match SRD
  - Speed limit default 40 km/h and configurable + speed calibration factor
  - RS232 UART implemented on IO15/IO34 (Serial2)
  - CAN timeout handling (5s) -> fail-safe relay OFF
  - Watchdog enabled
  - CAN RX callback only stores data (no blocking / no delays inside callback)

OUTPUT METHOD (PWM vs DAC):
  ESP32-WROVER-E-N4R8 has DAC only on GPIO25 and GPIO26.
  SRD specifies IO21/IO22 for outputs (these pins do NOT have DAC capability).
  
  Default: PWM mode (USE_DAC_OUTPUT = 0)
    - Uses PWM on IO21/IO22 (matches SRD pin assignment)
    - Requires external RC filter (e.g., 1kΩ + 10µF) for smooth analog signal
    - 10-bit resolution (0-1023 duty cycle)
    - 5 kHz PWM frequency (suitable for low-frequency pedal signals)
  
  Alternative: DAC mode (USE_DAC_OUTPUT = 1)
    - Uses true analog DAC on IO25/IO26
    - No external filter needed (true analog output)
    - 8-bit resolution (0-255, ~12.9 mV per step)
    - Note: Changes pin assignments (SDPS1=IO25, SDPS2=IO26)
    - Note: Potentiometer inputs must be reassigned (conflicts possible)
  
  To enable DAC mode, uncomment or define: #define USE_DAC_OUTPUT 1
  before compiling.
*/

#include <Arduino.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"
#if USE_DAC_OUTPUT
#include "driver/dac.h"
#endif

#include <esp32_can.h>
#include <Preferences.h>

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_task_wdt.h"
#include "esp_idf_version.h"

// =============================================================================
// Output Method Selection
// =============================================================================
// ESP32 has DAC only on GPIO25 and GPIO26.
// SRD specifies IO21/IO22 for outputs (no DAC available).
// Choose one:
//   - USE_DAC_OUTPUT = 0: Use PWM on IO21/IO22 (matches SRD pin assignment)
//   - USE_DAC_OUTPUT = 1: Use DAC on IO25/IO26 (true analog, but different pins)
#ifndef USE_DAC_OUTPUT
#define USE_DAC_OUTPUT     0   // Default: PWM (matches SRD pins IO21/IO22)
#endif

// =============================================================================
// Pin Assignments (NEW SRD)
// =============================================================================
#define CAL_BTN_PIN        4   // IO4  - calibration button (Mode 2)
#define BUZZER_PIN         14  // IO14 - warning buzzer
#define RELAY_PIN          23  // IO23 - relay control (HIGH=cut pedal / connect default)
#define MODE_SWITCH_PIN    13  // IO13 - hardware mode select switch input (Pull-up)
#define DIG_OUT_PIN        18  // IO18 - general purpose digital output (default mirrors limiter)

#define CAN_RX_PIN         32  // IO32
#define CAN_TX_PIN         33  // IO33

#define APS1_PIN           39  // IO39 - actual pedal signal 1 (ADC)
#define APS2_PIN           36  // IO36 - actual pedal signal 2 (ADC)

#if USE_DAC_OUTPUT
  // DAC mode: Use GPIO25/GPIO26 (ESP32 DAC pins)
  #define SDPS1_PIN        25  // IO25 - stimulated/default pedal output 1 (DAC_CHANNEL_1)
  #define SDPS2_PIN        26  // IO26 - stimulated/default pedal output 2 (DAC_CHANNEL_2)
  // Note: Potentiometer inputs must use different pins in DAC mode
  // Using IO27 and IO14 (IO14 conflicts with buzzer, but Mode1 may not need buzzer)
  #define POT1_PIN         27  // IO27 - pot input 1 (ADC2) - moved from IO25
  #define POT2_PIN         14  // IO14 - pot input 2 (ADC2) - moved from IO26
  // WARNING: In DAC mode, POT2 (IO14) conflicts with buzzer (IO14).
  //          Mode 1 (Manual) typically doesn't need buzzer, so this may be acceptable.
  //          If buzzer is needed in Mode 1, use a different pin for POT2.
#else
  // PWM mode: Use IO21/IO22 (matches SRD pin assignment)
  #define SDPS1_PIN        21  // IO21 - stimulated/default pedal output 1 (PWM)
  #define SDPS2_PIN        22  // IO22 - stimulated/default pedal output 2 (PWM)
  // Mode 1 (Manual) potentiometer inputs
  #define POT1_PIN         25  // IO25 - pot input 1 (ADC2)
  #define POT2_PIN         26  // IO26 - pot input 2 (ADC2)
#endif

// RS232 interface (NEW SRD)
#define RS232_TX_PIN      15   // IO15
#define RS232_RX_PIN      34   // IO34

// =============================================================================
// Constants & Defaults (NEW SRD)
// =============================================================================
static const uint32_t CAN_BAUD_DEFAULT = 500000;
static const uint32_t RS232_BAUD_DEFAULT = 115200;

static const uint16_t SPEED_LIMIT_DEFAULT_KMH = 40;
static const uint32_t SPEED_TIMEOUT_MS = 5000;   // SRD: keep last speed max 5s, then safe state

// SRD correction formula: Actual = ADC * (69.6 / 47.5)
static const float PEDAL_SCALE = (69.6f / 47.5f);

// SRD output divider compensation: to generate 1V output, generate 2V internally
static const float OUTPUT_DIVIDER_GAIN = 2.0f;

// SRD default signal values (Mode 1); also used as fallback if Mode 2 calibration is missing/invalid
static const float DEFAULT_SIGNAL_S1_V = 0.350f;
static const float DEFAULT_SIGNAL_S2_V = 0.700f;

// PWM output defaults (not fixed by SRD; kept compatible with old firmware)
static const uint32_t PWM_FREQ_HZ = 5000;
static const ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_10_BIT; // 0-1023
static const uint32_t PWM_MAX_DUTY = 1023;

// Buzzer pattern (non-blocking)
static const uint32_t BUZZER_TONE1_HZ = 1000;
static const uint32_t BUZZER_TONE2_HZ = 1500;
static const uint32_t BUZZER_TONE_MS  = 200;
static const uint32_t BUZZER_PAUSE_MS = 250;

// =============================================================================
// Types
// =============================================================================
enum class OperatingMode : uint8_t {
  Manual = 1,
  Automatic = 2,
};

enum class RelayOverride : uint8_t {
  Auto = 0,
  ForceOff = 1,
  ForceOn = 2,
};

enum ErrorFlags : uint32_t {
  ERR_NONE         = 0,
  ERR_CAN_TIMEOUT  = (1u << 0),
  ERR_ADC_READ     = (1u << 1),
  ERR_CAL_INVALID  = (1u << 2),
};

struct Config {
  // Mode / IO behavior
  bool mode_switch_active_low = true;       // IO13 LOW -> Manual, HIGH -> Automatic
  bool io18_mirror_limiter = true;          // IO18 mirrors limiter unless overridden
  bool io18_manual_state = false;

  // Speed source
  bool use_obd = true;                      // default follows old firmware approach
  uint32_t obd_request_interval_ms = 100;   // >=10Hz (SRD)
  uint16_t obd_request_id = 0x7DF;
  uint16_t obd_response_id = 0x7E8;
  uint8_t  obd_pid_speed = 0x0D;

  // Broadcast speed config (if use_obd=false)
  uint16_t speed_frame_id = 0x000;          // user must set if using broadcast mode
  uint8_t  speed_byte_index = 0;            // 0..7

  // Speed calibration & limit
  uint16_t speed_limit_kmh = SPEED_LIMIT_DEFAULT_KMH;
  float speed_cal_factor = 1.0f;
  float speed_cal_offset = 0.0f;
  uint8_t hysteresis_kmh = 0;               // 0 = strict SRD behavior

  // Comms
  uint32_t can_baud = CAN_BAUD_DEFAULT;
  uint32_t rs232_baud = RS232_BAUD_DEFAULT;

  // Optional output ramp (ms). 0 disables ramp.
  uint16_t output_ramp_ms = 0;
};

// =============================================================================
// Globals
// =============================================================================
Preferences prefs;
Config cfg;

// Global speed limit variable (default: 40 km/h)
// Access via: cfg.speed_limit_kmh
// Change via CLI: "set limit <kmh>" or "get limit" to read
// Persists in NVM (Preferences)

// ADC (two units: ADC1 for APS, ADC2 for pots)
static bool adc1_ok = false;
static bool adc2_ok = false;
static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc2_chars;

// ESP32 Arduino core in PlatformIO is typically ESP-IDF 4.4 (Arduino-ESP32 2.x).
// Use classic ADC driver + esp_adc_cal (IDF 4.4 compatible).
static const adc_bits_width_t ADC_WIDTH = ADC_WIDTH_BIT_12;
static const adc_atten_t ADC_ATTEN = ADC_ATTEN_DB_12; // ~0-3.3V
static const uint32_t DEFAULT_VREF_MV = 1100;

// ADC1 channels for APS signals
static const adc1_channel_t APS2_CH = ADC1_CHANNEL_0; // GPIO36
static const adc1_channel_t APS1_CH = ADC1_CHANNEL_3; // GPIO39

// ADC2 channels for potentiometers (may change in DAC mode)
#if USE_DAC_OUTPUT
static const adc2_channel_t POT1_CH = ADC2_CHANNEL_7; // GPIO27
static const adc2_channel_t POT2_CH = ADC2_CHANNEL_6; // GPIO14
#else
static const adc2_channel_t POT1_CH = ADC2_CHANNEL_8; // GPIO25
static const adc2_channel_t POT2_CH = ADC2_CHANNEL_9; // GPIO26
#endif

// Calibration (Mode 2 default outputs, ECU-level volts)
static bool defaults_valid = false;
static float defaults_v1 = 0.0f;
static float defaults_v2 = 0.0f;

// Latest measured signals (ECU-level volts)
static float pedal_v1 = 0.0f;
static float pedal_v2 = 0.0f;
static float pot_v1 = 0.0f;
static float pot_v2 = 0.0f;

// Output tracking (ECU-level volts)
static float out_v1 = 0.0f;
static float out_v2 = 0.0f;

// Limiter state
static bool limiter_active = false;
static RelayOverride relay_override = RelayOverride::Auto;
static uint32_t last_speed_rx_ms = 0;
static volatile uint8_t raw_speed_kmh = 0;

// Optional: engine RPM via OBD-II PID 0x0C (if supported by vehicle/ECU)
static uint32_t last_rpm_rx_ms = 0;
static volatile uint16_t raw_rpm = 0;

// CAN/OBD diagnostics (USB-friendly)
static volatile uint32_t can_rx_total = 0;
static volatile uint32_t can_rx_last_ms = 0;
static volatile uint32_t can_rx_last_id = 0;
static volatile bool can_rx_last_ext = false;

static volatile uint32_t obd_tx_speed_count = 0;
static volatile uint32_t obd_tx_rpm_count = 0;
static volatile uint32_t obd_rx_speed_count = 0;
static volatile uint32_t obd_rx_rpm_count = 0;
static volatile uint32_t obd_rx_other_count = 0;   // OBD-looking frames that didn't match speed/rpm PIDs
static volatile uint32_t obd_rx_unmatched_id = 0;  // OBD-looking frames ignored due to rspid mismatch

// USB status streaming (periodic one-line status output on Serial/USB)
static bool usb_stream_enabled = true;
static uint32_t usb_stream_interval_ms = 2000; // 5 Hz by default
static uint32_t usb_stream_last_ms = 0;

static uint32_t error_flags = ERR_NONE;

// =============================================================================
// Data Logger (for drive test recording)
// =============================================================================
// Records: timestamp, speed, rpm, aps1, aps2
// Can store ~30 minutes at 1Hz or ~6 minutes at 5Hz
// Retrieve with "log" command over USB
struct LogRecord {
  uint32_t timestamp_ms;  // millis() since boot
  float speed_kmh;        // calibrated speed
  uint16_t rpm;           // engine RPM
  float aps1_v;           // APS1 voltage (ECU-level)
  float aps2_v;           // APS2 voltage (ECU-level)
  uint8_t flags;          // bit0=speed_valid, bit1=rpm_valid, bit2=limiter_active
};

static const uint16_t LOG_MAX_RECORDS = 2000;  // ~40KB RAM usage
static LogRecord log_buffer[LOG_MAX_RECORDS];
static uint16_t log_count = 0;
static uint16_t log_write_idx = 0;
static bool log_enabled = false;
static bool log_wrap = false;  // true if buffer wrapped (ring buffer mode)
static uint32_t log_interval_ms = 1000;  // Default 1Hz recording
static uint32_t log_last_ms = 0;
static uint32_t log_start_time_ms = 0;  // When logging started

// Buzzer state machine
enum class BuzzerState : uint8_t { Off, Tone1, Pause, Tone2 };
static BuzzerState buz_state = BuzzerState::Off;
static uint32_t buz_next_ms = 0;

// CLI buffers (USB Serial + RS232 Serial2)
struct CliBuf {
  char buf[160];
  uint8_t len = 0;
};
static CliBuf cli_usb;
static CliBuf cli_rs232;

// =============================================================================
// Helpers
// =============================================================================
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static bool parseBool(const char *s, bool *out) {
  if (!s || !out) return false;
  if (!strcasecmp(s, "1") || !strcasecmp(s, "true") || !strcasecmp(s, "on") || !strcasecmp(s, "yes")) { *out = true; return true; }
  if (!strcasecmp(s, "0") || !strcasecmp(s, "false") || !strcasecmp(s, "off") || !strcasecmp(s, "no")) { *out = false; return true; }
  return false;
}

static void logBoth(const char *s) {
  Serial.println(s);
  Serial2.println(s);
}

static void logBothF(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  Serial2.print(buf);
}

// =============================================================================
// LEDC (PWM outputs + buzzer) / DAC (true analog outputs)
// =============================================================================
static void setupOutputs() {
#if USE_DAC_OUTPUT
  // DAC mode: Initialize true analog outputs on GPIO25/GPIO26
  dac_output_enable(DAC_CHANNEL_1); // GPIO25
  dac_output_enable(DAC_CHANNEL_2); // GPIO26
  dac_output_voltage(DAC_CHANNEL_1, 0);
  dac_output_voltage(DAC_CHANNEL_2, 0);
#else
  // PWM mode: Initialize PWM outputs on IO21/IO22
  ledc_timer_config_t timer0 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ_HZ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer0);

  ledc_channel_config_t ch0 = {
    .gpio_num = SDPS1_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&ch0);

  ledc_channel_config_t ch1 = {
    .gpio_num = SDPS2_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&ch1);
#endif

  // Timer 1 for buzzer (separate so we can change frequency without affecting outputs)
  ledc_timer_config_t timer1 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_1,
    .freq_hz = BUZZER_TONE1_HZ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer1);

  ledc_channel_config_t buz = {
    .gpio_num = BUZZER_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&buz);
}

static void buzzerOff() {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

static void buzzerOnFreq(uint32_t freq_hz) {
  ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, freq_hz);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 512); // ~50%
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

static void buzzerUpdate(bool want_active) {
  uint32_t now = millis();
  if (!want_active) {
    buz_state = BuzzerState::Off;
    buz_next_ms = 0;
    buzzerOff();
    return;
  }

  if (buz_state == BuzzerState::Off) {
    buz_state = BuzzerState::Tone1;
    buz_next_ms = now + BUZZER_TONE_MS;
    buzzerOnFreq(BUZZER_TONE1_HZ);
    return;
  }

  if (buz_next_ms != 0 && now < buz_next_ms) return;

  switch (buz_state) {
    case BuzzerState::Tone1:
      buz_state = BuzzerState::Pause;
      buz_next_ms = now + BUZZER_PAUSE_MS;
      buzzerOff();
      break;
    case BuzzerState::Pause:
      buz_state = BuzzerState::Tone2;
      buz_next_ms = now + BUZZER_TONE_MS;
      buzzerOnFreq(BUZZER_TONE2_HZ);
      break;
    case BuzzerState::Tone2:
      buz_state = BuzzerState::Tone1;
      buz_next_ms = now + BUZZER_TONE_MS;
      buzzerOnFreq(BUZZER_TONE1_HZ);
      break;
    default:
      buz_state = BuzzerState::Off;
      buzzerOff();
      break;
  }
}

// =============================================================================
// ADC
// =============================================================================
static float readVoltageAdc1(adc1_channel_t ch) {
  if (!adc1_ok) return NAN;
  int raw = adc1_get_raw(ch);
  if (raw < 0) return NAN;
  uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc1_chars);
  return (float)mv / 1000.0f;
}

static float readVoltageAdc2(adc2_channel_t ch) {
  if (!adc2_ok) return NAN;
  int raw = 0;
  esp_err_t err = adc2_get_raw(ch, ADC_WIDTH, &raw);
  if (err != ESP_OK) return NAN;
  uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc2_chars);
  return (float)mv / 1000.0f;
}

static void setupADC() {
  adc1_ok = false;
  adc2_ok = false;

  // ADC1 setup
  if (adc1_config_width(ADC_WIDTH) != ESP_OK) {
    logBoth("ADC1 width config failed");
    error_flags |= ERR_ADC_READ;
  } else if (adc1_config_channel_atten(APS1_CH, ADC_ATTEN) != ESP_OK ||
             adc1_config_channel_atten(APS2_CH, ADC_ATTEN) != ESP_OK) {
    logBoth("ADC1 channel config failed");
    error_flags |= ERR_ADC_READ;
  } else {
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF_MV, &adc1_chars);
    adc1_ok = true;
  }

  // ADC2 setup
  if (adc2_config_channel_atten(POT1_CH, ADC_ATTEN) != ESP_OK ||
      adc2_config_channel_atten(POT2_CH, ADC_ATTEN) != ESP_OK) {
    logBoth("ADC2 channel config failed");
    error_flags |= ERR_ADC_READ;
  } else {
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF_MV, &adc2_chars);
    adc2_ok = true;
  }
}

// =============================================================================
// Preferences (Config + Calibration)
// =============================================================================
static void loadDefaultsFromPrefs() {
  uint32_t magic = prefs.getUInt("cal_magic", 0);
  defaults_v1 = prefs.getFloat("cal_v1", 0.0f);
  defaults_v2 = prefs.getFloat("cal_v2", 0.0f);

  defaults_valid = (magic == 0xA55A1234u);

  // Validate range (ECU-level; after divider it must be <= 3.3/2)
  float max_ecu_v = 3.3f / OUTPUT_DIVIDER_GAIN;
  if (!defaults_valid || defaults_v1 < 0.0f || defaults_v2 < 0.0f || defaults_v1 > max_ecu_v || defaults_v2 > max_ecu_v) {
    defaults_valid = false;
    defaults_v1 = DEFAULT_SIGNAL_S1_V;
    defaults_v2 = DEFAULT_SIGNAL_S2_V;
    error_flags |= ERR_CAL_INVALID;
  }
}

static void saveDefaultsToPrefs(float v1, float v2) {
  prefs.putUInt("cal_magic", 0xA55A1234u);
  prefs.putFloat("cal_v1", v1);
  prefs.putFloat("cal_v2", v2);
  prefs.putUInt("cal_count", prefs.getUInt("cal_count", 0) + 1);
  defaults_valid = true;
  defaults_v1 = v1;
  defaults_v2 = v2;
  error_flags &= ~ERR_CAL_INVALID;
}

static void loadConfigFromPrefs() {
  cfg.can_baud = prefs.getUInt("can_baud", CAN_BAUD_DEFAULT);
  cfg.rs232_baud = prefs.getUInt("rs232_baud", RS232_BAUD_DEFAULT);

  cfg.use_obd = prefs.getBool("use_obd", true);
  cfg.obd_request_interval_ms = prefs.getUInt("obd_int", 100);
  cfg.obd_request_id = (uint16_t)prefs.getUShort("obd_req", 0x7DF);
  cfg.obd_response_id = (uint16_t)prefs.getUShort("obd_rsp", 0x7E8);
  cfg.obd_pid_speed = (uint8_t)prefs.getUChar("obd_pid", 0x0D);

  cfg.speed_frame_id = (uint16_t)prefs.getUShort("spd_id", 0x000);
  cfg.speed_byte_index = (uint8_t)prefs.getUChar("spd_idx", 0);

  cfg.speed_limit_kmh = (uint16_t)prefs.getUShort("limit_kmh", SPEED_LIMIT_DEFAULT_KMH);
  cfg.speed_cal_factor = prefs.getFloat("spd_fac", 1.0f);
  cfg.speed_cal_offset = prefs.getFloat("spd_off", 0.0f);
  cfg.hysteresis_kmh = (uint8_t)prefs.getUChar("hyst_kmh", 0);

  cfg.output_ramp_ms = (uint16_t)prefs.getUShort("ramp_ms", 0);

  cfg.mode_switch_active_low = prefs.getBool("mode_al", true);
  cfg.io18_mirror_limiter = prefs.getBool("io18_m", true);
  cfg.io18_manual_state = prefs.getBool("io18_s", false);
}

static void factoryResetPrefs() {
  prefs.clear();
  cfg = Config();
  defaults_valid = false;
  defaults_v1 = defaults_v2 = 0.0f;
  error_flags = ERR_NONE;
}

// =============================================================================
// CAN speed (OBD or broadcast)
// =============================================================================
static const uint8_t OBD_PID_RPM = 0x0C;
static const uint32_t OBD_RPM_INTERVAL_MS = 200;

static void sendObdRequestPid(uint8_t pid) {
  CAN_FRAME tx;
  tx.rtr = 0;
  tx.id = cfg.obd_request_id;
  tx.extended = (cfg.obd_request_id > 0x7FF);
  tx.length = 8;
  tx.data.uint8[0] = 0x02;
  tx.data.uint8[1] = 0x01;
  tx.data.uint8[2] = pid;
  tx.data.uint8[3] = 0x00;
  tx.data.uint8[4] = 0x00;
  tx.data.uint8[5] = 0x00;
  tx.data.uint8[6] = 0x00;
  tx.data.uint8[7] = 0x00;
  CAN0.sendFrame(tx);

  if (pid == cfg.obd_pid_speed) obd_tx_speed_count++;
  else if (pid == OBD_PID_RPM) obd_tx_rpm_count++;
}

static void sendObdSpeedRequest() { sendObdRequestPid(cfg.obd_pid_speed); }
static void sendObdRpmRequest() { sendObdRequestPid(OBD_PID_RPM); }

static void onCanRx(CAN_FRAME *frame) {
  if (!frame) return;

  can_rx_total++;
  can_rx_last_ms = millis();
  can_rx_last_id = frame->id;
  can_rx_last_ext = frame->extended ? true : false;

  if (cfg.use_obd) {
    if (frame->length < 4) return;
    if (frame->data.byte[1] != 0x41) return;

    uint8_t pid = frame->data.byte[2];

    // Accept OBD responses from:
    // - the configured response ID (exact match), OR
    // - any standard 11-bit ECU response ID 0x7E8..0x7EF (common in ISO 15765-4 11-bit OBD)
    // This avoids "no data" when the vehicle responds from 0x7E9/0x7EA/etc.
    bool rspid_ok = false;
    if (frame->id == cfg.obd_response_id) rspid_ok = true;
    else if (!frame->extended && (frame->id >= 0x7E8 && frame->id <= 0x7EF)) rspid_ok = true;

    if (!rspid_ok) {
      obd_rx_unmatched_id++;
      return;
    }

    // Speed (PID 0x0D): A = km/h
    if (pid == cfg.obd_pid_speed) {
      raw_speed_kmh = frame->data.byte[3];
      last_speed_rx_ms = millis();
      obd_rx_speed_count++;
      return;
    }

    // RPM (PID 0x0C): RPM = ((A*256) + B) / 4
    if (pid == OBD_PID_RPM) {
      if (frame->length < 5) return;
      uint16_t v = ((uint16_t)frame->data.byte[3] << 8) | (uint16_t)frame->data.byte[4];
      raw_rpm = (uint16_t)(v / 4u);
      last_rpm_rx_ms = millis();
      obd_rx_rpm_count++;
      return;
    }

    obd_rx_other_count++;
    return;
  }

  // Broadcast speed mode
  if (cfg.speed_frame_id == 0) return;
  if (frame->id != cfg.speed_frame_id) return;
  if (cfg.speed_byte_index >= frame->length) return;
  raw_speed_kmh = frame->data.byte[cfg.speed_byte_index];
  last_speed_rx_ms = millis();
}

static void setupCAN() {
  CAN0.setCANPins((gpio_num_t)CAN_RX_PIN, (gpio_num_t)CAN_TX_PIN);
  CAN0.begin(cfg.can_baud);
  CAN0.watchFor();
  CAN0.setCallback(0, onCanRx);
}

// =============================================================================
// Mode + Limiter logic
// =============================================================================
static OperatingMode readMode() {
  bool pin_high = (digitalRead(MODE_SWITCH_PIN) == HIGH);
  if (cfg.mode_switch_active_low) {
    return pin_high ? OperatingMode::Automatic : OperatingMode::Manual;
  }
  return pin_high ? OperatingMode::Manual : OperatingMode::Automatic;
}

static bool speedIsValid(uint32_t now_ms) {
  if (last_speed_rx_ms == 0) return false;
  return (now_ms - last_speed_rx_ms) <= SPEED_TIMEOUT_MS;
}

static bool rpmIsValid(uint32_t now_ms) {
  if (last_rpm_rx_ms == 0) return false;
  return (now_ms - last_rpm_rx_ms) <= SPEED_TIMEOUT_MS;
}

static float calibratedSpeedKmh() {
  return ((float)raw_speed_kmh * cfg.speed_cal_factor) + cfg.speed_cal_offset;
}

static bool updateLimiterState(bool speed_valid, float spd_kmh) {
  // SRD strict: activate when speed >= limit, deactivate when speed < limit
  // Optional hysteresis prevents oscillation (default = 0 to match SRD).
  if (!speed_valid) {
    limiter_active = false;
    return false;
  }

  if (!limiter_active) {
    if (spd_kmh >= (float)cfg.speed_limit_kmh) limiter_active = true;
  } else {
    float off_th = (float)cfg.speed_limit_kmh;
    if (cfg.hysteresis_kmh > 0) off_th -= (float)cfg.hysteresis_kmh;
    if (spd_kmh < off_th) limiter_active = false;
  }
  return limiter_active;
}

static void setRelay(bool on) {
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

static void updateIo18(bool limiter_on) {
  if (cfg.io18_mirror_limiter) {
    digitalWrite(DIG_OUT_PIN, limiter_on ? HIGH : LOW);
    return;
  }
  digitalWrite(DIG_OUT_PIN, cfg.io18_manual_state ? HIGH : LOW);
}

// =============================================================================
// Output generation
// =============================================================================
static void outputVoltagesInternal(float internal_v1, float internal_v2) {
  internal_v1 = clampf(internal_v1, 0.0f, 3.3f);
  internal_v2 = clampf(internal_v2, 0.0f, 3.3f);

#if USE_DAC_OUTPUT
  // DAC mode: True analog output (8-bit: 0-255 for 0-3.3V)
  // ESP32 DAC is 8-bit, so resolution is 3.3V / 255 = ~12.9 mV per step
  uint8_t dac1 = (uint8_t)((internal_v1 / 3.3f) * 255.0f);
  uint8_t dac2 = (uint8_t)((internal_v2 / 3.3f) * 255.0f);
  dac_output_voltage(DAC_CHANNEL_1, dac1);
  dac_output_voltage(DAC_CHANNEL_2, dac2);
#else
  // PWM mode: Digital PWM output (10-bit: 0-1023 for 0-3.3V)
  // Requires external RC filter for smooth analog signal
  uint32_t duty1 = (uint32_t)((internal_v1 / 3.3f) * (float)PWM_MAX_DUTY);
  uint32_t duty2 = (uint32_t)((internal_v2 / 3.3f) * (float)PWM_MAX_DUTY);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty1);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty2);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
#endif
}

static void outputEcuVoltages(float ecu_v1, float ecu_v2) {
  float max_ecu_v = 3.3f / OUTPUT_DIVIDER_GAIN;
  ecu_v1 = clampf(ecu_v1, 0.0f, max_ecu_v);
  ecu_v2 = clampf(ecu_v2, 0.0f, max_ecu_v);

  float internal_v1 = ecu_v1 * OUTPUT_DIVIDER_GAIN;
  float internal_v2 = ecu_v2 * OUTPUT_DIVIDER_GAIN;
  outputVoltagesInternal(internal_v1, internal_v2);
}

static void updateOutputs(OperatingMode mode, bool limiter_on, uint32_t now_ms, uint32_t dt_ms) {
  // Pedal sampling target: 1 kHz (SRD). Pots are sampled slower.
  static uint32_t last_pedal_ms = 0;
  static uint32_t last_pot_ms = 0;

  if ((uint32_t)(now_ms - last_pedal_ms) >= 1) {
    last_pedal_ms = now_ms;

    // Pedal (ADC1)
    float aps2_v = readVoltageAdc1(APS2_CH); // GPIO36
    float aps1_v = readVoltageAdc1(APS1_CH); // GPIO39

    if (isnan(aps1_v) || isnan(aps2_v)) {
      error_flags |= ERR_ADC_READ;
    } else {
      pedal_v1 = aps1_v * PEDAL_SCALE;
      pedal_v2 = aps2_v * PEDAL_SCALE;
    }
  }

  if ((uint32_t)(now_ms - last_pot_ms) >= 20) { // 50 Hz for pots is sufficient
    last_pot_ms = now_ms;

    // Pots (ADC2) for Mode 1
    float p1_v = readVoltageAdc2(POT1_CH);
    float p2_v = readVoltageAdc2(POT2_CH);
    if (!isnan(p1_v)) pot_v1 = p1_v;
    if (!isnan(p2_v)) pot_v2 = p2_v;
  }

  // Determine target ECU-level outputs
  float target_v1 = 0.0f;
  float target_v2 = 0.0f;

  if (mode == OperatingMode::Manual) {
    // Manual: outputs track potentiometers for calibration/measurement (always)
    target_v1 = pot_v1;
    target_v2 = pot_v2;

    // If pots appear not connected, fall back to SRD default values (350mV, 700mV)
    if (target_v1 < 0.05f && target_v2 < 0.05f) {
      target_v1 = DEFAULT_SIGNAL_S1_V;
      target_v2 = DEFAULT_SIGNAL_S2_V;
    }
  } else {
    // Automatic: outputs track pedal when below limit, default-calibration when limiting
    if (limiter_on) {
      if (defaults_valid) {
        target_v1 = defaults_v1;
        target_v2 = defaults_v2;
      } else {
        // SRD requires using default values if calibration is missing/invalid
        target_v1 = DEFAULT_SIGNAL_S1_V;
        target_v2 = DEFAULT_SIGNAL_S2_V;
      }
    } else {
      target_v1 = pedal_v1;
      target_v2 = pedal_v2;
    }
  }

  // Optional ramping
  if (cfg.output_ramp_ms == 0 || dt_ms == 0) {
    out_v1 = target_v1;
    out_v2 = target_v2;
  } else {
    float alpha = (float)dt_ms / (float)cfg.output_ramp_ms;
    if (alpha > 1.0f) alpha = 1.0f;
    out_v1 = out_v1 + (target_v1 - out_v1) * alpha;
    out_v2 = out_v2 + (target_v2 - out_v2) * alpha;
  }

  outputEcuVoltages(out_v1, out_v2);
}

// =============================================================================
// Calibration (Mode 2)
// =============================================================================
static void calibrateDefaults() {
  // Average multiple samples and require stability (basic "pedal at zero position" guard)
  const uint8_t samples = 20;
  const uint32_t delay_ms = 10;
  const float stable_tol = 0.03f; // 30mV tolerance across sampling window

  float min1 = 1000.0f, max1 = -1000.0f, sum1 = 0.0f;
  float min2 = 1000.0f, max2 = -1000.0f, sum2 = 0.0f;

  for (uint8_t i = 0; i < samples; i++) {
    float aps2_v = readVoltageAdc1(APS2_CH);
    float aps1_v = readVoltageAdc1(APS1_CH);

    if (isnan(aps1_v) || isnan(aps2_v)) {
      error_flags |= ERR_ADC_READ;
      logBoth("Calibration failed: ADC read error");
      buzzerOnFreq(400); delay(200); buzzerOff();
      return;
    }

    float v1 = aps1_v * PEDAL_SCALE;
    float v2 = aps2_v * PEDAL_SCALE;

    sum1 += v1; sum2 += v2;
    if (v1 < min1) min1 = v1;
    if (v1 > max1) max1 = v1;
    if (v2 < min2) min2 = v2;
    if (v2 > max2) max2 = v2;

    esp_task_wdt_reset();
    delay(delay_ms);
  }

  if ((max1 - min1) > stable_tol || (max2 - min2) > stable_tol) {
    logBoth("Calibration rejected: pedal signals not stable (ensure pedal at rest)");
    buzzerOnFreq(800); delay(200); buzzerOff();
    return;
  }

  float avg1 = sum1 / (float)samples;
  float avg2 = sum2 / (float)samples;

  saveDefaultsToPrefs(avg1, avg2);
  logBothF("Calibration saved (Mode2 defaults): V1=%.3f V, V2=%.3f V\n", avg1, avg2);

  // Confirmation tones
  buzzerOnFreq(2000); delay(100); buzzerOff();
  delay(150);
  buzzerOnFreq(2500); delay(100); buzzerOff();
}

// =============================================================================
// CLI (USB + RS232)
// =============================================================================
static void printHelp(Stream &out) {
  out.println("Commands:");
  out.println("  help");
  out.println("  status");
  out.println("  stream <on|off> [ms]         (USB: periodic one-line status output)");
  out.println("  get limit                  (show current speed limit)");
  out.println("  set limit <kmh>            (change limit; persists)");
  out.println("  set factor <float>");
  out.println("  set offset <float>");
  out.println("  set hyst <kmh>              (0 disables)");
  out.println("  set ramp <ms>               (0 disables)");
  out.println("  set canbaud <250000|500000> (reboot recommended)");
  out.println("  set rs232baud <baud>        (reboot recommended)");
  out.println("  set modepol <low|high>      (low: LOW=Manual, high: HIGH=Manual)");
  out.println("  set src <obd|broadcast>");
  out.println("  set obd reqid <0x7DF>");
  out.println("  set obd rspid <0x7E8>");
  out.println("  set obd pid <0x0D>");
  out.println("  set obd interval <ms>");
  out.println("  set spd id <0x123>          (broadcast mode)");
  out.println("  set spd idx <0..7>          (broadcast mode)");
  out.println("  io18 mirror <on|off>");
  out.println("  io18 set <0|1>              (only if mirror=off)");
  out.println("  calibrate                   (Mode2 only; same as button)");
  out.println("  factory_reset               (clears NVM config+calibration)");
}

static void printStatus(Stream &out) {
  uint32_t now = millis();
  OperatingMode mode = readMode();
  bool speed_valid = speedIsValid(now);
  float spd = calibratedSpeedKmh();
  bool rpm_valid = rpmIsValid(now);
  uint16_t rpm = raw_rpm;

  out.printf("Mode: %s\n", mode == OperatingMode::Manual ? "Mode1-Manual" : "Mode2-Automatic");
  out.printf("Speed: raw=%u km/h | cal=%.2f km/h | valid=%s | age=%lu ms\n",
             (unsigned)raw_speed_kmh, spd, speed_valid ? "YES" : "NO",
             (unsigned long)(speed_valid ? (now - last_speed_rx_ms) : 0));
  out.printf("RPM: %u | valid=%s | age=%lu ms\n",
             (unsigned)rpm,
             rpm_valid ? "YES" : "NO",
             (unsigned long)(rpm_valid ? (now - last_rpm_rx_ms) : 0));
  out.printf("Limit: %u km/h | hysteresis=%u km/h | limiter_active=%s | relay=%s\n",
             (unsigned)cfg.speed_limit_kmh, (unsigned)cfg.hysteresis_kmh,
             limiter_active ? "ON" : "OFF",
             digitalRead(RELAY_PIN) ? "ON" : "OFF");
  out.printf("Relay override: %s\n",
             relay_override == RelayOverride::Auto ? "AUTO" :
             (relay_override == RelayOverride::ForceOn ? "FORCE_ON" : "FORCE_OFF"));
  out.printf("SpeedCal: factor=%.4f offset=%.2f\n", cfg.speed_cal_factor, cfg.speed_cal_offset);
  out.printf("Signals (ECU-level V): pedal(V1=%.3f,V2=%.3f) pot(V1=%.3f,V2=%.3f)\n", pedal_v1, pedal_v2, pot_v1, pot_v2);
  out.printf("Outputs (ECU-level V): out(V1=%.3f,V2=%.3f)  ramp=%u ms\n", out_v1, out_v2, (unsigned)cfg.output_ramp_ms);
  out.printf("Output method: %s (SDPS1=IO%u, SDPS2=IO%u)\n",
#if USE_DAC_OUTPUT
             "DAC (true analog)",
#else
             "PWM (requires RC filter)",
#endif
             SDPS1_PIN, SDPS2_PIN);
  out.printf("Defaults (Mode2): valid=%s V1=%.3f V2=%.3f\n", defaults_valid ? "YES" : "NO", defaults_v1, defaults_v2);
  out.printf("CAN: baud=%lu src=%s (OBD req=0x%03X rsp=0x%03X pid=0x%02X interval=%lu ms) | broadcast(id=0x%03X idx=%u)\n",
             (unsigned long)cfg.can_baud,
             cfg.use_obd ? "OBD" : "BROADCAST",
             cfg.obd_request_id, cfg.obd_response_id, cfg.obd_pid_speed,
             (unsigned long)cfg.obd_request_interval_ms,
             cfg.speed_frame_id, (unsigned)cfg.speed_byte_index);
  out.printf("RS232: baud=%lu TX=%u RX=%u\n", (unsigned long)cfg.rs232_baud, RS232_TX_PIN, RS232_RX_PIN);
  out.printf("IO13(mode switch)=%u | IO18=%u (mirror=%s)\n",
             digitalRead(MODE_SWITCH_PIN),
             digitalRead(DIG_OUT_PIN),
             cfg.io18_mirror_limiter ? "ON" : "OFF");
  out.printf("Errors: 0x%08lX\n", (unsigned long)error_flags);
}

static void usbStatusStreamUpdate(uint32_t now_ms, OperatingMode mode, bool speed_valid, float spd_kmh, bool limiter_on) {
  if (!usb_stream_enabled) return;
  if ((uint32_t)(now_ms - usb_stream_last_ms) < usb_stream_interval_ms) return;
  usb_stream_last_ms = now_ms;

  bool rpm_valid = rpmIsValid(now_ms);
  uint16_t rpm = raw_rpm;

  // Compact one-line status for USB monitoring/logging
  Serial.print("STAT ");
  Serial.print("mode=");
  Serial.print(mode == OperatingMode::Manual ? "M1" : "M2");

  Serial.print(" speed=");
  Serial.print(spd_kmh, 1);
  Serial.print("kmh");
  if (!speed_valid) Serial.print("(stale)");

  Serial.print(" rpm=");
  if (rpm_valid) Serial.print((unsigned)rpm);
  else Serial.print("NA");

  Serial.print(" in_aps=(");
  Serial.print(pedal_v1, 3); Serial.print(",");
  Serial.print(pedal_v2, 3); Serial.print(")");

  Serial.print(" in_pot=(");
  Serial.print(pot_v1, 3); Serial.print(",");
  Serial.print(pot_v2, 3); Serial.print(")");

  Serial.print(" out=(");
  Serial.print(out_v1, 3); Serial.print(",");
  Serial.print(out_v2, 3); Serial.print(")");

  Serial.print(" relay=");
  Serial.print((digitalRead(RELAY_PIN) == HIGH) ? 1 : 0);

  Serial.print(" limiter=");
  Serial.print(limiter_on ? 1 : 0);

  Serial.print(" lim=");
  Serial.print((unsigned)cfg.speed_limit_kmh);
  Serial.print("kmh");

  // Basic CAN/OBD debug so we can quickly see if the bus is alive
  Serial.print(" can_rx=");
  Serial.print((unsigned long)can_rx_total);
  if (can_rx_total > 0) {
    Serial.print(" last_id=0x");
    Serial.print((unsigned long)can_rx_last_id, HEX);
    Serial.print(can_rx_last_ext ? "x" : "");
    Serial.print(" age=");
    Serial.print((unsigned long)(now_ms - can_rx_last_ms));
    Serial.print("ms");
  }

  Serial.print(" obd_tx(s=");
  Serial.print((unsigned long)obd_tx_speed_count);
  Serial.print(",r=");
  Serial.print((unsigned long)obd_tx_rpm_count);
  Serial.print(")");

  Serial.print(" obd_rx(s=");
  Serial.print((unsigned long)obd_rx_speed_count);
  Serial.print(",r=");
  Serial.print((unsigned long)obd_rx_rpm_count);
  Serial.print(",other=");
  Serial.print((unsigned long)obd_rx_other_count);
  Serial.print(",badid=");
  Serial.print((unsigned long)obd_rx_unmatched_id);
  Serial.print(")");

  Serial.println();
}

static void processCommand(char *line, Stream &out) {
  if (!line) return;
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  char *cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "help")) { printHelp(out); return; }
  if (!strcasecmp(cmd, "status")) { printStatus(out); return; }

  if (!strcasecmp(cmd, "get")) {
    char *key = strtok(NULL, " \t");
    if (!key) { out.println("usage: get <limit>"); return; }
    if (!strcasecmp(key, "limit")) {
      out.printf("Speed limit: %u km/h\n", (unsigned)cfg.speed_limit_kmh);
      return;
    }
    out.println("usage: get <limit>");
    return;
  }

  if (!strcasecmp(cmd, "stream")) {
    char *sub = strtok(NULL, " \t");
    if (!sub) {
      out.printf("stream: %s interval=%lu ms\n",
                 usb_stream_enabled ? "ON" : "OFF",
                 (unsigned long)usb_stream_interval_ms);
      return;
    }

    if (!strcasecmp(sub, "on")) {
      usb_stream_enabled = true;
      char *ms = strtok(NULL, " \t");
      if (ms) {
        long v = strtol(ms, NULL, 10);
        if (v < 50) v = 50;
        if (v > 5000) v = 5000;
        usb_stream_interval_ms = (uint32_t)v;
      }
      out.println("stream enabled (USB)");
      return;
    }

    if (!strcasecmp(sub, "off")) {
      usb_stream_enabled = false;
      out.println("stream disabled (USB)");
      return;
    }

    // Allow: stream <ms>
    long v = strtol(sub, NULL, 10);
    if (v > 0) {
      if (v < 50) v = 50;
      if (v > 5000) v = 5000;
      usb_stream_interval_ms = (uint32_t)v;
      usb_stream_enabled = true;
      out.println("stream interval updated (USB)");
      return;
    }

    out.println("usage: stream <on|off> [ms]  OR  stream <ms>");
    return;
  }

  if (!strcasecmp(cmd, "relay")) {
    char *v = strtok(NULL, " \t");
    if (!v) { out.println("usage: relay <auto|on|off>"); return; }
    if (!strcasecmp(v, "auto")) relay_override = RelayOverride::Auto;
    else if (!strcasecmp(v, "on")) relay_override = RelayOverride::ForceOn;
    else if (!strcasecmp(v, "off")) relay_override = RelayOverride::ForceOff;
    else { out.println("usage: relay <auto|on|off>"); return; }
    out.println("relay override updated");
    return;
  }

  if (!strcasecmp(cmd, "factory_reset")) {
    factoryResetPrefs();
    out.println("Factory reset done (reboot recommended)");
    return;
  }

  if (!strcasecmp(cmd, "calibrate")) {
    if (readMode() != OperatingMode::Automatic) {
      out.println("calibrate only allowed in Mode2-Automatic");
      return;
    }
    calibrateDefaults();
    return;
  }

  if (!strcasecmp(cmd, "io18")) {
    char *sub = strtok(NULL, " \t");
    if (!sub) { out.println("usage: io18 mirror <on|off> | io18 set <0|1>"); return; }

    if (!strcasecmp(sub, "mirror")) {
      char *val = strtok(NULL, " \t");
      bool b = false;
      if (!parseBool(val, &b)) { out.println("usage: io18 mirror <on|off>"); return; }
      cfg.io18_mirror_limiter = b;
      prefs.putBool("io18_m", cfg.io18_mirror_limiter);
      out.println("io18 mirror updated");
      return;
    }

    if (!strcasecmp(sub, "set")) {
      char *val = strtok(NULL, " \t");
      bool b = false;
      if (!parseBool(val, &b)) { out.println("usage: io18 set <0|1>"); return; }
      cfg.io18_manual_state = b;
      prefs.putBool("io18_s", cfg.io18_manual_state);
      out.println("io18 state updated");
      return;
    }

    out.println("usage: io18 mirror <on|off> | io18 set <0|1>");
    return;
  }

  if (!strcasecmp(cmd, "set")) {
    char *key = strtok(NULL, " \t");
    if (!key) { out.println("usage: set <limit|factor|offset|hyst|ramp|canbaud|rs232baud|modepol|src|obd|spd> ..."); return; }

    if (!strcasecmp(key, "canbaud")) {
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("usage: set canbaud <250000|500000>"); return; }
      uint32_t baud = (uint32_t)strtoul(v, NULL, 10);
      if (!(baud == 250000 || baud == 500000)) { out.println("canbaud must be 250000 or 500000"); return; }
      cfg.can_baud = baud;
      prefs.putUInt("can_baud", cfg.can_baud);
      out.println("CAN baud updated (reboot recommended)");
      return;
    }

    if (!strcasecmp(key, "rs232baud")) {
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("usage: set rs232baud <baud>"); return; }
      uint32_t baud = (uint32_t)strtoul(v, NULL, 10);
      if (baud < 1200 || baud > 1000000) { out.println("invalid rs232 baud"); return; }
      cfg.rs232_baud = baud;
      prefs.putUInt("rs232_baud", cfg.rs232_baud);
      out.println("RS232 baud updated (reboot recommended)");
      return;
    }

    if (!strcasecmp(key, "modepol")) {
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("usage: set modepol <low|high>"); return; }
      if (!strcasecmp(v, "low")) cfg.mode_switch_active_low = true;
      else if (!strcasecmp(v, "high")) cfg.mode_switch_active_low = false;
      else { out.println("usage: set modepol <low|high>"); return; }
      prefs.putBool("mode_al", cfg.mode_switch_active_low);
      out.println("Mode switch polarity updated");
      return;
    }

    if (!strcasecmp(key, "limit")) {
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("usage: set limit <kmh>"); return; }
      long kmh = strtol(v, NULL, 10);
      if (kmh < 0 || kmh > 250) { out.println("invalid limit (0..250)"); return; }
      cfg.speed_limit_kmh = (uint16_t)kmh;
      prefs.putUShort("limit_kmh", cfg.speed_limit_kmh);
      out.println("limit updated");
      return;
    }

    if (!strcasecmp(key, "factor")) {
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("usage: set factor <float>"); return; }
      cfg.speed_cal_factor = strtof(v, NULL);
      prefs.putFloat("spd_fac", cfg.speed_cal_factor);
      out.println("speed factor updated");
      return;
    }

    if (!strcasecmp(key, "offset")) {
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("usage: set offset <float>"); return; }
      cfg.speed_cal_offset = strtof(v, NULL);
      prefs.putFloat("spd_off", cfg.speed_cal_offset);
      out.println("speed offset updated");
      return;
    }

    if (!strcasecmp(key, "hyst")) {
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("usage: set hyst <kmh>"); return; }
      long kmh = strtol(v, NULL, 10);
      if (kmh < 0 || kmh > 20) { out.println("invalid hyst (0..20)"); return; }
      cfg.hysteresis_kmh = (uint8_t)kmh;
      prefs.putUChar("hyst_kmh", cfg.hysteresis_kmh);
      out.println("hysteresis updated");
      return;
    }

    if (!strcasecmp(key, "ramp")) {
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("usage: set ramp <ms>"); return; }
      long ms = strtol(v, NULL, 10);
      if (ms < 0 || ms > 5000) { out.println("invalid ramp (0..5000)"); return; }
      cfg.output_ramp_ms = (uint16_t)ms;
      prefs.putUShort("ramp_ms", cfg.output_ramp_ms);
      out.println("ramp updated");
      return;
    }

    if (!strcasecmp(key, "src")) {
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("usage: set src <obd|broadcast>"); return; }
      if (!strcasecmp(v, "obd")) cfg.use_obd = true;
      else if (!strcasecmp(v, "broadcast")) cfg.use_obd = false;
      else { out.println("usage: set src <obd|broadcast>"); return; }
      prefs.putBool("use_obd", cfg.use_obd);
      out.println("source updated (reboot recommended)");
      return;
    }

    if (!strcasecmp(key, "obd")) {
      char *sub = strtok(NULL, " \t");
      if (!sub) { out.println("usage: set obd <reqid|rspid|pid|interval> <value>"); return; }
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("missing value"); return; }

      if (!strcasecmp(sub, "reqid")) {
        cfg.obd_request_id = (uint16_t)strtoul(v, NULL, 0);
        prefs.putUShort("obd_req", cfg.obd_request_id);
        out.println("obd reqid updated");
        return;
      }
      if (!strcasecmp(sub, "rspid")) {
        cfg.obd_response_id = (uint16_t)strtoul(v, NULL, 0);
        prefs.putUShort("obd_rsp", cfg.obd_response_id);
        out.println("obd rspid updated");
        return;
      }
      if (!strcasecmp(sub, "pid")) {
        cfg.obd_pid_speed = (uint8_t)strtoul(v, NULL, 0);
        prefs.putUChar("obd_pid", cfg.obd_pid_speed);
        out.println("obd pid updated");
        return;
      }
      if (!strcasecmp(sub, "interval")) {
        cfg.obd_request_interval_ms = (uint32_t)strtoul(v, NULL, 10);
        if (cfg.obd_request_interval_ms < 50) cfg.obd_request_interval_ms = 50;
        prefs.putUInt("obd_int", cfg.obd_request_interval_ms);
        out.println("obd interval updated");
        return;
      }

      out.println("usage: set obd <reqid|rspid|pid|interval> <value>");
      return;
    }

    if (!strcasecmp(key, "spd")) {
      char *sub = strtok(NULL, " \t");
      if (!sub) { out.println("usage: set spd <id|idx> <value>"); return; }
      char *v = strtok(NULL, " \t");
      if (!v) { out.println("missing value"); return; }

      if (!strcasecmp(sub, "id")) {
        cfg.speed_frame_id = (uint16_t)strtoul(v, NULL, 0);
        prefs.putUShort("spd_id", cfg.speed_frame_id);
        out.println("speed frame id updated");
        return;
      }
      if (!strcasecmp(sub, "idx")) {
        long idx = strtol(v, NULL, 10);
        if (idx < 0 || idx > 7) { out.println("idx must be 0..7"); return; }
        cfg.speed_byte_index = (uint8_t)idx;
        prefs.putUChar("spd_idx", cfg.speed_byte_index);
        out.println("speed byte index updated");
        return;
      }

      out.println("usage: set spd <id|idx> <value>");
      return;
    }

    out.println("unknown set key");
    return;
  }

  out.println("unknown command (type: help)");
}

static void handleCli(Stream &io, CliBuf &cli) {
  while (io.available()) {
    char c = (char)io.read();
    if (c == '\r') continue;

    if (c == '\n') {
      cli.buf[cli.len] = '\0';
      processCommand(cli.buf, io);
      cli.len = 0;
      continue;
    }

    if (cli.len < (sizeof(cli.buf) - 1)) {
      cli.buf[cli.len++] = c;
    }
  }
}

// =============================================================================
// Watchdog
// =============================================================================
static void setupWatchdog() {
  // 3s watchdog
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms = 3000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_cfg);
#else
  esp_task_wdt_init(3, true);
#endif
  esp_task_wdt_add(NULL);
}

// =============================================================================
// Arduino entry points
// =============================================================================
void setup() {
  Serial.begin(115200);

  // Pins
  pinMode(CAL_BTN_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(DIG_OUT_PIN, OUTPUT);

  // Fail-safe: relay OFF (pedal connected) at boot
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(DIG_OUT_PIN, LOW);

  prefs.begin("speedLimiter", false);
  loadConfigFromPrefs();
  loadDefaultsFromPrefs();

  // Start RS232 after config is loaded
  Serial2.begin(cfg.rs232_baud, SERIAL_8N1, RS232_RX_PIN, RS232_TX_PIN);

  if (error_flags & ERR_CAL_INVALID) {
    logBothF("Warning: Mode2 calibration missing/invalid -> using defaults S1=%.3fV S2=%.3fV\n",
             DEFAULT_SIGNAL_S1_V, DEFAULT_SIGNAL_S2_V);
  }

  setupWatchdog();
  setupADC();
  setupOutputs();
  setupCAN();

  logBoth("Speed Limiter NEW firmware ready (SRD compliant)");
  logBoth("Type 'help' on USB Serial or RS232 for commands.");
}

void loop() {
  static uint32_t last_loop_ms = millis();
  uint32_t now = millis();
  uint32_t dt = now - last_loop_ms;
  last_loop_ms = now;

  esp_task_wdt_reset();

  // CLI
  handleCli(Serial, cli_usb);
  handleCli(Serial2, cli_rs232);

  // OBD request periodic
  static uint32_t last_obd_speed_ms = 0;
  static uint32_t last_obd_rpm_ms = 0;
  if (cfg.use_obd) {
    if ((now - last_obd_speed_ms) >= cfg.obd_request_interval_ms) {
      sendObdSpeedRequest();
      last_obd_speed_ms = now;
    }
    if ((now - last_obd_rpm_ms) >= OBD_RPM_INTERVAL_MS) {
      sendObdRpmRequest();
      last_obd_rpm_ms = now;
    }
  }

  // Speed validity / fail-safe
  bool spd_valid = speedIsValid(now);
  if (!spd_valid) error_flags |= ERR_CAN_TIMEOUT;
  else error_flags &= ~ERR_CAN_TIMEOUT;

  float spd_kmh = calibratedSpeedKmh();
  bool limiter_on = updateLimiterState(spd_valid, spd_kmh);

  // Mode selection
  OperatingMode mode = readMode();

  // Relay control (SRD + optional override)
  bool want_relay = (limiter_on && spd_valid);
  if (relay_override == RelayOverride::ForceOff) want_relay = false;
  if (relay_override == RelayOverride::ForceOn) want_relay = true;
  setRelay(want_relay);

  // Outputs (always generated; relay decides ECU connection)
  updateOutputs(mode, limiter_on && spd_valid, now, dt);

  // Buzzer + IO18
  buzzerUpdate(limiter_on && spd_valid);
  updateIo18(limiter_on && spd_valid);

  // USB status streaming
  usbStatusStreamUpdate(now, mode, spd_valid, spd_kmh, limiter_on && spd_valid);

  // Calibration button (Mode 2)
  static bool btn_last = HIGH;
  bool btn = digitalRead(CAL_BTN_PIN);
  if (btn_last == HIGH && btn == LOW) {
    delay(30); // debounce
    if (digitalRead(CAL_BTN_PIN) == LOW) {
      if (mode == OperatingMode::Automatic) calibrateDefaults();
      else logBoth("Calibration ignored (Mode1-Manual)");
    }
  }
  btn_last = btn;

  // Yield
  delay(1);
}

