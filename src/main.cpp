// =============================================================================
// Build selector
// =============================================================================
// BUILD_TEST_LOGGER = 0  -> Speed Limiter firmware (default)
// BUILD_TEST_LOGGER = 1  -> Logger firmware (see src/test_logger_fw.h)
#ifndef BUILD_TEST_LOGGER
#define BUILD_TEST_LOGGER 0
#endif

#if BUILD_TEST_LOGGER
#include "test_logger_fw.h"
#else

// =============================================================================
// Speed Limiter (minimal)
//
// Requirements implemented:
// - Speed limit stored in Preferences (NVS)
// - USB CLI command ONLY:  SL=<kmh>\r\n
// - Read CAN speed (default: OBD-II PID 0x0D response frames)
// - When speed reaches the limit: relay becomes ACTIVE, outputs initially equal APS inputs
// - While limiting: adapt APS outputs to keep speed near the limit
// - Release relay when pedal drops below captured-at-limit position by ~100 mV
//
// Notes:
// - No Mode1/Mode2, no RS232 CLI, no calibration, no buzzer, no extra config.
// - Speed source is fixed at compile-time (see SPEED_SOURCE_OBD below).
// =============================================================================

// Output method default (overridden by PlatformIO build flags)
#ifndef USE_DAC_OUTPUT
#define USE_DAC_OUTPUT 0
#endif

#include <Arduino.h>

#include <Preferences.h>
#include <esp32_can.h>

#include "driver/adc.h"
#include "driver/ledc.h"
#if USE_DAC_OUTPUT
#include "driver/dac.h"
#endif
#include "esp_adc_cal.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

// =============================================================================
// Pin assignments
// =============================================================================
static const int RELAY_PIN = 23;   // IO23 - relay control (HIGH = relay active)

static const int CAN_RX_PIN = 32;  // IO32
static const int CAN_TX_PIN = 33;  // IO33

static const int APS1_PIN = 39;    // IO39 - APS1 (ADC1_CH3)
static const int APS2_PIN = 36;    // IO36 - APS2 (ADC1_CH0)

#if USE_DAC_OUTPUT
static const int SDPS1_PIN = 25;   // IO25 - DAC1
static const int SDPS2_PIN = 26;   // IO26 - DAC2
#else
static const int SDPS1_PIN = 21;   // IO21 - PWM output
static const int SDPS2_PIN = 22;   // IO22 - PWM output
#endif

// =============================================================================
// Speed source selection (no runtime config)
// =============================================================================
// 1 = OBD-II speed response (PID 0x0D)   (default)
// 0 = broadcast frame (SPEED_FRAME_ID / SPEED_BYTE_INDEX)
#ifndef SPEED_SOURCE_OBD
#define SPEED_SOURCE_OBD 1
#endif

// Broadcast speed config if SPEED_SOURCE_OBD=0
#ifndef SPEED_FRAME_ID
#define SPEED_FRAME_ID 0x123
#endif
#ifndef SPEED_BYTE_INDEX
#define SPEED_BYTE_INDEX 0
#endif

// =============================================================================
// Constants
// =============================================================================
static const uint32_t CAN_BAUD = 500000;

// OBD-II speed PID request defaults (PID 0x0D)
static const uint16_t OBD_REQ_ID = 0x7DF;
static const uint8_t OBD_PID_SPEED = 0x0D;
static const uint32_t OBD_REQ_INTERVAL_MS = 100; // 10 Hz

// Speed validity timeout (failsafe: relay OFF)
static const uint32_t SPEED_TIMEOUT_MS = 500;

// Preferences
static const char *PREF_NS = "speedLimiter";
static const char *PREF_KEY_SL = "sl"; // speed limit in km/h
static const uint16_t SPEED_LIMIT_DEFAULT_KMH = 40;

// SRD correction formula: Actual = ADC * (69.6 / 47.5)
static const float PEDAL_SCALE = (69.6f / 47.5f);

// Output divider compensation: to generate 1V ECU-output, generate 2V internally
static const float OUTPUT_DIVIDER_GAIN = 2.0f;

// Minimum safe ECU-level values (floor)
static const float DEFAULT_SIGNAL_S1_V = 0.350f;
static const float DEFAULT_SIGNAL_S2_V = 0.700f;

// PWM settings
static const uint32_t PWM_FREQ_HZ = 5000;
static const ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_10_BIT; // 0..1023
static const uint32_t PWM_MAX_DUTY = 1023;

// Limiter behavior
static const uint16_t PEDAL_RELEASE_MARGIN_MV = 100; // ~100 mV below captured -> release relay
static const uint32_t CONTROL_INTERVAL_MS = 50;
static const float SPEED_DEADBAND_KMH = 0.3f;
static const float FACTOR_RATE_PER_KMH_PER_S = 0.06f; // tune to your vehicle

// =============================================================================
// Globals
// =============================================================================
static Preferences prefs;

static volatile uint8_t g_speed_kmh = 0;
static volatile uint32_t g_last_speed_ms = 0;

static esp_adc_cal_characteristics_t adc1_chars;
static bool adc1_ok = false;

static uint16_t speed_limit_kmh = SPEED_LIMIT_DEFAULT_KMH;

// Latest measured APS inputs (ECU-level volts)
static float pedal_v1 = DEFAULT_SIGNAL_S1_V;
static float pedal_v2 = DEFAULT_SIGNAL_S2_V;

// Latest generated outputs (ECU-level volts)
static float out_v1 = DEFAULT_SIGNAL_S1_V;
static float out_v2 = DEFAULT_SIGNAL_S2_V;

// Limiter state
static bool limiting = false;
static uint16_t captured_pedal_avg_mv = 0;
static float throttle_factor = 1.0f; // 1.0 = pass-through, <1 reduces APS outputs
static uint32_t last_control_ms = 0;

// USB CLI buffer
static char cli_buf[32];
static uint8_t cli_len = 0;

// =============================================================================
// Helpers
// =============================================================================
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void setRelayActive(bool active) {
  digitalWrite(RELAY_PIN, active ? HIGH : LOW);
}

static inline bool speedValid(uint32_t now_ms) {
  uint32_t last = g_last_speed_ms;
  if (last == 0) return false;
  return (now_ms - last) <= SPEED_TIMEOUT_MS;
}

static inline uint16_t mvFromVolts(float v) {
  if (v <= 0.0f) return 0;
  float mv = v * 1000.0f;
  if (mv > 65535.0f) mv = 65535.0f;
  return (uint16_t)(mv + 0.5f);
}

// =============================================================================
// ADC (APS inputs)
// =============================================================================
static const adc_bits_width_t ADC_WIDTH = ADC_WIDTH_BIT_12;
static const adc_atten_t ADC_ATTEN = ADC_ATTEN_DB_12; // ~0-3.3V
static const uint32_t DEFAULT_VREF_MV = 1100;

static const adc1_channel_t APS2_CH = ADC1_CHANNEL_0; // GPIO36
static const adc1_channel_t APS1_CH = ADC1_CHANNEL_3; // GPIO39

static void setupADC() {
  adc1_ok = false;
  if (adc1_config_width(ADC_WIDTH) != ESP_OK) return;
  if (adc1_config_channel_atten(APS1_CH, ADC_ATTEN) != ESP_OK) return;
  if (adc1_config_channel_atten(APS2_CH, ADC_ATTEN) != ESP_OK) return;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF_MV, &adc1_chars);
  adc1_ok = true;
}

static float readVoltageAdc1(adc1_channel_t ch) {
  if (!adc1_ok) return NAN;
  int raw = adc1_get_raw(ch);
  if (raw < 0) return NAN;
  uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc1_chars);
  return (float)mv / 1000.0f;
}

static void samplePedal() {
  float aps1 = readVoltageAdc1(APS1_CH);
  float aps2 = readVoltageAdc1(APS2_CH);
  if (!isnan(aps1)) pedal_v1 = aps1 * PEDAL_SCALE;
  if (!isnan(aps2)) pedal_v2 = aps2 * PEDAL_SCALE;
}

// =============================================================================
// Outputs (PWM/DAC)
// =============================================================================
static void setupOutputs() {
#if USE_DAC_OUTPUT
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  dac_output_voltage(DAC_CHANNEL_1, 0);
  dac_output_voltage(DAC_CHANNEL_2, 0);
#else
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
}

static void outputVoltagesInternal(float internal_v1, float internal_v2) {
  internal_v1 = clampf(internal_v1, 0.0f, 3.3f);
  internal_v2 = clampf(internal_v2, 0.0f, 3.3f);

#if USE_DAC_OUTPUT
  uint8_t dac1 = (uint8_t)((internal_v1 / 3.3f) * 255.0f);
  uint8_t dac2 = (uint8_t)((internal_v2 / 3.3f) * 255.0f);
  dac_output_voltage(DAC_CHANNEL_1, dac1);
  dac_output_voltage(DAC_CHANNEL_2, dac2);
#else
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

// =============================================================================
// CAN speed (OBD or broadcast)
// =============================================================================
static void sendObdSpeedRequest() {
  CAN_FRAME tx;
  tx.rtr = 0;
  tx.id = OBD_REQ_ID;
  tx.extended = (OBD_REQ_ID > 0x7FF);
  tx.length = 8;
  tx.data.uint8[0] = 0x02;
  tx.data.uint8[1] = 0x01;
  tx.data.uint8[2] = OBD_PID_SPEED;
  tx.data.uint8[3] = 0x00;
  tx.data.uint8[4] = 0x00;
  tx.data.uint8[5] = 0x00;
  tx.data.uint8[6] = 0x00;
  tx.data.uint8[7] = 0x00;
  CAN0.sendFrame(tx);
}

static void onCanRx(CAN_FRAME *frame) {
  if (!frame) return;
  uint32_t now = millis();

#if SPEED_SOURCE_OBD
  // Expect OBD positive response: [len, 0x41, 0x0D, speed_kmh, ...]
  if (frame->extended) return;
  if (frame->id < 0x7E8 || frame->id > 0x7EF) return;
  if (frame->length < 4) return;
  if (frame->data.byte[1] != 0x41) return;
  if (frame->data.byte[2] != OBD_PID_SPEED) return;

  g_speed_kmh = frame->data.byte[3];
  g_last_speed_ms = now;
#else
  if (frame->id != SPEED_FRAME_ID) return;
  if (SPEED_BYTE_INDEX >= frame->length) return;
  g_speed_kmh = frame->data.byte[SPEED_BYTE_INDEX];
  g_last_speed_ms = now;
#endif
}

static void setupCAN() {
  CAN0.setCANPins((gpio_num_t)CAN_RX_PIN, (gpio_num_t)CAN_TX_PIN);
  CAN0.begin(CAN_BAUD);
  CAN0.watchFor();
  CAN0.setCallback(0, onCanRx);
}

// =============================================================================
// USB CLI (only: SL=<kmh>)
// =============================================================================
static void applySpeedLimitFromCli(const char *line) {
  if (!line) return;

  while (*line == ' ' || *line == '\t') line++;

  if (strncasecmp(line, "SL", 2) != 0) return;
  line += 2;

  while (*line == ' ' || *line == '\t') line++;
  if (*line != '=') return;
  line++;

  long v = strtol(line, NULL, 10);
  if (v < 0 || v > 250) {
    Serial.println("ERR");
    return;
  }

  speed_limit_kmh = (uint16_t)v;
  prefs.putUShort(PREF_KEY_SL, speed_limit_kmh);
  Serial.printf("OK SL=%u\r\n", (unsigned)speed_limit_kmh);
}

static void handleUsbCli() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      cli_buf[cli_len] = '\0';
      if (cli_len > 0) applySpeedLimitFromCli(cli_buf);
      cli_len = 0;
      continue;
    }

    if (cli_len < (sizeof(cli_buf) - 1)) {
      cli_buf[cli_len++] = c;
    }
  }
}

// =============================================================================
// Arduino entry points
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(150);

  pinMode(RELAY_PIN, OUTPUT);
  setRelayActive(false); // fail-safe at boot

  prefs.begin(PREF_NS, false);
  speed_limit_kmh = prefs.getUShort(PREF_KEY_SL, SPEED_LIMIT_DEFAULT_KMH);

  setupADC();
  setupOutputs();
  setupCAN();

  Serial.printf("SpeedLimiter ready. SL=%u km/h\r\n", (unsigned)speed_limit_kmh);
  Serial.println("Set speed limit with: SL=<0..250>");
}

void loop() {
  uint32_t now = millis();

  handleUsbCli();

#if SPEED_SOURCE_OBD
  static uint32_t last_obd_ms = 0;
  if ((uint32_t)(now - last_obd_ms) >= OBD_REQ_INTERVAL_MS) {
    sendObdSpeedRequest();
    last_obd_ms = now;
  }
#endif

  // Sample pedal quickly to keep outputs aligned with inputs
  static uint32_t last_pedal_ms = 0;
  if ((uint32_t)(now - last_pedal_ms) >= 1) {
    last_pedal_ms = now;
    samplePedal();
  }

  bool spd_ok = speedValid(now);
  uint16_t spd = (uint16_t)g_speed_kmh;

  if (!spd_ok || speed_limit_kmh == 0) {
    // Fail-safe: if speed is unknown, do NOT keep relay active.
    limiting = false;
    throttle_factor = 1.0f;
    setRelayActive(false);
  } else if (!limiting) {
    // Not limiting yet: wait until speed reaches the threshold.
    if (spd >= speed_limit_kmh) {
      limiting = true;
      captured_pedal_avg_mv = mvFromVolts((pedal_v1 + pedal_v2) * 0.5f);
      throttle_factor = 1.0f; // at threshold: output == input
      last_control_ms = now;
      setRelayActive(true);
    } else {
      throttle_factor = 1.0f;
      setRelayActive(false);
    }
  } else {
    // Limiting active.
    uint16_t pedal_avg_mv = mvFromVolts((pedal_v1 + pedal_v2) * 0.5f);

    // Release when driver eases pedal below captured position by ~100mV.
    if (pedal_avg_mv + PEDAL_RELEASE_MARGIN_MV < captured_pedal_avg_mv) {
      limiting = false;
      throttle_factor = 1.0f;
      setRelayActive(false);
    } else {
      setRelayActive(true);

      // Adapt throttle_factor to keep speed near the threshold.
      if ((uint32_t)(now - last_control_ms) >= CONTROL_INTERVAL_MS) {
        float dt_s = (float)(now - last_control_ms) / 1000.0f;
        last_control_ms = now;

        float err = (float)spd - (float)speed_limit_kmh;
        if (fabsf(err) > SPEED_DEADBAND_KMH) {
          // err > 0 => too fast => reduce factor
          // err < 0 => too slow => increase factor (up to 1.0)
          throttle_factor -= err * FACTOR_RATE_PER_KMH_PER_S * dt_s;
          throttle_factor = clampf(throttle_factor, 0.0f, 1.0f);
        }
      }
    }
  }

  // Always generate outputs (keeps relay transitions smooth)
  float v1 = pedal_v1 * throttle_factor;
  float v2 = pedal_v2 * throttle_factor;

  // Floor to safe minimums (avoid going below typical idle voltages)
  if (v1 < DEFAULT_SIGNAL_S1_V) v1 = DEFAULT_SIGNAL_S1_V;
  if (v2 < DEFAULT_SIGNAL_S2_V) v2 = DEFAULT_SIGNAL_S2_V;

  out_v1 = v1;
  out_v2 = v2;
  outputEcuVoltages(out_v1, out_v2);

  delay(1);
}

#endif // BUILD_TEST_LOGGER

