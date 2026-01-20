#include "logic_module.h"
#include "shared_state.h"
#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>
#include <Preferences.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==========================================================
// APS table-based speed limiter
// ==========================================================
//
// - Start limiting at: activation_kmh = SL - SPEED_LIMIT_ACTIVATION_OFFSET_KMH (margin).
// - Cap APS_out by a speed->APS table:
//     cap_high = APS_SPEED_MAP(SL)
//     cap_low  = APS_SPEED_MAP(activation_kmh)  (never cut below this while limiting)
// - Use a smooth overshoot factor (0..1) between cap_low and cap_high based on
//   predicted overspeed (speed + d(speed)/dt * lookahead).
// - Never increase throttle: APS_out <= APS_in always.
// - Driver override: if APS_in drops below APS_out by PEDAL_RELEASE_MARGIN_MV -> relay OFF.
//
// ==========================================================

static constexpr uint32_t TELEMETRY_MS = 100;
static constexpr uint32_t LOGIC_LOOP_MS = 10;
static constexpr float APS_IDLE_MARGIN_V = 0.05f;

static TaskHandle_t g_logic_task = nullptr;

enum class LimiterState {
  PASS_THROUGH = 0,
  OVERSHOOT_CONTROL = 1,
  LIMIT_ACTIVE = 2,
  FAULT = 3,
};

static Preferences prefs;
static uint16_t g_speed_limit_kmh = SPEED_LIMIT_DEFAULT_KMH;

static LimiterState state = LimiterState::PASS_THROUGH;
static bool relay_active = false;
static bool limiter_active = false;

// Commanded output (ECU-level volts)
static float aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
static float aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;

// Overshoot factor (0..1): 1 -> cap_high, 0 -> cap_low
static float limit_factor = 1.0f;

// Speed derivative (filtered)
static uint8_t last_speed_kmh = 0;
static uint32_t last_speed_update_ms = 0;
static float speed_rate_kmh_s = 0.0f;

// Timing
static uint32_t last_control_ms = 0;
static uint32_t last_telemetry_ms = 0;

// USB CLI buffer (only: SL=<kmh>)
static char cli_buf[32];
static uint8_t cli_len = 0;

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float maxf(float a, float b) { return (a > b) ? a : b; }

static float mvToVolts(uint16_t mv) { return (float)mv / 1000.0f; }

// Keep both channels within the physical output range (preserve ratio).
// This avoids PWM/DAC saturation that can break APS1/APS2 correlation.
static void clampPairToMaxEcuV(float *v1, float *v2) {
  if (!v1 || !v2) return;

  float max_ecu_v = 3.3f;
  if (OUTPUT_DIVIDER_GAIN > 0.001f) max_ecu_v = 3.3f / OUTPUT_DIVIDER_GAIN;

  float peak = maxf(*v1, *v2);
  if (peak > max_ecu_v && peak > 0.0f) {
    float s = max_ecu_v / peak;
    *v1 *= s;
    *v2 *= s;
  }
}

static void setRelay(bool on) {
  relay_active = on;
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

static bool pedalReleased(float in1, float in2) {
  return (in1 <= (DEFAULT_SIGNAL_S1_V + APS_IDLE_MARGIN_V)) && (in2 <= (DEFAULT_SIGNAL_S2_V + APS_IDLE_MARGIN_V));
}

static bool driverRequestsLess(float in1, float in2) {
  float rel_margin_v = mvToVolts(PEDAL_RELEASE_MARGIN_MV);
  return ((in1 + rel_margin_v) < aps_cmd_v1) || ((in2 + rel_margin_v) < aps_cmd_v2);
}

static void applySpeedLimitFromCli(const char *line) {
  if (!line) return;

  while (*line == ' ' || *line == '\t') line++;

  // Case-insensitive "SL" prefix (avoid non-portable strncasecmp dependency).
  if (!((line[0] == 'S' || line[0] == 's') && (line[1] == 'L' || line[1] == 'l'))) return;
  line += 2;

  while (*line == ' ' || *line == '\t') line++;
  if (*line != '=') return;
  line++;

  long v = strtol(line, NULL, 10);
  if (v < 0 || v > 250) {
    Serial.println("ERR");
    return;
  }

  g_speed_limit_kmh = (uint16_t)v;
  prefs.putUShort(PREF_KEY_SL, g_speed_limit_kmh);
  Serial.printf("OK SL=%u\r\n", (unsigned)g_speed_limit_kmh);
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

static void updateSpeedDerivative(uint8_t speed_kmh, uint32_t speed_update_ms) {
  if (speed_update_ms == 0 || speed_update_ms == last_speed_update_ms) return;

  if (last_speed_update_ms != 0 && speed_update_ms > last_speed_update_ms) {
    float dt_s = (float)(speed_update_ms - last_speed_update_ms) / 1000.0f;
    if (dt_s > 0.001f) {
      float inst_rate = ((float)speed_kmh - (float)last_speed_kmh) / dt_s;
      speed_rate_kmh_s =
          (speed_rate_kmh_s * LIMIT_SPEED_RATE_LPF_ALPHA) + (inst_rate * (1.0f - LIMIT_SPEED_RATE_LPF_ALPHA));
    }
  }

  last_speed_kmh = speed_kmh;
  last_speed_update_ms = speed_update_ms;
}

static void lookupApsCapForSpeedKmh(uint16_t speed_kmh, float *out_v1, float *out_v2) {
  if (!out_v1 || !out_v2) return;

  if (APS_SPEED_MAP_LEN == 0) {
    *out_v1 = DEFAULT_SIGNAL_S1_V;
    *out_v2 = DEFAULT_SIGNAL_S2_V;
    return;
  }

  const ApsSpeedMapPoint &first = APS_SPEED_MAP[0];
  if (speed_kmh <= first.speed_kmh) {
    *out_v1 = mvToVolts(first.aps1_mv);
    *out_v2 = mvToVolts(first.aps2_mv);
    return;
  }

  const ApsSpeedMapPoint &last = APS_SPEED_MAP[APS_SPEED_MAP_LEN - 1];
  if (speed_kmh >= last.speed_kmh) {
    *out_v1 = mvToVolts(last.aps1_mv);
    *out_v2 = mvToVolts(last.aps2_mv);
    return;
  }

  for (uint8_t i = 0; (uint8_t)(i + 1) < APS_SPEED_MAP_LEN; i++) {
    const ApsSpeedMapPoint &a = APS_SPEED_MAP[i];
    const ApsSpeedMapPoint &b = APS_SPEED_MAP[i + 1];
    if (speed_kmh <= b.speed_kmh) {
      uint16_t s0 = a.speed_kmh;
      uint16_t s1 = b.speed_kmh;
      float t = 0.0f;
      if (s1 > s0) t = (float)(speed_kmh - s0) / (float)(s1 - s0);

      float aps1_mv = (float)a.aps1_mv + ((float)b.aps1_mv - (float)a.aps1_mv) * t;
      float aps2_mv = (float)a.aps2_mv + ((float)b.aps2_mv - (float)a.aps2_mv) * t;

      *out_v1 = aps1_mv / 1000.0f;
      *out_v2 = aps2_mv / 1000.0f;
      return;
    }
  }

  // Fallback (should never hit).
  *out_v1 = mvToVolts(last.aps1_mv);
  *out_v2 = mvToVolts(last.aps2_mv);
}

static void resetLimiter(uint32_t now_ms) {
  limiter_active = false;
  limit_factor = 1.0f;
  aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
  aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;
  last_control_ms = now_ms;
  speed_rate_kmh_s = 0.0f;
  last_speed_kmh = 0;
  last_speed_update_ms = 0;
}

void LogicModule_Begin() {
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false);

  prefs.begin(PREF_NS, false);
  g_speed_limit_kmh = prefs.getUShort(PREF_KEY_SL, SPEED_LIMIT_DEFAULT_KMH);

  uint32_t now = millis();
  resetLimiter(now);
  last_telemetry_ms = now;

  Serial.printf("LogicModule (APS-table) ready. SL=%u km/h\r\n", (unsigned)g_speed_limit_kmh);
  Serial.println("Set speed limit with: SL=<0..250>");
}

static void LogicModule_Update() {
  uint32_t now = millis();

  handleUsbCli();

  // Read APS inputs (ECU-level volts)
  float aps_in_v1 = DEFAULT_SIGNAL_S1_V;
  float aps_in_v2 = DEFAULT_SIGNAL_S2_V;
  uint32_t aps_ts = 0;
  SharedState_GetAps(&aps_in_v1, &aps_in_v2, &aps_ts);

  // Clamp inputs to safety floors.
  aps_in_v1 = maxf(aps_in_v1, DEFAULT_SIGNAL_S1_V);
  aps_in_v2 = maxf(aps_in_v2, DEFAULT_SIGNAL_S2_V);

  // Read speed (km/h) + validity.
  bool speed_valid = SharedState_SpeedValid(now, SPEED_TIMEOUT_MS);
  uint8_t speed_kmh = (uint8_t)g_speed_kmh;
  uint32_t speed_update_ms = g_speed_last_update_ms;

  updateSpeedDerivative(speed_kmh, speed_update_ms);

  // Caps derived from table.
  float cap_high_v1 = 0.0f, cap_high_v2 = 0.0f;
  float cap_low_v1 = 0.0f, cap_low_v2 = 0.0f;

  float predicted_speed_kmh = (float)speed_kmh + speed_rate_kmh_s * LIMIT_SPEED_LOOKAHEAD_S;

  // Fail-safe: invalid speed or disabled SL -> relay OFF + pass-through.
  if (!speed_valid || g_speed_limit_kmh == 0) {
    resetLimiter(now);
    setRelay(false);
    state = LimiterState::PASS_THROUGH;
    SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
    return;
  }

  // Start limiting at (SL - margin), release at (activation - hysteresis).
  uint16_t activation_kmh = g_speed_limit_kmh;
  if (g_speed_limit_kmh > SPEED_LIMIT_ACTIVATION_OFFSET_KMH) {
    activation_kmh = (uint16_t)(g_speed_limit_kmh - SPEED_LIMIT_ACTIVATION_OFFSET_KMH);
  }
  uint16_t release_kmh = 0;
  if (activation_kmh > SPEED_LIMIT_DEACTIVATION_HYSTERESIS_KMH) {
    release_kmh = (uint16_t)(activation_kmh - SPEED_LIMIT_DEACTIVATION_HYSTERESIS_KMH);
  }

  lookupApsCapForSpeedKmh(g_speed_limit_kmh, &cap_high_v1, &cap_high_v2);
  lookupApsCapForSpeedKmh(activation_kmh, &cap_low_v1, &cap_low_v2);
  clampPairToMaxEcuV(&cap_high_v1, &cap_high_v2);
  clampPairToMaxEcuV(&cap_low_v1, &cap_low_v2);

  if (!limiter_active) {
    // Pass-through (relay OFF)
    setRelay(false);
    state = LimiterState::PASS_THROUGH;
    SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
    aps_cmd_v1 = aps_in_v1;
    aps_cmd_v2 = aps_in_v2;
    limit_factor = 1.0f;

    // Arm limiter only when driver is not at idle (avoid relay toggling at cruise/idle).
    if (speed_kmh >= activation_kmh && !pedalReleased(aps_in_v1, aps_in_v2)) {
      limiter_active = true;
      setRelay(true);
      last_control_ms = now;

      // Start with cap_high (example: SL=50 => cap_high(50)=900/1800 mV).
      limit_factor = 1.0f;
      aps_cmd_v1 = (aps_in_v1 > cap_high_v1) ? cap_high_v1 : aps_in_v1;
      aps_cmd_v2 = (aps_in_v2 > cap_high_v2) ? cap_high_v2 : aps_in_v2;

      // Reset derivative baseline at activation to avoid stale cuts.
      speed_rate_kmh_s = 0.0f;
      last_speed_kmh = speed_kmh;
      last_speed_update_ms = speed_update_ms;

      SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
      state = LimiterState::LIMIT_ACTIVE;
    }
  } else {
    // Active limiting (relay ON)
    setRelay(true);

    // Speed-based release (hysteresis)
    if (speed_kmh <= release_kmh) {
      limiter_active = false;
      setRelay(false);
      state = LimiterState::PASS_THROUGH;
      SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
      aps_cmd_v1 = aps_in_v1;
      aps_cmd_v2 = aps_in_v2;
      limit_factor = 1.0f;
      last_control_ms = now;
      return;
    }

    // Driver override: immediate return control.
    if (driverRequestsLess(aps_in_v1, aps_in_v2) || pedalReleased(aps_in_v1, aps_in_v2)) {
      limiter_active = false;
      setRelay(false);
      state = LimiterState::PASS_THROUGH;
      SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
      aps_cmd_v1 = aps_in_v1;
      aps_cmd_v2 = aps_in_v2;
      limit_factor = 1.0f;
      last_control_ms = now;
      return;
    }

    // Apply control at fixed interval.
    if ((uint32_t)(now - last_control_ms) >= CONTROL_INTERVAL_MS) {
      float dt_s = (float)(now - last_control_ms) / 1000.0f;
      if (dt_s <= 0.0f) dt_s = 0.001f;
      last_control_ms = now;

      // Overspeed prediction based factor update.
      float overspeed_kmh = predicted_speed_kmh - (float)g_speed_limit_kmh; // + = predicted overspeed

      if (overspeed_kmh > 0.0f) {
        // Reduce quickly when overspeed is predicted.
        limit_factor -= FACTOR_RATE_DOWN_PER_KMH_PER_S * overspeed_kmh * dt_s;
      } else {
        // Relax slowly only when safely under the limit and not rising.
        bool safely_under = ((float)g_speed_limit_kmh - (float)speed_kmh) >= LIMIT_RELAX_BAND_KMH;
        bool not_rising = (speed_rate_kmh_s <= LIMIT_RELAX_MAX_RISE_KMH_PER_S);
        if (safely_under && not_rising) {
          limit_factor += FACTOR_RATE_UP_PER_S * dt_s;
        }
      }

      limit_factor = clampf(limit_factor, 0.0f, 1.0f);

      // Blend between cap_low and cap_high.
      float cap_v1 = cap_low_v1 + (cap_high_v1 - cap_low_v1) * limit_factor;
      float cap_v2 = cap_low_v2 + (cap_high_v2 - cap_low_v2) * limit_factor;

      // Never increase throttle: output is limited by driver demand AND cap.
      aps_cmd_v1 = (aps_in_v1 > cap_v1) ? cap_v1 : aps_in_v1;
      aps_cmd_v2 = (aps_in_v2 > cap_v2) ? cap_v2 : aps_in_v2;

      // Safety floors.
      aps_cmd_v1 = maxf(aps_cmd_v1, DEFAULT_SIGNAL_S1_V);
      aps_cmd_v2 = maxf(aps_cmd_v2, DEFAULT_SIGNAL_S2_V);

      SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
    } else {
      // Hold last command between control updates.
      SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
    }

    state = (predicted_speed_kmh > (float)g_speed_limit_kmh || limit_factor < 0.999f) ? LimiterState::OVERSHOOT_CONTROL
                                                                                      : LimiterState::LIMIT_ACTIVE;
  }

  // ==================================================
  // TELEMETRY (throttled)
  // ==================================================
  if ((uint32_t)(now - last_telemetry_ms) >= TELEMETRY_MS) {
    last_telemetry_ms = now;
    Serial.printf(
        "SPD=%.1f APSin=(%.2f,%.2f) APSout=(%.2f,%.2f) Relay=%d State=%d Accel=%.2f Factor=%.2f SL=%u Act=%u Caps=(%.2f,%.2f)->(%.2f,%.2f) Pred=%.1f\n",
        (float)speed_kmh,
        aps_in_v1,
        aps_in_v2,
        aps_cmd_v1,
        aps_cmd_v2,
        relay_active,
        (int)state,
        speed_rate_kmh_s,
        limit_factor,
        (unsigned)g_speed_limit_kmh,
        (unsigned)activation_kmh,
        cap_high_v1,
        cap_high_v2,
        cap_low_v1,
        cap_low_v2,
        predicted_speed_kmh);
  }
}

static void logicTask(void *) {
  for (;;) {
    LogicModule_Update();
    vTaskDelay(pdMS_TO_TICKS(LOGIC_LOOP_MS));
  }
}

void LogicModule_StartTask() {
  if (g_logic_task) return;

  xTaskCreatePinnedToCore(
      logicTask,
      "LOGIC",
      4096,
      nullptr,
      4,
      &g_logic_task,
      0);
}

bool LogicModule_IsRelayActive() { return relay_active; }

#else

void LogicModule_Begin() {}
void LogicModule_StartTask() {}
bool LogicModule_IsRelayActive() { return false; }

#endif // !BUILD_TEST_LOGGER
