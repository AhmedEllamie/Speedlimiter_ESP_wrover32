#include "logic_module.h"
#include "shared_state.h"
#include "sl_config.h"

#if !BUILD_TEST_LOGGER && !USE_ALGORITHM_MODULE

#include "new_algorithm.h"

#include <Arduino.h>
#include <Preferences.h>

#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -----------------------------------------------------------------------------
// New algorithm: APPS table + band decay state machine
// -----------------------------------------------------------------------------

static constexpr uint32_t LOGIC_LOOP_MS = 10;
static constexpr uint32_t TELEMETRY_MS = 100;
static constexpr float APS_IDLE_MARGIN_V = 0.05f;

// (bandSpeed) Start controlling when SPD > (SL - bandSpeed)
static constexpr uint16_t BAND_SPEED_KMH = SPEED_LIMIT_ACTIVATION_OFFSET_KMH;

// (9) LIMIT_ACTIVE when SPD in (SL +/- 2)
static constexpr uint8_t LIMIT_ACTIVE_TOLERANCE_KMH = 2;

// Decay tuning:
// - Keep these conservative to avoid dropping APS_hold toward DEFAULT_SIGNAL too fast.
static constexpr float DECAY_RATE_MIN_PER_S = 0.005f;       // gentle cut at band start
static constexpr float DECAY_RATE_MAX_PER_S = 0.025f;       // stronger cut near SL
static constexpr float DECAY_RATE_OVERSPEED_PER_S = 0.60f; // cut when already above SL
static constexpr float DECAY_DT_MAX_S = 0.10f;             // avoid big one-step drops

enum class LimiterState {
  PASS_THROUGH = 0,
  OVERSHOOT_CONTROL = 1,
  LIMIT_ACTIVE = 2,
  FAULT = 3,
};

enum class SpeedTrend {
  UNKNOWN = 0,
  SPEED_FALLING = 1,
  SPEED_STABLE = 2,
  SPEED_RISING = 3,
};

static TaskHandle_t g_logic_task = nullptr;
static Preferences prefs;
static uint16_t g_speed_limit_kmh = SPEED_LIMIT_DEFAULT_KMH;

static LimiterState state = LimiterState::PASS_THROUGH;
static bool relay_active = false;

// Outputs (ECU-level volts)
static float aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
static float aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;

// (8) Hold value while speed is fixed near limit
static float aps_hold_v1 = DEFAULT_SIGNAL_S1_V;
static float aps_hold_v2 = DEFAULT_SIGNAL_S2_V;

static uint32_t last_control_ms = 0;
static uint32_t last_telemetry_ms = 0;

// Speed trend tracking (based on CAN update timestamp)
static uint8_t last_speed_sample_kmh = 0;
static uint32_t last_speed_sample_update_ms = 0;
static SpeedTrend last_speed_trend = SpeedTrend::UNKNOWN;

static float maxf(float a, float b) { return (a > b) ? a : b; }
static float mvToVolts(uint16_t mv) { return (float)mv / 1000.0f; }

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void setRelay(bool on) {
  relay_active = on;
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

static bool pedalReleased(float in1, float in2) {
  return (in1 <= (DEFAULT_SIGNAL_S1_V + APS_IDLE_MARGIN_V)) && (in2 <= (DEFAULT_SIGNAL_S2_V + APS_IDLE_MARGIN_V));
}

static uint16_t computeBandStartKmh(uint16_t sl_kmh) {
  if (sl_kmh <= BAND_SPEED_KMH) return 0;
  return (uint16_t)(sl_kmh - BAND_SPEED_KMH);
}

static SpeedTrend updateSpeedTrend(uint8_t speed_kmh, uint32_t speed_update_ms) {
  if (speed_update_ms == 0 || speed_update_ms == last_speed_sample_update_ms) {
    return last_speed_trend;
  }

  if (speed_kmh < last_speed_sample_kmh) {
    last_speed_trend = SpeedTrend::SPEED_FALLING;
  } else if (speed_kmh > last_speed_sample_kmh) {
    last_speed_trend = SpeedTrend::SPEED_RISING;
  } else {
    last_speed_trend = SpeedTrend::SPEED_STABLE;
  }

  last_speed_sample_kmh = speed_kmh;
  last_speed_sample_update_ms = speed_update_ms;
  return last_speed_trend;
}

// (10) Detect speed falling (not increasing).
static bool isSpeedFalling(uint8_t speed_kmh, uint32_t speed_update_ms) {
  return updateSpeedTrend(speed_kmh, speed_update_ms) == SpeedTrend::SPEED_FALLING;
}

// (2) Function that takes g_speed_limit_kmh and returns APPS table value (2-D array).
static void lookupAppsTableForSpeedLimitKmh(uint16_t speed_limit_kmh, float *out_v1, float *out_v2) {
  if (!out_v1 || !out_v2) return;

  if (NISSAN_SENTRA_APPS_SPEED_MAP_LEN == 0) {
    *out_v1 = DEFAULT_SIGNAL_S1_V;
    *out_v2 = DEFAULT_SIGNAL_S2_V;
    return;
  }

  const NissanSentraAppsMapPoint &first = NISSAN_SENTRA_APPS_SPEED_MAP[0];
  if (speed_limit_kmh <= first.speed_kmh) {
    *out_v1 = mvToVolts(first.aps1_mv);
    *out_v2 = mvToVolts(first.aps2_mv);
    *out_v1 = maxf(*out_v1, DEFAULT_SIGNAL_S1_V);
    *out_v2 = maxf(*out_v2, DEFAULT_SIGNAL_S2_V);
    return;
  }

  const NissanSentraAppsMapPoint &last = NISSAN_SENTRA_APPS_SPEED_MAP[NISSAN_SENTRA_APPS_SPEED_MAP_LEN - 1];
  if (speed_limit_kmh >= last.speed_kmh) {
    *out_v1 = mvToVolts(last.aps1_mv);
    *out_v2 = mvToVolts(last.aps2_mv);
    *out_v1 = maxf(*out_v1, DEFAULT_SIGNAL_S1_V);
    *out_v2 = maxf(*out_v2, DEFAULT_SIGNAL_S2_V);
    return;
  }

  for (size_t i = 0; (i + 1) < NISSAN_SENTRA_APPS_SPEED_MAP_LEN; i++) {
    const NissanSentraAppsMapPoint &a = NISSAN_SENTRA_APPS_SPEED_MAP[i];
    const NissanSentraAppsMapPoint &b = NISSAN_SENTRA_APPS_SPEED_MAP[i + 1];
    if (speed_limit_kmh <= b.speed_kmh) {
      uint16_t s0 = a.speed_kmh;
      uint16_t s1 = b.speed_kmh;
      float t = 0.0f;
      if (s1 > s0) t = (float)(speed_limit_kmh - s0) / (float)(s1 - s0);

      float aps1_mv = (float)a.aps1_mv + ((float)b.aps1_mv - (float)a.aps1_mv) * t;
      float aps2_mv = (float)a.aps2_mv + ((float)b.aps2_mv - (float)a.aps2_mv) * t;

      *out_v1 = mvToVolts((uint16_t)(aps1_mv + 0.5f));
      *out_v2 = mvToVolts((uint16_t)(aps2_mv + 0.5f));
      *out_v1 = maxf(*out_v1, DEFAULT_SIGNAL_S1_V);
      *out_v2 = maxf(*out_v2, DEFAULT_SIGNAL_S2_V);
      return;
    }
  }

  // Fallback (shouldn't hit).
  *out_v1 = mvToVolts(last.aps1_mv);
  *out_v2 = mvToVolts(last.aps2_mv);

  // Safety floors.
  *out_v1 = maxf(*out_v1, DEFAULT_SIGNAL_S1_V);
  *out_v2 = maxf(*out_v2, DEFAULT_SIGNAL_S2_V);
}

static void resetLimiter(uint32_t now_ms) {
  state = LimiterState::PASS_THROUGH;
  aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
  aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;
  aps_hold_v1 = DEFAULT_SIGNAL_S1_V;
  aps_hold_v2 = DEFAULT_SIGNAL_S2_V;
  last_control_ms = now_ms;

  last_speed_sample_kmh = 0;
  last_speed_sample_update_ms = 0;
  last_speed_trend = SpeedTrend::UNKNOWN;
}

bool LogicModule_IsRelayActive() { return relay_active; }

void LogicModule_Begin() {
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false);

  prefs.begin(PREF_NS, false);
  g_speed_limit_kmh = prefs.getUShort(PREF_KEY_SL, SPEED_LIMIT_DEFAULT_KMH);

  uint32_t now = millis();
  resetLimiter(now);
  last_telemetry_ms = now;

  Serial.printf("LogicModule (new algorithm) ready. SL=%u km/h\r\n", (unsigned)g_speed_limit_kmh);
  Serial.println("Set speed limit with: SL=<0..250>");
}

static void LogicModule_Update() {
  uint32_t now = millis();

  // Read APS inputs (ECU-level volts).
  float aps_in_v1 = DEFAULT_SIGNAL_S1_V;
  float aps_in_v2 = DEFAULT_SIGNAL_S2_V;
  uint32_t aps_ts = 0;
  SharedState_GetAps(&aps_in_v1, &aps_in_v2, &aps_ts);
  aps_in_v1 = maxf(aps_in_v1, DEFAULT_SIGNAL_S1_V);
  aps_in_v2 = maxf(aps_in_v2, DEFAULT_SIGNAL_S2_V);

  // Read speed (km/h) + validity.
  bool speed_valid = SharedState_SpeedValid(now, SPEED_TIMEOUT_MS);
  uint8_t speed_kmh = (uint8_t)g_speed_kmh;
  uint32_t speed_update_ms = g_speed_last_update_ms;

  // Trend update + (10) falling detection.
  bool speed_falling = isSpeedFalling(speed_kmh, speed_update_ms);
  bool speed_rising = (last_speed_trend == SpeedTrend::SPEED_RISING);
  bool speed_stable = (last_speed_trend == SpeedTrend::SPEED_STABLE);

  // Fail-safe: invalid speed or disabled SL -> relay OFF + pass-through.
  if (!speed_valid || g_speed_limit_kmh == 0) {
    state = LimiterState::PASS_THROUGH;
    setRelay(false);
    aps_cmd_v1 = aps_in_v1;
    aps_cmd_v2 = aps_in_v2;
    aps_hold_v1 = aps_in_v1;
    aps_hold_v2 = aps_in_v2;
    last_control_ms = now;
    SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
    return;
  }

  uint16_t band_start_kmh = computeBandStartKmh(g_speed_limit_kmh); // (SL - bandSpeed)

  switch (state) {
    case LimiterState::PASS_THROUGH: {
      setRelay(false);

      // Default outputs are pass-through.
      aps_cmd_v1 = aps_in_v1;
      aps_cmd_v2 = aps_in_v2;

      // Arm limiter when we're within band and driver is not at idle.
      if (speed_kmh >= band_start_kmh && !pedalReleased(aps_in_v1, aps_in_v2)) {
        // Prime hold from the table-capped driver demand (so we never increase throttle).
        float apps_table_v1 = DEFAULT_SIGNAL_S1_V;
        float apps_table_v2 = DEFAULT_SIGNAL_S2_V;
        lookupAppsTableForSpeedLimitKmh(g_speed_limit_kmh, &apps_table_v1, &apps_table_v2);

        float base_v1 = (aps_in_v1 > apps_table_v1) ? apps_table_v1 : aps_in_v1;
        float base_v2 = (aps_in_v2 > apps_table_v2) ? apps_table_v2 : aps_in_v2;

        aps_hold_v1 = base_v1;
        aps_hold_v2 = base_v2;
        last_control_ms = now;

        state = LimiterState::OVERSHOOT_CONTROL;
      }

      SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
      break;
    }

    case LimiterState::OVERSHOOT_CONTROL: {
      // (1) Activate relay.
      setRelay(true);

      // (2/3) Get APPS table for g_speed_limit_kmh.
      float apps_table_v1 = DEFAULT_SIGNAL_S1_V;
      float apps_table_v2 = DEFAULT_SIGNAL_S2_V;
      lookupAppsTableForSpeedLimitKmh(g_speed_limit_kmh, &apps_table_v1, &apps_table_v2);

      // (4/5) APS_out = min(APS_in, APPS_table)
      float aps_out_v1 = (aps_in_v1 > apps_table_v1) ? apps_table_v1 : aps_in_v1;
      float aps_out_v2 = (aps_in_v2 > apps_table_v2) ? apps_table_v2 : aps_in_v2;

      // Keep hold never above current table-capped driver demand.
      if (aps_hold_v1 > aps_out_v1) aps_hold_v1 = aps_out_v1;
      if (aps_hold_v2 > aps_out_v2) aps_hold_v2 = aps_out_v2;

      // (11) If speed is falling and below (SL - bandSpeed) -> PASS_THROUGH.
      if (speed_falling && speed_kmh < band_start_kmh) {
        state = LimiterState::PASS_THROUGH;
        setRelay(false);
        aps_cmd_v1 = aps_in_v1;
        aps_cmd_v2 = aps_in_v2;
        aps_hold_v1 = aps_in_v1;
        aps_hold_v2 = aps_in_v2;
        SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
        break;
      }

      // (6/7) If speed is within the band, start decaying APPS_out (via APPS_hold)
      // while speed is rising (or already over the limit).
      bool in_band = ((uint16_t)speed_kmh > band_start_kmh);
      bool above_limit = ((uint16_t)speed_kmh > g_speed_limit_kmh);

      if (in_band && (speed_rising || above_limit)) {
        float dt_s = (float)(now - last_control_ms) / 1000.0f;
        if (dt_s < 0.0f) dt_s = 0.0f;
        if (dt_s > DECAY_DT_MAX_S) dt_s = DECAY_DT_MAX_S; // prevents big steps after pauses
        last_control_ms = now;

        float t = 1.0f;
        if (g_speed_limit_kmh > band_start_kmh) {
          t = (float)((uint16_t)speed_kmh - band_start_kmh) / (float)(g_speed_limit_kmh - band_start_kmh);
        }
        t = clampf(t, 0.0f, 1.0f);

        // Decay ramps up as we approach SL. Overspeed -> harsh cut.
        float decay_rate_per_s = DECAY_RATE_MIN_PER_S + (DECAY_RATE_MAX_PER_S - DECAY_RATE_MIN_PER_S) * t;
        if (above_limit) decay_rate_per_s = DECAY_RATE_OVERSPEED_PER_S;

        float scale = 1.0f - (decay_rate_per_s * dt_s);
        scale = clampf(scale, 0.0f, 1.0f);

        aps_hold_v1 *= scale;
        aps_hold_v2 *= scale;

        if (aps_hold_v1 < DEFAULT_SIGNAL_S1_V) aps_hold_v1 = DEFAULT_SIGNAL_S1_V;
        if (aps_hold_v2 < DEFAULT_SIGNAL_S2_V) aps_hold_v2 = DEFAULT_SIGNAL_S2_V;
      } else {
        // Keep timing baseline fresh even when not decaying.
        last_control_ms = now;
      }

      // Final output: do NOT exceed driver demand or hold.
      aps_cmd_v1 = aps_out_v1;
      aps_cmd_v2 = aps_out_v2;
      if (aps_cmd_v1 > aps_hold_v1) aps_cmd_v1 = aps_hold_v1;
      if (aps_cmd_v2 > aps_hold_v2) aps_cmd_v2 = aps_hold_v2;

      // (8) When speed is fixed, set APPS_hold = APPS_out.
      if (speed_stable && (uint16_t)speed_kmh <= g_speed_limit_kmh && in_band) {
        aps_hold_v1 = aps_cmd_v1;
        aps_hold_v2 = aps_cmd_v2;
      }

      SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);

      // (9) If speed within (SL +/- 2) -> LIMIT_ACTIVE.
      int32_t diff = (int32_t)speed_kmh - (int32_t)g_speed_limit_kmh;
      if (diff >= -(int32_t)LIMIT_ACTIVE_TOLERANCE_KMH && diff <= (int32_t)LIMIT_ACTIVE_TOLERANCE_KMH) {
        state = LimiterState::LIMIT_ACTIVE;
      }
      break;
    }

    case LimiterState::LIMIT_ACTIVE: {
      setRelay(true);

      // Keep output <= APPS_hold while still respecting APPS_table and driver demand.
      float apps_table_v1 = DEFAULT_SIGNAL_S1_V;
      float apps_table_v2 = DEFAULT_SIGNAL_S2_V;
      lookupAppsTableForSpeedLimitKmh(g_speed_limit_kmh, &apps_table_v1, &apps_table_v2);

      float base_v1 = (aps_in_v1 > apps_table_v1) ? apps_table_v1 : aps_in_v1;
      float base_v2 = (aps_in_v2 > apps_table_v2) ? apps_table_v2 : aps_in_v2;

      if (aps_hold_v1 > base_v1) aps_hold_v1 = base_v1;
      if (aps_hold_v2 > base_v2) aps_hold_v2 = base_v2;

      // (11) Falling and below band -> PASS_THROUGH.
      if (speed_falling && speed_kmh < band_start_kmh) {
        state = LimiterState::PASS_THROUGH;
        setRelay(false);
        aps_cmd_v1 = aps_in_v1;
        aps_cmd_v2 = aps_in_v2;
        aps_hold_v1 = aps_in_v1;
        aps_hold_v2 = aps_in_v2;
        SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
        break;
      }

      // If we drift above SL + tolerance, go back to OVERSHOOT_CONTROL to decay further.
      int32_t d = (int32_t)speed_kmh - (int32_t)g_speed_limit_kmh;
      if (d > (int32_t)LIMIT_ACTIVE_TOLERANCE_KMH) {
        state = LimiterState::OVERSHOOT_CONTROL;
        last_control_ms = now;
      }

      aps_cmd_v1 = base_v1;
      aps_cmd_v2 = base_v2;
      if (aps_cmd_v1 > aps_hold_v1) aps_cmd_v1 = aps_hold_v1;
      if (aps_cmd_v2 > aps_hold_v2) aps_cmd_v2 = aps_hold_v2;
      SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
      break;
    }

    case LimiterState::FAULT:
    default:
      setRelay(false);
      aps_cmd_v1 = aps_in_v1;
      aps_cmd_v2 = aps_in_v2;
      SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
      break;
  }

  // Telemetry (throttled)
  if ((uint32_t)(now - last_telemetry_ms) >= TELEMETRY_MS) {
    last_telemetry_ms = now;
    Serial.printf(
        "SPD=%u SL=%u bandStart=%u State=%d Trend=%d APSin=(%.2f,%.2f) APShold=(%.2f,%.2f) APSout=(%.2f,%.2f) Relay=%d\r\n",
        (unsigned)speed_kmh,
        (unsigned)g_speed_limit_kmh,
        (unsigned)band_start_kmh,
        (int)state,
        (int)last_speed_trend,
        aps_in_v1,
        aps_in_v2,
        aps_hold_v1,
        aps_hold_v2,
        aps_cmd_v1,
        aps_cmd_v2,
        relay_active ? 1 : 0);
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

#endif // !BUILD_TEST_LOGGER && !USE_ALGORITHM_MODULE