#include "speed_controller_module.h"

#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>
#include <Preferences.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/task.h"

#include "shared_state.h"

// -----------------------------------------------------------------------------
// Tuning (new algorithm)
// -----------------------------------------------------------------------------
// Speed controller activates when we're within this many km/h of SL.
static const uint16_t SC_ACTIVATION_GAP_KMH = 10;

// Low-pass filter for speed derivative.
static const float SC_SPEED_RATE_LPF_ALPHA = 0.85f; // 0..1 (higher = more smoothing)

// Minimum time between *reductions* (car response delay aware).
// User-requested: wait 400ms after each reduction before reducing again.
static const uint32_t SC_CUT_INTERVAL_MS = 1200;

// Minimum time between relax (increasing output back toward APS_in). Keep it faster than cuts.
static const uint32_t SC_RELAX_INTERVAL_MS = 150;

// Speed error bands to avoid dithering around SL.
static const float SC_RELAX_BAND_KMH = 0.8f; // must be under SL by this to relax up

// Output adjustment rates.
// - Down: proportional to overspeed (km/h) and/or positive accel (km/h/s)
// - Up: very slow relaxation toward APS input while safely under SL and not rising
static const float SC_RATE_DOWN_PER_KMH_PER_S = 0.40f;
static const float SC_RATE_DOWN_PER_KMHPS_PER_S = 0.12f;
static const float SC_RATE_UP_PER_S = 0.07f;

// Driver override: if APS_in drops below APS_out (with margin), release relay.
static const uint16_t SC_DRIVER_RELEASE_MARGIN_MV = 20;

// Hold outputs after activation before allowing cuts. Keep at 0 so the first cut can happen early.
static const uint32_t SC_HOLD_AFTER_ACTIVATION_MS = 0;

// Clamp cut aggressiveness (prevents very large step drops).
static const float SC_CUT_RATE_MAX_PER_S = 1.0f;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float maxf(float a, float b) { return (a > b) ? a : b; }
static float mvToVolts(uint16_t mv) { return (float)mv / 1000.0f; }

// -----------------------------------------------------------------------------
// Module state
// -----------------------------------------------------------------------------
static TaskHandle_t g_sc_task = nullptr;

static Preferences prefs;
static uint16_t s_speed_limit_kmh = SPEED_LIMIT_DEFAULT_KMH;

// USB CLI buffer (only: SL=<kmh>)
static char cli_buf[32];
static uint8_t cli_len = 0;

struct LinkedInputs {
  uint32_t sample_ms; // when we latched the set
  bool speed_valid;
  uint8_t speed_kmh;
  uint32_t speed_update_ms;
  uint16_t rpm;
  uint32_t rpm_update_ms;
  float aps1_in_v;
  float aps2_in_v;
  uint32_t aps_update_ms;
};

struct SpeedControllerState {
  bool active;
  float aps1_out_v;
  float aps2_out_v;

  uint8_t last_speed_kmh;
  uint32_t last_speed_update_ms;
  float speed_rate_kmh_s;

  uint32_t last_control_ms;
  uint32_t last_cut_ms;
  uint32_t last_relax_ms;
  uint32_t activation_ms;
};

static SpeedControllerState st;

static void setRelayActive(bool active) { digitalWrite(RELAY_PIN, active ? HIGH : LOW); }

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

  s_speed_limit_kmh = (uint16_t)v;
  prefs.putUShort(PREF_KEY_SL, s_speed_limit_kmh);
  Serial.printf("OK SL=%u\r\n", (unsigned)s_speed_limit_kmh);
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

static void resetController(uint32_t now_ms) {
  st.active = false;
  st.aps1_out_v = DEFAULT_SIGNAL_S1_V;
  st.aps2_out_v = DEFAULT_SIGNAL_S2_V;
  st.last_speed_kmh = 0;
  st.last_speed_update_ms = 0;
  st.speed_rate_kmh_s = 0.0f;
  st.last_control_ms = now_ms;
  st.last_cut_ms = 0;
  st.last_relax_ms = 0;
  st.activation_ms = 0;
}

static uint16_t computeActivationKmh(uint16_t sl_kmh) {
  if (sl_kmh <= SC_ACTIVATION_GAP_KMH) return 0;
  return (uint16_t)(sl_kmh - SC_ACTIVATION_GAP_KMH);
}

static void updateSpeedDerivative(uint8_t speed_kmh, uint32_t speed_update_ms) {
  if (speed_update_ms == 0 || speed_update_ms == st.last_speed_update_ms) return;

  if (st.last_speed_update_ms != 0 && speed_update_ms > st.last_speed_update_ms) {
    float dt_s = (float)(speed_update_ms - st.last_speed_update_ms) / 1000.0f;
    if (dt_s > 0.001f) {
      float inst_rate = ((float)speed_kmh - (float)st.last_speed_kmh) / dt_s;
      st.speed_rate_kmh_s =
          (st.speed_rate_kmh_s * SC_SPEED_RATE_LPF_ALPHA) + (inst_rate * (1.0f - SC_SPEED_RATE_LPF_ALPHA));
    }
  }

  st.last_speed_kmh = speed_kmh;
  st.last_speed_update_ms = speed_update_ms;
}

static void publishOutputsAndRelay(const LinkedInputs &in) {
  if (st.active) {
    setRelayActive(true);
    SharedState_SetDesiredOutputs(st.aps1_out_v, st.aps2_out_v);
  } else {
    setRelayActive(false);
    // Pass-through (still publish for visibility / PWM alignment)
    SharedState_SetDesiredOutputs(in.aps1_in_v, in.aps2_in_v);
  }
}

static void printStatusLine(const LinkedInputs &in) {
  int relay_pin_level = digitalRead(RELAY_PIN);
  Serial.printf(
      "CAN_SPEED=%u km/h, RPM=%u, APS_in=(%.3fV, %.3fV), APS_out=(%.3fV, %.3fV), Relay=%s (IO%d=%s)\r\n",
      (unsigned)in.speed_kmh,
      (unsigned)in.rpm,
      in.aps1_in_v,
      in.aps2_in_v,
      st.active ? st.aps1_out_v : in.aps1_in_v,
      st.active ? st.aps2_out_v : in.aps2_in_v,
      st.active ? "ON" : "OFF",
      RELAY_PIN,
      relay_pin_level == HIGH ? "HIGH" : "LOW");
}

static void speedControllerStep(const LinkedInputs &in) {
  // Fail-safe: invalid speed or disabled SL -> relay OFF + pass-through.
  if (!in.speed_valid || s_speed_limit_kmh == 0) {
    st.active = false;
    return;
  }

  uint16_t activation_kmh = computeActivationKmh(s_speed_limit_kmh);
  int32_t gap_kmh = (int32_t)s_speed_limit_kmh - (int32_t)in.speed_kmh;

  // Rule #5: if SL - CAN_SPEED > 10 -> do nothing (not active).
  if (gap_kmh > (int32_t)SC_ACTIVATION_GAP_KMH) {
    st.active = false;
    return;
  }

  // Rule #4: if we're within 10 km/h of SL (or above), run SpeedController.
  (void)activation_kmh; // activation_kmh computed for clarity/debug; gap logic is authoritative.

  if (!st.active) {
    // Rule #6a/#6b: capture current APS inputs as outputs, relay ON.
    st.active = true;
    st.aps1_out_v = maxf(in.aps1_in_v, DEFAULT_SIGNAL_S1_V);
    st.aps2_out_v = maxf(in.aps2_in_v, DEFAULT_SIGNAL_S2_V);
    st.last_control_ms = in.sample_ms;
    // Allow first cut immediately after activation (no extra waiting), but still
    // enforce SC_CUT_INTERVAL_MS between subsequent cuts.
    st.last_cut_ms = (in.sample_ms >= SC_CUT_INTERVAL_MS) ? (in.sample_ms - SC_CUT_INTERVAL_MS) : 0;
    st.last_relax_ms = in.sample_ms;
    st.activation_ms = in.sample_ms;
    // Discard stale accel data and restart derivative baseline at activation.
    st.speed_rate_kmh_s = 0.0f;
    st.last_speed_kmh = in.speed_kmh;
    st.last_speed_update_ms = in.speed_update_ms;
    return;
  }

  // Rule #6f: driver override -> if APS_in < APS_out => relay OFF.
  float rel_margin_v = mvToVolts(SC_DRIVER_RELEASE_MARGIN_MV);
  if ((in.aps1_in_v + rel_margin_v) < st.aps1_out_v || (in.aps2_in_v + rel_margin_v) < st.aps2_out_v) {
    st.active = false;
    return;
  }

  // Apply control at a fixed interval.
  if ((uint32_t)(in.sample_ms - st.last_control_ms) < CONTROL_INTERVAL_MS) return;

  float dt_s = (float)(in.sample_ms - st.last_control_ms) / 1000.0f;
  st.last_control_ms = in.sample_ms;
  if (dt_s <= 0.0f) return;

  // Give the vehicle some time after activation before applying the first cut.
  uint32_t since_activation = (uint32_t)(in.sample_ms - st.activation_ms);
  if (since_activation < SC_HOLD_AFTER_ACTIVATION_MS) return;

  // Enforce timing:
  // - Cuts (reductions) are limited to every SC_CUT_INTERVAL_MS.
  // - Relax steps are limited to every SC_RELAX_INTERVAL_MS.
  bool allow_cut = ((uint32_t)(in.sample_ms - st.last_cut_ms) >= SC_CUT_INTERVAL_MS);
  bool allow_relax = ((uint32_t)(in.sample_ms - st.last_relax_ms) >= SC_RELAX_INTERVAL_MS);

  // Control objective: keep speed at exactly SL by trimming APS_out when speed rises,
  // and relaxing slowly back toward APS_in when safely under SL and not rising.
  float speed_err_kmh = (float)in.speed_kmh - (float)s_speed_limit_kmh; // + = overspeed
  float rise_kmh_s = st.speed_rate_kmh_s;

  // Rule #6c: if speed is increasing -> reduce outputs.
  bool speed_rising = (rise_kmh_s > 0.05f);

  if (allow_cut && (speed_rising || speed_err_kmh > 0.0f)) {
    float overspeed_kmh = (speed_err_kmh > 0.0f) ? speed_err_kmh : 0.0f;
    float pos_rise = (rise_kmh_s > 0.0f) ? rise_kmh_s : 0.0f;

    float cut_rate_per_s = (SC_RATE_DOWN_PER_KMH_PER_S * overspeed_kmh) + (SC_RATE_DOWN_PER_KMHPS_PER_S * pos_rise);
    // Ensure *some* cut when rising near the limit to prevent overshoot.
    if (cut_rate_per_s < 0.05f) cut_rate_per_s = 0.05f;
    if (cut_rate_per_s > SC_CUT_RATE_MAX_PER_S) cut_rate_per_s = SC_CUT_RATE_MAX_PER_S;

    float scale = 1.0f - (cut_rate_per_s * dt_s);
    scale = clampf(scale, 0.0f, 1.0f);

    st.aps1_out_v *= scale;
    st.aps2_out_v *= scale;

    st.last_cut_ms = in.sample_ms;
  } else if (allow_relax) {
    // Rule #6e: calibrate/relax upward when we're safely under SL and not rising,
    // converging to "just enough" APS_out to hold speed near SL.
    bool safely_under = ((float)s_speed_limit_kmh - (float)in.speed_kmh) >= SC_RELAX_BAND_KMH;
    bool not_rising = (rise_kmh_s <= 0.05f);
    if (safely_under && not_rising) {
      float k = clampf(SC_RATE_UP_PER_S * dt_s, 0.0f, 1.0f);
      // Relax toward current APS input, but never exceed it (keep driver demand as upper bound).
      st.aps1_out_v += (in.aps1_in_v - st.aps1_out_v) * k;
      st.aps2_out_v += (in.aps2_in_v - st.aps2_out_v) * k;
      if (st.aps1_out_v > in.aps1_in_v) st.aps1_out_v = in.aps1_in_v;
      if (st.aps2_out_v > in.aps2_in_v) st.aps2_out_v = in.aps2_in_v;
      st.last_relax_ms = in.sample_ms;
    }
  }

  // Safety floors.
  if (st.aps1_out_v < DEFAULT_SIGNAL_S1_V) st.aps1_out_v = DEFAULT_SIGNAL_S1_V;
  if (st.aps2_out_v < DEFAULT_SIGNAL_S2_V) st.aps2_out_v = DEFAULT_SIGNAL_S2_V;
}

static LinkedInputs readLinkedInputs(uint32_t now_ms) {
  LinkedInputs in{};

  // Speed validity is based on the "now" of this snapshot.
  in.sample_ms = now_ms;
  in.speed_valid = SharedState_SpeedValid(now_ms, SPEED_TIMEOUT_MS);
  in.speed_kmh = (uint8_t)g_speed_kmh;
  in.speed_update_ms = g_speed_last_update_ms;

  in.rpm = (uint16_t)g_rpm;
  in.rpm_update_ms = g_rpm_last_update_ms;

  float aps1 = DEFAULT_SIGNAL_S1_V;
  float aps2 = DEFAULT_SIGNAL_S2_V;
  uint32_t aps_ms = 0;
  SharedState_GetAps(&aps1, &aps2, &aps_ms);

  // Clamp APS inputs to safety floors for pass-through + captures.
  in.aps1_in_v = maxf(aps1, DEFAULT_SIGNAL_S1_V);
  in.aps2_in_v = maxf(aps2, DEFAULT_SIGNAL_S2_V);
  in.aps_update_ms = aps_ms;

  return in;
}

static void speedControllerTask(void *param) {
  (void)param;

  resetController(millis());

  uint32_t last_logged_ms = 0;
  uint32_t last_speed_seen_update_ms = 0;

  for (;;) {
    uint32_t now = millis();

    handleUsbCli();

    // "Link" APS + CAN values: only latch/process when CAN speed updates.
    uint32_t speed_update_ms = g_speed_last_update_ms;
    bool new_speed = (speed_update_ms != 0 && speed_update_ms != last_speed_seen_update_ms);
    if (new_speed) {
      last_speed_seen_update_ms = speed_update_ms;

      LinkedInputs in = readLinkedInputs(now);

      // Update derivative from CAN timestamps (captures "same-time" speed behavior).
      updateSpeedDerivative(in.speed_kmh, in.speed_update_ms);

      // Always check CAN speed with SL and run the controller rules.
      speedControllerStep(in);

      // Apply relay + outputs.
      publishOutputsAndRelay(in);

      // Print requested line (throttle to avoid serial overload).
      // Note: We print at most ~20Hz even if CAN speed is faster.
      if ((uint32_t)(now - last_logged_ms) >= 50) {
        printStatusLine(in);
        last_logged_ms = now;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void SpeedControllerModule_Begin() {
  pinMode(RELAY_PIN, OUTPUT);
  setRelayActive(false); // fail-safe at boot

  prefs.begin(PREF_NS, false);
  s_speed_limit_kmh = prefs.getUShort(PREF_KEY_SL, SPEED_LIMIT_DEFAULT_KMH);

  resetController(millis());

  Serial.printf("SpeedControllerModule ready. SL=%u km/h\r\n", (unsigned)s_speed_limit_kmh);
  Serial.println("Set speed limit with: SL=<0..250>");
}

void SpeedControllerModule_StartTask() {
  if (g_sc_task) return;

  xTaskCreatePinnedToCore(
      speedControllerTask,
      "SPEED_CTRL_TASK",
      4096, // words
      nullptr,
      4,
      &g_sc_task,
      1);
}

#else

void SpeedControllerModule_Begin() {}
void SpeedControllerModule_StartTask() {}

#endif // !BUILD_TEST_LOGGER

