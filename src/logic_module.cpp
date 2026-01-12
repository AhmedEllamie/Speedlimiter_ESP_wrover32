#include "logic_module.h"

#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>
#include <Preferences.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared_state.h"

static TaskHandle_t g_logic_task = nullptr;

static Preferences prefs;
static uint16_t speed_limit_kmh = SPEED_LIMIT_DEFAULT_KMH;

// Limiter state
static bool limiting = false;
static uint16_t captured_pedal_avg_mv = 0;
static float captured_pedal_v1 = DEFAULT_SIGNAL_S1_V;
static float captured_pedal_v2 = DEFAULT_SIGNAL_S2_V;
static float limit_factor = 1.0f; // 1.0 = pass-through, <1 reduces APS outputs
static uint16_t last_speed_kmh = 0;
static uint32_t last_speed_update_ms = 0;
static float speed_rate_kmh_s = 0.0f; // filtered d(speed)/dt
static uint32_t last_control_ms = 0;

// USB CLI buffer (only: SL=<kmh>)
static char cli_buf[32];
static uint8_t cli_len = 0;

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline uint16_t mvFromVolts(float v) {
  if (v <= 0.0f) return 0;
  float mv = v * 1000.0f;
  if (mv > 65535.0f) mv = 65535.0f;
  return (uint16_t)(mv + 0.5f);
}

static void setRelayActive(bool active) {
  digitalWrite(RELAY_PIN, active ? HIGH : LOW);
}

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

static void logicTask(void *param) {
  (void)param;

  for (;;) {
    uint32_t now = millis();

    handleUsbCli();

    bool spd_ok = SharedState_SpeedValid(now, SPEED_TIMEOUT_MS);
    uint16_t spd = (uint16_t)g_speed_kmh;
    uint32_t spd_update_ms = g_speed_last_update_ms;
    uint16_t effective_limit_kmh = speed_limit_kmh;
    if (speed_limit_kmh > SPEED_LIMIT_ACTIVATION_OFFSET_KMH) {
      effective_limit_kmh = (uint16_t)(speed_limit_kmh - SPEED_LIMIT_ACTIVATION_OFFSET_KMH);
    }
    uint16_t effective_release_kmh = 0;
    if (effective_limit_kmh > SPEED_LIMIT_DEACTIVATION_HYSTERESIS_KMH) {
      effective_release_kmh = (uint16_t)(effective_limit_kmh - SPEED_LIMIT_DEACTIVATION_HYSTERESIS_KMH);
    }

    float aps1 = DEFAULT_SIGNAL_S1_V;
    float aps2 = DEFAULT_SIGNAL_S2_V;
    SharedState_GetAps(&aps1, &aps2, nullptr);

    // Update speed derivative when a NEW speed sample arrives (avoids 1ms loop quantization spikes)
    if (spd_update_ms != 0 && spd_update_ms != last_speed_update_ms) {
      if (last_speed_update_ms != 0 && spd_update_ms > last_speed_update_ms) {
        float dt_s = (float)(spd_update_ms - last_speed_update_ms) / 1000.0f;
        if (dt_s > 0.001f) {
          float inst_rate = ((float)spd - (float)last_speed_kmh) / dt_s;
          // Light low-pass filter (reduces jitter from integer km/h steps)
          speed_rate_kmh_s = (speed_rate_kmh_s * 0.7f) + (inst_rate * 0.3f);
        }
      }
      last_speed_kmh = spd;
      last_speed_update_ms = spd_update_ms;
    }

    if (!spd_ok || speed_limit_kmh == 0) {
      // Fail-safe: if speed is unknown, do NOT keep relay active.
      limiting = false;
      limit_factor = 1.0f;
      setRelayActive(false);
    } else if (!limiting) {
      // Not limiting yet: wait until speed reaches the threshold.
      if (spd >= effective_limit_kmh) {
        limiting = true;
        captured_pedal_v1 = aps1;
        captured_pedal_v2 = aps2;
        captured_pedal_avg_mv = mvFromVolts((aps1 + aps2) * 0.5f);
        limit_factor = 1.0f; // at threshold: output == input
        last_control_ms = now;
        setRelayActive(true);
      } else {
        limit_factor = 1.0f;
        setRelayActive(false);
      }
    } else {
      // Limiting active.
      uint16_t pedal_avg_mv = mvFromVolts((aps1 + aps2) * 0.5f);

      // Release when driver eases pedal below captured position by ~100mV.
      // Also release when speed drops well below the effective limit (hysteresis).
      if (spd <= effective_release_kmh || (pedal_avg_mv + PEDAL_RELEASE_MARGIN_MV < captured_pedal_avg_mv)) {
        limiting = false;
        limit_factor = 1.0f;
        setRelayActive(false);
      } else {
        setRelayActive(true);

        // SPEED LIMITER control (not cruise control):
        // - Reduce factor quickly when overspeed is predicted.
        // - Relax factor back slowly only when speed is safely under the limit AND not rising.
        if ((uint32_t)(now - last_control_ms) >= CONTROL_INTERVAL_MS) {
          float dt_s = (float)(now - last_control_ms) / 1000.0f;
          last_control_ms = now;

          float spd_pred = (float)spd + speed_rate_kmh_s * LIMIT_SPEED_LOOKAHEAD_S;
          float err_pred = spd_pred - (float)effective_limit_kmh;

          if (err_pred > SPEED_DEADBAND_KMH) {
            // Predicted overspeed -> cut fast
            limit_factor -= err_pred * FACTOR_RATE_DOWN_PER_KMH_PER_S * dt_s;
          } else if (err_pred < -LIMIT_RELAX_BAND_KMH && speed_rate_kmh_s <= LIMIT_RELAX_MAX_RISE_KMH_PER_S) {
            // Safely under limit and not rising -> relax slowly
            limit_factor += FACTOR_RATE_UP_PER_S * dt_s;
          }

          limit_factor = clampf(limit_factor, 0.0f, 1.0f);
        }
      }
    }

    // Compute desired outputs (ECU-level volts)
    float v1, v2;
    if (limiting) {
      // IMPORTANT: output never higher than the real pedal (driver can always ease off)
      v1 = aps1 * limit_factor;
      v2 = aps2 * limit_factor;

      // Safety floors (avoid invalid sensor lows)
      if (v1 < DEFAULT_SIGNAL_S1_V) v1 = DEFAULT_SIGNAL_S1_V;
      if (v2 < DEFAULT_SIGNAL_S2_V) v2 = DEFAULT_SIGNAL_S2_V;
    } else {
      v1 = aps1;
      v2 = aps2;

      if (v1 < DEFAULT_SIGNAL_S1_V) v1 = DEFAULT_SIGNAL_S1_V;
      if (v2 < DEFAULT_SIGNAL_S2_V) v2 = DEFAULT_SIGNAL_S2_V;
    }

    SharedState_SetDesiredOutputs(v1, v2);

    // Log current speed, APS inputs, and outputs
    int relay_pin_level = digitalRead(RELAY_PIN);
    float spd_pred = (float)spd + speed_rate_kmh_s * LIMIT_SPEED_LOOKAHEAD_S;
    Serial.printf("Speed=%u km/h (SL=%u->%u off<=%u pred=%.1f rate=%.1f F=%.2f), Relay=%s (IO%d=%s), APS_in=(%.3fV, %.3fV), APS_out=(%.3fV, %.3fV)\r\n",
                  spd,
                  (unsigned)speed_limit_kmh,
                  (unsigned)effective_limit_kmh,
                  (unsigned)effective_release_kmh,
                  spd_pred,
                  speed_rate_kmh_s,
                  limit_factor,
                  limiting ? "ON" : "OFF",
                  RELAY_PIN,
                  relay_pin_level == HIGH ? "HIGH" : "LOW",
                  aps1, aps2, v1, v2);

    vTaskDelay(pdMS_TO_TICKS(LOGIC_LOOP_INTERVAL_MS));
  }
}

void LogicModule_Begin() {
  pinMode(RELAY_PIN, OUTPUT);
  setRelayActive(false); // fail-safe at boot

  prefs.begin(PREF_NS, false);
  speed_limit_kmh = prefs.getUShort(PREF_KEY_SL, SPEED_LIMIT_DEFAULT_KMH);

  limiting = false;
  limit_factor = 1.0f;
  last_speed_kmh = 0;
  last_speed_update_ms = 0;
  speed_rate_kmh_s = 0.0f;
  last_control_ms = millis();

  Serial.printf("SpeedLimiter ready. SL=%u km/h\r\n", (unsigned)speed_limit_kmh);
  Serial.println("Set speed limit with: SL=<0..250>");
}

void LogicModule_StartTask() {
  if (g_logic_task) return;

  xTaskCreatePinnedToCore(
      logicTask,
      "LOGIC_TASK",
      4096, // words
      nullptr,
      4,
      &g_logic_task,
      1);
}

#else

void LogicModule_Begin() {}
void LogicModule_StartTask() {}

#endif // !BUILD_TEST_LOGGER

