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
static float throttle_factor = 1.0f; // 1.0 = pass-through, <1 reduces APS outputs
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

    float aps1 = DEFAULT_SIGNAL_S1_V;
    float aps2 = DEFAULT_SIGNAL_S2_V;
    SharedState_GetAps(&aps1, &aps2, nullptr);

    if (!spd_ok || speed_limit_kmh == 0) {
      // Fail-safe: if speed is unknown, do NOT keep relay active.
      limiting = false;
      throttle_factor = 1.0f;
      setRelayActive(false);
    } else if (!limiting) {
      // Not limiting yet: wait until speed reaches the threshold.
      if (spd >= speed_limit_kmh) {
        limiting = true;
        captured_pedal_v1 = aps1;
        captured_pedal_v2 = aps2;
        captured_pedal_avg_mv = mvFromVolts((aps1 + aps2) * 0.5f);
        throttle_factor = 1.0f; // at threshold: output == input
        last_control_ms = now;
        setRelayActive(true);
      } else {
        throttle_factor = 1.0f;
        setRelayActive(false);
      }
    } else {
      // Limiting active.
      uint16_t pedal_avg_mv = mvFromVolts((aps1 + aps2) * 0.5f);

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

          // Asymmetric response:
          // - Reduce throttle faster when over the limit
          // - Increase throttle slower when under the limit (prevents oscillation)
          if (err > SPEED_DEADBAND_KMH) {
            float reduction_rate = FACTOR_RATE_PER_KMH_PER_S * 1.5f;
            throttle_factor -= err * reduction_rate * dt_s;
            throttle_factor = clampf(throttle_factor, 0.0f, 1.0f);
          } else if (err < -SPEED_DEADBAND_KMH) {
            float increase_rate = FACTOR_RATE_PER_KMH_PER_S * 0.4f;
            throttle_factor -= err * increase_rate * dt_s; // err negative => increase
            throttle_factor = clampf(throttle_factor, 0.0f, 1.0f);
          }
        }
      }
    }

    // Compute desired outputs (ECU-level volts)
    float v1, v2;
    if (limiting) {
      v1 = captured_pedal_v1 * throttle_factor;
      v2 = captured_pedal_v2 * throttle_factor;

      float min_v1 =
          (captured_pedal_v1 * 0.3f > DEFAULT_SIGNAL_S1_V) ? captured_pedal_v1 * 0.3f : DEFAULT_SIGNAL_S1_V;
      float min_v2 =
          (captured_pedal_v2 * 0.3f > DEFAULT_SIGNAL_S2_V) ? captured_pedal_v2 * 0.3f : DEFAULT_SIGNAL_S2_V;
      if (v1 < min_v1) v1 = min_v1;
      if (v2 < min_v2) v2 = min_v2;
    } else {
      v1 = aps1 * throttle_factor;
      v2 = aps2 * throttle_factor;

      if (v1 < DEFAULT_SIGNAL_S1_V) v1 = DEFAULT_SIGNAL_S1_V;
      if (v2 < DEFAULT_SIGNAL_S2_V) v2 = DEFAULT_SIGNAL_S2_V;
    }

    SharedState_SetDesiredOutputs(v1, v2);

    vTaskDelay(pdMS_TO_TICKS(LOGIC_LOOP_INTERVAL_MS));
  }
}

void LogicModule_Begin() {
  pinMode(RELAY_PIN, OUTPUT);
  setRelayActive(false); // fail-safe at boot

  prefs.begin(PREF_NS, false);
  speed_limit_kmh = prefs.getUShort(PREF_KEY_SL, SPEED_LIMIT_DEFAULT_KMH);

  limiting = false;
  throttle_factor = 1.0f;
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

