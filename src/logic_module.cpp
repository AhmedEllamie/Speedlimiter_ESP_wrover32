#include "logic_module.h"

#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>
#include <Preferences.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/task.h"

#include "overshoot_controller.h"
#include "shared_state.h"

static TaskHandle_t g_logic_task = nullptr;

static Preferences prefs;
static uint16_t speed_limit_kmh = SPEED_LIMIT_DEFAULT_KMH;

static OvershootControllerState g_oc;

// USB CLI buffer (only: SL=<kmh>)
static char cli_buf[32];
static uint8_t cli_len = 0;

static void setRelayActive(bool active) {
  digitalWrite(RELAY_PIN, active ? HIGH : LOW);
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

    float aps1 = DEFAULT_SIGNAL_S1_V;
    float aps2 = DEFAULT_SIGNAL_S2_V;
    SharedState_GetAps(&aps1, &aps2, nullptr);

    OvershootControllerResult oc;
    OvershootController_Update(&g_oc,
                               now,
                               spd_ok,
                               spd,
                               spd_update_ms,
                               speed_limit_kmh,
                               aps1,
                               aps2,
                               &oc);

    setRelayActive(oc.relay_active);
    SharedState_SetDesiredOutputs(oc.out_v1, oc.out_v2);

    // Log current speed, APS inputs, outputs, and overshoot controller status.
    int relay_pin_level = digitalRead(RELAY_PIN);
    Serial.printf(
        "Speed=%u km/h (SL=%u act@%u rate=%.2f cut=%.2f/s), OC=%s, Relay=%s (IO%d=%s), APS_in=(%.3fV, %.3fV), "
        "APS_out=(%.3fV, %.3fV)\r\n",
        (unsigned)spd,
        (unsigned)oc.target_limit_kmh,
        (unsigned)oc.activation_kmh,
        oc.speed_rate_kmh_s,
        oc.applied_cut_rate_per_s,
        oc.active ? "ON" : "OFF",
        oc.relay_active ? "ON" : "OFF",
        RELAY_PIN,
        relay_pin_level == HIGH ? "HIGH" : "LOW",
        aps1,
        aps2,
        oc.out_v1,
        oc.out_v2);

    vTaskDelay(pdMS_TO_TICKS(LOGIC_LOOP_INTERVAL_MS));
  }
}

void LogicModule_Begin() {
  pinMode(RELAY_PIN, OUTPUT);
  setRelayActive(false); // fail-safe at boot

  prefs.begin(PREF_NS, false);
  speed_limit_kmh = prefs.getUShort(PREF_KEY_SL, SPEED_LIMIT_DEFAULT_KMH);

  OvershootController_Reset(&g_oc, millis());

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

