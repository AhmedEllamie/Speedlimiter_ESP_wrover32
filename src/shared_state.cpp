#include "shared_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

volatile uint8_t g_speed_kmh = 0;
volatile uint32_t g_speed_last_update_ms = 0;

volatile float g_aps1_v = 0.0f;
volatile float g_aps2_v = 0.0f;
volatile uint32_t g_aps_last_update_ms = 0;

volatile float g_sdps1_v = 0.0f;
volatile float g_sdps2_v = 0.0f;

static portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

void SharedState_SetSpeed(uint8_t kmh, uint32_t now_ms) {
  // Intentionally lock-free: 8-bit + 32-bit stores are atomic on ESP32, and
  // speed consumers tolerate minor skew between value and timestamp.
  g_speed_kmh = kmh;
  g_speed_last_update_ms = now_ms;
}

bool SharedState_SpeedValid(uint32_t now_ms, uint32_t timeout_ms) {
  uint32_t last = g_speed_last_update_ms;
  if (last == 0) return false;
  return (uint32_t)(now_ms - last) <= timeout_ms;
}

void SharedState_SetAps(float v1, float v2, uint32_t now_ms) {
  portENTER_CRITICAL(&g_state_mux);
  g_aps1_v = v1;
  g_aps2_v = v2;
  g_aps_last_update_ms = now_ms;
  portEXIT_CRITICAL(&g_state_mux);
}

void SharedState_GetAps(float *v1, float *v2, uint32_t *last_ms) {
  if (!v1 || !v2) return;
  portENTER_CRITICAL(&g_state_mux);
  *v1 = g_aps1_v;
  *v2 = g_aps2_v;
  if (last_ms) *last_ms = g_aps_last_update_ms;
  portEXIT_CRITICAL(&g_state_mux);
}

void SharedState_SetDesiredOutputs(float v1, float v2) {
  portENTER_CRITICAL(&g_state_mux);
  g_sdps1_v = v1;
  g_sdps2_v = v2;
  portEXIT_CRITICAL(&g_state_mux);
}

void SharedState_GetDesiredOutputs(float *v1, float *v2) {
  if (!v1 || !v2) return;
  portENTER_CRITICAL(&g_state_mux);
  *v1 = g_sdps1_v;
  *v2 = g_sdps2_v;
  portEXIT_CRITICAL(&g_state_mux);
}

