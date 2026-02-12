/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#include "shared_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

volatile uint8_t g_speed_kmh = 0;
volatile uint32_t g_speed_last_update_ms = 0;

// 0 = disabled until loaded from Preferences (fail-safe).
volatile uint16_t g_speed_limit_kmh = 0;

// 1 = active (relay ON / limiting), 0 = inactive.
volatile uint8_t g_speed_limiter_active = 0;

// Manual override flags (used in MANUAL mode for relay control).
volatile uint8_t g_manual_override_enabled = 0;
volatile uint8_t g_manual_override_relay_on = 0;

volatile uint16_t g_rpm = 0;
volatile uint32_t g_rpm_last_update_ms = 0;

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

void SharedState_SetSpeedLimitKmh(uint16_t kmh) {
  portENTER_CRITICAL(&g_state_mux);
  g_speed_limit_kmh = kmh;
  portEXIT_CRITICAL(&g_state_mux);
}

uint16_t SharedState_GetSpeedLimitKmh() {
  portENTER_CRITICAL(&g_state_mux);
  uint16_t v = g_speed_limit_kmh;
  portEXIT_CRITICAL(&g_state_mux);
  return v;
}

void SharedState_SetLimiterActive(bool active) {
  portENTER_CRITICAL(&g_state_mux);
  g_speed_limiter_active = active ? 1 : 0;
  portEXIT_CRITICAL(&g_state_mux);
}

bool SharedState_IsLimiterActive() {
  portENTER_CRITICAL(&g_state_mux);
  bool v = (g_speed_limiter_active != 0);
  portEXIT_CRITICAL(&g_state_mux);
  return v;
}

void SharedState_SetManualOverrideEnabled(bool enabled) {
  portENTER_CRITICAL(&g_state_mux);
  g_manual_override_enabled = enabled ? 1 : 0;
  portEXIT_CRITICAL(&g_state_mux);
}

bool SharedState_GetManualOverrideEnabled() {
  portENTER_CRITICAL(&g_state_mux);
  bool v = (g_manual_override_enabled != 0);
  portEXIT_CRITICAL(&g_state_mux);
  return v;
}

void SharedState_SetManualOverrideRelay(bool on) {
  portENTER_CRITICAL(&g_state_mux);
  g_manual_override_relay_on = on ? 1 : 0;
  portEXIT_CRITICAL(&g_state_mux);
}

bool SharedState_GetManualOverrideRelay() {
  portENTER_CRITICAL(&g_state_mux);
  bool v = (g_manual_override_relay_on != 0);
  portEXIT_CRITICAL(&g_state_mux);
  return v;
}

void SharedState_SetRpm(uint16_t rpm, uint32_t now_ms) {
  g_rpm = rpm;
  g_rpm_last_update_ms = now_ms;
}

bool SharedState_RpmValid(uint32_t now_ms, uint32_t timeout_ms) {
  uint32_t last = g_rpm_last_update_ms;
  if (last == 0) return false;
  return (uint32_t)(now_ms - last) <= timeout_ms;
}

uint16_t SharedState_GetRpm() {
  return g_rpm;
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

