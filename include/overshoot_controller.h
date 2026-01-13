#pragma once

#include <stdint.h>

// Overshoot controller (speed limiter logic)
//
// New algorithm summary:
// - When speed reaches (speed_limit - margin) -> latch ("capture") current APS input and activate relay.
// - While active:
//   - If vehicle still accelerating: reduce the *latched* output quickly (harsh) or slowly based on acceleration.
//   - If speed is at the limit and not increasing: hold the latched output (no increase).
//   - If driver eases off (APS_in < APS_out): deactivate relay immediately and return to pass-through.
// - Fail-safe: if speed is invalid or SL=0 -> relay OFF and pass-through.

struct OvershootControllerState {
  bool active = false;

  // Latched / edited output voltages (ECU-level volts).
  float out_v1 = 0.0f;
  float out_v2 = 0.0f;

  // Speed derivative tracking (filtered d(speed)/dt).
  uint16_t last_speed_kmh = 0;
  uint32_t last_speed_update_ms = 0;
  float speed_rate_kmh_s = 0.0f;

  uint32_t last_control_ms = 0;
  uint32_t activation_time_ms = 0; // When we first activated (for hold period)
};

struct OvershootControllerResult {
  bool active = false;
  bool relay_active = false;

  // Output ECU-level volts (what PWM/DAC should generate).
  float out_v1 = 0.0f;
  float out_v2 = 0.0f;

  // For logging/telemetry.
  uint16_t activation_kmh = 0;
  uint16_t release_kmh = 0;
  uint16_t target_limit_kmh = 0;
  float speed_rate_kmh_s = 0.0f;
  float applied_cut_rate_per_s = 0.0f;
};

void OvershootController_Reset(OvershootControllerState *st, uint32_t now_ms);

void OvershootController_Update(OvershootControllerState *st,
                                uint32_t now_ms,
                                bool speed_valid,
                                uint16_t speed_kmh,
                                uint32_t speed_update_ms,
                                uint16_t speed_limit_kmh,
                                float aps1_v,
                                float aps2_v,
                                OvershootControllerResult *out);

