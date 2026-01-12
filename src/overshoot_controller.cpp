#include "overshoot_controller.h"

#include "sl_config.h"

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float maxf(float a, float b) { return (a > b) ? a : b; }

static float mvToVolts(uint16_t mv) { return (float)mv / 1000.0f; }

// -----------------------------------------------------------------------------
// Overshoot controller tuning (new algorithm)
// -----------------------------------------------------------------------------
// NOTE: These are intentionally conservative defaults. Tune on-road using logs.
static const float OC_SPEED_RATE_LPF_ALPHA = 0.90f; // 0..1 (higher = more smoothing)

// "No increase" threshold for rule #6.
static const float OC_STABLE_MAX_RISE_KMH_PER_S = 0.05f;

// Threshold between "slowly" and "harshly" (rules #4/#5).
static const float OC_HARSH_RISE_KMH_PER_S = 3.0f;

// Output cut rates (fraction of current output per second).
static const float OC_CUT_RATE_SLOW_PER_S = 0.60f;
static const float OC_CUT_RATE_FAST_PER_S = 2.20f;

// Driver override (rule #7): release relay when APS_in goes below APS_out.
// A small margin avoids noise causing relay chatter.
static const uint16_t OC_DRIVER_RELEASE_MARGIN_MV = 10;

void OvershootController_Reset(OvershootControllerState *st, uint32_t now_ms) {
  if (!st) return;

  st->active = false;
  st->out_v1 = DEFAULT_SIGNAL_S1_V;
  st->out_v2 = DEFAULT_SIGNAL_S2_V;
  st->last_speed_kmh = 0;
  st->last_speed_update_ms = 0;
  st->speed_rate_kmh_s = 0.0f;
  st->last_control_ms = now_ms;
}

void OvershootController_Update(OvershootControllerState *st,
                                uint32_t now_ms,
                                bool speed_valid,
                                uint16_t speed_kmh,
                                uint32_t speed_update_ms,
                                uint16_t speed_limit_kmh,
                                float aps1_v,
                                float aps2_v,
                                OvershootControllerResult *out) {
  if (!st || !out) return;

  // Clamp APS inputs to safety floors.
  float in_v1 = maxf(aps1_v, DEFAULT_SIGNAL_S1_V);
  float in_v2 = maxf(aps2_v, DEFAULT_SIGNAL_S2_V);

  // Compute activation threshold (Speed_limit - SpeedMargin).
  uint16_t target_limit_kmh = speed_limit_kmh;
  uint16_t activation_kmh = speed_limit_kmh;
  if (speed_limit_kmh > SPEED_LIMIT_ACTIVATION_OFFSET_KMH) {
    activation_kmh = (uint16_t)(speed_limit_kmh - SPEED_LIMIT_ACTIVATION_OFFSET_KMH);
  }

  // Update speed derivative ONLY when a new speed sample arrives.
  if (speed_update_ms != 0 && speed_update_ms != st->last_speed_update_ms) {
    if (st->last_speed_update_ms != 0 && speed_update_ms > st->last_speed_update_ms) {
      float dt_s = (float)(speed_update_ms - st->last_speed_update_ms) / 1000.0f;
      if (dt_s > 0.001f) {
        float inst_rate = ((float)speed_kmh - (float)st->last_speed_kmh) / dt_s;
        st->speed_rate_kmh_s =
            (st->speed_rate_kmh_s * OC_SPEED_RATE_LPF_ALPHA) + (inst_rate * (1.0f - OC_SPEED_RATE_LPF_ALPHA));
      }
    }
    st->last_speed_kmh = speed_kmh;
    st->last_speed_update_ms = speed_update_ms;
  }

  // Default output (may be overwritten below).
  out->active = false;
  out->relay_active = false;
  out->out_v1 = in_v1;
  out->out_v2 = in_v2;
  out->activation_kmh = activation_kmh;
  out->target_limit_kmh = target_limit_kmh;
  out->speed_rate_kmh_s = st->speed_rate_kmh_s;
  out->applied_cut_rate_per_s = 0.0f;

  // Fail-safe: invalid speed or disabled limit => relay OFF + pass-through.
  if (!speed_valid || speed_limit_kmh == 0) {
    st->active = false;
    st->out_v1 = in_v1;
    st->out_v2 = in_v2;
    st->last_control_ms = now_ms;
    return;
  }

  if (!st->active) {
    // Not active yet: arm at (limit - margin).
    if (speed_kmh >= activation_kmh) {
      st->active = true;
      st->out_v1 = in_v1; // capture exactly current inputs (rule #3)
      st->out_v2 = in_v2;
      st->last_control_ms = now_ms;
    } else {
      // Pass-through.
      st->out_v1 = in_v1;
      st->out_v2 = in_v2;
    }
  } else {
    // Active: apply rule #7 (driver override) continuously.
    float rel_margin_v = mvToVolts(OC_DRIVER_RELEASE_MARGIN_MV);
    if ((in_v1 + rel_margin_v) < st->out_v1 || (in_v2 + rel_margin_v) < st->out_v2) {
      st->active = false;
      st->out_v1 = in_v1;
      st->out_v2 = in_v2;
      st->last_control_ms = now_ms;
    } else {
      // Keep relay active while limiting.
      // Apply output edits at a fixed interval.
      if ((uint32_t)(now_ms - st->last_control_ms) >= CONTROL_INTERVAL_MS) {
        float dt_s = (float)(now_ms - st->last_control_ms) / 1000.0f;
        st->last_control_ms = now_ms;

        float cut_rate = 0.0f;

        // Rule #6: if speed is at/below limit and not increasing -> hold (no change).
        // Rule #4/#5: if still accelerating -> cut (fast or slow).
        if (speed_kmh > target_limit_kmh) {
          // Already over the limit -> ensure we cut (prevents "stuck above limit").
          cut_rate = OC_CUT_RATE_FAST_PER_S;
        } else if (st->speed_rate_kmh_s > OC_STABLE_MAX_RISE_KMH_PER_S) {
          // Still accelerating -> cut based on how harsh acceleration is.
          cut_rate = (st->speed_rate_kmh_s >= OC_HARSH_RISE_KMH_PER_S) ? OC_CUT_RATE_FAST_PER_S
                                                                       : OC_CUT_RATE_SLOW_PER_S;
        } else {
          cut_rate = 0.0f; // hold
        }

        if (cut_rate > 0.0f && dt_s > 0.0f) {
          float scale = 1.0f - cut_rate * dt_s;
          scale = clampf(scale, 0.0f, 1.0f);
          st->out_v1 *= scale;
          st->out_v2 *= scale;
        }

        // Safety floors.
        if (st->out_v1 < DEFAULT_SIGNAL_S1_V) st->out_v1 = DEFAULT_SIGNAL_S1_V;
        if (st->out_v2 < DEFAULT_SIGNAL_S2_V) st->out_v2 = DEFAULT_SIGNAL_S2_V;

        out->applied_cut_rate_per_s = cut_rate;
      }
    }
  }

  // Publish final state.
  if (st->active) {
    out->active = true;
    out->relay_active = true;
    out->out_v1 = st->out_v1;
    out->out_v2 = st->out_v2;
  } else {
    out->active = false;
    out->relay_active = false;
    out->out_v1 = in_v1;
    out->out_v2 = in_v2;
  }
}

