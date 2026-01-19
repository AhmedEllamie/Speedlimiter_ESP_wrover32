// Speed limiter logic module (Corrected Speed Limiter Algorithm - Final Logical Model)
//
// Relay authority model:
// - Relay OFF -> pedal directly controls ECU (MCU output ignored by ECU)
// - Relay ON  -> MCU fully controls ECU APS input
//
// States:
// - PASS_THROUGH: relay OFF, outputs follow pedal (for PWM alignment / visibility)
// - OVERSHOOT_CONTROL: relay ON pre-limit, MCU reduces APS_out based on accel trend (never increases)
// - LIMIT_ACTIVE: at/near limit, hold APS_out; only reduce again if speed rises/overspeeds
// - FAULT: relay OFF, pass-through, recovery via reset/supervisor
//
// IMPORTANT (startup behavior):
// - At boot CAN speed is not valid until the first frame arrives (timestamp == 0).
// - We DO NOT latch FAULT in PASS_THROUGH just because speed isn't valid yet.
// - If speed becomes invalid while the relay is under MCU authority, we latch FAULT (fail-safe).

#include "logic_module.h"

#include "shared_state.h"
#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================== CONFIG ==============================

// Pre-limit activation threshold: (speed_limit - margin) => OVERSHOOT_CONTROL.
// Use the same offset as defined in sl_config.h for consistency.
static constexpr float SPEED_MARGIN_KMH = (float)SPEED_LIMIT_ACTIVATION_OFFSET_KMH;

// Deactivation threshold in LIMIT_ACTIVE: (speed_limit - hysteresis) => PASS_THROUGH.
static constexpr float SPEED_HYSTERESIS_KMH = (float)SPEED_LIMIT_DEACTIVATION_HYSTERESIS_KMH;

// Windowed acceleration config (trend estimate only).
static constexpr uint8_t ACCEL_WINDOW_SAMPLES = 5;

// Acceleration thresholds (km/h/s) AFTER windowing.
static constexpr float FAST_ACCEL_KMH_S = 2.0f;
// NOTE: CAN speed is integer km/h; small dithering can look like non-zero accel.
// This threshold is a deadband to avoid one-way "ratchet down" on noise.
static constexpr float SLOW_ACCEL_KMH_S = 0.5f;

// Treat "near the limit" as "at the limit" due to integer speed steps.
// Example: SL=55 => enter LIMIT_ACTIVE at 54 km/h when accel is stable.
static constexpr float LIMIT_ENTRY_BAND_BELOW_KMH = 5.0f;

// If OVERSHOOT_CONTROL pulls speed down far enough below activation, give back control.
// Example: activation=52 (55-3), release at <=50 (activation-2).
static constexpr float OVERSHOOT_RELEASE_HYSTERESIS_KMH = 10.0f;

// Output decay rates (ECU-level volts per second).
// These preserve the previous "per-10ms-step" behavior while remaining dt-scaled:
//  - FAST: 0.020V per 10ms => 2.0 V/s
//  - SLOW: 0.005V per 10ms => 0.5 V/s
static constexpr float FAST_DECAY_V_PER_S = 2.0f;
static constexpr float SLOW_DECAY_V_PER_S = 0.5f;

// Driver release detection margin (ECU-level volts).
// We release relay when APS_in falls below APS_out by this margin.
static constexpr float APS_RELEASE_DELTA_V = 0.05f;

// Treat pedal as "fully released" when APS is close to the known idle floor.
static constexpr float APS_IDLE_MARGIN_V = 0.05f;

// USB telemetry interval.
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;

// Fixed speed limit for this module (km/h). Can be replaced by CLI/NVS later.
static constexpr float SPEED_LIMIT_KMH = (float)SPEED_LIMIT_DEFAULT_KMH;

// ===================================================================

enum class LimiterState
{
    PASS_THROUGH,
    OVERSHOOT_CONTROL,
    LIMIT_ACTIVE,
    FAULT
};

static TaskHandle_t g_logic_task = nullptr;

static LimiterState state = LimiterState::PASS_THROUGH;
static LimiterState last_state_for_log = LimiterState::PASS_THROUGH;

// Commanded ECU-level APS outputs (latched and edited while relay is ON).
static float aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
static float aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;

// Relay output tracking.
static bool relay_active = false;

// Time tracking (loop dt + telemetry pacing).
static uint32_t prev_time_ms = 0;
static uint32_t last_telemetry_ms = 0;

// State timing for debouncing (prevent rapid oscillations).
static uint32_t state_entry_time_ms = 0;
static LimiterState prev_state_for_timing = LimiterState::PASS_THROUGH;
static constexpr uint32_t MIN_STATE_TIME_MS = 50; // Minimum 50ms in LIMIT_ACTIVE before allowing transitions

// Speed window buffer for windowed acceleration estimate.
static float speed_window_kmh[ACCEL_WINDOW_SAMPLES] = {0.0f};
static uint32_t speed_window_ms[ACCEL_WINDOW_SAMPLES] = {0};
static uint8_t speed_idx = 0;
static bool speed_window_full = false;
static uint32_t last_speed_sample_ms = 0;
static float last_accel_kmh_s = 0.0f;

// ============================== HELPERS =============================

static float maxf(float a, float b) { return (a > b) ? a : b; }
static float absf(float x) { return (x < 0.0f) ? -x : x; }

static void setRelayActive(bool active)
{
    digitalWrite(RELAY_PIN, active ? HIGH : LOW);
}

static const char *stateToString(LimiterState s)
{
    switch (s)
    {
        case LimiterState::PASS_THROUGH:      return "PASS_THROUGH";
        case LimiterState::OVERSHOOT_CONTROL: return "OVERSHOOT_CONTROL";
        case LimiterState::LIMIT_ACTIVE:      return "LIMIT_ACTIVE";
        case LimiterState::FAULT:             return "FAULT";
        default:                              return "UNKNOWN";
    }
}

static void logStateTransitionReason(LimiterState from,
                                     LimiterState to,
                                     const char *reason,
                                     float speed_kmh,
                                     float speed_limit_kmh,
                                     float accel_kmh_s,
                                     uint32_t time_in_state_ms)
{
    Serial.printf(
        "STATE_REASON %s -> %s: %s (speed=%.1f, limit=%.1f, accel=%.2f, time=%ums)\r\n",
        stateToString(from),
        stateToString(to),
        reason,
        speed_kmh,
        speed_limit_kmh,
        accel_kmh_s,
        (unsigned)time_in_state_ms);
}

static void Logic_EnterFault()
{
    state = LimiterState::FAULT;
    relay_active = false;
    setRelayActive(false);
}

static void clearSpeedWindow()
{
    for (uint8_t i = 0; i < ACCEL_WINDOW_SAMPLES; i++)
    {
        speed_window_kmh[i] = 0.0f;
        speed_window_ms[i] = 0;
    }
    speed_idx = 0;
    speed_window_full = false;
    last_speed_sample_ms = 0;
    last_accel_kmh_s = 0.0f;
}

// Windowed accel using CAN speed timestamps (updates only when speed sample updates).
static float ComputeWindowedAccel(float speed_kmh, uint32_t speed_sample_ms)
{
    if (speed_sample_ms == 0 || speed_sample_ms == last_speed_sample_ms)
        return last_accel_kmh_s;

    last_speed_sample_ms = speed_sample_ms;

    speed_window_kmh[speed_idx] = speed_kmh;
    speed_window_ms[speed_idx] = speed_sample_ms;
    speed_idx = (uint8_t)((speed_idx + 1) % ACCEL_WINDOW_SAMPLES);

    if (speed_idx == 0)
        speed_window_full = true;

    if (!speed_window_full)
    {
        last_accel_kmh_s = 0.0f;
        return last_accel_kmh_s;
    }

    // speed_idx now points to the oldest entry (next slot to overwrite).
    uint8_t oldest = speed_idx;
    float speed_old = speed_window_kmh[oldest];
    uint32_t old_ms = speed_window_ms[oldest];

    uint32_t dt_ms = (uint32_t)(speed_sample_ms - old_ms);
    if (dt_ms == 0) dt_ms = 1;

    last_accel_kmh_s = (speed_kmh - speed_old) / ((float)dt_ms / 1000.0f);
    return last_accel_kmh_s;
}

static void clampCmdToFloors()
{
    if (aps_cmd_v1 < DEFAULT_SIGNAL_S1_V) aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
    if (aps_cmd_v2 < DEFAULT_SIGNAL_S2_V) aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;
}

static bool pedalFullyReleased(float aps_in_v1, float aps_in_v2)
{
    return (aps_in_v1 <= (DEFAULT_SIGNAL_S1_V + APS_IDLE_MARGIN_V)) &&
           (aps_in_v2 <= (DEFAULT_SIGNAL_S2_V + APS_IDLE_MARGIN_V));
}

static bool driverOverrideRelease(float aps_in_v1, float aps_in_v2)
{
    // Driver is requesting LESS torque than our current commanded output.
    // This can happen when we latched aps_cmd and the driver lifts.
    //
    // IMPORTANT:
    // We should NEVER output more than the driver is requesting.
    // Instead of dropping relay authority (which can cause PASS_THROUGH <-> OVERSHOOT chatter
    // while speed is still above activation), we use this signal to clamp aps_cmd downward.
    return (aps_in_v1 < (aps_cmd_v1 - APS_RELEASE_DELTA_V)) ||
           (aps_in_v2 < (aps_cmd_v2 - APS_RELEASE_DELTA_V));
}

static void applyAccelBasedDecay(float accel_kmh_s, float dt_s, bool force_slow_cut)
{
    float dv = 0.0f;

    if (accel_kmh_s > FAST_ACCEL_KMH_S)
        dv = FAST_DECAY_V_PER_S * dt_s;
    else if (accel_kmh_s > SLOW_ACCEL_KMH_S || force_slow_cut)
        dv = SLOW_DECAY_V_PER_S * dt_s;

    if (dv > 0.0f)
    {
        // Never increase APS_out in limiter-controlled states.
        aps_cmd_v1 -= dv;
        aps_cmd_v2 -= dv;
    }
}

// ==========================================================

void LogicModule_Begin()
{
    // Initialization (fail-safe)
    state = LimiterState::PASS_THROUGH;
    last_state_for_log = state;
    relay_active = false;
    aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
    aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;

    prev_time_ms = millis();
    last_telemetry_ms = prev_time_ms;
    state_entry_time_ms = prev_time_ms;
    prev_state_for_timing = state;

    pinMode(RELAY_PIN, OUTPUT);
    setRelayActive(false); // fail-safe at boot

    clearSpeedWindow();
}

// ==========================================================

void LogicModule_Update(float speed_limit_kmh)
{
    // ---------------- Compute dt ----------------
    uint32_t now_ms = millis();
    float dt_s = (float)(now_ms - prev_time_ms) / 1000.0f;
    if (dt_s <= 0.0f) dt_s = 0.001f;
    prev_time_ms = now_ms;

    // ---------------- Read inputs ----------------
    float aps_in_v1 = DEFAULT_SIGNAL_S1_V;
    float aps_in_v2 = DEFAULT_SIGNAL_S2_V;
    uint32_t aps_ts = 0;
    SharedState_GetAps(&aps_in_v1, &aps_in_v2, &aps_ts);

    // Clamp APS inputs to known safety floors (ECU-level volts).
    aps_in_v1 = maxf(aps_in_v1, DEFAULT_SIGNAL_S1_V);
    aps_in_v2 = maxf(aps_in_v2, DEFAULT_SIGNAL_S2_V);

    float speed_kmh = (float)g_speed_kmh;
    uint32_t speed_sample_ms = g_speed_last_update_ms;

    bool speed_valid = SharedState_SpeedValid(now_ms, SPEED_TIMEOUT_MS);
    if (!speed_valid)
    {
        // Fail-safe: never let MCU hold authority without valid speed feedback.
        // But do NOT latch FAULT in PASS_THROUGH at startup (speed isn't valid yet).
        if (state != LimiterState::PASS_THROUGH)
        {
            logStateTransitionReason(
                state,
                LimiterState::FAULT,
                "speed invalid while relay under MCU authority",
                speed_kmh,
                speed_limit_kmh,
                0.0f,
                0);
            Logic_EnterFault();
        }
    }

    // RPM is for logging only.
    uint16_t rpm = (uint16_t)g_rpm;

    // ---------------- Compute windowed acceleration ----------------
    float accel_kmh_s = ComputeWindowedAccel(speed_kmh, speed_sample_ms);

    // ---------------- State machine ----------------
    // Track state entry time for debouncing
    if (state != prev_state_for_timing)
    {
        state_entry_time_ms = now_ms;
        prev_state_for_timing = state;
    }
    uint32_t time_in_state_ms = now_ms - state_entry_time_ms;

    switch (state)
    {
        case LimiterState::PASS_THROUGH:
        {
            // Relay OFF => ECU sees pedal directly.
            relay_active = false;
            setRelayActive(false);

            // Keep PWM aligned with pedal for visibility / bumpless transfer.
            SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);

            // Track current input as our "would-be" command.
            aps_cmd_v1 = aps_in_v1;
            aps_cmd_v2 = aps_in_v2;

            // Pre-limit activation: close relay slightly before reaching the limit.
            // Only if speed is valid (avoid using stale/invalid speed at boot).
            if (speed_valid && speed_kmh >= (speed_limit_kmh - SPEED_MARGIN_KMH))
            {
                logStateTransitionReason(
                    state,
                    LimiterState::OVERSHOOT_CONTROL,
                    "pre-limit activation (speed>=limit-margin)",
                    speed_kmh,
                    speed_limit_kmh,
                    accel_kmh_s,
                    time_in_state_ms);
                Serial.printf("STATE_DETAIL margin_kmh=%.1f\r\n", SPEED_MARGIN_KMH);

                // Bumpless capture.
                aps_cmd_v1 = aps_in_v1;
                aps_cmd_v2 = aps_in_v2;
                clampCmdToFloors();

                // Publish captured output first, then take relay control.
                SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);

                relay_active = true;
                setRelayActive(true);

                state = LimiterState::OVERSHOOT_CONTROL;
            }
            break;
        }

        case LimiterState::OVERSHOOT_CONTROL:
        {
            // Relay ON => MCU controls ECU APS.
            relay_active = true;
            setRelayActive(true);

            bool overspeed = (speed_kmh > speed_limit_kmh);
            bool speed_falling = (accel_kmh_s < -SLOW_ACCEL_KMH_S);

            // Full pedal release => immediate return to pass-through.
            if (pedalFullyReleased(aps_in_v1, aps_in_v2))
            {
                logStateTransitionReason(
                    state,
                    LimiterState::PASS_THROUGH,
                    "pedal fully released",
                    speed_kmh,
                    speed_limit_kmh,
                    accel_kmh_s,
                    time_in_state_ms);

                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
                SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
                break;
            }

            // If we undershoot well below the activation threshold, release authority.
            // This prevents getting "stuck" at the minimum clamp when the accel estimate is noisy.
            float activation_kmh = speed_limit_kmh - SPEED_MARGIN_KMH;
            float release_kmh = activation_kmh - OVERSHOOT_RELEASE_HYSTERESIS_KMH;
            if (release_kmh < 0.0f) release_kmh = 0.0f;
            if (speed_kmh <= release_kmh)
            {
                logStateTransitionReason(
                    state,
                    LimiterState::PASS_THROUGH,
                    "underspeed release (speed<=release)",
                    speed_kmh,
                    speed_limit_kmh,
                    accel_kmh_s,
                    time_in_state_ms);
                Serial.printf("STATE_DETAIL release_kmh=%.1f activation_kmh=%.1f\r\n",
                              release_kmh,
                              activation_kmh);

                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
                SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
                break;
            }

            // If we are below the limit and decelerating, return authority to the driver.
            if ((speed_kmh < speed_limit_kmh) && speed_falling)
            {
                logStateTransitionReason(
                    state,
                    LimiterState::PASS_THROUGH,
                    "below limit and decelerating",
                    speed_kmh,
                    speed_limit_kmh,
                    accel_kmh_s,
                    time_in_state_ms);

                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
                SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
                break;
            }

            // Driver asks for LESS than our current output:
            // keep relay authority, but clamp aps_cmd down to match driver input (never increases).
            if (driverOverrideRelease(aps_in_v1, aps_in_v2))
            {
                aps_cmd_v1 = aps_in_v1;
                aps_cmd_v2 = aps_in_v2;
                clampCmdToFloors();
                SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
                break;
            }

            // Accel-based trimming (never increase in this state).
            // If we're already above the limit, force at least a slow cut regardless of accel estimate.
            applyAccelBasedDecay(accel_kmh_s, dt_s, overspeed /*force_slow_cut*/);
            clampCmdToFloors();

            SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);

            // Transition to LIMIT_ACTIVE when we're at/near the limit and stable.
            // Use a tighter band (2 km/h instead of 5) and require stable acceleration to prevent oscillation.
            // Also require minimum time in OVERSHOOT_CONTROL to prevent rapid transitions.
            bool near_limit = (speed_kmh >= (speed_limit_kmh - 2.0f)) && (speed_kmh <= (speed_limit_kmh + 0.5f));
            bool stable_accel = absf(accel_kmh_s) <= SLOW_ACCEL_KMH_S;
            bool min_time_elapsed = time_in_state_ms >= MIN_STATE_TIME_MS; // Full debounce time for entry
            
            if (!overspeed && near_limit && stable_accel && min_time_elapsed)
            {
                logStateTransitionReason(
                    state,
                    LimiterState::LIMIT_ACTIVE,
                    "enter limit_active (near && stable && min_time)",
                    speed_kmh,
                    speed_limit_kmh,
                    accel_kmh_s,
                    time_in_state_ms);
                Serial.printf("STATE_DETAIL near_limit=%d stable_accel=%d min_time_elapsed=%d\r\n",
                              near_limit ? 1 : 0,
                              stable_accel ? 1 : 0,
                              min_time_elapsed ? 1 : 0);
                state = LimiterState::LIMIT_ACTIVE;
            }
            break;
        }

        case LimiterState::LIMIT_ACTIVE:
        {
            // Relay remains ON.
            relay_active = true;
            setRelayActive(true);

            bool speed_falling = (accel_kmh_s < -SLOW_ACCEL_KMH_S);

            // LIMIT_ACTIVE is a freeze/hold state: no decay is applied here.
            // If we actually overspeed, jump back to OVERSHOOT_CONTROL where cuts are applied.
            // Add small deadband to prevent oscillation: only transition if significantly overspeed
            // AND we've been in this state long enough (debouncing).
            if (speed_kmh > (speed_limit_kmh + 0.5f) && time_in_state_ms >= MIN_STATE_TIME_MS)
            {
                logStateTransitionReason(
                    state,
                    LimiterState::OVERSHOOT_CONTROL,
                    "overspeed re-entry (speed>limit+0.5, min_time)",
                    speed_kmh,
                    speed_limit_kmh,
                    accel_kmh_s,
                    time_in_state_ms);
                Serial.printf("STATE_DETAIL min_time_ms=%u\r\n", (unsigned)MIN_STATE_TIME_MS);
                state = LimiterState::OVERSHOOT_CONTROL;
                SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
                break;
            }

            // Full pedal release => return to pass-through.
            if (pedalFullyReleased(aps_in_v1, aps_in_v2))
            {
                logStateTransitionReason(
                    state,
                    LimiterState::PASS_THROUGH,
                    "pedal fully released",
                    speed_kmh,
                    speed_limit_kmh,
                    accel_kmh_s,
                    time_in_state_ms);

                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
                SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
                break;
            }

            // If speed is decreasing, release authority back to the driver.
            if (speed_falling)
            {
                logStateTransitionReason(
                    state,
                    LimiterState::PASS_THROUGH,
                    "decelerating",
                    speed_kmh,
                    speed_limit_kmh,
                    accel_kmh_s,
                    time_in_state_ms);

                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
                SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
                break;
            }

            // Driver asks for LESS than our current output:
            // stay in LIMIT_ACTIVE, but clamp aps_cmd down to match driver input (never increases).
            if (driverOverrideRelease(aps_in_v1, aps_in_v2))
            {
                aps_cmd_v1 = aps_in_v1;
                aps_cmd_v2 = aps_in_v2;
                clampCmdToFloors();
                SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
                break;
            }

            // Exit when speed is safely below limit (hysteresis).
            // Add debouncing: must be below threshold for minimum time to prevent oscillation.
            if (speed_kmh <= (speed_limit_kmh - SPEED_HYSTERESIS_KMH) && time_in_state_ms >= MIN_STATE_TIME_MS)
            {
                logStateTransitionReason(
                    state,
                    LimiterState::PASS_THROUGH,
                    "below hysteresis (speed<=limit-hyst, min_time)",
                    speed_kmh,
                    speed_limit_kmh,
                    accel_kmh_s,
                    time_in_state_ms);
                Serial.printf("STATE_DETAIL hysteresis_kmh=%.1f min_time_ms=%u\r\n",
                              SPEED_HYSTERESIS_KMH,
                              (unsigned)MIN_STATE_TIME_MS);

                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
                SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
                break;
            }

            SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
            break;
        }

        case LimiterState::FAULT:
        default:
        {
            // Fail-safe: relay OFF, pedal directly to ECU.
            relay_active = false;
            setRelayActive(false);
            SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
            break;
        }
    }

    // Print state transitions immediately (telemetry is throttled, but state changes can be fast).
    if (state != last_state_for_log)
    {
        Serial.printf("STATE %s -> %s\r\n", stateToString(last_state_for_log), stateToString(state));
        last_state_for_log = state;
    }

    // ---------------- USB telemetry output ----------------
    if ((uint32_t)(now_ms - last_telemetry_ms) >= TELEMETRY_INTERVAL_MS)
    {
        last_telemetry_ms = now_ms;

        float out_v1 = relay_active ? aps_cmd_v1 : aps_in_v1;
        float out_v2 = relay_active ? aps_cmd_v2 : aps_in_v2;

        Serial.printf(
            "Speed=%.1f km/h, RPM=%u, APS_in=(%.3fV,%.3fV), APS_out=(%.3fV,%.3fV), Relay=%s, State=%s, Accel=%.2f km/h/s\r\n",
            speed_kmh,
            (unsigned)rpm,
            aps_in_v1, aps_in_v2,
            out_v1, out_v2,
            relay_active ? "ON" : "OFF",
            stateToString(state),
            accel_kmh_s);
    }
}

// ==========================================================

static void logicTask(void *param)
{
    (void)param;

    for (;;)
    {
        LogicModule_Update(SPEED_LIMIT_KMH);
        vTaskDelay(pdMS_TO_TICKS(LOGIC_LOOP_INTERVAL_MS));
    }
}

// ==========================================================

void LogicModule_StartTask()
{
    if (g_logic_task) return;

    xTaskCreatePinnedToCore(
        logicTask,
        "LOGIC_TASK",
        2048,
        nullptr,
        4,
        &g_logic_task,
        0);
}

// ==========================================================

bool LogicModule_IsRelayActive()
{
    return relay_active;
}

// ==========================================================

#else // BUILD_TEST_LOGGER

// Logger firmware build: logic module is disabled.
void LogicModule_Begin() {}
void LogicModule_StartTask() {}
bool LogicModule_IsRelayActive() { return false; }

#endif // !BUILD_TEST_LOGGER
