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
static constexpr float SPEED_MARGIN_KMH = 3.0f;

// Deactivation threshold in LIMIT_ACTIVE: (speed_limit - hysteresis) => PASS_THROUGH.
static constexpr float SPEED_HYSTERESIS_KMH = (float)SPEED_LIMIT_DEACTIVATION_HYSTERESIS_KMH;

// Windowed acceleration config (trend estimate only).
static constexpr uint8_t ACCEL_WINDOW_SAMPLES = 10;

// Acceleration thresholds (km/h/s) AFTER windowing.
static constexpr float FAST_ACCEL_KMH_S = 2.0f;
static constexpr float SLOW_ACCEL_KMH_S = 0.3f;

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

// Commanded ECU-level APS outputs (latched and edited while relay is ON).
static float aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
static float aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;

// Relay output tracking.
static bool relay_active = false;

// Time tracking (loop dt + telemetry pacing).
static uint32_t prev_time_ms = 0;
static uint32_t last_telemetry_ms = 0;

// Speed window buffer for windowed acceleration estimate.
static float speed_window_kmh[ACCEL_WINDOW_SAMPLES] = {0.0f};
static uint32_t speed_window_ms[ACCEL_WINDOW_SAMPLES] = {0};
static uint8_t speed_idx = 0;
static bool speed_window_full = false;
static uint32_t last_speed_sample_ms = 0;
static float last_accel_kmh_s = 0.0f;

// ============================== HELPERS =============================

static float maxf(float a, float b) { return (a > b) ? a : b; }

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
    // Release relay when driver's current demand is lower than our commanded output.
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
    relay_active = false;
    aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
    aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;

    prev_time_ms = millis();
    last_telemetry_ms = prev_time_ms;

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

    bool speed_valid = SharedState_SpeedValid(now_ms, SPEED_TIMEOUT_MS);
    if (!speed_valid)
    {
        // Fail-safe: never let MCU hold authority without valid speed feedback.
        // But do NOT latch FAULT in PASS_THROUGH at startup (speed isn't valid yet).
        if (state != LimiterState::PASS_THROUGH)
        {
            Logic_EnterFault();
        }
    }

    float speed_kmh = (float)g_speed_kmh;
    uint32_t speed_sample_ms = g_speed_last_update_ms;

    // RPM is for logging only.
    uint16_t rpm = (uint16_t)g_rpm;

    // ---------------- Compute windowed acceleration ----------------
    float accel_kmh_s = ComputeWindowedAccel(speed_kmh, speed_sample_ms);

    // ---------------- State machine ----------------
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

            // Driver releases pedal => immediate return to pass-through.
            if (pedalFullyReleased(aps_in_v1, aps_in_v2) || driverOverrideRelease(aps_in_v1, aps_in_v2))
            {
                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
                SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
                break;
            }

            // Accel-based trimming (never increase in this state).
            applyAccelBasedDecay(accel_kmh_s, dt_s, false /*force_slow_cut*/);
            clampCmdToFloors();

            SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);

            // Reached the limit => hold torque in LIMIT_ACTIVE.
            if (speed_kmh >= speed_limit_kmh)
            {
                state = LimiterState::LIMIT_ACTIVE;
            }
            break;
        }

        case LimiterState::LIMIT_ACTIVE:
        {
            // Relay remains ON.
            relay_active = true;
            setRelayActive(true);

            // Exit on driver release.
            if (pedalFullyReleased(aps_in_v1, aps_in_v2) || driverOverrideRelease(aps_in_v1, aps_in_v2))
            {
                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
                SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
                break;
            }

            // Exit when speed is safely below limit (hysteresis).
            if (speed_kmh < (speed_limit_kmh - SPEED_HYSTERESIS_KMH))
            {
                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
                SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
                break;
            }

            // Freeze APS_out at the hold value. Only reduce again if speed rises/overspeeds.
            bool overspeed = (speed_kmh > speed_limit_kmh);
            bool rising = (accel_kmh_s > SLOW_ACCEL_KMH_S);

            if (overspeed || rising)
            {
                // If overspeed, force at least a slow cut even if accel estimate is ~0.
                applyAccelBasedDecay(accel_kmh_s, dt_s, overspeed /*force_slow_cut*/);
                clampCmdToFloors();
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
