// ==========================================================
// Speed Limiter Logic Module — FINAL STABLE VERSION
// ==========================================================
//
// Key properties:
// - Relay-based APS authority
// - Pre-limit capture
// - Windowed accel
// - HARD FREEZE band in LIMIT_ACTIVE
// - Never increases throttle
// - Stable with 2 km/h CAN speed quantization
//
// ==========================================================

#include "logic_module.h"
#include "shared_state.h"
#include "sl_config.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================== PARAMETERS ==============================

// ---- Speed control ----
static constexpr float SPEED_LIMIT_KMH        = 55.0f;
static constexpr float SPEED_MARGIN_KMH       = 3.0f;   // enter OVERSHOOT at 52
static constexpr float SPEED_DEADBAND_KMH     = 1.0f;   // ±1 km/h freeze band

// ---- Acceleration thresholds (windowed) ----
static constexpr float FAST_ACCEL_KMH_S       = 1.5f;
static constexpr float SLOW_ACCEL_KMH_S       = 0.3f;
static constexpr float FREEZE_ACCEL_KMH_S     = 0.2f;

// ---- APS decay (per loop, ~10 ms) ----
static constexpr float FAST_DECAY_V            = 0.020f;
static constexpr float SLOW_DECAY_V            = 0.005f;

// ---- APS safety ----
static constexpr float APS_MIN_V               = 0.75f;
static constexpr float APS_RELEASE_DELTA_V     = 0.05f;
static constexpr float APS_IDLE_MARGIN_V       = 0.05f;

// ---- Timing ----
static constexpr uint32_t TELEMETRY_MS          = 100;
static constexpr uint32_t LOGIC_LOOP_MS         = 10;

// ---- Accel window ----
static constexpr uint8_t ACCEL_WINDOW_SAMPLES  = 5;

// ============================== STATE ==============================

enum class LimiterState
{
    PASS_THROUGH,
    OVERSHOOT_CONTROL,
    LIMIT_ACTIVE,
    FAULT
};

static LimiterState state = LimiterState::PASS_THROUGH;
static bool relay_active = false;

// APS command (latched)
static float aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
static float aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;

// Timing
static uint32_t last_loop_ms = 0;
static uint32_t last_telemetry_ms = 0;

// Accel window
static float speed_win[ACCEL_WINDOW_SAMPLES] = {0};
static uint32_t time_win[ACCEL_WINDOW_SAMPLES] = {0};
static uint8_t win_idx = 0;
static bool win_full = false;

// ============================== HELPERS ==============================

static void setRelay(bool on)
{
    relay_active = on;
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

static bool pedalReleased(float in1, float in2)
{
    return (in1 <= DEFAULT_SIGNAL_S1_V + APS_IDLE_MARGIN_V) &&
           (in2 <= DEFAULT_SIGNAL_S2_V + APS_IDLE_MARGIN_V);
}

static bool driverRequestsLess(float in1, float in2)
{
    return (in1 < aps_cmd_v1 - APS_RELEASE_DELTA_V) ||
           (in2 < aps_cmd_v2 - APS_RELEASE_DELTA_V);
}

static void clampAPS()
{
    if (aps_cmd_v1 < APS_MIN_V) aps_cmd_v1 = APS_MIN_V;
    if (aps_cmd_v2 < APS_MIN_V) aps_cmd_v2 = APS_MIN_V;
}

static float computeAccel(float speed, uint32_t ts)
{
    if (ts == 0) return 0.0f;

    speed_win[win_idx] = speed;
    time_win[win_idx] = ts;
    win_idx = (win_idx + 1) % ACCEL_WINDOW_SAMPLES;

    if (win_idx == 0) win_full = true;
    if (!win_full) return 0.0f;

    uint8_t oldest = win_idx;
    float ds = speed - speed_win[oldest];
    uint32_t dt = ts - time_win[oldest];
    if (dt == 0) dt = 1;

    return ds / (dt / 1000.0f);
}

// ============================== INIT ==============================

void LogicModule_Begin()
{
    pinMode(RELAY_PIN, OUTPUT);
    setRelay(false);

    aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
    aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;

    last_loop_ms = millis();
    last_telemetry_ms = last_loop_ms;
}

// ============================== MAIN UPDATE ==============================

void LogicModule_Update()
{
    uint32_t now = millis();
    float dt = (now - last_loop_ms) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    last_loop_ms = now;

    float aps_in_v1, aps_in_v2;
    uint32_t aps_ts;
    SharedState_GetAps(&aps_in_v1, &aps_in_v2, &aps_ts);

    float speed_kmh = g_speed_kmh;
    uint32_t speed_ts = g_speed_last_update_ms;

    float accel = computeAccel(speed_kmh, speed_ts);

    switch (state)
    {
        // ==================================================
        case LimiterState::PASS_THROUGH:
        {
            setRelay(false);
            SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);

            aps_cmd_v1 = aps_in_v1;
            aps_cmd_v2 = aps_in_v2;

            if (speed_kmh >= (SPEED_LIMIT_KMH - SPEED_MARGIN_KMH))
            {
                aps_cmd_v1 = aps_in_v1;
                aps_cmd_v2 = aps_in_v2;
                setRelay(true);
                state = LimiterState::OVERSHOOT_CONTROL;
            }
            break;
        }

        // ==================================================
        case LimiterState::OVERSHOOT_CONTROL:
        {
            setRelay(true);

            if (pedalReleased(aps_in_v1, aps_in_v2))
            {
                setRelay(false);
                state = LimiterState::PASS_THROUGH;
                break;
            }

            if (driverRequestsLess(aps_in_v1, aps_in_v2))
            {
                aps_cmd_v1 = aps_in_v1;
                aps_cmd_v2 = aps_in_v2;
            }

            bool near_limit =
                speed_kmh >= (SPEED_LIMIT_KMH - 1.0f) &&
                speed_kmh <= (SPEED_LIMIT_KMH + 0.5f);

            if (near_limit && fabs(accel) < FREEZE_ACCEL_KMH_S)
            {
                state = LimiterState::LIMIT_ACTIVE;
                break;
            }

            // decay
            if (accel > FAST_ACCEL_KMH_S)
            {
                aps_cmd_v1 -= FAST_DECAY_V;
                aps_cmd_v2 -= FAST_DECAY_V;
            }
            else if (accel > SLOW_ACCEL_KMH_S)
            {
                aps_cmd_v1 -= SLOW_DECAY_V;
                aps_cmd_v2 -= SLOW_DECAY_V;
            }

            clampAPS();
            SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
            break;
        }

        // ==================================================
        case LimiterState::LIMIT_ACTIVE:
        {
            setRelay(true);

            const float LOW_BAND  = SPEED_LIMIT_KMH - SPEED_DEADBAND_KMH;
            const float HIGH_BAND = SPEED_LIMIT_KMH + SPEED_DEADBAND_KMH;

            // ---- FREEZE ----
            if (speed_kmh >= LOW_BAND &&
                speed_kmh <= HIGH_BAND &&
                fabs(accel) < FREEZE_ACCEL_KMH_S)
            {
                SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
                break;
            }

            // ---- OVERSPEED ----
            if (speed_kmh > HIGH_BAND)
            {
                if (accel > FAST_ACCEL_KMH_S)
                {
                    aps_cmd_v1 -= FAST_DECAY_V;
                    aps_cmd_v2 -= FAST_DECAY_V;
                }
                else
                {
                    aps_cmd_v1 -= SLOW_DECAY_V;
                    aps_cmd_v2 -= SLOW_DECAY_V;
                }
            }

            clampAPS();
            SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);

            // ---- DRIVER RELEASE ----
            if (driverRequestsLess(aps_in_v1, aps_in_v2))
            {
                setRelay(false);
                state = LimiterState::PASS_THROUGH;
            }
            break;
        }

        // ==================================================
        case LimiterState::FAULT:
        default:
        {
            setRelay(false);
            SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
            break;
        }
    }

    // ================= TELEMETRY =================
    if (now - last_telemetry_ms >= TELEMETRY_MS)
    {
        last_telemetry_ms = now;
        Serial.printf(
            "SPD=%.1f APSin=(%.2f,%.2f) APSout=(%.2f,%.2f) Relay=%d State=%d Accel=%.2f\n",
            speed_kmh,
            aps_in_v1, aps_in_v2,
            aps_cmd_v1, aps_cmd_v2,
            relay_active,
            (int)state,
            accel);
    }
}

// ============================== TASK ==============================

static void logicTask(void *)
{
    for (;;)
    {
        LogicModule_Update();
        vTaskDelay(pdMS_TO_TICKS(LOGIC_LOOP_MS));
    }
}

void LogicModule_StartTask()
{
    xTaskCreatePinnedToCore(
        logicTask,
        "LOGIC",
        4096,
        nullptr,
        4,
        nullptr,
        0);
}
