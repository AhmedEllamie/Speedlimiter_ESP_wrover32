#include "shared_state.h"
#include "sl_config.h"
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ================= CONFIG =================

static constexpr float SPEED_MARGIN_KMH      = 3.0f;

// Acceleration thresholds (VALID after windowing)
static constexpr float FAST_ACCEL_KMH_S      = 2.0f;
static constexpr float SLOW_ACCEL_KMH_S      = 0.3f;

static constexpr float FAST_DECAY_V           = 0.020f;
static constexpr float SLOW_DECAY_V           = 0.005f;

static constexpr float APS_MIN_V              = 0.50f;
static constexpr float APS_RELEASE_DELTA_V    = 0.05f;

static constexpr float SPEED_LIMIT_KMH        = 55.0f;

// Windowed acceleration config
static constexpr uint8_t ACCEL_WINDOW_SAMPLES = 10;

static TaskHandle_t g_logic_task = nullptr;

// ==========================================

enum class LimiterState
{
    PASS_THROUGH,
    OVERSHOOT_ARMED,
    LIMIT_ACTIVE,
    FAULT
};

static LimiterState state = LimiterState::PASS_THROUGH;

// Latched APS values
static float aps_cmd_v1 = 0.0f;
static float aps_cmd_v2 = 0.0f;

// Speed window buffer
static float speed_window[ACCEL_WINDOW_SAMPLES] = {0.0f};
static uint8_t speed_idx = 0;
static bool speed_window_full = false;

// Time tracking
static uint32_t prev_time_ms = 0;

// Relay output
static bool relay_active = false;

// ==========================================================

static void setRelayActive(bool active)
{
    digitalWrite(RELAY_PIN, active ? HIGH : LOW);
}

static void Logic_EnterFault()
{
    state = LimiterState::FAULT;
    relay_active = false;
    setRelayActive(false);
}

// ==========================================================

void LogicModule_Begin()
{
    state = LimiterState::PASS_THROUGH;
    prev_time_ms = millis();

    pinMode(RELAY_PIN, OUTPUT);
    setRelayActive(false);

    // Clear speed window
    for (auto &v : speed_window) v = 0.0f;
    speed_idx = 0;
    speed_window_full = false;
}

// ==========================================================

static float ComputeWindowedAccel(float speed_kmh, float dt)
{
    speed_window[speed_idx] = speed_kmh;
    speed_idx = (speed_idx + 1) % ACCEL_WINDOW_SAMPLES;

    if (speed_idx == 0)
        speed_window_full = true;

    if (!speed_window_full)
        return 0.0f;

    float speed_old = speed_window[speed_idx];
    float window_time = ACCEL_WINDOW_SAMPLES * dt;

    if (window_time <= 0.0f)
        window_time = 0.001f;

    return (speed_kmh - speed_old) / window_time;
}

// ==========================================================

void LogicModule_Update(float speed_limit_kmh)
{
    uint32_t now_ms = millis();
    float dt = (now_ms - prev_time_ms) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    prev_time_ms = now_ms;

    // ---------------- Read inputs ----------------

    float aps_in_v1, aps_in_v2;
    uint32_t aps_ts;
    SharedState_GetAps(&aps_in_v1, &aps_in_v2, &aps_ts);

    if (!SharedState_SpeedValid(now_ms, 200))
    {
        Logic_EnterFault();
    }

    float speed_kmh = (float)g_speed_kmh;

    float accel_kmh_s = ComputeWindowedAccel(speed_kmh, dt);

    // ---------------- State machine ----------------

    switch (state)
    {
        case LimiterState::PASS_THROUGH:
        {
            relay_active = false;
            setRelayActive(false);
            SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);

            if (speed_kmh >= (speed_limit_kmh - SPEED_MARGIN_KMH))
            {
                state = LimiterState::OVERSHOOT_ARMED;
            }
            break;
        }

        case LimiterState::OVERSHOOT_ARMED:
        {
            aps_cmd_v1 = aps_in_v1;
            aps_cmd_v2 = aps_in_v2;

            SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);

            if (speed_kmh >= speed_limit_kmh)
            {
                relay_active = true;
                setRelayActive(true);
                state = LimiterState::LIMIT_ACTIVE;
            }

            if (aps_in_v1 < aps_cmd_v1 - APS_RELEASE_DELTA_V)
            {
                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
            }
            break;
        }

        case LimiterState::LIMIT_ACTIVE:
        {
            relay_active = true;
            setRelayActive(true);

            if (accel_kmh_s > FAST_ACCEL_KMH_S)
            {
                aps_cmd_v1 -= FAST_DECAY_V;
                aps_cmd_v2 -= FAST_DECAY_V;
            }
            else if (accel_kmh_s > SLOW_ACCEL_KMH_S)
            {
                aps_cmd_v1 -= SLOW_DECAY_V;
                aps_cmd_v2 -= SLOW_DECAY_V;
            }

            if (aps_cmd_v1 < APS_MIN_V) aps_cmd_v1 = APS_MIN_V;
            if (aps_cmd_v2 < APS_MIN_V) aps_cmd_v2 = APS_MIN_V;

            SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);

            if (aps_in_v1 < aps_cmd_v1 - APS_RELEASE_DELTA_V)
            {
                relay_active = false;
                setRelayActive(false);
                state = LimiterState::PASS_THROUGH;
            }
            break;
        }

        case LimiterState::FAULT:
        default:
        {
            relay_active = false;
            setRelayActive(false);
            SharedState_SetDesiredOutputs(aps_in_v1, aps_in_v2);
            break;
        }
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
