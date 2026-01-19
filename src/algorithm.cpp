#include "logic_module.h"
#include "shared_state.h"
#include "sl_config.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ====================== TUNED CONFIG ======================

// Speed limit
static constexpr float SPEED_LIMIT_KMH = (float)SPEED_LIMIT_DEFAULT_KMH;

// Pre-limit capture margin
static constexpr float SPEED_MARGIN_KMH = 2.0f;

// Exit hysteresis
static constexpr float SPEED_HYSTERESIS_KMH = 2.0f;

// Acceleration window
static constexpr uint8_t ACCEL_WINDOW_SAMPLES = 5;

// Accel thresholds (CAN-aware)
static constexpr float FAST_ACCEL_KMH_S = 2.0f;
static constexpr float SLOW_ACCEL_KMH_S = 0.6f;

// APS decay rates (V/s)
static constexpr float FAST_DECAY_V_PER_S = 2.0f;
static constexpr float SLOW_DECAY_V_PER_S = 0.5f;

// APS handling
static constexpr float APS_RELEASE_DELTA_V = 0.05f;
static constexpr float APS_IDLE_MARGIN_V   = 0.05f;

// State timing
static constexpr uint32_t MIN_STATE_TIME_MS = 50;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;

// =========================================================

enum class LimiterState
{
    PASS_THROUGH,
    OVERSHOOT_CONTROL,
    LIMIT_ACTIVE,
    FAULT
};

static LimiterState state = LimiterState::PASS_THROUGH;
static TaskHandle_t logic_task = nullptr;

// APS command
static float aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
static float aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;

// Relay
static bool relay_active = false;

// Time
static uint32_t prev_ms = 0;
static uint32_t state_entry_ms = 0;
static uint32_t last_log_ms = 0;

// Speed window
static float speed_buf[ACCEL_WINDOW_SAMPLES] = {};
static uint32_t time_buf[ACCEL_WINDOW_SAMPLES] = {};
static uint8_t idx = 0;
static bool full = false;
static uint32_t last_speed_ms = 0;
static float accel_kmh_s = 0.0f;

// ====================== HELPERS ======================

static void setRelay(bool on)
{
    relay_active = on;
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

static bool pedalReleased(float v1, float v2)
{
    return (v1 <= DEFAULT_SIGNAL_S1_V + APS_IDLE_MARGIN_V) &&
           (v2 <= DEFAULT_SIGNAL_S2_V + APS_IDLE_MARGIN_V);
}

static bool driverAsksLess(float in1, float in2)
{
    return (in1 < aps_cmd_v1 - APS_RELEASE_DELTA_V) ||
           (in2 < aps_cmd_v2 - APS_RELEASE_DELTA_V);
}

static void clampAPS()
{
    if (aps_cmd_v1 < DEFAULT_SIGNAL_S1_V) aps_cmd_v1 = DEFAULT_SIGNAL_S1_V;
    if (aps_cmd_v2 < DEFAULT_SIGNAL_S2_V) aps_cmd_v2 = DEFAULT_SIGNAL_S2_V;
}

static float computeAccel(float speed, uint32_t ms)
{
    if (ms == 0 || ms == last_speed_ms)
        return accel_kmh_s;

    last_speed_ms = ms;

    speed_buf[idx] = speed;
    time_buf[idx]  = ms;
    idx = (idx + 1) % ACCEL_WINDOW_SAMPLES;

    if (idx == 0) full = true;
    if (!full) return accel_kmh_s = 0.0f;

    uint8_t oldest = idx;
    float ds = speed - speed_buf[oldest];
    float dt = (ms - time_buf[oldest]) / 1000.0f;
    if (dt <= 0) dt = 0.01f;

    return accel_kmh_s = ds / dt;
}

// ====================== LOGIC ======================

void LogicModule_Begin()
{
    pinMode(RELAY_PIN, OUTPUT);
    setRelay(false);

    state = LimiterState::PASS_THROUGH;
    prev_ms = millis();
    state_entry_ms = prev_ms;
}

void LogicModule_Update()
{
    uint32_t now = millis();
    float dt = (now - prev_ms) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    prev_ms = now;

    float aps_in1, aps_in2;
    uint32_t aps_ts;
    SharedState_GetAps(&aps_in1, &aps_in2, &aps_ts);

    float speed = (float)g_speed_kmh;
    uint32_t speed_ts = g_speed_last_update_ms;

    bool speed_valid = SharedState_SpeedValid(now, SPEED_TIMEOUT_MS);
    accel_kmh_s = computeAccel(speed, speed_ts);

    uint32_t time_in_state = now - state_entry_ms;

    switch (state)
    {
        case LimiterState::PASS_THROUGH:
        {
            setRelay(false);
            aps_cmd_v1 = aps_in1;
            aps_cmd_v2 = aps_in2;
            SharedState_SetDesiredOutputs(aps_in1, aps_in2);

            if (speed_valid &&
                speed >= (SPEED_LIMIT_KMH - SPEED_MARGIN_KMH))
            {
                aps_cmd_v1 = aps_in1;
                aps_cmd_v2 = aps_in2;
                clampAPS();

                SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);
                setRelay(true);
                state = LimiterState::OVERSHOOT_CONTROL;
                state_entry_ms = now;
            }
            break;
        }

        case LimiterState::OVERSHOOT_CONTROL:
        {
            setRelay(true);

            if (!speed_valid)
            {
                state = LimiterState::FAULT;
                break;
            }

            if (pedalReleased(aps_in1, aps_in2))
            {
                setRelay(false);
                state = LimiterState::PASS_THROUGH;
                break;
            }

            if (driverAsksLess(aps_in1, aps_in2))
            {
                aps_cmd_v1 = aps_in1;
                aps_cmd_v2 = aps_in2;
            }
            else
            {
                if (speed > SPEED_LIMIT_KMH)
                {
                    aps_cmd_v1 -= SLOW_DECAY_V_PER_S * dt;
                    aps_cmd_v2 -= SLOW_DECAY_V_PER_S * dt;
                }
                else if (accel_kmh_s > FAST_ACCEL_KMH_S)
                {
                    aps_cmd_v1 -= FAST_DECAY_V_PER_S * dt;
                    aps_cmd_v2 -= FAST_DECAY_V_PER_S * dt;
                }
                else if (accel_kmh_s > SLOW_ACCEL_KMH_S)
                {
                    aps_cmd_v1 -= SLOW_DECAY_V_PER_S * dt;
                    aps_cmd_v2 -= SLOW_DECAY_V_PER_S * dt;
                }
            }

            clampAPS();
            SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);

            if (speed >= (SPEED_LIMIT_KMH - 1.0f) &&
                accel_kmh_s <= 0.0f &&
                fabs(accel_kmh_s) <= SLOW_ACCEL_KMH_S &&
                time_in_state >= MIN_STATE_TIME_MS)
            {
                state = LimiterState::LIMIT_ACTIVE;
                state_entry_ms = now;
            }
            break;
        }

        case LimiterState::LIMIT_ACTIVE:
        {
            setRelay(true);

            SharedState_SetDesiredOutputs(aps_cmd_v1, aps_cmd_v2);

            if (speed > SPEED_LIMIT_KMH + 0.5f &&
                time_in_state >= MIN_STATE_TIME_MS)
            {
                state = LimiterState::OVERSHOOT_CONTROL;
                state_entry_ms = now;
            }
            else if (pedalReleased(aps_in1, aps_in2) ||
                     speed <= (SPEED_LIMIT_KMH - SPEED_HYSTERESIS_KMH))
            {
                setRelay(false);
                state = LimiterState::PASS_THROUGH;
                state_entry_ms = now;
            }
            break;
        }

        case LimiterState::FAULT:
        {
            setRelay(false);
            SharedState_SetDesiredOutputs(aps_in1, aps_in2);
            break;
        }
    }

    if (now - last_log_ms >= TELEMETRY_INTERVAL_MS)
    {
        last_log_ms = now;
        Serial.printf(
            "Speed=%.1f Accel=%.2f APS_in=%.3f APS_out=%.3f Relay=%d State=%d\r\n",
            speed, accel_kmh_s, aps_in1,
            relay_active ? aps_cmd_v1 : aps_in1,
            relay_active, (int)state);
    }
}

static void logicTask(void *)
{
    for (;;)
    {
        LogicModule_Update();
        vTaskDelay(pdMS_TO_TICKS(LOGIC_LOOP_INTERVAL_MS));
    }
}

void LogicModule_StartTask()
{
    xTaskCreatePinnedToCore(
        logicTask, "LOGIC", 2048, nullptr, 4, nullptr, 0);
}

bool LogicModule_IsRelayActive()
{
    return relay_active;
}
