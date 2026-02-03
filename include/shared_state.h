/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// Shared globals (simple cross-module publish/subscribe)
// -----------------------------------------------------------------------------
// CAN speed (km/h) + timestamp (ms)
extern volatile uint8_t g_speed_kmh;
extern volatile uint32_t g_speed_last_update_ms;

// Configured speed limit (km/h). 0 = disabled (fail-safe / relay OFF).
extern volatile uint16_t g_speed_limit_kmh;

// Speed limiter active flag (1 = relay/limiting active, 0 = inactive).
extern volatile uint8_t g_speed_limiter_active;

// CAN RPM (rpm) + timestamp (ms)
extern volatile uint16_t g_rpm;
extern volatile uint32_t g_rpm_last_update_ms;

// Pedal inputs (APS1/APS2) at ECU-level volts + timestamp (ms)
extern volatile float g_aps1_v;
extern volatile float g_aps2_v;
extern volatile uint32_t g_aps_last_update_ms;

// Desired ECU-level outputs (what PWM/DAC should generate)
extern volatile float g_sdps1_v;
extern volatile float g_sdps2_v;

// -----------------------------------------------------------------------------
// Access helpers (use when you need consistent pairs)
// -----------------------------------------------------------------------------
void SharedState_SetSpeed(uint8_t kmh, uint32_t now_ms);
bool SharedState_SpeedValid(uint32_t now_ms, uint32_t timeout_ms);

void SharedState_SetSpeedLimitKmh(uint16_t kmh);
uint16_t SharedState_GetSpeedLimitKmh();

void SharedState_SetLimiterActive(bool active);
bool SharedState_IsLimiterActive();

void SharedState_SetRpm(uint16_t rpm, uint32_t now_ms);
bool SharedState_RpmValid(uint32_t now_ms, uint32_t timeout_ms);
uint16_t SharedState_GetRpm();

void SharedState_SetAps(float v1, float v2, uint32_t now_ms);
void SharedState_GetAps(float *v1, float *v2, uint32_t *last_ms);

void SharedState_SetDesiredOutputs(float v1, float v2);
void SharedState_GetDesiredOutputs(float *v1, float *v2);

