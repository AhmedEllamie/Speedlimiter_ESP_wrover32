#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// Speed limit persistence (NVS Preferences) + shared-state integration.
// - Value is stored under (PREF_NS / PREF_KEY_SL).
// - Value is also mirrored into `g_speed_limit_kmh` in shared_state.
// - Range: 0..250 km/h (0 disables limiter).
// -----------------------------------------------------------------------------

void SpeedLimitStore_Begin();

uint16_t SpeedLimitStore_GetKmh();

void SpeedLimitStore_SetKmh(uint16_t kmh);

void SpeedLimitStore_ResetToDefault();

