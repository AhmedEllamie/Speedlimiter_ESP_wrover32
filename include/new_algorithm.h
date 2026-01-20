#pragma once

#include <stddef.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// APPS (APS1/APS2) lookup table for the new algorithm.
//
// The algorithm selects an APPS pair based on the *speed limit* (km/h):
// - Row 0: SL < 5
// - Row 1: SL < 10
// - Row 2: SL < 20
// - Row 3: SL < 30
// - Row 4: SL in [30..100] (and above, clamped)
//
// Values are ECU-level volts (same domain as g_aps*_v / g_sdps*_v).
// TODO: Tune these values per vehicle.
// -----------------------------------------------------------------------------

static constexpr uint16_t NISSAN_SENTRA_APPS_SPEED_BOUNDS_KMH[] = {5, 10, 20, 30, 100};

// 2-D array: [row][0]=APS1_V, [row][1]=APS2_V
static constexpr float NISSAN_SENTRA_APPS_TABLE_V[][2] = {
    {0.350f, 0.700f}, // SL < 5
    {0.500f, 1.100f}, // SL < 10
    {0.600f, 1.250f}, // SL < 20
    {0.700f, 1.500f}, // SL < 30
    {0.900f, 1.800f}, // SL 30..100 (and above)
};

static constexpr size_t NISSAN_SENTRA_APPS_TABLE_LEN =
    sizeof(NISSAN_SENTRA_APPS_SPEED_BOUNDS_KMH) / sizeof(NISSAN_SENTRA_APPS_SPEED_BOUNDS_KMH[0]);

static_assert(NISSAN_SENTRA_APPS_TABLE_LEN == (sizeof(NISSAN_SENTRA_APPS_TABLE_V) / sizeof(NISSAN_SENTRA_APPS_TABLE_V[0])),
              "APPS bounds/table length mismatch");