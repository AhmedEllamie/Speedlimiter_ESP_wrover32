#pragma once

#include <stddef.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// APPS (APS1/APS2) lookup table for the new algorithm.
//
// The algorithm selects an APPS pair based on the *speed limit* (km/h) using this
// speed->APPS map and LINEAR INTERPOLATION between points.
//
// Values are ECU-level millivolts (after pedal scaling) taken from the car.
// -----------------------------------------------------------------------------

struct NissanSentraAppsMapPoint {
  uint16_t speed_kmh;
  uint16_t aps1_mv;
  uint16_t aps2_mv;
};

static constexpr NissanSentraAppsMapPoint NISSAN_SENTRA_APPS_SPEED_MAP[] = {
    // speed_kmh, aps1_mv, aps2_mv
    {10, 350, 700},
    {20, 450, 850},
    {30, 480, 900},
    {40, 500, 970},
    {50, 510, 1020},
    {60, 580, 1120},
    {70, 610, 1160},
};

static constexpr size_t NISSAN_SENTRA_APPS_SPEED_MAP_LEN =
    sizeof(NISSAN_SENTRA_APPS_SPEED_MAP) / sizeof(NISSAN_SENTRA_APPS_SPEED_MAP[0]);