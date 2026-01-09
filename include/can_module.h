#pragma once

#include <stdint.h>

// CAN module
// - Own task periodically issues OBD speed requests (if enabled)
// - RX callback parses speed frames and publishes `g_speed_kmh`
void CanModule_Begin();
void CanModule_StartTask();

