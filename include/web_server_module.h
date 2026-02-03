/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#pragma once

#include "sl_config.h"

// -----------------------------------------------------------------------------
// Simple ESP32 AP + HTTP server:
// - SSID: SpeedLimiterXXXX (XXXX = last 4 hex digits of ESP MAC)
// - Password: P@ssw0rd
// - Pages:
//    * GET /        -> System Status
//    * GET /config  -> Config (set/reset speed limit)
// -----------------------------------------------------------------------------

#if !BUILD_TEST_LOGGER

void WebServerModule_Begin();
void WebServerModule_HandleClient();

#else

// Logger firmware doesn't need the web UI.
inline void WebServerModule_Begin() {}
inline void WebServerModule_HandleClient() {}

#endif

