/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// Global build flags (ensure defaults exist in every translation unit)
// -----------------------------------------------------------------------------
#ifndef BUILD_TEST_LOGGER
#define BUILD_TEST_LOGGER 0
#endif

// Select which LogicModule implementation to compile:
// - USE_ALGORITHM_MODULE=1 -> `src/algorithm.cpp` (default)
// - USE_ALGORITHM_MODULE=0 -> `src/new_algorithm.cpp`
#ifndef USE_ALGORITHM_MODULE
#define USE_ALGORITHM_MODULE 0
#endif

#ifndef USE_DAC_OUTPUT
#define USE_DAC_OUTPUT 0
#endif

// -----------------------------------------------------------------------------
// Speed source selection (no runtime config)
// -----------------------------------------------------------------------------
// 1 = OBD-II speed response (PID 0x0D)   (default)
// 0 = broadcast frame (SPEED_FRAME_ID / SPEED_BYTE_INDEX)
#ifndef SPEED_SOURCE_OBD
#define SPEED_SOURCE_OBD 1
#endif

// Broadcast speed config if SPEED_SOURCE_OBD=0
#ifndef SPEED_FRAME_ID
#define SPEED_FRAME_ID 0x123
#endif
#ifndef SPEED_BYTE_INDEX
#define SPEED_BYTE_INDEX 0
#endif

// -----------------------------------------------------------------------------
// Pin assignments
// -----------------------------------------------------------------------------
static const int RELAY_PIN = 23;   // IO23 - relay control (HIGH = relay active)

static const int CAN_RX_PIN = 32;  // IO32
static const int CAN_TX_PIN = 33;  // IO33

static const int APS1_PIN = 39;    // IO39 - APS1 (ADC1_CH3)
static const int APS2_PIN = 36;    // IO36 - APS2 (ADC1_CH0)

#if USE_DAC_OUTPUT
static const int SDPS1_PIN = 25;   // IO25 - DAC1
static const int SDPS2_PIN = 26;   // IO26 - DAC2
#else
static const int SDPS1_PIN = 21;   // IO21 - PWM output
static const int SDPS2_PIN = 22;   // IO22 - PWM output
#endif

// -----------------------------------------------------------------------------
// CAN constants
// -----------------------------------------------------------------------------
static const uint32_t CAN_BAUD = 500000;

// OBD-II speed PID request defaults (PID 0x0D)
static const uint16_t OBD_REQ_ID = 0x7DF;
static const uint8_t OBD_PID_SPEED = 0x0D;
static const uint8_t OBD_PID_RPM = 0x0C;
static const uint32_t OBD_REQ_INTERVAL_MS = 40; // 25 Hz - higher rate for accurate limit detection

// Speed validity timeout (failsafe: relay OFF)
static const uint32_t SPEED_TIMEOUT_MS = 500;

// -----------------------------------------------------------------------------
// Preferences (NVS)
// -----------------------------------------------------------------------------
static const char *PREF_NS = "speedLimiter";
static const char *PREF_KEY_SL = "sl"; // speed limit in km/h
static const uint16_t SPEED_LIMIT_DEFAULT_KMH = 50;

// -----------------------------------------------------------------------------
// Dashboard/CAN gap compensation
// -----------------------------------------------------------------------------
// Some vehicles show a higher speed on the dashboard than the CAN/OBD speed.
// To compensate, we activate limiting earlier than the user-set limit:
//   effective_limit = speed_limit_kmh - SPEED_LIMIT_ACTIVATION_OFFSET_KMH
static const uint16_t SPEED_LIMIT_ACTIVATION_OFFSET_KMH = 10;

// Turn relay OFF once speed falls this far below the effective limit.
// Example: SL=40, activation offset=5 => effective_limit=35.
// With hysteresis=5, relay turns OFF at <=30 km/h (CAN speed).
static const uint16_t SPEED_LIMIT_DEACTIVATION_HYSTERESIS_KMH = 10;

// -----------------------------------------------------------------------------
// Pedal scaling + safety floors
// -----------------------------------------------------------------------------
// SRD correction formula: Actual = ADC * (69.6 / 47.5)
static const float PEDAL_SCALE = (69.6f / 47.5f);

// Output divider compensation: to generate 1V ECU-output, generate 2V internally
static const float OUTPUT_DIVIDER_GAIN = 2.0f;

// Minimum safe ECU-level values (floor)
static const float DEFAULT_SIGNAL_S1_V = 0.350f;
static const float DEFAULT_SIGNAL_S2_V = 0.700f;

// -----------------------------------------------------------------------------
// Limiter behavior
// -----------------------------------------------------------------------------
static const uint16_t PEDAL_RELEASE_MARGIN_MV = 100; // ~100 mV below captured -> release relay
static const uint32_t CONTROL_INTERVAL_MS = 50;
//static const float SPEED_DEADBAND_KMH = 0.3f;

// Relay anti-chatter: minimum time between ANY relay state changes.
// This protects the relay from fast ON/OFF toggling due to noisy thresholds.
static const uint32_t RELAY_MIN_CHANGE_INTERVAL_MS = 2000;

// -------------------------- Limiter controller tuning --------------------------
// This is a SPEED LIMITER (not cruise control):
// - Reduce throttle quickly when overspeed is predicted.
// - Relax throttle back slowly only when speed is safely under the limit AND not rising.
//
// Units:
// - factor is unitless (0..1)
// - rates are "factor per second" or "factor per (km/h) per second"
static const float LIMIT_SPEED_LOOKAHEAD_S = 0.35f;     // predict ahead using speed derivative
static const float LIMIT_RELAX_BAND_KMH = 0.8f;         // must be under limit by this to relax
static const float LIMIT_RELAX_MAX_RISE_KMH_PER_S = 0.2f; // do NOT relax if speed is rising faster than this

static const float FACTOR_RATE_DOWN_PER_KMH_PER_S = 0.35f; // FAST reduction when overspeed
static const float FACTOR_RATE_UP_PER_S = 0.08f;           // SLOW relax back toward 1.0

// Speed derivative smoothing for the limiter (0..1). Higher = more smoothing.
static const float LIMIT_SPEED_RATE_LPF_ALPHA = 0.85f;

// -----------------------------------------------------------------------------
// APS speed map (ECU-level millivolts)
// -----------------------------------------------------------------------------
// Used by the table-based limiter to cap APS_out based on speed limit.
// Provide points in ascending speed order. The limiter will linearly interpolate
// between points.
struct ApsSpeedMapPoint {
  uint16_t speed_kmh;
  uint16_t aps1_mv;
  uint16_t aps2_mv;
};

static constexpr ApsSpeedMapPoint APS_SPEED_MAP[] = {
    // speed_kmh, aps1_mv, aps2_mv
    {10, 500, 1100},
    {20, 600, 1250},
    {30, 700, 1500},
    {40, 800, 1700},
    {50, 900, 1800},
};

static constexpr uint8_t APS_SPEED_MAP_LEN =
    (uint8_t)(sizeof(APS_SPEED_MAP) / sizeof(APS_SPEED_MAP[0]));

// -----------------------------------------------------------------------------
// Task rates
// -----------------------------------------------------------------------------
static const uint32_t ADC_SAMPLE_INTERVAL_MS = 1;
static const uint32_t PWM_UPDATE_INTERVAL_MS = 1;
static const uint32_t LOGIC_LOOP_INTERVAL_MS = 1;

