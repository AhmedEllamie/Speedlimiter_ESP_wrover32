#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// Global build flags (ensure defaults exist in every translation unit)
// -----------------------------------------------------------------------------
#ifndef BUILD_TEST_LOGGER
#define BUILD_TEST_LOGGER 0
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
static const uint32_t OBD_REQ_INTERVAL_MS = 40; // 25 Hz - higher rate for accurate limit detection

// Speed validity timeout (failsafe: relay OFF)
static const uint32_t SPEED_TIMEOUT_MS = 500;

// -----------------------------------------------------------------------------
// Preferences (NVS)
// -----------------------------------------------------------------------------
static const char *PREF_NS = "speedLimiter";
static const char *PREF_KEY_SL = "sl"; // speed limit in km/h
static const uint16_t SPEED_LIMIT_DEFAULT_KMH = 40;

// -----------------------------------------------------------------------------
// Dashboard/CAN gap compensation
// -----------------------------------------------------------------------------
// Some vehicles show a higher speed on the dashboard than the CAN/OBD speed.
// To compensate, we activate limiting earlier than the user-set limit:
//   effective_limit = speed_limit_kmh - SPEED_LIMIT_ACTIVATION_OFFSET_KMH
static const uint16_t SPEED_LIMIT_ACTIVATION_OFFSET_KMH = 5;

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
static const float SPEED_DEADBAND_KMH = 0.3f;
static const float FACTOR_RATE_PER_KMH_PER_S = 0.06f; // tune to your vehicle

// -----------------------------------------------------------------------------
// Task rates
// -----------------------------------------------------------------------------
static const uint32_t ADC_SAMPLE_INTERVAL_MS = 1;
static const uint32_t PWM_UPDATE_INTERVAL_MS = 1;
static const uint32_t LOGIC_LOOP_INTERVAL_MS = 1;

