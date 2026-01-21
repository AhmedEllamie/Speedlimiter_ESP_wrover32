// =============================================================================
// OLD CODE - COMMENTED OUT (using // comments to avoid preprocessor issues)
// =============================================================================
// Build selector
// BUILD_TEST_LOGGER = 0  -> Speed Limiter firmware (default)
// BUILD_TEST_LOGGER = 1  -> Logger firmware (see src/test_logger_fw.h)
// #ifndef BUILD_TEST_LOGGER
// #define BUILD_TEST_LOGGER 0
// #endif
//
// #if BUILD_TEST_LOGGER
// #include "test_logger_fw.h"
// #else
//
// Build selector for PWM test
// #ifndef BUILD_PWM_TEST
// #define BUILD_PWM_TEST 0
// #endif
//
// #if !BUILD_PWM_TEST
//
// Speed Limiter (modular tasks)
// Modules:
// - CAN: reads speed (OBD or broadcast) -> publishes g_speed_kmh
// - ADC: reads APS1/APS2 -> publishes ECU-level volts
// - PWM: outputs desired ECU-level volts (reads shared globals)
// - Logic: compares speed vs limit, controls relay, publishes desired outputs
//
// #include <Arduino.h>
// #include "adc_module.h"
// #include "can_module.h"
// #include "pwm_module.h"
// #include "speed_controller_module.h"
//
// void setup() {
//   Serial.begin(115200);
//   delay(150);
//   AdcModule_Begin();
//   PwmModule_Begin();
//   CanModule_Begin();
//   SpeedControllerModule_Begin();
//   CanModule_StartTask();
//   AdcModule_StartTask();
//   PwmModule_StartTask();
//   SpeedControllerModule_StartTask();
// }
//
// void loop() {
//   delay(1000);
// }
//
// #else // BUILD_PWM_TEST
// ... PWM test code ...
// #endif // !BUILD_PWM_TEST
// #endif // BUILD_TEST_LOGGER
// =============================================================================

// =============================================================================
// NEW Speed Limiter (using only: algorithm, adc, can, shared_state, pwm)
// =============================================================================
// Modules:
// - CAN: reads speed (OBD or broadcast) -> publishes g_speed_kmh
// - ADC: reads APS1/APS2 -> publishes ECU-level volts
// - PWM: outputs desired ECU-level volts (reads shared globals)
// - Logic (algorithm): compares speed vs limit, controls relay, publishes desired outputs
// =============================================================================

#include <Arduino.h>

#include "sl_config.h"


#if BUILD_TEST_LOGGER
// Logger firmware (defines its own setup()/loop()).
#include "test_logger_fw.h"
#else

#ifdef MANUAL
#include "can_module.h"
#include "shared_state.h"
#include "speed_limit_store.h"
#include "web_server_module.h"

static const uint8_t SPEED_LIMIT_HYST_KMH = 5;

static bool g_relay_active = false;

static void setRelayActive(bool active) {
  g_relay_active = active;
  digitalWrite(RELAY_PIN, active ? HIGH : LOW);
  SharedState_SetLimiterActive(active);
}

void setup() {
  Serial.begin(115200);
  delay(150);

  Serial.println("Speed Limiter - MANUAL MODE");
  Serial.println("CAN speed -> relay on/off only");

  pinMode(RELAY_PIN, OUTPUT);
  setRelayActive(false);

  SpeedLimitStore_Begin();
  WebServerModule_Begin();

  CanModule_Begin();
  CanModule_StartTask();
}

void loop() {
  WebServerModule_HandleClient();

  uint32_t now = millis();
  bool speed_valid = SharedState_SpeedValid(now, SPEED_TIMEOUT_MS);
  uint16_t sl_kmh = SharedState_GetSpeedLimitKmh();

  if (!speed_valid) {
    if (g_relay_active) {
      setRelayActive(false);
    }
  } else if (sl_kmh == 0) {
    // Disabled -> fail-safe relay OFF.
    if (g_relay_active) setRelayActive(false);
  } else {
    uint8_t speed_kmh = g_speed_kmh;
    int32_t off_kmh = (sl_kmh > SPEED_LIMIT_HYST_KMH) ? ((int32_t)sl_kmh - SPEED_LIMIT_HYST_KMH) : 0;
    if (!g_relay_active && (uint16_t)speed_kmh >= sl_kmh) {
      setRelayActive(true);
    } else if (g_relay_active && (int32_t)speed_kmh <= off_kmh) {
      setRelayActive(false);
    }
  }

  delay(20);
}

#else

#include "adc_module.h"
#include "can_module.h"
#include "pwm_module.h"
#include "logic_module.h"
#include "speed_limit_store.h"
#include "web_server_module.h"

void setup() {
  Serial.begin(115200);
  delay(150);

  Serial.println("Speed Limiter - Modular Architecture");
  Serial.println("Modules: CAN, ADC, PWM, Logic (algorithm)");

  SpeedLimitStore_Begin();

  // Initialize modules (hardware + defaults)
  AdcModule_Begin();
  PwmModule_Begin();
  CanModule_Begin();
  LogicModule_Begin();

  // Start module tasks
  CanModule_StartTask();
  AdcModule_StartTask();
  PwmModule_StartTask();
  LogicModule_StartTask();

  WebServerModule_Begin();

  Serial.println("All modules started");
}

void loop() {
  // Everything runs in module tasks.
  // Main loop can be used for low-priority tasks or monitoring if needed.
  WebServerModule_HandleClient();
  delay(5);
}

#endif

#endif // BUILD_TEST_LOGGER