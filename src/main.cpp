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

#include "adc_module.h"
#include "can_module.h"
#include "pwm_module.h"
#include "logic_module.h"

void setup() {
  Serial.begin(115200);
  delay(150);

  Serial.println("Speed Limiter - Modular Architecture");
  Serial.println("Modules: CAN, ADC, PWM, Logic (algorithm)");

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

  Serial.println("All modules started");
}

void loop() {
  // Everything runs in module tasks.
  // Main loop can be used for low-priority tasks or monitoring if needed.
  delay(1000);
}

