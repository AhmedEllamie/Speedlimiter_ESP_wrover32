// =============================================================================
// Build selector
// =============================================================================
// BUILD_TEST_LOGGER = 0  -> Speed Limiter firmware (default)
// BUILD_TEST_LOGGER = 1  -> Logger firmware (see src/test_logger_fw.h)
#ifndef BUILD_TEST_LOGGER
#define BUILD_TEST_LOGGER 0
#endif

#if BUILD_TEST_LOGGER
#include "test_logger_fw.h"
#else

// =============================================================================
// Speed Limiter (modular tasks)
//
// Modules:
// - CAN: reads speed (OBD or broadcast) -> publishes g_speed_kmh
// - ADC: reads APS1/APS2 -> publishes ECU-level volts
// - PWM: outputs desired ECU-level volts (reads shared globals)
// - Logic: compares speed vs limit, controls relay, publishes desired outputs
// =============================================================================

#include <Arduino.h>

#include "adc_module.h"
#include "can_module.h"
#include "logic_module.h"
#include "pwm_module.h"

void setup() {
  Serial.begin(115200);
  delay(150);

  // Init modules (hardware + defaults)
  AdcModule_Begin();
  PwmModule_Begin();
  CanModule_Begin();
  LogicModule_Begin();

  // Start module tasks
  CanModule_StartTask();
  AdcModule_StartTask();
  PwmModule_StartTask();
  LogicModule_StartTask();
}

void loop() {
  // Everything runs in module tasks.
  delay(1000);
}

#endif // BUILD_TEST_LOGGER

