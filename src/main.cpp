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
// Build selector for PWM test
// =============================================================================
#ifndef BUILD_PWM_TEST
#define BUILD_PWM_TEST 0
#endif

#if !BUILD_PWM_TEST

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
#include "pwm_module.h"
#include "speed_controller_module.h"

void setup() {
  Serial.begin(115200);
  delay(150);

  // Init modules (hardware + defaults)
  AdcModule_Begin();
  PwmModule_Begin();
  CanModule_Begin();
  SpeedControllerModule_Begin();

  // Start module tasks
  CanModule_StartTask();
  AdcModule_StartTask();
  PwmModule_StartTask();
  SpeedControllerModule_StartTask();
}

void loop() {
  // Everything runs in module tasks.
  delay(1000);
}

#else // BUILD_PWM_TEST

// =============================================================================
// PWM Test Code
// Cycles between default PWM values and 1200/2000 mV every 10 seconds
// =============================================================================

#include <Arduino.h>

#include "pwm_module.h"
#include "shared_state.h"
#include "sl_config.h"

void setup() {
  Serial.begin(115200);
  delay(150);

  Serial.println("PWM Test Mode");
  Serial.println("Output pattern: Default -> 1200/2000 mV -> Default (10s each)");

  // Initialize relay pin and activate it (HIGH = relay active)
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Ensure LOW first
  delay(10);
  digitalWrite(RELAY_PIN, HIGH); // Then set HIGH
  delay(10);
  
  // Verify relay state
  int relay_state = digitalRead(RELAY_PIN);
  Serial.printf("Relay pin (IO%d) set to: %s\r\n", RELAY_PIN, relay_state == HIGH ? "HIGH (ACTIVE)" : "LOW (INACTIVE)");
  if (relay_state != HIGH) {
    Serial.println("WARNING: Relay pin is not HIGH!");
  }

  // Initialize PWM module only
  PwmModule_Begin();
  PwmModule_StartTask();
}

void loop() {
  // Ensure relay stays HIGH (in case something resets it)
  digitalWrite(RELAY_PIN, HIGH);
  
  // Default values: 0.350V and 0.700V (350mV and 700mV)
  float default_v1 = DEFAULT_SIGNAL_S1_V;  // 0.350V
  float default_v2 = DEFAULT_SIGNAL_S2_V;  // 0.700V

  // Test values: 1200mV = 1.200V, 2000mV = 2.000V
  float test_v1 = 1.200f;  // 1200 mV
  float test_v2 = 2.000f;  // 2000 mV

  // Output default values
  SharedState_SetDesiredOutputs(default_v1, default_v2);
  Serial.printf("Output: Default values - S1=%.3fV (%.0fmV), S2=%.3fV (%.0fmV)\r\n",
                default_v1, default_v1 * 1000.0f, default_v2, default_v2 * 1000.0f);
  delay(10000);  // 10 seconds

  // Output test values (1200mV and 2000mV)
  SharedState_SetDesiredOutputs(test_v1, test_v2);
  Serial.printf("Output: Test values - S1=%.3fV (%.0fmV), S2=%.3fV (%.0fmV)\r\n",
                test_v1, test_v1 * 1000.0f, test_v2, test_v2 * 1000.0f);
  delay(10000);  // 10 seconds

  // Loop repeats automatically
}

#endif // !BUILD_PWM_TEST

#endif // BUILD_TEST_LOGGER

