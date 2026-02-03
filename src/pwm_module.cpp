/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#include "pwm_module.h"

#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>

#include "driver/ledc.h"
#if USE_DAC_OUTPUT
#include "driver/dac.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared_state.h"

// -----------------------------------------------------------------------------
// PWM settings
// -----------------------------------------------------------------------------
static const uint32_t PWM_FREQ_HZ = 5000;
static const ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_10_BIT; // 0..1023
static const uint32_t PWM_MAX_DUTY = 1023;

static TaskHandle_t g_pwm_task = nullptr;

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void setupOutputs() {
#if USE_DAC_OUTPUT
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  dac_output_voltage(DAC_CHANNEL_1, 0);
  dac_output_voltage(DAC_CHANNEL_2, 0);
#else
  ledc_timer_config_t timer0 = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = PWM_RESOLUTION,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = PWM_FREQ_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&timer0);

  ledc_channel_config_t ch0 = {
      .gpio_num = SDPS1_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0,
  };
  ledc_channel_config(&ch0);

  ledc_channel_config_t ch1 = {
      .gpio_num = SDPS2_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_1,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0,
  };
  ledc_channel_config(&ch1);
#endif
}

static void outputVoltagesInternal(float internal_v1, float internal_v2) {
  internal_v1 = clampf(internal_v1, 0.0f, 3.3f);
  internal_v2 = clampf(internal_v2, 0.0f, 3.3f);

#if USE_DAC_OUTPUT
  uint8_t dac1 = (uint8_t)((internal_v1 / 3.3f) * 255.0f);
  uint8_t dac2 = (uint8_t)((internal_v2 / 3.3f) * 255.0f);
  dac_output_voltage(DAC_CHANNEL_1, dac1);
  dac_output_voltage(DAC_CHANNEL_2, dac2);
#else
  uint32_t duty1 = (uint32_t)((internal_v1 / 3.3f) * (float)PWM_MAX_DUTY);
  uint32_t duty2 = (uint32_t)((internal_v2 / 3.3f) * (float)PWM_MAX_DUTY);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty1);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty2);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
#endif
}

static void outputEcuVoltages(float ecu_v1, float ecu_v2) {
  float max_ecu_v = 3.3f / OUTPUT_DIVIDER_GAIN;
  ecu_v1 = clampf(ecu_v1, 0.0f, max_ecu_v);
  ecu_v2 = clampf(ecu_v2, 0.0f, max_ecu_v);

  float internal_v1 = ecu_v1 * OUTPUT_DIVIDER_GAIN;
  float internal_v2 = ecu_v2 * OUTPUT_DIVIDER_GAIN;
  outputVoltagesInternal(internal_v1, internal_v2);
}

static void pwmTask(void *param) {
  (void)param;

  for (;;) {
    float v1 = DEFAULT_SIGNAL_S1_V;
    float v2 = DEFAULT_SIGNAL_S2_V;
    SharedState_GetDesiredOutputs(&v1, &v2);

    outputEcuVoltages(v1, v2);

    vTaskDelay(pdMS_TO_TICKS(PWM_UPDATE_INTERVAL_MS));
  }
}

void PwmModule_Begin() {
  // Default to safe pass-through floors until logic publishes a new desired value.
  SharedState_SetDesiredOutputs(DEFAULT_SIGNAL_S1_V, DEFAULT_SIGNAL_S2_V);

  setupOutputs();

  // Push initial value immediately.
  outputEcuVoltages(DEFAULT_SIGNAL_S1_V, DEFAULT_SIGNAL_S2_V);
}

void PwmModule_StartTask() {
  if (g_pwm_task) return;

  xTaskCreatePinnedToCore(
      pwmTask,
      "PWM_TASK",
      2048, // words
      nullptr,
      3,
      &g_pwm_task,
      1);
}

#else

void PwmModule_Begin() {}
void PwmModule_StartTask() {}

#endif // !BUILD_TEST_LOGGER

