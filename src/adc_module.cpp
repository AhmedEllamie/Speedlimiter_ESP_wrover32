/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#include "adc_module.h"

#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared_state.h"

// -----------------------------------------------------------------------------
// ADC config (APS inputs)
// -----------------------------------------------------------------------------
static const adc_bits_width_t ADC_WIDTH = ADC_WIDTH_BIT_12;
static const adc_atten_t ADC_ATTEN = ADC_ATTEN_DB_12; // ~0-3.3V
static const uint32_t DEFAULT_VREF_MV = 1100;

static const adc1_channel_t APS2_CH = ADC1_CHANNEL_0; // GPIO36
static const adc1_channel_t APS1_CH = ADC1_CHANNEL_3; // GPIO39

static esp_adc_cal_characteristics_t adc1_chars;
static bool adc1_ok = false;

static TaskHandle_t g_adc_task = nullptr;

static void setupAdc() {
  adc1_ok = false;
  if (adc1_config_width(ADC_WIDTH) != ESP_OK) return;
  if (adc1_config_channel_atten(APS1_CH, ADC_ATTEN) != ESP_OK) return;
  if (adc1_config_channel_atten(APS2_CH, ADC_ATTEN) != ESP_OK) return;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF_MV, &adc1_chars);
  adc1_ok = true;
}

static float readVoltageAdc1(adc1_channel_t ch) {
  if (!adc1_ok) return NAN;
  int raw = adc1_get_raw(ch);
  if (raw < 0) return NAN;
  uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc1_chars);
  return (float)mv / 1000.0f;
}

static void adcTask(void *param) {
  (void)param;

  float last_v1 = DEFAULT_SIGNAL_S1_V;
  float last_v2 = DEFAULT_SIGNAL_S2_V;

  for (;;) {
    uint32_t now = millis();

    float aps1 = readVoltageAdc1(APS1_CH);
    float aps2 = readVoltageAdc1(APS2_CH);

    // Convert to ECU-level volts using SRD scaling
    if (!isnan(aps1)) last_v1 = aps1 * PEDAL_SCALE;
    if (!isnan(aps2)) last_v2 = aps2 * PEDAL_SCALE;

    SharedState_SetAps(last_v1, last_v2, now);

    vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_INTERVAL_MS));
  }
}

void AdcModule_Begin() {
  g_aps1_v = DEFAULT_SIGNAL_S1_V;
  g_aps2_v = DEFAULT_SIGNAL_S2_V;
  g_aps_last_update_ms = 0;

  setupAdc();
}

void AdcModule_StartTask() {
  if (g_adc_task) return;

  xTaskCreatePinnedToCore(
      adcTask,
      "ADC_TASK",
      2048, // words
      nullptr,
      3,
      &g_adc_task,
      1);
}

#else

void AdcModule_Begin() {}
void AdcModule_StartTask() {}

#endif // !BUILD_TEST_LOGGER

