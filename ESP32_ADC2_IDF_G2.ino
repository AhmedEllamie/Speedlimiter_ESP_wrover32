#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"
#include <esp32_can.h>
#include <Preferences.h>

#define ADC1_PIN 26    // ADC2_CH9 (GPIO26)
#define ADC2_PIN 27    // ADC2_CH7 (GPIO27)
#define PWM_PIN_1 21   // PWM output for ADC1
#define PWM_PIN_2 22   // PWM output for ADC2
#define CONF_BTN 4
#define BUZ 18
#define SMUX_1 14
#define SMUX_2 23
#define SPEED_THRESHOLD 20      // km/h threshold
#define SPEED_HYSTERESIS 2      // km/h hysteresis to prevent oscillation
#define TRANSITION_DURATION 200 // ms for smooth transition

adc_cali_handle_t cali_handle_1 = NULL;
adc_cali_handle_t cali_handle_2 = NULL;
adc_oneshot_unit_handle_t adc_handle;

Preferences preferences;
float saved_v1 = 0.0;
float saved_v2 = 0.0;
float current_output_v1 = 0.0;
float current_output_v2 = 0.0;
bool limiter_active = false;
uint8_t last_speed = 0;

void playDangerTone() {
  // Play warning tone on buzzer
  tone(BUZ, 1000, 200);  // 1kHz for 200ms
  delay(250);
  tone(BUZ, 1500, 200);  // 1.5kHz for 200ms
}

void smoothTransition(float target_v1, float target_v2, int duration_ms) {
  int steps = 50; // More steps for smoother transition
  int delay_per_step = duration_ms / steps;
  
  for(int i = 0; i <= steps; i++) {
    // Use ease-in-out curve for smoother feel
    float t = (float)i / steps;
    float progress = t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
    
    float v1 = current_output_v1 + (target_v1 - current_output_v1) * progress;
    float v2 = current_output_v2 + (target_v2 - current_output_v2) * progress;
    
    outputVoltages(v1, v2);
    delay(delay_per_step);
  }
  
  // Update current output values
  current_output_v1 = target_v1;
  current_output_v2 = target_v2;
}

void gotHundred(CAN_FRAME *frame)
{
  if(frame->id == 0x7E8) {
    uint8_t speed = frame->data.byte[3];
    
    // Activate limiter when speed exceeds threshold
    if(speed >= SPEED_THRESHOLD-(SPEED_THRESHOLD/4)) {
      if(!limiter_active) {
        Serial.println("⚠️ Speed limit reached - Activating limiter");
        
        // Switch SMUX to HIGH to override pedal signal
        current_output_v1 = readVoltage(ADC_CHANNEL_9, cali_handle_1) * 1.46 - 0.025;
        current_output_v2 = readVoltage(ADC_CHANNEL_7, cali_handle_2) * 1.46 - 0.025;
        outputVoltages(current_output_v1, current_output_v2/2.0);
        digitalWrite(SMUX_1, HIGH);
        digitalWrite(SMUX_2, HIGH);
        
        // Smooth transition to limited throttle position
        smoothTransition(saved_v1, saved_v2, TRANSITION_DURATION);
        
        limiter_active = true;
        Serial.printf("Limiter active: Holding at V1=%.3f V, V2=%.3f V\n", saved_v1, saved_v2);
        playDangerTone();
      }
      
      // Keep outputting limited throttle position
      outputVoltages(saved_v1, saved_v2);
      current_output_v1 = saved_v1;
      current_output_v2 = saved_v2;
    }
    // Deactivate limiter when speed drops below threshold with hysteresis
    else if(speed <= SPEED_THRESHOLD + SPEED_HYSTERESIS) {
      if(limiter_active) {
        Serial.println("✓ Speed below limit - Releasing limiter");
        
        // Read current pedal position
        float pedal_v1 = readVoltage(ADC_CHANNEL_9, cali_handle_1) * 1.46 - 0.025;
        float pedal_v2 = readVoltage(ADC_CHANNEL_7, cali_handle_2) * 1.46 - 0.025;
        
        // Smooth transition back to pedal control
        smoothTransition(pedal_v1 * 1, pedal_v2 /2.0, TRANSITION_DURATION);
        
        // Switch SMUX to LOW to restore pedal signal
        digitalWrite(SMUX_1, LOW);
        digitalWrite(SMUX_2, LOW);
        
        limiter_active = false;
        Serial.println("✓ Pedal control restored");
      }
    }
    // In hysteresis zone - maintain current state
    /*else if(limiter_active) {
      // Keep limiting
      outputVoltages(saved_v1, saved_v2);
    }*/
    
    // Log speed changes
    if(speed != last_speed) {
      Serial.printf("Speed: %d km/h | Limiter: %s | Output: V1=%.3f V, V2=%.3f V\n", 
                    speed, limiter_active ? "ACTIVE" : "OFF", current_output_v1, current_output_v2);
      last_speed = speed;
    }
  }
}

void send_OBD_cmd()
{
  CAN_FRAME txFrame;
  txFrame.rtr = 0;
  txFrame.id = 0x7DF;
  txFrame.extended = false;
  txFrame.length = 8;
  txFrame.data.uint8[0] = 0x02;
  txFrame.data.uint8[1] = 0x01;
  txFrame.data.uint8[2] = 0x0D;
  txFrame.data.uint8[3] = 0x00;
  txFrame.data.uint8[4] = 0x00;
  txFrame.data.uint8[5] = 0x00;
  txFrame.data.uint8[6] = 0x00;
  txFrame.data.uint8[7] = 0x00;
  CAN0.sendFrame(txFrame);
}

void setupADC() {
  // Initialize ADC unit
  adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT_2,
  };
  adc_oneshot_new_unit(&init_config, &adc_handle);

  // Configure ADC channel
  adc_oneshot_chan_cfg_t chan_cfg;
  chan_cfg.atten = ADC_ATTEN_DB_12;
  chan_cfg.bitwidth = ADC_BITWIDTH_12;

  adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_9, &chan_cfg); // GPIO26
  adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &chan_cfg); // GPIO27

  adc_cali_line_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_2,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
  };

  adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle_1);
  adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle_2);
}

void setupOutputs() {
  // LEDC timer config
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 5000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // PWM channel 1 for GPIO22
  ledc_channel_config_t ledc_channel_1 = {
    .gpio_num = PWM_PIN_1,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&ledc_channel_1);

  // PWM channel 2 for GPIO21
  ledc_channel_config_t ledc_channel_2 = {
    .gpio_num = PWM_PIN_2,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&ledc_channel_2);
}

float readVoltage(adc_channel_t channel, adc_cali_handle_t cali_handle) {
  int raw = 0;
  adc_oneshot_read(adc_handle, channel, &raw);
  int voltage_mv = 0;
  adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv);
  return voltage_mv / 1000.0;
}

void outputVoltages(float v1, float v2) {
  // Output to PWM channel 0 (GPIO21)
  uint32_t duty1 = (uint32_t)(v1 / 3.3 * 1023);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty1);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  // Output to PWM channel 1 (GPIO22)
  uint32_t duty2 = (uint32_t)(v2 / 3.3 * 1023);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty2);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void saveVoltages() {
  float v1 = readVoltage(ADC_CHANNEL_9, cali_handle_1) * 1.46 - 0.025;
  float v2 = readVoltage(ADC_CHANNEL_7, cali_handle_2) * 1.46 - 0.025;
  
  preferences.begin("voltages", false);
  preferences.putFloat("v1", v1 * 1);
  preferences.putFloat("v2", v2 /2.0);
  preferences.end();
  
  saved_v1 = v1 * 1;
  saved_v2 = v2 / 2.0;
  
  // Confirmation tone
  tone(BUZ, 2000, 100);
  delay(150);
  tone(BUZ, 2500, 100);
  
  Serial.printf("✓ Voltages saved: V1=%.3f V, V2=%.3f V\n", saved_v1, saved_v2);
  delay(1000);
  ESP.restart();
}

void loadVoltages() {
  preferences.begin("voltages", true);
  saved_v1 = preferences.getFloat("v1", 0.0);
  saved_v2 = preferences.getFloat("v2", 0.0);
  preferences.end();
  
  Serial.printf("Loaded saved voltages: V1=%.3f V, V2=%.3f V\n", saved_v1, saved_v2);
}

void setup() {
  Serial.begin(115200);
  
  // Configure pins
  pinMode(SMUX_1, OUTPUT);
  pinMode(SMUX_2, OUTPUT);
  pinMode(CONF_BTN, INPUT_PULLUP);
  pinMode(BUZ, OUTPUT);
  
  digitalWrite(SMUX_1, HIGH);
  digitalWrite(SMUX_2, HIGH);
  
  setupADC();
  setupOutputs();
  delay(2000);
  
  // Load saved voltages from memory
  loadVoltages();
  
  // Initialize with saved values
  current_output_v1 = saved_v1;
  current_output_v2 = saved_v2;
  outputVoltages(saved_v1, saved_v2);
  
  Serial.println("✅ Speed Limiter System Ready");
  Serial.printf("Speed Threshold: %d km/h | Limited Throttle: V1=%.3f V, V2=%.3f V\n", 
                SPEED_THRESHOLD, saved_v1, saved_v2);
  
  CAN0.setCANPins(GPIO_NUM_32, GPIO_NUM_33);
  CAN0.begin(500000);
  CAN0.watchFor(0x7E0, 0xF00);
  CAN0.watchFor();
  CAN0.setCallback(0, gotHundred);
}

void loop() {
  // Check if configuration button is pressed
  static bool btn_last_state = HIGH;
  bool btn_state = digitalRead(CONF_BTN);
  
  if(btn_last_state == HIGH && btn_state == LOW) {
    // Button pressed
    delay(50); // Debounce
    if(digitalRead(CONF_BTN) == LOW) {
      saveVoltages();
    }
  }
  btn_last_state = btn_state;
  
  // Send OBD request periodically
  static unsigned long last_obd_request = 0;
  if(millis() - last_obd_request > 300) {
    send_OBD_cmd();
    last_obd_request = millis();
  }
  
  // When limiter is not active, MUX passes through pedal signal automatically
  // SMUX_1 and SMUX_2 are LOW, so no action needed
  
  
  delay(100);
}