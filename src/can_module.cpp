/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#include "can_module.h"

#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>
#include <esp32_can.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared_state.h"

static TaskHandle_t g_can_task = nullptr;

static void sendObdPidRequest(uint8_t pid) {
  CAN_FRAME tx;
  tx.rtr = 0;
  tx.id = OBD_REQ_ID;
  tx.extended = (OBD_REQ_ID > 0x7FF);
  tx.length = 8;
  tx.data.uint8[0] = 0x02;
  tx.data.uint8[1] = 0x01;
  tx.data.uint8[2] = pid;
  tx.data.uint8[3] = 0x00;
  tx.data.uint8[4] = 0x00;
  tx.data.uint8[5] = 0x00;
  tx.data.uint8[6] = 0x00;
  tx.data.uint8[7] = 0x00;
  CAN0.sendFrame(tx);
}

static void sendObdSpeedRequest() {
  sendObdPidRequest(OBD_PID_SPEED);
}

static void onCanRx(CAN_FRAME *frame) {
  if (!frame) return;
  uint32_t now = millis();

#if SPEED_SOURCE_OBD
  // Expect OBD positive response: [len, 0x41, PID, data...]
  if (frame->extended) return;
  if (frame->id < 0x7E8 || frame->id > 0x7EF) return;
  if (frame->length < 4) return;
  if (frame->data.byte[1] != 0x41) return;

  uint8_t pid = frame->data.byte[2];
  if (pid == OBD_PID_SPEED) {
    SharedState_SetSpeed(frame->data.byte[3], now);
  } else if (pid == OBD_PID_RPM) {
    // RPM response: [len, 0x41, 0x0C, A, B] where RPM = (256*A + B) / 4
    if (frame->length >= 5) {
      uint16_t rpm_raw = ((uint16_t)frame->data.byte[3] << 8) | frame->data.byte[4];
      uint16_t rpm = rpm_raw / 4;
      SharedState_SetRpm(rpm, now);
    }
  }
#else
  if (frame->id != SPEED_FRAME_ID) return;
  if (SPEED_BYTE_INDEX >= frame->length) return;
  SharedState_SetSpeed(frame->data.byte[SPEED_BYTE_INDEX], now);
#endif
}

static void canTask(void *param) {
  (void)param;

  uint32_t last_speed_req_ms = 0;
  uint32_t last_rpm_req_ms = 0;

  for (;;) {
    uint32_t now = millis();

#if SPEED_SOURCE_OBD
    if ((uint32_t)(now - last_speed_req_ms) >= OBD_REQ_INTERVAL_MS) {
      sendObdSpeedRequest();
      last_speed_req_ms = now;
    }
    // Request RPM at same rate as speed (25 Hz)
    if ((uint32_t)(now - last_rpm_req_ms) >= OBD_REQ_INTERVAL_MS) {
      sendObdPidRequest(OBD_PID_RPM);
      last_rpm_req_ms = now;
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void CanModule_Begin() {
  g_speed_kmh = 0;
  g_speed_last_update_ms = 0;
  g_rpm = 0;
  g_rpm_last_update_ms = 0;

  CAN0.setCANPins((gpio_num_t)CAN_RX_PIN, (gpio_num_t)CAN_TX_PIN);
  CAN0.begin(CAN_BAUD);
  CAN0.watchFor();
  CAN0.setCallback(0, onCanRx);
}

void CanModule_StartTask() {
  if (g_can_task) return;

  xTaskCreatePinnedToCore(
      canTask,
      "CAN_TASK",
      2048, // words
      nullptr,
      2,
      &g_can_task,
      1);
}

#else

void CanModule_Begin() {}
void CanModule_StartTask() {}

#endif // !BUILD_TEST_LOGGER

