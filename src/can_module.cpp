#include "can_module.h"

#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>
#include <esp32_can.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared_state.h"

static TaskHandle_t g_can_task = nullptr;

static void sendObdSpeedRequest() {
  CAN_FRAME tx;
  tx.rtr = 0;
  tx.id = OBD_REQ_ID;
  tx.extended = (OBD_REQ_ID > 0x7FF);
  tx.length = 8;
  tx.data.uint8[0] = 0x02;
  tx.data.uint8[1] = 0x01;
  tx.data.uint8[2] = OBD_PID_SPEED;
  tx.data.uint8[3] = 0x00;
  tx.data.uint8[4] = 0x00;
  tx.data.uint8[5] = 0x00;
  tx.data.uint8[6] = 0x00;
  tx.data.uint8[7] = 0x00;
  CAN0.sendFrame(tx);
}

static void onCanRx(CAN_FRAME *frame) {
  if (!frame) return;
  uint32_t now = millis();

#if SPEED_SOURCE_OBD
  // Expect OBD positive response: [len, 0x41, 0x0D, speed_kmh, ...]
  if (frame->extended) return;
  if (frame->id < 0x7E8 || frame->id > 0x7EF) return;
  if (frame->length < 4) return;
  if (frame->data.byte[1] != 0x41) return;
  if (frame->data.byte[2] != OBD_PID_SPEED) return;

  SharedState_SetSpeed(frame->data.byte[3], now);
#else
  if (frame->id != SPEED_FRAME_ID) return;
  if (SPEED_BYTE_INDEX >= frame->length) return;
  SharedState_SetSpeed(frame->data.byte[SPEED_BYTE_INDEX], now);
#endif
}

static void canTask(void *param) {
  (void)param;

  uint32_t last_obd_ms = 0;

  for (;;) {
    uint32_t now = millis();

#if SPEED_SOURCE_OBD
    if ((uint32_t)(now - last_obd_ms) >= OBD_REQ_INTERVAL_MS) {
      sendObdSpeedRequest();
      last_obd_ms = now;
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void CanModule_Begin() {
  g_speed_kmh = 0;
  g_speed_last_update_ms = 0;

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

