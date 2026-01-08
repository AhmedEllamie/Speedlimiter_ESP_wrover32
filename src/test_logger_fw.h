// Logger firmware (compiled only when BUILD_TEST_LOGGER=1)
//
// Implements:
// 1) Read OBD-II speed/RPM + APS1/APS2 and store 4-value records (Speed,RPM,APS1,APS2)
// 2) Save records into Preferences (NVS) in a compact binary format (8 bytes/record)
// 3) USB CLI: ONLY "log" -> dump all stored data (CSV) then clear log for next drive
// 4) Effective storage: chunked binary + fixed sample interval + compact packing
//
// Notes:
// - Default logging rate: 1Hz (every 1000ms)
// - Storage is limited by NVS partition size. If full, logging stops but data remains dumpable.

#pragma once

#include <Arduino.h>

#include <Preferences.h>
#include <esp32_can.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "esp_system.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

// ------------------------- Hardware pins (match main FW) ----------------------
static const int TEST_CAN_RX_PIN = 32;
static const int TEST_CAN_TX_PIN = 33;
static const int TEST_APS1_PIN = 39; // ADC1_CH3
static const int TEST_APS2_PIN = 36; // ADC1_CH0

// ------------------------------ OBD constants ---------------------------------
static const uint16_t TEST_OBD_REQ_ID = 0x7DF;
static const uint8_t  TEST_OBD_MODE_CURRENT = 0x01;
static const uint8_t  TEST_PID_SPEED = 0x0D;
static const uint8_t  TEST_PID_RPM   = 0x0C;

static const uint32_t TEST_CAN_BAUD = 500000;
static const uint32_t TEST_OBD_REQ_INTERVAL_MS = 1000; // request speed+rpm every 1000ms (1Hz)

// ------------------------------ APS scaling -----------------------------------
// SRD correction formula: Actual = ADC * (69.6 / 47.5)
static const float TEST_PEDAL_SCALE = (69.6f / 47.5f);

// ------------------------------ ADC config ------------------------------------
static const adc_bits_width_t TEST_ADC_WIDTH = ADC_WIDTH_BIT_12;
static const adc_atten_t TEST_ADC_ATTEN = ADC_ATTEN_DB_12; // ~0-3.3V
static const uint32_t TEST_DEFAULT_VREF_MV = 1100;
static const adc1_channel_t TEST_APS2_CH = ADC1_CHANNEL_0; // GPIO36
static const adc1_channel_t TEST_APS1_CH = ADC1_CHANNEL_3; // GPIO39

static esp_adc_cal_characteristics_t test_adc1_chars;
static bool test_adc1_ok = false;

static void testSetupAdc() {
  test_adc1_ok = false;
  if (adc1_config_width(TEST_ADC_WIDTH) != ESP_OK) return;
  if (adc1_config_channel_atten(TEST_APS1_CH, TEST_ADC_ATTEN) != ESP_OK) return;
  if (adc1_config_channel_atten(TEST_APS2_CH, TEST_ADC_ATTEN) != ESP_OK) return;
  esp_adc_cal_characterize(ADC_UNIT_1, TEST_ADC_ATTEN, TEST_ADC_WIDTH, TEST_DEFAULT_VREF_MV, &test_adc1_chars);
  test_adc1_ok = true;
}

static float testReadAdcMv(adc1_channel_t ch) {
  if (!test_adc1_ok) return NAN;
  int raw = adc1_get_raw(ch);
  if (raw < 0) return NAN;
  uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &test_adc1_chars);
  return (float)mv; // millivolts at ADC pin (0..3300)
}

// ------------------------------ Latest signals --------------------------------
static volatile uint8_t test_raw_speed_kmh = 0;
static volatile uint16_t test_raw_rpm = 0;
static volatile uint32_t test_last_speed_ms = 0;
static volatile uint32_t test_last_rpm_ms = 0;

static inline bool testSpeedValid(uint32_t now) { return test_last_speed_ms != 0 && (now - test_last_speed_ms) <= 2000; }
static inline bool testRpmValid(uint32_t now) { return test_last_rpm_ms != 0 && (now - test_last_rpm_ms) <= 2000; }

// ------------------------------ Preferences log --------------------------------
static Preferences test_prefs;
static const char *TEST_PREF_NS = "drvlog";

static const uint32_t TEST_LOG_MAGIC = 0x31474F4Cu; // 'L''O''G''1'
// Bump version when log format / behavior changes (forces a fresh log)
static const uint16_t TEST_LOG_VER = 2;

// Record: 8 bytes (compact 4-value record + flags)
// speed_kmh (u8)
// rpm (u16)
// aps1_mv (u16) ECU-level (after TEST_PEDAL_SCALE)
// aps2_mv (u16) ECU-level (after TEST_PEDAL_SCALE)
// flags (u8): bit0 speed_valid, bit1 rpm_valid, bit2 aps_valid
struct __attribute__((packed)) TestLogRec {
  uint8_t speed_kmh;
  uint16_t rpm;
  uint16_t aps1_mv;
  uint16_t aps2_mv;
  uint8_t flags;
};
static_assert(sizeof(TestLogRec) == 8, "TestLogRec must be 8 bytes");

struct __attribute__((packed)) TestLogMeta {
  uint32_t magic;
  uint16_t ver;
  uint16_t rec_size;
  uint32_t interval_ms;
  uint32_t total_records;
  uint16_t chunk_count;     // number of stored chunks (0..N)
  uint16_t last_chunk_len;  // bytes used in last chunk (0..CHUNK_BYTES)
};
static_assert(sizeof(TestLogMeta) == 20, "TestLogMeta size unexpected");

// Logging rate: fixed interval (1Hz)
static const uint32_t TEST_LOG_INTERVAL_MS = 1000; // 1 Hz

// Chunking: write chunks to NVS to reduce flash wear
static const size_t TEST_CHUNK_BYTES = 1920; // multiple of 8 => 240 records/chunk
static const uint16_t TEST_MAX_CHUNKS_TO_CLEAR = 256; // sweep when clearing

static TestLogMeta test_meta{};
static uint16_t test_cur_chunk_idx = 0; // current chunk being filled (0-based)
static size_t test_cur_chunk_len = 0;
static uint8_t test_cur_chunk_buf[TEST_CHUNK_BYTES];
static bool test_logging_enabled = true;
static bool test_nvs_full_reported = false;

static void testMakeChunkKey(char *out, size_t out_sz, uint16_t idx) {
  snprintf(out, out_sz, "c%03u", (unsigned)idx);
}

static bool testLoadMeta(TestLogMeta &m) {
  size_t n = test_prefs.getBytesLength("meta");
  if (n != sizeof(TestLogMeta)) return false;
  if (test_prefs.getBytes("meta", &m, sizeof(TestLogMeta)) != sizeof(TestLogMeta)) return false;
  if (m.magic != TEST_LOG_MAGIC) return false;
  if (m.ver != TEST_LOG_VER) return false;
  if (m.rec_size != sizeof(TestLogRec)) return false;
  if (m.interval_ms == 0) return false;
  if (m.last_chunk_len > TEST_CHUNK_BYTES) return false;
  return true;
}

static void testSaveMeta(const TestLogMeta &m) {
  test_prefs.putBytes("meta", &m, sizeof(TestLogMeta));
}

static void testClearAllLog() {
  test_prefs.remove("meta");
  for (uint16_t i = 0; i < TEST_MAX_CHUNKS_TO_CLEAR; i++) {
    char key[8];
    testMakeChunkKey(key, sizeof(key), i);
    test_prefs.remove(key);
  }
  memset(&test_meta, 0, sizeof(test_meta));
  test_cur_chunk_idx = 0;
  test_cur_chunk_len = 0;
  test_logging_enabled = true;
}

static bool testFlushCurrentChunkIfNeeded() {
  if (test_cur_chunk_len == 0) return true;

  char key[8];
  testMakeChunkKey(key, sizeof(key), test_cur_chunk_idx);

  size_t wrote = test_prefs.putBytes(key, test_cur_chunk_buf, test_cur_chunk_len);
  if (wrote != test_cur_chunk_len) {
    test_logging_enabled = false;
    if (!test_nvs_full_reported) {
      test_nvs_full_reported = true;
      Serial.println("LOG: NVS full (Preferences NOT_ENOUGH_SPACE). Logging stopped. Type: log");
    }
    return false;
  }

  if (test_meta.chunk_count < (uint16_t)(test_cur_chunk_idx + 1)) test_meta.chunk_count = (uint16_t)(test_cur_chunk_idx + 1);
  test_meta.last_chunk_len = (uint16_t)test_cur_chunk_len;
  testSaveMeta(test_meta);
  return true;
}

static bool testCommitFullChunkAndAdvance() {
  char key[8];
  testMakeChunkKey(key, sizeof(key), test_cur_chunk_idx);

  size_t wrote = test_prefs.putBytes(key, test_cur_chunk_buf, TEST_CHUNK_BYTES);
  if (wrote != TEST_CHUNK_BYTES) {
    test_logging_enabled = false;
    if (!test_nvs_full_reported) {
      test_nvs_full_reported = true;
      Serial.println("LOG: NVS full (Preferences NOT_ENOUGH_SPACE). Logging stopped. Type: log");
    }
    return false;
  }

  if (test_meta.chunk_count < (uint16_t)(test_cur_chunk_idx + 1)) test_meta.chunk_count = (uint16_t)(test_cur_chunk_idx + 1);
  test_meta.last_chunk_len = (uint16_t)TEST_CHUNK_BYTES;
  testSaveMeta(test_meta);

  test_cur_chunk_idx++;
  test_cur_chunk_len = 0;
  return true;
}

static void testLoggerInitOrResume() {
  if (testLoadMeta(test_meta)) {
    if (test_meta.chunk_count == 0) {
      test_cur_chunk_idx = 0;
      test_cur_chunk_len = 0;
      return;
    }

    test_cur_chunk_idx = (uint16_t)(test_meta.chunk_count - 1);
    test_cur_chunk_len = test_meta.last_chunk_len;
    if (test_cur_chunk_len == 0) return;

    char key[8];
    testMakeChunkKey(key, sizeof(key), test_cur_chunk_idx);

    size_t len = test_prefs.getBytesLength(key);
    if (len == 0 || len > TEST_CHUNK_BYTES) {
      testClearAllLog();
      test_meta.magic = TEST_LOG_MAGIC;
      test_meta.ver = TEST_LOG_VER;
      test_meta.rec_size = sizeof(TestLogRec);
      test_meta.interval_ms = TEST_LOG_INTERVAL_MS;
      testSaveMeta(test_meta);
      return;
    }

    if (test_cur_chunk_len > len) test_cur_chunk_len = len;
    test_prefs.getBytes(key, test_cur_chunk_buf, test_cur_chunk_len);
    return;
  }

  memset(&test_meta, 0, sizeof(test_meta));
  test_meta.magic = TEST_LOG_MAGIC;
  test_meta.ver = TEST_LOG_VER;
  test_meta.rec_size = sizeof(TestLogRec);
  test_meta.interval_ms = TEST_LOG_INTERVAL_MS;
  test_meta.total_records = 0;
  test_meta.chunk_count = 0;
  test_meta.last_chunk_len = 0;
  testSaveMeta(test_meta);

  test_cur_chunk_idx = 0;
  test_cur_chunk_len = 0;
  test_nvs_full_reported = false;
}

static void testLogAppend(const TestLogRec &r) {
  if (!test_logging_enabled) return;

  if (test_cur_chunk_len + sizeof(TestLogRec) > TEST_CHUNK_BYTES) {
    if (!testCommitFullChunkAndAdvance()) return;
  }

  memcpy(&test_cur_chunk_buf[test_cur_chunk_len], &r, sizeof(TestLogRec));
  test_cur_chunk_len += sizeof(TestLogRec);
  test_meta.total_records++;
}

// ------------------------------ OBD over CAN ----------------------------------
static void testSendObdPid(uint8_t pid) {
  CAN_FRAME tx;
  tx.rtr = 0;
  tx.id = TEST_OBD_REQ_ID;
  tx.extended = 0;
  tx.length = 8;
  tx.data.uint8[0] = 0x02;
  tx.data.uint8[1] = TEST_OBD_MODE_CURRENT;
  tx.data.uint8[2] = pid;
  tx.data.uint8[3] = 0x00;
  tx.data.uint8[4] = 0x00;
  tx.data.uint8[5] = 0x00;
  tx.data.uint8[6] = 0x00;
  tx.data.uint8[7] = 0x00;
  CAN0.sendFrame(tx);
}

static void testOnCanRx(CAN_FRAME *frame) {
  if (!frame) return;
  if (frame->extended) return;
  if (frame->id < 0x7E8 || frame->id > 0x7EF) return;
  if (frame->length < 4) return;
  if (frame->data.byte[1] != 0x41) return; // positive response

  uint8_t pid = frame->data.byte[2];
  uint32_t now = millis();

  if (pid == TEST_PID_SPEED) {
    test_raw_speed_kmh = frame->data.byte[3];
    test_last_speed_ms = now;
    return;
  }

  if (pid == TEST_PID_RPM) {
    if (frame->length < 5) return;
    uint16_t v = ((uint16_t)frame->data.byte[3] << 8) | (uint16_t)frame->data.byte[4];
    test_raw_rpm = (uint16_t)(v / 4u);
    test_last_rpm_ms = now;
    return;
  }
}

static void testSetupCan() {
  CAN0.setCANPins((gpio_num_t)TEST_CAN_RX_PIN, (gpio_num_t)TEST_CAN_TX_PIN);
  CAN0.begin(TEST_CAN_BAUD);
  CAN0.watchFor();
  CAN0.setCallback(0, testOnCanRx);
}

// ------------------------------ USB CLI ---------------------------------------
static char test_cli_buf[32];
static uint8_t test_cli_len = 0;

static void testDumpLogCsvAndClear() {
  // IMPORTANT: do NOT force a flush here.
  // If NVS is full, flushing fails and you lose the ability to dump.
  // We'll dump whatever is already in NVS + the in-RAM unsaved tail.
  TestLogMeta m{};
  bool have_meta = testLoadMeta(m);

  Serial.println("CSV_START");
  Serial.println("speed_kmh,rpm,aps1_mv,aps2_mv");

  // 1) Dump all fully saved chunks from NVS (best effort)
  if (have_meta && m.chunk_count > 0) {
    for (uint16_t c = 0; c < m.chunk_count; c++) {
      char key[8];
      testMakeChunkKey(key, sizeof(key), c);

      size_t len = test_prefs.getBytesLength(key);
      if (len == 0 || len > TEST_CHUNK_BYTES) continue;

      // For the last saved chunk, trust meta.last_chunk_len (avoids printing garbage padding)
      if (c == (uint16_t)(m.chunk_count - 1) && m.last_chunk_len > 0 && m.last_chunk_len <= TEST_CHUNK_BYTES) {
        if (m.last_chunk_len <= len) len = m.last_chunk_len;
      }

      test_prefs.getBytes(key, test_cur_chunk_buf, len);

      size_t recs = len / sizeof(TestLogRec);
      const TestLogRec *p = (const TestLogRec *)test_cur_chunk_buf;
      for (size_t i = 0; i < recs; i++) {
        const TestLogRec &r = p[i];
        Serial.printf("%u,%u,%u,%u\n",
                      (unsigned)r.speed_kmh,
                      (unsigned)r.rpm,
                      (unsigned)r.aps1_mv,
                      (unsigned)r.aps2_mv);
        if ((i & 0x3F) == 0) delay(0); // yield during long dumps
      }
    }
  }

  // 2) Dump unsaved tail from RAM (if current chunk has grown beyond what meta says is saved)
  size_t saved_len = 0;
  if (have_meta && m.chunk_count > 0 && test_cur_chunk_idx == (uint16_t)(m.chunk_count - 1)) {
    saved_len = m.last_chunk_len;
    if (saved_len > test_cur_chunk_len) saved_len = test_cur_chunk_len;
  }

  if (test_cur_chunk_len > saved_len) {
    size_t len = test_cur_chunk_len - saved_len;
    const TestLogRec *p = (const TestLogRec *)&test_cur_chunk_buf[saved_len];
    size_t recs = len / sizeof(TestLogRec);
    for (size_t i = 0; i < recs; i++) {
      const TestLogRec &r = p[i];
      Serial.printf("%u,%u,%u,%u\n",
                    (unsigned)r.speed_kmh,
                    (unsigned)r.rpm,
                    (unsigned)r.aps1_mv,
                    (unsigned)r.aps2_mv);
      if ((i & 0x3F) == 0) delay(0);
    }
  }

  Serial.println("CSV_END");

  // One-command flow: after dump, clear for next drive.
  testClearAllLog();
  testLoggerInitOrResume();
  Serial.println("LOG: cleared");
}

static void testHandleCli() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      test_cli_buf[test_cli_len] = '\0';
      if (test_cli_len > 0) {
        if (!strcasecmp(test_cli_buf, "log")) {
          testDumpLogCsvAndClear();
        }
      }
      test_cli_len = 0;
      continue;
    }

    if (test_cli_len < (sizeof(test_cli_buf) - 1)) {
      test_cli_buf[test_cli_len++] = c;
    }
  }
}

// ------------------------------ Arduino entry ---------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Useful when diagnosing random restarts
  Serial.printf("RESET reason=%d\n", (int)esp_reset_reason());

  test_prefs.begin(TEST_PREF_NS, false);
  testSetupAdc();
  testSetupCan();
  testLoggerInitOrResume();

  Serial.println("TEST_LOGGER ready (1Hz). Type: log");
}

void loop() {
  static uint32_t last_speed_req_ms = 0;
  static uint32_t last_rpm_req_ms = 0;
  static uint32_t last_log_ms = 0;
  static uint32_t last_persist_ms = 0;

  uint32_t now = millis();
  testHandleCli();

  // periodic OBD requests (spread them slightly; some ECUs ignore back-to-back requests)
  if ((uint32_t)(now - last_speed_req_ms) >= TEST_OBD_REQ_INTERVAL_MS) {
    testSendObdPid(TEST_PID_SPEED);
    last_speed_req_ms = now;
  }
  if ((uint32_t)(now - last_rpm_req_ms) >= TEST_OBD_REQ_INTERVAL_MS) {
    // Keep RPM requests slightly offset from SPEED
    if ((uint32_t)(now - last_speed_req_ms) >= 150) {
      testSendObdPid(TEST_PID_RPM);
      last_rpm_req_ms = now;
    }
  }

  // periodic logging at fixed interval
  if (test_logging_enabled && (uint32_t)(now - last_log_ms) >= TEST_LOG_INTERVAL_MS) {
    last_log_ms = now;

    // Without validity flags in output, do NOT force 0 when frames are missing.
    // We keep the last known values (0 only until first successful read).
    (void)testSpeedValid(now);
    (void)testRpmValid(now);

    float aps1_adc_mv = testReadAdcMv(TEST_APS1_CH);
    float aps2_adc_mv = testReadAdcMv(TEST_APS2_CH);
    bool av = !isnan(aps1_adc_mv) && !isnan(aps2_adc_mv);

    uint16_t aps1_ecu_mv = 0;
    uint16_t aps2_ecu_mv = 0;
    if (av) {
      float v1 = (aps1_adc_mv / 1000.0f) * TEST_PEDAL_SCALE; // ECU-level volts
      float v2 = (aps2_adc_mv / 1000.0f) * TEST_PEDAL_SCALE;
      float mv1 = v1 * 1000.0f;
      float mv2 = v2 * 1000.0f;
      if (mv1 < 0) mv1 = 0;
      if (mv2 < 0) mv2 = 0;
      if (mv1 > 65535) mv1 = 65535;
      if (mv2 > 65535) mv2 = 65535;
      aps1_ecu_mv = (uint16_t)(mv1 + 0.5f);
      aps2_ecu_mv = (uint16_t)(mv2 + 0.5f);
    }

    TestLogRec r{};
    r.speed_kmh = test_raw_speed_kmh;
    r.rpm = test_raw_rpm;
    r.aps1_mv = aps1_ecu_mv;
    r.aps2_mv = aps2_ecu_mv;
    r.flags = 0;
    // Flags kept for internal debugging/backward compatibility, but not printed.
    if (testSpeedValid(now)) r.flags |= 0x01;
    if (testRpmValid(now)) r.flags |= 0x02;
    if (av) r.flags |= 0x04;

    // Publish to USB BEFORE saving (4 elements only)
    Serial.printf("%u,%u,%u,%u\n",
                  (unsigned)r.speed_kmh,
                  (unsigned)r.rpm,
                  (unsigned)r.aps1_mv,
                  (unsigned)r.aps2_mv);

    testLogAppend(r);

    // Persist to NVS periodically (reduce flash wear). Worst-case loss: last ~5 samples.
    if ((uint32_t)(now - last_persist_ms) >= 5000) {
      last_persist_ms = now;
      testFlushCurrentChunkIfNeeded();
      testSaveMeta(test_meta);
    }
  }
}


