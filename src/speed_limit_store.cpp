/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#include "speed_limit_store.h"

#include "shared_state.h"
#include "sl_config.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>
#include <Preferences.h>

static Preferences s_prefs;
static bool s_started = false;

static uint16_t clampSpeedLimitKmh(int v) {
  if (v < 0) return 0;
  if (v > 250) return 250;
  return (uint16_t)v;
}

void SpeedLimitStore_Begin() {
  if (s_started) return;

  s_prefs.begin(PREF_NS, false);
  uint16_t sl = s_prefs.getUShort(PREF_KEY_SL, SPEED_LIMIT_DEFAULT_KMH);
  sl = clampSpeedLimitKmh((int)sl);

  SharedState_SetSpeedLimitKmh(sl);
  s_started = true;
}

uint16_t SpeedLimitStore_GetKmh() { return SharedState_GetSpeedLimitKmh(); }

void SpeedLimitStore_SetKmh(uint16_t kmh) {
  if (!s_started) SpeedLimitStore_Begin();

  uint16_t v = clampSpeedLimitKmh((int)kmh);
  SharedState_SetSpeedLimitKmh(v);
  s_prefs.putUShort(PREF_KEY_SL, v);
}

void SpeedLimitStore_ResetToDefault() { SpeedLimitStore_SetKmh(SPEED_LIMIT_DEFAULT_KMH); }

#else

void SpeedLimitStore_Begin() {}

uint16_t SpeedLimitStore_GetKmh() { return SPEED_LIMIT_DEFAULT_KMH; }

void SpeedLimitStore_SetKmh(uint16_t) {}

void SpeedLimitStore_ResetToDefault() {}

#endif

