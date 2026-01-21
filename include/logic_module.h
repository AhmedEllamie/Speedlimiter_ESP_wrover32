#pragma once

// Logic module (speed limiter state machine).
// Implemented in: `src/algorithm.cpp` or `src/new_algorithm.cpp` (build-flag selected).

void LogicModule_Begin();
void LogicModule_StartTask();

// For diagnostics / telemetry.
bool LogicModule_IsRelayActive();

