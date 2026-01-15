#pragma once

// Logic module (speed limiter state machine).
// Implemented in: src/algorithm.cpp

void LogicModule_Begin();
void LogicModule_StartTask();

// For diagnostics / telemetry.
bool LogicModule_IsRelayActive();

