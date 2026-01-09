#pragma once

// Logic module
// - Reads CAN speed + APS inputs
// - Applies speed limiting logic
// - Controls relay and publishes desired outputs for PWM module
void LogicModule_Begin();
void LogicModule_StartTask();

