#pragma once

// SpeedController module (new algorithm)
// - Reads linked snapshot of (CAN_SPEED, CAN_RPM, APS1/APS2) from shared state
// - Compares CAN_SPEED vs SL (speed limit)
// - If (SL - CAN_SPEED) > 10 -> do nothing (relay OFF, pass-through outputs)
// - Else -> SpeedController:
//    * capture APS inputs as outputs
//    * relay ON
//    * if speed increases -> reduce APS outputs in staged steps (car response delay aware)
//    * relax outputs back up slowly when safely under limit and not rising
//    * if APS input < APS output -> relay OFF (driver override)
// - Prints: CAN_SPEED, RPM, APS_in, APS_out, Relay_status
void SpeedControllerModule_Begin();
void SpeedControllerModule_StartTask();

