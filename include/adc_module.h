/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#pragma once

// ADC module (APS inputs)
// - Own task continuously samples APS1/APS2 and publishes ECU-level volts.
void AdcModule_Begin();
void AdcModule_StartTask();

