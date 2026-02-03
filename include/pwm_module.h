/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#pragma once

// PWM/DAC output module
// - Reads desired ECU-level output voltages from shared globals
// - Generates corresponding PWM (IO21/IO22) or DAC (IO25/IO26) signals
void PwmModule_Begin();
void PwmModule_StartTask();

