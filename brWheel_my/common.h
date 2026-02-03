#pragma once

#include "ffb_pro.h"

extern s16a accel, clutch, hbrake;
extern s32a brake;

extern volatile TEffectState gEffectStates[MAX_EFFECTS + 1];

extern cFFB gFFB;
extern BRFFB brWheelFFB;

void InitPWM();
void SetPWM (s32v *torque);

void InitInputs();
void InitButtons();
void readInputButtons(uint16_t& buttons, uint8_t& hat);

void configCDC();
