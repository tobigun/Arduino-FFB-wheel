#pragma once

#include "ffb_pro.h"
#include "ffb_hid.h"

#define ANALOG_MAX 1023
#define ANALOG_BITS 10

extern s16a accel, clutch, hbrake;
extern s32a brake;

extern volatile TEffectState gEffectStates[MAX_EFFECTS + 1];

extern cFFB gFFB;

extern HidAdapter hidAdapter;

void InitPWM();
void SetPWM(s32v *torque);

void InitInputs();
void InitButtons();
void readInputButtons(uint16_t& buttons, uint8_t& hat);
bool checkPedalsConnected(int16_t axisY, int16_t axisZ);

void configCDC();

void blinkFFBclipLED();
void activateFFBclipLED(int32_t t);
