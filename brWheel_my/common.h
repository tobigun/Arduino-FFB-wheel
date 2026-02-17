#pragma once

#include <stdint.h>

#ifdef __AVR__
#define ANALOG_BITS 10
#define ANALOG_MAX ((1 << ANALOG_BITS) - 1) // 1023
#else
#define ANALOG_BITS 12
#define ANALOG_MAX ((1 << ANALOG_BITS) - 1) // 4095
#endif

#define SWAP_BITS(bits) ((((bits) << 1) & 0x2) | (((bits) >> 1) & 0x1))

#define AVG_AXIS_NUM_BITS 5
#define AVG_AXIS_NUM_MAX_SAMPLES (1 << AVG_AXIS_NUM_BITS)

enum HID_PROFILE_ID {
  GENERIC_AXES,
  DRIVING_WHEEL
};

enum {
  AVG_AXIS_ID_X = 0,
  AVG_AXIS_ID_Y,
  AVG_AXIS_ID_Z,
  AVG_AXIS_ID_RX,
  AVG_AXIS_ID_RY,
  AVG_AXIS_ID_COUNT
};

struct s32v { //  2 dimensional vector structure (for ffb and position)
  int32_t x;
};

struct s16a { //  holds individual 16bit axis properties
  int16_t val;
  int16_t min;
  int16_t max;
};

extern s16a accel;
extern s16a clutch;
extern s16a hbrake;
extern s16a brake;

void initPWM();
void setPWM(s32v *torque);

void initInputs();
void readAxisSamples(uint8_t axisIndex, uint8_t sampleCount, uint8_t pin);
int16_t getAxisValue(uint8_t axisIndex, uint8_t outputBits, uint8_t sampleCount = AVG_AXIS_NUM_MAX_SAMPLES);
HID_PROFILE_ID readHidProfileId();
void readInputButtons(uint16_t& buttons, uint8_t& hat);
bool checkPedalsConnected(int16_t axisY, int16_t axisZ);

void configCDC();

void blinkFFBclipLED();
void activateFFBclipLED(int32_t t);
