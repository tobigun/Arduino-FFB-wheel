/*
  Copyright 2015  Etienne Saint-Paul
  Copyright 2017  Fernando Igor  (fernandoigor [at] msn [dot] com)
  Copyright 2018-2025  Milos Rankovic (ranenbg [at] gmail [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include <Arduino.h>

#include "config.h"
#include "common.h"
#include "ffb_pro.h"
#include "debug.h"
#include "ffb_hid.h"
#ifdef __AVR__
#include <digitalWriteFast.h>
#include <avdweb_AnalogReadFast.h>
#else
#define pinModeFast pinMode
#define digitalWriteFast(pin, val) digitalWrite(pin, val)
#define analogReadFast analogRead
#endif

uint8_t analog_inputs_pins[] =
{
  Z_AXIS_PIN,
  Y_AXIS_PIN,
  RX_AXIS_PIN,
  RY_AXIS_PIN
};

// first index contains the newest sample per axis
static uint16_t axisSamples[AVG_AXIS_ID_COUNT][AVG_AXIS_NUM_MAX_SAMPLES];


void initButtonMatrix();
void setMatrixRow(uint8_t row, uint8_t value);
bool readMatrixCol(uint8_t col);


void initInputs() {
  for (uint8_t i = 0; i < sizeof(analog_inputs_pins); i++) {
    pinMode(analog_inputs_pins[i], INPUT);
  }

  initButtonMatrix();

  pinMode(PROFILE_SWITCH_PIN, INPUT_PULLUP);

  if (digitalRead(RX_AXIS_PIN) == HIGH) {
    useCombinedAxes = true;
  }
}

void readAxisSamples(uint8_t axisIndex, uint8_t sampleCount, uint8_t pin) {
  uint16_t* samples = axisSamples[axisIndex];
  memmove(&samples[sampleCount], &samples[0], (AVG_AXIS_NUM_MAX_SAMPLES - sampleCount) * sizeof(int16_t));
  for (int8_t i = sampleCount - 1; i >= 0; --i) {
    samples[i] = analogReadFast(pin);
  }
}

int16_t getAxisValue(uint8_t axisIndex, uint8_t outputBits, uint8_t sampleCount) {
  uint32_t axisXSum = 0;
  for (int i = 0; i < sampleCount; ++i) {
    axisXSum += axisSamples[axisIndex][i];
  }

  axisXSum = (outputBits >= ANALOG_BITS)
    ? axisXSum << (outputBits - ANALOG_BITS)
    : axisXSum >> (ANALOG_BITS - outputBits);
  return axisXSum / sampleCount;
}

HID_PROFILE_ID readHidProfileId() {
  return digitalRead(PROFILE_SWITCH_PIN) ? GENERIC_AXES : DRIVING_WHEEL;
}

#define PEDALS_DISCONNECTED_THRESHOLD (AXIS_LOG_MAX - 2)
#define PEDALS_CONNECTED_THRESHOLD (AXIS_LOG_MAX * 3 / 4)

bool checkPedalsConnected(int16_t axisY, int16_t axisZ) {
  static uint32_t yzHighTimeMs = 0;
  static bool pedalsConnected = true;
  static bool startUp = true;

  // Pedal inputs are pulled high (equal to pedals fully pushed) if no pedals are connected
  if (pedalsConnected && (axisY >= PEDALS_DISCONNECTED_THRESHOLD && axisZ >= PEDALS_DISCONNECTED_THRESHOLD)) {
    if (startUp) { // on startup consider pedals as disconnected immediately if both axes are high (as this is very unlikely at startup)
      pedalsConnected = false;
    } else { // after startup require them to be high for some time to avoid false disconnect detections
      uint32_t curMs = millis();
      if (yzHighTimeMs == 0) {
        yzHighTimeMs = curMs;
      } else if (curMs - yzHighTimeMs > 5000) { // if pedals are high for more than 5 seconds, consider them disconnected
        pedalsConnected = false;
        yzHighTimeMs = 0;
      }
    }
  } else if (!pedalsConnected && (axisY < PEDALS_CONNECTED_THRESHOLD || axisZ < PEDALS_CONNECTED_THRESHOLD)) {
    pedalsConnected = true;
  } else {
    yzHighTimeMs = 0;
  }

  startUp = false;
  return pedalsConnected;
}

void initButtonMatrix() { // if not using shift register, allocate some free pins for buttons
  pinMode(BUTTON_MATRIX_COL0_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MATRIX_COL1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MATRIX_COL2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MATRIX_COL3_PIN, INPUT_PULLUP);

  pinModeFast(BUTTON_MATRIX_ROW0_PIN, OUTPUT);
  pinModeFast(BUTTON_MATRIX_ROW1_PIN, OUTPUT);
  pinModeFast(BUTTON_MATRIX_ROW2_PIN, OUTPUT);
  pinModeFast(BUTTON_MATRIX_ROW3_PIN, OUTPUT);
  setMatrixRow(BUTTON_MATRIX_ROW0_PIN, HIGH);
  setMatrixRow(BUTTON_MATRIX_ROW1_PIN, HIGH);
  setMatrixRow(BUTTON_MATRIX_ROW2_PIN, HIGH);
  setMatrixRow(BUTTON_MATRIX_ROW3_PIN, HIGH);
}

// decodes hat switch values into only 1st 4 buttons (button0-up, button1-right, button2-down, button3-left)
uint8_t decodeHat(uint16_t bits) {
  switch ((uint8_t) bits & 0b1111) { // only take 1st 4 bits from inbits
    case 0b0001: return 1; // up
    case 0b0011: return 2; // up_right
    case 0b0010: return 3; // right
    case 0b0110: return 4; // down_right
    case 0b0100: return 5; // down
    case 0b1100: return 6; // down_left
    case 0b1000: return 7; // left
    case 0b1001: return 8; // up_left
    default: return 0; // center
  }
}

// buttons 0-3 of are columns j
// buttons 4-7 of are rows i
// Matrix element is Bij
//     D4  A4  A5  D12
// D6 |b11 b12 b13 b14|
// D7 |b21 b22 b23 b24|
// D8 |b31 b32 b33 b34|
// D5 |b41 b42 b43 b44|
void readInputButtons(uint16_t& buttons, uint8_t& hat) {
  for (uint8_t i = 0; i < 4; i++) { // rows (along X), we set each row low, one at a time
    setMatrixRow(i, LOW);
    delayMicroseconds(5); // required to avoid the detection of false button presses
    for (uint8_t j = 0; j < 4; j++) { // columns (along Y), read each button from that row by scanning over columns
      bitWrite(buttons, i * 4 + j, readMatrixCol(j));
    }
    setMatrixRow(i, HIGH);
  }

  hat = decodeHat(buttons);
  buttons >>= 4; // shift out the hat bits from buttons variable
}

#ifdef __AVR__
#define READ_MATRIX_COL_(pin, bit) (!bitRead(digitalReadFast(pin), (bit)))
#define READ_MATRIX_COL(id) READ_MATRIX_COL_(BUTTON_MATRIX_COL##id##_PIN, BMCOL##id##_PORTBIT)
#else
#define READ_MATRIX_COL(id) (digitalRead(BUTTON_MATRIX_COL##id##_PIN))
#endif

bool readMatrixCol(uint8_t col) {
  if (col == 0) {
    return READ_MATRIX_COL(0);
  } else if (col == 1) {
    return READ_MATRIX_COL(1);
  } else if (col == 2) {
    return READ_MATRIX_COL(2);
  } else if (col == 3) {
    return READ_MATRIX_COL(3);
  } else {
    return false;
  }
}

void setMatrixRow(uint8_t row, uint8_t value) {
  if (row == 0) {
    digitalWriteFast(BUTTON_MATRIX_ROW0_PIN, value);
  } else if (row == 1) {
    digitalWriteFast(BUTTON_MATRIX_ROW1_PIN, value);
  } else if (row == 2) {
    digitalWriteFast(BUTTON_MATRIX_ROW2_PIN, value);
  } else if (row == 3) {
    digitalWriteFast(BUTTON_MATRIX_ROW3_PIN, value);
  }
}
