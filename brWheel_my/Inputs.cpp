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

#include "Config.h"
#include "common.h"
#include "ffb_pro.h"
#include "debug.h"
#include "ffb_hid.h"
#include <Wire.h>
#ifdef __AVR__
#include <digitalWriteFast.h>
#else
#define pinModeFast pinMode
#define digitalWriteFast(pin, val) digitalWrite(pin, val)
#endif

//--------------------------------------- Globals --------------------------------------------------------

uint8_t analog_inputs_pins[] =
{
  Z_AXIS_PIN,
  Y_AXIS_PIN,
  RX_AXIS_PIN,
  RY_AXIS_PIN
};

//----------------------------------------- Options -------------------------------------------------------

void setMatrixRow(uint8_t row, uint8_t value);
bool readMatrixCol(uint8_t col);


//--------------------------------------------------------------------------------------------------------

void InitInputs() {
  for (uint8_t i = 0; i < sizeof(analog_inputs_pins); i++) {
    pinMode(analog_inputs_pins[i], INPUT);
  }

  InitButtons();

  pinMode(PROFILE_SWITCH_PIN, INPUT_PULLUP);
  if (digitalRead(PROFILE_SWITCH_PIN) == LOW) {
    useDrivingHidProfile = true;
  }

  if (digitalRead(RX_AXIS_PIN) == HIGH) {
    useCombinedAxes = true;
  }
}

void InitButtons() { // if not using shift register, allocate some free pins for buttons
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

#define PEDALS_DISCONNECTED_THRESHOLD (ANALOG_MAX - 2)
#define PEDALS_CONNECTED_THRESHOLD (ANALOG_MAX * 3 / 4)

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

// decodes hat switch values into only 1st 4 buttons (button0-up, button1-right, button2-down, button3-left)
uint8_t decodeHat(uint16_t inbits) {
  byte hat;
  byte dec = 0b1111 & inbits; //only take 1st 4 bits from inbits
  if (dec == 1) { //up
    hat = 1;
  } else if (dec == 2) { //right
    hat = 3;
  } else if (dec == 4) { //down
    hat = 5;
  } else if (dec == 8) { //left
    hat = 7;
  } else if (dec == 3) { //up_right
    hat = 2;
  } else if (dec == 6) { //down_right
    hat = 4;
  } else if (dec == 9) { //up_left
    hat = 8;
  } else if (dec == 12) { //down_left
    hat = 6;
  } else {
    hat = 0;
  }
  return hat;
}

void readInputButtons(uint16_t& buttons, uint8_t& hat) {
  // buttons 0-3 of are columns j
  // buttons 4-7 of are rows i
  // Matrix element is Bij
  //     D4  A4  A5  D12
  // D6 |b11 b12 b13 b14|
  // D7 |b21 b22 b23 b24|
  // D8 |b31 b32 b33 b34|
  // D5 |b41 b42 b43 b44|
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
