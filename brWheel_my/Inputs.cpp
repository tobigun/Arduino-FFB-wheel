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
#include "HID.h"
#include <Wire.h>
#include <digitalWriteFast.h>

//--------------------------------------- Globals --------------------------------------------------------

u8 analog_inputs_pins[] = // milos, changed to u8, from u16
{
  Z_AXIS_PIN,
  Y_AXIS_PIN,
  RX_AXIS_PIN,
  RY_AXIS_PIN
};

//----------------------------------------- Options -------------------------------------------------------

void setMatrixRow (uint8_t j, uint8_t k);
bool readSingleButton (uint8_t i);


//--------------------------------------------------------------------------------------------------------

void InitInputs() {
  //analogReference(INTERNAL); // sets 2.56V on AREF pin for Leonardo or Micro, can be EXTERNAL also
  for (u8 i = 0; i < sizeof(analog_inputs_pins); i++) {
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

void InitButtons() { // milos, added - if not using shift register, allocate some free pins for buttons
  pinMode(BUTTON0, INPUT_PULLUP);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP); // on proMicro only available if we do not use z-index
  pinModeFast(BUTTON4, OUTPUT);
  pinModeFast(BUTTON5, OUTPUT);
  pinModeFast(BUTTON6, OUTPUT);
  setMatrixRow(BUTTON4, HIGH);
  setMatrixRow(BUTTON5, HIGH);
  setMatrixRow(BUTTON6, HIGH);
  pinModeFast(BUTTON7, OUTPUT);
  setMatrixRow(BUTTON7, HIGH);
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
  byte dec = 0b1111 & inbits; //milos, only take 1st 4 bits from inbits
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
  for (uint8_t i = 0; i < 4; i++) { // rows (along X), we set each row high, one at a time
    setMatrixRow (i, LOW);
    delayMicroseconds(5); // required to avoid the detection of false button presses
    for (uint8_t j = 0; j < 4; j++) { // columns (along Y), read each button from that row by scanning over columns
      bitWrite(buttons, i * 4 + j, readSingleButton(j));
    }
    setMatrixRow (i, HIGH);
  }

  hat = decodeHat(buttons);
  buttons >>= 4; // shift out the hat bits from buttons variable
}

bool readSingleButton (uint8_t i) { // milos, added
  bool temp;
  if (i == 0) {
    temp = !bitRead(digitalReadFast(BUTTON0), B0PORTBIT); // milos, read bit4 from PINF A3 (or bit4 from PIND D4 when no LC, or bit6 from PINC D5 on leonardo/micro when XY shifter) into buttons bit0
  } else if (i == 1) {
    temp = !bitRead(digitalReadFast(BUTTON1), B1PORTBIT); // milos, read bit1 from PINF A4 (or bit3 from PINB D14 on ProMicro) into buttons bit1
  } else if (i == 2) {
    temp = !bitRead(digitalReadFast(BUTTON2), B2PORTBIT); // milos, read bit0 from PINF A5 (or bit1 from PINB D15 on ProMicro) into buttons bit2
  } else if (i == 3) {
    temp = !bitRead(digitalReadFast(BUTTON3), B3PORTBIT); // milos, read bit6 from PIND D12 into buttons bit3
  } else if (i == 4) {
    temp = !bitRead(digitalReadFast(BUTTON4), B4PORTBIT); // milos, read bit7 from PIND D6 into buttons bit4
  } else if (i == 5) {
    temp = !bitRead(digitalReadFast(BUTTON5), B5PORTBIT); // milos, read bit6 from PINE D7 into buttons bit5
  } else if (i == 6) {
    temp = !bitRead(digitalReadFast(BUTTON6), B6PORTBIT); // milos, read bit4 from PINB D8 into buttons bit6
  } else {
    temp = false;
  }
  return temp;
}

void setMatrixRow (uint8_t j, uint8_t k) { // milos, added
  if (j == 0) {
    digitalWriteFast(BUTTON4, k);
  } else if (j == 1) {
    digitalWriteFast(BUTTON5, k);
  } else if (j == 2) {
    digitalWriteFast(BUTTON6, k);
  } else if (j == 3) {
    digitalWriteFast(BUTTON7, k);
  }
}
