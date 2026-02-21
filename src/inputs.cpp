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

static void initAdc();
static void initButtonMatrix();

void initInputs() {
  initAdc();

  initButtonMatrix();

  pinMode(PROFILE_SWITCH_PIN, INPUT_PULLUP);

  if (digitalRead(RX_AXIS_PIN) == HIGH) {
    useCombinedAxes = true;
  }
}

///////////////////////////////////////////////////
// Analog Axis
///////////////////////////////////////////////////

// first index contains the newest sample per axis
static uint16_t axisSamples[AXIS_COUNT][AVG_AXIS_NUM_MAX_SAMPLES];

#ifdef __AVR__
void readAxisSamples(uint8_t axisIndex, uint8_t sampleCount, uint8_t pin) {
  uint16_t* samples = axisSamples[axisIndex];
  memmove(&samples[sampleCount], &samples[0], (AVG_AXIS_NUM_MAX_SAMPLES - sampleCount) * sizeof(int16_t));
  for (int8_t i = sampleCount - 1; i >= 0; --i) {
    samples[i] = analogReadFast(pin);
  }
}
#else
#define CONV_PER_PIN 4

static volatile bool adcConversionDone = false;

static const uint8_t adcAxisPins[AXIS_COUNT] = {
  X_AXIS_PIN,
  Y_AXIS_PIN,
  Z_AXIS_PIN,
  RX_AXIS_PIN,
  RY_AXIS_PIN
};

static void ARDUINO_ISR_ATTR adcCallback() {
  adcConversionDone = true;
}

static void initAdc() {  
  for (uint8_t i = 0; i < sizeof(adcAxisPins); i++) {
    pinMode(adcAxisPins[i], INPUT);
  }

  analogContinuous(adcAxisPins, sizeof(adcAxisPins), CONV_PER_PIN, 5000, &adcCallback); //611 - 83333Hz
  analogContinuousStart();
}

void readAxisSamples() {
  if (!adcConversionDone) {
    return;
  }

  adcConversionDone = false;

  adc_continuous_result_t* results;
  if (!analogContinuousRead(&results, 0)) {
    Serial0.println("Error reading samples");
    return;
  }

  for (int axisIndex = 0; axisIndex < AXIS_COUNT; ++axisIndex) {
    uint16_t* samples = axisSamples[axisIndex];
    memmove(&samples[1], &samples[0], (AVG_AXIS_NUM_MAX_SAMPLES - 1) * sizeof(int16_t));
    samples[0] = results[axisIndex].avg_read_raw;
  }
}
#endif

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

///////////////////////////////////////////////////
// Pedal Detection
///////////////////////////////////////////////////

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

///////////////////////////////////////////////////
// Input Buttons
///////////////////////////////////////////////////

// decodes hat switch values into only 1st 4 buttons (button0-up, button1-right, button2-down, button3-left)
uint8_t decodeHatSwitch(uint8_t hatBits) {
  switch (hatBits) {
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

static const uint8_t matrixColPins[] = {
  BUTTON_MATRIX_COL0_PIN,
  BUTTON_MATRIX_COL1_PIN,
  BUTTON_MATRIX_COL2_PIN,
  BUTTON_MATRIX_COL3_PIN,
};

static const uint8_t matrixRowPins[] = {
  BUTTON_MATRIX_ROW0_PIN,
  BUTTON_MATRIX_ROW1_PIN,
  BUTTON_MATRIX_ROW2_PIN,
  BUTTON_MATRIX_ROW3_PIN,
};

#ifdef __AVR__
#define READ_MATRIX_COL_(col) (!bitRead(digitalReadFast(matrixColPins[col]), BMCOL##col##_PORTBIT))
#else
#define READ_MATRIX_COL(col) (!digitalRead(matrixColPins[col]))
#endif

static void setMatrixRow(uint8_t row, uint8_t value) {
  digitalWriteFast(matrixRowPins[row], value);
}

// buttons 0-3 of are columns j
// buttons 4-7 of are rows i
// Matrix element is Bij
//     D4  A4  A5  D12
// D6 |b11 b12 b13 b14|
// D7 |b21 b22 b23 b24|
// D8 |b31 b32 b33 b34|
// D5 |b41 b42 b43 b44|
static uint16_t readInputButtonsRawInternal() {
  uint16_t buttonsRaw = 0;
  for (uint8_t row = 0; row < 4; row++) { // rows (along X), we set each row low, one at a time
    setMatrixRow(row, LOW);
    delayMicroseconds(5); // required to avoid the detection of false button presses
    for (uint8_t col = 0; col < 4; col++) { // columns (along Y), read each button from that row by scanning over columns      
      bool buttonPressed = READ_MATRIX_COL(col);
      bitWrite(buttonsRaw, row * 4 + col, buttonPressed);
    }
    setMatrixRow(row, HIGH);
  }
  return buttonsRaw;
}

#ifdef __AVR__
uint16_t readInputButtonsRaw() {
  return readInputButtonsRawInternal();
}
#else
volatile uint16_t lastInputButtonsRaw;

uint16_t readInputButtonsRaw() {
  return lastInputButtonsRaw;
}

static void readInputButtonsTask(void* pvParameters) {
  //Serial.printf("Task0 running on core %d\n", xPortGetCoreID());

  while (true) {
    lastInputButtonsRaw = readInputButtonsRawInternal();
    delay(1);
  }
}
#endif

static void initButtonMatrix() { // if not using shift register, allocate some free pins for buttons
  for (uint8_t col = 0; col < 4; ++col) {
    pinMode(matrixColPins[col], INPUT_PULLUP);
  }
 
  for (uint8_t row = 0; row < 4; ++row) {
    pinModeFast(matrixRowPins[row], OUTPUT);
    setMatrixRow(row, HIGH);
  }

#ifndef __AVR__
  // put input read on second core as otherwise the sleep delay to stabilize the readings would increase the latency 
  TaskHandle_t readInputButtonsTaskHandle;
  xTaskCreatePinnedToCore(readInputButtonsTask, "inputTask0", 10000, NULL, 1, &readInputButtonsTaskHandle, 0);
#endif
}
