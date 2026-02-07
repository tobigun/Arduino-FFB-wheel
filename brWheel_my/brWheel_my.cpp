/* Arduino Leonardo Force Feedback Wheel firmware

  Copyright 2015  Etienne Saint-Paul  (esaintpaul [at] gameseed [dot] fr)
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
#include "HID.h"
#include "WHID.h"
#include "hidDescriptor.h"
#include "common.h"
#include "debug.h"
#include "packed.h"
#include <Wire.h>

//--------------------------------------- Globals --------------------------------------------------------

#define SWAP_BITS(bits) ((((bits) << 1) & 0x2) | (((bits) >> 1) & 0x1))

fwOpt fwOptions; // struct that holds all firmware options
s16a accel, clutch, hbrake; // changed from s16
s32a brake; // we need 32bit due to 24 bits on load cell ADC, changed from s32

cFFB gFFB;

u32 last_ConfigSerial = 0;
u32 last_refresh = 0;
u32 now_micros = micros();
u32 timeDiffConfigSerial = now_micros;

struct ATTR_PACKED InputReport {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t rx;
  int16_t ry;
  uint8_t hat;
  uint16_t buttons : NB_BUTTONS;
  uint8_t padding : 8 - (NB_BUTTONS % 8);
};

static uint16_t rearrangeButtons(uint16_t buttons) {
  uint8_t gearBtns = buttons & 0b11;
  uint8_t dpadBtns = (buttons >> 2) & 0b1111;
  uint8_t sideBtns = SWAP_BITS((buttons >> 8) & 0b11);
  uint16_t frontButtons = SWAP_BITS((buttons >> 10) & 0b11);
  return dpadBtns | (frontButtons << 4) | (sideBtns << 6) | (gearBtns << 8);
}

static void sendInputReport(int16_t x, int16_t y, int16_t z, int16_t rx, int16_t ry, uint8_t hat, uint16_t buttons) {
  InputReport report = {
    x: x,
    y: y,
    z: z,
    rx: rx,
    ry: ry,
    hat: hat,
    buttons: rearrangeButtons(buttons)
  };
  HID().SendReport(INPUT_REPORT_ID, (uint8_t*)&report, sizeof(report));
}

//--------------------------------------------------------------------------------------------------------
//-------------------------------------------- SETUP -----------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void setup() {
  CONFIG_SERIAL.begin(115200);

  accel.val = 0;
  brake.val = 0;
  clutch.val = 0;

  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);

  SetEEPROMConfig(); // check firmware version from EEPROM (if any) and load defaults if required
  LoadEEPROMConfig(); // read firmware setings from EEPROM and update current firmware settings

  InitInputs();
  FfbSetDriver(0);

  InitPWM(); // initialize PWM (or DAC) settings

  s32v ffbs; // init xFFB at zero
  ffbs.x = 0; 
  SetPWM(&ffbs); // zero PWM at startup
  
  last_refresh = micros();
}

//-- Averaging

#define AVG_AXIS_NUM_BITS 5
#define AVG_AXIS_NUM_MAX_SAMPLES (1 << AVG_AXIS_NUM_BITS)
#define AVG_AXIS_OUT_MAX_PREC_BITS (ANALOG_BITS + AVG_AXIS_NUM_BITS) // precision in bits after oversampling
#define AVG_AXIS_OUT_MAX_PREC_VALUE ((1L << AVG_AXIS_OUT_MAX_PREC_BITS) - 1) // max. value with highest precision after oversampling

enum {
  AVG_AXIS_ID_X = 0,
  AVG_AXIS_ID_Y,
  AVG_AXIS_ID_Z,
  AVG_AXIS_ID_RX,
  AVG_AXIS_ID_RY,
  AVG_AXIS_ID_COUNT
};

// first index contains the newest sample per axis
static uint16_t axisSamples[AVG_AXIS_ID_COUNT][AVG_AXIS_NUM_MAX_SAMPLES];

static void readAxisSamples(size_t axisIndex, uint8_t sampleCount, uint8_t pin) {
  uint16_t* samples = axisSamples[axisIndex];
  memmove(&samples[sampleCount], &samples[0], (AVG_AXIS_NUM_MAX_SAMPLES - sampleCount) * sizeof(int16_t));
  for (int8_t i = sampleCount - 1; i >= 0; --i) {
    samples[i] = analogRead(pin);
  }
}

static int16_t getAxisValue(size_t axisIndex, uint8_t outputBits, uint8_t sampleCount = AVG_AXIS_NUM_MAX_SAMPLES) {
  uint32_t axisXSum = 0;
  for (int i = 0; i < sampleCount; ++i) {
    axisXSum += axisSamples[axisIndex][i];
  }
  return (axisXSum << (outputBits - ANALOG_BITS)) / sampleCount;
}

//--------------------------------------------------------------------------------------------------------
//------------------------------------ Main firmware loop ------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void loop() {
  now_micros = micros(); // we are polling the loop (FFB and USB reports are sent periodicaly)
  {
    timeDiffConfigSerial = now_micros - last_ConfigSerial; // timer for serial interface

    readAxisSamples(AVG_AXIS_ID_X, 2, X_AXIS_PIN);
    readAxisSamples(AVG_AXIS_ID_Y, 1, Y_AXIS_PIN);
    readAxisSamples(AVG_AXIS_ID_Z, 1, Z_AXIS_PIN);
    readAxisSamples(AVG_AXIS_ID_RX, 1, RX_AXIS_PIN);
    readAxisSamples(AVG_AXIS_ID_RY, 1, RY_AXIS_PIN);

    if ((now_micros - last_refresh) >= CONTROL_PERIOD) {
      last_refresh = now_micros;  // timer for FFB and USB reports

      int16_t ffbAxisValueRaw = getAxisValue(AVG_AXIS_ID_X, AVG_AXIS_OUT_MAX_PREC_BITS, 4); // only use the newest samples for averaging of FFB axis to reduce latency
      s32v ffbAxisValue; // struct containing x and y-axis position input for calculating ffb
      ffbAxisValue.x = map(ffbAxisValueRaw, 0, AVG_AXIS_OUT_MAX_PREC_VALUE, -FFB_ROTATION_MID - 1, FFB_ROTATION_MID); // xFFB on X-axis
      s32v ffbs = gFFB.CalcTorqueCommands(&ffbAxisValue); // passing pointer struct with x and y-axis, in encoder raw units -inf,0,inf
      SetPWM(&ffbs); // FFB signal is generated as digital PWM or analog DAC output (ffbs is a struct containing 2-axis FFB, here we pass it as pointer for calculating PWM or DAC signals)

      int16_t turnXRaw = getAxisValue(AVG_AXIS_ID_X, X_AXIS_NB_BITS); // get averaged X axis for all samples for smoother USB report
      int16_t deadZoneX = (X_AXIS_LOG_MAX - (X_AXIS_LOG_MAX * ROTATION_DEG / FFB_ROTATION_DEG)) / 2;
      int32_t turnX = map(turnXRaw, 0, X_AXIS_LOG_MAX, deadZoneX, X_AXIS_LOG_MAX - deadZoneX);
      turnX = constrain(turnX, 0, X_AXIS_LOG_MAX);

      // USB Report
      {
        int16_t yAxisValue = getAxisValue(AVG_AXIS_ID_Y, Y_AXIS_NB_BITS);
        int16_t zAxisValue = getAxisValue(AVG_AXIS_ID_Z, Z_AXIS_NB_BITS);
        bool pedalsConnected = checkPedalsConnected(yAxisValue, zAxisValue);
        if (pedalsConnected) { // pedals attached, use levers as additional axes
          accel.val = zAxisValue;
          brake.val = yAxisValue;
          clutch.val = getAxisValue(AVG_AXIS_ID_RX, RX_AXIS_NB_BITS);
          hbrake.val = getAxisValue(AVG_AXIS_ID_RY, RY_AXIS_NB_BITS);
        } else { // no pedals attached, use levers as accel and brake
          accel.val = getAxisValue(AVG_AXIS_ID_RY, Z_AXIS_NB_BITS);
          brake.val = getAxisValue(AVG_AXIS_ID_RX, Y_AXIS_NB_BITS);
          clutch.val = 0; // RX axis
          hbrake.val = 0; // RY axis
        }

        if (useCombinedAxes) {
          if (accel.val > brake.val) {
            brake.val = (Y_AXIS_LOG_MAX + accel.val) / 2;
          } else {
            brake.val = (Y_AXIS_LOG_MAX - brake.val) / 2;;
          }
          accel.val = 0;
        }

        // rescale all analog axis according to a new manual calibration
        brake.val = map(brake.val, brake.min, brake.max, 0, Y_AXIS_LOG_MAX);
        brake.val = constrain(brake.val, 0, Y_AXIS_LOG_MAX);

        accel.val = map(accel.val, accel.min, accel.max, 0, Z_AXIS_LOG_MAX);  // with manual calibration and dead zone
        accel.val = constrain(accel.val, 0, Z_AXIS_LOG_MAX);

        clutch.val = map(clutch.val, clutch.min, clutch.max, 0, RX_AXIS_LOG_MAX);
        clutch.val = constrain(clutch.val, 0, RX_AXIS_LOG_MAX);

        hbrake.val = map(hbrake.val, hbrake.min, hbrake.max, 0, RY_AXIS_LOG_MAX);
        hbrake.val = constrain(hbrake.val, 0, RY_AXIS_LOG_MAX);

        uint16_t buttons;
        uint8_t hat;
        readInputButtons(buttons, hat); // read all buttons including matrix and hat switch

        sendInputReport(turnX, brake.val, accel.val, clutch.val, hbrake.val, hat, buttons);

        if (timeDiffConfigSerial >= CONFIG_SERIAL_PERIOD) {
          configCDC(); // configure firmware with virtual serial port
          last_ConfigSerial = now_micros;
        }
      }
      UpdateDataLed();
    }
  }
}
