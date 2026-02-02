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
#include "ConfigHID.h"
#include "HID.h"
#include "common.h"
#include "debug.h"
#include <Wire.h>

//--------------------------------------- Globals --------------------------------------------------------

fwOpt fwOptions; // struct that holds all firmware options
s16a accel, clutch, hbrake; // changed from s16
s32a brake; // we need 32bit due to 24 bits on load cell ADC, changed from s32
s32v turn; // struct containing scaled x and y-axis for usb send report (for one optical or two magnetic encoders)
s32v axis; // struct containing x and y-axis position input for calculating xy ffb
s32v ffbs; // instance of struct holding 2 axis FFB data
u32 button = 0;

cFFB gFFB;
BRFFB brWheelFFB;

u32 last_ConfigSerial = 0;
u32 last_refresh = 0;
u32 now_micros = micros();
u32 timeDiffConfigSerial = now_micros;

void SendInputReport(uint16_t x, uint16_t y, uint16_t z, uint16_t rx, uint16_t ry, uint32_t buttons);

//--------------------------------------------------------------------------------------------------------
//-------------------------------------------- SETUP -----------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void setup() {
  CONFIG_SERIAL.begin(115200);

  accel.val = 0;
  brake.val = 0;
  clutch.val = 0;
  axis.x = 0;
  turn.x = 0;

  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);

  SetEEPROMConfig(); // check firmware version from EEPROM (if any) and load defaults if required
  LoadEEPROMConfig(); // read firmware setings from EEPROM and update current firmware settings
  ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG));
  ROTATION_MID = ROTATION_MAX >> 1;

#ifdef USE_CONFIGHID // used only through HID configuration interface
  update(&fwOptions); // added - update firmware options based on Config.h predefines
#endif // end of config hid
  InitInputs();
  FfbSetDriver(0);

  InitPWM(); // initialize PWM (or DAC) settings
  ffbs.x = 0; // init xFFB at zero
  SetPWM(&ffbs); // zero PWM at startup
  
  last_refresh = micros();
}

//-- Averaging

#define AVG_OLD_WEIGHT 15
#define AVG_TOTAL_WEIGHT (AVG_OLD_WEIGHT + 1)

// As analog measurements on the AVR are pretty slow we only get 2 samples at best per axis with AVG_INPUTS. So do not use this.
// Instead use a weighted average to reduce the jitter.
template<typename T> // s16a / s32a
int32_t averageAxis(int32_t value, T& average) {
  average = ((int32_t)average * AVG_OLD_WEIGHT + value) / AVG_TOTAL_WEIGHT;
  if (value % AVG_TOTAL_WEIGHT > AVG_TOTAL_WEIGHT / 2) {
    average++; // rounding
  }
  return average;
}

#define AXIS_X_NUM_SAMPLES 4

//--------------------------------------------------------------------------------------------------------
//------------------------------------ Main firmware loop ------------------------------------------------
//--------------------------------------------------------------------------------------------------------

#define PEDALS_CONNECTED_THRESHOLD (ANALOG_MAX * 3 / 4)

static bool checkPedalsConnected(int axisY, int axisZ) {
  static uint32_t yzHighTimeMs = 0;
  static bool pedalsConnected = true;
  static bool startUp = true;

  // Pedal inputs are pulled high (equal to pedals fully pushed) if no pedals are connected
  if (pedalsConnected && (axisY == ANALOG_MAX && axisZ == ANALOG_MAX)) {
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

void loop() {
  now_micros = micros(); // we are polling the loop (FFB and USB reports are sent periodicaly)
  {
    timeDiffConfigSerial = now_micros - last_ConfigSerial; // timer for serial interface

    if ((now_micros - last_refresh) >= CONTROL_PERIOD) {
      last_refresh = now_micros;  // timer for FFB and USB reports

      // do not use running average for ffb input. Use more expensive oversampling instead
      int32_t axisXSum = 0;
      for (int i = 0; i < AXIS_X_NUM_SAMPLES; ++i) {
        axisXSum += analogRead(XAXIS_PIN);
      }
      int32_t axisXRawVal = (axisXSum << 6) / AXIS_X_NUM_SAMPLES; // max. 22 bits used

      // no running average for ffb input
      axis.x = map(axisXRawVal, 0, X_AXIS_LOG_MAX, -ROTATION_MID - 1, ROTATION_MID); // xFFB on X-axis

      // apply running average to turn.x for smoother usb report
      turn.x = axisXRawVal; //averageAxis(axisXRawVal, turn.avg);
      turn.x = map(turn.x, 0, X_AXIS_LOG_MAX, -ROTATION_MID - 1, ROTATION_MID);

      ffbs = gFFB.CalcTorqueCommands(&axis); // passing pointer struct with x and y-axis, in encoder raw units -inf,0,inf
      
      turn.x *= float(X_AXIS_LOG_MAX) / float(ROTATION_MAX); // conversion to physical HID units
      turn.x = constrain(turn.x, -MID_REPORT_X - 1, MID_REPORT_X); // -32768,0,32767 constrained to signed 16bit range

      SetPWM(&ffbs); // FFB signal is generated as digital PWM or analog DAC output (ffbs is a struct containing 2-axis FFB, here we pass it as pointer for calculating PWM or DAC signals)
      // USB Report
      {
        int axisZ = analogRead(ACCEL_PIN);
        int axisY = analogRead(BRAKE_PIN);

        bool pedalsConnected = checkPedalsConnected(axisY, axisZ);
        if (pedalsConnected) { // pedals attached, use levers as additional axes
          accel.val = averageAxis(axisZ, accel.avg); // Z axis
          brake.val = averageAxis(axisY, brake.avg); // Y axis
          clutch.val = averageAxis(analogRead(CLUTCH_PIN), clutch.avg); // RX axis
          hbrake.val = averageAxis(analogRead(HBRAKE_PIN), hbrake.avg); // RY axis
        } else { // no pedals attached, use levers as accel and brake
          accel.val = averageAxis(analogRead(HBRAKE_PIN), accel.avg); // Z axis
          brake.val = averageAxis(analogRead(CLUTCH_PIN), brake.avg); // Y axis
          clutch.val = 0; // RX axis
          hbrake.val = 0; // RY axis
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

        button = readInputButtons(); // read all buttons including matrix and hat switch

        SendInputReport(turn.x + MID_REPORT_X + 1, brake.val, accel.val, clutch.val, hbrake.val, button); // milos

#ifdef USE_CONFIGCDC
        if (timeDiffConfigSerial >= CONFIG_SERIAL_PERIOD) {
          configCDC(); // configure firmware with virtual serial port
          last_ConfigSerial = now_micros;
        }
#endif // end of use config cdc
      }
      UpdateDataLed();
    }
  }
}


// 16+16+12+12+12 bits axis + 10 buttons
void SendInputReport(uint16_t x, uint16_t y, uint16_t z, uint16_t rx, uint16_t ry, uint32_t buttons)
{
  // total of 12 bytes, 2B for x, 2B for y, 3B for z and rx, 4B for ry and buttons
  uint8_t j[12];
  j[0] = x;
  j[1] = x >> 8;
  j[2] = y;
  j[3] = y >> 8;
  j[4] = z;
  j[5] = ((z >> 8) & 0xf) | ((rx & 0xf) << 4);
  j[6] = rx >> 4;
  j[7] = ry;
  j[8] = ((ry >> 8) & 0xf) | ((buttons & 0xf) << 4);
  j[9] = ((buttons >> 4) & 0x3f) | ((buttons >> 6) & 0xc0);
  j[10] = buttons >> 14;

  HID().SendReport(4, j, 11);
}

