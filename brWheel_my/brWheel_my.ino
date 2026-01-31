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

#include "Config.h"
#include "ConfigHID.h"
#include "HID.h"
#include "WHID.h"
#include "debug.h"
#include "ffb_pro.h"
//#include "ffb.h" // milos, commented out
#include "USBDesc.h"
#ifdef USE_QUADRATURE_ENCODER
#include "QuadEncoder.h"
#endif
#include <Wire.h>
#ifdef USE_EEPROM
#include <EEPROM.h> // milos, uncommented
#endif

//extern u8 valueglobal; // milos, commented out

//--------------------------------------- Globals --------------------------------------------------------

//b8 fault; // milos, commented out
fwOpt fwOptions; // milos, added - struct that holds all firmware options
s16a accel, clutch, hbrake; // milos, changed from s16
s32a brake; // milos, we need 32bit due to 24 bits on load cell ADC, changed from s32
//s32 turn.x; // milos, x-axis (for optical or magnetic encoder)
//s32 turn.y; // milos, y-axis (for 2nd magnetic encoder)
s32v turn; // milos, struct containing scaled x and y-axis for usb send report (for one optical or two magnetic encoders)
s32v axis; // milos, struct containing x and y-axis position input for calculating xy ffb
s32v ffbs; // milos, instance of struct holding 2 axis FFB data
u32 button = 0; // milos, added

//milos, added

cFFB gFFB;
BRFFB brWheelFFB;

//milos, added
#ifdef AVG_INPUTS
extern s32 analog_inputs[];
u8 asc = 0; // milos, added - sample counter for averaging of analog inputs
#endif

u32 last_ConfigSerial = 0;
u32 last_refresh = 0;
u32 now_micros = micros();
u32 timeDiffConfigSerial = now_micros;

uint16_t dz, bdz; // milos

//----------------------------------------- Options -------------------------------------------------------

#ifdef USE_QUADRATURE_ENCODER
cQuadEncoder myEnc;
#endif

void SendInputReport(uint16_t x, uint16_t y, uint16_t z, uint16_t rx, uint16_t ry, uint32_t buttons);

//--------------------------------------------------------------------------------------------------------
//-------------------------------------------- SETUP -----------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void setup() {
  CONFIG_SERIAL.begin(115200);
  //fault = false; // milos, commented out
  accel.val = 0;
  brake.val = 0;
  clutch.val = 0;
  //turn.x = 0;
  axis.x = 0;
  turn.x = 0;

  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);

#ifdef USE_EEPROM
  SetEEPROMConfig(); // milos, check firmware version from EEPROM (if any) and load defaults if required
  LoadEEPROMConfig(); // milos, read firmware setings from EEPROM and update current firmware settings
#else //milos, otherwise you can set it here (firmware settings will be set to these defaults on each arduino power up or reset)
  // CPR (counts per revolution) depends on encoder's pulse per revolution - PPR, and gear ratio - GR you have coupling it with wheel shaft
  // mine has PPR=600 and I use 10:1 GR (for 1 wheel rotation I have 10 rotaions of encoder's shaft)
  // each pulse has 4 states (counts), so final equation for CPR is:
  // CPR = 600*10*4 = 24000
  ROTATION_DEG = 1080; // milos, here we set wheel's initial deg of rotation (can be changed latter through serial interface, or my program wheel_control.pde written in processing)
  CPR = 2400; // milos, here we set our optical encoder's counts per (wheel shaft) revolution (this is a constant value - your hardware/mechanical parameter)
  configGeneralGain = 100;
  configDamperGain = 50;
  configFrictionGain = 50;
  configConstantGain = 100;
  configPeriodicGain = 100;
  configSpringGain = 50;
  configInertiaGain = 50;
  configCenterGain = 70;
  configStopGain = 100;
  effstate = 0b00000001; // milos, only desktop spring effect is enabled
  LC_scaling = 128; // milos, FFB left/right balance (128=center)
  //pwmstate = 0b00001001; // milos, PWM out enabled, phase correct, pwm+-, 16kHz, TOP 500
  pwmstate = 0b00001100; // milos, PWM out enabled, fast pwm, pwm+-, 7.8kHz, TOP 11bit (2047)
  MM_MIN_MOTOR_TORQUE = 0;
  minTorquePP = 0.0;
#ifdef USE_AUTOCALIB //milos, reset limits for autocalibration of pedals
  accel.min = Z_AXIS_LOG_MAX;
  accel.max = 0;
  brake.min = Y_AXIS_LOG_MAX;
  brake.max = 0;
  clutch.min = RX_AXIS_LOG_MAX;
  clutch.max = 0;
  hbrake.min = RY_AXIS_LOG_MAX;
  hbrake.max = 0;
#else
  accel.min = 0;
  accel.max = maxCal;
  brake.min = 0;
  brake.max = maxCal;
  clutch.min = 0;
  clutch.max = maxCal;
  hbrake.min = 0;
  hbrake.max = maxCal;
#endif // end of autocalib
#endif // end of eeprom
  ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos
  ROTATION_MID = ROTATION_MAX >> 1; // milos

#ifdef USE_QUADRATURE_ENCODER
  //myEnc.Init(ROTATION_MID + brWheelFFB.offset, true); //ROTATION_MID + gCalibrator.mOffset); // milos, pullups enabled
  myEnc.Init(ROTATION_MID, true); // milos, pullups enabled, do not apply any encoder offset at this point
#endif // end of quad enc

#ifdef USE_CONFIGHID // milos, used only through HID configuration interface
  update(&fwOptions); // milos, added - update firmware options based on Config.h predefines
#endif // end of config hid
  InitInputs();
  FfbSetDriver(0);

  InitPWM(); // milos, initialize PWM (or DAC) settings
  ffbs.x = 0; // milos, init xFFB at zero
  SetPWM(&ffbs); // milos, zero PWM at startup

#ifdef USE_QUADRATURE_ENCODER
  (CALIBRATE_AT_INIT ? brWheelFFB.calibrate() : myEnc.Write(ROTATION_MID)); // milos, allways set encoder at 0deg (ROTATION_MID) at startup and calibrate if enabled
#endif // end of quad enc

#ifdef USE_AUTOCALIB
#ifdef AVG_INPUTS
  dz = 8; // milos, set the accel, brake and clutch pedal dead zones that will be taken from min and max axis val (default is 8 out of 4095)
#else //
  dz = 2; // milos, set 2 out of 1023
#endif // end of avg inputs
#else // when no autocal
  dz = 0; // milos, we can set min/max cal limits manualy so we do not need this
#endif // end of autocal
  bdz = 2047; // milos, set the brake pedal dead zone taken from min axis val, when using load cell (default is 2047 out of 65535)
  
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

void loop() {
#ifdef AVG_INPUTS //milos, added option see config.h
  if (asc < avgSamples) {
    ReadAnalogInputs(); // milos, get readings for averaging (only do it until we get all samples)
    asc++; // milos
  }
#endif // end of avg inp

  now_micros = micros(); // milos, we are polling the loop (FFB and USB reports are sent periodicaly)
  {
    timeDiffConfigSerial = now_micros - last_ConfigSerial; // milos, timer for serial interface

    if ((now_micros - last_refresh) >= CONTROL_PERIOD) {
#ifdef AVG_INPUTS //milos
      asc = 0; // milos, reset counter for averaging
#endif // end of avg_inputs
      last_refresh = now_micros;  // milos, timer for FFB and USB reports
      //SYNC_LED_HIGH(); // milos

#ifdef USE_QUADRATURE_ENCODER
      if (zIndexFound) {
        turn.x = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; // milos, only apply z-index offset if z-index pulse is found
      } else {
        turn.x = myEnc.Read() - ROTATION_MID;
      }
      axis.x = turn.x; // milos, xFFB on X-axis (optical or magnetic encoder)
#else // milos, if no optical enc and no as5600, use pot for X-axis
      // do not use running average for ffb input. Use more expensive oversampling instead
      int32_t axisXSum = 0;
      for (int i = 0; i < AXIS_X_NUM_SAMPLES; ++i) {
        axisXSum += analogRead(XAXIS_PIN);
      }
      int32_t axisXRawVal = (axisXSum << 6) / AXIS_X_NUM_SAMPLES; // max. 22 bits used
      
      // no running average for ffb input
      axis.x = map(axisXRawVal, 0, X_AXIS_PHYS_MAX, -ROTATION_MID - 1, ROTATION_MID); // xFFB on X-axis

      // apply running average to turn.x for smoother usb report
      turn.x = averageAxis(axisXRawVal, turn.avg);
      turn.x = map(turn.x, 0, X_AXIS_PHYS_MAX, -ROTATION_MID - 1, ROTATION_MID);
#endif // end of quad enc
      ffbs = gFFB.CalcTorqueCommands(&axis); // milos, passing pointer struct with x and y-axis, in encoder raw units -inf,0,inf
      
      turn.x *= float(X_AXIS_PHYS_MAX) / float(ROTATION_MAX); // milos, conversion to physical HID units
      turn.x = constrain(turn.x, -MID_REPORT_X - 1, MID_REPORT_X); // milos, -32768,0,32767 constrained to signed 16bit range

      SetPWM(&ffbs); // milos, FFB signal is generated as digital PWM or analog DAC output (ffbs is a struct containing 2-axis FFB, here we pass it as pointer for calculating PWM or DAC signals)
      //SYNC_LED_LOW(); //milos
      // USB Report
      {
        //last_send = now_micros;
#ifdef AVG_INPUTS //milos, added option see config.h
        AverageAnalogInputs();				// Average readings
#endif

#ifdef AVG_INPUTS // milos, we do not average h-shifter axis, only pedal axis
        accel.val = analog_inputs[ACCEL_INPUT];
        clutch.val = analog_inputs[CLUTCH_INPUT];
        hbrake.val = analog_inputs[HBRAKE_INPUT];
        brake.val = analog_inputs[BRAKE_INPUT];
#else // if no avg
        accel.val = averageAxis(analogRead(ACCEL_PIN), accel.avg); // Z axis
        brake.val = averageAxis(analogRead(BRAKE_PIN), brake.avg); // Y axis
        clutch.val = averageAxis(analogRead(CLUTCH_PIN), clutch.avg); // RX axis
        hbrake.val = averageAxis(analogRead(HBRAKE_PIN), hbrake.avg); // RY axis
#endif // end of avg

#ifdef  USE_AUTOCALIB // milos, update limits for pedal autocalibration
        if (accel.val < accel.min) accel.min = accel.val;
        if (accel.val > accel.max) accel.max = accel.val;
        if (brake.val < brake.min) brake.min = brake.val;
        if (brake.val > brake.max) brake.max = brake.val;
        if (clutch.val < clutch.min) clutch.min = clutch.val;
        if (clutch.val > clutch.max) clutch.max = clutch.val;
        if (hbrake.val < hbrake.min) hbrake.min = hbrake.val;
        if (hbrake.val > hbrake.max) hbrake.max = hbrake.val;
#endif // end of autocalib
#ifdef USE_AVGINPUTS
        // milos, update calibration limits for increased axis resolution due to averaging (depends on num of samples)
        accel.min *= avgSamples;
        accel.max *= avgSamples;
        brake.min *= avgSamples;
        brake.max *= avgSamples;
        clutch.min *= avgSamples;
        clutch.max *= avgSamples;
        hbrake.min *= avgSamples;
        hbrake.max *= avgSamples;
#endif // end of avg inputs

        // milos, rescale all analog axis according to a new manual calibration and add small deadzones
        accel.val = map(accel.val, accel.min + dz, accel.max - dz, 0, Z_AXIS_PHYS_MAX);  // milos, with manual calibration and dead zone
        accel.val = constrain(accel.val, 0, Z_AXIS_PHYS_MAX);

        clutch.val = map(clutch.val, clutch.min + dz, clutch.max - dz, 0, RX_AXIS_PHYS_MAX);
        clutch.val = constrain(clutch.val, 0, RX_AXIS_PHYS_MAX);

        hbrake.val = map(hbrake.val, hbrake.min + dz, hbrake.max - dz, 0, RY_AXIS_PHYS_MAX);
        hbrake.val = constrain(hbrake.val, 0, RY_AXIS_PHYS_MAX);

        brake.val = map(brake.val, brake.min + dz, brake.max - dz, 0, Y_AXIS_PHYS_MAX);
        brake.val = constrain(brake.val, 0, Y_AXIS_PHYS_MAX);

        button = readInputButtons(); // milos, read all buttons including matrix and hat switch

#ifdef USE_QUADRATURE_ENCODER // milos, if we use quad enc
        SendInputReport(turn.x + MID_REPORT_X + 1, brake.val, accel.val, clutch.val, hbrake.val, button); // milos, X, Y, Z, RX, RY, hat+button; (0-65535) X-axis range, center at 32768
#else // milos, if no quad enc
        SendInputReport(turn.x + MID_REPORT_X + 1, brake.val, accel.val, clutch.val, hbrake.val, button); // milos
#endif // end of quad enc

#ifdef AVG_INPUTS //milos, added option see config.h
        ClearAnalogInputs();
#endif // end of avg inp
#ifdef USE_CONFIGCDC
        if (timeDiffConfigSerial >= CONFIG_SERIAL_PERIOD) {
          configCDC(); // milos, configure firmware with virtual serial port
          last_ConfigSerial = now_micros;
        }
#endif // end of use config cdc
      }
      //SYNC_LED_LOW(); //milos
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

