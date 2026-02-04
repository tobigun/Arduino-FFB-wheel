

/* Copyright (c) 2011, Peter Barrett
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#pragma once

#include "HID.h"

#define USAGE_PAGE_GENERIC_DESKTOP 0x01
#define USAGE_PAGE_SIMULATION_CONTROLS 0x02

#define USAGE_GENERIC_X 0x30
#define USAGE_GENERIC_Y 0x31
#define USAGE_GENERIC_Z 0x32

#define USAGE_SIM_STEERING 0xC8
#define USAGE_SIM_BRAKE 0xC5
#define USAGE_SIM_ACCELERATOR 0xC4

#define USE_GENERIC_AXIS_DEFAULT_PROFILE
#ifdef USE_GENERIC_AXIS_DEFAULT_PROFILE
#define DEFAULT_USAGE_PAGE USAGE_PAGE_GENERIC_DESKTOP
#define DEFAULT_X_USAGE  USAGE_GENERIC_X
#define DEFAULT_Y_USAGE  USAGE_GENERIC_Y
#define DEFAULT_Z_USAGE  USAGE_GENERIC_Z
#else
#define DEFAULT_USAGE_PAGE USAGE_PAGE_SIMULATION_CONTROLS
#define DEFAULT_X_USAGE  USAGE_SIM_STEERING
#define DEFAULT_Y_USAGE  USAGE_SIM_BRAKE
#define DEFAULT_Z_USAGE  USAGE_SIM_ACCELERATOR
#endif

#define MAIN_AXES_USAGE_PAGE_OFFSET 11
#define AXES_X_USAGE_OFFSET (MAIN_AXES_USAGE_PAGE_OFFSET + 6)
#define AXES_Y_USAGE_OFFSET (AXES_X_USAGE_OFFSET + 14)
#define AXES_Z_USAGE_OFFSET (AXES_Y_USAGE_OFFSET + 4)
#define FFB_AXES_USAGE_PAGE_OFFSET (sizeof(_dynamicHidReportDescriptor) - 12 - NB_FF_AXIS * 2)
#define FFB_AXES_USAGE_OFFSET (FFB_AXES_USAGE_PAGE_OFFSET + 2)

#define INPUT_REPORT_ID 4

const uint8_t _dynamicHidReportDescriptor[] PROGMEM =
{
  0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
  0x09, 0x04,	// USAGE (Joystick)
  0xA1, 0x01,	// COLLECTION (Application)
    0x85, INPUT_REPORT_ID,	// REPORT_ID (04)
    0xA1, 0x00, // COLLECTION (Physical)
      #pragma region
      0x05, DEFAULT_USAGE_PAGE, // USAGE_PAGE (Generic Desktop: 0x01 / Simulation Controls: 0x02) [AXES_USAGE_PAGE_OFFSET]
      0x75, 0x10,            // REPORT_SIZE (16)
      0x95, 0x01,            // REPORT_COUNT (1)

      0x09, DEFAULT_X_USAGE, // USAGE (X / Steering) [AXES_X_USAGE_OFFSET]
      0x15, 0,               // LOGICAL_MINIMUM (X_AXIS_LOG_MIN = 0)
      0x26, 0xFF, 0x7F,      // LOGICAL_MAXIMUM (X_AXIS_LOG_MAX = 32767)
      0x81, 0x02,            // INPUT (Data,Var,Abs)

      0x15, 0,               // LOGICAL_MINIMUM (AXIS_LOG_MIN = 0)
      0x26, 0xFF, 0x03,      // LOGICAL_MAXIMUM (AXIS_LOG_MAX = 1023)
      0x09, DEFAULT_Y_USAGE, // USAGE (Y / Brake) [AXES_Y_USAGE_OFFSET]
      0x81, 0x02,            // INPUT (Data,Var,Abs)
      0x09, DEFAULT_Z_USAGE, // USAGE (Z / Accelerator) [AXES_Z_USAGE_OFFSET]
      0x81, 0x02,            // INPUT (Data,Var,Abs)
      0x05, 0x01,	           // USAGE_PAGE (Generic Desktop)
      0x09, 0x33,            // USAGE (rx)
      0x81, 0x02,            // INPUT (Data,Var,Abs)
      0x09, 0x34,            // USAGE (ry)
      0x81, 0x02,            // INPUT (Data,Var,Abs)

      0x09, 0x39,                     // USAGE (HAT SWITCH)
      0x15, 0x01,                     // LOGICAL_MINIMUM (1)
      0x25, 0x08,                     // LOGICAL_MAXIMUM (8)
      0x35, 0x00,                     // PHYSICAL_MINIMUM (0)
      0x46, 0x3B, 0x01,               // PHYSICAL_MAXIMUM (315)
      0x65, 0x14,                     // UNIT (Eng Rot:Angular Pos)
      0x55, 0x00,                     // UNIT_EXPONENT (0)
      0x75, 0x08,                     // REPORT_SIZE (8)
      0x95, 0x01,                     // REPORT_COUNT (1)
      0x81, 0x02,                     // Input (Data,Var,Abs)

      0x05, 0x09,                     // USAGE_PAGE (Button)
      0x15, 0x00,                     // LOGICAL_MINIMUM (0)
      0x25, 0x01,                     // LOGICAL_MAXIMUM (1)
      0x55, 0x00,                     // UNIT_EXPONENT (0)
      0x65, 0x00,                     // UNIT (None)
      0x19, 0x01,					            // USAGE_MINIMUM (button 1)
      0x29, NB_BUTTONS,               // USAGE_MAXIMUM (button NB_BUTTONS)
      0x75, 0x01,                     // REPORT_SIZE (1)
      0x95, NB_BUTTONS,               // REPORT_COUNT (number of buttons)
      0x81, 0x02,                     // Input (Data,Var,Abs)
        
      // --- Padding bits ---
      0x05, 0x01,                     // USAGE_PAGE (Generic Desktop)
      0x75, 0x01,                     // REPORT_SIZE (1)
      0x95, 8 - (NB_BUTTONS % 8),     // REPORT_COUNT (padding bits)
      0x81, 0x03,                     // Input (Const,Var,Abs)
      #pragma endregion
    0xc0, // END_COLLECTION

    //FFB part (PID) starts from here
    0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
    0x09, 0x92,	// USAGE (PID State Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x02,	// REPORT_ID (02)
      0x09, 0x9F,	// USAGE (Device Paused)
      0x09, 0xA0,	// USAGE (Actuators Enabled)
      0x09, 0xA4,	// USAGE (Safety Switch)
      0x09, 0xA5,	// USAGE (Actuator Override Switch)
      0x09, 0xA6,	// USAGE (Actuator Power)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x25, 0x01,	// LOGICAL_MINIMUM (01)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
      0x75, 0x01,	// REPORT_SIZE (01)
      0x95, 0x05,	// REPORT_COUNT (05)
      0x81, 0x02,	// INPUT (Data,Var,Abs)
      0x95, 0x03,	// REPORT_COUNT (03)
      0x81, 0x03,	// INPUT (Constant,Var,Abs)
      0x09, 0x94,	// USAGE (Effect Playing)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x25, 0x01,	// LOGICAL_MAXIMUM (01)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
      0x75, 0x01,	// REPORT_SIZE (01)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x81, 0x02,	// INPUT (Data,Var,Abs)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x07,	// REPORT_SIZE (07)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x81, 0x02,	// INPUT (Data,Var,Abs)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x09, 0x21,	// USAGE (Set Effect Output Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x01,	// REPORT_ID (01)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x25,	// USAGE (Effect type)
      0xA1, 0x02,	// COLLECTION (Logical)
        #pragma region
        0x09, 0x26,	// USAGE (ET Constant Force)
        0x09, 0x27,	// USAGE (ET Ramp)
        0x09, 0x30,	// USAGE (ET Square)
        0x09, 0x31,	// USAGE (ET Sine)
        0x09, 0x32,	// USAGE (ET Triangle)
        0x09, 0x33,	// USAGE (ET Sawtooth Up)
        0x09, 0x34,	// USAGE (ET Sawtooth Down)
        0x09, 0x40,	// USAGE (ET Spring)
        0x09, 0x41,	// USAGE (ET Damper)
        0x09, 0x42,	// USAGE (ET Inertia)
        0x09, 0x43,	// USAGE (ET Friction)
        //0x09, 0x28,	// USAGE (ET Custom Force Data) //milos, removed custom force effect block
        //0x25, 0x0C,	// LOGICAL_MAXIMUM (12)
        0x25, 0x0B,  // LOGICAL_MAXIMUM (11) //milos, 1 less effect
        0x15, 0x01,	// LOGICAL_MINIMUM (01)
        0x35, 0x01,	// PHYSICAL_MINIMUM (01)
        //0x45, 0x0C,	// PHYSICAL_MAXIMUM (12)
        0x45, 0x0B,  // PHYSICAL_MAXIMUM (11) //milos, 1 less effect
        0x75, 0x08,	// REPORT_SIZE (08)
        0x95, 0x01,	// REPORT_COUNT (01)
        0x91, 0x00,	// OUTPUT (Data)
        #pragma endregion
      0xC0,	// END COLLECTION ()
      0x09, 0x50,	// USAGE (Duration)
      0x09, 0x54,	// USAGE (Trigger Repeat Interval)
      //0x09, 0x51,	// USAGE (Sample Period) //milos, commented
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x27, 0xFF, 0xFF, 0x00, 0x00,	// LOGICAL_MAXIMUM (65535)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
      0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
      0x55, 0xFD,  // UNIT_EXPONENT (-3)
      0x75, 0x10,	// REPORT_SIZE (16)
      //0x95, 0x03,	// REPORT_COUNT (03)
      0x95, 0x02,  // REPORT_COUNT (02) //milos
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x55, 0x00,	// UNIT_EXPONENT (00)
      0x66, 0x00, 0x00,	// UNIT (None)
      0x09, 0x52,	// USAGE (Gain)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      //0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
      0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      //0x46, 0x10, 0x27,	// PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      //0x75, 0x08,	// REPORT_SIZE (08)
      0x75, 0x10,  // REPORT_SIZE (16) //milos
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x53,	// USAGE (Trigger Button)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x08,	// LOGICAL_MAXIMUM (08)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x08,	// PHYSICAL_MAXIMUM (08)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x55,	// USAGE (Axes Enable)
      0xA1, 0x02,	// COLLECTION (Logical)
        #pragma region
        0x05, DEFAULT_USAGE_PAGE, // USAGE_PAGE (Generic Desktop: 0x01 / Simulation Controls: 0x02) [FFB_AXES_USAGE_PAGE_OFFSET]
        0x09, DEFAULT_X_USAGE, // USAGE (X / Steering) [FFB_AXES_USAGE_OFFSET]
        #if (NB_FF_AXIS > 1)
          0x09, 0x31,	// USAGE (Y)
        #endif
        0x15, 0x00,	// LOGICAL_MINIMUM (00)
        0x25, 0x01,	// LOGICAL_MAXIMUM (01)
        0x75, 0x01,	// REPORT_SIZE (01)
        0x95, NB_FF_AXIS,	// REPORT_COUNT (NB_FF_AXIS)
        0x91, 0x02,	// OUTPUT (Data,Var,Abs)
        #pragma endregion
      0xC0,	// END COLLECTION ()
      #pragma endregion // "Set Effect Output Report" continued in _staticHidReportDescriptor ...
};

const uint8_t _staticHidReportDescriptor[] PROGMEM = {
      #pragma region // "Set Effect Output Report" -- continue
      0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
      0x09, 0x56,	// USAGE (Direction Enable)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x95, 0x07 - NB_FF_AXIS,	// REPORT_COUNT (05 (2 axes) or 06 (1 axes)) seems to be for padding
      0x91, 0x03,	// OUTPUT (Constant,Var,Abs)
      0x09, 0x57,	// USAGE (Direction)
      0xA1, 0x02,	// COLLECTION (Logical)
        #pragma region
        0x0B, 0x01, 0x00, 0x0A, 0x00, // USAGE (Ordinals:Instance 1)
        0x0B, 0x02, 0x00, 0x0A, 0x00, // USAGE (Ordinals:Instance 2)
        0x66, 0x14, 0x00,	// UNIT (Eng Rot:Angular Pos)
        0x55, 0xFE,  // UNIT_EXPONENT (-2) //milos, 16bit
        //0x55, 0x00, // UNIT_EXPONENT (0) //milos, 8bit
        0x15, 0x00,	// LOGICAL_MINIMUM (00)
        0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos, 16bit
        //0x26, 0xFF, 0x00,  // LOGICAL_MAXIMUM (255) //milos, 8bit
        0x35, 0x00,	// PHYSICAL_MINIMUM (0)
        0x47, 0x9F, 0x8C, 0x00, 0x00,	// PHYSICAL_MAXIMUM (35999) //milos, 16bit
        //0x46, 0x67, 0x01,  // PHYSICAL_MAXIMUM (359) //milos, 8bit
        0x75, 0x10,  // REPORT_SIZE (16) //milos, 16bit
        //0x75, 0x08,  // REPORT_SIZE (08) //milos, 8bit
        0x95, 0x01,	// REPORT_COUNT (01)
        0x91, 0x02,	// OUTPUT (Data,Var,Abs)
        0x55, 0x00,	// UNIT_EXPONENT (00)
        0x66, 0x00, 0x00,	// UNIT (None)
        #pragma endregion
      0xC0,	// END COLLECTION ()
      0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
      0x09, 0xA7,	// USAGE (Start Delay) //milos, uncommented
      0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
      0x55, 0xFD,  // UNIT_EXPONENT (-3)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x27, 0xFF, 0xFF, 0x00, 0x00,	// LOGICAL_MAXIMUM (65535)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
      0x75, 0x10,	// REPORT_SIZE (16)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs) //milos, uncommented
      0x66, 0x00, 0x00,	// UNIT (None)
      0x55, 0x00,	// UNIT_EXPONENT (00)
      #pragma endregion
    0xC0,	// END COLLECTION () "Set Effect Output Report"

    0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
    0x09, 0x5A,	// USAGE (Set Envelope Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x02,	// REPORT_ID (02)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x5B,	// USAGE (Attack Level)
      0x09, 0x5D,	// USAGE (Fade Level)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x26, 0xFF, 0x00,  // LOGICAL_MAXIMUM (255)
      //0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767) //milos
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      0x75, 0x08, // REPORT_SIZE (08)
      0x95, 0x02,	// REPORT_COUNT (02)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x5C,	// USAGE (Attack Time)
      0x09, 0x5E,	// USAGE (Fade Time)
      0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
      0x55, 0xFD,  // UNIT_EXPONENT (-3)
      //0x26, 0xFF, 0xFF,	// LOGICAL_MAXIMUM (65535)
      //0x46, 0xFF, 0xFF,	// PHYSICAL_MAXIMUM (65535)
      0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos
      0x46, 0xFF, 0x7F, // PHYSICAL_MAXIMUM (32767) //milos
      0x75, 0x10,	// REPORT_SIZE (16)
      0x95, 0x02, // REPORT_COUNT (02) //milos, added
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x66, 0x00, 0x00,	// UNIT (None)
      0x55, 0x00,	// UNIT_EXPONENT (00)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x09, 0x5F,	// USAGE (Set Condition Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x03,	// REPORT_ID (03)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x23,	// USAGE (Parameter Block Offset)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x25, 0x01,	// LOGICAL_MAXIMUM (01)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
      0x75, 0x04,	// REPORT_SIZE (04)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x58,	// USAGE (Type Specific Block Offset)
      0xA1, 0x02,	// COLLECTION (Logical)
        #pragma region
        0x0B, 0x01, 0x00, 0x0A, 0x00,	// USAGE (Instance 1)
        0x0B, 0x02, 0x00, 0x0A, 0x00,	// USAGE (Instance 2)
        0x75, 0x02,	// REPORT_SIZE (02)
        0x95, 0x02,	// REPORT_COUNT (02)
        0x91, 0x02,	// OUTPUT (Data,Var,Abs)
        #pragma endregion
      0xC0,	// END COLLECTION ()
      0x09, 0x60, // USAGE (CP Offset)
      //0x15, 0x80,	// LOGICAL_MINIMUM (-128)
      0x16, 0x00, 0x80,  // LOGICAL_MINIMUM (-32768) //milos
      //0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
      0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos
      //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000)
      0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      //0x75, 0x08,	// REPORT_SIZE (08)
      0x75, 0x10, // REPORT_SIZE (16) //milos
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x61, // USAGE (Positive Coefficient)
      //   0x09,0x62,  // USAGE (Negative Coefficient)
      //0x36, 0xF0, 0xD8,	// PHYSICAL_MINIMUM (-10000)
      0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      0x95, 0x01,	// REPORT_COUNT (01)	// ???? WAS 2 with "negative coeff"
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      //0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
      0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      0x09, 0x63,	// USAGE (Positive Saturation) //milos, uncommented
      //  0x09, 0x64,	// USAGE (Negative Saturation)
      0x75, 0x10,	// REPORT_SIZE (16) //milos
      0x95, 0x01,	// REPORT_COUNT (01) //milos, uncommented
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x65,	// USAGE (Dead Band ) //milos, uncommented
      0x15, 0x00,  // LOGICAL_MINIMUM (00) //milos
      0x26, 0xFF, 0x00, // LOGICAL_MAXIMUM (255) //milos
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      0x75, 0x08, // REPORT_SIZE (08) //milos
      0x95, 0x01,	// REPORT_COUNT (01) //milos, uncommented
      0x91, 0x02,	// OUTPUT (Data,Var,Abs) //milos, uncommented
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x09, 0x6E,	// USAGE (Set Periodic Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x04,	// REPORT_ID (04)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x70,	// USAGE (Magnitude)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      //0x26, 0xFF, 0x00,   // LOGICAL_MAXIMUM (255)
      0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767) //milos
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F, // PHYSICAL_MAXIMUM (32767) //milos
      //0x75, 0x08,	// REPORT_SIZE (08)
      0x75, 0x10,  // REPORT_SIZE (16) //milos
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x6F,	// USAGE (Offset)
      //0x15, 0x80,	// LOGICAL_MINIMUM (-128)
      0x16, 0x00, 0x80,  // LOGICAL_MINIMUM (-32768) //milos
      //0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
      0x26, 0xFF, 0x7F,   // LOGICAL_MAXIMUM (32737) //milos
      //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000)
      0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x71,	// USAGE (Phase)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
      //0x27, 0xFF, 0xFF, 0x00, 0x00,  // LOGICAL_MAXIMUM (65535) //milos
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      //0x47, 0x9F, 0x8C, 0x00, 0x00,	// PHYSICAL_MAXIMUM (35999) //milos
      0x46, 0x67, 0x01,  // PHYSICAL_MAXIMUM (359) //milos
      0x66, 0x14, 0x00,  // UNIT (Eng Rot:Angular Pos)
      //0x55, 0xFE, // UNIT_EXPONENT (-2)
      0x55, 0x00, // UNIT_EXPONENT (0) //milos
      0x75, 0x08, // REPORT_SIZE (08) //milos
      0x95, 0x01, // REPORT_COUNT (01) //milos
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x72,	// USAGE (Period)
      //0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767)
      0x27, 0xFF, 0xFF, 0x00, 0x00,  // LOGICAL_MAXIMUM (65535) //milos
      //0x46, 0xFF, 0x7F,	// PHYSICAL_MAXIMUM (32767)
      0x47, 0xFF, 0xFF, 0x00, 0x00, // PHYSICAL_MAXIMUM (65535) //milos
      0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
      0x55, 0xFD,  // UNIT_EXPONENT (-3)
      0x75, 0x10,	// REPORT_SIZE (16)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x66, 0x00, 0x00,	// UNIT (None)
      0x55, 0x00,	// UNIT_EXPONENT (00)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x09, 0x73,	// USAGE (Set Constant Force Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x05,	// REPORT_ID (05)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x70,	// USAGE (Magnitude)
      //0x16, 0x01, 0xFF,	// LOGICAL_MINIMUM (-255)
      0x16, 0x01, 0x80,  // LOGICAL_MINIMUM (-32767) //milos, my original
      //0x16, 0xF0, 0xD8,  // LOGICAL_MINIMUM (-10000) //milos, testing
      //0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
      0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos, my original
      //0x26, 0x10, 0x27, // LOGICAL_MAXIMUM (10000) //milos, testing
      //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000) //milos, testing
      0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos, my original
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000) //milos, testing
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos, my original
      0x75, 0x10,	// REPORT_SIZE (16)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x09, 0x74,	// USAGE (Set Ramp Force Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x06,	// REPORT_ID (06)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x75,	// USAGE (Ramp Start)
      0x09, 0x76,	// USAGE (Ramp End)
      0x15, 0x81,	// LOGICAL_MINIMUM (-127)
      0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
      //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000)
      0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x02,	// REPORT_COUNT (02)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    //milos, commented since it was not used
    /*
    0x09, 0x68,	// USAGE (Custom Force Data Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x07,	// REPORT_ID (07)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x6C,	// USAGE (Custom Force Data Offset)
      //0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x16, 0x00, 0x80,  // LOGICAL_MINIMUM (-32768) //milos
      //0x26, 0x10, 0x27,	// LOGICAL_MAXIMUM (10000)
      0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos
      //0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
      //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      0x75, 0x10,	// REPORT_SIZE (16)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x69,	// USAGE (Custom Force Data)
      0x15, 0x81,	// LOGICAL_MINIMUM (-127)
      0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
      //0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos
      //0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x0C,	// REPORT_COUNT (12)
      0x92, 0x02, 0x01,	// OUTPUT (Data,Var,Abs,Buf)
      #pragma endregion
    0xC0,	// END COLLECTION ()
    */

    //milos, commented since it was not used
    /*
    0x09, 0x66,	// USAGE (Download Force Sample)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x08,	// REPORT_ID (08)
      0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
      0x09, 0x30,	// USAGE (X)
      0x09, 0x31,	// USAGE (Y)
      0x15, 0x81,	// LOGICAL_MINIMUM (-127)
      0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
      //0x35, 0x00, // PHYSICAL_MINIMUM (00)
      0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos
      //0x46, 0xFF, 0x00, // PHYSICAL_MAXIMUM (255)
      0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x02,	// REPORT_COUNT (02)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      #pragma endregion
    0xC0,	// END COLLECTION ()
    */

    0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
    0x09, 0x77,	// USAGE (Effect Operation Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x0A,	// REPORT_ID (10)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x78,	// USAGE (78)
      0xA1, 0x02,	// COLLECTION (Logical)
        #pragma region
        0x09, 0x79,	// USAGE (Op Effect Start)
        0x09, 0x7A,	// USAGE (Op Effect Start Solo)
        0x09, 0x7B,	// USAGE (Op Effect Stop)
        0x15, 0x01,	// LOGICAL_MINIMUM (01)
        0x25, 0x03,	// LOGICAL_MAXIMUM (03)
        0x75, 0x08,	// REPORT_SIZE (08)
        0x95, 0x01,	// REPORT_COUNT (01)
        0x91, 0x00,	// OUTPUT (Data,Ary,Abs)
        #pragma endregion
      0xC0,	// END COLLECTION ()
      0x09, 0x7C,	// USAGE (Loop Count)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x09, 0x90,	// USAGE (PID Block Free Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x0B,	// REPORT_ID (11)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01, // LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x09, 0x96,	// USAGE (PID Device Control)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x0C,	// REPORT_ID (12)
      0x09, 0x97,	// USAGE (DC Enable Actuators)
      0x09, 0x98,	// USAGE (DC Disable Actuators)
      0x09, 0x99,	// USAGE (DC Stop All Effects)
      0x09, 0x9A,	// USAGE (DC Device Reset)
      0x09, 0x9B,	// USAGE (DC Device Pause)
      0x09, 0x9C,	// USAGE (DC Device Continue)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x06,	// LOGICAL_MAXIMUM (06)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x00,	// OUTPUT (Data)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x09, 0x7D,	// USAGE (Device Gain Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x0D,	// REPORT_ID (14)
      0x09, 0x7E,	// USAGE (Device Gain)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x26, 0xFF, 0x00,  // LOGICAL_MAXIMUM (255)
      //0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos, back to 8bit
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
      //0x46, 0xFF, 0x7F, // PHYSICAL_MAXIMUM (32767) //milos
      0x75, 0x08,	// REPORT_SIZE (08)
      //0x75, 0x10,  // REPORT_SIZE (16) //milos, back to 8bit
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    //milos, commented since it was not used
    /*
    0x09, 0x6B,	// USAGE (Set Custom Force Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      0x85, 0x0E,	// REPORT_ID (14)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x25, 0x28,	// LOGICAL_MAXIMUM (28)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (28)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x6D,	// USAGE (Sample Count)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x09, 0x51,	// USAGE (Sample Period)
      0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
      0x55, 0xFD,  // UNIT_EXPONENT (-3)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x46, 0xFF, 0x7F,	// PHYSICAL_MAXIMUM (32767)
      0x75, 0x10,	// REPORT_SIZE (16)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x91, 0x02,	// OUTPUT (Data,Var,Abs)
      0x55, 0x00,	// UNIT_EXPONENT (00)
      0x66, 0x00, 0x00,	// UNIT (None)
    0xC0,	// END COLLECTION ()
    */

    0x09, 0xAB,	// USAGE (Create New Effect Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x05,	// REPORT_ID (05)
      0x09, 0x25,	// USAGE (Effect Type)
      0xA1, 0x02,	// COLLECTION (Logical)
        #pragma region
        0x09, 0x26, // USAGE (ET Constant Force)
        0x09, 0x27, // USAGE (ET Ramp)
        0x09, 0x30, // USAGE (ET Square)
        0x09, 0x31, // USAGE (ET Sine)
        0x09, 0x32, // USAGE (ET Triangle)
        0x09, 0x33, // USAGE (ET Sawtooth Up)
        0x09, 0x34, // USAGE (ET Sawtooth Down)
        0x09, 0x40, // USAGE (ET Spring)
        0x09, 0x41, // USAGE (ET Damper)
        0x09, 0x42, // USAGE (ET Inertia)
        0x09, 0x43, // USAGE (ET Friction)
        //0x09, 0x28,	// USAGE (ET Custom Force data) //milos, removed custom force effect
        0x15, 0x01,  // LOGICAL_MINIMUM (01)
        0x25, 0x0B,	// LOGICAL_MAXIMUM (11) //milos, was 0x0C (12)
        0x35, 0x01,	// PHYSICAL_MINIMUM (01)
        0x45, 0x0B,	// PHYSICAL_MAXIMUM (11) //milos, was 0x0C (12)
        0x75, 0x08,	// REPORT_SIZE (08)
        0x95, 0x01,	// REPORT_COUNT (01)
        0xB1, 0x00,	// FEATURE (Data)
        #pragma endregion
      0xC0,	// END COLLECTION ()
      0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
      0x09, 0x3B,	// USAGE (Byte Count)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x26, 0xFF, 0x01,	// LOGICAL_MAXIMUM (511)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x46, 0xFF, 0x01,	// PHYSICAL_MAXIMUM (511)
      0x75, 0x0A,	// REPORT_SIZE (10)
      0x95, 0x01,	// REPORT_COUNT (01)
      0xB1, 0x02,	// FEATURE (Data,Var,Abs)
      0x75, 0x06,	// REPORT_SIZE (06)
      0xB1, 0x01,	// FEATURE (Constant,Ary,Abs)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
    0x09, 0x89,	// USAGE (PID Block Load Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x06,	// REPORT_ID (06)
      0x09, 0x22,	// USAGE (Effect Block Index)
      0x25, 0x28,	// LOGICAL_MAXIMUM (40)
      0x15, 0x01,	// LOGICAL_MINIMUM (01)
      0x35, 0x01,	// PHYSICAL_MINIMUM (01)
      0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0xB1, 0x02,	// FEATURE (Data,Var,Abs)
      0x09, 0x8B,	// USAGE (Block Load Status)
      0xA1, 0x02,	// COLLECTION (Logical)
        #pragma region
        0x09, 0x8C,	// USAGE (Block Load Success)
        0x09, 0x8D,	// USAGE (Block Load Full)
        0x09, 0x8E,	// USAGE (Block Load Error)
        0x25, 0x03,	// LOGICAL_MAXIMUM (03)
        0x15, 0x01,	// LOGICAL_MINIMUM (01)
        0x35, 0x01,	// PHYSICAL_MINIMUM (01)
        0x45, 0x03,	// PHYSICAL_MAXIMUM (03)
        0x75, 0x08,	// REPORT_SIZE (08)
        0x95, 0x01,	// REPORT_COUNT (01)
        0xB1, 0x00,	// FEATURE (Data)
        #pragma endregion
      0xC0,	// END COLLECTION ()
      0x09, 0xAC,	// USAGE (RAM Pool Available)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x27, 0xFF, 0xFF, 0x00, 0x00,	// LOGICAL_MAXIMUM (65535)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
      0x75, 0x10,	// REPORT_SIZE (16)
      0x95, 0x01,	// REPORT_COUNT (01)
      0xB1, 0x00,	// FEATURE (Data)
      #pragma endregion
    0xC0,	// END COLLECTION ()

    0x09, 0x7F,	// USAGE (PID Pool Report)
    0xA1, 0x02,	// COLLECTION (Logical)
      #pragma region
      0x85, 0x07,	// REPORT_ID (07)
      0x09, 0x80,	// USAGE (RAM Pool Size)
      0x75, 0x10,	// REPORT_SIZE (16)
      0x95, 0x01,	// REPORT_COUNT (01)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x27, 0xFF, 0xFF, 0x00, 0x00,  // LOGICAL_MAXIMUM (65535)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
      0xB1, 0x02,	// FEATURE (Data,Var,Abs)
      0x09, 0x83,	// USAGE (Simultaneous Effects Max)
      0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
      0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
      0x75, 0x08,	// REPORT_SIZE (08)
      0x95, 0x01,	// REPORT_COUNT (01)
      0xB1, 0x02,	// FEATURE (Data,Var,Abs)
      0x09, 0xA9,	// USAGE (Device Managed Pool)
      0x09, 0xAA,	// USAGE (Shared Parameter Blocks)
      0x75, 0x01,	// REPORT_SIZE (01)
      0x95, 0x02,	// REPORT_COUNT (02)
      0x15, 0x00,	// LOGICAL_MINIMUM (00)
      0x25, 0x01,	// LOGICAL_MAXIMUM (01)
      0x35, 0x00,	// PHYSICAL_MINIMUM (00)
      0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
      0xB1, 0x02,	// FEATURE (Data,Var,Abs)
      0x75, 0x06,	// REPORT_SIZE (06)
      0x95, 0x01,	// REPORT_COUNT (01)
      0xB1, 0x03,	// FEATURE ( Cnst,Var,Abs)
      #pragma endregion
    0xC0,	// END COLLECTION ()

  0xC0,	// END COLLECTION ()
};
