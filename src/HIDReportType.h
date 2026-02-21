/*
  Force Feedback Joystick
  USB HID descriptors for a force feedback joystick.
  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2016  Jaka Simonic (telesimke [at] gmail [dot] com)
  Copyright 2018-2025  Milos Rankovic (ranenbg [at] gmail [dot] com)
  Copyright 2019  Hoan Tran (tranvanhoan206 [at] gmail [dot] com)
  Copyright 2021  Sulako
  MIT License.

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

#pragma once

#include <stdint.h>

/* Type Defines: */
/** Type define for the joystick HID report structure, for creating and sending HID reports to the host PC.
    This mirrors the layout described to the host in the HID report descriptor, in Descriptors.c.
*/

#ifdef __AVR__
#define FFBREPORT_WITH_REPORTID
#endif

#define FIRST_EID	1

// Maximum number of parallel effects in memory
#define MAX_EFFECTS     14
#define SIZE_EFFECT     sizeof(TEffectState)
#define MEMORY_SIZE     (uint16_t)(MAX_EFFECTS*SIZE_EFFECT)

// ---- Input

struct ATTR_PACKED USB_FFBReport_PIDStatus_Input_Data_t
{
  uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
  uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)
};

// ---- Output

struct ATTR_PACKED USB_FFBReport_SetEffect_Output_Data_t
{ // FFB: Set Effect Output Report
  uint8_t	reportId;	// =1
  uint8_t	effectBlockIndex;	// 1..40
  uint8_t	effectType;	// 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28)
  uint16_t duration; // 0..32767 ms
  uint16_t triggerRepeatInterval; // 0..32767 ms
  uint16_t samplePeriod;	// 0..32767 ms
  uint8_t	gain;	// 0..255	 (physical 0..10000)
  uint8_t	triggerButton;	// button ID (0..8)
  uint8_t	enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
  uint8_t	directionX;	// angle (0=0 .. 255=360deg)
  uint8_t	directionY;	// angle (0=0 .. 255=360deg)
  //	uint16_t	startDelay;	// 0..32767 ms
};

struct ATTR_PACKED USB_FFBReport_SetEnvelope_Output_Data_t
{ // FFB: Set Envelope Output Report
  uint8_t	reportId;	// =2
  uint8_t	effectBlockIndex;	// 1..40
  uint16_t attackLevel;
  uint16_t	fadeLevel;
  uint16_t	attackTime;	// ms
  uint16_t	fadeTime;	// ms
};

struct ATTR_PACKED USB_FFBReport_SetCondition_Output_Data_t
{ // FFB: Set Condition Output Report
  uint8_t	reportId;	// =3
  uint8_t	effectBlockIndex;	// 1..40
  uint8_t	parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
  int16_t cpOffset;	// 0..255
  int16_t	positiveCoefficient;	// -128..127
  int16_t	negativeCoefficient;	// -128..127
  uint16_t	positiveSaturation;	// -	128..127
  uint16_t	negativeSaturation;	// -128..127
  uint16_t	deadBand;	// 0..255
};

struct ATTR_PACKED USB_FFBReport_SetPeriodic_Output_Data_t
{ // FFB: Set Periodic Output Report
  uint8_t	reportId;	// =4
  uint8_t	effectBlockIndex;	// 1..40
  uint16_t magnitude;
  int16_t	offset;
  uint16_t	phase;	// 0..255 (=0..359, exp-2)
  uint16_t	period;	// 0..32767 ms
};

struct ATTR_PACKED USB_FFBReport_SetConstantForce_Output_Data_t
{ // FFB: Set ConstantForce Output Report
  uint8_t	reportId;	// =5
  uint8_t	effectBlockIndex;	// 1..40
  int16_t magnitude;	// -255..255
};

struct ATTR_PACKED USB_FFBReport_SetRampForce_Output_Data_t
{ // FFB: Set RampForce Output Report
  uint8_t	reportId;	// =6
  uint8_t	effectBlockIndex;	// 1..40
  int16_t startMagnitude;
  int16_t	endMagnitude;
};

struct ATTR_PACKED USB_FFBReport_SetCustomForceData_Output_Data_t
{ // FFB: Set CustomForceData Output Report
  uint8_t	reportId;	// =7
  uint8_t	effectBlockIndex;	// 1..40
  uint16_t dataOffset;
  int8_t	data[12];
};

struct ATTR_PACKED USB_FFBReport_SetDownloadForceSample_Output_Data_t
{ // FFB: Set DownloadForceSample Output Report
  uint8_t	reportId;	// =8
  int8_t	x;
//  int8_t	y;
};

struct ATTR_PACKED USB_FFBReport_EffectOperation_Output_Data_t
{ // FFB: Set EffectOperation Output Report
  uint8_t	reportId;	// =10
  uint8_t effectBlockIndex;	// 1..40
  uint8_t operation; // 1=Start, 2=StartSolo, 3=Stop
  uint8_t	loopCount; //0..255 (physical 0..255)
};

struct ATTR_PACKED USB_FFBReport_BlockFree_Output_Data_t
{ // FFB: Block Free Output Report
  uint8_t	reportId;	// =11
  uint8_t effectBlockIndex;	// 1..40
};

struct ATTR_PACKED USB_FFBReport_DeviceControl_Output_Data_t
{ // FFB: Device Control Output Report
  uint8_t	reportId;	// =12
  uint8_t control;	// 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
};

struct ATTR_PACKED USB_FFBReport_DeviceGain_Output_Data_t
{ // FFB: DeviceGain Output Report
  uint8_t	reportId;	// =13
  uint8_t gain;
};

struct ATTR_PACKED USB_FFBReport_SetCustomForce_Output_Data_t
{ // FFB: Set Custom Force Output Report
  uint8_t	reportId;	// =14
  uint8_t effectBlockIndex;	// 1..40
  uint8_t	sampleCount;
  uint16_t samplePeriod;	// 0..32767 ms
};

// ---- Features

struct ATTR_PACKED USB_FFBReport_CreateNewEffect_Feature_Data_t
{ // FFB: Create New Effect Feature Report
#ifdef FFBREPORT_WITH_REPORTID
  uint8_t reportId;	// =1
#endif
  uint8_t	effectType;	// Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43 //,28 //milos, total 11, 28 custom force is removed
  uint16_t byteCount;	// 0..511
};

struct ATTR_PACKED USB_FFBReport_PIDBlockLoad_Feature_Data_t
{ // FFB: PID Block Load Feature Report
#ifdef FFBREPORT_WITH_REPORTID
  uint8_t	reportId;	// =2
#endif
  uint8_t effectBlockIndex;	// 1..40
  uint8_t	loadStatus;	// 1=Success,2=Full,3=Error
  uint16_t ramPoolAvailable;	// =0 or 0xFFFF?
};

struct ATTR_PACKED USB_FFBReport_PIDPool_Feature_Data_t
{ // FFB: PID Pool Feature Report
#ifdef FFBREPORT_WITH_REPORTID
  uint8_t	reportId;	// =3
#endif
  uint16_t ramPoolSize;	// 65535
  uint8_t	maxSimultaneousEffects;	// ?? 40?
  uint8_t	memoryManagement;	// Bits: 0=DeviceManagedPool, 1=SharedParameterBlocks
};

// ---- effect

#define USB_DURATION_INFINITE		0x7FFF

#define USB_EFFECT_CONSTANT	  	0x01
#define USB_EFFECT_RAMP					0x02
#define USB_EFFECT_SQUARE 			0x03
#define USB_EFFECT_SINE 				0x04
#define USB_EFFECT_TRIANGLE			0x05
#define USB_EFFECT_SAWTOOTHDOWN	0x06
#define USB_EFFECT_SAWTOOTHUP		0x07
#define USB_EFFECT_SPRING				0x08
#define USB_EFFECT_DAMPER				0x09
#define USB_EFFECT_INERTIA			0x0A
#define USB_EFFECT_FRICTION			0x0B
#define USB_EFFECT_CUSTOM				0x0C

// Bit-masks for effect states
#define MEFFECTSTATE_FREE				0x00
#define MEFFECTSTATE_ALLOCATED	0x01
#define MEFFECTSTATE_PLAYING		0x02


typedef struct {
  uint8_t state;  // see constants <MEffectState_*>
  uint8_t effectType;
  uint8_t gain;
  uint8_t enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
  uint8_t directionX; // angle (0=0 .. 255=360deg)
  uint8_t directionY; // angle (0=0 .. 255=360deg)
  uint16_t elapsedTime;
  uint16_t duration;
  
  int16_t magnitude;

  int16_t offset;
  uint16_t period;              // 0..10000 ms

  uint16_t attackLevel, fadeLevel;
  uint16_t fadeTime, attackTime;
  
  int16_t  positiveCoefficient;
  int16_t  negativeCoefficient;
  uint16_t positiveSaturation;  
  uint16_t negativeSaturation; 
  int16_t cpOffset;           
  uint16_t deadBand;           

  uint16_t halfPeriod;              
  uint16_t periodTime;
 
  float fadeTimeC, attackTimeC, periodC;

} TEffectState;
