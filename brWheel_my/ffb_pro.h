
#ifndef _FFB_PRO_
#define _FFB_PRO_

#include <stdint.h>
#include "ffb.h"
#include "common.h"

const int32_t SPD_THRESHOLD	= 0; //8
const int32_t ACL_THRESHOLD = 0; //milos, added
const int32_t FRC_THRESHOLD = 1; //milos, added - friction treshold

const uint8_t NB_TAPS =	10; //milos, was 9
const uint8_t NB_TAPS_A = 20; //milos, added - for acceleration calc

#define FFB_ROTATION_MAX 7200
#define FFB_ROTATION_MID (FFB_ROTATION_MAX / 2)

// always keep the rotation range of a 1080° wheel for FFB calculations, even if the actual rotation range is smaller
#define FFB_ROTATION_DEG 1080

void FfbproSetAutoCenter(uint8_t enable);

void FfbproStartEffect(uint8_t id);
void FfbproStopEffect(uint8_t id);
void FfbproFreeEffect(uint8_t id);

void FfbproModifyDuration(uint8_t effectId, uint16_t duration, uint16_t stdelay); //milos, added stdelay
//void FfbproSetDeviceGain(USB_FFBReport_DeviceGain_Output_Data_t* data, volatile TEffectState* effect); //milos, added

void FfbproSetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetCondition(USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetRampForce(USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState* effect);
uint8_t FfbproSetEffect(USB_FFBReport_SetEffect_Output_Data_t* data, volatile TEffectState* effect); //milos, changed from int to uint8_t
void FfbproCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, volatile TEffectState* effect);

void FfbproEnableInterrupts(void);
const uint8_t* FfbproGetSysExHeader(uint8_t* hdr_len);

class cSpeedObs {
  public:
    cSpeedObs()	{
      Init();
    }
    void Init();
    float Update(int32_t new_pos);
    int32_t mLastPos;
    float mLastSpeed; //milo, was int32_t
    int8_t mLastValueValid;
    float mLastSpeeds[NB_TAPS]; //milos, was int32_t
    uint8_t mCurrentLS;
};

class cAccelObs {//milos, addded - acceleration
  public:
    cAccelObs()  {
      Init();
    }
    void Init();
    float Update(float new_spd); //milos, was int32_t new_spd
    float mLastSpd; //milos, was int32_t
    float mLastAccel; //milo, was int32_t
    int8_t mLastValueValid;
    float mLastAccels[NB_TAPS_A]; //milos, was int32_t
    uint8_t mCurrentLA;
};

class cFFB {
  public:
    cFFB();
    //int32_t CalcTorqueCommand (int32_t pos); // milos, returns single force (1 axis) value
    //int32_t CalcTorqueCommands (int32_t pos, int32_t pos2); // milos, returns only xFFB value, yFFB is passed through global variable
    s32v CalcTorqueCommands (s32v *pos); // milos, argument is pointer struct and returns struct holding xFFB and yFFB
    cSpeedObs mSpeed;
    cAccelObs mAccel; //milos, added
    int8_t mAutoCenter;
};

extern cFFB gFFB;

#endif // _FFB_PRO_
