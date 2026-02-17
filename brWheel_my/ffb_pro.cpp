/*
  Force Feedback Wheel

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2013  Saku Kekkonen
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

#include "ffb_pro.h"
#include "debug.h"
#ifdef __AVR__
#include <util/delay.h>
#endif

//------------------------------------- Defines ----------------------------------------------------------

const int32_t INERTIA_COEF = 16; //milos, added (wheel mass)
const int32_t DAMPER_COEF = 16; //milos, added (wheel viscosity)
const int32_t FRICTION_COEF = 8; //milos, modified (wheel friction)
const int32_t SPRING_COEF = 8; //milos, modified (wheel elasticity)

const int32_t AUTO_CENTER_SPRING = 512; //milos, modified
//#define AUTO_CENTER_DAMPER    64 //milos, commented
const int32_t BOUNDARY_SPRING	=	 32767; //milos, modified
//#define BOUNDARY_FRICTION		32 //milos, commented

//const float gSpeedSet = 0.1f;             //milos, commented
//const float gSpringSet = 0.05f; // 0.005  //milos, commented

//milos, commented
/*#define CALIBRATOR_INDEX_SEARCH_TORQUE	200
  #define CALIBRATOR_SPD_THRESHOLD		100//8
  #define CALIBRATION_DAMPER				32//250
  #define CALIBRATOR_HOMING_SPEED			5
  #define CALIBRATOR_HOMING_SPRING		128

  #define CALIBRATION_TIMEOUT_SECONDS		20

  #define SEC2FRAMES(m_sec)		(m_sec*CONTROL_FPS)
  #define INDEX_INT_NUM			1*/

//--------------------------------------- Globals --------------------------------------------------------

// milos, all effects can be divided into two groups:
// [1] time dependant effects:
//     [a] ramp
//     [b] periodic effects: sine, triangle, square, sawtoothup, sawtoothdown
//
// [2] time independant effects:
//     [a] postition dependant (X axis): spring (pos), damper (speed), inertia (acceleration), friction (speed)
//     [b] position independant: constant
//

//milos, commented since it was not used
/*float const PROGMEM fir_coefs[NB_TAPS] =
  {
  0.070573279,
  0.208085075,
  0.311521993,
  0.275324301,
  0.133483934,
  0.015952111,
  -0.01492828,
  -1.24126E-05,
  };*/

#define Btest(data,val) ((data&(val))==(val))

void cSpeedObs::Init () {
  mLastPos = 0;
  mLastSpeed = 0;
  mLastValueValid = false;
  mCurrentLS = 0;
  for (uint8_t i = 0; i < NB_TAPS; i++)
    mLastSpeeds[i] = 0;
}

float cSpeedObs::Update (int32_t new_pos) {
  float speed = new_pos - mLastPos; //milos, was int32_t speed
  mLastPos = new_pos;
  if (mLastValueValid) {
    mLastSpeeds[mCurrentLS] = speed;
    uint8_t fls = mCurrentLS;
    float avg_speed = 0;
    for (uint8_t i = 0; i < NB_TAPS; i++) {
      avg_speed += mLastSpeeds[fls]; //* fir_coefs[i];
      if (fls == 0) {
        fls = NB_TAPS - 1;
      } else {
        fls--;
      }
    }
    avg_speed /= NB_TAPS;
    mCurrentLS++;
    if (mCurrentLS >= NB_TAPS)
      mCurrentLS = 0;
    return (avg_speed);
  }
  mLastValueValid = true;
  mLastSpeed = 0;
  return (0);
}

void cAccelObs::Init () { //milos, added
  mLastSpd = 0;
  mLastAccel = 0;
  mLastValueValid = false;
  mCurrentLA = 0;
  for (uint8_t i = 0; i < NB_TAPS_A; i++) {
    mLastAccels[i] = 0;
  }
}

float cAccelObs::Update (float new_spd) { //milos, added, was int32_t new_spd
  float accel = new_spd - mLastSpd; //int32_t
  mLastSpd = new_spd;
  if (mLastValueValid) {
    mLastAccels[mCurrentLA] = accel;
    uint8_t fla = mCurrentLA;
    float avg_accel = 0;
    for (uint8_t i = 0; i < NB_TAPS_A; i++) {
      avg_accel += mLastAccels[fla]; // * fir_coefs[i];
      if (fla == 0) {
        fla = NB_TAPS_A - 1;
      } else {
        fla--;
      }
    }
    avg_accel /= NB_TAPS_A;
    mCurrentLA++;
    if (mCurrentLA >= NB_TAPS_A)
      mCurrentLA = 0;
    return (avg_accel);
  }
  mLastValueValid = true;
  mLastAccel = 0;
  return (0);
}

cFFB::cFFB() {
  mAutoCenter = true;
}

//--------------------------------------- Effects --------------------------------------------------------

int32_t ConstrainEffect (int32_t val) {
  return (constrain(val, -((int32_t)MM_MAX_MOTOR_TORQUE), (int32_t)MM_MAX_MOTOR_TORQUE));
}

float wDegScl() { // milos, added - scaling factor to convert encoder position to wheel angle units
  return (float(FFB_ROTATION_DEG) / float(FFB_ROTATION_MAX));
}

int16_t DamperEffect (float spd, int16_t mag) {
  //milos, speed in the units of wheel_angle/time_step
  if (spd > SPD_THRESHOLD)
    return (-ConstrainEffect(((spd - SPD_THRESHOLD) * mag * wDegScl()) * DAMPER_COEF * 10 / 512)); //milos
  if (spd < -SPD_THRESHOLD)
    return (-ConstrainEffect(((spd + SPD_THRESHOLD) * mag * wDegScl()) * DAMPER_COEF * 10 / 512)); //milos
  return (0);
}

int16_t InertiaEffect(float acl, int16_t mag) { //milos, modified
  //milos, acceleration in the units of wheel_angle/time_step^2
  //int16_t cmd = ConstrainEffect(mag * abs(acl) * INERTIA_COEF / 32); //milos, my new
  int16_t cmd = ConstrainEffect(mag * abs(acl) * wDegScl() * INERTIA_COEF * 10 / 32); //milos
  if (acl > ACL_THRESHOLD)
    return (-cmd);
  if (acl < -ACL_THRESHOLD)
    return (cmd);
  return (0);
}

int16_t FrictionEffect (float spd, int16_t mag) { //milos, modified
  //milos, simplified friction force model (constant above treshold, otherwise linear)
  //milos, speed in the units of wheel_angle/time_step
  int16_t cmd = mag * FRICTION_COEF / 32;
  if (spd * wDegScl() * 10 > FRC_THRESHOLD)
    return (-ConstrainEffect(cmd));
  if (spd * wDegScl() * 10 < -FRC_THRESHOLD)
    return (ConstrainEffect(cmd));
  return (-ConstrainEffect(spd * cmd * wDegScl() * 10 / FRC_THRESHOLD));
}

int32_t SpringEffect (int32_t err, int16_t mag) { //milos, modified - normalized to wheel angle
  //return (-ConstrainEffect((int32_t)((int32_t)mag * err * SPRING_COEF / 256)));
  return (-ConstrainEffect((int32_t)((int32_t)mag * err * wDegScl() * SPRING_COEF * 10 / 256))); //milos, added - with wheel angle as metric
}

int16_t SineEffect (int16_t mag, uint16_t period, uint8_t phase, uint16_t t) { //milos
  t %= period; //milos, reset timer after period reached
  return ((int16_t)(((float)mag) * sin(TWO_PI * (1.0 / (((float)period) / 1000.0) * (((float)t) / 1000.0) + (((float)phase) / 256.0))))); //milos, t increments in each cycle
}

int8_t sgn(float value) { //milos added, sign function
  return (value >= 0.0) ? 1 : -1;
}

int16_t linFunction (float k, float x, int32_t n) { //milos added, linear function y=kx+n
  return ((int16_t)(k * x) + n);
}

int16_t SquareEffect (int16_t mag, uint16_t period, uint8_t phase, uint16_t t) { //milos, added
  t %= period; //milos, reset timer after period reached
  return ((int16_t)(((float)mag) * sgn(sin(TWO_PI * (1.0 / (((float)period) / 1000.0) * (((float)t) / 1000.0) + (((float)phase) / 256.0)))))); //milos
}

int16_t TriangleEffect (int16_t mag, uint16_t period, uint8_t phase, uint16_t t) { //milos, added
  phase += 64; //milos, moved phase by PI/4, starts from 0 with rising edge
  t += (uint16_t)(((float)phase) / 256.0 * period); //milos, phase shifts time
  t %= period; //milos, reset timer after period reached
  if ((((float)t) / 1000.0) >= 0.0 && (((float)t) / 1000.0) < (((float)period) / 1000.0) / 2.0) {
    return (linFunction(4.0 * ((float)mag) / (((float)period) / 1000.0), ((float)t) / 1000.0, -mag));
  } else if ((((float)t) / 1000.0) >= (((float)period) / 1000.0) / 2.0 && (((float)t) / 1000.0) < (((float)period) / 1000.0)) {
    t -= period / 2;
    return (linFunction(-4.0 * ((float)mag) / (((float)period) / 1000.0), ((float)t) / 1000.0, mag));
  }
  return 0;
}

int16_t SawtoothUpEffect (int16_t mag, uint16_t period, uint8_t phase, uint16_t t) { //milos, added
  phase += 128; //milos, moved phase by PI/2, starts from 0 with rising edge
  t += (uint16_t)(((float)phase) / 256.0 * period); //milos, phase shifts time
  t %= period; //milos, reset timer after period reached
  if (t >= 0 && t < period) {
    return (linFunction(2.0 * ((float)mag) / (((float)period) / 1000.0), ((float)t) / 1000.0, -mag));
  }
  return 0;
}

int16_t SawtoothDownEffect (int16_t mag, uint16_t period, uint8_t phase, uint16_t t) { //milos, added
  phase += 128; //milos, moved phase by PI/2, starts from 0 with falling edge
  t += (uint16_t)(((float)phase) / 256.0 * period);
  t %= period; //milos, reset timer after period reached
  if (t >= 0 && t < period) {
    return (linFunction(2.0 * ((float)(-mag)) / (((float)period) / 1000.0), ((float)t) / 1000.0, mag));
  }
  return 0;
}

int16_t RampEffect (int8_t rStart, int8_t rEnd, uint16_t rPeriod, uint16_t t) { //milos, added
  float rSlope;
  t %= rPeriod; //milos, reset timer after period reached
  rSlope = ((float)((int32_t)(rEnd - rStart) * 256)) / (((float)rPeriod) / 1000.0); //milos, ramp slope (units magnitude/seconds)
  return (linFunction(rSlope, ((float)t) / 1000.0, (int16_t)rStart * 256));
}

int16_t ApplyEnvelope (int16_t metric, uint16_t t, uint8_t atLvl, uint8_t fdLvl, uint16_t atTime, uint16_t fdTime, uint16_t eDuration, uint16_t eStartDelay) { //milos, added - envelope block effect, start delay and duration
  float kA, kF;
  int16_t nA;
  if (metric >= 0) { // for positive magnitudes
    kA = (float)(metric - (int16_t)(atLvl * 128)) / ((float)atTime); // postitive attack slope
    nA = (int16_t)(atLvl * 128); // postitive attack offset
    kF = (float)((int16_t)(fdLvl * 128) - metric) / ((float)fdTime); // postitive fade slope
  } else { // for negative magnitudes
    kA = (float)(metric + (int16_t)(atLvl * 128)) / ((float)atTime); // negative attack slope
    nA = -(int16_t)(atLvl * 128); // negative attack offset
    kF = (float)(-metric - (int16_t)(fdLvl * 128)) / ((float)fdTime); // negative fade slope
  }

  if (t >= eStartDelay && t < eDuration + eStartDelay) { // play effect after start delay with a given effect duration
    if (t >= eStartDelay && t < atTime + eStartDelay) { // attack
      return (linFunction(kA, t - eStartDelay, nA));
    } else if (t >= atTime + eStartDelay && t <= eDuration + eStartDelay - fdTime) { // magnitude
      return (metric);
    } else if (t > eDuration + eStartDelay - fdTime && t < eDuration + eStartDelay) { // fade
      return (linFunction(kF, t - eDuration - eStartDelay + fdTime, metric));
    }
  }
  return (0);
}

int32_t ScaleMagnitude (int32_t eMag, uint16_t eGain, float eDivider) { //milos, added
  return ((int32_t)eMag * (int32_t)eGain / 32767 / eDivider); // normalizes magnitude to effect gain and all PWM modes
}

//--------------------------------------------------------------------------------------------------------

float EffectDivider() { //milos, added, calculates effects divider in order to scale magnitudes equaly for all PWM modes
  return (32767.0 / float(MM_MAX_MOTOR_TORQUE)); //milos, was 32767.0
}

//--------------------------------------------------------------------------------------------------------
//int32_t cFFB::CalcTorqueCommand (int32_t pos) { // milos, commented - old 1 axis input
s32v cFFB::CalcTorqueCommands (s32v *pos) { // milos, pointer struct agument, returns struct, pos->x: x-axis, pos->y: y-axis
  s32v command; // milos, added 2-axis FFB data
  command.x = int32_t(0);
  if (pos != NULL) { // milos, this check is always required for pointers

    float spd = mSpeed.Update(pos->x);
    float acl = mAccel.Update(spd); //milos, added - acceleration

    if (gFFB.mAutoCenter) { // milos, desktop autocenter spring effect if no FFB from any app or game
      if (bitRead(effstate, 0)) {
        /*if (abs(pos->x) > 1)*/ command.x += SpringEffect(pos->x, AUTO_CENTER_SPRING / EffectDivider() * configCenterGain / 100); //milos, autocenter spring force is equal (scaled accordingly) for all PWM modes
      }
    } else for (uint8_t id = FIRST_EID; id <= MAX_EFFECTS; id++) { // milos, if an app or game is sending FFB
        volatile TEffectState &ef = gEffectStates[id];
        if (Btest(ef.state, MEffectState_Allocated | MEffectState_Playing)) {

          int32_t mag = ScaleMagnitude(ef.magnitude, ef.gain, EffectDivider()); // milos, effects are scaled equaly for all PWM modes

          if (ef.period <= (CONTROL_PERIOD / 1000) * 2) { //milos, make sure to cap the max frequency (or to limit min period)
            ef.period = (CONTROL_PERIOD / 1000) * 2; //milos, do now allow periods less than 4ms (more than 250Hz wave we can not reproduce with 500Hz FFB calculation rate anyway)
          }

          switch (ef.type) {
            case USB_EFFECT_CONSTANT:
              command.x -= ConstrainEffect(ScaleMagnitude(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay) //milos, added
                                           , ef.gain, EffectDivider())) * configConstantGain / 100; //milos, added
              if (bitRead(ef.enableAxis, 2)) { // milos, if direction is enabled (bit2 of enableAxis byte)
                command.x *= sin(TWO_PI * ef.direction / 32768.0); //milos, added - project force vector on xFFB-axis
              }
              break;
            case USB_EFFECT_RAMP:
              command.x -= ConstrainEffect(ScaleMagnitude(ApplyEnvelope(RampEffect(ef.rampStart, ef.rampEnd, ef.duration, effectTime[id - 1]), effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay) //milos, added
                                           , ef.gain, EffectDivider())); //milos, added
              break;
            case USB_EFFECT_SINE:
              command.x += ScaleMagnitude((int32_t)ef.offset + SineEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              if (bitRead(ef.enableAxis, 2)) { // milos, if direction is enabled (bit2 of enableAxis byte)
                command.x *= sin(TWO_PI * ef.direction / 32768.0); //milos, added - project force vector on xFFB-axis
              }
              break;
            case USB_EFFECT_SQUARE:
              command.x += ScaleMagnitude((int32_t)ef.offset + SquareEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              break;
            case USB_EFFECT_TRIANGLE:
              command.x += ScaleMagnitude((int32_t)ef.offset + TriangleEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              break;
            case USB_EFFECT_SAWTOOTHUP:
              command.x += ScaleMagnitude((int32_t)ef.offset + SawtoothUpEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              break;
            case USB_EFFECT_SAWTOOTHDOWN:
              command.x += ScaleMagnitude((int32_t)ef.offset + SawtoothDownEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              break;
            case USB_EFFECT_SPRING:
              command.x += SpringEffect(pos->x - (int16_t)((int32_t(ef.offset) * FFB_ROTATION_MID) >> 15), mag * configSpringGain / 100 / 16); //milos, for spring, damper, inertia and friction forces ef.offset is cpOffset, here we scale it to FFB_ROTATION_MID
              break;
            case USB_EFFECT_DAMPER:
              command.x += DamperEffect(spd - (float)ef.offset / 1638.3, mag * configDamperGain / 100) ; //milos, here we scale it to speed
              break;
            case USB_EFFECT_INERTIA:
              command.x += InertiaEffect(acl - (float)ef.offset / 32767.0, mag * configInertiaGain / 100); //milos, here we scale it to acceleration
              break;
            case USB_EFFECT_FRICTION:
              command.x += FrictionEffect(spd - (float)ef.offset / 1638.3, mag * configFrictionGain / 100);
              break;
            case USB_EFFECT_CUSTOM:
              break;
            case USB_EFFECT_PERIODIC:
              command.x -= ConstrainEffect(ScaleMagnitude(ef.offset, 32767, EffectDivider())) * configPeriodicGain / 100; //milos, for periodic forces ef.offset changes magnitude, here we scale it to all PWM modes
              break;
            default:
              break;
          }
          effectTime[id - 1] = millis() - t0; //milos, added - advance FFB timer
        }
      }
    // milos, at the moment only xFFB axis has conditional desktop (internal) effects
    // milos, casted effect gain into uint16_t to fix desktop effects oveflow when using gains above 100
    if (bitRead(effstate, 1)) command.x += DamperEffect(spd, ScaleMagnitude((uint16_t)configDamperGain * 327, 32767, EffectDivider())) ; //milos, added - user damper effect
    if (bitRead(effstate, 2)) command.x += InertiaEffect(acl, ScaleMagnitude((uint16_t)configInertiaGain * 327, 32767, EffectDivider())) ; //milos, added - user inertia effect
    if (bitRead(effstate, 3)) command.x += FrictionEffect(spd, ScaleMagnitude((uint16_t)configFrictionGain * 327, 32767, EffectDivider())) ; //milos, added - user friction effect

    int32_t limit = FFB_ROTATION_MID; // milos, +-FFB_ROTATION_MID distance from center is where endstop spring force will start
    limit -= (FFB_ROTATION_MID >> 6); // milos, here you can offset endstop activation point by FFB_ROTATION_MID/64 (optical or magnetic encoders can go past the axis range limit, this is required only for analog input - pot)
    if ((pos->x < -limit) || (pos->x > limit)) {
      if (pos->x >= 0) {
        pos->x = pos->x - limit; //milos
      } else {
        pos->x = pos->x + limit; //milos
      }
      command.x += SpringEffect(pos->x, BOUNDARY_SPRING / EffectDivider() * configStopGain / 100); // milos, boundary spring force is equal (scaled accordingly) for all PWM modes, endstop force for xFFB axis
    }

    command.x = ConstrainEffect(command.x * configGeneralGain / 100);
    if (bitRead(effstate, 4)) CONFIG_SERIAL.println(command.x); // milos, added - FFB real time monitor
  }
  return (command); // milos, passing the struct
}

//--------------------------------------------------------------------------------------------------------

void FfbproSetAutoCenter(uint8_t enable)
{
  gFFB.mAutoCenter = enable;
}

void FfbproEnableInterrupts(void)
{
}

const uint8_t* FfbproGetSysExHeader(uint8_t* hdr_len)
{
  static const uint8_t header[] = {0xf0, 0x00, 0x01, 0x0a, 0x01};
  *hdr_len = sizeof(header);
  return header;
}

// effect operations ---------------------------------------------------------

void FfbproStartEffect(uint8_t effectId)
{
  gFFB.mAutoCenter = false;
  effectTime[effectId - 1] = 0; //milos, added - reset timer for this effect when we start it
}

void FfbproStopEffect(uint8_t effectId)
{
}

void FfbproFreeEffect(uint8_t effectId)
{
}

// modify operations ---------------------------------------------------------

void FfbproModifyDuration(uint8_t effectId, uint16_t duration, uint16_t stdelay) //milos, added stdelay
{
  volatile TEffectState* effect = &gEffectStates[effectId]; // milos, added
  effect->duration = duration; // milos, added
  effect->startDelay = stdelay; // milos, added
  effectTime[effectId - 1] = 0; // milos, added - reset timer for this effect when duration is changed
}

void FfbproSetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState * effect)
{
  effect->attackLevel = (uint8_t)data->attackLevel;
  effect->fadeLevel = (uint8_t)data->fadeLevel;
  effect->attackTime = (uint16_t)data->attackTime; // milos, added
  effect->fadeTime = (uint16_t)data->fadeTime;
}

void FfbproSetCondition (USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState * effect)
{
  effect->parameterBlockOffset = (uint8_t)data->parameterBlockOffset; // milos, added - pass the effect condition block index
  if (effect->parameterBlockOffset == 0) { // milos, condition block for xFFB
    effect->magnitude = (int16_t)data->positiveCoefficient; // milos, postitve coefficient can also be negative
    effect->offset = (int16_t)data->cpOffset; // milos, this offset changes X-pos
    effect->deadBand = (uint8_t)data->deadBand; // milos, added
  } else if (effect->parameterBlockOffset == 1) { // milos, condition block for yFFB
  }
  //effect->positiveSaturation = (int16_t)data->positiveSaturation; // milos, posititve saturation can also be negative (not used currently)
}

void FfbproSetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState * effect)
{
  effect->magnitude = (uint16_t)data->magnitude;
  effect->offset = (((int16_t)data->offset)); // milos, this offset changes magnitude
  effect->phase = (uint8_t)data->phase;
  effect->period = (uint16_t)data->period;
}

void FfbproSetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState * effect)
{
  effect->magnitude = data->magnitude;
}

void FfbproSetRampForce (USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState * effect)
{
  effect->rampStart = data->rampStart; // milos, added
  effect->rampEnd = data->rampEnd; // milos, added
}

uint8_t FfbproSetEffect (USB_FFBReport_SetEffect_Output_Data_t* data, volatile TEffectState * effect)  //milos, changed from int to uint8_t
{
  uint8_t eid = data->effectBlockIndex;
  effect->type = data->effectType; // milos, this is where effect type is being set
  effect->gain = (int16_t)data->gain;
  FfbproModifyDuration(eid, (uint16_t)data->duration, (uint16_t)data->startDelay); // milos, added
  effect->direction = (uint16_t)data->direction; // milos, added
  effect->enableAxis = (uint8_t)data->enableAxis; // milos, added
  return 1;
}

void FfbproCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, volatile TEffectState * effect)
{
}
