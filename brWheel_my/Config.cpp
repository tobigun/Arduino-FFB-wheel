#include "Config.h"
#include "common.h"
#include <EEPROM.h>

uint8_t ffbBalance; // milos, load cell scaling factor (affects brake pressure, but depends on your load cell's maximum specified load)

// milos, these are now loaded from EEPROM
uint8_t effstate; // = 0b00000001; // milos, added - turn on/off desktop effects through serial interface, bit 7 is MSB

//---------------------
// bit0-phase correct (0 is fast pwm), bit1-dir enabled (0 is pwm+-), bits 2-5 are frequency select, bit6-enable pwm0-50-100, bit7 is unused
// bit0 pwm_type
// 0    fast pwm
// 1    phase correct

// bits2-5 define frequency index, bit2 is MSB, see firmware_info.txt for details

// bit1 bit6 pwm_mode
// 0    0    pwm+-
// 0    1    pwm0-50-100
// 1    0    pwm+dir
// 1    1    rcm
//----------------------

// milos, if USE_MCP4725 is defined then pwmstate byte has the following interpretation
//----------------
// bits 0-4 unused, bit5-enable dac0-50-100, bit6-enable dac+dir (0 is dac+-), bit7-enable dac (0 is zero dac output)

// bit7 dac_out
// 0   disabled (set to 0V or 2.5V depending on dac mode)
// 1   enabled

// bit6 bit5 dac_mode      dac_mode2 (2-ffb axis)
// 0    0    dac+-         1ch dac+- (xFFB)
// 0    1    dac0-50-100   2ch dac0-50-100
// 1    0    dac+dir       2ch dac+dir
// 1    1    none
//----------------
uint8_t pwmstate; // milos, PWM settings configuration byte, bit7 is MSB

// milos, changed these from float to uint8_t (loaded from EEPROM)
uint8_t configGeneralGain;  // = 1.0f;  // was 1.0f
uint8_t configDamperGain; // = 1.0f;		// was 0.5f
uint8_t configFrictionGain; // = 1.0f;	// was 0.5f
uint8_t configConstantGain; // = 1.0f;	// was 1.0f
uint8_t configPeriodicGain; // = 1.0f;	// was 1.0f
uint8_t configSpringGain; // = 1.0f;	// was 1.0f
uint8_t configInertiaGain; // = 1.0f; // was 1.0f
uint8_t configCenterGain; // = 0.7f;	// was 0.7f
uint8_t configStopGain; // = 1.0f;	// was 1.0f


uint16_t PWMtops [13] =
{
  400,  // 0
  800,  // 1
  1023, // 2
  2047, // 3
  4095, // 4
  5000, // 5
  10000, // 6
  16383, // 7
  20000, // 8
  32767, // 9
  30000, // 10
  40000, // 11
  65535  // 12
};

int16_t ROTATION_DEG; // milos
int32_t CPR; // counts-per-rotation. Unused, as this only makes sense for optical or magnetic encoders
uint16_t MM_MIN_MOTOR_TORQUE; // milos, loaded from EEPROM
uint16_t MM_MAX_MOTOR_TORQUE; // milos, loaded from EEPROM
uint16_t MAX_DAC; // milos, loaded from EEPROM

//-------------------------------------------------------------------------------------------------------

void getParam (uint16_t offset, uint8_t *addr_to, uint8_t size) {
  for (uint8_t i = 0; i < size; i++) {
    addr_to[i] = EEPROM.read(offset + i);
  }
}

void setParam (uint16_t offset, uint8_t *addr_to, uint8_t size) {
  for (uint8_t i = 0; i < size; i++) {
  #ifdef __AVR__
    EEPROM.update(offset + i, addr_to[i]); //milos, re-write only when neccessary
  #else
    EEPROM.write(offset + i, addr_to[i]);
  #endif
  }
}

void InitEEPROMConfig() {
#ifndef __AVR__
  EEPROM.begin(EEPROM_SIZE);
#endif
}

void SetDefaultEEPROMConfig() { // milos - store default firmware settings in EEPROM
  uint16_t v16;
  int32_t v32;
  uint8_t v8;
  v16 = FIRMWARE_VERSION;
  SetParam(PARAM_ADDR_FW_VERSION, v16);
  v32 = 0;
  SetParam(PARAM_ADDR_ENC_OFFSET, v32);
  v16 = 1080; //default degrees of rotation. Although we only have 210°, 1080° seem to result in better force feedback behavior
  SetParam(PARAM_ADDR_ROTATION_DEG, v16); //milos, added
  v8 = 100; //milos, added
  SetParam(PARAM_ADDR_GEN_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_CNT_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_PER_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_STP_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_SPR_GAIN, v8); //milos, added
  v8 = 50; //milos, added
  SetParam(PARAM_ADDR_DMP_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_FRC_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_INR_GAIN, v8); //milos, added
  v8 = 70; //milos, added
  SetParam(PARAM_ADDR_CTS_GAIN, v8); // milos, added
  v16 = 0; // milos, added
  SetParam(PARAM_ADDR_MIN_TORQ, v16); //milos, added
  v16 = MAX_TORQ_DEFAULT; // milos, for PWM signals
  SetParam(PARAM_ADDR_MAX_TORQ, v16); //milos, added
  v16 = 4095; // milos, for 12bit DAC
  SetParam(PARAM_ADDR_MAX_DAC, v16); // milos, added
  v8 = 128; // milos, ffb balance center value
  SetParam(PARAM_ADDR_BRK_PRES, v8); // milos, added
  v8 = 0b00000001; // milos, added, autocenter spring on
  SetParam(PARAM_ADDR_DSK_EFFC, v8); // milos, added
  v32 = 2400; // milos, default CPR value for optical encoder (this is for 600PPR)
  SetParam(PARAM_ADDR_ENC_CPR, v32); // milos, added
  v8 = 0b01000100; // milos, PWM out enabled, fast pwm, pwm+-, 7.8kHz, TOP 11bit (2047)
  SetParam(PARAM_ADDR_PWM_SET, v8); // milos, added
  v16 = 0;
  SetParam(PARAM_ADDR_ACEL_LO, v16);
  SetParam(PARAM_ADDR_BRAK_LO, v16);
  SetParam(PARAM_ADDR_CLUT_LO, v16);
  SetParam(PARAM_ADDR_HBRK_LO, v16);
  v16 = 1023;
  SetParam(PARAM_ADDR_ACEL_HI, v16);
  SetParam(PARAM_ADDR_BRAK_HI, v16);
  SetParam(PARAM_ADDR_CLUT_HI, v16);
  SetParam(PARAM_ADDR_HBRK_HI, v16);
#ifndef __AVR__
  EEPROM.commit();
#endif
}

void SetEEPROMConfig() { // milos, changed FIRMWARE_VERSION to 16bit from 32bit
  uint16_t v16;
  GetParam(PARAM_ADDR_FW_VERSION, v16);
  if (v16 != FIRMWARE_VERSION) { // milos, first time run, or version change - set default values for safety
    SetDefaultEEPROMConfig(); // milos, set default firmware settings
  }
}

void LoadEEPROMConfig () { //milos, added - updates all v8 parameters from EEPROM
  GetParam(PARAM_ADDR_ROTATION_DEG, ROTATION_DEG);
  GetParam(PARAM_ADDR_ENC_CPR, CPR);
  GetParam(PARAM_ADDR_GEN_GAIN, configGeneralGain);
  GetParam(PARAM_ADDR_DMP_GAIN, configDamperGain);
  GetParam(PARAM_ADDR_FRC_GAIN, configFrictionGain);
  GetParam(PARAM_ADDR_CNT_GAIN, configConstantGain);
  GetParam(PARAM_ADDR_PER_GAIN, configPeriodicGain);
  GetParam(PARAM_ADDR_SPR_GAIN, configSpringGain);
  GetParam(PARAM_ADDR_INR_GAIN, configInertiaGain);
  GetParam(PARAM_ADDR_CTS_GAIN, configCenterGain);
  GetParam(PARAM_ADDR_STP_GAIN, configStopGain);
  GetParam(PARAM_ADDR_BRK_PRES, ffbBalance);
  GetParam(PARAM_ADDR_DSK_EFFC, effstate);
  GetParam(PARAM_ADDR_MIN_TORQ, MM_MIN_MOTOR_TORQUE);
  GetParam(PARAM_ADDR_MAX_TORQ, MM_MAX_MOTOR_TORQUE);
  GetParam(PARAM_ADDR_MAX_DAC, MAX_DAC);
  GetParam(PARAM_ADDR_PWM_SET, pwmstate);
  GetParam(PARAM_ADDR_ACEL_LO, accel.min);
  GetParam(PARAM_ADDR_ACEL_HI, accel.max);
  GetParam(PARAM_ADDR_BRAK_LO, brake.min);
  GetParam(PARAM_ADDR_BRAK_HI, brake.max);
  GetParam(PARAM_ADDR_CLUT_LO, clutch.min);
  GetParam(PARAM_ADDR_CLUT_HI, clutch.max);
  GetParam(PARAM_ADDR_HBRK_LO, hbrake.min);
  GetParam(PARAM_ADDR_HBRK_HI, hbrake.max);
}

void SaveEEPROMConfig () { //milos, added - saves all v8 parameters in EEPROM
  SetParam(PARAM_ADDR_ROTATION_DEG, ROTATION_DEG);
  SetParam(PARAM_ADDR_ENC_CPR, CPR);
  SetParam(PARAM_ADDR_GEN_GAIN, configGeneralGain);
  SetParam(PARAM_ADDR_DMP_GAIN, configDamperGain);
  SetParam(PARAM_ADDR_FRC_GAIN, configFrictionGain);
  SetParam(PARAM_ADDR_CNT_GAIN, configConstantGain);
  SetParam(PARAM_ADDR_PER_GAIN, configPeriodicGain);
  SetParam(PARAM_ADDR_SPR_GAIN, configSpringGain);
  SetParam(PARAM_ADDR_INR_GAIN, configInertiaGain);
  SetParam(PARAM_ADDR_CTS_GAIN, configCenterGain);
  SetParam(PARAM_ADDR_STP_GAIN, configStopGain);
  SetParam(PARAM_ADDR_BRK_PRES, ffbBalance);
  SetParam(PARAM_ADDR_DSK_EFFC, effstate);
  SetParam(PARAM_ADDR_MIN_TORQ, MM_MIN_MOTOR_TORQUE);
  SetParam(PARAM_ADDR_MAX_TORQ, MM_MAX_MOTOR_TORQUE);
  SetParam(PARAM_ADDR_MAX_DAC, MAX_DAC);
  //SetParam(PARAM_ADDR_PWM_SET, pwmstate); // milos, do not save it with command A (we do it with W instead)
  SetParam(PARAM_ADDR_ACEL_LO, accel.min);
  SetParam(PARAM_ADDR_ACEL_HI, accel.max);
  SetParam(PARAM_ADDR_BRAK_LO, brake.min);
  SetParam(PARAM_ADDR_BRAK_HI, brake.max);
  SetParam(PARAM_ADDR_CLUT_LO, clutch.min);
  SetParam(PARAM_ADDR_CLUT_HI, clutch.max);
  SetParam(PARAM_ADDR_HBRK_LO, hbrake.min);
  SetParam(PARAM_ADDR_HBRK_HI, hbrake.max);
#ifndef __AVR__
  EEPROM.commit();
#endif
}

void ClearEEPROMConfig() { //milos, added - clears EEPROM (1KB on ATmega32U4)
  uint8_t zero = 0;
  for (uint16_t i = 0; i < EEPROM_SIZE; i++) {
    SetParam(i, zero);
  }
#ifndef __AVR__
  EEPROM.commit();
#endif
}

// arduino's map function works only up to range of int16_t variable (-32k,32k)
// I wrote mine that can handle 32bit variables or int32_t
int32_t myMap (int32_t value, int32_t x0, int32_t x1, int32_t y0, int32_t y1) {
  return (y0 + value * (y1 - y0) / (x1 - x0));
}

uint16_t calcTOP(byte b) { // milos, added - function which returns TOP value from pwmstate byte argument
#ifdef __AVR__
  byte j = 0b00000000; // index of frequency and PWM resolution
  for (uint8_t i = 0; i < 4; i++) {
    bitWrite(j, i, bitRead(b, i + 2)); // milos, decode bits2-5 from pwmstate byte into index
  }
  if (bitRead(b, 0)) { // if phase correct PWM mode (pwmstate bit0=1)
    return (PWMtops[j] >> 1); // milos, divide by 2
  } else { // if fast PWM mode (pwmstate bit0=0)
    return (PWMtops[j]);
  }
#else
  return MM_MAX_MOTOR_TORQUE;
#endif
}
