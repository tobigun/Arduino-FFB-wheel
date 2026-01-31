#include "Config.h"
#include "common.h"
#ifdef USE_EEPROM
#include <EEPROM.h> // milos, re-implemented
#endif

uint8_t LC_scaling; // milos, load cell scaling factor (affects brake pressure, but depends on your load cell's maximum specified load)

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
int32_t CPR; // milos
int32_t ROTATION_MAX; // milos
int32_t ROTATION_MID; // milos
uint16_t MM_MIN_MOTOR_TORQUE; // milos, loaded from EEPROM
uint16_t MM_MAX_MOTOR_TORQUE; // milos, loaded from EEPROM
uint16_t MAX_DAC; // milos, loaded from EEPROM
uint16_t TOP; // milos, pwmstate byte loaded from EEPROM, then in InitPWM() function calcTOP(pwmstate) defines TOP value

float FFB_bal; // milos, FFB balance slider
float L_bal; // milos, left PWM balance multiplier
float R_bal; // milos, right PWM balance multiplier
float minTorquePP; // milos, added - min torque in percents

// milos, added - RCM pwm mode definitions
float RCM_min = 1.0; // minimal RCM pulse width in ms
float RCM_zer = 1.5; // zero RCM pulse width in ms
float RCM_max = 2.0; // maximal RCM pulse width in ms

boolean zIndexFound = false; // milos, added - keeps track if z-index pulse from encoder was found after powerup

//-------------------------------------------------------------------------------------------------------

void getParam (uint16_t offset, uint8_t *addr_to, uint8_t size) {
#ifdef USE_EEPROM
  for (uint8_t i = 0; i < size; i++) {
    addr_to[i] = EEPROM.read(offset + i);
  }
#endif
}

void setParam (uint16_t offset, uint8_t *addr_to, uint8_t size) {
#ifdef USE_EEPROM
  for (uint8_t i = 0; i < size; i++) {
    //EEPROM.write(offset + i, addr_to[i]);
    EEPROM.update(offset + i, addr_to[i]); //milos, re-write only when neccessary
  }
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
  v16 = 1080; //milos, default degrees of rotation
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
  v16 = 2047; // milos, for PWM signals
  SetParam(PARAM_ADDR_MAX_TORQ, v16); //milos, added
  v16 = 4095; // milos, for 12bit DAC
  SetParam(PARAM_ADDR_MAX_DAC, v16); // milos, added
#ifdef USE_LOAD_CELL
  v8 = 45; // milos, default max brake pressure
#else
  v8 = 128; // milos, ffb balance center value
#endif
  SetParam(PARAM_ADDR_BRK_PRES, v8); // milos, added
  v8 = 0b00000001; // milos, added, autocenter spring on
  SetParam(PARAM_ADDR_DSK_EFFC, v8); // milos, added
#ifndef USE_AS5600
  v32 = 2400; // milos, default CPR value for optical encoder (this is for 600PPR)
#else
  v32 = 4096; // milos, default CPR value for 12bit magnetic encoder AS5600
#endif
  SetParam(PARAM_ADDR_ENC_CPR, v32); // milos, added
#ifndef USE_MCP4725
  v8 = 0b01000100; // milos, PWM out enabled, fast pwm, pwm+-, 7.8kHz, TOP 11bit (2047)
#else
  v8 = 0b10000000; // milos, DAC out enabled, DAC+- mode
#endif
  SetParam(PARAM_ADDR_PWM_SET, v8); // milos, added
#ifdef USE_XY_SHIFTER
  v16 = 255;
  SetParam(PARAM_ADDR_SHFT_X0, v16); // milos, added
  v16 = 511;
  SetParam(PARAM_ADDR_SHFT_X1, v16); // milos, added
  v16 = 767;
  SetParam(PARAM_ADDR_SHFT_X2, v16); // milos, added
  v16 = 255;
  SetParam(PARAM_ADDR_SHFT_Y0, v16); // milos, added
  v16 = 511;
  SetParam(PARAM_ADDR_SHFT_Y1, v16); // milos, added
  v16 = 0; // milos, 0b00000000 - default is rev gear in 6th, no inv x-axis, no inv y-axis, no inv rev gear button
  SetParam(PARAM_ADDR_SHFT_CFG, v16); // milos, added
#endif // end of xy shifter
#ifndef USE_AUTOCALIB //milos, added - load default min/max manual cal values for all pedal axis
  v16 = 0;
  SetParam(PARAM_ADDR_ACEL_LO, v16);
  SetParam(PARAM_ADDR_BRAK_LO, v16);
  SetParam(PARAM_ADDR_CLUT_LO, v16);
  SetParam(PARAM_ADDR_HBRK_LO, v16);
  v16 = maxCal;
  SetParam(PARAM_ADDR_ACEL_HI, v16);
  SetParam(PARAM_ADDR_BRAK_HI, v16);
  SetParam(PARAM_ADDR_CLUT_HI, v16);
  SetParam(PARAM_ADDR_HBRK_HI, v16);
#endif // end of autocalib
}

void SetEEPROMConfig() { // milos, changed FIRMWARE_VERSION to 16bit from 32bit
  uint16_t v16;
  GetParam(PARAM_ADDR_FW_VERSION, v16);
  if (v16 != FIRMWARE_VERSION) { // milos, first time run, or version change - set default values for safety
    //ClearEEPROMConfig(); // milos, clear EEPROM before loading defaults
    SetDefaultEEPROMConfig(); // milos, set default firmware settings
  }
}

void LoadEEPROMConfig () { //milos, added - updates all v8 parameters from EEPROM
  GetParam(PARAM_ADDR_ROTATION_DEG, ROTATION_DEG);
#ifdef USE_ZINDEX
  GetParam(PARAM_ADDR_ENC_OFFSET, brWheelFFB.offset);
#endif
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
  GetParam(PARAM_ADDR_BRK_PRES, LC_scaling);
  GetParam(PARAM_ADDR_DSK_EFFC, effstate);
  GetParam(PARAM_ADDR_MIN_TORQ, MM_MIN_MOTOR_TORQUE);
  GetParam(PARAM_ADDR_MAX_TORQ, MM_MAX_MOTOR_TORQUE);
  GetParam(PARAM_ADDR_MAX_DAC, MAX_DAC);
#ifdef USE_MCP4725
  MM_MAX_MOTOR_TORQUE = MAX_DAC;
#endif
  GetParam(PARAM_ADDR_PWM_SET, pwmstate);
#ifdef USE_XY_SHIFTER
  GetParam(PARAM_ADDR_SHFT_X0, shifter.cal[0]); //milos, added
  GetParam(PARAM_ADDR_SHFT_X1, shifter.cal[1]); //milos, added
  GetParam(PARAM_ADDR_SHFT_X2, shifter.cal[2]); //milos, added
  GetParam(PARAM_ADDR_SHFT_Y0, shifter.cal[3]); //milos, added
  GetParam(PARAM_ADDR_SHFT_Y1, shifter.cal[4]); //milos, added
  GetParam(PARAM_ADDR_SHFT_CFG, shifter.cfg); //milos, added
#endif
#ifdef USE_AUTOCALIB // milos, added - reset autocalibration
  accel.min = Z_AXIS_LOG_MAX;
  accel.max = 0;
  brake.min = s16(Z_AXIS_LOG_MAX);
  brake.max = 0;
  clutch.min = RX_AXIS_LOG_MAX;
  clutch.max = 0;
  hbrake.min = RY_AXIS_LOG_MAX;
  hbrake.max = 0;
#else //milos, load min/max manual cal values from EEPROM
  GetParam(PARAM_ADDR_ACEL_LO, accel.min);
  GetParam(PARAM_ADDR_ACEL_HI, accel.max);
  GetParam(PARAM_ADDR_BRAK_LO, brake.min);
  GetParam(PARAM_ADDR_BRAK_HI, brake.max);
  GetParam(PARAM_ADDR_CLUT_LO, clutch.min);
  GetParam(PARAM_ADDR_CLUT_HI, clutch.max);
  GetParam(PARAM_ADDR_HBRK_LO, hbrake.min);
  GetParam(PARAM_ADDR_HBRK_HI, hbrake.max);
#endif
}

void SaveEEPROMConfig () { //milos, added - saves all v8 parameters in EEPROM
  SetParam(PARAM_ADDR_ROTATION_DEG, ROTATION_DEG);
#ifdef USE_ZINDEX
  SetParam(PARAM_ADDR_ENC_OFFSET, brWheelFFB.offset);
#endif
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
  SetParam(PARAM_ADDR_BRK_PRES, LC_scaling);
  SetParam(PARAM_ADDR_DSK_EFFC, effstate);
  SetParam(PARAM_ADDR_MIN_TORQ, MM_MIN_MOTOR_TORQUE);
  SetParam(PARAM_ADDR_MAX_TORQ, MM_MAX_MOTOR_TORQUE);
  SetParam(PARAM_ADDR_MAX_DAC, MAX_DAC);
  //SetParam(PARAM_ADDR_PWM_SET, pwmstate); // milos, do not save it with command A (we do it with W instead)
#ifdef USE_XY_SHIFTER //milos, added - save curent limits for xy shifter calibration and config
  SetParam(PARAM_ADDR_SHFT_X0, shifter.cal[0]);
  SetParam(PARAM_ADDR_SHFT_X1, shifter.cal[1]);
  SetParam(PARAM_ADDR_SHFT_X2, shifter.cal[2]);
  SetParam(PARAM_ADDR_SHFT_Y0, shifter.cal[3]);
  SetParam(PARAM_ADDR_SHFT_Y1, shifter.cal[4]);
  SetParam(PARAM_ADDR_SHFT_CFG, shifter.cfg);
#endif // end of xy shifter
#ifndef USE_AUTOCALIB //milos, added - save current manual pedal calibration config
  SetParam(PARAM_ADDR_ACEL_LO, accel.min);
  SetParam(PARAM_ADDR_ACEL_HI, accel.max);
  SetParam(PARAM_ADDR_BRAK_LO, brake.min);
  SetParam(PARAM_ADDR_BRAK_HI, brake.max);
  SetParam(PARAM_ADDR_CLUT_LO, clutch.min);
  SetParam(PARAM_ADDR_CLUT_HI, clutch.max);
  SetParam(PARAM_ADDR_HBRK_LO, hbrake.min);
  SetParam(PARAM_ADDR_HBRK_HI, hbrake.max);
#endif
}

void ClearEEPROMConfig() { //milos, added - clears EEPROM (1KB on ATmega32U4)
#ifdef USE_EEPROM
  uint8_t zero;
  zero = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    SetParam(i, zero);
  }
#endif
}

int32_t SetCPR(uint32_t newCpr, int32_t encRead) { // milos, added - update encoder CPR (for both optical and magnetic), returns new encoder possition
  int32_t temp1;
  newCpr = constrain(newCpr, 4, 600000); // milos, extended to 32bit (100000*6)
#ifdef USE_AS5600 // milos, with AS5600
  temp1 = encRead - ROTATION_MID; // milos
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
  temp1 = encRead - ROTATION_MID + brWheelFFB.offset; // milos
#endif // end of quad enc
#endif // end of as5600
  float wheelAngle = float(newCpr) * float(ROTATION_DEG) / float(ROTATION_MAX); // milos, current wheel angle
  CPR = newCpr; // milos, update CPR
  ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos, updated
  ROTATION_MID = ROTATION_MAX >> 1; // milos, updated, divide by 2
  temp1 = int32_t(wheelAngle * float(ROTATION_MAX) / float(ROTATION_DEG)); // milos, here we recover the old wheel angle
#ifdef USE_AS5600 // milos, with AS5600
  temp1 += ROTATION_MID; // milos
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
  temp1 += ROTATION_MID - brWheelFFB.offset; // milos
#endif // end of quad enc
#endif // end of as5600
  return temp1;
}

// arduino's map function works only up to range of int16_t variable (-32k,32k)
// I wrote mine that can handle 32bit variables or int32_t
int32_t myMap (int32_t value, int32_t x0, int32_t x1, int32_t y0, int32_t y1) {
  return (y0 + value * (y1 - y0) / (x1 - x0));
}

void update(fwOpt *option) { // milos, added - update firmware options from predefines above
#ifdef USE_AUTOCALIB
  option->a = true;
#endif
#ifdef USE_TWOFFBAXIS
  option->b = true;
#endif
#ifdef USE_CENTERBTN
  option->c = true;
#endif
#ifndef USE_QUADRATURE_ENCODER
  option->d = true;
#endif
#ifdef USE_EXTRABTN
  option->e = true;
#endif
#ifdef USE_XY_SHIFTER
  option->f = true;
#endif
#ifdef USE_MCP4725
  option->g = true;
#endif
#ifdef USE_HATSWITCH
  option->h = true;
#endif
#ifdef AVG_INPUTS
  option->i = true;
#endif
#ifdef USE_LOAD_CELL
  option->l = true;
#endif
#ifdef USE_PROMICRO
  option->m = true;
#endif
#ifndef USE_EEPROM
  option->p = true;
#endif
#ifdef USE_SHIFT_REGISTER
  option->n = true;
#endif
#ifdef USE_SN74ALS166N
  option->r = true;
#endif
#ifdef USE_ADS1015
  option->s = true;
#endif
#ifdef USE_BTNMATRIX
  option->t = true;
#endif
#ifdef USE_AS5600
  option->w = true;
#endif
#ifdef USE_ANALOGFFBAXIS
  option->x = true;
#endif
#ifdef USE_ZINDEX
  option->z = true;
#endif
}

// bit0-autocentering spring, bit1-damper, bit2-inertia, bit3-friction, bit4-ffb monitor (sends ffb signal data to com port), bits 5-7 are FFB axis index
// bits 5-7 define an index that sets which axis is tied to xFFB axis (by default it's at X-axis)
// index FFB-axis
// 0     X
// 1     Y
// 2     Z
// 3     RX
// 4     RY
#ifdef USE_ANALOGFFBAXIS
byte indxFFBAxis(byte value) { // milos, argument should be effstate
  byte temp = 0b00000000; // index of FFB Axis index selection (0-5)
  for (uint8_t i = 0; i < 3; i++) {
    bitWrite(temp, i, bitRead(value, i + 5)); //milos, decode bits5-7 from value byte into temp
  }
  return temp;
}
#endif

uint16_t calcTOP(byte b) { // milos, added - function which returns TOP value from pwmstate byte argument
#ifndef USE_MCP4725
  byte j = 0b00000000; // index of frequency and PWM resolution
  for (uint8_t i = 0; i < 4; i++) {
    bitWrite(j, i, bitRead(b, i + 2)); // milos, decode bits2-5 from pwmstate byte into index
  }
  if (bitRead(b, 0)) { // if phase correct PWM mode (pwmstate bit0=1)
    return (PWMtops[j] >> 1); // milos, divide by 2
  } else { // if fast PWM mode (pwmstate bit0=0)
    return (PWMtops[j]);
  }
#else // if mcp4725
  return (MAX_DAC);
#endif
}

float RCMscaler (byte value) { // milos, added - scales correctly RCM pwm mode
  if (bitRead(value, 0)) { // if pwmstate bit0=1
    return 1000.0; // for phase correct pwm mode
  } else {  // if pwmstate bit0=0
    return 2000.0; // for fast pwm mode
  }
}

// milos, added - function for decoding hat switch bits
uint32_t decodeHat(uint32_t inbits) {
  byte hat;
  byte dec = 0b00001111 & inbits; //milos, only take 1st 4 bits from inbits
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
  return ((inbits & 0b11111111111111111111111111110000) | (hat & 0b00001111)); // milos, put hat bits into first 4 bits of buttons and keep the rest unchanged
}

// milos - added, function for decoding XY shifter analog values into last 8 buttons
uint32_t decodeXYshifter (uint32_t inbits, xysh *s) {
  uint32_t gears = 0; // shifter gears represented as digital buttons (1 bit for each gear)
  const uint8_t InpRevButtonBit = 0; // input reverse gear button bit number (normal buttons start from bit4, bit0-bit3 are reserved for hat switch)
  const uint8_t OutRevButtonBit = 0; // output reverse gear button bit number (this is where we want to put reverse gear button, out of 24 available buttons)
  if (bitRead(s->cfg, 0)) {   // if reverse gear button is inverted (logitech shifters)
    if (bitRead(inbits, InpRevButtonBit + 4)) { // read rev gear button bit
      bitClear(inbits, InpRevButtonBit + 4); // if 1, set rev gear bit to 0
    } else {
      bitSet(inbits, InpRevButtonBit + 4); // if 0, set rev gear bit to 1
    }
  }
  if (s->x < int16_t(s->cal[0]) && s->y >= int16_t(s->cal[4])) { // 1st gear
    bitSet(gears, 16);
  } else if (s->x < int16_t(s->cal[0]) && s->y < int16_t(s->cal[3])) { // 2nd gear
    bitSet(gears, 17);
  } else if (s->x >= int16_t(s->cal[0]) && s->x < int16_t(s->cal[1]) && s->y >= int16_t(s->cal[4])) { // 3rd gear
    bitSet(gears, 18);
  } else if (s->x >= int16_t(s->cal[0]) && s->x < int16_t(s->cal[1]) && s->y < int16_t(s->cal[3])) { // 4th gear
    bitSet(gears, 19);
  } else if (s->x >= int16_t(s->cal[1]) && s->x < int16_t(s->cal[2]) && s->y >= int16_t(s->cal[4])) { // 5th gear
    bitSet(gears, 20);
  } else if (s->x >= int16_t(s->cal[1]) && s->x < int16_t(s->cal[2]) && s->y < int16_t(s->cal[3])) { // 6th gear
    if (bitRead(s->cfg, 1)) { // if 8 gear shifter
      bitSet(gears, 21); // set 6th gear
    } else { // if 6 gear shifter
      if (bitRead(inbits, InpRevButtonBit + 4)) {
        bitSet(gears, OutRevButtonBit); // reverse gear
      } else {
        bitSet(gears, 21); // still set 6th gear
      }
    }
  } else if (s->x >= int16_t(s->cal[2]) && s->y >= int16_t(s->cal[4])) { // 7th gear
    bitSet(gears, 22);
  } else if (s->x >= int16_t(s->cal[2]) && s->y < int16_t(s->cal[3])) { // 8th gear
    if (bitRead(s->cfg, 1)) { // if 8 gear shifter
      if (bitRead(inbits, InpRevButtonBit + 4)) {
        bitSet(gears, OutRevButtonBit); // reverse gear
      } else {
        bitSet(gears, 23); // set 8th gear
      }
    } else { // if 6 gear shifter
      bitSet(gears, 23); // still set 8th gear
    }
  }
  // milos, on leonardo/micro when both xy shifter and hat switch are used pins D5,D6,D7 are remaped to buttons 0,1,2
  // milos, so we need to stop reading these pins twice as normal buttons
  // milos, pins D5,D6,D7 are taken care of by bit mask at bit4,bit5,bit7 in xy shifter decoding function bellow
  uint32_t bitMask = 0b11110000000011111111111111101111; // milos, default bit mask, bit4=0 for reverse gear button, bits20-27 are 0 for gear buttons to be inserted
  //#ifdef USE_XY_SHIFTER // milos, with xy shifter
#ifdef USE_HATSWITCH // milos, with hat switch
#ifndef USE_PROMICRO // milos, only for leonardo/micro
#ifndef USE_SHIFT_REGISTER // milos, for shift register we need default bitmask
  bitMask = 0b11110000000011111111111101001111; // milos, modifyed bit mask not to show duplicate buttons0,1,3
#endif // end of shift register
#else // milos, for proMicro
  bitMask = 0b11110000000011111111111111101111; // milos, default bit mask
#endif // end of proMicro
#else // milos, no hat switch with xy shifter
#ifndef USE_PROMICRO // milos, only for leonardo/micro
  bitMask = 0b11110000000011111111111110001111; // milos, modifyed bit mask not to show duplicate buttons0,1,2
#endif // end of proMicro
#endif // end of hat switch
  //#endif // end of xy shifter

  return ((inbits & bitMask) | (gears << 4)); // milos, gears are shifted to the left by 4 bits to skip updating hat switch, reverse gear is at bit4 (1st bit of buttons)
}
