#pragma once

#include <Arduino.h>

//------------------------------------- Firmware options -------------------------------------------------

#define USE_CONFIGCDC      // milos, use virtual RS232 serial port for configuring firmware settings
//#define USE_CONFIGHID      // milos, use native USB HID protocol for configuring firmware settings (not implemented fully yet)
//#define USE_QUADRATURE_ENCODER		// milos, optical quadrature encoder (you may comment it out)
//#define USE_ZINDEX          // milos, use Z-index encoder channel (caution, can not be used with USE_ADS1015, USE_MCP4725 or USE_AS5600)
#define USE_HATSWITCH        // milos, uncomment to use first 4 buttons for hat switch (D-pad)
#define USE_BTNMATRIX        // milos, uncomment to use 8 pins as a 4x4 button matrix for total of 16 buttons (can not be used with load cell, shift register or XY shifter)
//#define AVG_INPUTS        // milos, uncomment to use averaging of arduino analog inputs (can not be used with USE_ADS1015)
//#define USE_AUTOCALIB        // milos, uncomment to use autocalibration for pedal axis (if left commented manual calibration is enabled)
#define USE_ANALOGFFBAXIS // milos, uncomment to enable other than X-axis to be tied with xFFB axis (you can use analog inputs instead of digital encoders
#define USE_PROMICRO    // milos, uncomment if you are using Arduino ProMicro board (leave commented for Leonardo or Micro variants)
#define USE_EEPROM     // milos, uncomment to enable loading/saving settings from EEPROM (if commented out, default settings will be loaded on each powerup, one needs to reconfigure firmware defautls or use GUI configuration after each powerup) 
#define USE_ANALOG_XAXIS // use dedicated analog X-Axis

#define USE_PWM_0_50_100_MODE
//#define USE_PWM_DIR_MODE
//#define USE_PWM_PLUS_MINUS_MODE

#define CALIBRATE_AT_INIT	0 // milos, was 1

//------------------------------------- Pins -------------------------------------------------------------

//#define LED_PIN				12
//#define	LCSYNC_LED_PIN		12
//#define	SYNC_LED_PIN		12 //milos, USB polling clock

//milos, added - ffb clip LED indicator

#ifdef USE_PROMICRO // milos, if we use proMicro
//#define LED_GREEN_PIN 1 // Green LED directly connected to 12V supply
#define LED_BLUE_PIN 0
#define LED_RED_PIN 3 // for ProMicro if no above i2C devices
#define FFBCLIP_LED_PIN LED_RED_PIN // for ProMicro if no above i2C devices
#endif // end of promicro

#ifdef USE_ANALOGFFBAXIS
#define XAXIS_PIN			A0
#define BRAKE_PIN     A1
#define ACCEL_PIN			A2
#define CLUTCH_PIN		A3
#define HBRAKE_PIN    A8
#else
#define ACCEL_PIN			A0
#define BRAKE_PIN     A1
#define CLUTCH_PIN    A2
#define HBRAKE_PIN    A3
#endif

#define BUTTON0 5 // D5, used for button0
#define B0PORTBIT 6 // read bit6 of PINC
#define BUTTON1 14 // D14, used for button1 instead
#define B1PORTBIT 3 // read bit3 of PINB
#define BUTTON2 15 // D15, used for button2 instead
#define B2PORTBIT 1 // read bit1 of PINB
#define BUTTON3 2 // D2, used for button3 instead on proMicro
#define B3PORTBIT 1 // read bit1 of PIND
#define BUTTON4 4 // D4, used for button4
#define B4PORTBIT 4 // read bit4 of PIND
#define BUTTON5 7 // D7 or bit6 of PINE
#define B5PORTBIT 6 // bit6
#define BUTTON6 1 // D1 or bit3 of PIND
#define B6PORTBIT 3 // bit3
#define BUTTON7 6 // D6, used for button7
#define B7PORTBIT 7 // read bit7 of PIND

#define PWM_PIN_L     9 // milos, left PWM pin
#if defined(USE_PWM_0_50_100_MODE) || defined(USE_PWM_DIR_MODE)
#define DIR_PIN       10
#else
#define PWM_PIN_R     10 // milos, right PWM pin
#endif

#define ACCEL_INPUT 0
#define BRAKE_INPUT 1
#define CLUTCH_INPUT 2
#define HBRAKE_INPUT 3

//------------------------------------- EEPROM Config -----------------------------------------------------

#define PARAM_ADDR_FW_VERSION		 0x00 //milos, firmware version
#define PARAM_ADDR_ENC_OFFSET    0x02 //milos, Z-index offset
#define PARAM_ADDR_ROTATION_DEG  0x06 //milos, rotation degrees
#define PARAM_ADDR_GEN_GAIN      0x08 //milos, general
#define PARAM_ADDR_DMP_GAIN      0x09 //milos, damper
#define PARAM_ADDR_FRC_GAIN      0x0A //milos, friction
#define PARAM_ADDR_CNT_GAIN      0x0B //milos, constant
#define PARAM_ADDR_PER_GAIN      0x0C //milos, periodic
#define PARAM_ADDR_SPR_GAIN      0x0D //milos, spring
#define PARAM_ADDR_INR_GAIN      0x0E //milos, inertia
#define PARAM_ADDR_CTS_GAIN      0x0F //milos, centering spring
#define PARAM_ADDR_STP_GAIN      0x10 //milos, end stop
#define PARAM_ADDR_MIN_TORQ      0x11 //milos, min torque
#define PARAM_ADDR_MAX_TORQ      0x13 //milos, max torque
#define PARAM_ADDR_MAX_DAC       0x15 //milos, max DAC value
#define PARAM_ADDR_BRK_PRES      0x17 //milos, max brake pressure
#define PARAM_ADDR_DSK_EFFC      0x18 //milos, desktop effects (byte contents is in effstate)
#define PARAM_ADDR_ENC_CPR       0x19 //milos, encoder CPR
#define PARAM_ADDR_PWM_SET       0x1D //milos, PWM settings and frequency (byte contents is in pwmstate)
#define PARAM_ADDR_SHFT_X0       0x1E //milos, XY shifter limit x0
#define PARAM_ADDR_SHFT_X1       0x20 //milos, XY shifter limit x1
#define PARAM_ADDR_SHFT_X2       0x22 //milos, XY shifter limit x2
#define PARAM_ADDR_SHFT_Y0       0x24 //milos, XY shifter limit y0
#define PARAM_ADDR_SHFT_Y1       0x26 //milos, XY shifter limit y1
#define PARAM_ADDR_SHFT_CFG      0x28 //milos, shifter configuration byte
#define PARAM_ADDR_ACEL_LO       0x2A //milos, accelerator pedal cal min
#define PARAM_ADDR_ACEL_HI       0x2C //milos, accelerator pedal cal max
#define PARAM_ADDR_BRAK_LO       0x2E //milos, brake pedal cal min
#define PARAM_ADDR_BRAK_HI       0x30 //milos, brake pedal cal max
#define PARAM_ADDR_CLUT_LO       0x32 //milos, clutch pedal cal min
#define PARAM_ADDR_CLUT_HI       0x34 //milos, clutch pedal cal max
#define PARAM_ADDR_HBRK_LO       0x36 //milos, hand brake pedal cal min
#define PARAM_ADDR_HBRK_HI       0x38 //milos, hand brake pedal cal max

#define FIRMWARE_VERSION         0xFA // milos, firmware version (FA=250, FB=251, FC=252, FD=253)

void setParam (uint16_t offset, uint8_t *addr_to, uint8_t size);

#define GetParam(m_offset,m_data)	getParam((m_offset),(uint8_t*)&(m_data),sizeof(m_data))
#define SetParam(m_offset,m_data)	setParam((m_offset),(uint8_t*)&(m_data),sizeof(m_data))

//------------------------------------- Main Config -----------------------------------------------------

//#define CONTROL_FPS		500 // milos, commented out
#define CONTROL_PERIOD	2000 // milos, original 2000 (us), be careful since this defines ffb calculation rate (min is 1000us for max 1000Hz ffb calc rate, but 16MHz clock is not fast enough)
//#define SEND_PERIOD		4000 // milos, commented out
#define CONFIG_SERIAL_PERIOD 10000 // milos, original 50000 (us)

//------------------------------------- FFB/Firmware config -----------------------------------------------------

extern uint8_t LC_scaling;
extern uint8_t effstate;

struct fwOpt { // milos, added - firmware option stuct
  boolean a = false; // autocalibration (of analog axis)
  boolean b = false; // 2-ffb axis
  boolean c = false; // center button
  boolean d = false; // no optical encoder
  boolean e = false; // extra buttons
  boolean f = false; // xy shifter
  boolean g = false; // external dac (mcp4725)
  boolean h = false; // hat switch
  boolean i = false; // averaging (of analog axis)
  boolean l = false; // load cell
  boolean m = false; // proMicro pinouts
  boolean n = false; // shift register (nano button box)
  boolean p = false; // no EEPROM
  boolean r = false; // shift register (SN74ALS166N)
  boolean s = false; // exernal adc (ads1015 or ads1115)
  boolean t = false; // button matrix
  boolean u = false; // i2C multiplexer chip (tca9548), for 2nd as5600
  boolean w = false; // magnetic encoder (as5600)
  boolean x = false; // analog axis for ffb
  boolean z = false; // z-index
};

void update(fwOpt *option);


#ifdef USE_ANALOGFFBAXIS
byte indxFFBAxis(byte value);
#endif

extern uint8_t pwmstate; // =0b00000101; // milos, PWM settings configuration byte, bit7 is MSB

extern uint8_t configGeneralGain;
extern uint8_t configDamperGain;
extern uint8_t configFrictionGain;
extern uint8_t configConstantGain;
extern uint8_t configPeriodicGain;
extern uint8_t configSpringGain;
extern uint8_t configInertiaGain;
extern uint8_t configCenterGain;
extern uint8_t configStopGain;

// milos, here we set the PWM resolution and frequency per channel (old, now loaded from EEPROM)
// there are 2 PWM channels - one for each direction, so the actual FFB resolution is doubled
// Set 'TOP' for PWM resolution.  Assumes 16 MHz clock.
// fast PWM mode (for phase correct mode num of PWM steps is halved), number in brackets are index for PWMtop[] array (see pwm.ino and InitPWM() function)
// const unsigned int TOP = 0xFFFF; // 16-bit resolution,   244 Hz PWM (12)
// const unsigned int TOP = 0x9C40; // 40000 pwm steps,     400 Hz PWM (11)
// const unsigned int TOP = 0x7530; // 30000 pwm steps,     533 Hz PWM (10)
// const unsigned int TOP = 0x7FFF; // 15-bit resolution,   488 Hz PWM (9)
// const unsigned int TOP = 0x4E20; // 20000 pwm steps,     800 Hz PWM (8)
// const unsigned int TOP = 0x3FFF; // 14-bit resolution,   976 Hz PWM (7)
// const unsigned int TOP = 0x2710; // 10000 pwm steps,    1600 Hz PWM (6)
// const unsigned int TOP = 0x1FFF; // 13-bit resolution,  1953 Hz PWM (5)
// const unsigned int TOP = 0x0FFF; // 12-bit resolution,  3907 Hz PWM (4)
// const unsigned int TOP = 0x07FF; // 11-bit resolution,  7812 Hz PWM (3)
// const unsigned int TOP = 0x03FF; // 10-bit resolution, 15624 Hz PWM (2)
// const unsigned int TOP = 0x0320; // 800 pwm steps,     20000 Hz PWM (1)
// const unsigned int TOP = 0x0190; // 400 pwm steps,     40000 Hz PWM (0)
// milos, the downside of using fast PWM mode is that when you set PWM to 0, it is actualy not true zero, there is still a tiny 100ns pulse on both chanells at the same time apearing randomly
// this might be a trouble for some very powerful H-bridges if you use PWM+- mode, but on most it will be just fine
// in PWM+dir mode it does not matter since only one channel of PWM is used, dir pin is not on the same pin as the second PWM channel which is unused in dir mode

// milos, added - available TOP selection, defines PWM output resolution and frequency => affects output FFB fidelity, you want high TOP's
// I've set FFB steps to 32767 in HID descriptor, but games usualy have only up to 10000 FFB steps for force magnitude
// therefore you don't really need anything above PWMtops = 10000
// bare in mind that what you can feel in your hands will finaly depend on used motor and motor driver (good quality of digital audio signal will sound bad on shitty speakers, even with a good amp)
// drivers: AASD - great, brushless driver with FOC - good, BTS7960 - ok
// motors: brushless AC servo - great, brushless DC - good, brushed DC - ok

extern uint16_t PWMtops [13];

extern int16_t ROTATION_DEG; // milos
extern int32_t CPR; // milos
extern int32_t ROTATION_MAX; // milos
extern int32_t ROTATION_MID; // milos
extern uint16_t MM_MIN_MOTOR_TORQUE; // milos, loaded from EEPROM
extern uint16_t MM_MAX_MOTOR_TORQUE; // milos, loaded from EEPROM
extern uint16_t MAX_DAC; // milos, loaded from EEPROM
extern uint16_t TOP; // milos, pwmstate byte loaded from EEPROM, then in InitPWM() function calcTOP(pwmstate) defines TOP value

uint16_t calcTOP(byte b);

struct s32v { // milos, added - 2 dimensional vector structure (for ffb and position)
  int32_t x;
  int32_t avg;
};

extern float FFB_bal; // milos, FFB balance slider
extern float L_bal; // milos, left PWM balance multiplier
extern float R_bal; // milos, right PWM balance multiplier
extern float minTorquePP; // milos, added - min torque in percents

// milos, added - RCM pwm mode definitions
extern float RCM_min; // minimal RCM pulse width in ms
extern float RCM_zer; // zero RCM pulse width in ms
extern float RCM_max; // maximal RCM pulse width in ms

extern boolean zIndexFound;

float RCMscaler (byte value);


uint32_t decodeHat(uint32_t inbits);

struct xysh { // milos, added - holds shifter configuration
  uint16_t cal[5]; // calibration limits that define where the gears are
  // i  cal gears (if <=)
  // 0  x0  1
  // 1  x1  3
  // 2  x2  5
  // 3  y0  2,4,6,8, R
  // 4  y1  N
  uint8_t cfg;
  //bit0-revBtn invert,bit1-8 gears,bit2-X invert,bit3-Y invert,bit4-unused,bit5-unused,bit6-unused,bit7-unused
  //bit0=1: reverse gear button inverted (for logitech G25/G27/G29/G923 H-shifters)
  //bit1=1: 8 gear mode (r gear in 8th)
  //bit2=1: X-axis inverted
  //bit3=1: Y-axis inverted
  int16_t x; // horizontal position
  int16_t y; // vertical position
};

// milos - added, function for decoding XY shifter analog values into last 8 buttons
uint32_t decodeXYshifter (uint32_t inbits, xysh *s);

struct s16a { // milos, added - holds individual 16bit axis properties
  int16_t val;
  int16_t min;
  int16_t max;
  int16_t avg;
};

struct s32a { // milos, added - holds individual bit axis properties
  int32_t val; // milos, when using load cell we can have more than 16bit range for brake axis
  int16_t min; // milos, these are used for manual/autocalib so we can keep them 16bit as analog axis are 10bit only
  int16_t max; // milos, when we use load cell min/max are unused for brake axis
  int16_t avg;
};

const uint8_t avgSamples = 4; // milos, added - number of samples for averaging of arduino analog inputs
// milos, default axis calibration values depend on usage of averaging or external ADC
#ifdef AVG_INPUTS
const uint16_t maxCal = 4095;
#else // if no avg inputs
const uint16_t maxCal = 1023;
#endif // end of avg inputs

void SetEEPROMConfig();
void LoadEEPROMConfig();
void SaveEEPROMConfig ();