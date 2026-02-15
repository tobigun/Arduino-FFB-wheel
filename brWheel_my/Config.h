#pragma once

#include <Arduino.h>
#include "ffb_hid.h"

//------------------------------------- Firmware options -------------------------------------------------

#define USE_HATSWITCH        //  uncomment to use first 4 buttons for hat switch (D-pad)
#define USE_BTNMATRIX        //  uncomment to use 8 pins as a 4x4 button matrix for total of 16 buttons (can not be used with load cell, shift register or XY shifter)
#define USE_ANALOGFFBAXIS //  uncomment to enable other than X-axis to be tied with xFFB axis (you can use analog inputs instead of digital encoders
#define USE_ANALOG_XAXIS // use dedicated analog X-Axis

#define USE_PWM_0_50_100_MODE
//#define USE_PWM_DIR_MODE
//#define USE_PWM_PLUS_MINUS_MODE

#define MAX_TORQ_BITS 11L
#define MAX_TORQ_DEFAULT ((1 << MAX_TORQ_BITS) - 1) // 2047

//------------------------------------- Pins -------------------------------------------------------------

//#define LED_GREEN_PIN 1 // Green LED directly connected to 12V supply
#ifdef __AVR__
#define LED_BLUE_PIN 0
#define LED_RED_PIN 3
#else
#define LED_BLUE_PIN 41
#define LED_RED_PIN 42
#endif
#define FFBCLIP_LED_PIN LED_RED_PIN // for ProMicro if no above i2C devices

#ifdef __AVR__
#define X_AXIS_PIN		A0
#define Y_AXIS_PIN    A1
#define Z_AXIS_PIN		A2
#define RX_AXIS_PIN		A3
#define RY_AXIS_PIN   A8
#else
#define X_AXIS_PIN		A0
#define Y_AXIS_PIN    A1
#define Z_AXIS_PIN		A3
#define RX_AXIS_PIN		A4
#define RY_AXIS_PIN   A5
#endif

#ifdef __AVR__
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
#else
#define BUTTON0 38
#define BUTTON1 37
#define BUTTON2 36
#define BUTTON3 35
#define BUTTON4 34
#define BUTTON5 33
#define BUTTON6 18
#define BUTTON7 17
#endif

#define PROFILE_SWITCH_PIN 16

#ifdef __AVR__
#define PWM_PIN_L     9 // milos, left PWM pin
#if defined(USE_PWM_0_50_100_MODE) || defined(USE_PWM_DIR_MODE)
#define DIR_PIN       10
#else
#define PWM_PIN_R     10 // milos, right PWM pin
#endif
#else
#define PWM_PIN_L     40 // left PWM pin
#if defined(USE_PWM_0_50_100_MODE) || defined(USE_PWM_DIR_MODE)
#define DIR_PIN       39
#else
#define PWM_PIN_R     39 // right PWM pin
#endif
#endif

//------------------------------------- EEPROM Config -----------------------------------------------------

#define PARAM_ADDR_FW_VERSION		 0x00 // firmware version
#define PARAM_ADDR_ENC_OFFSET    0x02 // Z-index offset
#define PARAM_ADDR_ROTATION_DEG  0x06 // rotation degrees
#define PARAM_ADDR_GEN_GAIN      0x08 // general
#define PARAM_ADDR_DMP_GAIN      0x09 // damper
#define PARAM_ADDR_FRC_GAIN      0x0A // friction
#define PARAM_ADDR_CNT_GAIN      0x0B // constant
#define PARAM_ADDR_PER_GAIN      0x0C // periodic
#define PARAM_ADDR_SPR_GAIN      0x0D // spring
#define PARAM_ADDR_INR_GAIN      0x0E // inertia
#define PARAM_ADDR_CTS_GAIN      0x0F // centering spring
#define PARAM_ADDR_STP_GAIN      0x10 // end stop
#define PARAM_ADDR_MIN_TORQ      0x11 // min torque
#define PARAM_ADDR_MAX_TORQ      0x13 // max torque
#define PARAM_ADDR_MAX_DAC       0x15 // max DAC value
#define PARAM_ADDR_BRK_PRES      0x17 // max brake pressure
#define PARAM_ADDR_DSK_EFFC      0x18 // desktop effects (byte contents is in effstate)
#define PARAM_ADDR_ENC_CPR       0x19 // encoder CPR
#define PARAM_ADDR_PWM_SET       0x1D // PWM settings and frequency (byte contents is in pwmstate)
#define PARAM_ADDR_SHFT_X0       0x1E // XY shifter limit x0
#define PARAM_ADDR_SHFT_X1       0x20 // XY shifter limit x1
#define PARAM_ADDR_SHFT_X2       0x22 // XY shifter limit x2
#define PARAM_ADDR_SHFT_Y0       0x24 // XY shifter limit y0
#define PARAM_ADDR_SHFT_Y1       0x26 // XY shifter limit y1
#define PARAM_ADDR_SHFT_CFG      0x28 // shifter configuration byte
#define PARAM_ADDR_ACEL_LO       0x2A // accelerator pedal cal min
#define PARAM_ADDR_ACEL_HI       0x2C // accelerator pedal cal max
#define PARAM_ADDR_BRAK_LO       0x2E // brake pedal cal min
#define PARAM_ADDR_BRAK_HI       0x30 // brake pedal cal max
#define PARAM_ADDR_CLUT_LO       0x32 // clutch pedal cal min
#define PARAM_ADDR_CLUT_HI       0x34 // clutch pedal cal max
#define PARAM_ADDR_HBRK_LO       0x36 // hand brake pedal cal min
#define PARAM_ADDR_HBRK_HI       0x38 // hand brake pedal cal max

#define EEPROM_SIZE              1024 // same as used for Arduino Pro Micro

#define FIRMWARE_VERSION         0x01 //  firmware version

void setParam (uint16_t offset, uint8_t *addr_to, uint8_t size);

#define GetParam(m_offset,m_data)	getParam((m_offset),(uint8_t*)&(m_data),sizeof(m_data))
#define SetParam(m_offset,m_data)	setParam((m_offset),(uint8_t*)&(m_data),sizeof(m_data))

//------------------------------------- Main Config -----------------------------------------------------

//#define CONTROL_FPS		500 //  commented out
#define CONTROL_PERIOD	2000 //  original 2000 (us), be careful since this defines ffb calculation rate (min is 1000us for max 1000Hz ffb calc rate, but 16MHz clock is not fast enough)
//#define SEND_PERIOD		4000 //  commented out
#define CONFIG_SERIAL_PERIOD 10000 //  original 50000 (us)

//------------------------------------- FFB/Firmware config -----------------------------------------------------

extern uint8_t ffbBalance;
extern uint8_t effstate;


extern uint8_t pwmstate; // =0b00000101; //  PWM settings configuration byte, bit7 is MSB

extern uint8_t configGeneralGain;
extern uint8_t configDamperGain;
extern uint8_t configFrictionGain;
extern uint8_t configConstantGain;
extern uint8_t configPeriodicGain;
extern uint8_t configSpringGain;
extern uint8_t configInertiaGain;
extern uint8_t configCenterGain;
extern uint8_t configStopGain;

//  here we set the PWM resolution and frequency per channel (old, now loaded from EEPROM)
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
//  the downside of using fast PWM mode is that when you set PWM to 0, it is actualy not true zero, there is still a tiny 100ns pulse on both chanells at the same time apearing randomly
// this might be a trouble for some very powerful H-bridges if you use PWM+- mode, but on most it will be just fine
// in PWM+dir mode it does not matter since only one channel of PWM is used, dir pin is not on the same pin as the second PWM channel which is unused in dir mode

//  added - available TOP selection, defines PWM output resolution and frequency => affects output FFB fidelity, you want high TOP's
// I've set FFB steps to 32767 in HID descriptor, but games usualy have only up to 10000 FFB steps for force magnitude
// therefore you don't really need anything above PWMtops = 10000
// bare in mind that what you can feel in your hands will finaly depend on used motor and motor driver (good quality of digital audio signal will sound bad on shitty speakers, even with a good amp)
// drivers: AASD - great, brushless driver with FOC - good, BTS7960 - ok
// motors: brushless AC servo - great, brushless DC - good, brushed DC - ok

extern uint16_t PWMtops [13];

extern int16_t ROTATION_DEG; // milos
extern int32_t CPR; // unused

extern uint16_t MM_MIN_MOTOR_TORQUE; //  loaded from EEPROM
extern uint16_t MM_MAX_MOTOR_TORQUE; //  loaded from EEPROM
extern uint16_t MAX_DAC; //  loaded from EEPROM

uint16_t calcTOP(byte b);

struct s32v { //  added - 2 dimensional vector structure (for ffb and position)
  int32_t x;
};

uint32_t decodeHat(uint32_t inbits);

struct s16a { //  added - holds individual 16bit axis properties
  int16_t val;
  int16_t min;
  int16_t max;
};

struct s32a { //  added - holds individual bit axis properties
  int32_t val; //  when using load cell we can have more than 16bit range for brake axis
  int16_t min; //  these are used for manual/autocalib so we can keep them 16bit as analog axis are 10bit only
  int16_t max; //  when we use load cell min/max are unused for brake axis
};

void InitEEPROMConfig();
void SetEEPROMConfig();
void LoadEEPROMConfig();
void SaveEEPROMConfig ();