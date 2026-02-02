// milos, completely rewritten - all possible configurations of PWM and DAC settings are handeled here

#include <avr/io.h>
#include "Config.h"
#include <digitalWriteFast.h>

inline void PWM16A(uint16_t PWMValue);
void PWM16Begin();
void PWM16EnableA();
void blinkFFBclipLED();

void InitPWM() {
#ifdef DIR_PIN
  pinModeFast(DIR_PIN, OUTPUT);
#endif
  TOP = calcTOP(pwmstate); // milos, this will set appropriate TOP value for all PWM modes, depending on pwmstate loaded from EEPROM
  MM_MAX_MOTOR_TORQUE = TOP;
  minTorquePP = ((float)MM_MIN_MOTOR_TORQUE) / ((float)MM_MAX_MOTOR_TORQUE); // milos
  RCM_min *= RCMscaler(pwmstate); // milos - takes into account fast pwm or phase correct mode
  RCM_zer *= RCMscaler(pwmstate); // milos
  RCM_max *= RCMscaler(pwmstate); // milos

  PWM16Begin(); // Timer1 and Timer3 configuration: frequency and mode depend on pwmstate byte
  PWM16A(0);  // Set initial PWM value for Pin 9
  PWM16EnableA();  // Turn PWM on for Pin 9
#ifdef PWM_PIN_R
  PWM16B(0);  // Set initial PWM value for Pin 10
  PWM16EnableB();  // Turn PWM on for Pin 10
#endif

  pinMode(FFBCLIP_LED_PIN, OUTPUT); // milos, for promicro we can only use ffb clip led on D3 if not using all above
  blinkFFBclipLED(); // milos, signals end of configuration
}

void blinkFFBclipLED() { // milos, added - blink FFB clip LED a few times at startup to indicate succesful boot
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(FFBCLIP_LED_PIN, HIGH);
    delay(20);
    digitalWrite(FFBCLIP_LED_PIN, LOW);
    delay(20);
  }
}

void activateFFBclipLED(int32_t t) {  // milos, added - turn on FFB clip LED if max FFB signal reached (shows 90-99% of FFB signal as linear increase from 0 to 1/4 of full brightness)
  float level = 0.01 * configGeneralGain;
  if (abs(t) >= 0.9 * MM_MAX_MOTOR_TORQUE * level && abs(t) < level * MM_MAX_MOTOR_TORQUE - 1) {
    analogWrite(FFBCLIP_LED_PIN, map(abs(t), 0.9 * MM_MAX_MOTOR_TORQUE * level, level * MM_MAX_MOTOR_TORQUE, 1, 63)); // for 90%-99% ffb map brightness linearly from 1-63 (out of 255)
  } else if (abs(t) >= level * MM_MAX_MOTOR_TORQUE - 1) {
    digitalWrite(FFBCLIP_LED_PIN, HIGH); // for 100% FFB set full brightness
  } else {
    digitalWrite(FFBCLIP_LED_PIN, LOW); // if under 90% FFB turn off LED
  }
}

//void SetPWM (int32_t torque)  { //torque between -MM_MAX_MOTOR and +MM_MAX_MOTOR // milos, torque is xFFB, while yFFB is passed from torqueY global variable outside of this function
void SetPWM (s32v *torque) { // milos, takes pointer struct as argument - 2 axis FFB data
  if (torque != NULL) { // milos, this check is always required for pointers
    activateFFBclipLED(torque->x); // milos, for promicro we can only use ffb clip led on D3 if not using all above

    FFB_bal = (float)(LC_scaling - 128) / 255.0; // milos, max value is 0.5
    if (FFB_bal >= 0) {
      L_bal = 1.0 - FFB_bal;
      R_bal = 1.0;
    } else {
      L_bal = 1.0;
      R_bal = 1.0 + FFB_bal;
    }

#if defined(USE_PWM_PLUS_MINUS_MODE) // PWM+- mode
      if (torque->x > 0) {
        torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, R_bal * MM_MAX_MOTOR_TORQUE);
        PWM16A(torque->x);
        PWM16B(0);
        #ifdef DIR_PIN
        digitalWriteFast(DIR_PIN, HIGH); //use dir pin as BTS7960 pwm motor enable signal
        #endif
      } else if (torque->x < 0) {
        torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, L_bal * MM_MAX_MOTOR_TORQUE);
        PWM16A(0);
        PWM16B(torque->x);
        #ifdef DIR_PIN
        digitalWriteFast(DIR_PIN, HIGH);
        #endif
      } else {
        PWM16A(0);
        PWM16B(0);
        #ifdef DIR_PIN
        digitalWriteFast(DIR_PIN, LOW); // disable bts output when no pwm signal to make it rotate freely
        #endif
      }
#elif defined(USE_PWM_0_50_100_MODE) // PWM0.50.100 mode
        if (torque->x > 0) {
          torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
          PWM16A(torque->x);
        } else if (torque->x < 0) {
          torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 - MM_MIN_MOTOR_TORQUE, 0);
          PWM16A(torque->x);
        } else {
          PWM16A(MM_MAX_MOTOR_TORQUE / 2);
        }
        digitalWriteFast(DIR_PIN, HIGH); // enable motor
#elif defined(USE_PWM_DIR_MODE) // PWM+dir mode
      if (torque->x >= 0) {
        digitalWriteFast(DIR_PIN, HIGH);
      } else {
        digitalWriteFast(DIR_PIN, LOW);
      }
      torque->x = map(abs(torque->x), 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
      PWM16A(torque->x);
#elif defined(USE_PWM_RCM_MODE) // RCM mode
      if (torque->x > 0) {
        torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 + minTorquePP), RCM_max);
        PWM16A(torque->x);
      } else if (torque->x < 0) {
        torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 - minTorquePP), RCM_min);
        PWM16A(torque->x);
      } else {
        PWM16A(RCM_zer);
      }
    }
    PWM16B(RCM_zer);
    digitalWriteFast(PWM_PIN_R, LOW);
#else
  #error "No PWM Mode selected"
#endif
  }
}

void PWM16Begin() { // milos - added, reconfigure timer1 automatically depending on pwm settings
  // Stop Timer/Counter1
  TCCR1A = 0; // Timer/Counter1 Control Register A
  TCCR1B = 0; // Timer/Counter1 Control Register B
  TIMSK1 = 0; // Timer/Counter1 Interrupt Mask Register
  TIFR1 = 0;  // Timer/Counter1 Interrupt Flag Register
  ICR1 = TOP; // milos, set upper counter flag
  OCR1A = 0;  // Default to 0% PWM, D9
  OCR1B = 0;  // Default to 0% PWM, D10

  if (bitRead(pwmstate, 1) && bitRead(pwmstate, 6)) { // if RMC pwm mode
    TCCR1B |= (1 << CS11); // milos, Set clock prescaler to 8
  } else { // for all other pwm modes
    TCCR1B |= (1 << CS10); // Set clock prescaler to 1 for maximum PWM frequency
  }
  if (bitRead(pwmstate, 0)) { // if pwmstate bit0=1, configure timer for phase correct mode
    // milos, Set Timer/Counter1 to Waveform Generation Mode 10: Phase and Frequency Correct PWM with TOP set by ICR1
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13);
  } else {  // if pwmstate bit0=0, fast pwm mode
    // milos, Set Timer/Counter1 to Waveform Generation Mode 14: Fast PWM with TOP set by ICR1
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
  }
}

void PWM16EnableA() {  // milos
  // Enable Fast PWM on Pin 9: Set OC1A at BOTTOM and clear OC1A on OCR1A compare
  TCCR1A |= (1 << COM1A1);
  pinModeFast(PWM_PIN_L, OUTPUT);
}

#ifdef PWM_PIN_R
void PWM16EnableB() {  // milos
  // Enable Fast PWM on Pin 10: Set OC1B at BOTTOM and clear OC1B on OCR1B compare
  TCCR1A |= (1 << COM1B1);
  pinModeFast(PWM_PIN_R, OUTPUT);
}
#endif

inline void PWM16A(uint16_t PWMValue) { // milos
  OCR1A = constrain(PWMValue, 0, TOP);
}

inline void PWM16B(uint16_t PWMValue) { // milos
  OCR1B = constrain(PWMValue, 0, TOP);
}
