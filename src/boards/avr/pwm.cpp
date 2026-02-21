#include <Arduino.h>
#include "config.h"
#include "common.h"

#include <avr/io.h>
#include <digitalWriteFast.h>

static uint16_t TOP;

inline void PWM16A(uint16_t PWMValue);
void PWM16Begin();
void PWM16EnableA();

void initPWM() {
#ifdef DIR_PIN
  pinModeFast(DIR_PIN, OUTPUT);
#endif

  PWM16Begin(); // Timer1 and Timer3 configuration: frequency and mode depend on pwmstate byte
  PWM16A(0);  // Set initial PWM value for Pin 9
  PWM16EnableA();  // Turn PWM on for Pin 9
#ifdef PWM_PIN_R
  PWM16B(0);  // Set initial PWM value for Pin 10
  PWM16EnableB();  // Turn PWM on for Pin 10
#endif
}

//torque between -MM_MAX_MOTOR and +MM_MAX_MOTOR
void setPWM(int32_t torqueX) {
#if defined(USE_PWM_PLUS_MINUS_MODE) // PWM+- mode
  float L_bal; // left PWM balance multiplier
  float R_bal; // right PWM balance multiplier
  float FFB_bal = (float)(ffbBalance - 128) / 255.0; // max value is 0.5
  if (FFB_bal >= 0) {
    L_bal = 1.0 - FFB_bal;
    R_bal = 1.0;
  } else {
    L_bal = 1.0;
    R_bal = 1.0 + FFB_bal;
  }
  if (torqueX > 0) {
    torqueX = map(torqueX, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, R_bal * MM_MAX_MOTOR_TORQUE);
    PWM16A(torqueX);
    PWM16B(0);
    #ifdef DIR_PIN
    digitalWriteFast(DIR_PIN, HIGH); //use dir pin as BTS7960 pwm motor enable signal
    #endif
  } else if (torqueX < 0) {
    torqueX = map(-torqueX, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, L_bal * MM_MAX_MOTOR_TORQUE);
    PWM16A(0);
    PWM16B(torqueX);
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
  if (torqueX > 0) {
    torqueX = map(torqueX, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
    PWM16A(torqueX);
  } else if (torqueX < 0) {
    torqueX = map(-torqueX, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 - MM_MIN_MOTOR_TORQUE, 0);
    PWM16A(torqueX);
  } else {
    PWM16A(MM_MAX_MOTOR_TORQUE / 2);
  }
  digitalWriteFast(DIR_PIN, HIGH); // enable motor
#elif defined(USE_PWM_DIR_MODE) // PWM+dir mode
  if (torqueX >= 0) {
    digitalWriteFast(DIR_PIN, HIGH);
  } else {
    digitalWriteFast(DIR_PIN, LOW);
  }
  torqueX = map(abs(torqueX), 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
  PWM16A(torqueX);
#else
  #error "No PWM Mode selected"
#endif
}

void PWM16Begin() {  - added, reconfigure timer1 automatically depending on pwm settings
  // Stop Timer/Counter1
  TCCR1A = 0; // Timer/Counter1 Control Register A
  TCCR1B = 0; // Timer/Counter1 Control Register B
  TIMSK1 = 0; // Timer/Counter1 Interrupt Mask Register
  TIFR1 = 0;  // Timer/Counter1 Interrupt Flag Register
  ICR1 = TOP; // set upper counter flag
  OCR1A = 0;  // Default to 0% PWM, D9
  OCR1B = 0;  // Default to 0% PWM, D10

  if (bitRead(pwmstate, 1) && bitRead(pwmstate, 6)) { // if RMC pwm mode
    TCCR1B |= (1 << CS11); // Set clock prescaler to 8
  } else { // for all other pwm modes
    TCCR1B |= (1 << CS10); // Set clock prescaler to 1 for maximum PWM frequency
  }
  if (bitRead(pwmstate, 0)) { // if pwmstate bit0=1, configure timer for phase correct mode
    // Set Timer/Counter1 to Waveform Generation Mode 10: Phase and Frequency Correct PWM with TOP set by ICR1
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13);
  } else {  // if pwmstate bit0=0, fast pwm mode
    // Set Timer/Counter1 to Waveform Generation Mode 14: Fast PWM with TOP set by ICR1
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
  }
}

void PWM16EnableA() {
  // Enable Fast PWM on Pin 9: Set OC1A at BOTTOM and clear OC1A on OCR1A compare
  TCCR1A |= (1 << COM1A1);
  pinModeFast(PWM_PIN_L, OUTPUT);
}

#ifdef PWM_PIN_R
void PWM16EnableB() {
  // Enable Fast PWM on Pin 10: Set OC1B at BOTTOM and clear OC1B on OCR1B compare
  TCCR1A |= (1 << COM1B1);
  pinModeFast(PWM_PIN_R, OUTPUT);
}
#endif

inline void PWM16A(uint16_t PWMValue) {
  OCR1A = constrain(PWMValue, 0, TOP);
}

inline void PWM16B(uint16_t PWMValue) {
  OCR1B = constrain(PWMValue, 0, TOP);
}
