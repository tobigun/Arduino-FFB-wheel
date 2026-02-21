#include <Arduino.h>

#include "config.h"
#include "common.h"
#ifdef __AVR__
#include <avr/io.h>
#include <digitalWriteFast.h>
#endif

void initPWM() {
#ifdef DIR_PIN
  pinMode(DIR_PIN, OUTPUT);
#endif

  pinMode(PWM_PIN_L, OUTPUT);
  analogWriteFrequency(PWM_PIN_L, 19569); // ~20kHz is the highest frequency with MAX_TORQ_BITS (here: 11 bits) resolution (see ESP32S3 AnalogOut/ledcFrequency example)
  analogWriteResolution(PWM_PIN_L, MOTOR_PWM_BITS);
}

// torque between -MM_MAX_MOTOR and +MM_MAX_MOTOR
void setPWM(int32_t torqueX) {
#if defined(USE_PWM_0_50_100_MODE) // PWM0.50.100 mode
  if (torqueX > 0) {
    uint16_t pwm = map(torqueX, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
    analogWrite(PWM_PIN_L, pwm);
  } else if (torqueX < 0) {
    uint16_t pwm = map(-torqueX, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 - MM_MIN_MOTOR_TORQUE, 0);
    analogWrite(PWM_PIN_L, pwm);
  } else {
    analogWrite(PWM_PIN_L, MM_MAX_MOTOR_TORQUE / 2);
  }
  digitalWrite(DIR_PIN, HIGH); // enable motor
#else
  #error "No PWM Mode selected"
#endif
}
