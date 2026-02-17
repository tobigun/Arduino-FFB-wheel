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
  analogWriteResolution(PWM_PIN_L, MAX_TORQ_BITS);
}

//void setPWM (int32_t torque)  { //torque between -MM_MAX_MOTOR and +MM_MAX_MOTOR // torque is xFFB, while yFFB is passed from torqueY global variable outside of this function
void setPWM (s32v *torque) { // takes pointer struct as argument - 2 axis FFB data
  if (torque == NULL) { // this check is always required for pointers
    return;
  }

  activateFFBclipLED(torque->x); // for promicro we can only use ffb clip led on D3 if not using all above

#if defined(USE_PWM_0_50_100_MODE) // PWM0.50.100 mode
  if (torque->x > 0) {
    torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
    analogWrite(PWM_PIN_L, torque->x);
  } else if (torque->x < 0) {
    torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 - MM_MIN_MOTOR_TORQUE, 0);
    analogWrite(PWM_PIN_L, torque->x);
  } else {
    analogWrite(PWM_PIN_L, MM_MAX_MOTOR_TORQUE / 2);
  }
  digitalWrite(DIR_PIN, HIGH); // enable motor
#else
  #error "No PWM Mode selected"
#endif
}
