#include <Arduino.h>
#include "config.h"
#include "common.h"

//--------------------------------------------------------------------------------------------------------

uint8_t toUpper(uint8_t c) {
  if ((c >= 'a') && (c <= 'z'))
    return (c + 'A' - 'a');
  return (c);
}

void configCDC() { // virtual serial port firmware configuration interface
  if (CONFIG_SERIAL.available() > 0) {
    uint8_t c = toUpper(CONFIG_SERIAL.read());
    int32_t temp;
    uint8_t ffb_temp;
    switch (c) {
      case 'U': // send all firmware settings
        CONFIG_SERIAL.print(ROTATION_DEG);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configGeneralGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configDamperGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configFrictionGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configConstantGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configPeriodicGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configSpringGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configInertiaGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configCenterGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configStopGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(MM_MIN_MOTOR_TORQUE);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(ffbBalance);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(effstate, DEC); //desktop effects in decimal form
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(MM_MAX_MOTOR_TORQUE); //send max torque as parameter
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(CPR); //send CPR as parameter
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.println(pwmstate, DEC); //send pwmstate byte in decimal form
        break;
      case 'V':
        CONFIG_SERIAL.print("fw-v");
        CONFIG_SERIAL.print(FIRMWARE_VERSION, DEC);
        // firmware options
        //CONFIG_SERIAL.print("d"); // no optical encoder (pretend that we are an optical encoder to be able to calibrate Z-Axis) 
        CONFIG_SERIAL.print("h"); // USE_HATSWITCH
        CONFIG_SERIAL.print("t"); // USE_BTNMATRIX
        CONFIG_SERIAL.print("m"); // promicro
        CONFIG_SERIAL.print("\r\n");
        break;
      case 'S':
        // unused
        break;
      case 'R':
        // not useful for potentiometer based wheel
        break;
      case 'B': // added to adjust balance
        ffb_temp = CONFIG_SERIAL.parseInt();
        ffbBalance = constrain(ffb_temp, 1, 255);
        CONFIG_SERIAL.println(1);
        break;
      case 'P': // added to recalibrate pedals
        // unused
        CONFIG_SERIAL.println(0);
        break;
      case 'O': // added to adjust optical encoder CPR
        // unused
        CONFIG_SERIAL.println(0);
        break;
      case 'C': // set to zero angle
        //myEnc.Write(ROTATION_MID);
        CONFIG_SERIAL.println(0);
        break;
      case 'Z': // hard reset the z-index offset
        CONFIG_SERIAL.println(0);
        break;
      case 'G': // set new rotation angle
        temp = CONFIG_SERIAL.parseInt();
        ROTATION_DEG = constrain(temp, 30, 1800); // update degrees of rotation
        CONFIG_SERIAL.println(1);
        break;
      case 'E': //added - turn desktop effects and ffb monitor on/off
        ffb_temp = CONFIG_SERIAL.parseInt();
        ffb_temp = constrain(ffb_temp, 0, 255);
        for (uint8_t i = 0; i < 8; i++) { //decode incomming number into individual bits
          bitWrite(effstate, i, bitRead(ffb_temp, i));
        }
        CONFIG_SERIAL.println(effstate, BIN);
        break;
      case 'W': { //added - configure PWM settings and frequency
        ffb_temp = CONFIG_SERIAL.parseInt();
        ffb_temp = constrain(ffb_temp, 0, 255);
        for (uint8_t i = 0; i < 8; i++) { // decode incomming number into individual bits
          bitWrite(pwmstate, i, bitRead(ffb_temp, i));
        }
        SetParam(PARAM_ADDR_PWM_SET, pwmstate); // update EEPROM with new pwm settings
        float minTorquePP = ((float)MM_MIN_MOTOR_TORQUE) / ((float)MM_MAX_MOTOR_TORQUE);
        temp = calcTOP(pwmstate) * minTorquePP; // recalculate new min torque for curent min torque %
        SetParam(PARAM_ADDR_MIN_TORQ, temp); // update min torque in EEPROM
        CONFIG_SERIAL.println(calcTOP(pwmstate));
        break;
      }  
      case 'H': // added - configure the XY shifter calibration
        // unused
        CONFIG_SERIAL.println(0);
        break;
      case 'Y': // added - configure manual calibration for pedals
        c = toUpper(CONFIG_SERIAL.read());
        switch (c) {
          case 'A':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            brake.min = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'B':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            brake.max = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'C':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            accel.min = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'D':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            accel.max = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'E':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            clutch.min = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'F':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            clutch.max = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'G':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            hbrake.min = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'H':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            hbrake.max = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'R':
            CONFIG_SERIAL.print(brake.min);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(brake.max);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(accel.min);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(accel.max);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(clutch.min);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(clutch.max);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(hbrake.min);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.println(hbrake.max);
            break;
        }
        break;
      case 'A': //save all firmware settings in EEPROM
        saveEEPROMConfig ();
        CONFIG_SERIAL.println(1);
        break;
      case 'F':
        c = toUpper(CONFIG_SERIAL.read());
        switch (c) {
          case 'G':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configGeneralGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'C':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configConstantGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'D':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configDamperGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'F':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configFrictionGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'S':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configPeriodicGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'M':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configSpringGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'I':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configInertiaGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'A':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configCenterGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'B':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configStopGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'J': {
            ffb_temp = CONFIG_SERIAL.parseInt();
            ffb_temp = constrain(ffb_temp, 0, 255); //milos
            float minTorquePP = (float)ffb_temp * 0.001; // max is 25.5% or 0.255
            MM_MIN_MOTOR_TORQUE = (uint16_t)(minTorquePP * (float)MM_MAX_MOTOR_TORQUE); // we can set it during run time
            CONFIG_SERIAL.println(1);
            break;
          }
        }
        break;
    }
  }
}
