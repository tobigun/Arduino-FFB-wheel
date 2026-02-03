#include "Config.h"
#include "debug.h"
#include "common.h"

//--------------------------------------------------------------------------------------------------------

uint8_t toUpper(uint8_t c) {
  if ((c >= 'a') && (c <= 'z'))
    return (c + 'A' - 'a');
  return (c);
}

void configCDC() { // milos, virtual serial port firmware configuration interface
  if (CONFIG_SERIAL.available() > 0) {
    uint8_t c = toUpper(CONFIG_SERIAL.read());
    //DEBUG_SERIAL.println(c);
    int32_t temp, temp1;
    float wheelAngle;
    uint8_t ffb_temp;
    switch (c) {
      case 'U': // milos, send all firmware settings
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
        CONFIG_SERIAL.print(LC_scaling);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(effstate, DEC); //milos, desktop effects in decimal form
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(MM_MAX_MOTOR_TORQUE); //milos, send max torque as parameter
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(CPR); //milos, send CPR as parameter
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.println(pwmstate, DEC); //milos, send pwmstate byte in decimal form
        break;
      case 'V':
        CONFIG_SERIAL.print("fw-v");
        CONFIG_SERIAL.print(FIRMWARE_VERSION, DEC);
        // milos, firmware options
#ifndef USE_QUADRATURE_ENCODER
        CONFIG_SERIAL.print("d"); // milos, if no optical encoder
#endif
#ifdef USE_HATSWITCH
        CONFIG_SERIAL.print("h");
#endif
#ifdef USE_BTNMATRIX
        CONFIG_SERIAL.print("t");
#endif
#ifdef USE_ANALOGFFBAXIS
        CONFIG_SERIAL.print("x");
#endif
#ifdef USE_PROMICRO
        CONFIG_SERIAL.print("m");
#endif
        CONFIG_SERIAL.print("\r\n");
        break;
      case 'S':
        CONFIG_SERIAL.println(brWheelFFB.state, DEC);
        break;
      case 'R':
        brWheelFFB.calibrate();
        break;
      case 'B': // milos, added to adjust brake load cell pressure
        ffb_temp = CONFIG_SERIAL.parseInt();
        LC_scaling = constrain(ffb_temp, 1, 255);
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_BRK_PRES, LC_scaling); // milos, update EEPROM
        break;
      case 'P': // milos, added to recalibrate pedals
        CONFIG_SERIAL.println(0);
        break;
      case 'O': // milos, added to adjust optical encoder CPR
        //temp1 = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; // milos
        temp1 = 0;
        temp = CONFIG_SERIAL.parseInt();
        temp = constrain(temp, 4, 600000); // milos, extended to 32bit (100000*6)
        wheelAngle = float(temp1) * float(ROTATION_DEG) / float(ROTATION_MAX); // milos, current wheel angle
        CPR = temp; // milos, update CPR
        ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos, updated
        ROTATION_MID = ROTATION_MAX >> 1; // milos, updated, divide by 2
        temp1 = int32_t(wheelAngle * float(ROTATION_MAX) / float(ROTATION_DEG)); // milos, here we recover the old wheel angle
        //myEnc.Write(temp1 + ROTATION_MID - brWheelFFB.offset); // milos
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_ENC_CPR, CPR); // milos, update EEPROM
        break;
      case 'C':
        //myEnc.Write(ROTATION_MID); // milos, just set to zero angle
        CONFIG_SERIAL.println(0);
        break;
      case 'Z': // milos, hard reset the z-index offset
        CONFIG_SERIAL.println(0);
        break;
      case 'G': // milos, set new rotation angle
        //temp1 = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; // milos
        temp1 = 0;
        temp = CONFIG_SERIAL.parseInt();
        temp = constrain(temp, 30, 1800); // milos
        wheelAngle = float(temp1) * float(ROTATION_DEG) / float(ROTATION_MAX); // milos, current wheel angle
        ROTATION_DEG = temp; // milos, update degrees of rotation
        ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos, updated
        ROTATION_MID = ROTATION_MAX >> 1; // milos, updated, divide by 2
        temp1 = int32_t(wheelAngle * float(ROTATION_MAX) / float(ROTATION_DEG)); // milos, here we recover the old wheel angle
        //myEnc.Write(temp1 + ROTATION_MID - brWheelFFB.offset); // milos
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_ROTATION_DEG, temp);// milos, update EEPROM
        break;
      case 'E': //milos, added - turn desktop effects and ffb monitor on/off
        ffb_temp = CONFIG_SERIAL.parseInt();
        ffb_temp = constrain(ffb_temp, 0, 255);
        for (uint8_t i = 0; i < 8; i++) { //milos, decode incomming number into individual bits
          bitWrite(effstate, i, bitRead(ffb_temp, i));
        }
        CONFIG_SERIAL.println(effstate, BIN);
        //CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_DSK_EFFC, effstate); // milos, update EEPROM
        break;
      case 'W': { //milos, added - configure PWM settings and frequency
        ffb_temp = CONFIG_SERIAL.parseInt();
        ffb_temp = constrain(ffb_temp, 0, 255);
        for (uint8_t i = 0; i < 8; i++) { // milos, decode incomming number into individual bits
          bitWrite(pwmstate, i, bitRead(ffb_temp, i));
        }
        SetParam(PARAM_ADDR_PWM_SET, pwmstate); // milos, update EEPROM with new pwm settings
        float minTorquePP = ((float)MM_MIN_MOTOR_TORQUE) / ((float)MM_MAX_MOTOR_TORQUE);
        temp = calcTOP(pwmstate) * minTorquePP; // milos, recalculate new min torque for curent min torque %
        SetParam(PARAM_ADDR_MIN_TORQ, temp); // milos, update min torque in EEPROM
        CONFIG_SERIAL.println(calcTOP(pwmstate));
        break;
      }  
      case 'H': // milos, added - configure the XY shifter calibration
        CONFIG_SERIAL.println(0);
        break;
      case 'Y': // milos, added - configure manual calibration for pedals
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
      case 'A': //milos, save all firmware settings in EEPROM
        SaveEEPROMConfig ();
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
            float minTorquePP = (float)ffb_temp * 0.001; // milos, max is 25.5% or 0.255
            MM_MIN_MOTOR_TORQUE = (u16)(minTorquePP * (float)MM_MAX_MOTOR_TORQUE); // milos, we can set it during run time
            CONFIG_SERIAL.println(1);
            break;
          }
        }
        break;
    }
  }
}
