#include "ConfigHID.h"

void configHID(USB_ConfigReport *data) {
#ifdef USE_CONFIGHID
  if (data->Info == 1) { // milos, update firmware settings from an incomming HID packet
    int16_t temp = data->Rotation;
    temp = constrain(temp, 30, 1800);
    ROTATION_MAX = CPR * temp / 360;
    ROTATION_MID = ROTATION_MAX / 2;

    if (data->Calibrate == HIGH) brWheelFFB.calibrate();

    configGeneralGain = constrain(data->GeneralGain, 0, 200);
    configConstantGain = constrain(data->ConstantGain, 0, 200);
    configDamperGain = constrain(data->DamperGain, 0, 200);
    configFrictionGain = constrain(data->FrictionGain, 0, 200);
    configPeriodicGain = constrain(data->PeriodicGain, 0, 200);
    configSpringGain = constrain(data->SpringGain, 0, 200);
    configInertiaGain = constrain(data->InertiaGain, 0, 200);
    configCenterGain = constrain(data->CenterGain, 0, 200);
    configStopGain = constrain(data->StopGain, 0, 200);
  } else if (data->Info == 255) { // milos, send all firmware settings to HID packet
    //data->ReportId = 0xF2; // milos, already
    data->Rotation = ROTATION_DEG; //milos, added
    data->Offset = brWheelFFB.offset; //milos, added
    data->EncCPR = CPR; // milos, added
    data->GeneralGain = configGeneralGain;
    data->ConstantGain = configConstantGain;
    data->DamperGain = configDamperGain;
    data->FrictionGain = configFrictionGain;
    data->PeriodicGain = configPeriodicGain;
    data->SpringGain = configSpringGain;
    data->InertiaGain = configInertiaGain;
    data->CenterGain = configCenterGain;
    data->StopGain = configStopGain;
    data->MaxForce = MM_MAX_MOTOR_TORQUE;
    data->MinForce = MM_MIN_MOTOR_TORQUE;
    data->PWMstate = pwmstate; // milos, added
    data->EFFstate = effstate; // milos, added
    data->LCscale = ffbBalance; // milos, added
    data->Version = FIRMWARE_VERSION;
    data->fwOption = fwOptions; // milos, added
  }
#endif // end of use config hid
  HID().SendReport(0xF2, (uint8_t*)data, 64);
}
