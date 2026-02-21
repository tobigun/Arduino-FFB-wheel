#include <Arduino.h>

#include "WHID.h"
#include "common.h"
#include "hidDescriptor.h"
#include "ffb.h"
#include "ffb_hid.h"

static uint8_t dynamicHidReportDescriptor[sizeof(_dynamicHidReportDescriptor)];

void buildHIDDescriptor()
{
  memcpy_P((void*) dynamicHidReportDescriptor, _dynamicHidReportDescriptor, sizeof(_dynamicHidReportDescriptor));

  HID_PROFILE_ID profileId = readHidProfileId();
  if (profileId == DRIVING_WHEEL) {
    dynamicHidReportDescriptor[MAIN_AXES_USAGE_PAGE_OFFSET] = USAGE_PAGE_SIMULATION_CONTROLS;
    dynamicHidReportDescriptor[AXES_X_USAGE_OFFSET] = USAGE_SIM_STEERING;
    dynamicHidReportDescriptor[AXES_Y_USAGE_OFFSET] = USAGE_SIM_BRAKE;
    dynamicHidReportDescriptor[AXES_Z_USAGE_OFFSET] = USAGE_SIM_ACCELERATOR;
    dynamicHidReportDescriptor[FFB_AXES_USAGE_PAGE_OFFSET] = USAGE_PAGE_SIMULATION_CONTROLS;    
    dynamicHidReportDescriptor[FFB_AXES_USAGE_OFFSET] = USAGE_SIM_STEERING;    
  }
}

void HidAdapter::begin() {
  buildHIDDescriptor();

  static HIDSubDescriptor dynamicHidNode(dynamicHidReportDescriptor, sizeof(dynamicHidReportDescriptor));
  static HIDSubDescriptor staticPidNode(_staticHidReportDescriptor, sizeof(_staticHidReportDescriptor), TRANSFER_PGM);
  HID().AppendDescriptor(&dynamicHidNode);
  HID().AppendDescriptor(&staticPidNode);
  HID().begin();
}

bool HidAdapter::isReady() {
  return true; // WHID does not offer a ready check
}

void HidAdapter::recvFromUsb() {
  // no polling. RecvFfbReport is triggered by IRQ instead
}

bool HidAdapter::sendInputReport(uint8_t id, const void* data, uint8_t len) {
  return HID().SendReport(INPUT_REPORT_ID, data, len) >= 0;
}

void HID_::RecvFfbReport() {
  if (AvailableReport() > 0) {
    uint8_t out_ffbdata[64];
    uint16_t len = USB_Recv(HID_ENDPOINT_OUT, &out_ffbdata, 64);
    if (len >= 0) {
      FfbOnUsbData(out_ffbdata, len);
    }
  }
}

bool HID_::HID_GetReport(USBSetup& setup)
{
  uint8_t report_id = setup.wValueL;
  uint8_t report_type = setup.wValueH;
  if (report_type != HID_REPORT_TYPE_FEATURE) {
    return false;
  }

  if ((report_id == 6))// && (gNewEffectBlockLoad.reportId==6))
  {
    _delay_us(500);
    USB_SendControl(TRANSFER_RELEASE, &gNewEffectBlockLoad, sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
    gNewEffectBlockLoad.reportId = 0;
    return true;
  }
  if (report_id == 7)
  {
    USB_FFBReport_PIDPool_Feature_Data_t ans;
    ans.reportId = report_id;
    ans.ramPoolSize = 0xffff; // MEMORY_SIZE;
    ans.maxSimultaneousEffects = MAX_EFFECTS;
    ans.memoryManagement = 3;
    USB_SendControl(TRANSFER_RELEASE, &ans, sizeof(USB_FFBReport_PIDPool_Feature_Data_t));
    return true;
  }
  return false;
}

bool HID_::HID_SetReport(USBSetup& setup)
{
  u8 report_id = setup.wValueL;
  u8 report_type = setup.wValueH;
  uint16_t length = setup.wLength;
  uint8_t data[10];

  if (report_type != HID_REPORT_TYPE_FEATURE) {
    return false;
  }

  if (length == 0)
  {
    USB_RecvControl(&data, length);
    // Block until data is read (make length negative)
    //disableFeatureReport();
    return true;
  }
  if (report_id == 5)
  {
    USB_FFBReport_CreateNewEffect_Feature_Data_t ans;
    USB_RecvControl(&ans, sizeof(USB_FFBReport_CreateNewEffect_Feature_Data_t));
    FfbOnCreateNewEffect(&ans, &gNewEffectBlockLoad);
  }

  return true;
}
