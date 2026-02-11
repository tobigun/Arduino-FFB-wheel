#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <tusb.h>
#include "hidDescriptor.h"
#include "ffb.h"
#include "ffb_hid.h"
#include "packed.h"

struct ATTR_PACKED InputReport {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t rx;
  int16_t ry;
  uint8_t hat;
  uint16_t buttons : NB_BUTTONS;
  uint8_t padding : 8 - (NB_BUTTONS % 8);
};

Adafruit_USBD_HID usb_hid(NULL, 0, HID_ITF_PROTOCOL_NONE, 2, true);

volatile uint16_t ffbReportLength = 0;
volatile uint8_t ffbReport[64];

bool useDrivingHidProfile = false;
bool useCombinedAxes = false;

static uint8_t dynamicHidReportDescriptor[sizeof(_dynamicHidReportDescriptor)];

uint16_t get_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen);
void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);

void buildHIDDescriptor()
{
  memcpy_P((void*) dynamicHidReportDescriptor, _dynamicHidReportDescriptor, sizeof(_dynamicHidReportDescriptor));
  if (useDrivingHidProfile) {
    dynamicHidReportDescriptor[MAIN_AXES_USAGE_PAGE_OFFSET] = USAGE_PAGE_SIMULATION_CONTROLS;
    dynamicHidReportDescriptor[AXES_X_USAGE_OFFSET] = USAGE_SIM_STEERING;
    dynamicHidReportDescriptor[AXES_Y_USAGE_OFFSET] = USAGE_SIM_BRAKE;
    dynamicHidReportDescriptor[AXES_Z_USAGE_OFFSET] = USAGE_SIM_ACCELERATOR;
    dynamicHidReportDescriptor[FFB_AXES_USAGE_PAGE_OFFSET] = USAGE_PAGE_SIMULATION_CONTROLS;    
    dynamicHidReportDescriptor[FFB_AXES_USAGE_OFFSET] = USAGE_SIM_STEERING;    
  }
}

void HidAdapter::begin() {
  //if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  //}
  //  TinyUSBDevice.clearConfiguration();

  //buildHIDDescriptor();
  //usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(_dynamicHidReportDescriptor, sizeof(_dynamicHidReportDescriptor));
  usb_hid.setReportCallback(get_report_callback, set_report_callback);
  usb_hid.begin();

  Serial.println("USB HID added");
  
  // if already enumerated, re-attach
  //if (TinyUSBDevice.mounted()) {
  //  TinyUSBDevice.detach();
  //  delay(10);
  //  TinyUSBDevice.attach();
  //}

  Serial0.println("USB wait");
  while (!TinyUSBDevice.mounted()) { delay(10); }
  Serial0.println("USB HID started");
}

void HidAdapter::recvFromUsb() 
{
	if (ffbReportLength > 0) {
		uint8_t out_ffbdata[64];
		memcpy(out_ffbdata, (void*) ffbReport, ffbReportLength);

    FfbOnUsbData(out_ffbdata, ffbReportLength);
		ffbReportLength = 0;
	}
}

#define SWAP_BITS(bits) ((((bits) << 1) & 0x2) | (((bits) >> 1) & 0x1))

static uint16_t rearrangeButtons(uint16_t buttons) {
  uint8_t gearBtns = buttons & 0b11;
  uint8_t dpadBtns = (buttons >> 2) & 0b1111;
  uint8_t sideBtns = SWAP_BITS((buttons >> 8) & 0b11);
  uint16_t frontButtons = SWAP_BITS((buttons >> 10) & 0b11);
  return dpadBtns | (frontButtons << 4) | (sideBtns << 6) | (gearBtns << 8);
}


void HidAdapter::sendInputReport(int16_t x, int16_t y, int16_t z, int16_t rx, int16_t ry, uint8_t hat, uint16_t buttons) {
  InputReport report = {
    x: x,
    y: y,
    z: z,
    rx: rx,
    ry: ry,
    hat: hat,
    buttons: rearrangeButtons(buttons)
  };
  usb_hid.sendReport(INPUT_REPORT_ID, (uint8_t*)&report, sizeof(report));
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t get_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  if (report_type != HID_REPORT_TYPE_FEATURE) {
    return 0;
  }

  if ((report_id == 6))// && (gNewEffectBlockLoad.reportId==6))
  {
    delayMicroseconds(500); // TODO
    memcpy(buffer, &gNewEffectBlockLoad, sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
    gNewEffectBlockLoad.reportId = 0;

    return sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t);
  }
  else if (report_id == 7)
  {
    USB_FFBReport_PIDPool_Feature_Data_t ans;
    //ans.reportId = report_id;
    ans.ramPoolSize = 0xffff;
    ans.maxSimultaneousEffects = MAX_EFFECTS;
    ans.memoryManagement = 3;
    memcpy(buffer, &ans, sizeof(USB_FFBReport_PIDPool_Feature_Data_t));
    return sizeof(USB_FFBReport_PIDPool_Feature_Data_t);
  }

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  if (report_id == 0 && report_type == 0) {
    ffbReportLength = bufsize;
    memcpy((void*) ffbReport, buffer, bufsize);
  }
  else if (report_type == HID_REPORT_TYPE_FEATURE)
  {
    if (bufsize == 0) { // can this really happen?
      return;
    }
		if (report_id == 5)
		{
      USB_FFBReport_CreateNewEffect_Feature_Data_t* ans = (USB_FFBReport_CreateNewEffect_Feature_Data_t*) buffer;
      FfbOnCreateNewEffect(ans, &gNewEffectBlockLoad);
		}
  }
}
