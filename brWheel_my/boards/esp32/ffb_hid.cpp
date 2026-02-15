#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <tusb.h>
#include <USB.h>
#include "hidDescriptor.h"
#include "ffb.h"
#include "ffb_hid.h"
#include "packed.h"

Adafruit_USBD_HID usb_hid(NULL, 0, HID_ITF_PROTOCOL_NONE, 2, true);

volatile uint16_t ffbReportLength = 0;
volatile uint8_t ffbReport[64];

constexpr size_t hidReportDescriptorSize = sizeof(_dynamicHidReportDescriptor) + sizeof(_staticHidReportDescriptor);
static uint8_t hidReportDescriptor[hidReportDescriptorSize];

uint16_t get_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen);
void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);

void buildHIDDescriptor()
{
  memcpy(hidReportDescriptor, _dynamicHidReportDescriptor, sizeof(_dynamicHidReportDescriptor));
  if (useDrivingHidProfile) {
    hidReportDescriptor[MAIN_AXES_USAGE_PAGE_OFFSET] = USAGE_PAGE_SIMULATION_CONTROLS;
    hidReportDescriptor[AXES_X_USAGE_OFFSET] = USAGE_SIM_STEERING;
    hidReportDescriptor[AXES_Y_USAGE_OFFSET] = USAGE_SIM_BRAKE;
    hidReportDescriptor[AXES_Z_USAGE_OFFSET] = USAGE_SIM_ACCELERATOR;
    hidReportDescriptor[FFB_AXES_USAGE_PAGE_OFFSET] = USAGE_PAGE_SIMULATION_CONTROLS;    
    hidReportDescriptor[FFB_AXES_USAGE_OFFSET] = USAGE_SIM_STEERING;    
  }
  memcpy(&hidReportDescriptor[sizeof(_dynamicHidReportDescriptor)], _staticHidReportDescriptor, sizeof(_staticHidReportDescriptor));
}

void HidAdapter::begin() {
  TinyUSBDevice.setSerialDescriptor("HIDPH");

  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  buildHIDDescriptor();
  
  //usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(hidReportDescriptor, sizeof(hidReportDescriptor));
  usb_hid.setReportCallback(get_report_callback, set_report_callback);
  usb_hid.begin();

  USB.begin();

  while (!TinyUSBDevice.mounted()) {
    delay(10);
  }
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

void HidAdapter::sendInputReport(uint8_t id, const void* data, uint8_t len) {
  usb_hid.sendReport(INPUT_REPORT_ID, data, len);
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
    if (reqlen != sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t)) {
      Serial0.println("Wrong reqlen for reportId=6: ");
      Serial0.println(reqlen);
    }

    delayMicroseconds(500); // TODO
    memcpy(buffer, &gNewEffectBlockLoad, sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
#ifdef FFBREPORT_WITH_REPORTID
    gNewEffectBlockLoad.reportId = 0;
#endif
    return sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t);
  }
  else if (report_id == 7)
  {
    if (reqlen != sizeof(USB_FFBReport_PIDPool_Feature_Data_t)) {
      Serial0.println("Wrong reqlen for reportId=6: ");
      Serial0.println(reqlen);
    }

    USB_FFBReport_PIDPool_Feature_Data_t ans;
#ifdef FFBREPORT_WITH_REPORTID
    ans.reportId = report_id;
#endif
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
  if (report_id == 0 && report_type == HID_REPORT_TYPE_OUTPUT) {
    ffbReportLength = bufsize;
    memcpy((void*) ffbReport, buffer, bufsize);
  }
  else if (report_type == HID_REPORT_TYPE_FEATURE)
  {
		if (report_id == 5)
		{
      if (bufsize != sizeof(USB_FFBReport_CreateNewEffect_Feature_Data_t)) {
        Serial0.println("Wrong bufsize for reportId=5: ");
        Serial0.println(bufsize);
      }
      USB_FFBReport_CreateNewEffect_Feature_Data_t* ans = (USB_FFBReport_CreateNewEffect_Feature_Data_t*) buffer;
      FfbOnCreateNewEffect(ans, &gNewEffectBlockLoad);
		}
  }
}
