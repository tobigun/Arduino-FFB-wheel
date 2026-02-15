#pragma once

#include <Arduino.h>

class HidAdapter
{
public:
  void begin();

  void recvFromUsb();

  void sendInputReport(uint8_t id, const void* data, int len);
};
