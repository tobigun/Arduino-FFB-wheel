#pragma once

#include <Arduino.h>

class HidAdapter
{
public:
  void begin();

  void recvFromUsb();

  void sendInputReport(int16_t x, int16_t y, int16_t z, int16_t rx, int16_t ry, uint8_t hat, uint16_t buttons);
};
