#pragma once

#include <stdint.h>

#define NB_FF_AXIS		2 // must be 2, otherwise Forza Horizon 4 will crash when starting
#define NB_BUTTONS		10
#define X_AXIS_NB_BITS	15
#define AXIS_NB_BITS    10
#define Y_AXIS_NB_BITS	AXIS_NB_BITS
#define Z_AXIS_NB_BITS	AXIS_NB_BITS
#define RX_AXIS_NB_BITS	AXIS_NB_BITS
#define RY_AXIS_NB_BITS AXIS_NB_BITS

#define AXIS_LOG_MAX	((1L << AXIS_NB_BITS) - 1) // 1023
#define AXIS_LOG_MIN	0L

#define X_AXIS_LOG_MAX	((1L << X_AXIS_NB_BITS) - 1) // 32767
#define X_AXIS_LOG_MIN	0L
#define X_AXIS_LOG_MID  (X_AXIS_LOG_MAX / 2)

#define Y_AXIS_LOG_MAX	AXIS_LOG_MAX
#define Y_AXIS_LOG_MIN	AXIS_LOG_MIN

#define Z_AXIS_LOG_MAX	AXIS_LOG_MAX
#define Z_AXIS_LOG_MIN	AXIS_LOG_MIN

#define RX_AXIS_LOG_MAX	AXIS_LOG_MAX
#define RX_AXIS_LOG_MIN	AXIS_LOG_MIN

#define RY_AXIS_LOG_MAX AXIS_LOG_MAX
#define RY_AXIS_LOG_MIN AXIS_LOG_MIN

#define RZ_AXIS_LOG_MAX AXIS_LOG_MAX
#define RZ_AXIS_LOG_MIN AXIS_LOG_MIN

class HidAdapter
{
public:
  void begin();

  void recvFromUsb();

  bool isReady();

  bool sendInputReport(uint8_t id, const void* data, uint8_t len);
};

extern HidAdapter hidAdapter;
