#pragma once

#include <Arduino.h>

class SerialDummy : public Stream
{
  public:
    void begin(unsigned long baud) {}
    void end(void) {}

    virtual int available(void) { return 0; }
    virtual void accept(void) {}
    virtual int peek() { return 0; }
    virtual int read(void) { return -1; }
    virtual void flush(void) {}
    virtual size_t write(uint8_t c) { return write(&c, 1); }
    //using Print::write;
    size_t write(const uint8_t *buf, size_t length) { return 0; }
    operator bool() { return false; }
};

extern SerialDummy SerialDummy_;
