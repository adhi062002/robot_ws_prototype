#pragma once
#include <stdint.h>
#include <stddef.h>
struct TwoWire {
  void begin(); void setClock(uint32_t); void beginTransmission(uint8_t);
  size_t write(uint8_t); uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(int addr, int n); int read();
};
extern TwoWire Wire, Wire1, Wire2;
