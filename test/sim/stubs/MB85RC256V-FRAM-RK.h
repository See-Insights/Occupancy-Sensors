#pragma once
#include "Particle.h"
inline uint8_t SIM_FRAM[8192];
struct MB85RC64 {
  MB85RC64(TwoWire&, int) {}
  void begin() {}
  void erase() { memset(SIM_FRAM, 0, sizeof SIM_FRAM); }
  template<class T> void get(size_t a, T& v) { memcpy((void*)&v, SIM_FRAM + a, sizeof(T)); }
  template<class T> void put(size_t a, const T& v) { memcpy(SIM_FRAM + a, (const void*)&v, sizeof(T)); }
};
