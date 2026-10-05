#pragma once
#include "Particle.h"
struct AB1805 {
  static const int WATCHDOG_MAX_SECONDS = 124;
  AB1805(TwoWire&) {}
  AB1805& withFOUT(pin_t) { return *this; }
  void setup() {} void setWDT(int) {} void stopWDT() {} void resumeWDT() {} void loop() {}
  bool deepPowerDown(int s = 30) { SIM.powerDownAttempts++; if (SIM.powerDownFails) return false; SIM.powerCuts.push_back({SIM.ms, SIM.cellularOn}); if (SIM.cellularOn) SIM.uncleanPowerDowns++; SIM.powerDowns.push_back(std::to_string(simNow()) + (SIM.cellularOn ? " modem ON" : " modem off")); SIM.ms += s * 1000LL; throw SimReboot{"deepPowerDown"}; }
};
