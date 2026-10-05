#include "stubs/Particle.h"
#include <cassert>

SystemSleepResult simSleep(const SystemSleepConfiguration&) { return {}; }

int main() {
  SIM = {};
  auto start = SIM.ms;
  assert(simWaitFor([] { return true; }, 1000));
  assert(SIM.ms == start);
  assert(simWaitFor([] { return SIM.ms >= 250; }, 1000));
  assert(SIM.ms == 250);
  assert(!simWaitFor([] { return false; }, 1000));
  assert(SIM.ms == 1250 && SIM.waitTimeouts == 1);

  SIM.cellularOn = SIM.particleConnected = true;
  SIM.cloudDisconnectDelayMs = 5000;
  SIM.offDelayMs = 20000;
  start = SIM.ms;
  Particle.disconnect();
  assert(Particle.connected());
  assert(waitFor(Particle.disconnected, 15000));
  assert(SIM.ms - start == 5000);
  Cellular.off();
  assert(!Cellular.isOff());
  assert(waitFor(Cellular.isOff, 30000));
  assert(SIM.ms - start == 25000);

  Cellular.on();
  SIM.offFails = true;
  start = SIM.ms;
  Cellular.off();
  assert(!waitFor(Cellular.isOff, 30000));
  assert(SIM.ms - start == 30000 && Cellular.isOn());
  SIM.cloudDisconnectFails = true;
  SIM.particleConnected = true;
  Particle.disconnect();
  assert(!waitFor(Particle.disconnected, 15000));
  assert(SIM.ms - start == 45000);

  PMIC pmic(true);
  pmic.disableCharging();
  simWakePowerReload();
  assert(SIM.chargingEnabled); // A direct PMIC write is transient.
  SystemPowerConfiguration config;
  config.feature(SystemPowerFeature::DISABLE_CHARGING);
  System.setPowerConfiguration(config);
  simWakePowerReload();
  assert(!SIM.chargingEnabled); // Configured disable survives Power Manager reload.
  puts("model checks: waits, delayed/failed shutdown, and persistent charge-disable PASS");
}
