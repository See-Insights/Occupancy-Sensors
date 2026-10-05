// Focused reproductions using the existing stand-ins. Each mode runs in a fresh process.
// Historical v19.01 witness; success means the old defect was reproduced.
// Superseded by the ordinary pass/fail scenarios in sim.cpp and check.py.
// adversarial.sh now runs the hardened suite instead of compiling this file.
#define main originalSimulationMain
#include "sim.cpp"
#undef main

int main(int argc, char** argv) {
  std::string mode = argc > 1 ? argv[1] : "boundary";
  SIM.epochBase = L(2026, 9, 29, 6, 59, 20);
  SIM.soc = 60;
  setup();
  sysStatus.lowPowerMode = true;
  sysStatus.openTime = 6;
  sysStatus.closeTime = 21;
  current.debounceMin = 5;

  if (mode == "boundary") {
    // A late boot/PIR report succeeds in the last minute of the hour.
    state = REPORTING_STATE;
    oldState = IDLE_STATE;
    stayAwake = 1000;
    runUntil(L(2026, 9, 29, 7, 1));
    if (SIM.modemOn.size() < 2) return 1;
    auto gap = SIM.modemOn[1] - SIM.modemOn[0];
    printf("boundary: modem power-ups %lld s apart (minimum 600 s)\n", (long long)gap);
    return gap < 600 ? 0 : 1;
  }
  if (mode == "temperature") {
    SIM.adc = 1191;
    const double sensedC = ((SIM.adc * 3.3 / 4096.0) - 0.5) * 100;
    takeMeasurements();
    printf("temperature: ideal TMP36 %.4f C, firmware %d F, charging %d\n",
           sensedC, current.temperature, SIM.chargingEnabled);
    return sensedC > 45 && SIM.chargingEnabled ? 0 : 1;
  }
  if (mode == "drain") {
    // Queued events cannot be acknowledged although the cloud session stays up.
    // Pause publishing to model a queue unable to progress. Run the real loop.
    PublishQueuePosix::instance().setPausePublishing(true);
    SIM.particleConnected = true;
    sysStatus.lastHookResponse = Time.now();
    current.debounceMin = 1;
    current.occupancyStatus = false;
    state = IDLE_STATE;
    oldState = INITIALIZATION_STATE;
    stayAwakeTimeStamp = millis();
    stayAwake = 1000;
    sendEvent();
    auto started = SIM.ms;
    for (int i = 0; i < 6000; ++i) {
      if (i % 650 == 0) sensorDetect = true;
      loop();
      SIM.ms += 100;
      if (readyToDisconnect()) return 1;
    }
    printf("drain: oldest event pending for %lld s, readyToDisconnect=%d\n",
           (long long)((SIM.ms - started) / 1000), readyToDisconnect());
    return 0;
  }
  return 2;
}
