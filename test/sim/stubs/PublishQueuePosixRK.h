#pragma once
#include "Particle.h"
void UbidotsHandler(const char *event, const char *data);
struct PublishQueuePosix {                               // File-backed queue: survives reboots, drains 1/sec when connected
  bool pausePublishing = false;
  void setPausePublishing(bool value) { pausePublishing = value; }
  static PublishQueuePosix& instance() { static PublishQueuePosix q; return q; }
  PublishQueuePosix& withFileQueueSize(size_t) { return *this; }
  void setup() {}
  size_t getNumEvents() { return SIM.pending.size(); }   // Library counts the in-flight event too; stub sends atomically
  void publish(const char* e, const char* d, int) { SIM.queued.push_back({simNow(), e, d}); SIM.pending.push_back({simNow(), e, d}); }
  void loop() {
    if (pausePublishing) return;
    if (SIM.pending.empty() || !Particle.connected() || SIM.ms - SIM.connectedAtMs < 2000 || SIM.ms - SIM.lastSendMs < 1000) return;   // Library: 2 s after connect, then 1/sec
    SimReport r = SIM.pending.front(); SIM.pending.erase(SIM.pending.begin());
    SIM.lastSendMs = SIM.ms; r.t = simNow(); SIM.delivered.push_back(r);
    if (r.event == "Ubidots-Sensor-Hook-v1") SIM.respAtMs.push_back(SIM.ms + 2000);   // Webhook response ~2 s later
  }
};
