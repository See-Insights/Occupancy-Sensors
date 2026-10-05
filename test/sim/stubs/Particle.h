// Host-side stand-in for the Particle Device OS API - just enough to run Occupancy-Sensors.ino on a virtual clock
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include <functional>
using namespace std::chrono_literals;
using std::min;
using std::max;

typedef int32_t time32_t;
typedef uint16_t pin_t;
typedef uint8_t byte;

// ---------------------------------------------------------------- simulation state
struct SimReport { int64_t t; std::string event; std::string data; };
struct SimLogLine { int64_t t; std::string text; };
struct SimCharge { int64_t ms; int adc; bool enabled; std::string cause; };
struct SimPowerConfig { int64_t ms; int minV, chargeV, chargeCurrent; };
struct SimState {
  int64_t epochBase = 0;          // UTC seconds at simMs == 0
  int64_t ms = 0;                 // virtual millis
  bool networkUp = true;          // Can we connect?
  int connectDelayMs = 2000;
  float soc = 77;
  bool particleConnected = false, connecting = false, cellularOn = false;
  int64_t connectAt = 0;
  int64_t connectedAtMs = 0;      // When the cloud session came up (queue waits 2 s after this)
  int64_t netUpAt = 0;
  std::vector<int64_t> modemOn;   // UTC seconds of each modem off->on transition
  int adc = 881;                  // TMP36 reading (~70F)
  int rawBatteryState = 4;        // What System.batteryState() returns
  int pmicMinV = -1;              // powerSourceMinVoltage of the last applied PMIC config (0 = Device OS defaults)
  int defaultsLeft = 0;           // Times a default-only PMIC config was left in place at the end of a takeMeasurements
  bool chargingEnabled = true;
  int uncleanPowerDowns = 0;      // deepPowerDown with the modem still on
  int offDelayMs = 0, cloudDisconnectDelayMs = 0;
  bool offFails = false, cloudDisconnectFails = false;
  int64_t offAtMs = -1, cloudDisconnectAtMs = -1;
  std::vector<SimCharge> chargingChanges;
  std::vector<SimPowerConfig> powerConfigs;
  std::vector<SimPowerConfig> powerChecks;
  int pmicChargeV = 0, pmicChargeCurrent = 0;
  bool pmicChargeDisabled = false;
  int64_t maxConnectedPendingMs = 0, connectedPendingSinceMs = -1;
  int waitTimeouts = 0;
  int offRequests = 0;
  bool powerDownFails = false;
  int powerDownAttempts = 0;
  std::vector<std::pair<int64_t, bool>> powerCuts;
  std::vector<std::string> powerDowns;            // UTC seconds the network outage ended (nonet scenario)
  std::vector<SimReport> queued;  // Everything handed to PublishQueuePosix (timestamped at queue time)
  std::vector<SimReport> pending; // Not yet delivered
  std::vector<SimReport> delivered;
  std::vector<SimReport> direct;  // Particle.publish
  std::vector<SimLogLine> log;
  bool verbose = false;
  // Interrupt / sleep model
  void (*isr)() = nullptr; pin_t isrPin = 0xFFFF;
  std::vector<int64_t> pir;       // Scheduled PIR rising edges (UTC seconds)
  size_t pirNext = 0;
  bool sensorEnabled = false;
  int64_t lastSendMs = 0;
  int sleeps = 0;
  std::vector<int64_t> respAtMs;  // Pending webhook responses
};
inline SimState SIM;
inline int64_t simNow() { return SIM.epochBase + SIM.ms / 1000; }
struct SimReboot { std::string why; };
inline std::function<void()> simTick;
inline void simHardwareTick() {
  if (SIM.cloudDisconnectAtMs >= 0 && SIM.ms >= SIM.cloudDisconnectAtMs) {
    SIM.particleConnected = false; SIM.cloudDisconnectAtMs = -1;
    SIM.connectedPendingSinceMs = -1; // End the episode at the actual transition, even between input ticks.
  }
  if (SIM.offAtMs >= 0 && SIM.ms >= SIM.offAtMs) {
    SIM.cellularOn = SIM.particleConnected = SIM.connecting = false; SIM.offAtMs = -1;
    SIM.connectedPendingSinceMs = -1;
  }
}
inline void simAdvance(int64_t durationMs) {
  const auto end = SIM.ms + durationMs;
  while (SIM.ms < end) {
    SIM.ms = std::min(end, SIM.ms + 10);
    simHardwareTick();
    if (simTick) simTick();
  }
}
template<class F> bool simWaitFor(F condition, int64_t timeoutMs) {
  const auto end = SIM.ms + timeoutMs;
  while (!condition() && SIM.ms < end) simAdvance(std::min<int64_t>(10, end - SIM.ms));
  const bool result = condition();
  if (!result) SIM.waitTimeouts++;
  return result;
}
inline void simCharging(bool enabled, const char* cause, bool recordAction = true) {
  if (recordAction || enabled != SIM.chargingEnabled)
    SIM.chargingChanges.push_back({SIM.ms, SIM.adc, enabled, cause});
  SIM.chargingEnabled = enabled;
}
inline void simWakePowerReload() {
  simCharging(!SIM.pmicChargeDisabled, "device-os-wake-reload", false);
}

// ---------------------------------------------------------------- String
class String {
public:
  std::string s;
  String() {}
  String(const char* c) : s(c ? c : "") {}
  String(const std::string& x) : s(x) {}
  bool operator==(const char* o) const { return s == o; }
  bool operator!=(const char* o) const { return s != o; }
  operator const char*() const { return s.c_str(); }
  const char* c_str() const { return s.c_str(); }
  void toCharArray(char* buf, unsigned n) const { strncpy(buf, s.c_str(), n); buf[n-1] = 0; }
};

// ---------------------------------------------------------------- macros / misc
#define PRODUCT_VERSION(x)
#define SYSTEM_MODE(x)
#define SYSTEM_THREAD(x)
#define retained
#define STARTUP(x)
#define waitFor(cond, t) simWaitFor([] { return (cond)(); }, (t))
#define waitUntil(cond) simWaitFor([] { return (cond)(); }, 3600000)
template<class T, class L, class H> T constrain(T v, L lo, H hi) { return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v); }
inline unsigned long millis() { return (uint32_t)SIM.ms; }
inline void delay(unsigned long ms) { simAdvance(ms); }

enum { INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN };
enum InterruptMode { CHANGE, RISING, FALLING };
enum { LOW = 0, HIGH = 1 };
const pin_t A4 = 15, D4 = 4, D7 = 7, D8 = 8, SCK = 13, MOSI = 12, MISO = 11;
inline void pinMode(pin_t, int) {}
inline void digitalWrite(pin_t p, int v) { if (p == MOSI) SIM.sensorEnabled = (v == 0); }   // disableModule low = sensor on
inline int digitalRead(pin_t p) { return p == D4 ? 1 : 0; }                                  // user switch not pressed
inline void pinSetFast(pin_t) {}
inline int analogRead(pin_t) { return SIM.adc; }                                                  // ~70F
inline void attachInterrupt(pin_t p, void (*f)(), InterruptMode) { SIM.isr = f; SIM.isrPin = p; }
inline void detachInterrupt(pin_t) { SIM.isr = nullptr; }

enum PublishFlag { PUBLIC = 0, PRIVATE = 1, WITH_ACK = 2, NO_ACK = 4 };
enum { MY_DEVICES = 1 };

// ---------------------------------------------------------------- Log
enum LogLevel { LOG_LEVEL_ALL, LOG_LEVEL_INFO };
struct SerialLogHandler { SerialLogHandler(LogLevel) {} };
struct SimLogger {
  void info(const char* fmt, ...) {
    char b[512]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a);
    SIM.log.push_back({simNow(), b});
    if (SIM.verbose) printf("  [%lld] %s\n", (long long)simNow(), b);
  }
  void trace(const char*, ...) {}
};
inline SimLogger Log;
struct SimSerial { static bool isConnected() { return false; } };
inline SimSerial Serial;
struct TwoWire {}; inline TwoWire Wire;

// ---------------------------------------------------------------- Time
struct SimTime {
  float zoneH = 0, dstH = 1; bool dst = false;
  int64_t off() const { return (int64_t)((zoneH + (dst ? dstH : 0)) * 3600); }
  std::tm lt(int64_t t) const { time_t x = (time_t)(t + off()); std::tm r; gmtime_r(&x, &r); return r; }
  time32_t now() const { return (time32_t)simNow(); }
  time32_t local() const { return (time32_t)(simNow() + off()); }
  bool isValid() const { return true; }
  void zone(float z) { zoneH = z; }
  void setDSTOffset(float d) { dstH = d; }
  void beginDST() { dst = true; }
  void endDST() { dst = false; }
  int hour() const { return lt(simNow()).tm_hour; }
  int hour(int64_t t) const { return lt(t).tm_hour; }
  int minute() const { return lt(simNow()).tm_min; }
  int day() const { return lt(simNow()).tm_mday; }
  int day(int64_t t) const { return lt(t).tm_mday; }
  int month() const { return lt(simNow()).tm_mon + 1; }
  int year() const { return lt(simNow()).tm_year + 1900; }
  int weekday() const { return lt(simNow()).tm_wday + 1; }
  String timeStr(int64_t t) const { char b[32]; std::tm r = lt(t); strftime(b, sizeof b, "%F %T", &r); return String(b); }
};
inline SimTime Time;

// ---------------------------------------------------------------- Particle cloud
struct CloudDisconnectOptions {
  CloudDisconnectOptions& graceful(bool) { return *this; }
  CloudDisconnectOptions& timeout(std::chrono::milliseconds) { return *this; }
};
struct SimParticle {
  static bool connected() {
    if (SIM.connecting && SIM.networkUp && SIM.ms >= SIM.connectAt) { SIM.connecting = false; SIM.particleConnected = true; SIM.connectedAtMs = SIM.ms; }
    return SIM.particleConnected;
  }
  static bool disconnected() { simHardwareTick(); return !SIM.particleConnected; }
  static void connect() { if (!SIM.cellularOn) SIM.modemOn.push_back(simNow()); SIM.cellularOn = true; if (!SIM.particleConnected && !SIM.connecting) { SIM.connecting = true; SIM.connectAt = SIM.ms + SIM.connectDelayMs; } }
  static void disconnect() {
    SIM.connecting = false;
    if (!SIM.cloudDisconnectFails) {
      SIM.cloudDisconnectAtMs = SIM.ms + SIM.cloudDisconnectDelayMs;
      simHardwareTick();
    }
  }
  static bool publish(const char* e, const char* d, int) { SIM.direct.push_back({simNow(), e, d}); return true; }
  static bool publish(const char* e, const String& d, int f) { return publish(e, d.c_str(), f); }
  template<class A, class B, class C> static bool subscribe(A, B, C) { return true; }
  template<class A, class B> static bool variable(A, B) { return true; }
  template<class A, class B> static bool function(A, B) { return true; }
  static void syncTime() {}
  static bool syncTimeDone() { return true; }
  static void setDisconnectOptions(const CloudDisconnectOptions&) {}
};
inline SimParticle Particle;

struct CellularSignal {
  int getAccessTechnology() { return 8; }
  float getStrength() { return 50; } float getQuality() { return 50; }
  float getStrengthValue() { return -90; } float getQualityValue() { return -10; }
};
struct SimCellular {
  static bool isOff() { simHardwareTick(); return !SIM.cellularOn; }
  static bool isOn() { return SIM.cellularOn; }
  static void on() { if (!SIM.cellularOn) SIM.modemOn.push_back(simNow()); SIM.cellularOn = true; }
  static void off() {
    SIM.offRequests++;
    if (!SIM.offFails && SIM.offAtMs < 0) {
      SIM.offAtMs = SIM.ms + SIM.offDelayMs; simHardwareTick();
    }
  }
  static void disconnect() {}
  static bool ready() { return SIM.cellularOn && SIM.networkUp; }
  static CellularSignal RSSI() { return {}; }
  static int command(int, const char*) { return 0; }
};
inline SimCellular Cellular;
enum { NETWORK_INTERFACE_CELLULAR = 1 };

// ---------------------------------------------------------------- System / sleep / power
enum { RESET_REASON_NONE = 0, RESET_REASON_PIN_RESET = 20, RESET_REASON_USER = 140, FEATURE_RESET_INFO = 1, SLEEP_MODE_DEEP = 1 };
typedef int system_event_t; enum { out_of_memory = 1 };
enum class SystemSleepMode { ULTRA_LOW_POWER, STOP, HIBERNATE };
enum class SystemSleepNetworkFlag { NONE, INACTIVE_STANDBY };
struct SystemSleepConfiguration {
  std::vector<pin_t> pins; int64_t durMs = 0; bool netStandby = false;
  SystemSleepConfiguration& mode(SystemSleepMode) { return *this; }
  SystemSleepConfiguration& gpio(pin_t p, InterruptMode) { if (std::find(pins.begin(), pins.end(), p) == pins.end()) pins.push_back(p); return *this; }  // Device OS keeps adding to the same list
  SystemSleepConfiguration& duration(int64_t ms) { durMs = ms; return *this; }
  SystemSleepConfiguration& network(int, SystemSleepNetworkFlag f) { netStandby = (f == SystemSleepNetworkFlag::INACTIVE_STANDBY); return *this; }
};
struct SystemSleepResult { pin_t pin = 0xFFFF; pin_t wakeupPin() const { return pin; } };
enum class SystemPowerFeature { USE_VIN_SETTINGS_WITH_USB_HOST, DISABLE_CHARGING };
struct SystemPowerConfiguration {
  int minV = 0, chargeV = 0, chargeCurrent = 0;
  bool disableCharge = false;
  SystemPowerConfiguration& powerSourceMaxCurrent(int) { return *this; }
  SystemPowerConfiguration& powerSourceMinVoltage(int v) { minV = v; return *this; }
  SystemPowerConfiguration& batteryChargeCurrent(int v) { chargeCurrent = v; return *this; }
  SystemPowerConfiguration& batteryChargeVoltage(int v) { chargeV = v; return *this; }
  SystemPowerConfiguration& feature(SystemPowerFeature f) { if (f == SystemPowerFeature::DISABLE_CHARGING) disableCharge = true; return *this; }
};
SystemSleepResult simSleep(const SystemSleepConfiguration& c);   // Defined in the harness
struct SimSystem {
  static int resetReasonValue;
  static int resetReason() { return resetReasonValue; }
  static void enableFeature(int) {}
  template<class F> static void on(int, F) {}
  static void disableUpdates() {} static void enableUpdates() {}
  static SystemSleepResult sleep(const SystemSleepConfiguration& c) { return simSleep(c); }
  static void sleep(int, int s) { SIM.ms += s * 1000LL; throw SimReboot{"deep sleep"}; }
  static int setPowerConfiguration(const SystemPowerConfiguration& c) {
    SIM.pmicMinV = c.minV; SIM.pmicChargeV = c.chargeV; SIM.pmicChargeCurrent = c.chargeCurrent;
    SIM.pmicChargeDisabled = c.disableCharge;
    SIM.powerConfigs.push_back({SIM.ms, c.minV, c.chargeV, c.chargeCurrent});
    // Conservative synchronous reload; real Device OS queues the reload.
    simCharging(!c.disableCharge, "config-reload", false);
    return 0;
  }
  [[noreturn]] static void reset() { throw SimReboot{"System.reset"}; }
  static int batteryState() { return SIM.rawBatteryState; }
  static String deviceID() { return String("e00fce68sim"); }
};
inline int SimSystem::resetReasonValue = RESET_REASON_NONE;
inline SimSystem System;

struct FuelGauge { void wakeup() {} void quickStart() {} float getSoC() { return SIM.soc; } };
struct PMIC { PMIC(bool) {} void enableCharging() { simCharging(true, "enableCharging"); } void disableCharging() { simCharging(false, "disableCharging"); } };
