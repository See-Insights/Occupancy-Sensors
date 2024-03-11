/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/chipmc/Documents/Maker/Particle/Projects/Occupancy-Sensors/src/Occupancy-Sensors.ino"
/*
* Project Occupancy Sensors - Building a variant of the visitaion counters to determine the occupancy state
* Description: Cellular Connected Occupancy Sensors
* Author: Chip McClelland
* Date:20 April 6th, 2021
*/

/*  This is a refinement on the Boron Connected Counter firmware and incorporates new watchdog and RTC
*   capabilities as laid out in AN0023 - https://github.com/particle-iot/app-notes/tree/master/AN023-Watchdog-Timers
*   This software will work with both pressure and PIR sensor counters
*/

//v1 - Adapted from the Visitation Counter Code at release v9.02
//v2 - Fixed an issue that could cause the device to ignore counts
//v2.01 - Bug fixes - version for first deploayment at Morrisville Aquatic and Fitness Center
//v2.02 - Reporting bugs - will stay awake while the court is occupied for now.
//v2.03 - Rethinking the sleep thing - will wake on time but different based on whether occupied (debounce period) or not occupied (till next hour).
//v3.00 - Removed line that restarted the session 
//v4.00 - Looking at issue with Morrisville 2
//v5.00 - Fixed excessive connection issue
//v5.10 - Version specific to the Machine Vision example
//v5.20 - Updated to better match new approach to managing connection
//v6.00 - Updated with logic and structure from v40.01 in Visitation Counters code
//v6.01 - Fixed issue with ERROR state, tested sleep mode, fixed response_wait state, Added WITH_ACK 
//v6.02 - Fixed issue with too many reconnects - only when changed
//v6.03 - Added a check for Particle connected
//v7.00 - Looked at issue that could cause repeated publishes, also resets alerts after publish
//v8.00 - Small fixes - looking for repeated sending issue.
//v9.00 - Fixed issue with missed clearing of the occupied flag (>= not >) and potential issue for infinite publishes when going to sleep
//v10.00 - Turns off cellular modem at night and does not maintain network connection if battery level is under 65%
//v11.00 - Need to make some additional adjustments - Logging and hourly connections when battery < 65%
//v12.00 - Update to deviceOS@4.0.2 and removed firmware update state
//v13.00 - Added a check last connection time and deviceOS@4.1.0
//v14.00 - Working on why the device keeps dropping off-line compiled for deviceOS@4.2.0
//v15.00 - Compiled for deviceOS@5.5.0 - so we can use the Boron BRN-404X
//v16.00 - Issue with the variable type which could be causing memory corruption - Added an off-line mode for testing


// Included Libraries
#include "Particle.h"                               // Particle Library
#include "3rdGenDevicePinoutdoc.h"                  // Pinout Documentation File
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "PublishQueuePosixRK.h"                    // Allows for queuing of messages - https://github.com/rickkas7/PublishQueuePosixRK

// Libraries with helper functions
#include "time_zone_fn.h"
#include "sys_status.h"

// Particle Product definitions
void setup();
void loop();
bool timeToSleep();
void sensorControl(bool enableSensor);
void  recordConnectionDetails();
void serviceSensorEvent();
void serviceDebounceEvent();
void sendEvent();
void UbidotsHandler(const char *event, const char *data);
void takeMeasurements();
bool isItSafeToCharge();
void getSignalStrength();
int getTemperature();
void outOfMemoryHandler(system_event_t event, int param);
void sensorISR();
int setPowerConfig();
void loadSystemDefaults();
void checkSystemValues();
void makeUpParkHourStrings();
void makeUpStringMessages();
bool disconnectFromParticle();
int resetCounts(String command);
int hardResetNow(String command);
int sendNow(String command);
void resetEverything();
int setSolarMode(String command);
int setVerboseMode(String command);
String batteryContextMessage();
int setOpenTime(String command);
int setCloseTime(String command);
int setDebounceMin(String command);
int setLowPowerMode(String command);
void publishStateTransition(void);
void fullModemReset();
void dailyCleanup();
#line 51 "/Users/chipmc/Documents/Maker/Particle/Projects/Occupancy-Sensors/src/Occupancy-Sensors.ino"
PRODUCT_VERSION(16);

char currentPointRelease[6] = "16.00";

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentStateAddr      = 0x50                    // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 1;                    // Increment this number each time the memory map is changed

struct currentStatus_structure {                    // currently 10 bytes long
  uint16_t maxConnectTime = 0;                      // Longest connect time for the day
  uint8_t updateAttempts = 0;                       // How many update attempts today
  bool occupancyStatus = false;                     // Occupied or not - false at boot up
  time32_t lastOccupancyTime;                         // When as it last occupied?
  time32_t lastOccupancyChange;                       // When did the occupancy status last change
  int dailyOccupancyMinutes;                        // How many minutes has it been occupied today
  int longestOccupancyMinutes;                      // What was the longest it was occupied today
  int debounceMin;                                  // What is the time we will wait before assuming no occupancy
  int temperature;                                  // Current Temperature
  int alerts;                                       // What is the current alert count
  int minBatteryLevel = 100;                        // Lowest Battery level for the day
} current;

struct systemStatus_structure sysStatus;

#define DSTRULES isDSTusa
// Using this flag will allow the code to run without connecting to the cellular network
#define OFFLINEMODE 0                               // Value = 1 - no cellular and 0 for normal operations

// exceeded, do a deep power down. This should not be less than 10 minutes. 11 minutes
// is a reasonable value to use.
unsigned long connectMaxTimeSec = 10 * 60;   // Timeout for trying to connect to Particle cloud in seconds - reduced to 10 mins
// If updating, we need to delay sleep in order to give the download time to come through before sleeping
const std::chrono::milliseconds firmwareUpdateMaxTime = 10min; // Set at least 5 minutes

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
FuelGauge fuelGauge;                                // Needed to address issue with updates in low battery state

// For monitoring / debugging, you can uncomment the next line
// SerialLogHandler logHandler(LOG_LEVEL_ALL);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Connecting", "Reporting", "Response Wait"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Battery Conect variables
// Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
const char* batteryContext[7] = {"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};

// Pin Constants - Boron Carrier Board v1.2a
const int tmp36Pin =      A4;                       // Simple Analog temperature sensor
const int wakeUpPin =     D8;                       // This is the Particle Electron WKP pin
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int intPin =        SCK;                      // Sensor inerrupt pin
const int disableModule = MOSI;                     // Bringining this low turns on the sensor (pull-up on sensor board)
const int ledPower =      MISO;                     // Allows us to control the indicator LED on the sensor board

// Timing Variables                         
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 30000;            // How long will we wait for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
char currentOffsetStr[10];                          // What is our offset from UTC
unsigned long lastReportedTime = 0;                 // Need to keep this separate from time so we know when to report
unsigned long connectionStartTime;


// Program Variables
bool dataInFlight = false;                          // Tracks if we have sent data but not yet received a response
bool firmwareUpdateInProgress = false;              // Helps us track if a firmware update is in progress
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char batteryContextStr[16];                         // Tracks the battery context
char occupancyStateStr[16];                         // Text for occupancy
char lowPowerModeStr[16];                            // In low power mode?
char openTimeStr[8]="NA";                           // Park Open Time
char closeTimeStr[8]="NA";                          // Park close Time
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write
bool currentStatusWriteNeeded = false;

// These variables are associated with the watchdog timer and will need to be better integrated
int outOfMemory = -1;

// This section is where we will initialize sensor specific variables, libraries and function prototypes
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered

void setup()                                        // Note: Disconnected Setup()
{

  if (OFFLINEMODE) waitFor(Serial.isConnected,10000);    // If we are in off-line mode, we need to wait for the serial connection

  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output
  
  // Pressure / PIR Module Pin Setup
  pinMode(intPin,INPUT_PULLDOWN);                   // pressure sensor interrupt
  pinMode(disableModule,OUTPUT);                    // Turns on the module when pulled low
  pinMode(ledPower,OUTPUT);                         // Turn on the lights
  
  digitalWrite(blueLED,HIGH);                       // Turn on the led so we can see how long the Setup() takes

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple devices share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event
  System.on(out_of_memory, outOfMemoryHandler);     // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.


  Particle.variable("OccupancyState", occupancyStateStr);         // Define my Particle variables
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Temperature",current.temperature);
  Particle.variable("Release",currentPointRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  Particle.variable("OpenTime", openTimeStr);
  Particle.variable("CloseTime",closeTimeStr);
  Particle.variable("Alerts",current.alerts);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("BatteryContext",batteryContextMessage);
  Particle.variable("Debounce",current.debounceMin);
  Particle.variable("TotalTime",current.dailyOccupancyMinutes);

  Particle.function("Set-Debounce", setDebounceMin);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);

  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));  // Don't disconnect abruptly
// Watchdog Timer and Real Time Clock Initialization
  ab1805.withFOUT(D8).setup();                                         // The carrier board has D8 connected to FOUT for wake interrupts
  ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);                         // Enable watchdog

  // Take a look at the batter state of charge - good to do this before turning on the cellular modem
  fuelGauge.wakeup();                                                  // Expliciely wake the Feul gauge and give it a half-sec
  delay(500);
  fuelGauge.quickStart();                                              // May help us re-establish a baseline for SoC

  // Next we will load FRAM and check or reset variables to their correct values
  fram.begin();                                                        // Initialize the FRAM module
  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);                            // Load the FRAM memory map version into a variable for comparison
  if (tempVersion != FRAMversionNumber) {                              // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                      // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                    // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                          // See if this worked
    if (tempVersion != FRAMversionNumber) {
      state = ERROR_STATE;                                             // Device will not work without FRAM will need to reset
      resetTimeStamp = millis();                                       // Likely close to zero but, for form's sake
      current.alerts = 12;                                             // FRAM is messed up so can't store but will be read in ERROR state
    }
    else loadSystemDefaults();                                         // Out of the box, we need the device to be awake and connected
  }
  else {
    fram.get(FRAM::systemStatusAddr,sysStatus);                        // Loads the System Status array from FRAM
    fram.get(FRAM::currentStateAddr,current);                         // Loead the current values array from FRAM
  }

  if (OFFLINEMODE) {                                                   // This sets values for testing without a cellular connection
    sysStatus.openTime = 0;                                            // This is for the edge case where the clock is not set and the device won't connect as it thinks it is off hours
    sysStatus.closeTime = 24;                                          // This only resets if the device beleives it is off-hours
    current.debounceMin = 1;                                           // This is for the edge case where the clock is not set and the device won't connect as it thinks it is off hours; 
  }

  // Now that the system object is loaded - let's make sure the values make sense
  checkSystemValues();                                                // Make sure System values are all in valid range

  // Take note if we are restarting due to a pin reset - either by the user or the watchdog - could be sign of trouble
  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    if (sysStatus.resetCount > 3) current.alerts = 13;                 // Excessive resets
  }

  // Publish Queue Posix is used exclusively for sending webhooks and update alerts in order to conserve RAM and reduce writes / wear
  PublishQueuePosix::instance().setup();                               // Start the Publish Queie

  // Next - check to make sure we are not in an endless update loop
  if (current.updateAttempts >= 3) {
    char data[64];
    System.disableUpdates();                                           // We will only try to update three times in a day
    current.alerts = 23;                                                // Set an alert that we have maxed out our updates for the day
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
  }

  // Next we set the timezone and check is we are in daylight savings time
  Time.setDSTOffset(sysStatus.dstOffset);                              // Set the value from FRAM if in limits
  isDSTusa() ? Time.beginDST() : Time.endDST();                        // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                       // Set the Time Zone for our device
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string
  Log.info("From setup DST offset is %s",currentOffsetStr);

  // If  the user is holding the user button - we will load defaults
  if (!digitalRead(userSwitch)) setLowPowerMode("0");                   // The user button can make sure the device connects (unless it is during closing hours)

  // Strings make it easier to read the system values in the console / mobile app
  makeUpStringMessages();                                              // Updated system settings - refresh the string messages

  // Make sure we have the right power settings
  setPowerConfig();                                                    // Executes commands that set up the Power configuration between Solar and DC-Powered

  // Here is where the code diverges based on why we are running Setup()
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over  
  if (Time.day() != Time.day(current.lastOccupancyChange)) {           // Check to see if the device was last on in a different day
    resetEverything();                                                 // Zero the counts for the new day
  }

  takeMeasurements();                                                  // Populates values so you can read them before the hour
  if (sysStatus.lowBatteryMode) setLowPowerMode("1");                  // If battery is low we need to go to low power state

  if ((Time.hour() >= sysStatus.openTime) && (Time.hour() < sysStatus.closeTime)) { // Park is open let's get ready for the day
    sensorControl(true);                                               // Turn on the sensor
    current.occupancyStatus = false;                                   // Reset at power up
    attachInterrupt(intPin, sensorISR, RISING);                        // Sensor interrupt from low to high
    stayAwake = stayAwakeLong;                                         // Keeps Boron awake after reboot - helps with recovery
    lastReportedTime = current.lastOccupancyChange;                    // When did we last see a change in occupancy
    if (!sysStatus.lowPowerMode) state = CONNECTING_STATE;             // If we are not in low power mode, we should connect
  }

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;               // IDLE unless otherwise from above code

  systemStatusWriteNeeded = true;                                      // Update FRAM with any changes from setup
  Log.info("Startup complete with version %s in %s mode", currentPointRelease, OFFLINEMODE ? "Off Line" : "Normal");
  digitalWrite(blueLED,LOW);                                           // Signal the end of startup
}


void loop()
{
  switch(state) {
  case IDLE_STATE:                                                     // Where we spend most time - note, the order of these conditionals is important
    if (state != oldState) {
      publishStateTransition();
      Log.info("Idle state - Hours are %i, open is %i, close is %i and hour of last report is %i",Time.hour(), sysStatus.openTime, sysStatus.closeTime, Time.hour(lastReportedTime));
    } 
    if (sensorDetect) serviceSensorEvent();                           // The ISR had raised the sensor flag - we can service it here as we don't need to catch every one like when counting cars
    if (current.occupancyStatus && (Time.now() >= current.lastOccupancyTime + current.debounceMin * 60)) serviceDebounceEvent();   // Ran out of time waiting for next event - court now unoccupied
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;         // When in low power mode, we can nap between taps
    if (Time.hour() != Time.hour(lastReportedTime)) {
        stayAwake = stayAwakeLong;                                    // Keeps device awake after reboot - helps with recovery
        state = REPORTING_STATE;                                      // We want to report on the hour but not after bedtime
    }     
    if (timeToSleep()) state = SLEEPING_STATE;                        // The park is closed - sleep
    break;

  case SLEEPING_STATE: {                                               // This state is triggered once the park closes and runs until it opens - Sensor is off and interrupts disconnected
    if (state != oldState) publishStateTransition();
    detachInterrupt(intPin);                                           // Done sensing for the day
    sensorControl(false);                                              // Turn off the sensor module for the hour
    if (current.occupancyStatus) {                                     // If the court is still showing as occupied, we need to wait
      serviceDebounceEvent();                                          // Reset for the day
      break;
    }
    if (Particle.connected() || !Cellular.isOff()) disconnectFromParticle();  // Disconnect cleanly from Particle
    stayAwake = 1000;                                                  // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    state = IDLE_STATE;                                                // Head back to the idle state after we sleep
    ab1805.stopWDT();                                                  // No watchdogs interrupting our slumber
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .duration(wakeInSeconds * 1000);
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device continues operations from here
    ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
    fuelGauge.wakeup();                                                // Make sure the fuelGauge is woke
    stayAwakeTimeStamp = millis();
    if (result.wakeupPin() == userSwitch) {                            // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
      setLowPowerMode("0");                                            // We are waking the device for a reason
      Log.info("Resetting opening hours");
      sysStatus.openTime = 0;                                          // This is for the edge case where the clock is not set and the device won't connect as it thinks it is off hours
      sysStatus.closeTime = 24;                                        // This only resets if the device beleives it is off-hours
      stayAwakeTimeStamp = millis();
      stayAwake = stayAwakeLong;
      systemStatusWriteNeeded = true;
    }
    else if (Time.hour() < sysStatus.closeTime && Time.hour() >= sysStatus.openTime) { // We might wake up and find it is opening time.  Park is open let's get ready for the day
      sensorControl(true);                                             // Turn off the sensor module for the hour
      attachInterrupt(intPin, sensorISR, RISING);                      // Pressure Sensor interrupt from low to high
      stayAwake = stayAwakeLong;                                       // Keeps Boron awake after deep sleep - may not be needed
    }
    } break;

  case NAPPING_STATE: {                                                // This state puts the device in low power mode quickly - napping supports the sensor activity and interrupts
    int wakeInSeconds = 0;
    if (state != oldState) publishStateTransition();
    stayAwake = 1000;                                                  // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    state = IDLE_STATE;                                                // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    ab1805.stopWDT();                                                  // If we are sleeping, we will miss petting the watchdog
    if (current.occupancyStatus) {                                     // We will set different sleep times based on whether the court is occupied or not
      wakeInSeconds = constrain((current.debounceMin* 60 - (Time.now() - current.lastOccupancyTime)),1,current.debounceMin*60); // Need to calc based on delay
    }
    else wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);  // Not occupied so wait till the next hour
    if (sysStatus.stateOfCharge > 65) {                                // Will stay on network standby if we have the power
      Log.info("Napping with radio on");
      config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .gpio(intPin,RISING)
      .duration(wakeInSeconds * 1000)
      .network(NETWORK_INTERFACE_CELLULAR, SystemSleepNetworkFlag::INACTIVE_STANDBY);           // Not sure how long we will sleep so need to keep the network active - 14mA power 
      if (Particle.connected()) Particle.disconnect();                // Disconnects from Particle but not from the cellular network
    }
    else {                                                            // Else we will drop off the network
      Log.info("Napping with radio off");
      config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .gpio(intPin,RISING)
      .duration(wakeInSeconds * 1000);
      if (Particle.connected() || Cellular.isOn()) disconnectFromParticle();  // This will turn off the cellular radio
    }
    delay(200);                     // Time to write to the Log
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep
    Log.info("Waking");
    ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
    fuelGauge.wakeup();                                                // Make sure the fuelGauge is woke
    stayAwakeTimeStamp = millis();
    if (result.wakeupPin() == userSwitch) setLowPowerMode("0");        // The user woke the device and we need to make sure it stays awake
    } break;

  case CONNECTING_STATE:{                                              // Will connect - or not and head back to the Idle state
    static State retainedOldState;                                     // Keep track for where to go next (depends on whether we were called from Reporting)
    static unsigned long connectionStartTimeStamp;                     // Time in Millis that helps us know how long it took to connect

    if (state != oldState) {                                           // Non-blocking function - these are first time items
      retainedOldState = oldState;                                     // Keep track for where to go next
      sysStatus.lastConnectionDuration = 0;                            // Will exit with 0 if we do not connect or are connected or the connection time if we do
      publishStateTransition();


      // Let's make sure we need to connect
      if (Particle.connected() || OFFLINEMODE) {                       // If we are already connected or we are in off-line mode we don't need to connect
        Log.info("Connecting state but already connected");
        stayAwakeTimeStamp = millis();
        (retainedOldState == REPORTING_STATE) ? state = RESP_WAIT_STATE : state = IDLE_STATE;
        break;
      }

      // If we are in a low battery state - we are not going to connect unless we are over-riding with user switch (active low)
      if (sysStatus.lowBatteryMode && digitalRead(userSwitch)) {
        Log.info("Connecting state but low battery mode");
        state = IDLE_STATE;
        break;
      }
      // If we are in low power mode, we may bail if battery is too low and we need to reduce reporting frequency
      if (sysStatus.lowPowerMode && digitalRead(userSwitch)) {         // Low power mode and user switch not pressed
        if (sysStatus.stateOfCharge <= 50 && (Time.hour() % 3)) {      // If the battery level is <50%, only connect every third hour
          Log.info("Connecting but <50%% charge - three hour schedule");
          state = IDLE_STATE;                                          // Will send us to connecting state - and it will send us back here
          break;
        }                                                              // Leave this state and go connect - will return only if we are successful in connecting
        else if (sysStatus.stateOfCharge <= 65 && (Time.hour() - sysStatus.lastConnection < 3600)) { // If the battery level is 50% -  65%, connect only once an hour
          Log.info("Connecting but 50-65%% charge - two hour schedule");
          state = IDLE_STATE;                                          // Will send us to connecting state - and it will send us back here
          break;                                                       // Leave this state and go connect - will return only if we are successful in connecting
        }
      }
      // OK, let's do this thing!
      connectionStartTimeStamp = millis();                             // Have to use millis as the clock will get reset on connect
      if (Cellular.isOff()) Cellular.on();                             // Needed until they fix this: https://github.com/particle-iot/device-os/issues/1631
      Particle.connect();                                              // Told the Particle to connect, now we need to wait
    }

    sysStatus.lastConnectionDuration = int((millis() - connectionStartTimeStamp)/1000);

    if (Particle.connected()) {
      if (!sysStatus.clockSet ) {
        sysStatus.clockSet = true;
        Particle.syncTime();                                           // Set the clock each day
        waitFor(Particle.syncTimeDone,30000);                          // Wait for up to 30 seconds for the SyncTime to complete
      }
      sysStatus.lastConnection = Time.now();                           // This is the last time we attempted to connect
      stayAwakeTimeStamp = millis();
      recordConnectionDetails();                                       // Record outcome of connection attempt
      (retainedOldState == REPORTING_STATE) ? state = RESP_WAIT_STATE : state = IDLE_STATE;
    }
    else if (sysStatus.lastConnectionDuration > connectMaxTimeSec) {
      recordConnectionDetails();                                       // Record outcome of connection attempt
      Log.info("cloud connection unsuccessful");
      disconnectFromParticle();                                        // Make sure the modem is turned off
      if (sysStatus.solarPowerMode) setLowPowerMode("1");              // If we cannot connect, there is no point to stayng out of low power mode
      if ((Time.now() - sysStatus.lastConnection) > 3 * 3600L) {       // Only sends to ERROR_STATE if it has been over three hours - this ties to reporting and low battery state
        state = ERROR_STATE;
        resetTimeStamp = millis();
        break;
      }
      else state = IDLE_STATE;
    }
    } break;

  case REPORTING_STATE:
    if (state != oldState) publishStateTransition();
    lastReportedTime = Time.now();                                    // We are only going to report once each hour from the IDLE state.  We may or may not connect to Particle
    takeMeasurements();                                               // Take Measurements here for reporting
    Log.info("Measurements taken - on to sending");
    if (Time.hour() == sysStatus.openTime) dailyCleanup();            // Once a day, clean house and publish to Google Sheets
    sendEvent();                                                      // Publish hourly but not at opening time as there is nothing to publish
    state = CONNECTING_STATE;                                         // We are only passing through this state once each hour
    break;

  case RESP_WAIT_STATE: {
    static unsigned long webhookTimeStamp = 0;                        // Webhook time stamp
    if (state != oldState) {
      webhookTimeStamp = millis();                                    // We are connected and we have published, head to the response wait state
      dataInFlight = true;                                            // set the data inflight flag
      publishStateTransition();
    }
    if (!dataInFlight || OFFLINEMODE)  {                              // Response received or we are in off-line mode --> back to IDLE state
      state = IDLE_STATE;
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      current.alerts = 40;                                            // Raise the missed webhook flag
      currentStatusWriteNeeded = true;
      if (Time.now() - sysStatus.lastHookResponse > 3 * 3600L) {      // Failed to get a webhook response for over three hours
        resetTimeStamp = millis();
        state = ERROR_STATE;                                          // Response timed out
      } 
      else state = IDLE_STATE;
    }
    } break;

  case ERROR_STATE: {                                                  // New and improved - now looks at the alert codes
    if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
    if (millis() - resetTimeStamp > resetWait) {                       // This simply gives us some time to catch the device if it is in a reset loop
      if (Particle.connected()) {                                      // If we are connected to Particle - let's publish to let folks know what is going on
        char errorStr[64];
        snprintf(errorStr, sizeof(errorStr),"Resetting device with alert code %i",current.alerts);
        Particle.publish("ERROR_STATE", errorStr, PRIVATE);
        Log.info(errorStr);
        delay(2000);
        disconnectFromParticle();                                      // Since we are resetting, let's disconnect cleanly
      }

      switch (current.alerts) {                                        // For now, no default state as there are only a few paths that lead to this state
        case 12:                                                       // This is an initialization error - likely FRAM - need to power cycle to clear
          ab1805.deepPowerDown();                                      // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
          break;

        case 30 ... 31:                                                // Device failed to connect too many times
          sysStatus.lastConnection = Time.now();                       // Make sure we don't do this very often
          current.alerts = 0;
          fram.put(FRAM::currentStateAddr,current);
          fram.put(FRAM::systemStatusAddr,sysStatus);                  // Unless a FRAM error sent us here - store alerts value
          delay(100);                                                  // Time to write to FRAM
          System.reset();  
          break;

        case 13:                                                       // Excessive resets of the device - time to power cycle
          sysStatus.resetCount = 0;                                    // Reset so we don't do this too often
          current.alerts = 0;
          fram.put(FRAM::currentStateAddr,current);
          fram.put(FRAM::systemStatusAddr,sysStatus);                  // Won't get back to the main loop
          delay (100);
          ab1805.deepPowerDown();                                      // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
          break;

        case 14:                                                       // This is an out of memory error
          System.reset();
          break;

        case 40:                                                       // This is for failed webhook responses over three hours
          System.reset();
          break;

        default:                                                        // Make sure that, no matter what - we do not get stuck here
          System.reset();
          break;
      }
    }
    } break;

  }
  // Take care of housekeeping items here

  ab1805.loop();                                                       // Keeps the RTC synchronized with the Boron's clock

  PublishQueuePosix::instance().loop();                                // Check to see if we need to tend to the message queue

  if (systemStatusWriteNeeded) {                                       // These flags get set when a value is changed
    fram.put(FRAM::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (currentStatusWriteNeeded) {
    fram.put(FRAM::currentStateAddr,current);
    currentStatusWriteNeeded = false;
  }

  if (outOfMemory >= 0) {                                              // In this function we are going to reset the system if there is an out of memory error
    current.alerts = 14;                                               // Out of memory alert
    resetTimeStamp = millis();
    state = ERROR_STATE;
  }
  // End of housekeeping - end of main loop
}

bool timeToSleep() {          // Returns true if we should be sleeping
  if((Time.hour() >= sysStatus.closeTime) || (Time.hour() < sysStatus.openTime)) return true;
  else return false;
}

void sensorControl(bool enableSensor) {                               // What is the sensor type - 0-Pressure Sensor, 1-PIR Sensor

  if (enableSensor) {
    digitalWrite(disableModule,false);                                // Enable or disable the sensor

    if (sysStatus.sensorType == 0) {                                  // This is the pressure sensor and we are enabling it
        digitalWrite(ledPower,HIGH);                                  // For the pressure sensor, this is how you activate it
    }
    else {
        digitalWrite(ledPower,LOW);                                   // Turns on the LED on the PIR sensor board
    }
  }

  else { 
    digitalWrite(disableModule,true);

    if (sysStatus.sensorType == 0) {                                  // This is the pressure sensor and we are enabling it
        digitalWrite(ledPower,LOW);                                   // Turns off the LED on the pressure sensor board
    }
    else {
        digitalWrite(ledPower,HIGH);                                  // Turns off the LED on the PIR sensor board
    }
  }

}

void  recordConnectionDetails()  {                                     // Whether the connection was successful or not, we will collect and publish metrics
  char data[64];

  if (sysStatus.lastConnectionDuration > connectMaxTimeSec+1) sysStatus.lastConnectionDuration = 0;
  else if (sysStatus.lastConnectionDuration > current.maxConnectTime) current.maxConnectTime = sysStatus.lastConnectionDuration; // Keep track of longest each day

  if (Cellular.ready()) getSignalStrength();                           // Test signal strength if the cellular modem is on and ready

  snprintf(data, sizeof(data),"Connected in %i secs",sysStatus.lastConnectionDuration);                   // Make up connection string and publish
  Log.info(data);

  if (Particle.connected()) {
    Log.info("Cloud connection successful");
    if (sysStatus.verboseMode) Particle.publish("Cellular",data,PRIVATE);
  }
  else if (Cellular.ready()) {                                        // We want to take note of this as it implies an issue with the Particle back-end
    Log.info("Connected to cellular but not Particle");
    current.alerts = 30;                                              // Record alert for timeout on Particle but connected to cellular
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
  }
  else {
    Log.info("Failed to connect");
    current.alerts = 31;                                              // Record alert for timeout on cellular
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
  }

  systemStatusWriteNeeded = true;
  currentStatusWriteNeeded = true;
}

void serviceSensorEvent()                                             // We only come here if the PIR sensor triggered an interrupt
{
  current.lastOccupancyTime = Time.now();                             // This will help us know how long to hold occupied state

  if (!current.occupancyStatus) {                                     // You can get multiple PIR events while the court is occupied - but we need to do some extra things if state was previously not occupied
    pinSetFast(blueLED);                                              // Turn on the blue LED
    current.occupancyStatus = true;
    snprintf(occupancyStateStr, sizeof(occupancyStateStr), "Occupied");  // Update the string for the Particle variable
    Log.info(occupancyStateStr);
    current.lastOccupancyChange = Time.now();
    state = REPORTING_STATE;                                          // Occupancy state changed - need to report
  }
  sensorDetect = false;                                               // Reset the flag
  currentStatusWriteNeeded = true;                                    // Write updated values to FRAM
}


void serviceDebounceEvent() {                                         // We get here when it has been over the debounce period and we are in occupied state
  current.occupancyStatus = false;
  snprintf(occupancyStateStr, sizeof(occupancyStateStr), "Not Occupied");  // Update the string for the Particle variable
  Log.info(occupancyStateStr);
  int newMinutes = (Time.now() - current.lastOccupancyChange)/60;
  current.lastOccupancyChange = Time.now();
  current.dailyOccupancyMinutes += newMinutes;
  currentStatusWriteNeeded = true;
  digitalWrite(blueLED,LOW);
  state = REPORTING_STATE;
}


void sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global

  snprintf(data, sizeof(data), "{\"occupancy\":%i, \"dailyoccupancy\":%i, \"battery\":%i,\"key1\":\"%s\",\"temp\":%i, \"resets\":%i, \"alerts\":%i,\"connecttime\":%i,\"timestamp\":%lu000}",current.occupancyStatus, current.dailyOccupancyMinutes, sysStatus.stateOfCharge, batteryContext[sysStatus.batteryState], current.temperature, sysStatus.resetCount, current.alerts, sysStatus.lastConnectionDuration, Time.now());
  PublishQueuePosix::instance().publish("Ubidots-Sensor-Hook-v1", data, (PRIVATE | WITH_ACK));
  Log.info("Ubidots Webhook: %s", data);                              // For monitoring via serial
  current.alerts = 0;                                                 // Reset the alert after publish
}

void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response
  char responseString[64];
    // Response is only a single number thanks to Template
  if (!strlen(data)) {                                                // No data in response - Error
    snprintf(responseString, sizeof(responseString),"No Data");
  }
  else if (atoi(data) == 200 || atoi(data) == 201) {
    snprintf(responseString, sizeof(responseString),"Response Received");
    sysStatus.lastHookResponse = Time.now();                          // Record the last successful Webhook Response
    systemStatusWriteNeeded = true;
    dataInFlight = false;                                             // Data has been received
  }
  else {
    snprintf(responseString, sizeof(responseString), "Unknown response recevied %i",atoi(data));
  }
  // Particle.publish("Ubidots Hook", responseString, PRIVATE);
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  getTemperature();                                                    // Get Temperature at startup as well

  sysStatus.batteryState = System.batteryState();                      // Call before isItSafeToCharge() as it may overwrite the context

  isItSafeToCharge();                                                  // See if it is safe to charge

  if (sysStatus.lowPowerMode) {                                        // Need to take these steps if we are sleeping
    fuelGauge.quickStart();                                            // May help us re-establish a baseline for SoC
    delay(500);
  }

  sysStatus.stateOfCharge = int(fuelGauge.getSoC());                   // Assign to system value

  if (sysStatus.stateOfCharge < 65 && sysStatus.batteryState == 1) {
    System.setPowerConfiguration(SystemPowerConfiguration());          // Reset the PMIC
    current.alerts = 11;                                               // Keep track of this
  }

  if (sysStatus.stateOfCharge < current.minBatteryLevel) {
    current.minBatteryLevel = sysStatus.stateOfCharge;                 // Keep track of lowest value for the day
    currentStatusWriteNeeded = true;
  }

  if (sysStatus.stateOfCharge < 30) {
    sysStatus.lowBatteryMode = true;                                   // Check to see if we are in low battery territory
    if (!sysStatus.lowPowerMode) setLowPowerMode("1");                 // Should be there already but just in case...
  }
  else sysStatus.lowBatteryMode = false;                               // We have sufficient to continue operations

  systemStatusWriteNeeded = true;
}

bool isItSafeToCharge()                                                // Returns a true or false if the battery is in a safe charging range.
{
  PMIC pmic(true);
  if (current.temperature < 36 || current.temperature > 100 )  {       // Reference: https://batteryuniversity.com/learn/article/charging_at_high_and_low_temperatures (32 to 113 but with safety)
    pmic.disableCharging();                                            // It is too cold or too hot to safely charge the battery
    sysStatus.batteryState = 1;                                        // Overwrites the values from the batteryState API to reflect that we are "Not Charging"
    current.alerts = 10;                                                // Set a value of 1 indicating that it is not safe to charge due to high / low temps
    return false;
  }
  else {
    pmic.enableCharging();                                             // It is safe to charge the battery
    if (current.alerts == 10) current.alerts = 0;                      // Reset the alerts flag if we previously had disabled charging
    return true;
  }
}

void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  current.temperature = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit
  currentStatusWriteNeeded=true;
  return current.temperature;
}


// Here are the various hardware and timer interrupt service routines
void outOfMemoryHandler(system_event_t event, int param) {
    outOfMemory = param;
}


void sensorISR()
{
  sensorDetect = true;                                              // sets the sensor flag for the main loop
}

// Power Management function
int setPowerConfig() {
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration
  if (sysStatus.solarPowerMode) {
    conf.powerSourceMaxCurrent(900) // Set maximum current the power source can provide (applies only when powered through VIN)
        .powerSourceMinVoltage(5080) // Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(1024) // Set battery charge current
        .batteryChargeVoltage(4208) // Set battery termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST); // For the cases where the device is powered through VIN
                                                                     // but the USB cable is connected to a USB host, this feature flag
                                                                     // enforces the voltage/current limits specified in the configuration
                                                                     // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
  else  {
    conf.powerSourceMaxCurrent(900)                                   // default is 900mA 
        .powerSourceMinVoltage(4208)                                  // This is the default value for the Boron
        .batteryChargeCurrent(900)                                    // higher charge current from DC-IN when not solar powered
        .batteryChargeVoltage(4112)                                   // default is 4.112V termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST) ;
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
}


void loadSystemDefaults() {                                         // Default settings for the device - connected, not-low power and always on
  if (Particle.connected()) Particle.publish("Mode","Loading System Defaults", PRIVATE);
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = false;
  sysStatus.clockSet = false;
  sysStatus.lowBatteryMode = false;
  setLowPowerMode("1");
  sysStatus.timezone = -5;                                          // Default is East Coast Time
  sysStatus.dstOffset = 1;
  sysStatus.openTime = 6;
  sysStatus.closeTime = 21;
  sysStatus.lastConnectionDuration = 0;                             // New measure
  fram.put(FRAM::systemStatusAddr,sysStatus);                       // Write it now since this is a big deal and I don't want values over written
}

void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range 
  sysStatus.sensorType = 1;                                         // These are all PIR sensors     
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (sysStatus.openTime < 0 || sysStatus.openTime > 12) sysStatus.openTime = 0;
  if (sysStatus.closeTime < 12 || sysStatus.closeTime > 24) sysStatus.closeTime = 24;  
  if (current.debounceMin < 1 || current.debounceMin > 10) current.debounceMin = 5;
  if (sysStatus.lastConnectionDuration > connectMaxTimeSec) sysStatus.lastConnectionDuration = 0;
  systemStatusWriteNeeded = true;
  currentStatusWriteNeeded = true;
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

 /**
  * @brief Simple Function to construct a string for the Open and Close Time
  * 
  * @details Looks at the open and close time and makes them into time strings.  Also looks at the special case of open 24 hours
  * and puts in an "NA" for both strings when this is the case.
  * 
  */
void makeUpParkHourStrings() {
  if (sysStatus.openTime == 0 && sysStatus.closeTime == 24) {
    snprintf(openTimeStr, sizeof(openTimeStr), "NA");
    snprintf(closeTimeStr, sizeof(closeTimeStr), "NA");
    return;
  }
  
  snprintf(openTimeStr, sizeof(openTimeStr), "%i:00", sysStatus.openTime);
  snprintf(closeTimeStr, sizeof(closeTimeStr), "%i:00", sysStatus.closeTime);
  return;
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

 /**
  * @brief Simple Function to construct the strings that make the console easier to read
  *
  * @details Looks at all the system setting values and creates the appropriate strings.  Note that this
  * is a little inefficient but it cleans up a fair bit of code.
  *
  */
void makeUpStringMessages() {

  if (sysStatus.openTime == 0 && sysStatus.closeTime == 24) {                                 // Special case for 24 hour operations
    snprintf(openTimeStr, sizeof(openTimeStr), "NA");
    snprintf(closeTimeStr, sizeof(closeTimeStr), "NA");
  }
  else {
    snprintf(openTimeStr, sizeof(openTimeStr), "%i:00", sysStatus.openTime);                  // Open and Close Times
    snprintf(closeTimeStr, sizeof(closeTimeStr), "%i:00", sysStatus.closeTime);
  }

  if (sysStatus.lowPowerMode) strncpy(lowPowerModeStr,"Low Power", sizeof(lowPowerModeStr));  // Low Power Mode Strings
  else strncpy(lowPowerModeStr,"Not Low Power", sizeof(lowPowerModeStr));

  return;
}


bool disconnectFromParticle()                                          // Ensures we disconnect cleanly from Particle
                                                                       // Updated based on this thread: https://community.particle.io/t/waitfor-particle-connected-timeout-does-not-time-out/59181
{
  Log.info("In the disconnect from Particle function");
  Particle.disconnect();
  waitFor(Particle.disconnected, 15000);                               // make sure before turning off the cellular modem
  Cellular.disconnect();                                               // Disconnect from the cellular network
  Cellular.off();                                                      // Turn off the cellular modem
  waitFor(Cellular.isOff, 30000);                                      // As per TAN004: https://support.particle.io/hc/en-us/articles/1260802113569-TAN004-Power-off-Recommendations-for-SARA-R410M-Equipped-Devices
  systemStatusWriteNeeded = true;
  return true;
}

int resetCounts(String command)                                       // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    current.longestOccupancyMinutes = 0;                              // Reset Daily Count in memory
    sysStatus.resetCount = 0;                                         // If so, store incremented number - watchdog must have done This
    current.alerts = 0;                                               // Reset count variables
    dataInFlight = false;
    currentStatusWriteNeeded = true;                                  // Make sure we write to FRAM back in the main loop
    systemStatusWriteNeeded = true;
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)                                      // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    if (Particle.connected()) Particle.publish("Reset","Hard Reset in 2 seconds",PRIVATE);
    ab1805.deepPowerDown(10);
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

/**
 * @brief Resets all counts to start a new day.
 * 
 * @details Once run, it will reset all daily-specific counts and trigger an update in FRAM.
 */
void resetEverything() {                                              // The device is waking up in a new day or is a new install
  char data[64];
  Log.info("Resetting everything");
  current.longestOccupancyMinutes = 0;                                // Reset the counts in FRAM as well
  current.dailyOccupancyMinutes = 0;                                  // Reset for the day
  current.lastOccupancyChange = Time.now();                           // Set the time context to the new day
  current.occupancyStatus = false;                                    // Reset at closing time
  if (current.alerts == 23 || current.updateAttempts >=3) {           // We had tried to update enough times that we disabled updates for the day - resetting
    System.enableUpdates();
    current.alerts = 0;
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
  }
  current.updateAttempts = 0;                                         // Reset the update attempts counter for the day
  currentStatusWriteNeeded=true;                                      // Make sure that the values are updated in FRAM

  sysStatus.resetCount = 0;                                           // Reset the reset count as well
  systemStatusWriteNeeded = true;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.solarPowerMode = true;
    setPowerConfig();                                               // Change the power management Settings
    systemStatusWriteNeeded=true;
    if (Particle.connected()) Particle.publish("Mode","Set Solar Powered Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.solarPowerMode = false;
    systemStatusWriteNeeded=true;
    setPowerConfig();                                                // Change the power management settings
    if (Particle.connected()) Particle.publish("Mode","Cleared Solar Powered Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

/**
 * @brief Turns on/off verbose mode.
 * 
 * @details Extracts the integer command. Turns on verbose mode if the command is "1" and turns
 * off verbose mode if the command is "0".
 *
 * @param command A string with the integer command indicating to turn on or off verbose mode.
 * Only values of "0" or "1" are accepted. Values outside this range will cause the function
 * to return 0 to indicate an invalid entry.
 * 
 * @return 1 if successful, 0 if invalid command
 */
int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) Particle.publish("Mode","Set Verbose Mode", PRIVATE);
    sensorControl(true);                                    // Make sure the sensor is on and correctly configured
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) Particle.publish("Mode","Cleared Verbose Mode", PRIVATE);
    sensorControl(true);                                      // Make sure the sensor is on and correctly configured
    return 1;
  }
  else return 0;
}

/**
 * @brief Returns a string describing the battery state.
 * 
 * @return String describing battery state.
 */
String batteryContextMessage() {
  return batteryContext[sysStatus.batteryState];
}

/**
 * @brief Sets the closing time of the facility.
 * 
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                                    // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                            // Make sure it falls in a valid range or send a "fail" result
  sysStatus.openTime = tempTime;
  makeUpParkHourStrings();                                                    // Create the strings for the console
  systemStatusWriteNeeded = true;                                            // Need to store to FRAM back in the main loop
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Open time set to %i",sysStatus.openTime);
    Particle.publish("Time",data, PRIVATE);
  }
  return 1;
}

/**
 * @brief Sets the closing time of the facility.
 * 
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.closeTime = tempTime;
  makeUpParkHourStrings();                                                    // Create the strings for the console
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",sysStatus.closeTime);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  return 1;
}

/**
 * @brief Sets the debounce period for occupancy - useful when you are replacing sensors.
 * 
 * @details This is how we set the number of minutes we will wait before assuming that the 
 * area is unoccupied.  
 *
 * @param command A string for the new daily count.  
 * Inputs outside of "0" - "10" will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setDebounceMin(String command)
{
  char * pEND;
  char data[256];
  int tempCount = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempCount < 0) || (tempCount > 10)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  current.debounceMin = tempCount;
  currentStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Debounce set to  %imin",current.debounceMin);
  if (Particle.connected()) Particle.publish("Daily",data, PRIVATE);
  return 1;
}

/**
 * @brief Toggles the device into low power mode based on the input command.
 * 
 * @details If the command is "1", sets the device into low power mode. If the command is "0",
 * sets the device into normal mode. Fails if neither of these are the inputs.
 *
 * @param command A string indicating whether to set the device into low power mode or into normal mode.
 * A "1" indicates low power mode, a "0" indicates normal mode. Inputs that are neither of these commands
 * will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    if (Particle.connected()) {
      Particle.publish("Mode","Low Power Mode", PRIVATE);
    }
    sysStatus.lowPowerMode = true;
    strncpy(lowPowerModeStr,"True", sizeof(lowPowerModeStr));
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    if (!Particle.connected()) {                                      // In case we are not connected, we will do so now.
      state = CONNECTING_STATE;
    }
    if (Particle.connected())Particle.publish("Mode","Normal Operations", PRIVATE);
    sysStatus.lowPowerMode = false;                                   // update the variable used for console status
    strncpy(lowPowerModeStr,"False", sizeof(lowPowerModeStr));        // Use capitalization so we know that we set this.
  }
  systemStatusWriteNeeded = true;
  return 1;
}

/**
 * @brief Publishes a state transition over serial and to the Particle/Unidash monitoring system.
 * 
 * @details A good debugging tool.
 */
void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected() && sysStatus.verboseMode) Particle.publish("State Transition",stateTransitionString, PRIVATE);
  Log.info(stateTransitionString);
}

/**
 * @brief Fully resets modem.
 * 
 * @details Disconnects from the cloud, resets modem and SIM, and deep sleeps for 10 seconds.
 * Adapted form Rikkas7's https://github.com/rickkas7/electronsample.
 */
void fullModemReset() {  // 
	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=15\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}

/**
 * @brief Cleanup function that is run at the end of the day.
 * 
 * @details Syncs time with remote service and sets low power mode. Called from Reporting State ONLY.
 * Clean house at the end of the day
 */
void dailyCleanup() {
  if (Particle.connected()) Particle.publish("Daily Cleanup","Running", PRIVATE);  // Make sure this is being run
  Log.info("Running daily cleanup");
  sysStatus.verboseMode = false;
  sysStatus.clockSet = false;
  isDSTusa() ? Time.beginDST() : Time.endDST();                        // Perform the DST calculation here - once a day
  if (sysStatus.solarPowerMode || sysStatus.stateOfCharge <= 70) {     // If Solar or if the battery is being discharged
    setLowPowerMode("1");
  }
  resetEverything();                                                   // If so, we need to Zero the counts for the new day
  systemStatusWriteNeeded = true;
}