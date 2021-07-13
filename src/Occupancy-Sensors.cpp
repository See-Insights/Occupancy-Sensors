/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
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



// Particle Product definitions
void setup();
void loop();
void sensorControl(bool enableSensor);
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
bool disconnectFromParticle();
bool notConnected();
int resetCounts(String command);
int hardResetNow(String command);
int sendNow(String command);
void resetEverything();
int setSolarMode(String command);
int setSensorType(String command);
int setVerboseMode(String command);
String batteryContextMessage();
int setOpenTime(String command);
int setCloseTime(String command);
int setDebounceMin(String command);
int setLowPowerMode(String command);
void publishStateTransition(void);
void fullModemReset();
void dailyCleanup();
#line 25 "/Users/chipmc/Documents/Maker/Particle/Projects/Occupancy-Sensors/src/Occupancy-Sensors.ino"
PRODUCT_ID(PLATFORM_ID);                            // No longer need to specify - but device needs to be added to product ahead of time.
PRODUCT_VERSION(5);
#define DSTRULES isDSTusa
char currentPointRelease[5] ="5.00";

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentStateAddr     = 0x50                    // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 1;                    // Increment this number each time the memory map is changed


struct currentStatus_structure {                    // currently 10 bytes long
  bool occupancyStatus;                             // Occupied or not
  time32_t lastOccupancyTime;                  // When as it last occupied?
  time32_t lastOccupancyChange;                // When did the occupancy status last change
  int dailyOccupancyMinutes;                        // How many minutes has it been occupied today
  int longestOccupancyMinutes;                      // What was the longest it was occupied today
  int debounceMin;                                  // What is the time we will wait before assuming no occupancy
  int temperature;                                  // Current Temperature
  int alertCount;                                   // What is the current alert count
  uint16_t maxConnectTime = 0;                      // Longest connect time for the day
  int minBatteryLevel = 100;                        // Lowest Battery level for the day
} current;


// Included Libraries
#include "3rdGenDevicePinoutdoc.h"                  // Pinout Documentation File
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "PublishQueueAsyncRK.h"                    // Async Particle Publish

// Libraries with helper functions
#include "time_zone_fn.h"
#include "sys_status.h"

struct systemStatus_structure sysStatus;

// This is the maximum amount of time to allow for connecting to cloud. If this time is
// exceeded, do a deep power down. This should not be less than 10 minutes. 11 minutes
// is a reasonable value to use.
unsigned long connectMaxTimeSec = 11 * 60;   // Timeout for trying to connect to Particle cloud in seconds


// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
retained uint8_t publishQueueRetainedBuffer[2048];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library

// For monitoring / debugging, you can uncomment the next line
SerialLogHandler logHandler(LOG_LEVEL_ALL);

// State Maching Variables
enum State { INITIALIZATION_STATE, CONNECTING_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Connecting", "Error", "Idle", "Sleeping", "Napping", "Reporting", "Response Wait" };
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
const int intPin =        SCK;                      // Pressure Sensor inerrupt pin
const int disableModule = MOSI;                     // Bringining this low turns on the sensor (pull-up on sensor board)
const int ledPower =      MISO;                     // Allows us to control the indicator LED on the sensor board

// Timing Variables
bool dataInFlight;                            
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
char sensorTypeConfigStr[16];
unsigned long connectionStartTime;
unsigned long connectionTimeout = 11 * 60 * 1000;             // Timeout for trying to connect to Particle cloud in milliseconds

// Program Variables
volatile bool watchdogFlag;                         // Flag to let us know we need to pet the dog
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char batteryContextStr[16];                         // Tracks the battery context
char occupancyStateStr[16];                         // Text for occupancy
char lowPowerModeStr[6];                            // In low power mode?
char openTimeStr[8]="NA";                           // Park Open Time
char closeTimeStr[8]="NA";                          // Park close Time
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write
bool currentStatusWriteNeeded = false;
bool particleConnectionNeeded = false;              // Do we need to connect to Particle

// These variables are associated with the watchdog timer and will need to be better integrated
int outOfMemory = -1;
time_t RTCTime;

// This section is where we will initialize sensor specific variables, libraries and function prototypes
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered

void setup()                                        // Note: Disconnected Setup()
{
  /* Setup is run for three reasons once we deploy a sensor:
       1) When you deploy the sensor
       2) Each hour while the device is sleeping
       3) After a reset event
    All three of these have some common code - this will go first then we will set a conditional
    to determine which of the three we are in and finish the code
  */
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

  Particle.variable("OccupancyState", occupancyStateStr);         // Define my Particle variables
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Temperature",current.temperature);
  Particle.variable("Release",currentPointRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  Particle.variable("OpenTime", openTimeStr);
  Particle.variable("CloseTime",closeTimeStr);
  Particle.variable("Alerts",current.alertCount);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("BatteryContext",batteryContextMessage);
  Particle.variable("SensorStatus",sensorTypeConfigStr);
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
  Particle.function("Set-SensorType",setSensorType);

  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));  // Don't disconnect abruptly

  // Load FRAM and reset variables to their correct values
  fram.begin();                                                       // Initialize the FRAM module

  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                             // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                     // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                   // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                         // See if this worked
    if (tempVersion != FRAMversionNumber) state = ERROR_STATE;        // Device will not work without FRAM
    else loadSystemDefaults();                                        // Out of the box, we need the device to be awake and connected
  }
  else {
    fram.get(FRAM::systemStatusAddr,sysStatus);                       // Loads the System Status array from FRAM
    fram.get(FRAM::currentStateAddr,current);                         // Loead the current values array from FRAM
  }

  checkSystemValues();                                                // Make sure System values are all in valid range

  current.occupancyStatus = 0;                                      // Always start with no occupancy
  snprintf(occupancyStateStr, sizeof(occupancyStateStr), "%s", (current.occupancyStatus) ? "Occupied" : "Not Occupied");

  makeUpParkHourStrings();                                            // Create the strings for the console

  // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.
  System.on(out_of_memory, outOfMemoryHandler);

  // The carrier board has D8 connected to FOUT for wake interrupts
  ab1805.withFOUT(D8).setup();

  // Note whether the RTC is set 
  sysStatus.clockSet = ab1805.isRTCSet();

  // Enable watchdog
  ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);

  Time.setDSTOffset(sysStatus.dstOffset);                              // Set the value from FRAM if in limits

  DSTRULES() ? Time.beginDST() : Time.endDST();                        // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                       // Set the Time Zone for our device
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string

  (sysStatus.lowPowerMode) ? strncpy(lowPowerModeStr,"True",sizeof(lowPowerModeStr)) : strncpy(lowPowerModeStr,"False",sizeof(lowPowerModeStr));

  sensorControl(true);                                                // Turn on the sensors.

  if (sysStatus.sensorType == 0) strncpy(sensorTypeConfigStr,"Pressure Sensor",sizeof(sensorTypeConfigStr));
  else if (sysStatus.sensorType == 1) strncpy(sensorTypeConfigStr,"PIR Sensor",sizeof(sensorTypeConfigStr));

  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    systemStatusWriteNeeded = true;                                    // If so, store incremented number - watchdog must have done This
  }

  lastReportedTime = Time.now();                                  // Initialize it to now so that reporting can begin as soon as the hour changes

  setPowerConfig();                                                    // Executes commands that set up the Power configuration between Solar and DC-Powered

  if (!digitalRead(userSwitch)) loadSystemDefaults();                  // Make sure the device wakes up and connects

  // Here is where the code diverges based on why we are running Setup()
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over  
  if (Time.day() != Time.day(current.lastOccupancyChange)) {           // Check to see if the device was last on in a different day
    resetEverything();                                                 // Zero the counts for the new day
  }

  if ((Time.hour() >= sysStatus.openTime) && (Time.hour() < sysStatus.closeTime)) { // Park is open let's get ready for the day                                                            
    attachInterrupt(intPin, sensorISR, RISING);                       // Pressure Sensor interrupt from low to high
    if (sysStatus.connectedStatus && !Particle.connected()) {         // If the system thinks we are connected, let's make sure that we are
      particleConnectionNeeded = true;                                   // This may happen if there was an unexpected reset during park open hours
      sysStatus.connectedStatus = false;
    }
    takeMeasurements();                                               // Populates values so you can read them before the hour
    stayAwake = stayAwakeLong;                                        // Keeps Boron awake after reboot - helps with recovery
  }

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;              // IDLE unless otherwise from above code

  digitalWrite(blueLED,LOW);                                          // Signal the end of startup
}


void loop()
{
  switch(state) {

case CONNECTING_STATE:{
    static unsigned long connectionStartTime;
    char connectionStr[32];
    static bool returnToReporting;

    if (state != oldState) {
      if (oldState == REPORTING_STATE) returnToReporting = true;
      else returnToReporting = false;                                 // Need to set value each time - just to be clear
      publishStateTransition();
      connectionStartTime = Time.now();                               // Start the clock first time we enter the state
      Cellular.on();                                                  // Needed until they fix this: https://github.com/particle-iot/device-os/issues/1631
      Particle.connect();                                             // Told the Particle to connect, now we need to wait
    }

    if (Particle.connected()) {
      particleConnectionNeeded = false;                               // Connected so we don't need this flag
      sysStatus.connectedStatus = true;
      sysStatus.lastConnection = Time.now();                          // This is the last time we attempted to connect
      Log.info("Cloud connection successful");
    }
    else if ((Time.now() - connectionStartTime) > connectMaxTimeSec) {
      particleConnectionNeeded = false;                               // Timed out so we will give up until the next hour
      if ((Time.now() - sysStatus.lastConnection) > 7200) {             // Only sends to ERROR_STATE if it has been over 2 hours
        state = ERROR_STATE;     
        resetTimeStamp = millis();
        break;
      }
      sysStatus.connectedStatus = false;
      Log.info("cloud connection unsuccessful");
    } 

    if (!particleConnectionNeeded) {                                  // Whether the connection was successful or not, we will collect and publish metrics
      sysStatus.lastConnectionDuration = Time.now() - connectionStartTime;
      if (sysStatus.lastConnectionDuration > connectMaxTimeSec) sysStatus.lastConnectionDuration = connectMaxTimeSec;           // This is clearly an erroneous result
      if (sysStatus.lastConnectionDuration > current.maxConnectTime) current.maxConnectTime = sysStatus.lastConnectionDuration; // Keep track of longest each day
      snprintf(connectionStr, sizeof(connectionStr),"Connected in %i secs",sysStatus.lastConnectionDuration);                   // Make up connection string and publish
      Log.info(connectionStr);
      if (sysStatus.verboseMode) publishQueue.publish("Cellular",connectionStr,PRIVATE);
      systemStatusWriteNeeded = true;
      currentStatusWriteNeeded = true;
      if (sysStatus.connectedStatus && returnToReporting) state = REPORTING_STATE;    // If we came here from reporting, this will send us back
      else state = IDLE_STATE;                                         // We are connected so, we can go to the IDLE state
    }
  } break;

  case IDLE_STATE:                                                    // Where we spend most time - note, the order of these conditionals is important
    if (state != oldState) publishStateTransition();
    if (sensorDetect) serviceSensorEvent();                           // The ISR had raised the sensor flag - we can service it here as we don't need to catch every one like when counting cars
    if ((Time.now() > current.lastOccupancyTime + current.debounceMin * 60) && current.occupancyStatus) serviceDebounceEvent();   // Ran out of time waiting for next event - court now unoccupied
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;  // When in low power mode we can nap
    if (Time.hour() != Time.hour(lastReportedTime)) {
        stayAwake = stayAwakeLong;                                    // Keeps device awake after reboot - helps with recovery
        state = REPORTING_STATE;                                      // We want to report on the hour but not after bedtime
    } 
    if ((Time.hour() >= sysStatus.closeTime) || (Time.hour() < sysStatus.openTime)) state = SLEEPING_STATE;   // The park is closed - sleep
    if (particleConnectionNeeded) state = CONNECTING_STATE;           // Someone raised the connection neeeded flag - will return to IDLE once attempt is completed
    break;

  case SLEEPING_STATE: {                                              // This state is triggered once the park closes and runs until it opens
    if (state != oldState) publishStateTransition();
    detachInterrupt(intPin);                                          // Done sensing for the day
    sensorControl(false);                                             // Turn off the sensor module for the hour
    if (current.occupancyStatus) {                                    // If the space is occupied, delay sleep
      state = REPORTING_STATE;
      break;
    }
    if (sysStatus.connectedStatus) disconnectFromParticle();          // Disconnect cleanly from Particle
    ab1805.stopWDT();                                                 // No watchdogs interrupting our slumber
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .duration(wakeInSeconds * 1000);
    SystemSleepResult result = System.sleep(config);                  // Put the device to sleep device reboots from here   
    ab1805.resumeWDT();                                               // Wakey Wakey - WDT can resume
    if (result.wakeupPin() == userSwitch) {                           // If the user woke the device we need to get up
      setLowPowerMode("0");
      sysStatus.openTime = 0;
      sysStatus.closeTime = 24;
    }
    if (Time.day() != Time.day(current.lastOccupancyChange)) {         // We might wake up in a new day
      resetEverything();                                               // If so, we need to Zero the counts for the new day
      if (sysStatus.solarPowerMode && !sysStatus.lowPowerMode) {
        setLowPowerMode("1");                                          // If we are running on solar, we will reset to lowPowerMode at Midnight
      }
    }
    if (Time.hour() >= sysStatus.openTime) {                           // We might wake up and find it is opening time.  Park is open let's get ready for the day
      sensorControl(true);                                             // Turn off the sensor module for the hour
      attachInterrupt(intPin, sensorISR, RISING);                      // Pressure Sensor interrupt from low to high
      stayAwake = stayAwakeLong;                                       // Keeps Boron awake after deep sleep - may not be needed
    }
    state = IDLE_STATE;                                                // Head back to the idle state to see what to do next
    } break;

  case NAPPING_STATE: {  // This state puts the device in low power mode quickly
    int wakeInSeconds = 0;
    if (state != oldState) publishStateTransition();
    state = IDLE_STATE;                                               // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    if (sensorDetect) break;                                          // Don't nap until we are done with event
    stayAwake = 5000;                                                 // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    ab1805.stopWDT();                                                 // If we are sleeping, we will miss petting the watchdog
    Particle.disconnect();                                            // Disconnect from Particle - but cellular connection will be active
    if (current.occupancyStatus) {                                    // We will set different sleep times based on whether the court is occupied or not
        wakeInSeconds = constrain((current.debounceMin* 60 - (Time.now() - current.lastOccupancyTime)),1,current.debounceMin*60); // Need to calc based on delay
    }
    else wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);  // Not occupied so wait till the next hour

    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .gpio(intPin,RISING)
      .duration(wakeInSeconds * 1000)
      .network(NETWORK_INTERFACE_CELLULAR, SystemSleepNetworkFlag::INACTIVE_STANDBY);           // Not sure how long we will sleep so need to keep the network active - 14mA power 
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep
    ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
    stayAwakeTimeStamp = millis();
    if (result.wakeupPin() == userSwitch) setLowPowerMode("0");
    if (!Particle.connected()) sysStatus.connectedStatus = false;     // We don't know how long we have slept and we may or may not be connected - need to check here
    } break;

  case REPORTING_STATE:
    if (state != oldState) publishStateTransition();
    if (!sysStatus.connectedStatus) {
      particleConnectionNeeded = true;
      state = CONNECTING_STATE;                                 // Go to connect state to connect and will return from there
      break;
    }
    if (Particle.connected()) {
      if (Time.hour() == sysStatus.openTime) dailyCleanup();          // Once a day, clean house
      takeMeasurements();                                             // Update Temp, Battery and Signal Strength values
      sendEvent();                                                    // Send data to Ubidots
      state = RESP_WAIT_STATE;                                        // Wait for Response
    }
    else {
      resetTimeStamp = millis();
      state = ERROR_STATE;
    }
    break;

  case RESP_WAIT_STATE:
    if (state != oldState) publishStateTransition();
    if (!dataInFlight)  {                                             // Response received --> back to IDLE state
      stayAwakeTimeStamp = millis();
      if (Time.hour() == 0) resetEverything();                        // It is a new day.  Zero everything so we can start fresh we generally only see this line if we are operating 24 hours
      state = IDLE_STATE;
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      state = ERROR_STATE;                                            // Response timed out
    }
    break;

  case ERROR_STATE:                                                   // To be enhanced - where we deal with errors
    if (state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait) {
      if (sysStatus.resetCount <= 3) {                                // First try simple reset
        if (Particle.connected()) publishQueue.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - sysStatus.lastHookResponse > 7200L) { //It has been more than two hours since a sucessful hook response
        if (Particle.connected()) publishQueue.publish("State","Error State - Power Cycle", PRIVATE);  // Broadcast Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                     // Zero the ResetCount
        systemStatusWriteNeeded=true;
        ab1805.deepPowerDown(10);
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (Particle.connected()) publishQueue.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                     // Zero the ResetCount
        systemStatusWriteNeeded=true;
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
  // Take care of housekeeping items here

  ab1805.loop();                                                      // Keeps the RTC synchronized with the Boron's clock

  if (systemStatusWriteNeeded) {
    fram.put(FRAM::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (currentStatusWriteNeeded) {
    fram.put(FRAM::currentStateAddr,current);
    currentStatusWriteNeeded = false;
  }

  if (outOfMemory >= 0) {                                             // In this function we are going to reset the system if there is an out of memory error
    char message[64];
    snprintf(message, sizeof(message), "Out of memory occurred size=%d",outOfMemory);
    Log.info(message);
    delay(100);
    publishQueue.publish("Memory",message,PRIVATE);                   // Publish to the console - this is important so we will not filter on verboseMod
    delay(2000);
    System.reset();                                                   // An out of memory condition occurred - reset device.
  }
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


void serviceSensorEvent()                                             // We only come here if the PIR sensor triggered an interrupt
{
  current.lastOccupancyTime = Time.now();                             // This will help us know how long to hold occupied state

  if (!current.occupancyStatus) {                                     // You can get multiple PIR events while the court is occupied - but we need to do some extra things if state was previously not occupied
    pinSetFast(blueLED);                                              // Turn on the blue LED
    current.occupancyStatus = true;
    snprintf(occupancyStateStr, sizeof(occupancyStateStr), "%s", (current.occupancyStatus) ? "Occupied" : "Not Occupied");  // Update the string for the Particle variable
    current.lastOccupancyChange = Time.now();
    state = REPORTING_STATE;                                          // Occupancy state changed - need to report
  }
  sensorDetect = false;                                               // Reset the flag
  currentStatusWriteNeeded = true;                                    // Write updated values to FRAM
}


void serviceDebounceEvent() {                                         // We get here when it has been over the debounce period and we are in occupied state
  current.occupancyStatus = false;
  snprintf(occupancyStateStr, sizeof(occupancyStateStr), "%s", (current.occupancyStatus) ? "Occupied" : "Not Occupied");  // Update the string for the Particle variable
  int newMinutes = (Time.now() - current.lastOccupancyChange)/60;
  current.lastOccupancyChange = Time.now();
  current.dailyOccupancyMinutes += newMinutes;
  currentStatusWriteNeeded = true;
  digitalWrite(blueLED,LOW);
  state = REPORTING_STATE;
}


void sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global
  unsigned long timeStampValue;                                       // Going to start sending timestamps - and will modify for midnight to fix reporting issue

  timeStampValue = Time.now();

  snprintf(data, sizeof(data), "{\"occupancy\":%i, \"dailyoccupancy\":%i, \"battery\":%i,\"key1\":\"%s\",\"temp\":%i, \"resets\":%i, \"alerts\":%i,\"connecttime\":%i,\"timestamp\":%lu000}",current.occupancyStatus, current.dailyOccupancyMinutes, sysStatus.stateOfCharge, batteryContext[sysStatus.batteryState], current.temperature, sysStatus.resetCount, current.alertCount, sysStatus.lastConnectionDuration, timeStampValue);
  publishQueue.publish("Ubidots-Sensor-Hook-v1", data, PRIVATE);
  dataInFlight = true;                                                // set the data inflight flag
  webhookTimeStamp = millis();
  lastReportedTime = Time.now();
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
  // publishQueue.publish("Ubidots Hook", responseString, PRIVATE);
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready

  getTemperature();                                                   // Get Temperature at startup as well
  
  // Battery Releated actions
  if (!isItSafeToCharge()) current.alertCount++;                      // Increment the alert count
  sysStatus.stateOfCharge = int(System.batteryCharge());              // Percentage of full charge
  if (sysStatus.stateOfCharge < 30) sysStatus.lowBatteryMode = true;  // Check to see if we are in low battery territory
  else sysStatus.lowBatteryMode = false;                              // We have sufficient to continue operations

  systemStatusWriteNeeded = true;
  currentStatusWriteNeeded = true;
}

bool isItSafeToCharge()                                               // Returns a true or false if the battery is in a safe charging range.  
{     
  sysStatus.batteryState = System.batteryState();
  PMIC pmic(true);                                                                
  if (current.temperature < 36 || current.temperature > 100 )  {      // Reference: https://batteryuniversity.com/learn/article/charging_at_high_and_low_temperatures (32 to 113 but with safety)
    pmic.disableCharging();                                           // It is too cold or too hot to safely charge the battery
    sysStatus.batteryState = 1;                                       // Overwrites the values from the batteryState API to reflect that we are "Not Charging"
    return false;
  }
  else {
    pmic.enableCharging();                                            // It is safe to charge the battery
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
  particleConnectionNeeded = true;                                  // Get connected to Particle - sets sysStatus.connectedStatus to true
  if (Particle.connected()) publishQueue.publish("Mode","Loading System Defaults", PRIVATE);
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
  sysStatus.sensorType = 1;                                       // These are all PIR sensors
  strncpy(sensorTypeConfigStr,"PIR Sensor",sizeof(sensorTypeConfigStr));
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (sysStatus.openTime < 0 || sysStatus.openTime > 12) sysStatus.openTime = 0;
  if (sysStatus.closeTime < 12 || sysStatus.closeTime > 24) sysStatus.closeTime = 24;
  if (sysStatus.lastConnectionDuration < 0 || sysStatus.lastConnectionDuration > connectionTimeout) sysStatus.lastConnectionDuration = 0;
  // None for lastHookResponse
  systemStatusWriteNeeded = true;
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


bool disconnectFromParticle()                                     // Ensures we disconnect cleanly from Particle
{
  Particle.disconnect();
  waitFor(notConnected, 15000);                                   // make sure before turning off the cellular modem
  sysStatus.connectedStatus = false;
  systemStatusWriteNeeded = true;
  delay(2000);                                                    // Bummer but only should happen once an hour
  return true;
}

bool notConnected() {                                             // Companion function for disconnectFromParticle
  return !Particle.connected();
}

int resetCounts(String command)                                       // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    current.longestOccupancyMinutes = 0;                              // Reset Daily Count in memory
    sysStatus.resetCount = 0;                                            // If so, store incremented number - watchdog must have done This
    current.alertCount = 0;                                           // Reset count variables
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
    publishQueue.publish("Reset","Hard Reset in 2 seconds",PRIVATE);
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
  current.longestOccupancyMinutes = 0;                                // Reset the counts in FRAM as well
  current.dailyOccupancyMinutes = 0;                                  // Reset for the day
  current.lastOccupancyChange = Time.now();                           // Set the time context to the new day
  sysStatus.resetCount = current.alertCount = 0;                      // Reset everything for the day
  currentStatusWriteNeeded=true;                                      // Make sure that the values are updated in FRAM
  systemStatusWriteNeeded=true;
  lastReportedTime = Time.now();
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.solarPowerMode = true;
    setPowerConfig();                                               // Change the power management Settings
    systemStatusWriteNeeded=true;
    if (Particle.connected()) publishQueue.publish("Mode","Set Solar Powered Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.solarPowerMode = false;
    systemStatusWriteNeeded=true;
    setPowerConfig();                                                // Change the power management settings
    if (Particle.connected()) publishQueue.publish("Mode","Cleared Solar Powered Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

/**
 * @brief Set the Sensor Type object
 * 
 * @details Over time, we may want to develop and deploy other sensot types.  The idea of this code is to allow us to select the sensor
 * we want via the console so all devices can run the same code.
 * 
 * @param command a string equal to "0" for pressure sensor and "1" for PIR sensor.  More sensor types possible in the future.
 * 
 * @return returns 1 if successful and 0 if not.
 */
int setSensorType(String command)                                     // Function to force sending data in current hour
{
  if (command == "0")
  {
    sysStatus.sensorType = 0;
    strncpy(sensorTypeConfigStr,"Pressure Sensor", sizeof(sensorTypeConfigStr));
    systemStatusWriteNeeded=true;
    if (Particle.connected()) publishQueue.publish("Mode","Set Sensor Mode to Pressure", PRIVATE);
    
    return 1;
  }
  else if (command == "1")
  {
    sysStatus.sensorType = 1;
    strncpy(sensorTypeConfigStr,"PIR Sensor", sizeof(sensorTypeConfigStr));
    systemStatusWriteNeeded=true;
    if (Particle.connected()) publishQueue.publish("Mode","Set Sensor Mode to PIR", PRIVATE);
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
    if (Particle.connected()) publishQueue.publish("Mode","Set Verbose Mode", PRIVATE);
    sensorControl(true);                                    // Make sure the sensor is on and correctly configured
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) publishQueue.publish("Mode","Cleared Verbose Mode", PRIVATE);
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
    publishQueue.publish("Time",data, PRIVATE);
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
  if (Particle.connected()) publishQueue.publish("Time",data, PRIVATE);
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
  if (Particle.connected()) publishQueue.publish("Daily",data, PRIVATE);
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
      publishQueue.publish("Mode","Low Power Mode", PRIVATE);
    }
    sysStatus.lowPowerMode = true;
    strncpy(lowPowerModeStr,"True", sizeof(lowPowerModeStr));
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    if (!sysStatus.connectedStatus) {                                      // In case we are not connected, we will do so now.
      particleConnectionNeeded = true;
    }
    publishQueue.publish("Mode","Normal Operations", PRIVATE);
    delay(1000);                                                      // Need to make sure the message gets out.
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
  if(Particle.connected() && sysStatus.verboseMode) publishQueue.publish("State Transition",stateTransitionString, PRIVATE);
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
  publishQueue.publish("Daily Cleanup","Running", PRIVATE);            // Make sure this is being run
  sysStatus.verboseMode = false;
  Particle.syncTime();                                                 // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                                // Wait for up to 30 seconds for the SyncTime to complete
  if (sysStatus.solarPowerMode || sysStatus.stateOfCharge <= 70) {     // If Solar or if the battery is being discharged
    setLowPowerMode("1");
  }
  systemStatusWriteNeeded = true;
}