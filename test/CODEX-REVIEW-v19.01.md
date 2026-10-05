# Adversarial review: Occupancy-Sensors firmware v19.00 + v19.01

## Your task

Do an **adversarial review** of the firmware changes listed below. Assume the author (another AI agent) made mistakes and that its simulation is too forgiving. Try to break the changes. A finding counts only if you can name a concrete sequence of events that leads to wrong behavior on a real Particle Boron.

- **Do not edit `src/`.** Report findings only.
- You may add scenarios or checks under `test/sim/` to prove a finding.
- Also review the test harness. It was written by the same agent, so it may share the same blind spots as the firmware.

## The device

- **Hardware:** Particle Boron (Gen 3, LTE-M, SARA-R410M modem) on a carrier board that has:
  - an AB1805 RTC and watchdog
  - FRAM
  - a TMP36 temperature sensor
  - a PIR sensor
- **Use:** it is a solar-powered occupancy sensor for a sports court.
- **Device OS mode:** `SYSTEM_MODE(SEMI_AUTOMATIC)` and `SYSTEM_THREAD(ENABLED)`.
- **Reports:** occupancy reports go to Ubidots through a webhook, using `PublishQueuePosixRK`. The queue is stored in a file, sends 1 event per second, and waits 2 s after connecting before the first send.
- **Firmware:** a single file, `src/Occupancy-Sensors.ino`. `src/Occupancy-Sensors.cpp` is generated from it, so ignore that file.

### Battery policy during open hours (default 06:00–21:00)

Low power mode (LPM) turns on when SoC is ≤70% at the daily cleanup, or when the device is in solar mode. It only turns off when someone presses the button or calls a console function.

| SoC | Connection rule | Nap mode |
|---|---|---|
| LPM off | Always connected | No naps |
| LPM on, >65% | Reconnects to the cloud on every event | ULP, cellular `INACTIVE_STANDBY` |
| 51–65% | At most one connection per clock hour | ULP, modem off |
| 30–50% | Connects only in hours divisible by 3 | ULP, modem off |
| <30% | Never connects | ULP, modem off |

## Scope

The `git diff HEAD -- src/Occupancy-Sensors.ino` output includes three layers of uncommitted changes:

- **v18.01:** made by the developer. This is the baseline. Only review it where it interacts with the changes below.
- **v19.00:** made by the agent. **In scope.**
- **v19.01:** made by the agent. **In scope.**

The `test/sim/baselines/` folder has full copies of v18.01 and v19.00 so you can diff the layers.

### v19.00: changes that fix late reports seen in the field
On a soak unit at about 60% SoC, top-of-hour reports were skipped and queued events were delivered hours late.

| Line | Change |
|---|---|
| L612–618 | New `readyToDisconnect()`: returns true if offline, if the queue is empty, or after 120 s measured from `stayAwakeTimeStamp` |
| L334 | IDLE naps only when `readyToDisconnect()` is true |
| L349 | SLEEPING_STATE `break`s, staying in that state until `readyToDisconnect()` is true, before calling `disconnectFromParticle()` |
| L449 | The 51–65% rule changed from a rolling `Time.now() - lastConnection < 3600` to the same clock hour, `Time.now()/3600 == lastConnection/3600` |

### v19.01: solar charging and Particle connectivity guidance

| Line | Change | Reason |
|---|---|---|
| L754–763 | Keeps the raw `System.batteryState()` value. When SoC <65%, raw state is 1 (not charging) and temperature is safe, calls `setPowerConfig()` instead of `System.setPowerConfiguration(SystemPowerConfiguration())` (factory defaults) | Before this, a hot afternoon (temperature check overwrote `batteryState` with 1) at SoC <65% reset the PMIC to defaults until the next reboot. That dropped the solar `powerSourceMinVoltage` (5080 mV) and the 4.208 V charge voltage. |
| (removed) | `fuelGauge.quickStart()` in `takeMeasurements()` while in LPM. It stays in `setup()` (L224) | It skews SoC under charging or radio current, which makes the SoC bands flap |
| L782 | Charging is now allowed up to 113°F (was 100°F) | Li-ion charge range is 32–113°F. The enclosure goes above 100°F in sun, which blocked charging when sun was available. |
| L135, L336 | Stay awake 30 s after the top-of-hour report at ≤65% SoC (was 90 s) | Energy. v19.00 already waits for the queue to empty. |
| L95 | `connectMaxTimeSec` changed from 10 min to 11 min | Particle: Device OS power-cycles the modem after 10 min of failing to connect, and recommends waiting at least 11 min |
| L96, L144, L455–459, L475, L484 | 30-minute backoff after a failed connection. `lastFailedConnection` is `retained` so the 3-hour `ERROR_STATE` `System.reset()` doesn't clear it. The user button bypasses the backoff. | Particle: cycling cellular more often than every 10 min can get the SIM banned by the carrier. Before this, there was no backoff, so each motion event started a new 10-minute attempt. |
| L535 | `ERROR_STATE` always calls `disconnectFromParticle()` before reset or `deepPowerDown()`. Before this, it only did so when `Particle.connected()` was true. | Particle: cutting modem power unexpectedly can brick it |
| L145, L586–589, L975 | The `hardResetNow` cloud function sets a flag. `loop()` then calls `disconnectFromParticle()` and then `ab1805.deepPowerDown(10)` | Same as above. Also avoids blocking for up to 45 s inside a function callback. |

Particle sources the agent relied on. Check that it read them correctly:
- https://docs.particle.io/reference/device-os/sleep/
- https://docs.particle.io/firmware/best-practices/firmware-introduction/ (section "Aggressively managing connectivity")
- https://docs.particle.io/firmware/low-power/wake-publish-sleep-cellular/ (the 11-minute and backoff guidance)
- https://docs.particle.io/hardware/best-practices/watchdog-timers/

## Simulation

```
test/sim/run.sh                                              # current src/Occupancy-Sensors.ino
test/sim/run.sh test/sim/baselines/Occupancy-Sensors-v19.00.ino
```

**How it works:** the real `.ino` is compiled on the host against the stand-ins in `test/sim/stubs/`, with a virtual clock in 100 ms steps. Two scripted days of PIR events are played through it, and `check.py` checks 11 rules. Output goes to `test/sim/out/`. `./sim <scenario> v` gives a verbose trace.

**Current result, v19.01:**
- `base`, `connected`, `lowbatt`, `slowconn`, `verylow`, `hot`, `hot115` and `notchg` pass all 11 rules.
- `nonet`, `badnet`, `badnetlow` and `hardreset` fail rules 4 and 5. `reboot` fails rule 7. These are known (see below).
- During outages, there are 6–8 modem power-ups at least 50 minutes apart. v19.00 had 11–14 power-ups, 10 minutes apart.

**Known weaknesses in the harness. Exploit them.**
- The stubs model only what the agent thought mattered:
  - `waitFor()` evaluates its condition once.
  - Publishing never fails while connected.
  - Sleep wakes exactly on time.
  - `Particle.connected()` turns true a fixed `connectDelayMs` after `connect()`.
  - Device OS's own 10-minute modem reset is **not modeled**.
- `reboot()` in `sim.cpp` resets RAM globals by hand. A new global that it doesn't reset will hide reboot bugs. This already happened once, with `hardResetRequested`.
- `retained` is a no-op macro. The sim keeps `lastFailedConnection` across `System.reset` and clears it on `deepPowerDown`. That is the agent's *assumption* about how Boron retained RAM behaves.
- The rule 9 limits (when a delivery counts as on time) were picked by the agent after it saw the v19 behavior. Check that they aren't tuned to pass.

## Known issues: don't report these unless a change makes them worse

- **Rule 7 in `reboot`:** after a reboot during closed hours, the next day's cleanup is skipped. This is "fix C", which the developer hasn't applied yet.
- **Rules 4 and 5 in the outage scenarios:** when a connection attempt is still running at the top of the hour, the hourly report is generated when the attempt ends. The 11-minute timeout makes these reports about 1 minute later than before. Report it if you think that tradeoff is wrong.
- **`hardreset`:** the 10 s power-down makes the 11:00 report 11 s late. This is expected.

## Questions to answer

1. **`retained` on Gen 3:** does a `retained` global on a Boron survive `System.reset()` without `STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY))`? Is it set to its initializer on a cold boot or after an AB1805 `deepPowerDown()`? Could it hold garbage, or a future timestamp from before an RTC or clock correction, that blocks connections? The `Time.now() >= lastFailedConnection` guard is supposed to cover the future-timestamp case. Can `Time.now()` be invalid or 0 at that point?
2. **Clock-hour gate (L449):** check the types of `sysStatus.lastConnection` (see `sys_status.h`) and `Time.now()` for signed/unsigned or width bugs. Is UTC-hour bucketing always the same as the local hour? Can `lastConnection` be 0, or in the future after a time sync, in a way that blocks a connection for good?
3. **Backoff interactions:** check how the backoff combines with:
   - the 3-hour `ERROR_STATE` rule (`Time.now() - sysStatus.lastConnection > 3h`, which only moves forward on success)
   - the ≤50% three-hour schedule
   - `setLowPowerMode("0")` from the console, which sets `state = CONNECTING_STATE`
   - devices that aren't in LPM (DC powered)

   Can a device end up never reconnecting, or reconnecting more often than every 10 minutes, under any SoC or outage pattern? Also: a device that never connects at boot has `lastConnection` = 0, so it goes to ERROR right after its first failure. Is that a reset loop?
4. **`readyToDisconnect()`:** it measures 120 s from `stayAwakeTimeStamp`, which is also reset on every nap wake and in other places.
   - Can the device stay awake with the radio on much longer than intended?
   - In SLEEPING_STATE, it `break`s and re-runs the state's top every loop (detach interrupt, `sensorControl(false)`, the final-report branch). Is any of that unsafe to repeat?
   - Does the watchdog still get petted?
5. **PMIC re-apply (L760):** `setPowerConfig()` first applies `SystemPowerConfiguration()` defaults and then the custom config. At SoC <65% with raw state 1, this now runs on every `takeMeasurements()` (about once an hour). Is that harmful: charger glitches, I2C contention with `PMIC pmic(true)` in `isItSafeToCharge()`, or a raised alert 11 every hour? Is raw `batteryState == 1` normal on solar, for example at dusk with panel voltage present but too low? If so, does this fire constantly?
6. **113°F limit:** the TMP36 measures the board, not the battery cell. Is removing all margin from the 113°F limit safe? Suggest a better approach if you have one, for example hysteresis.
7. **`disconnectFromParticle()` in more places:** it blocks for up to 15 s plus 30 s.
   - Can that trip the AB1805 watchdog (124 s) on any path?
   - Is it safe for alert 14 (out of memory)?
   - Is it safe for alert 12 (FRAM init failure), where `fram` may not be usable?
8. **Connection guidance:** is anything left that violates Particle's guidance? Look for paths that power the modem down hard, cycle cellular more often than every 10 minutes, or keep connection attempts under 5 minutes, which prevents IMSI rotation. Check `fullModemReset()`, the reset-count path (alert 13), the `deepPowerDown` calls in `setup()`, and rolling reboots.
9. **Other energy waste:** look for anything else that wastes energy on a solar unit and can be fixed without changing the reporting schedule.

## Report format

For each finding give:
- **Severity:** Critical (bricking, SIM ban, permanent loss of data or connectivity), High, Medium or Low
- **Location:** `file:line`
- **Failure scenario:** a concrete sequence of inputs and state that produces the wrong result
- **Evidence:** a sim scenario you added and its output, a Particle doc quote, or a Device OS source reference
- **Suggested fix**

Then list each v19.00 and v19.01 change from the tables above and mark it **Confirmed correct**, **Incorrect**, or **Unverifiable from available sources**, with one line of reasoning. Finish with any problems you found in the simulation harness itself.
