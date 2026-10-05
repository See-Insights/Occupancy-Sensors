# Bench test: Occupancy-Sensors v19.04

**Goal:** confirm on a real Boron the behaviors the host simulation can't prove: modem timing, charging control, power-down and recovery. Run tests 1–5 in order. Tests 6 and 7 take longer and can be done separately.

## Setup

**Hardware**
- Boron on the production carrier board, with the PIR sensor and battery attached
- USB to the Mac for power and logs
- For test 4: a hair dryer or heat gun on low, and a thermometer, ideally a thermocouple on the battery
- For test 6: a metal tin or other shielded box to block cellular

**Battery**
- Tests 1–4 at **>65% SoC**.
- Tests 3 and 5 also need a battery at **51–65% SoC**.
- Check SoC with the `stateOfChg` variable before starting.

**Build and flash**
1. **Check Device OS.** The binary targets Device OS **6.4.1**. If the Boron runs anything older, first run `particle update` over USB.
2. **Development device.** If the Boron belongs to a Particle product, mark it as a *Development Device* in the console. Otherwise the product's released firmware replaces your bench build when it connects.
3. **Build.** In Workbench, run *Particle: Clean application (local)*, then *Particle: Compile application (local)*, for target `boron` / `6.4.1`.
4. **Flash over USB:**
   ```sh
   particle usb dfu
   particle flash --usb target/6.4.1/boron/Occupancy-Sensors.bin
   ```
5. **Start logging** and leave it running for every test:
   ```sh
   particle serial monitor --follow | tee bench-v19.04.log
   ```

**Watch `ResetCount`.** RESET-button and user resets count toward a limit of 3 per day; the count clears at the daily cleanup at opening. A 4th reset sets alert 13, which power-cycles the device. That's correct behavior, not a test failure. Tests 3 and 4 each press RESET, so check `ResetCount` first. If it's already 2 or more, expect a power cycle after the next RESET.

**Charging state is only re-checked when the device takes a measurement.** That happens at every report, so to force a new check, trigger motion or call `SendNow` = `1`.

**Temperature readings.** The `Temperature` variable is °F, and it's calculated from whole °C, so:

| Board °C | `Temperature` reads |
|---|---|
| 39 | 102 |
| 40 | 104 |
| 43 | 109 |
| 45 | 113 |

## 1. Smoke test: version, occupancy, reporting

1. After flashing, check the `Release` variable reads **19.04** and the serial log shows `Startup complete with version 19.04`.
2. Set `Set-Debounce` = `1` (1 minute) to speed things up.
3. Wave at the PIR once. A webhook event should arrive with `occupancy:1` within seconds.
4. Don't move for 1 minute. An event with `occupancy:0` should arrive, and `dailyoccupancy` should have gone up.
5. Keep triggering the PIR for more than 15 minutes. An `occupancy:1` update should arrive about every 15 minutes.
6. The top-of-hour report should arrive at **hh:00:01**, give or take a few seconds.

**Pass:** all reports arrive, and their timestamps and values match.
**When done:** set `Set-Debounce` back to its production value.

## 2. Queue drains before napping (>65% SoC, low power mode)

1. Set `LowPowerMode` = `1`.
2. Trigger 3 motion events in quick succession, so that several reports are queued.
3. Watch the serial log.

**Pass:**
- Every queued event is published before the log shows `Napping with radio on`.
- In the Particle console's event stream, no event arrives late at the *next* wake.

## 3. Hour boundary: modem stays in standby (51–65% SoC)

1. With the device in low power mode and `stateOfChg` reading 51–65, wait until between **hh:51 and hh:58**.
2. Press RESET. The device reconnects as it boots.
3. Watch the serial log through the next top of the hour.

**Pass:**
- After the boot connection, the log shows `Napping with radio on`, even though SoC is 65% or less.
- The hh:00:01 report connects within seconds, with no modem start or registration messages.
- After that report, the next nap shows `Napping with radio off`.

**Fail:** a modem power-off followed by a power-up less than 10 minutes later.

## 4. Charging temperature limits (most important)

Run on USB or solar power, with the battery below full so that it's actually charging: the Boron's orange charge LED should be on.

**Stop if the battery itself goes above 50°C.**

1. **Heat up.** Warm the board area slowly with the hair dryer. Trigger a report each time `Temperature` rises. As soon as it reads **109** (43°C):
   - the charge LED goes **off**
   - `Alerts` = **10**
   - `BatteryContext` shows **Not Charging**
2. **Off across naps.** Hold it at 109–111°F with `LowPowerMode` = `1` through at least **2 naps**. Check the charge LED after each wake.
   - **Pass:** it stays off. This checks the v19.03 fix for the Power Manager wake reload.
3. **Hot reboot.** While still at 109°F or above, press RESET and watch the charge LED through boot.
   - **Pass:** it never turns on. This checks the v19.04 hot-boot fix.
4. **Cool down.** Remove the heat and trigger a report every minute or two.
   - At **104** (40°C): charging stays off.
   - At **102** (39°C) or lower: the LED comes back **on** and `Alerts` clears.
5. **Record** the thermocouple reading on the battery next to each `Temperature` reading. If the battery runs more than 2°C hotter than the board sensor, the 43°C limit needs to come down.

## 5. Hard reset shuts the modem down cleanly

1. While connected, call `HardReset` = `1` from the console.

**Pass:**
- The log shows `In the disconnect from Particle function`, then about a 10 s power-off, then a normal boot.
- The device reconnects. The reset reason in the log is a power-down or pin reset, not a watchdog reset.
- There's no hang between the disconnect message and the power-off.

## 6. Failed connection backoff (51–65% SoC, about 45 min)

1. Put the device in low power mode, then into the shielded box. You can also just disconnect the antenna.
2. Trigger a report.

**Pass:**
- The log shows `cloud connection unsuccessful` after about **11 minutes**, not 10.
- Motion during the next **30 minutes** shows `Connecting state but backing off after a failed connection`, and there are no new modem power-ups.
- Pressing the user button during the backoff starts a connection immediately. This bypass is intentional.
- Taking the device out of the box after the backoff ends: the next report connects and delivers the queued backlog.

Run this test only once per session. Repeated failed connections are exactly what carriers watch for.

## 7. Closing time and the next opening (about 2 h)

1. Set `Set-Close` to the next hour and `Set-OpenTime` to the hour after that.
2. Leave the court "occupied" across closing time by triggering motion just before the top of the hour.

**Pass:**
- At close, the final report shows `occupancy:0` with the full `dailyoccupancy` total.
- The log shows the queue drained and the radio turned off.
- There are no reports during closed hours.
- At opening, the cleanup runs and the first report shows `dailyoccupancy:0`.

**When done:** set the open and close times back to production values.

## Results record

| Test | Pass/Fail | Notes (times, readings, log excerpts) |
|---|---|---|
| 1 Smoke | | |
| 2 Queue drain | | |
| 3 Hour boundary | | |
| 4 Charging limits | | battery °C at off: ___ / at on: ___ |
| 5 Hard reset | | |
| 6 Backoff | | |
| 7 Closing / opening | | |

Keep `bench-v19.04.log` with the results. Next step after passing: a field soak on a ~60% SoC unit to confirm the hourly reports arrive on time, which was the original problem.
