# Adversarial review of v19.00 / v19.01

Reviewed 2026-10-05. No changes made to `src/`. The generated `.cpp` was excluded.
Compared both supplied baselines with the current `.ino`, ran both v19.00 and
v19.01 simulations, inspected the vendored AB1805 and publish-queue implementations,
and checked Particle documentation. The workspace currently targets Device OS
**6.4.1**, despite the firmware history mentioning 5.5.0. Source-level statements
below use the installed 6.4.1 source; the deployed Device OS version needs verification.

## Findings

### 1. High — clock-hour gating permits aggressive cellular cycling

**Location:** `src/Occupancy-Sensors.ino:449` (interacts with modem-off naps at 402–408).

**Failure scenario:** A 60% LPM unit boots or handles motion near 06:59:20 and
successfully connects before 07:00. Its queue drains and it naps with the modem
off. At 07:00 the hourly report enters CONNECTING; the clock-hour comparison
permits another modem power-up only seconds after the previous one. Both attempts
succeed, so the new failed-connection backoff does not intervene. A short connection
on a real LTE-M network is enough to reproduce this sequence.

**Evidence:** `bash test/sim/adversarial.sh` prints
`boundary: modem power-ups 41 s apart (minimum 600 s)`.
The stub's two-second connection is optimistic, but is not essential: the
sequence also fits within a minute with a 10–20 second connection.
Particle says: “You must not turn off and on cellular more than every 10 minutes”.
[Cellular API](https://docs.particle.io/reference/device-os/api/cellular/cellular/)

**Suggested fix:** Keep clock-hour reporting, but add a separate monotonic minimum
interval for cellular attachment/power cycling. When the next permitted report is
less than ten minutes away, retain cellular standby through that boundary rather
than switching the modem off. Simply restoring the rolling gate would restore the
late-report bug this change was meant to solve.

### 2. High — the 113°F gate allows charging above the assumed 45°C cell limit

**Location:** `src/Occupancy-Sensors.ino:782`, with conversion at 817–818.

**Failure scenario:** The sensor and cell are both at 45.95°C and the TMP36 is
perfectly accurate. The conversion truncates Celsius to 45, converts to 113°F,
and the inclusive gate enables charging. Sensor under-reading, ADC/reference
error, a warmer cell, or heating between measurements increase the exposure.
This removes the previous high-temperature margin; the finding does not require
assuming any particular battery/board temperature difference.

**Evidence:** The focused probe prints
`temperature: ideal TMP36 45.9546 C, firmware 113 F, charging 1`.
Analog Devices specifies typical ±2°C accuracy over temperature, which is an
additional uncertainty, not a guaranteed worst-case bound.
[TMP36 manufacturer specification](https://www.analog.com/en/products/tmp36.html)
The 45°C cell limit here is the dispatch's assumption; the actual pack datasheet
was not supplied, so its permissible charge range remains unverified.

**Suggested fix:** Compare an unrounded Celsius value; use the actual pack's
limits and an error/thermal budget. Until that budget is measured, retain a
conservative cutoff. Add a lower re-enable threshold, and sample temperature
more frequently while charging. Hysteresis alone cannot compensate for sensor
error or thermal lag. Prefer a cell-mounted sensor/thermistor and verify hardware
charge-temperature protection on the carrier board.

### 3. Medium — repeated reports can extend the queue-drain wait indefinitely

**Location:** `src/Occupancy-Sensors.ino:617`, with timestamp reset at 431.

**Failure scenario:** A connected 60% LPM unit has a publish that cannot complete
although `Particle.connected()` remains true. Configure the supported one-minute
debounce and trigger motion every 65 seconds. Each occupancy transition generates
a report; CONNECTING's already-connected branch resets `stayAwakeTimeStamp`.
The oldest queued event never advances, but the 120-second guard never expires.
The device spends many minutes awake with its modem on. The pre-existing three-hour
webhook-error policy can eventually interrupt this; it does not enforce the new
120-second energy budget.

**Evidence:** The probe runs the real firmware loop with queue publication paused
and reports `drain: oldest event pending for 600 s, readyToDisconnect=0`.
Pausing publication models lack of queue progress; it does not simulate the
specific underlying transport failure. The real queue supports this pause API.

**Suggested fix:** Start a separate drain deadline once per continuous drain
episode/connection. Do not reset it for subsequent reports or already-connected
checks. Retain `stayAwakeTimeStamp` for the ordinary wake/command window.

### 4. High — the new hard-power-down path proceeds after shutdown timeout

**Location:** `src/Occupancy-Sensors.ino:587` and `535`; helper at 950–952.

**Failure scenario:** A hard reset is requested while the modem is on, but the
system/NCP shutdown cannot complete within the allotted time. The 30-second
`waitFor(Cellular.isOff, ...)` returns false. The helper discards that result,
returns true, and the caller cuts board power anyway. Alert 12/13 power-downs
have the same issue. This defeats the stated purpose of adding orderly shutdown
to these paths and leaves a risk of modem damage; actual damage was not reproduced
on hardware.

**Evidence:** Particle documents `Cellular.off()` as asynchronous and documents
`Cellular.isOff()` as the completion check.
[Cellular API](https://docs.particle.io/reference/device-os/api/cellular/cellular/)
Installed Device OS 6.4.1 `wiring/inc/spark_wiring_system.h:755–771` shows that
`waitFor` returns the condition result after timeout. Both results are ignored
by this helper. The harness cannot exercise this sequence because its `off()`
always succeeds immediately.
The installed 6.4.1 `hal/network/ncp_client/sara/sara_ncp_client.cpp:404`
also contains explicit soft/hardware power-off failure paths; this is not an API
whose underlying hardware operation is guaranteed to succeed.

**Suggested fix:** Return and honor verified shutdown status. A hard-power-down
request should wait in a nonblocking, watchdog-serviced shutdown state; a timeout
must take an explicit recovery path rather than immediately cutting power.
Keep RAM alive until the modem's off state is confirmed.

## Answers to the nine questions

1. **Retained memory:** On Gen 3, retained RAM is always enabled regardless of
   the feature flag. It survives `System.reset()`. Device OS 1.5.0 onward initializes
   it on cold power-up, so the initializer is zero on a genuine loss of Boron power.
   An AB1805 power-down clears it only if the carrier actually removes Boron power;
   the library also has a software-reset fallback, which preserves it. The harness
   assumes ideal carrier operation. OTA changes/reset-during-write can still leave
   invalid retained data; Particle recommends validation. A future timestamp bypasses
   this gate, rather than blocking forever. `Time.now()` can be invalid, and the gate
   does not check `Time.isValid()`. The AB1805 normally restores it in setup if its
   RTC is initialized; a fresh/failed RTC provides no such guarantee. Use a monotonic
   timer for the running boot, with validated retained/FRAM state for reset recovery.
   [Particle retained-memory reference](https://docs.particle.io/reference/device-os/firmware/#retained-memory)

2. **Hour types:** Installed 6.4.1 `spark_wiring_time.h:71` returns `time32_t`,
   and `sys_status.h` stores `lastConnection` as the same type. There is no
   signed/unsigned mismatch in the hour comparison. `hal/shared/time_compat.h`
   defines the 32-bit type; on this modern implementation it is signed. The
   comparison retains the API's 2038 limitation, rather than introducing a new
   width problem. `time_t` used for the backoff is a distinct portability concern,
   but current positive timestamps compare normally. UTC buckets align with local
   clock-hour boundaries for this installation's whole-hour EDT/EST offsets, not
   for arbitrary half-hour/quarter-hour offsets supported by the timezone field.
   Zero only matches the epoch's first hour, and a future value matches only its
   one bucket; neither comparison alone blocks forever. Clock rollback can bypass
   scheduling protections. Finding 1 is the practical current defect.

3. **Backoff interactions:** The gate applies in DC/non-LPM mode too. Leaving LPM
   does not clear or bypass it; physical button-low does. A console function cannot
   ordinarily be called while disconnected, so an offline console override is not
   a practical recovery channel. At ≤50%, a 30-minute backoff can miss the rest of
   a divisible-by-three hour and defer retry to the next eligible hour. At 51–65%,
   it can defer retry until the next hourly/motion opportunity after expiry. That is
   finite, but expiry itself does not schedule an attempt. A never-connected unit
   does enter ERROR after its first failure with valid current time; cases 30/31
   write `lastConnection=now` before resetting, and retained backoff survives. Thus
   it is not an immediate reset storm, but that write is a fabricated successful
   connection and can suppress the remainder of the current clock hour. Repeated
   failures produce periodic resets, not permanent lockout. Failed attempts still
   last >11 minutes even if button bypasses backoff. Successful attempts are
   unprotected against Finding 1. Future/invalid clocks defeat elapsed wall-clock
   guarantees. The extra minute of hourly lateness during an ongoing failed attempt
   is a reasonable tradeoff for the documented modem recovery recommendation.

4. **Drain/watchdog:** Finding 3 disproves the global 120-second bound. Ordinarily
   the code still reaches `ab1805.loop()` while SLEEPING's drain check breaks.
   Repeated sensor-disable/detach operations are idempotent; the final report branch
   stops repeating after the reporting timestamp/occupancy state is updated. No
   separate repeated-final-report defect was demonstrated. Queue-empty counts the
   in-flight event in the vendored library; it is cloud-ACK completion, not proof
   that Ubidots durably stored an event.

5. **PMIC:** Raw state 1 does not diagnose a broken configuration. Input present
   with inadequate charging power can produce it; no-input battery operation is
   normally discharging (4), so dusk is not unconditionally state 1. When state 1,
   SoC <65%, and temperature-safe coincide, the branch runs on **every measurement**,
   including occupancy reports, not only hourly reports. It reapplies config and
   raises alert 11 even when no recovery was needed. This behavior largely existed
   already; v19.01 improves the final config and avoids the temperature-induced
   default reset. No new persistent charger fault or PMIC deadlock was demonstrated.
   `PMIC(true)` serializes accesses; it does not make the two configuration calls
   atomic. In installed 6.4.1 `system_power_manager.cpp:1005`, configuration is stored
   then queued for reload; each reload calls `initDefault(false)` at 463–467. The
   default configuration may be applied transiently between calls. Prefer applying
   the complete custom config once, inspecting return codes, and restricting any
   recovery retry to a diagnosed/persistent fault. Remove routine alert 11 when
   insufficient sunlight is the expected condition. Physical glitch magnitude and
   actual dusk behavior remain hardware questions.

6. **Temperature:** Finding 2. Use cell specifications, error margin, hysteresis,
   and frequent thermal sampling. Raising an enclosure reading to a cell's absolute
   limit is not a validated charge-temperature control policy.

7. **Longer disconnects:** The ordinary added 45-second wait alone should not trip
   the 124-second watchdog: vendored AB1805 pets at approximately 62-second intervals,
   leaving approximately 107 seconds worst-case between pets for that isolated block.
   It is wrong to equate the 120-second nonblocking drain with 120 seconds without
   petting. Other blocking work in the same loop can consume the remaining margin;
   no unconditional new watchdog trip was proved. The added helper itself performs
   no FRAM access, so it does not newly depend on FRAM being healthy for alert 12.
   OOM safety cannot be guaranteed: logging/network operations are not an established
   allocation-free recovery path. A pre-existing issue is that `outOfMemory >= 0`
   continually resets the ERROR timestamp in housekeeping; it was not counted as
   an in-scope finding. Prefer a minimal, nonblocking error shutdown and explicit
   watchdog service. Finding 4 applies to the subsequent power cut.

8. **Particle guidance:** Eleven minutes is supported, and 30 minutes between
   failed attempts is useful, though the Cellular API recommends increasing retry
   intervals up to 60 minutes. It is not protection for all cellular cycling.
   Finding 1 remains a violation. `fullModemReset()` is still defined but has no
   callers in this firmware; its unverified legacy reset command/deep-sleep path
   is therefore not a reachable new failure. There are no `deepPowerDown()` calls
   in current `setup()` and no scheduled rolling reboot. Alert 13 is handled in
   ERROR, after the new helper. Normal failure attempts permit the five-minute IMSI
   period. Unexpected watchdog/brownout resets cannot be eliminated by that timeout.
   Device OS's own recovery is outside the harness's modem-cycle model.
   [Particle connection timing](https://docs.particle.io/firmware/low-power/wake-publish-sleep-cellular/)
   [Particle aggressive-connectivity guidance](https://docs.particle.io/firmware/best-practices/firmware-introduction/#aggressively-managing-connectivity)

9. **Energy:** Fix the movable drain deadline and retain modem standby across a
   nearby hour boundary. Avoid normal-condition PMIC default/custom resets and
   misleading alerts. The remaining setup `quickStart()` can still skew SoC and
   is worth revisiting separately; removing periodic calls is beneficial.
   Wake/command-window duration is a deployment preference, not evidence of another
   reporting bug. These suggestions preserve report generation times.

## Change-by-change assessment

“Confirmed correct” below describes the specified change, not a guarantee for every
combination of carrier faults, power events, or downstream service failures.

| Change | Assessment | Reason |
|---|---|---|
| v19.00 readyToDisconnect helper | **Incorrect** | Shared timestamp can move repeatedly; not a true 120-second bound (Finding 3). |
| v19.00 IDLE drain gate | **Confirmed correct** | Prevents ordinary naps while a queued/in-flight publish remains; inherits helper defect. |
| v19.00 SLEEPING drain gate | **Confirmed correct** | Allows final-report queue progress while continuing housekeeping; inherits helper defect. |
| v19.00 clock-hour connection gate | **Incorrect** | Fixes rolling-hour lateness but permits <10-minute modem cycles (Finding 1). |
| v19.01 raw battery state / reapply custom PMIC settings | **Confirmed correct** | Prevents hot-condition default reset and restores chosen settings; routine reapply remains wasteful. |
| v19.01 remove measurement quickStart | **Confirmed correct** | Avoids repeatedly restarting the gauge estimate under charging/radio load; simulator cannot validate gauge accuracy. |
| v19.01 113°F charge limit | **Incorrect** | Rounding alone allows >45°C; no validated sensor/cell margin (Finding 2). |
| v19.01 30-second low-SoC hourly awake window | **Confirmed correct** | Queue gate still prevents immediate truncation of outstanding cloud publishes; shorter remote-command opportunity is a tradeoff. |
| v19.01 11-minute connect timeout | **Confirmed correct** | Matches Particle's ten-minute reset / eleven-minute wait guidance. |
| v19.01 retained 30-minute failed-connection backoff | **Confirmed correct** | Survives ordinary Boron reset and bounds normal valid-clock failure retries; clock validity and success cycling are separate limitations. |
| v19.01 unconditional ERROR shutdown helper | **Incorrect** | Better than conditional shutdown, but ignoring completion cannot ensure safe hard power-down (Finding 4). |
| v19.01 deferred hard-reset flag and helper | **Incorrect** | Moving work out of the callback is correct; cutting power after an unverified shutdown remains incorrect (Finding 4). |

## Simulation harness problems and validation limits

- The current suite reproduces the dispatch's known failures. v19.01 `nonet`,
  `badnet`, `badnetlow`, and `hardreset` fail rules 4/5; `reboot` fails rule 7.
  Other scenarios pass. v19.00 additionally fails power hygiene in hot/not-charging
  and hard-reset scenarios. No new claim is based on the excluded known failures.
- Every stock charging scenario uses `solarPowerMode=false`, as demonstrated by
  the 4208 mV output. None verifies the intended 5080 mV solar configuration.
- Rule 11 checks only the **last** config/charge state, ignores charge voltage/current,
  and uses the firmware's own rounded temperature as its oracle. It therefore misses
  unsafe quantization and transient charging/configuration changes. `hot115` actually
  produces a firmware value of 114°F; its name is not a calibrated test stimulus.
- Rule 10 misses hour-boundary success cycling with the fixed PIR schedule. Its
  six-per-hour check does not replace the pairwise ten-minute rule. Device OS modem
  recovery must be classified separately from aggressive application retries.
- Rule 9's broad tolerance is reasonable for deferred motion reporting only if it
  is explicitly a product requirement. It cannot establish delivery reliability:
  the outage recovery exclusion is a blanket 41-minute window, connection latency
  is fixed, publish always succeeds, and downstream webhook receipt is fabricated.
  A pending event at simulation end is treated as late even when its next permitted
  connection is after the test horizon. Validate per-event deadlines against policy
  and classify outage/deferred/pending events explicitly.
- `waitFor` has no elapsed-time behavior; shutdown always succeeds, RTC is always
  valid, sensor accuracy/thermal lag are absent, AB1805 watchdog is a no-op, and
  Device OS modem reset is absent. These tests cannot establish hardware shutdown,
  thermal safety, watchdog margin, or retained-memory semantics.
- Reboot uses a handwritten list, leaves static-local/library state alive, clears
  the modem instantly, and treats every reboot as USER. Real AB1805 power-down can
  fail or fall back to `System.reset()`; the stub always throws a successful power
  cycle. Run a new process with persisted FRAM/queue state for reliable reset tests.
- Host `unsigned long`/`time_t` widths differ from the MCU; warnings are suppressed.
  The simulation will not expose real 32-bit `millis()` rollover and ABI/layout errors.
- `check.py` ignores the simulator return code and does not exit nonzero when rules
  fail. `run.sh` therefore succeeds despite failed checks; it is a human-readable
  diagnostic, not an enforceable CI regression test.
- Added `adversarial.cpp` / `adversarial.sh` and the stub's existing-library-compatible
  pause API. Run `bash test/sim/adversarial.sh` for all three reproductions. These
  are intentionally regression witnesses: success means the current defect was
  reproduced, not that the firmware passed. They use existing stubs and do not
  substitute for Boron hardware tests.
- Added `test/sim/out/` and `test/sim/sim` to `.gitignore`. No commits created.
