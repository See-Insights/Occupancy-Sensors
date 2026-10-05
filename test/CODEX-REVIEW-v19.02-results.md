# Re-review of Occupancy-Sensors v19.02

Reviewed 2026-10-05 against the hardened `test/sim/` suite and the new v19.02 dispatch.
Scope is F1–F4, the PMIC cleanup, and regressions introduced by those changes.
No `src/` files were edited and no commits were made.

Revalidated on the explicit review request: the current firmware matches the
reviewed snapshot byte-for-byte. A fresh complete hardened-suite run reproduced
the results below, and the v19.00 `staledrain` control again delivered the closing
report after 60 seconds. No additional in-scope findings were confirmed.

The reviewed `.ino` SHA-256 is
`0d03afc09bfface812d9d388dcc4fffb4c55c75729f308c120331d4df95e5441`.
A copy is retained in `test/sim/baselines/Occupancy-Sensors-v19.02-reviewed.ino`.
The workspace targets Device OS 6.4.1. Source references below pin that version;
verify the deployed Device OS separately.

## Finding: F3 is improved, but a stale deadline can strand the closing report

- **Severity:** Medium.
- **Location:** `src/Occupancy-Sensors.ino:621`, interacting with the LPM-only call
  at 337 and the closing-time call at 352.
- **Failure scenario:** At 20:50 a connected LPM unit begins waiting for a queued
  event. The operator disables LPM through the console while it is still connected.
  The event is acknowledged and the queue empties, but IDLE no longer calls
  `readyToDisconnect()` because `lowPowerMode` is false. `drainStartMs` therefore
  retains 20:50. At 21:00 the new final report encounters a transient publish/ACK
  failure that recovers at 21:01. After the usual 30-second webhook wait, SLEEPING
  calls `readyToDisconnect()`: the old 20:50 timestamp is already expired, so it
  disconnects immediately. The final event stays queued through closed hours,
  despite a recovery that would have fit within a fresh 120-second drain budget.
- **Evidence:** `python3 test/sim/check.py staledrain` exits 1. It generates the
  20:50 and 21:00 reports, delivers only the former, and reports Rule 12:
  `new closing report not delivered within 120 s despite queue recovery after 60 s; earlier drain must not shorten this wait`.
  The connected closing queue episode lasts approximately 30.49 seconds before
  disconnection. As a control, the supplied v19.00 baseline delivers the same
  closing report at 21:01:00 (60 seconds late) and passes. The original v19.01
  helper also used the already-connected/report timestamp, rather than the new
  persistent drain timestamp; that specific stale-state regression was absent.
- **Suggested minimal fix:** Clear `drainStartMs` when the queue becomes empty or
  the cloud becomes disconnected even when LPM is disabled. For example, put that
  reset condition in ordinary housekeeping rather than relying exclusively on
  calls to `readyToDisconnect()`. Keep the start timestamp unchanged while one
  continuous connected/nonempty episode remains active. Do not reset it on every
  report, which would restore the original F3 defect.

## F1–F4 and cleanup verdicts

| Change | Assessment | Evidence / reasoning |
|---|---|---|
| F1 standby near a connection boundary | **Confirmed correct for the reviewed whole-hour configuration** | `boundary`, `boundarylow`, `boundaryfailed`, and `boundaryclosing` pass. The original boundary fixture now has one modem power-up; the next hourly report is delivered. Falling to 45% before the 09:00 slot retains standby, and closing does not require another short-cycle modem start. |
| F2 unrounded Celsius and 43°C/40°C hysteresis | **Confirmed correct for the original rounding finding** | `adc45` disables charging at ADC 1191 = 45.9546°C. `chargecycle` inspects the intermediate action, and `thermalhysteresis` passes the on/off/off/off/on sequence. This confirms software decisions, not calibrated cell-temperature safety. The pre-existing Power Manager issue below remains separate. |
| F3 separate drain deadline | **Incorrect in the stale-deadline variant** | Original `stuckqueue` now disconnects after approximately 120.2 seconds from its first explicit drain request instead of remaining connected for 600 seconds. However, `staledrain` fails as described above. |
| F4 verified modem-off result before hard power cut | **Confirmed correct** | `shutdownslow` cuts power with the modem off after 25 seconds (5-second cloud wait + 20-second modem wait). `shutdownfail`, `shutdownerror12`, and `shutdownerror13` reach timeout and capture `System.reset()` with no board-power cut. `shutdownpowerfail` confirms the AB1805 failure fallback. |
| One complete PMIC config and once-per-day recovery | **Confirmed correct for the change** | Solar fixtures preserve requested 5080/4208 mV and 1024 mA settings. Source now applies one complete config and caps not-charging recovery by day. Time changes, reboot-static behavior and actual PMIC persistence/wear remain outside this host proof. |

The drain budget begins at the first drain request, not at first publish enqueue.
The pre-existing webhook/state-machine wait can precede it. The `stuckqueue` fixture
explicitly begins that request and then exercises the real loop, without reading
or resetting the firmware's drain global. This distinguishes the original movable
deadline defect from a stricter total-awake-time requirement.

## Newly exposed thermal limitation — pre-existing, not a v19.02 regression

The hardened harness now models a Device OS behavior absent from the old harness:
Power Manager initialization after sleep can undo direct `PMIC::disableCharging()`.
Both `hot115` and `solarhot` produce 53 unsafe wake-reload transitions at ADC 1195
= **46.2769°C**. Their final charge state can still be off, which is why the original
final-state-only check missed this exposure.

**Concrete sequence:** A hot, connected-battery unit disables charging in
`isItSafeToCharge()` and sleeps. Device OS wakes it and reloads the Power Manager
configuration. Because the config lacks `DISABLE_CHARGING`, that reload enables
charging. The application may later take a measurement and disable it again;
during closed hours it can wake and sleep without taking another measurement.
The same mechanism applies below the cell's minimum charge temperature.

**Location:** `src/Occupancy-Sensors.ino:793` and `846`, plus Device OS
`system/src/system_sleep.cpp:205` and `system/src/system_power_manager.cpp:469,526,287`.
The power-manager thread's wake branch calls `initDefault()`, which calls
`handleCharging()` and enables charging unless its configuration disables it.

**Evidence:** [Particle's SystemPowerFeature documentation](https://docs.particle.io/reference/device-os/api/power-manager/systempowerfeature/#systempowerfeature-disable-charging)
explicitly distinguishes persistent configured disable from a direct PMIC write.
[Pinned Device OS 6.4.1 power-manager source](https://github.com/particle-iot/device-os/blob/v6.4.1/system/src/system_power_manager.cpp#L468)
contains the wake reload. The simulator implements one valid interleaving in which
that thread runs before application processing resumes; actual thread scheduling
can vary. This is source-backed application/Device OS behavior, not a hardware
temperature experiment.

**Suggested follow-up:** Include `SystemPowerFeature::DISABLE_CHARGING` in the full
custom configuration while thermal policy forbids charging, and clear it only when
the hysteresis policy permits charging. Preserve solar/DC limits in every config
application, including mode changes, and handle the asynchronous config reload.
An immediate direct PMIC disable may still be useful while the queued config applies.

This is an important **pre-existing exposure**, not a new finding charged to the
v19.02 patch under the dispatch's scope rule. It is not silently allowlisted: the
hardened suite correctly remains red until addressed or explicitly classified by
the developer. The specific F2 truncation defect is fixed.

## Hardened-suite results and comparison with v19.01

`test/sim/run.sh` runs 30 scenarios and exits **1** on this reviewed v19.02 snapshot:

- **22 PASS:** original normal cases, solar/non-charging settings, all four boundary
  variants, original blocked-queue deadline, ADC rounding/hysteresis, and all five
  shutdown variants.
- **5 KNOWN:** `nonet`, `badnet`, `badnetlow`, `reboot`, and `hardreset`. Only exact
  previously documented failure messages are exempt; unexpected additions still
  fail. These results do not claim the known problems are fixed.
- **3 FAIL:** `staledrain` is the v19.02 F3 regression; `hot115` and `solarhot` expose
  the older Power Manager thermal problem. Neither is allowlisted.

During harness development, current v19.01 reproduced the original 41-second
modem cycle, unsafe 45.9546°C charge enable, approximately 600-second blocked queue,
and unclean hard-reset power cut. v19.02 passes those original focused checks.
The older final-state hot tests passing does not contradict their new failures:
the hardening checks intermediate charging transitions and the wake reload.

Model checks and checker negative controls pass. They verify real virtual-time
waits, failed shutdown, persistent charge-disable behavior, rejection of unsafe
intermediate transitions/config states, short modem cycles, long drain episodes,
unclean cuts, simulator errors, malformed traces, and unallowlisted failures even
inside an allowlisted scenario. The runner snapshots its input and records a hash
so concurrent firmware edits cannot change the meaning of one compiled test run.

## Harness limitations still relevant to this review

- Publish stalls are represented by a paused background publisher; this reproduces
  lack of ACK progress but does not model the modem/protocol failure that causes it.
- Device OS's own ten-minute modem recovery and physical modem shutdown failure
  modes remain unmodeled. Shutdown timing tests prove caller decisions, not electrical
  safety or immunity to watchdog resets.
- The AB1805 watchdog is still a no-op. Sensor accuracy, battery/board thermal lag,
  fuel-gauge estimation, and PMIC register/current quantization are not reproduced.
- Power Manager configuration/wake reload is synchronous in the harness, representing
  one possible real thread ordering. Other interleavings need hardware validation.
- The manual multi-reboot fixture resets known v19.02 globals but cannot automatically
  clear every function-static variable (including `tooHot` and `pmicReappliedDay`).
  Their real reset behavior follows ordinary non-retained RAM semantics; the suite
  does not independently establish that lifecycle. A reset at 40–43°C may resume
  charging because hysteresis history is lost, as the dispatch anticipated; the
  raw reading remains below the chosen 43°C initial cutoff.
- Full-day delivery checks retain the older outage-recovery tolerance and synthetic
  webhook responses. They are not proof of downstream Ubidots durability.

Implementation and usage details are in `test/sim/README.md`. All harness edits are
under `test/sim/`; this report follows the dispatch's requested review destination.
