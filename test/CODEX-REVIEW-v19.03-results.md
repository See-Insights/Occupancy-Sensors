# Adversarial review results — v19.03

**Release verdict: two blocking findings under the dispatch criteria.** The new
thermal persistence call permits a lock/queue deadlock. Hot boot still enables
charging outside the specified temperature range; this is pre-existing, but the
dispatch explicitly makes any such Critical finding blocking.

Reviewed the v19.02 → v19.03 diff, dispatch questions, and hardened harness.
No `src/` files were edited and no commits were made. Reviewed SHA-256:
`e54f65c5dfc85d91afb35a2300140351b0d5f652c924d5510c66f8390823cea5`.
The runner preserved the compiled input and hash in `test/sim/out/`.
Device OS analysis is pinned to the workspace's installed 6.4.1 sources.

## Finding 1: configuration enqueue while holding PMIC can deadlock

- **Severity:** High; **new v19.03 regression**, therefore release-blocking.
- **Location:** `src/Occupancy-Sensors.ino:792,797`, calling `setPowerConfig()`
  at 867 or 878.
- **Concrete failure schedule:** The application acquires `PMIC pmic(true)`.
  Power Manager consumes a Wakeup and blocks acquiring that same lock. A PMIC
  interrupt queues an Update. The thermal decision changes, and the application
  calls `setPowerConfig()` before releasing its lock. Configuration enqueue waits
  indefinitely for queue space. Power Manager cannot consume the queued event
  until the application releases the lock; the application cannot release it
  until enqueue returns. Neither thread can progress. A thermal transition can
  also stall before reaching the direct charge-disable call.
- **Source evidence:** Device OS creates a **one-entry queue**, Update can fill it,
  and configuration enqueue uses an **unlimited wait**. Its wake/update handlers
  acquire PMIC. See [Device OS 6.4.1 Power Manager](https://github.com/particle-iot/device-os/blob/v6.4.1/system/src/system_power_manager.cpp)
  at lines 134, 249, 324, 469, 503, and 1010. PMIC's constructor/destructor acquire
  and release the Wire lock in the installed `wiring/src/spark_wiring_power.cpp`
  at 93–104, with lock operations at 1210–1215.
- **Reproducer:** `python3 test/sim/locking_witness.py`. A deterministic host-thread
  witness reproduces the blocked queue and lock; its release-before-enqueue
  control completes. This models the source-derived schedule, **not execution of
  Particle binaries**. The ordinary simulator cannot expose this: its PMIC lock
  is a no-op and configuration reload is synchronous. v19.02 has no configuration
  enqueue inside this thermal lock, so it has no corresponding wait cycle.
- **Minimal correction:** End the PMIC critical section before calling
  `setPowerConfig()`. Apply the immediate thermal PMIC decision within that short
  section, then persist the changed policy after the lock has been released.
  A watchdog may recover a stalled board; that does not make this path correct.

## Finding 2: hot boot overwrites the persisted safety flag before measuring

- **Severity:** Critical **under the dispatch's outside-0–45°C charging criterion**;
  pre-existing boot exposure, not a v19.03 regression.
- **Location:** `src/Occupancy-Sensors.ino:150,292,300,852`.
- **Failure scenario:** A hot unit previously disabled charging and persisted that
  policy. It reboots while still at ADC 1195, or **46.2769°C** under the harness's
  ideal TMP36 conversion. The RAM initializer makes `chargingAllowed = true`.
  Setup's first `setPowerConfig()` consequently removes `DISABLE_CHARGING` before
  `takeMeasurements()`. If Power Manager applies that enabled config before the
  correction, charging is enabled outside the permitted range. The later correct
  measurement does not erase this intermediate exposure.
- **Evidence:** `python3 test/sim/check.py hotboot` exits 1. Trace:
  ```text
  CH,500,1195,1,config-reload
  CH,500,1195,0,config-reload
  CH,500,1195,0,disableCharging
  ```
  The v19.02 control also fails this test, confirming its classification. A fresh
  process boots with the prior disabled state; no handwritten reboot-static reset
  assumption is needed. The synchronous reload represents an allowed thread
  ordering, not measured hardware timing. Both transitions share a model timestamp:
  **the test proves an unsafe enable decision, not pulse duration or actual current**.
  Hardware current and sensor-to-cell calibration remain unmeasured.
- **Minimal correction:** Start with charging disallowed and preserve that policy
  through setup until the first temperature measurement authorizes charging.
  Also address Finding 1 before persisting that first decision.

## F3, thermal policy, and harness verdicts

| Question | Verdict |
|---|---|
| Stale drain with LPM disabled | Fixed: v19.03 `staledrain` passes; v19.02 fails the identical fixture. Every-loop housekeeping handles the observed empty/offline state independently of LPM. |
| Original movable deadline | Still fixed: corrected `stuckqueue` passes v19.02 and v19.03; v19.00 remains connected with the blocked queue for 600,000 ms and fails. New reports do not move the drain start. |
| Sustained hot wake/reload | `hot115` and `solarhot` pass v19.03; both fail v19.02. The persisted flag fixes the previously reproduced steady hot-wake path. This does not prove all asynchronous orderings or boot safety. |
| Hysteresis | Correct on/off/off/off/on decisions at approximately 20, 43.13, 42.0, 40.07, and 39.91°C. The prior failure counted extra config records as measurements. |
| Solar/DC mode changes | Source carries the flag in the common full configuration before either mode branch; both console mode paths call it. Solar suite checkpoints retain 5080/4208 mV and 1024 mA requested settings. |
| Flash persistence | Normal stable-temperature reloads do not imply repeated physical writes: identical configurations are compared before DCT storage. Hot/cold boot currently flips the flag twice, producing two changed configuration stores per boot. |
| F1/F2/F4 guards | Existing boundary, raw-ADC, intermediate-charge, and shutdown success/failure checks pass. |

[Particle documents configured charge-disable persistence](https://docs.particle.io/reference/device-os/api/power-manager/systempowerfeature/#systempowerfeature-disable-charging).
The identical-config comparison is in installed Device OS 6.4.1
`hal/shared/power_hal.cpp:46–55`. Physical flash endurance was not measured.

**Harness corrections, all under `test/sim/`:**

- `stuckqueue` now seeds `lastConnection` and starts at 06:10, isolating its
  explicitly started 120-second drain from the separate hour-boundary reconnect
  and webhook wait. The prior 157,400 ms result mixed those phases; it did not
  establish that a single drain deadline moved. Disconnect transitions also reset
  episode instrumentation immediately, even if no input tick observes the gap.
- `thermalhysteresis` selects the direct policy action for each measurement;
  configuration reloads remain subject to the independent safety oracle.
  Negative controls accept extra config records and reject an incorrect enable
  at 42°C, where the absolute-temperature check alone would pass.
- Added `hotboot` to the main suite, and the separate source-derived locking
  witness. Added optional `stuckqueueboundary` to test the dispatch's real-cycle
  question. No new known-gap exemptions were added.

## Validation

`test/sim/run.sh` runs **31 scenarios: 25 PASS, 5 KNOWN, 1 FAIL (`hotboot`)**,
exiting **1**. Model checks, checker negative controls, and the locking witness
complete successfully. Success of the witness means the hazard and its control
were reproduced; it is not a firmware safety pass.

The same hardened suite against the preserved v19.02 snapshot gives
**22 PASS, 5 KNOWN, 4 FAIL**: `staledrain`, `hot115`, `solarhot`, and `hotboot`.
The isolated v19.00 `stuckqueue` control fails as intended. Reports, traces and
build manifests are in `test/sim/out/`; usage and limitations are in
`test/sim/README.md`.

The model does not execute Device OS concurrency, physical flash writes, modem
recovery at ten minutes, AB1805 watchdog expiry, actual charge current, or sensor
error/thermal lag. Scripted reboot tests still cannot reset arbitrary function
statics. The locking finding relies on the pinned source and explicit thread
schedule; board validation should establish occurrence rate and recovery behavior.

## Backlog

- **Pre-existing hour-boundary short cycle:** With a correctly seeded successful
  06:59:20 connection at 60% and a stalled queue, `stuckqueueboundary` powers the
  modem again at 07:01:26: starts only **126 seconds apart**, with roughly six
  seconds off. The already-connected hourly-report branch does not refresh
  `lastConnection`; after disconnect, the new clock hour permits reconnect.
  Both v19.02 and v19.03 reproduce it, so it is not a new regression. This proves
  the software timing violation, not a SIM ban or bricked modem; bench/later-release
  follow-up under the dispatch's criteria.
- **Boot flash churn:** Two changed stores per hot/cold reboot deserve reset-loop
  endurance testing. No demonstrated wear failure or unsupported lifetime estimate.
- **Asynchronous thermal ordering:** A Wakeup already using an older cached config
  can apply that policy before the new ReloadConfig is consumed. Steady-state
  persistence tests cover one ordering; benchmark transition timing on the board.
  The boot exposure and lock cycle above already provide concrete blockers.
- The five exact allowlisted legacy failures remain unchanged. They were not
  widened or reclassified as fixes.
