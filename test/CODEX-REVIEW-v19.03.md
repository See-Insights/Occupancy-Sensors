# Adversarial review: Occupancy-Sensors v19.03

v19.03 changes the code for your two open items from `test/CODEX-REVIEW-v19.02-results.md`: the F3 stale drain deadline, and the Power Manager wake reload that switches charging back on.

The run of your hardened suite against v19.03 also fails two fixtures, `stuckqueue` and `thermalhysteresis`. The agent believes these are harness assumptions, not firmware defects (evidence below). **Try to prove it wrong.**

- **This is the last software review round.** Only these findings block release:
  - a **regression** that v19.03 introduced in behavior that worked in v19.02
  - any **Critical** finding: a bricked modem, a SIM ban, charging outside 0–45°C, or permanent loss of connectivity or data

  Report everything else (Medium or Low, edge cases, pre-existing issues, harness ideas) in a short **Backlog** section at the end. Those go to bench testing and later releases; they won't be fixed now. Don't widen the scope beyond the items below.
- **Rules:** same as before. Don't edit `src/`, don't commit, and keep to the no-bloat rule: a finding needs a concrete failure.
- **Scope:** the diff from v19.02 to v19.03, 9 lines added and 3 removed. You can diff against `test/sim/baselines/Occupancy-Sensors-v19.02-reviewed.ino`.
- **File hash:** `src/Occupancy-Sensors.ino` SHA-256 is `e54f65c5dfc85d91afb35a2300140351b0d5f652c924d5510c66f8390823cea5`.

## What changed

| Item | v19.03 change | Line(s) |
|---|---|---|
| **F3 stale drain** (your Medium) | `readyToDisconnect()` no longer clears `drainStartMs`. A housekeeping line at the end of every `loop()` clears it whenever the device is offline or the queue is empty, in any mode, including with LPM off. The deadline is still set once per continuous connected, non-empty stretch, and new reports don't extend it. | 599, 624 |
| **Thermal: Power Manager wake reload** (the existing problem you exposed) | New global `chargingAllowed`. `isItSafeToCharge()` computes `allowed = !tooHot && temperatureC >= 2.2`. When that changes, it stores it and calls `setPowerConfig()`, which adds `SystemPowerFeature::DISABLE_CHARGING` to the full config when charging isn't allowed. The direct `pmic.disableCharging()` / `enableCharging()` calls stay, so the change takes effect immediately. | 150, 795–797, 852 |
| Version | History comment and `"19.03"` | 45, 62 |

**Harness edit by the agent:** in `test/sim/`, `run.sh` and `sim.cpp` now detect `^bool chargingAllowed` and reset it to `true` on a simulated reboot, in the same way as the existing v19.01 and v19.02 global blocks. Nothing else in `test/sim/` was changed.

## Hardened suite on v19.03 (`test/sim/run.sh`)

- **23 PASS**, including `staledrain`, `hot115` and `solarhot`, which failed on v19.02.
- **5 KNOWN**, unchanged.
- **2 FAIL.** The agent's analysis of each:

**`stuckqueue`: 157,400 ms blocked, against a 121,000 ms budget**
1. The deadline expires at about 120 s, and the device naps with the modem off.
2. At 126 s a motion report reconnects. This is the only `M` record, because the fixture forced the initial connection without going through CONNECTING, so `sysStatus.lastConnection` was never set and the once-per-clock-hour rule at 60% lets the reconnect through.
3. v19.03 gives that new connection a fresh 120 s deadline. That's the "once per connection" behavior your F3 fix asked for.
4. The checker adds up blocked time across both connections.

v19.02 also reconnects at 126 s (same `M,1790679686`). It passes only because its deadline from the first connection had already expired and was never cleared, which is the `staledrain` defect.

The agent's view is that the fixture should set `lastConnection` like a real connection would, or measure per connection. Show otherwise if you disagree. Also check whether a real device can reach that 6-second off/on cycle once `lastConnection` is set correctly.

**`thermalhysteresis`: "gave [False, False, False, True, True]"**
- The check reads the last five `CH` records and assumes one record per measurement. v19.03 correctly logs a `config-reload` record on each on/off change.
- The trace, per measurement: 20°C on → 43.13 `config-reload` + `disableCharging` off → 42.0 off → 40.07 off → 39.91 `config-reload` + `enableCharging` on. That matches on/off/off/off/on.

The agent's view is that the checker should group records by measurement.

## What to check

1. **F3:** is the stale-deadline path closed in every mode, without reopening the original movable-deadline defect?
2. **Thermal:**
   - Does `DISABLE_CHARGING` in the config survive every wake and reload ordering?
   - **Flash wear:** `System.setPowerConfiguration()` persists the config to flash. Writes now happen on each thermal change, which hysteresis limits, plus at most one not-charging re-apply per day. Is there a path that writes much more often?
   - **Locking:** `setPowerConfig()` now runs while `PMIC pmic(true)` holds its lock inside `isItSafeToCharge()`. Can that deadlock or stall the Power Manager thread?
   - **Boot:** `setup()` calls `setPowerConfig()` with `chargingAllowed = true`, then `takeMeasurements()` corrects it. Is a hot boot exposed during that gap?
   - **Mode change:** does `setSolarMode()` carry the flag correctly?
3. Are the two remaining failures harness artifacts, as claimed? If you agree, fix the fixtures and make sure they still catch the defects they were written for. Otherwise report a firmware finding.

Write results to `test/CODEX-REVIEW-v19.03-results.md` in the same format as before.
