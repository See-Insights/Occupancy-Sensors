# Re-review: Occupancy-Sensors v19.02

You've already reviewed v19.00 and v19.01 (`test/CODEX-REVIEW-v19.01-results.md`) and hardened `test/sim/`. v19.02 makes the fixes for your four findings and one cleanup you raised. Use your hardened harness to check them.

- **Scope:** the diff from v19.01 to v19.02 in `src/Occupancy-Sensors.ino`, about 26 lines added and 18 removed. Anything else in the file is out of scope unless a v19.02 change makes it worse.
- **Constraint:** the developer asked for **minimal edits with no bloat**. A finding that only says the code could be more defensive will be deferred. Show a concrete failure.
- **Rules:** don't edit `src/`, and don't commit.

## What changed

| Your finding | v19.02 change | Line(s) |
|---|---|---|
| **F1** Modem power-ups 41 s apart around the hour boundary | NAPPING uses `INACTIVE_STANDBY` when all of these hold: the modem is on, the next hour is a connect hour (SoC >50%, or the next hour is divisible by 3), the device isn't in `lowBatteryMode`, and the top of the hour is less than 600 s away. Otherwise it behaves as before. | 393–394 |
| **F2** Truncated temperature allowed charging above 45°C | Charging decisions use an unrounded `float temperatureC`. Charging stops at **≥43°C** and resumes **below 40°C** (a `static bool` in `isItSafeToCharge()`), and stays off below 2.2°C (36°F). The developer chose these limits: the carrier-board TMP36 is a few mm from the battery. The reported `current.temperature` keeps its old rounding. | 148, 790–792, 826–827 |
| **F3** Drain deadline kept moving | New `drainStartMs`. It's set the first time `readyToDisconnect()` finds the device connected with a non-empty queue, and cleared when the device is offline or the queue is empty. New reports don't extend it. | 147, 621–623 |
| **F4** Power cut after an unconfirmed modem shutdown | `disconnectFromParticle()` returns the `waitFor(Cellular.isOff, 30000)` result. Alerts 12 and 13 and the hard reset call `deepPowerDown()` only when that result is true. Otherwise, or if `deepPowerDown()` returns false, they call `System.reset()`. | 539–563, 593–594, 960 |
| Cleanup (your Q5) | `setPowerConfig()` no longer applies the defaults before the custom config. The not-charging re-apply, and alert 11, happen at most once per calendar day. | 766–770, 846 |

**Deliberately not changed:**
- **`Time.isValid()` in the backoff:** an invalid or earlier clock already fails `Time.now() >= lastFailedConnection`, so the backoff is skipped rather than blocking.
- **Backoff growing from 30 to 60 min:** optional, and the developer didn't ask for it.
- **Items you marked as existing before v19:** the fake `lastConnection` written by the ERROR path, the out-of-memory timer, `quickStart()` in setup, and non-whole-hour timezones.

## The agent's own results

These come from its scratchpad copy of the old harness, not your hardened version.

- **Your `adversarial.sh` reproductions:** none of them reproduce against v19.02.
  - **boundary:** one modem power-up; the 07:00 report goes out at 07:00:05 over the modem left in standby.
  - **temperature:** at 45.95°C charging is off.
  - **drain:** the device is ready to disconnect after 120 s with 5 events still queued.
- **New `stuckmodem` scenario** (`Cellular.off()` never completes, then a hard reset): v19.01 cut power with the modem on, while v19.02 does a `System.reset()` instead.
- **Temperature sweep** (30, 42.9, 43, 45.95, 42, 40.5, 40, 39.9, 41, 42.9, 43.1, 1, 3 °C): charging was 1, 1, 1\*, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1. The 43.0°C stimulus reads as 42.97°C after the 12-bit ADC (about 0.08°C per step).
- **Full suite:** same results as v19.01, apart from the stuck-modem fix. The known failures are unchanged (rule 7 in `reboot`; rules 4 and 5 in outage and hard-reset scenarios).

## What to check

1. Are F1–F4 actually closed? Try variants:
   - F1 at ≤50% across an hour divisible by 3, after a failed connection, and at closing time
   - F3 when the deadline expires while the device stays connected (LPM off, SLEEPING_STATE)
   - F4 when `deepPowerDown()` itself fails
2. Did any fix introduce a regression? For example:
   - an F1 standby nap that keeps the modem on far longer than 10 min
   - a stale `drainStartMs` from an earlier connection cutting a later drain short
   - the static `tooHot` flag across reboots (RAM is cleared, so the device resumes charging at 40–43°C after a reset)
   - `int(temperatureC)` rounding toward zero for negative temperatures
3. Run your hardened harness against v19.02 and report results that differ from the v19.01 results.

Report in the same format as `CODEX-REVIEW-v19.01-results.md`. Write it to `test/CODEX-REVIEW-v19.02-results.md`.
