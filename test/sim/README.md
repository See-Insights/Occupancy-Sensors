# Hardened host simulation

Run the complete suite from the repository root:

```sh
test/sim/run.sh
test/sim/run.sh test/sim/baselines/Occupancy-Sensors-v19.02-reviewed.ino
```

The runner snapshots the input into `out/`, records its SHA-256 in a build manifest,
builds the simulator, checks the timing model and checker with negative controls,
then runs 31 scenarios. It exits **1** if any unexpected rule failure occurs;
build/model/checker errors also exit nonzero. It continues through scenario failures
so the final run includes every scenario. `PASS`, `KNOWN`, and `FAIL` are distinct.
No firmware files are edited.

`known_gaps.json` names exact messages for the previously documented outage/hourly,
closed-hours reboot cleanup, and hard-reset timing gaps. An additional failure in
the same scenario or rule is not exempt. Fixed gaps may disappear. Nothing from
F1–F4 or the newly exposed Power Manager charging behavior is exempt.

After building, individual checks work from any directory:

```sh
python3 test/sim/check.py solar
python3 test/sim/check.py staledrain
python3 test/sim/check.py nonet --strict
python3 test/sim/check.py solar --trace test/sim/out/Occupancy-Sensors_solar.trace
python3 test/sim/selfcheck.py
```

`--strict` disables exemptions. `--sim PATH` selects another built simulator.
`--save-trace PATH` preserves all raw instrumentation records. The runner writes
both `.txt` check reports and `.trace` instrumentation files into `out/`.

## Coverage added

| Scenarios | Purpose |
|---|---|
| `solar`, `solarhot`, `solarnotchg` | Boot from a solar-mode FRAM fixture; verify 5080 mV VIN minimum, 4208 mV termination, and 1024 mA requested charge-current configuration at every stable checkpoint. |
| `adc45`, `chargecycle` | Use unrounded ADC-derived Celsius as the independent safety oracle; inspect every charging action/change, including an unsafe intermediate action followed by a safe final state. |
| `thermalhysteresis` | Exercise on → ≥43°C off → 42°C off → 40.1°C off → 39.9°C on; account for ADC quantization. |
| `boundary`, `boundarylow`, `boundaryfailed`, `boundaryclosing` | Hour-boundary reconnects at 60%, falling to 45% before the next three-hour connection slot, failure/backoff, and closing time. Successful report delivery is checked as well as modem power-ups. |
| `stuckqueue` | One continuous queue-drain episode with motion every 65 seconds and one-minute debounce; new reports cannot extend its 120-second deadline. |
| `staledrain` | LPM is disabled during a pending drain; the queue empties, then a new closing report encounters a 60-second transient queue failure. Its budget must not inherit the old expired deadline. |
| `shutdownslow` | Cloud disconnect takes 5 seconds and modem shutdown 20 seconds; power must not be cut early. |
| `shutdownfail`, `shutdownerror12`, `shutdownerror13` | Modem shutdown never completes; hard reset and both ERROR power-cycle paths must not cut board power. |
| `shutdownpowerfail` | Modem shuts down, but AB1805 power-down returns false; verify `System.reset()` fallback. |

The original 13 scenarios remain. Focused scenarios intentionally skip full-day
occupancy rules; their own safety/delivery checks still execute.

## Model changes

- `waitFor` polls while virtual time elapses, succeeds when its condition becomes
  true, and returns false on timeout. Delays advance time too. PIR, network/outage,
  delayed shutdown, webhook responses and the background publisher continue to
  progress during these waits. Conditions are not evaluated only once.
- Cellular and cloud shutdown can be delayed or fail independently. AB1805 power-down
  can fail without removing power. Actual power cuts and reset fallbacks are traced.
- Charging actions carry the raw ADC value, virtual timestamp, enabled state and
  cause. Config application and stable checkpoints retain voltage/current settings.
  Disabled charging in the safe band is valid; the oracle does not force it on.
- Power Manager reload can re-enable a direct PMIC charge disable. Wake reload is
  modeled using Device OS 6.4.1 `system_sleep.cpp:205` and
  `system_power_manager.cpp:469,526,287`. Configured `DISABLE_CHARGING` survives this
  reload; a direct `PMIC::disableCharging()` does not. This behavior is also explained
  by [Particle's power-feature reference](https://docs.particle.io/reference/device-os/api/power-manager/systempowerfeature/#systempowerfeature-disable-charging).
- `check.py` checks the simulator exit code and mandatory trace records. Negative
  controls prove an unsafe intermediate charging action, a transient bad stable
  config, short modem cycle, stale/long drain, unclean power cut, simulator error,
  malformed trace, or unallowlisted failure cannot silently pass.
- The runner resets the known v19.02 RAM globals on simulated reboot. The historic
  `adversarial.cpp` remains as the old inverse-success witness; `adversarial.sh`
  now delegates to the normal suite so its exit status has ordinary test meaning.

## Remaining limitations

This is a host model, not a Boron integration test. Device OS's ten-minute modem
recovery, AB1805 watchdog expiry, actual battery thermal lag/sensor error, fuel-gauge
estimation, modem electrical failure modes and flash wear are still unmodeled.
Configuration/wake reloads run synchronously to represent one real interleaving;
the actual Power Manager queues them on its thread. The temperature safety oracle
uses an ideal TMP36 transfer function with the dispatch's assumed 0–45°C cell range.

The handwritten reboot fixture still cannot reset function-static variables or
arbitrary new globals automatically. Fresh-process focused scenarios avoid that
limitation, but the scripted multi-reboot case is not proof of reset semantics.
Host ABI widths still differ from the MCU despite wrapping the `millis()` value.
The pre-existing delivery oracle's broad outage recovery exclusions remain; this
hardening does not claim to prove downstream Ubidots durability.

## v19.03 review adjustments

The isolated `stuckqueue` test starts at 06:10 and seeds `lastConnection`, so the
120-second drain test does not mix in an hour-boundary reconnect/webhook wait.
The optional `stuckqueueboundary` reproduces that separate cross-hour case with
both modem starts recorded. Disconnect transitions reset the episode metric even
when no input tick sees the offline interval. `thermalhysteresis` selects each
measurement's direct PMIC policy decision; all config reload actions still undergo
the raw-ADC safety check. Checker controls reject a wrong 42 C policy decision.

`hotboot` boots at ADC 1195 with charging previously disabled. It checks whether
setup preserves that safety state until the first measurement. This is a fresh
process, not the handwritten reboot model. `locking_witness.py` uses host threads
to reproduce the source-derived one-slot queue/PMIC-lock cycle and a control that
releases the lock before enqueue. It is not a Device OS or firmware integration
execution; its successful exit means the witness and control behaved as expected,
not that the firmware is safe. The ordinary simulator still has no PMIC locks or
asynchronous config queue.
