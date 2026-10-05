#!/bin/bash
# Host simulation of Occupancy-Sensors firmware - builds against stand-in Particle libraries and checks 11 rules per scenario
# Usage: test/sim/run.sh [firmware.ino]   (default: src/Occupancy-Sensors.ino). Results land in test/sim/out/<label>_<scenario>.txt
set -euo pipefail
R=$(cd "$(dirname "$0")/../.." && pwd)
arg=${1:-$R/src/Occupancy-Sensors.ino}
fw=$(cd "$(dirname "$arg")" && pwd)/$(basename "$arg")             # Resolve before changing directory
cd "$(dirname "$0")"
v=$(basename "$fw" .ino); mkdir -p out
snapshot="$PWD/out/${v}_firmware.ino"
cp "$fw" "$snapshot"
python3 - "$snapshot" "$fw" "out/${v}_build.json" <<'PY'
import hashlib, json, pathlib, sys
snapshot, source, manifest = map(pathlib.Path, sys.argv[1:])
manifest.write_text(json.dumps({"source": str(source), "snapshot": str(snapshot),
                              "sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest()}, indent=2) + "\n")
PY
globals1901=0
globals1902=0
globals1903=0
if grep -q hardResetRequested "$snapshot"; then globals1901=1; fi
if grep -q '^unsigned long drainStartMs' "$snapshot"; then globals1902=1; fi
if grep -q '^bool chargingAllowed' "$snapshot"; then globals1903=1; fi
clang++ -std=c++17 -Wno-format -Istubs -I"$R/src" -DFIRMWARE="\"$snapshot\"" -DHAS_V1901_GLOBALS="$globals1901" -DHAS_V1902_GLOBALS="$globals1902" -DHAS_V1903_GLOBALS="$globals1903" -x c++ sim.cpp -o sim
clang++ -std=c++17 model_checks.cpp -o out/model_checks
./out/model_checks
python3 selfcheck.py
python3 locking_witness.py
echo "######## $v"
failed=0
for s in base connected lowbatt slowconn verylow nonet reboot hot hot115 notchg badnet badnetlow hardreset solar solarhot solarnotchg boundary boundarylow boundaryfailed boundaryclosing stuckqueue staledrain hotboot adc45 chargecycle thermalhysteresis shutdownslow shutdownfail shutdownpowerfail shutdownerror12 shutdownerror13; do
  status=PASS
  if ! python3 check.py "$s" --save-trace "out/${v}_$s.trace" > "out/${v}_$s.txt" 2>&1; then
    status=FAIL
    failed=1
  elif grep -q '^KNOWN GAP' "out/${v}_$s.txt"; then
    status=KNOWN
  fi
  printf "%-18s %-5s | %s\n" "$s" "$status" "$(tail -1 "out/${v}_$s.txt")"
done
exit "$failed"
