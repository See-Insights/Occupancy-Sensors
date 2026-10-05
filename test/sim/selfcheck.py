#!/usr/bin/env python3
"""Negative controls for the checker. Run after building sim (run.sh does this)."""
import subprocess
import sys
import datetime as dt
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "out"
OUT.mkdir(exist_ok=True)

def check(scenario, lines, expected, rule=None, strict=False):
    path = OUT / "selfcheck-trace.txt"
    path.write_text("\n".join(lines) + "\n")
    args = [sys.executable, str(ROOT / "check.py"), scenario, "--trace", str(path)]
    if strict:
        args.append("--strict")
    result = subprocess.run(args, capture_output=True, text=True)
    assert result.returncode == expected, result.stdout + result.stderr
    if rule:
        assert any(line.startswith(f"Rule {rule} ") and line.endswith("FAIL") for line in result.stdout.splitlines()), result.stdout

base = subprocess.run([str(ROOT / "sim"), "base"], capture_output=True, text=True, check=True).stdout.splitlines()
check("base", base, 0)

# A later safe final state must not hide the earlier unsafe enable action.
check("base", base + ["CH,123,1191,1,injected-unsafe-enable", "CH,124,881,1,final-safe-enable"], 1, 11)
# A bad stable config must not be hidden by a later correct final config.
check("base", base + ["PS,123,0,0,0", "PS,124,4208,4112,900"], 1, 11)
meta = next(x for x in base if x.startswith("META,")).split(",")
end = int(meta[3])
check("base", base + [f"M,{end + 10}", f"M,{end + 51}"], 1, 10)
blocked = ["META,1," + ",".join(meta[2:]) if x.startswith("META,") else
           "ST,600000,0,0,1,0" if x.startswith("ST,") else x for x in base]
check("stuckqueue", blocked, 1, 12)
# Hysteresis grouping must ignore extra config records without overlooking a
# wrong direct decision inside the thermally safe band (42 C).
hysteresis = ["META,1," + ",".join(meta[2:]) if x.startswith("META,") else x
              for x in base if not x.startswith("CH,")]
actions = []
for i, (adc, on) in enumerate([(869, 1), (1156, 0), (1142, 0), (1118, 0), (1116, 1)]):
    actions.append(f"CH,{100+i},{adc},{on},{'enableCharging' if on else 'disableCharging'}")
check("thermalhysteresis", hysteresis + actions + ["CH,104,1116,1,config-reload"], 0)
wrong = actions.copy()
wrong[2] = "CH,102,1142,1,enableCharging"
check("thermalhysteresis", hysteresis + wrong, 1, 11)
unclean = [x[:3] + ",".join([*x[3:].split(",")[:2], "1", x.split(",")[-1]]) if x.startswith("PM,") else x for x in base]
check("base", unclean, 1, 11)

# Create the explicitly allowlisted cleanup gap from a passing trace. This does
# not assume the reviewed firmware still has that bug or any of F1-F4.
offset = int(next(x for x in base if x.startswith("CFG,")).split(",")[-1])
known = []
removed = 0
for line in base:
    if line.startswith("L,") and "Running daily cleanup" in line:
        timestamp = int(line.split(",", 2)[1])
        if dt.datetime.fromtimestamp(timestamp + offset, dt.timezone.utc).day == 30:
            removed += 1
            continue
    known.append(line)
assert removed == 1
check("reboot", known, 0)
check("reboot", known, 1, strict=True)
check("reboot", known + [f"M,{end + 10}", f"M,{end + 51}"], 1, 10)
check("base", [], 1)  # malformed/missing trace must never pass
result = subprocess.run([sys.executable, str(ROOT / "check.py"), "base", "--sim", "/usr/bin/false"], capture_output=True, text=True)
assert result.returncode == 2 and "SIMULATOR ERROR" in result.stderr
result = subprocess.run([str(ROOT / "sim"), "typo-scenario"], capture_output=True, text=True)
assert result.returncode == 2
print("checker self-checks: unsafe transitions, config, cycling, drain, power cut, allowlist and simulator errors PASS")
