#!/usr/bin/env python3
# Independent report, power, thermal, drain and shutdown checks.
import argparse, subprocess, sys, json, datetime as dt
from pathlib import Path
from collections import defaultdict

parser = argparse.ArgumentParser()
parser.add_argument("scenario", nargs="?", default="base")
parser.add_argument("--strict", action="store_true", help="disable the known-gap allowlist")
parser.add_argument("--trace", type=Path, help="check a saved trace instead of running the simulator")
parser.add_argument("--sim", type=Path, default=Path(__file__).resolve().parent / "sim")
parser.add_argument("--save-trace", type=Path, help="save all raw instrumentation records")
args = parser.parse_args()
sc = args.scenario
if args.trace:
    out = args.trace.read_text().splitlines()
else:
    result = subprocess.run([str(args.sim.resolve()), sc], capture_output=True, text=True, timeout=60)
    if result.returncode:
        print(f"SIMULATOR ERROR: exit {result.returncode}\n{result.stderr}", file=sys.stderr)
        sys.exit(2)
    if args.save_trace:
        args.save_trace.write_text(result.stdout)
    out = result.stdout.splitlines()
pir, reps, logs, xs, netUp, modem, pm = [], [], [], [], 0, [], None
charges, configs, snapshots = [], [], []
power_cuts = []
focused, solar, end, version, stats, deliv = None, None, None, None, None, None
for ln in out:
    k, rest = ln.split(",", 1)
    if k == "META":
        focused, solar, end, version = rest.split(",")
        focused, solar, end = bool(int(focused)), bool(int(solar)), int(end)
    elif k == "CFG": op, cl, deb, tz = map(int, rest.split(","))
    elif k == "P": pir.append(int(rest))
    elif k == "R":
        t, js = rest.split(",", 1); d = json.loads(js); reps.append((int(t), d["occupancy"], d["dailyoccupancy"]))
    elif k == "L": t, txt = rest.split(",", 1); logs.append((int(t), txt))
    elif k == "X": xs.append(tuple(map(int, rest.split(","))))
    elif k == "N": netUp = int(rest)
    elif k == "M": modem.append(int(rest))
    elif k == "PM": pm = list(map(int, rest.split(",")))
    elif k == "D": deliv = rest
    elif k == "CH":
        ms, adc, enabled, cause = rest.split(",")
        charges.append((int(ms), int(adc), bool(int(enabled)), cause))
    elif k == "PC": configs.append(tuple(map(int, rest.split(","))))
    elif k == "PS": snapshots.append(tuple(map(int, rest.split(","))))
    elif k == "ST": stats = tuple(map(int, rest.split(",")))
    elif k == "PD": power_cuts.append(tuple(map(int, rest.split(","))))
    else: raise ValueError(f"unknown trace record: {k}")
if focused is None or pm is None or stats is None or deliv is None or not charges or not snapshots:
    raise ValueError("missing mandatory trace records")
if not focused and not reps:
    raise ValueError("full-day scenario generated no reports")
TZ = dt.timezone(dt.timedelta(seconds=tz))
f = lambda t: dt.datetime.fromtimestamp(t, TZ).strftime("%m-%d %H:%M:%S")
loc = lambda t: dt.datetime.fromtimestamp(t, TZ)
D = deb * 60

# Expected occupancy intervals: sensor only counts during open hours, hold D after last PIR, clipped at close
iv = []
for p in pir:
    l = loc(p)
    if not (op <= l.hour < cl): continue
    close = int(l.replace(hour=cl, minute=0, second=0).timestamp())
    if iv and p < iv[-1][1] and iv[-1][2] == close: iv[-1][1] = min(p + D, close); iv[-1][3].append(p)
    else: iv.append([p, min(p + D, close), close, [p]])

fails = defaultdict(list)
T = 3                                                               # Tolerance (s) for loop / wake latency
def near(t, occ=None, lo=0, hi=T):
    return [r for r in reps if lo <= r[0] - t <= hi and (occ is None or r[1] == occ)]

if not focused:
    days = sorted({loc(r[0]).date() for r in reps} | {loc(p).date() for p in pir})
    for s, e, close, ps in iv:
        if not near(s, 1): fails[1].append(f"no 1 report at start {f(s)}")
        clipped = e == close
        if not clipped and not near(e, 0): fails[1].append(f"no 0 report at expected end {f(e)} (last PIR {f(ps[-1])})")
        bad0 = [r for r in reps if s < r[0] < e - T and r[1] == 0]
        if bad0: fails[2].append(f"0 reported while motion continued: {[f(r[0]) for r in bad0]}")
        inside = [s] + [r[0] for r in reps if s < r[0] <= e + T]
        gaps = [b - a for a, b in zip(inside, inside[1:])]
        if gaps and max(gaps) > 900 + T: fails[3].append(f"gap {max(gaps)} s in interval {f(s)}-{f(e)}")

    for i, r in enumerate(reps):
        if i == 0: continue
        p = reps[i - 1]; l = loc(r[0])
        if r[1] != p[1]: continue
        hourly = l.minute == 0 and l.second <= T
        update = r[1] == 1 and r[0] - p[0] >= 900 - 1
        if not (hourly or update): fails[4].append(f"repeat {r[1]} at {f(r[0])} ({r[0]-p[0]} s after previous) daily={r[2]}")

    for d in days:
        base = dt.datetime.combine(d, dt.time(0), TZ)
        for h in range(op, cl):
            t = int((base + dt.timedelta(hours=h)).timestamp())
            if t < reps[-1][0] and not near(t, None, 0, 5): fails[5].append(f"no report at {f(t)}")
        tc = int((base + dt.timedelta(hours=cl)).timestamp()); to = int((base + dt.timedelta(hours=op)).timestamp())
        exp = sum(round((e - s) / 60) for s, e, c, _ in iv if loc(s).date() == d)
        fin = near(tc, 0, 0, 5)
        if loc(reps[-1][0]) >= base + dt.timedelta(hours=cl):
            if not fin: fails[6].append(f"{d}: no final 0 report at close")
            elif abs(fin[0][2] - exp) > 2: fails[6].append(f"{d}: final daily {fin[0][2]} vs expected {exp}")
            nxt = int((base + dt.timedelta(days=1, hours=op)).timestamp())
        first = [r for r in reps if r[0] >= to]
        if first and first[0][0] < tc:
            cl_ = [t for t, x in logs if "Running daily cleanup" in x and loc(t).date() == d]
            boot = d == days[0]
            if not boot and len(cl_) != 1: fails[7].append(f"{d}: dailyCleanup ran {len(cl_)} times {[f(t) for t in cl_]}")
            elif cl_ and cl_[0] > first[0][0]: fails[7].append(f"{d}: cleanup after first report")
            if not (0 <= first[0][0] - to <= 5 and first[0][2] == 0): fails[8].append(f"{d}: first report {f(first[0][0])} daily={first[0][2]}")

    for r in reps:                                                       # Closed hours: only the final report at close is allowed
        l = loc(r[0])
        if not (op <= l.hour < cl) and not (l.hour == cl and l.minute == 0 and l.second <= 5):
            fails[6].append(f"report during closed hours {f(r[0])} occ={r[1]} daily={r[2]}")
lat = []
for q, dv in ([] if focused else xs):                                                     # Rule 9: delivered promptly (allowed wait for <=50% three-hour schedule)
    if q < netUp: continue                                           # Queued during the network outage - can only go once it is back
    if sc.startswith("badnet") and q < netUp + 41 * 60: continue  # Recovery window: an 11 min attempt + 30 min backoff may straddle the outage end
    lat.append(dv - q)
    if sc == "verylow": lim = 3 * 3600                                # <=50%: three-hour schedule
    elif sc in ("lowbatt", "slowconn", "hot", "hot115", "notchg", "badnetlow", "solar", "solarhot", "solarnotchg"): lim = 3600 - q % 3600 + 120   # 50-65%: by the next top-of-hour connection (+connect time)
    else: lim = 300
    if loc(q).minute == 0 and loc(q).second <= T and dv - q > 120 and sc != "verylow": fails[9].append(f"hourly report {f(q)} delivered {(dv-q)} s late")
    elif dv - q > lim: fails[9].append(f"queued {f(q)} delivered {f(dv)} ({(dv-q)//60} min late)")
# Rule 10: Particle aggressive-reconnection guidance - modem power-ups >= 10 min apart, never > 6 in any hour
for a, b in zip(modem, modem[1:]):
    if b - a < 600: fails[10].append(f"modem on at {f(a)} and again at {f(b)} ({b-a} s apart)")
for i, a in enumerate(modem):
    n = sum(1 for b in modem[i:] if b - a < 3600)
    if n > 6: fails[10].append(f"{n} modem power-ups in the hour from {f(a)}")
# Independent charge safety oracle: raw ADC -> unrounded Celsius.
# Disabled charging in the safe band is valid (hysteresis, faults, or unavailable input).
minV, chg, unclean, tempF = pm
expected = (5080, 4208, 1024) if solar else (4208, 4112, 900)
for ms, adc, enabled, cause in charges:
    true_c = (adc * 3.3 / 4096.0 - 0.5) * 100
    if enabled and not 0 <= true_c <= 45:
        fails[11].append(f"charging enabled by {cause} at {ms} ms: ADC {adc} = {true_c:.4f} C")
for ms, mv, cv, cc in snapshots:
    if (mv, cv, cc) != expected:
        fails[11].append(f"stable config at {ms} ms is {(mv, cv, cc)}, expected {expected}")
if solar and not any(c[1:] == expected for c in configs):
    fails[11].append("solar settings never applied")
if unclean: fails[11].append(f"{unclean} deepPowerDown(s) with the modem still on")
undelivered = len(reps) - len(xs)
if undelivered and not focused: fails[9].append(f"{undelivered} reports never delivered")
if sc == "stuckqueue" and stats[0] > 121000:
    fails[12].append(f"connected with blocked queue for {stats[0]} ms (budget 120000 ms + 1000 ms loop tolerance)")
if sc.startswith("boundary") and not modem:
    fails[10].append("boundary fixture never powered the modem on")
if sc == "staledrain":
    closing = [(t, occ, daily) for t, occ, daily in reps if loc(t).hour == cl]
    if not closing or not any(q == closing[0][0] and dv - q <= 120 for q, dv in xs):
        fails[12].append("new closing report not delivered within 120 s despite queue recovery after 60 s; earlier drain must not shorten this wait")
if sc in ("boundary", "boundarylow", "boundaryclosing"):
    target_hour = 9 if sc == "boundarylow" else 21 if sc == "boundaryclosing" else 7
    hourly = [r for r in reps if loc(r[0]).hour == target_hour and loc(r[0]).minute == 0]
    if not hourly or not any(q == hourly[0][0] and dv - q <= 120 for q, dv in xs):
        fails[10].append("boundary hourly/final report was not delivered within 120 s")
if sc.startswith("shutdown") and not stats[3]:
    fails[13].append("shutdown fixture never requested modem off")
if sc == "shutdownslow":
    if stats[1] or not power_cuts:
        fails[13].append("slow successful shutdown timed out or never completed")
    elif power_cuts[0][0] - snapshots[0][0] < 25000:
        fails[13].append("power cut before 5 s cloud disconnect + 20 s modem shutdown elapsed")
if sc == "shutdownpowerfail" and (not stats[4] or power_cuts or not any("Captured reset System.reset" in text for _, text in logs)):
    fails[13].append("failed deepPowerDown did not fall back to System.reset without a power cut")
if sc == "thermalhysteresis":
    # Select the direct policy decision per measurement; config-reload records
    # are additional actions and remain subject to the raw-ADC oracle above.
    decisions = [c for c in charges if c[3] in ("enableCharging", "disableCharging")][-5:]
    measured = [(adc * 3.3 / 4096.0 - 0.5) * 100 for _, adc, _, _ in decisions]
    enabled = [value for _, _, value, _ in decisions]
    if len(measured) != 5 or enabled != [True, False, False, False, True]:
        fails[11].append(f"hysteresis sequence at {measured} C gave {enabled}; expected on/off/off/off/on")
print(f"== scenario {sc}: open {op} close {cl} debounce {deb}  reports {len(reps)}  delivered,pending {deliv}")
for r in reps: print(f"   {f(r[0])}  occ={r[1]} daily={r[2]}")
for t, x in logs: print(f"   {f(t)}  LOG {x}")
names = {1: "5-min hold after 0->1", 2: "motion restarts hold", 3: "occupied update <=15 min", 4: "only changes (+hourly/15-min)",
         5: "hourly on the hour", 6: "final report at close, no accrual after", 7: "cleanup once, before first report", 8: "open report daily=0", 9: "delivered on policy schedule", 10: "modem cycles >=10 min apart, <=6/h", 11: "PMIC / ADC charging / clean power-down", 12: "blocked queue awake <=120 s", 13: "shutdown timing / failure fixture"}
print(f"   delivery latency s: max {max(lat) if lat else 0}  median {sorted(lat)[len(lat)//2] if lat else 0}")
print(f"   modem power-ups {len(modem)}  PMIC minV {minV}  charging {chg}  temp {tempF}F")
if sc in ("stuckqueue", "staledrain"): print(f"   longest connected blocked-queue episode: {stats[0]} ms")
if sc.startswith("shutdown"): print(f"   wait timeouts {stats[1]}, off requests {stats[3]}, power cuts (ms,on): {power_cuts}")
for k in range(1, 14):
    status = "SKIP" if focused and k <= 9 else "FAIL" if fails[k] else "PASS"
    print(f"Rule {k} {names[k]:40s} {status}")
    for m in fails[k]: print(f"        - {m}")

# Each exemption names an exact known failure message. New failures in the same rule
# or scenario still fail. Fixes may remove exemptions without causing a test failure.
allowlist = json.loads(Path(__file__).with_name("known_gaps.json").read_text())
allowed = {} if args.strict else allowlist.get(sc, {})
unexpected = []
for rule, messages in fails.items():
    known = allowed.get(str(rule), {}).get("messages", [])
    for message in messages:
        if message in known:
            print(f"KNOWN GAP Rule {rule}: {message}")
        else:
            unexpected.append((rule, message))
print(f"RESULT: {len(unexpected)} unexpected failure(s), firmware {version}")
sys.exit(1 if unexpected else 0)
