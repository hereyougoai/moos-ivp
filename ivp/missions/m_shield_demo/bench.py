#!/usr/bin/env python3
"""
bench.py -- measure what the fleet actually does to the intruder.

The outcome the mission cares about (did the intruder leave?) depends
partly on the intruder's own patrol script, so it is a noisy way to
judge a change to the fleet's geometry. This tool measures the fleet
instead, by reconstructing offline the objective function the intruder's
own helm is solving.

The intruder runs BHV_AvoidCollision. Its objective function is
metric(projected CPA): 0 below min_util_cpa_dist, 100 above
max_util_cpa_dist, linear between. So at every logged instant we can ask
the question its helm is asking -- over every course and speed available
to it, how good is the best option, and how good is carrying on? -- and
the difference between those two IS the pressure the fleet is exerting.

  PUSH = best escape utility - utility of holding course

If PUSH is zero the fleet is exerting nothing, whatever it looks like on
the chart and whatever weight the avoidance behavior carries: a flat
objective function does not move an IvP solution. That was true of the
trail arc at every station range from 30 m down to 14 m, which is why
the pair looked like an escort.

Usage:  bench.py <run_dir> [<run_dir> ...]
        where each run_dir holds the LOG_*/XLOG_* directories of one run.
"""
import sys, os, glob, math

# --- the intruder's avoidance parameters, from meta_target.bhv ---------
MIN_UTIL_CPA = 12.0
MAX_UTIL_CPA = 25.0
TGT_SPEEDS   = [0.4, 0.6, 0.8, 1.0, 1.2]   # its speed domain
TOL          = 60.0                        # CPA time horizon, seconds

# --- the region the operator drew (must match the launch scenario) -----
REGION = (-70.0, 210.0, -130.0, -10.0)     # x0, x1, y0, y1

PUSH_FLOOR = 5.0     # below this, call it no push at all
SAFE_FLOOR = 14.0    # avdtgt_ min_util_cpa_dist: our own "safe pass" floor


def metric(d):
    if d < MIN_UTIL_CPA:
        return 0.0
    if d > MAX_UTIL_CPA:
        return 100.0
    return 25.0 + 75.0 * (d - MIN_UTIL_CPA) / (MAX_UTIL_CPA - MIN_UTIL_CPA)


def cpa(px, py, vx, vy, qx, qy, wx, wy):
    """Closest point of approach between two vessels holding velocity."""
    rx, ry = qx - px, qy - py
    dx, dy = wx - vx, wy - vy
    vv = dx * dx + dy * dy
    if vv < 1e-9:
        return math.hypot(rx, ry)
    t = -(rx * dx + ry * dy) / vv
    t = max(0.0, min(TOL, t))
    return math.hypot(rx + dx * t, ry + dy * t)


def vel(spd, hdg):
    r = math.radians(hdg)
    return spd * math.sin(r), spd * math.cos(r)


def read_nav(logdir):
    """{time -> (x, y, hdg, spd)} at 1 Hz."""
    files = glob.glob(os.path.join(logdir, "*.alog"))
    if not files:
        return {}
    got = {}
    for line in open(files[0], errors="ignore"):
        p = line.split()
        if len(p) < 4 or p[1] not in ("NAV_X", "NAV_Y", "NAV_HEADING", "NAV_SPEED"):
            continue
        t = round(float(p[0]))
        try:
            v = float(p[3])
        except ValueError:
            continue
        got.setdefault(t, {})[p[1]] = v
    out = {}
    for t, d in got.items():
        if len(d) == 4:
            out[t] = (d["NAV_X"], d["NAV_Y"], d["NAV_HEADING"], d["NAV_SPEED"])
    return out


def read_var(logdir, name):
    files = glob.glob(os.path.join(logdir, "*.alog"))
    if not files:
        return []
    out = []
    for line in open(files[0], errors="ignore"):
        p = line.split()
        if len(p) >= 4 and p[1] == name:
            out.append((float(p[0]), p[3]))
    return out


def offset(x, y):
    """Signed distance from the region edge: + outside, - inside."""
    x0, x1, y0, y1 = REGION
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    if dx > 0 or dy > 0:
        return math.hypot(dx, dy)
    return -min(x - x0, x1 - x, y - y0, y1 - y)


def push(tgt, usvs):
    """The pressure the intruder feels, and the CPA it is being held to."""
    tx, ty, th, ts = tgt
    contacts = []
    for (ux, uy, uh, us) in usvs:
        wx, wy = vel(us, uh)
        contacts.append((ux, uy, wx, wy))
    if not contacts:
        return 0.0, 999.0

    def util(course, speed):
        vx, vy = vel(speed, course)
        return min(metric(cpa(tx, ty, vx, vy, qx, qy, wx, wy))
                   for (qx, qy, wx, wy) in contacts)

    hold = util(th, ts)
    best = max(util(c * 10.0, s) for c in range(36) for s in TGT_SPEEDS)
    held_cpa = min(cpa(tx, ty, *vel(ts, th), qx, qy, wx, wy)
                   for (qx, qy, wx, wy) in contacts)
    return best - hold, held_cpa


def analyse(run):
    tgt = read_nav(_find(run, "LOG_TARGET_*"))
    abe = read_nav(_find(run, "LOG_ABE_*"))
    ben = read_nav(_find(run, "LOG_BEN_*"))
    msh = _find(run, "XLOG_MOTHERSHIP_*")
    states = read_var(msh, "SHIELD_STATE")
    evict = read_var(msh, "SHIELD_EVICTIONS")

    def engaged_at(t):
        cur = "searching"
        for tt, s in states:
            if tt <= t:
                cur = s
        return cur == "intercepting"

    times = sorted(set(tgt) & set(abe) & set(ben))
    if not times:
        return None
    t0 = times[0]

    eng_push, eng_cpa, eng_rng, out_eng = [], [], [], 0
    all_rng, eng_n = [], 0
    for t in times:
        usvs = [abe[t], ben[t]]
        rng = min(math.hypot(u[0] - tgt[t][0], u[1] - tgt[t][1]) for u in usvs)
        all_rng.append((rng, t))
        if not engaged_at(t):
            continue
        eng_n += 1
        p, c = push(tgt[t], usvs)
        eng_push.append(p)
        eng_cpa.append(c)
        eng_rng.append((rng, t))
        if offset(tgt[t][0], tgt[t][1]) > 0:
            out_eng += 1

    n_ev = int(float(evict[-1][1])) if evict else 0
    first = None
    for tt, s in states:
        if s == "intercepting":
            first = tt - t0
            break

    return {
        "dur": times[-1] - t0,
        "engaged_s": eng_n,
        "first_engage": first,
        "evictions": n_ev,
        "push_mean": sum(eng_push) / len(eng_push) if eng_push else 0.0,
        "push_pct": 100.0 * sum(1 for p in eng_push if p > PUSH_FLOOR) / len(eng_push) if eng_push else 0.0,
        "cpa_med": sorted(eng_cpa)[len(eng_cpa) // 2] if eng_cpa else 0.0,
        "outside_pct": 100.0 * out_eng / eng_n if eng_n else 0.0,
        "min_rng_eng": min(eng_rng)[0] if eng_rng else 999.0,
        "unsafe_s": len(set(int(t) for r, t in eng_rng if r < SAFE_FLOOR)),
        "min_rng_all": min(all_rng)[0] if all_rng else 999.0,
    }


def _find(run, pat):
    hits = sorted(glob.glob(os.path.join(run, pat)))
    return hits[-1] if hits else run


def main():
    runs = sys.argv[1:]
    if not runs:
        print(__doc__)
        return
    rows = []
    for r in runs:
        a = analyse(r)
        if a:
            rows.append((os.path.basename(r.rstrip("/")), a))
    if not rows:
        print("no usable runs")
        return

    print("%-14s %6s %6s %7s %7s %7s %7s %6s %6s" % (
        "run", "engS", "1st", "PUSH", "push%", "CPA", "out%", "evic", "minR"))
    print("-" * 80)
    for name, a in rows:
        print("%-14s %6d %6s %7.1f %7.0f %7.1f %7.0f %6d %6.1f" % (
            name, a["engaged_s"],
            ("%.0f" % a["first_engage"]) if a["first_engage"] is not None else "-",
            a["push_mean"], a["push_pct"], a["cpa_med"],
            a["outside_pct"], a["evictions"], a["min_rng_eng"]))

    def col(k):
        return [a[k] for _, a in rows]
    n = len(rows)
    print("-" * 80)
    print("%-14s %6.0f %6s %7.1f %7.0f %7.1f %7.0f %6.1f %6.1f   (mean of %d)" % (
        "MEAN", sum(col("engaged_s")) / n, "",
        sum(col("push_mean")) / n, sum(col("push_pct")) / n,
        sum(col("cpa_med")) / n, sum(col("outside_pct")) / n,
        sum(col("evictions")) / n, min(col("min_rng_eng")), n))
    print()
    print("PUSH   mean utility the intruder gains by escaping vs holding course.")
    print("       0 = the fleet is exerting nothing at all. This is the number")
    print("       that matters; everything else is downstream of it.")
    print("push%%  share of engaged time with PUSH above %.0f." % PUSH_FLOOR)
    print("CPA    median closest approach the fleet is projecting onto it.")
    print("       Must be under %.0f m to register at all." % MAX_UTIL_CPA)
    print("out%    share of engaged time the intruder spent outside the region.")
    print("minR   closest the two ever got while engaged (safety floor %.0f m;" % SAFE_FLOOR)
    unsafe = sum(a["unsafe_s"] for _, a in rows)
    print("       %d engaged second(s) under it across all runs)." % unsafe)


if __name__ == "__main__":
    main()
