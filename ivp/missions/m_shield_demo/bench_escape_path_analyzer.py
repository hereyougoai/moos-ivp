#!/usr/bin/env python3
"""
bench_speed_stats.py -- Comprehensive statistical analysis for target speed benchmark runs.
Analyzes eviction success rates, time to first eviction, engagement time, and push metrics.
"""
import sys, os, glob, math, json
from collections import defaultdict

MIN_UTIL_CPA = 12.0
MAX_UTIL_CPA = 25.0
TGT_SPEEDS   = [0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
TOL          = 60.0
REGION       = (-70.0, 210.0, -130.0, -10.0) # x0, x1, y0, y1
PUSH_FLOOR   = 5.0
SAFE_FLOOR   = 14.0

def metric(d):
    if d < MIN_UTIL_CPA:
        return 0.0
    if d > MAX_UTIL_CPA:
        return 100.0
    return 25.0 + 75.0 * (d - MIN_UTIL_CPA) / (MAX_UTIL_CPA - MIN_UTIL_CPA)

def cpa(px, py, vx, vy, qx, qy, wx, wy):
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
    x0, x1, y0, y1 = REGION
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    if dx > 0 or dy > 0:
        return math.hypot(dx, dy)
    return -min(x - x0, x1 - x, y - y0, y1 - y)

# Precomputed course & speed vectors for fast push utility calculation
C_RADS = [math.radians(c * 10.0) for c in range(36)]
TGT_VELS = [(s * math.sin(r), s * math.cos(r)) for s in TGT_SPEEDS for r in C_RADS]

def push(tgt, usvs):
    tx, ty, th, ts = tgt
    contacts = []
    for (ux, uy, uh, us) in usvs:
        wx, wy = vel(us, uh)
        contacts.append((ux, uy, wx, wy))
    if not contacts:
        return 0.0, 999.0

    def util(vx, vy):
        return min(metric(cpa(tx, ty, vx, vy, qx, qy, wx, wy))
                   for (qx, qy, wx, wy) in contacts)

    vx_hold, vy_hold = vel(ts, th)
    hold = util(vx_hold, vy_hold)
    best = max(util(vx, vy) for (vx, vy) in TGT_VELS)
    held_cpa = min(cpa(tx, ty, vx_hold, vy_hold, qx, qy, wx, wy)
                   for (qx, qy, wx, wy) in contacts)
    return best - hold, held_cpa

def _find(run, pat):
    hits = sorted(glob.glob(os.path.join(run, pat)))
    return hits[-1] if hits else run

def analyse_run(run_dir):
    tgt = read_nav(_find(run_dir, "LOG_TARGET_*"))
    abe = read_nav(_find(run_dir, "LOG_ABE_*"))
    ben = read_nav(_find(run_dir, "LOG_BEN_*"))
    msh = _find(run_dir, "XLOG_MOTHERSHIP_*")
    states = read_var(msh, "SHIELD_STATE")
    evict = read_var(msh, "SHIELD_EVICTIONS")

    times = sorted(set(tgt) & set(abe) & set(ben))
    if not times:
        return None
    t0 = times[0]

    def engaged_at(t):
        cur = "searching"
        for tt, s in states:
            if tt <= t:
                cur = s
        return cur == "intercepting"

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

    first_engage = None
    for tt, s in states:
        if s == "intercepting":
            first_engage = tt - t0
            break

    first_evict_time = None
    first_evict_duration = None
    total_evictions = 0
    post_detect_dist = None
    direct_exit_dist = None
    tortuosity = None

    if evict:
        try:
            total_evictions = int(float(evict[-1][1]))
            for tt, count_str in evict:
                if float(count_str) >= 1.0:
                    first_evict_time = tt - t0
                    if first_engage is not None:
                        first_evict_duration = max(0.0, first_evict_time - first_engage)
                        # Compute actual distance traveled by target between engagement and eviction
                        t_start = round(first_engage + t0)
                        t_end = round(first_evict_time + t0)
                        sub_times = sorted([t for t in tgt.keys() if t_start <= t <= t_end])
                        d_accum = 0.0
                        for i in range(len(sub_times) - 1):
                            p1 = tgt[sub_times[i]]
                            p2 = tgt[sub_times[i+1]]
                            d_accum += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                        post_detect_dist = d_accum
                        if t_start in tgt and sub_times:
                            start_pos = (tgt[sub_times[0]][0], tgt[sub_times[0]][1])
                            exit_pos = (tgt[sub_times[-1]][0], tgt[sub_times[-1]][1])
                            start_hdg = tgt[sub_times[0]][2]
                            exit_hdg = tgt[sub_times[-1]][2]
                            
                            # Shortest distance from detection position to region boundary
                            x0, x1, y0, y1 = REGION
                            px, py = start_pos
                            direct_exit_dist = min(abs(px - x0), abs(px - x1), abs(py - y0), abs(py - y1))
                            if direct_exit_dist > 0:
                                tortuosity = post_detect_dist / direct_exit_dist
                    break
        except (ValueError, IndexError):
            pass

    success = (total_evictions >= 1)

    start_pos = start_pos if 'start_pos' in locals() else (0.0, 0.0)
    exit_pos = exit_pos if 'exit_pos' in locals() else (0.0, 0.0)
    start_hdg = start_hdg if 'start_hdg' in locals() else 0.0
    exit_hdg = exit_hdg if 'exit_hdg' in locals() else 0.0

    return {
        "run_dir": run_dir,
        "duration": times[-1] - t0,
        "success": success,
        "evictions": total_evictions,
        "first_engage": first_engage,
        "first_evict_time": first_evict_time,
        "first_evict_duration": first_evict_duration,
        "post_detect_dist": post_detect_dist,
        "direct_exit_dist": direct_exit_dist,
        "tortuosity": tortuosity,
        "start_pos": start_pos,
        "exit_pos": exit_pos,
        "start_hdg": start_hdg,
        "exit_hdg": exit_hdg,
        "engaged_s": eng_n,
        "push_mean": sum(eng_push) / len(eng_push) if eng_push else 0.0,
        "push_pct": 100.0 * sum(1 for p in eng_push if p > PUSH_FLOOR) / len(eng_push) if eng_push else 0.0,
        "cpa_med": sorted(eng_cpa)[len(eng_cpa) // 2] if eng_cpa else 0.0,
        "outside_pct": 100.0 * out_eng / eng_n if eng_n else 0.0,
        "min_rng_eng": min(eng_rng)[0] if eng_rng else 999.0,
        "unsafe_s": len(set(int(t) for r, t in eng_rng if r < SAFE_FLOOR)),
        "min_rng_all": min(all_rng)[0] if all_rng else 999.0,
    }

def main():
    if len(sys.argv) < 2:
        print("Usage: bench_escape_path_analyzer.py <profile_dirs...>")
        print("Example: bench_escape_path_analyzer.py bench_results/prof_*")
        return

    profile_groups = defaultdict(list)
    for arg in sys.argv[1:]:
        runs = sorted(glob.glob(os.path.join(arg, "run*")))
        if runs:
            grp_name = os.path.basename(arg.rstrip("/\\"))
            for r in runs:
                res = analyse_run(r)
                if res:
                    profile_groups[grp_name].append(res)
        else:
            res = analyse_run(arg)
            if res:
                parent = os.path.basename(os.path.dirname(arg.rstrip("/\\")))
                profile_groups[parent].append(res)

    if not profile_groups:
        print("No valid run directories found.")
        return

    print("=" * 115)
    print("                 M_SHIELD_DEMO 不同目標船情境 (Target Profile) 逃逸路徑與驅逐評估報告")
    print("=" * 115)

    summary_table = []

    for grp in sorted(profile_groups.keys()):
        runs = profile_groups[grp]
        n = len(runs)
        if n == 0:
            continue

        n_success = sum(1 for r in runs if r["success"])
        succ_rate = 100.0 * n_success / n

        evic_counts = [r["evictions"] for r in runs]
        mean_evic = sum(evic_counts) / n

        engage_times = [r["first_engage"] for r in runs if r["first_engage"] is not None]
        mean_1st_eng = sum(engage_times) / len(engage_times) if engage_times else None

        evict_durations = [r["first_evict_duration"] for r in runs if r["first_evict_duration"] is not None]
        mean_evict_dur = sum(evict_durations) / len(evict_durations) if evict_durations else None

        post_dists = [r["post_detect_dist"] for r in runs if r["post_detect_dist"] is not None]
        mean_post_dist = sum(post_dists) / len(post_dists) if post_dists else None

        tortuosities = [r["tortuosity"] for r in runs if r["tortuosity"] is not None]
        mean_tortuosity = sum(tortuosities) / len(tortuosities) if tortuosities else None

        mean_push = sum(r["push_mean"] for r in runs) / n
        mean_min_rng = min(r["min_rng_eng"] for r in runs)

        summary_table.append({
            "group": grp,
            "trials": n,
            "succ_rate": succ_rate,
            "mean_evic": mean_evic,
            "mean_evict_dur": mean_evict_dur,
            "mean_post_dist": mean_post_dist,
            "mean_tortuosity": mean_tortuosity,
            "mean_push": mean_push,
            "mean_min_rng": mean_min_rng,
        })

        print(f"\n[情境組別: {grp}] (測試次數: {n} 輪)")
        print(f"{'Run':<6} {'成功?':<8} {'驅離耗時':<10} {'逃逸航程':<12} {'迂迴比':<10} {'PUSH':<8} {'接戰起點(X,Y)':<20} {'邊界脫離點(X,Y)':<20}")
        print("-" * 115)
        for r in runs:
            run_name = os.path.basename(r["run_dir"])
            succ_str = "SUCCESS" if r["success"] else "FAIL"
            t_dur = f"{r['first_evict_duration']:.1f}s" if r["first_evict_duration"] is not None else "-"
            d_post = f"{r['post_detect_dist']:.1f}m" if r["post_detect_dist"] is not None else "-"
            tort = f"{r['tortuosity']:.2f}" if r["tortuosity"] is not None else "-"
            sp = f"({r['start_pos'][0]:.1f}, {r['start_pos'][1]:.1f})" if r["success"] else "-"
            ep = f"({r['exit_pos'][0]:.1f}, {r['exit_pos'][1]:.1f})" if r["success"] else "-"
            print(f"{run_name:<6} {succ_str:<8} {t_dur:<10} {d_post:<12} {tort:<10} {r['push_mean']:<8.1f} {sp:<20} {ep:<20}")

    print("\n" + "=" * 115)
    print("                                      情境橫向綜合比較匯總表")
    print("=" * 115)
    print(f"{'目標情境組別':<18} {'成功率':<9} {'平均驅逐次數':<13} {'包夾驅離耗時':<13} {'目標逃逸航行距離':<18} {'路徑迂迴比':<15} {'平均壓迫力(PUSH)':<15}")
    print("-" * 115)
    for s in summary_table:
        t_dur = f"{s['mean_evict_dur']:.1f}s" if s['mean_evict_dur'] is not None else "N/A"
        d_post = f"{s['mean_post_dist']:.1f} m" if s['mean_post_dist'] is not None else "N/A"
        tort = f"{s['mean_tortuosity']:.2f}" if s['mean_tortuosity'] is not None else "N/A"
        print(f"{s['group']:<18} {s['succ_rate']:>5.1f}%    {s['mean_evic']:<13.2f} {t_dur:<13} {d_post:<18} {tort:<15} {s['mean_push']:<15.1f}")

    print("=" * 115)

if __name__ == "__main__":
    main()

