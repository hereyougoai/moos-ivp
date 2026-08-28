#!/usr/bin/env python3
"""
bench_land_report.py -- one line per vehicle summarising a single mission run.

Reads the alogs a run left behind and reports the things that only the
shoreline work can break:

  ashore    fraction of position samples inside the charted land polygons.
            The number that matters. Anything but zero means a hull was on
            the beach, whatever else the run looked like.
  minclr    closest the hull ever came to the waterline. Distinguishes "kept
            well clear" from "got away with it".
  halt      helm all-stops by reason. BehaviorError is the deadlock case:
            the vehicle stops and cannot clear the condition that stopped it.
  breach    pLandGuard recoveries -- how often the escape behavior had to
            drive a vehicle back off the shore.
  stall     uSimMarine steps clamped by the stall guard, and the sim time
            those dropped. A run with a large figure here covered less
            ground than its duration implies.
"""

import glob
import math
import os
import re
import subprocess
import sys


def load_rings(land_txt):
    """True (non-convex) shoreline outlines -- the display rows of land.txt."""
    txt = open(land_txt).read()
    return [[tuple(float(v) for v in p.split(','))
             for p in m.group(1).split(':')]
            for m in re.finditer(r'display = pts=\{([^}]*)\}', txt)]


def inside(rings, x, y):
    for r in rings:
        n = len(r)
        ins = False
        for i in range(n):
            j = (i - 1) % n
            if ((r[i][1] > y) != (r[j][1] > y)) and \
               (x < (r[j][0] - r[i][0]) * (y - r[i][1]) /
                    (r[j][1] - r[i][1]) + r[i][0]):
                ins = not ins
        if ins:
            return True
    return False


def dist_to_shore(rings, x, y):
    if inside(rings, x, y):
        return 0.0
    best = 1e9
    for r in rings:
        n = len(r)
        for i in range(n):
            ax, ay = r[i - 1]
            bx, by = r[i]
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy
            t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - ax) * dx +
                                                       (y - ay) * dy) / l2))
            best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def grep(alog, varnames):
    """Values of the named variables, in time order, via aloggrep."""
    try:
        out = subprocess.run(['aloggrep', alog] + varnames,
                             capture_output=True, text=True, timeout=300).stdout
    except Exception:
        return []
    rows = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 4 or not parts[0][0].isdigit():
            continue
        rows.append((float(parts[0]), parts[1], ' '.join(parts[3:])))
    return rows


def report_vehicle(rings, alog, name):
    rows = grep(alog, ['NAV_X', 'NAV_Y', 'IVPHELM_ALLSTOP', 'BHV_ERROR',
                       'LAND_BREACH', 'USM_STALL_COUNT', 'USM_STALL_SKIPPED',
                       'LAND_CLEARANCE'])
    xs, ys = {}, {}
    halts, errors, breaches = {}, set(), 0
    stall_n, stall_s, min_clr = 0, 0.0, None

    for t, var, val in rows:
        if var == 'NAV_X':
            xs[round(t, 1)] = float(val)
        elif var == 'NAV_Y':
            ys[round(t, 1)] = float(val)
        elif var == 'IVPHELM_ALLSTOP':
            if val != 'clear':
                halts[val] = halts.get(val, 0) + 1
        elif var == 'BHV_ERROR':
            errors.add(val)
        elif var == 'LAND_BREACH':
            if val.lower() == 'true':
                breaches += 1
        elif var == 'USM_STALL_COUNT':
            stall_n = max(stall_n, int(float(val)))
        elif var == 'USM_STALL_SKIPPED':
            stall_s = max(stall_s, float(val))
        elif var == 'LAND_CLEARANCE':
            c = float(val)
            if (min_clr is None) or (c < min_clr):
                min_clr = c

    pts = [(xs[t], ys[t]) for t in sorted(xs) if t in ys]
    ashore = sum(1 for x, y in pts if inside(rings, x, y))

    # LAND_CLEARANCE comes from pLandGuard and is the same measurement, but
    # falling back to the track keeps the report meaningful for a vehicle
    # whose guard never started.
    if (min_clr is None) and pts:
        min_clr = min(dist_to_shore(rings, x, y) for x, y in pts)

    halt_str = ','.join('%s=%d' % kv for kv in sorted(halts.items())) or '-'
    return ("  %-7s ashore %4d/%-5d  minclr %6s m  halt %-22s "
            "breach %d  stall %d/%.0fs%s"
            % (name, ashore, len(pts),
               ('%.1f' % min_clr) if min_clr is not None else '?',
               halt_str, breaches, stall_n, stall_s,
               ('  ERR:' + '; '.join(sorted(errors))) if errors else ''))


def main():
    rundir = sys.argv[1]
    runno = sys.argv[2] if len(sys.argv) > 2 else '?'
    here = os.path.dirname(os.path.abspath(__file__))
    rings = load_rings(os.path.join(here, 'land.txt'))

    lines = ['run %s:' % runno]
    for name in ('ABE', 'BEN', 'TARGET'):
        dirs = sorted(glob.glob(os.path.join(rundir, 'LOG_%s_*' % name)))
        if not dirs:
            lines.append('  %-7s no log' % name)
            continue
        alogs = glob.glob(os.path.join(dirs[-1], '*.alog'))
        if not alogs:
            lines.append('  %-7s no alog' % name)
            continue
        lines.append(report_vehicle(rings, alogs[0], name))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
