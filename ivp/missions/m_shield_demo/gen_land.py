#!/usr/bin/env python3
"""
gen_land.py -- extract the Forrest Lake shoreline from the viewer backdrop
               and emit the land geometry used by m_shield_demo.

The mission's pMarineViewer backdrop (ivp/data/forrest19.tif) is a 2048x2048
uncompressed RGB aerial photo, and forrest19.info carries its geo bounds. That
is enough to recover the real shoreline in local mission coordinates, so the
"land" the vehicles must not sail onto is the actual lake shore rather than a
hand-drawn rectangle.

Two products come out of one source of truth, which is the point of doing this
in a script rather than by hand:

  display layer   one non-convex polygon per land mass, plus its shoreline as
                  a seglist. Only ever drawn. pMarineViewer parses VIEW_POLYGON
                  through stringStandard2Poly(), which adds vertices with the
                  convexity check disabled, so a non-convex polygon renders
                  fine even though no behavior could use it.

  collision layer a strip of convex parallelogram tiles laid along the
                  shoreline, each extending inland. BHV_AvoidObstacleV24 and
                  pObstacleMgr reject non-convex obstacles outright, so the
                  curved shore has to be decomposed. Tiles are built from a
                  single segment normal, which makes each one a parallelogram
                  and therefore convex by construction.

Both layers derive from the same buffered mask, so what the operator sees and
what the helm avoids cannot drift apart.

Usage:  ./gen_land.py            (writes land.txt and the two .moos plugs)
        ./gen_land.py --preview  (also writes a PNG mask for eyeballing)
"""

import argparse
import math
import os
import struct
import sys
import zlib

# --------------------------------------------------------------------------
# Backdrop / geodesy

MOOSIVP_DATA = os.environ.get("MOOSIVP_DATA", os.path.expanduser("~/moos-ivp/ivp/data"))
TIF = os.path.join(MOOSIVP_DATA, "forrest19.tif")
INFO = os.path.join(MOOSIVP_DATA, "forrest19.info")


def read_info(path):
    """Pull the lat/lon bounds and datum out of a pMarineViewer .info file."""
    vals = {}
    for line in open(path):
        line = line.split("//")[0].strip()
        if "=" not in line:
            continue
        k, v = line.split("=", 1)
        try:
            vals[k.strip()] = float(v.strip())
        except ValueError:
            pass
    need = ["lat_north", "lat_south", "lon_east", "lon_west",
            "datum_lat", "datum_lon"]
    missing = [k for k in need if k not in vals]
    if missing:
        sys.exit("%s: missing %s" % (path, ", ".join(missing)))
    return vals


def geo_bounds(info):
    """Local-grid extent of the backdrop, in meters from the mission datum.

    Flat-earth conversion at the datum latitude. Over a 440 m square the
    departure from MOOSGeodesy's local grid is well under a meter, which is
    far below the resolution the shoreline is extracted at.
    """
    phi = math.radians(info["datum_lat"])
    m_lat = (111132.92 - 559.82 * math.cos(2 * phi)
             + 1.175 * math.cos(4 * phi) - 0.0023 * math.cos(6 * phi))
    m_lon = (111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
             + 0.118 * math.cos(5 * phi))
    x_w = (info["lon_west"] - info["datum_lon"]) * m_lon
    x_e = (info["lon_east"] - info["datum_lon"]) * m_lon
    y_s = (info["lat_south"] - info["datum_lat"]) * m_lat
    y_n = (info["lat_north"] - info["datum_lat"]) * m_lat
    return x_w, x_e, y_s, y_n


# --------------------------------------------------------------------------
# TIFF reader (uncompressed, single-strip, 8-bit RGB -- which is what
# forrest19.tif is; no image library is available in this environment)

def read_tiff_rgb(path):
    d = open(path, "rb").read()
    if d[:2] not in (b"MM", b"II"):
        sys.exit("%s: not a TIFF" % path)
    bo = ">" if d[:2] == b"MM" else "<"
    ifd = struct.unpack(bo + "I", d[4:8])[0]
    n = struct.unpack(bo + "H", d[ifd:ifd + 2])[0]
    tags = {}
    for i in range(n):
        e = ifd + 2 + i * 12
        tag, typ, cnt = struct.unpack(bo + "HHI", d[e:e + 8])
        if typ == 3 and cnt == 1:
            tags[tag] = struct.unpack(bo + "H", d[e + 8:e + 10])[0]
        elif typ == 4 and cnt == 1:
            tags[tag] = struct.unpack(bo + "I", d[e + 8:e + 12])[0]
        else:
            tags[tag] = struct.unpack(bo + "I", d[e + 8:e + 12])[0]

    w, h = tags.get(256), tags.get(257)
    spp = tags.get(277, 1)
    comp = tags.get(259, 1)
    off = tags.get(273)
    if comp != 1:
        sys.exit("%s: compressed TIFF not supported (compression=%d)" % (path, comp))
    if spp != 3:
        sys.exit("%s: expected 3 samples/pixel, got %d" % (path, spp))
    if tags.get(278, h) < h:
        sys.exit("%s: multi-strip TIFF not supported" % path)
    return d, off, w, h


# --------------------------------------------------------------------------
# Land / water classification

def classify(d, off, w, h, step):
    """Downsample the backdrop and label each sample land or water.

    Water in this photo is dark and colour-neutral; the shore is forest, roofs
    and sand, all of which are either green-dominant or plainly bright. A
    brightness threshold alone does not separate them -- dark forest overlaps
    the lake -- so greenness carries the decision and brightness only catches
    roads and roofs.
    """
    ow = w // step
    oh = h // step
    m = []
    for oy in range(oh):
        base = off + oy * step * w * 3
        row = bytearray(ow)
        for ox in range(ow):
            p = base + ox * step * 3
            r, g, b = d[p], d[p + 1], d[p + 2]
            bright = (r * 30 + g * 59 + b * 11) // 100
            row[ox] = 1 if (g > b + 6 or bright > 90) else 0
        m.append(row)
    return m, ow, oh


def integral(m, ow, oh):
    """Summed-area table, so box queries below are O(1) instead of O(r^2)."""
    ii = [[0] * (ow + 1) for _ in range(oh + 1)]
    for y in range(oh):
        rowsum = 0
        prev = ii[y]
        cur = ii[y + 1]
        mr = m[y]
        for x in range(ow):
            rowsum += mr[x]
            cur[x + 1] = prev[x + 1] + rowsum
    return ii


def box_sum(ii, x0, y0, x1, y1):
    return (ii[y1 + 1][x1 + 1] - ii[y0][x1 + 1]
            - ii[y1 + 1][x0] + ii[y0][x0])


def majority(m, ow, oh, radius, passes):
    """Speckle removal. The raw per-pixel labels are noisy over open water
    (sun glint, wave texture); a few majority passes collapse that without
    moving the shoreline, which is a strong edge."""
    for _ in range(passes):
        ii = integral(m, ow, oh)
        out = []
        for y in range(oh):
            y0, y1 = max(0, y - radius), min(oh - 1, y + radius)
            row = bytearray(ow)
            for x in range(ow):
                x0, x1 = max(0, x - radius), min(ow - 1, x + radius)
                s = box_sum(ii, x0, y0, x1, y1)
                c = (x1 - x0 + 1) * (y1 - y0 + 1)
                row[x] = 1 if s * 2 > c else 0
            out.append(row)
        m = out
    return m


def dilate(m, ow, oh, radius):
    """Grow land into the water by `radius` cells.

    This is how the safety buffer is applied. Doing it on the mask rather than
    by offsetting polygons afterwards keeps the display layer and the collision
    tiles derived from one identical shape, and sidesteps polygon offsetting
    entirely (which is where self-intersections would come from)."""
    if radius <= 0:
        return m
    ii = integral(m, ow, oh)
    out = []
    for y in range(oh):
        y0, y1 = max(0, y - radius), min(oh - 1, y + radius)
        row = bytearray(ow)
        for x in range(ow):
            x0, x1 = max(0, x - radius), min(ow - 1, x + radius)
            row[x] = 1 if box_sum(ii, x0, y0, x1, y1) > 0 else 0
        out.append(row)
    return out


def pad_water(m, ow, oh, pad):
    """Surround the mask with water.

    Without this, land that runs off the edge of the backdrop produces an open
    contour rather than a closed loop, and an open path cannot be walked
    correctly from an arbitrary starting segment. Padding makes every contour
    closed, which is also the shape we want for display: land simply continues
    to the edge of the chart.
    """
    if pad <= 0:
        return m, ow, oh
    nw, nh = ow + 2 * pad, oh + 2 * pad
    out = [bytearray(nw) for _ in range(pad)]
    for row in m:
        nr = bytearray(nw)
        nr[pad:pad + ow] = row
        out.append(nr)
    out.extend(bytearray(nw) for _ in range(pad))
    return out, nw, nh


# --------------------------------------------------------------------------
# Connected components

def components(m, ow, oh, min_cells):
    """Label 8-connected land blobs, dropping anything too small to matter."""
    lab = [[0] * ow for _ in range(oh)]
    comps = []
    nid = 0
    for sy in range(oh):
        for sx in range(ow):
            if m[sy][sx] == 0 or lab[sy][sx]:
                continue
            nid += 1
            stack = [(sx, sy)]
            lab[sy][sx] = nid
            cells = []
            while stack:
                x, y = stack.pop()
                cells.append((x, y))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < ow and 0 <= ny < oh:
                            if m[ny][nx] and not lab[ny][nx]:
                                lab[ny][nx] = nid
                                stack.append((nx, ny))
            if len(cells) >= min_cells:
                comps.append((nid, cells))
            else:
                for x, y in cells:
                    lab[y][x] = 0
    return lab, comps


# --------------------------------------------------------------------------
# Contour extraction (marching squares)

def contours(mask_at, ow, oh):
    """Trace closed land/water boundaries.

    Marching squares over the sample grid. Every boundary cell contributes one
    or two segments at cell-edge midpoints; those get stitched into loops by
    endpoint matching. Preferred over Moore boundary tracing because it gives
    consistent orientation and handles the saddle cases without special-casing
    diagonal connectivity.
    """
    segs = []

    def key(p):
        return (int(round(p[0] * 2)), int(round(p[1] * 2)))

    for y in range(oh - 1):
        for x in range(ow - 1):
            tl = mask_at(x, y)
            tr = mask_at(x + 1, y)
            br = mask_at(x + 1, y + 1)
            bl = mask_at(x, y + 1)
            case = tl * 8 + tr * 4 + br * 2 + bl
            if case == 0 or case == 15:
                continue
            top = (x + 0.5, y)
            right = (x + 1.0, y + 0.5)
            bottom = (x + 0.5, y + 1.0)
            left = (x, y + 0.5)
            # Oriented so land lies to the left of travel.
            table = {
                1:  [(bottom, left)],
                2:  [(right, bottom)],
                3:  [(right, left)],
                4:  [(top, right)],
                5:  [(top, left), (bottom, right)],
                6:  [(top, bottom)],
                7:  [(top, left)],
                8:  [(left, top)],
                9:  [(bottom, top)],
                10: [(left, bottom), (right, top)],
                11: [(right, top)],
                12: [(left, right)],
                13: [(bottom, right)],
                14: [(left, bottom)],
            }
            for a, b in table[case]:
                segs.append((a, b))

    # Stitch segments into loops. Index by start point so the walk is O(n).
    from collections import defaultdict
    outgoing = defaultdict(list)
    for i, (a, b) in enumerate(segs):
        outgoing[key(a)].append(i)

    used = [False] * len(segs)
    loops = []

    # Any segment whose start is nobody's end begins an open path. Those must
    # be walked from their true head: starting mid-path and consuming segments
    # would strand the remainder as short fragments. Padding the mask with
    # water should leave none of these, but the ordering costs nothing and
    # keeps the tracer correct if it is ever run unpadded.
    endkeys = set()
    for a, b in segs:
        endkeys.add(key(b))
    heads = [i for i, (a, b) in enumerate(segs) if key(a) not in endkeys]
    order = heads + [i for i in range(len(segs)) if i not in set(heads)]

    for i in order:
        if used[i]:
            continue
        used[i] = True
        a0, b0 = segs[i]
        chain = [a0, b0]
        cur = b0
        while True:
            nxt = -1
            for j in outgoing.get(key(cur), ()):
                if not used[j]:
                    nxt = j
                    break
            if nxt < 0:
                break
            used[nxt] = True
            cur = segs[nxt][1]
            chain.append(cur)
            if key(cur) == key(a0):
                break
        if len(chain) > 8:
            loops.append(chain)
    return loops


# --------------------------------------------------------------------------
# Polyline utilities

def dp_simplify(pts, tol):
    """Douglas-Peucker."""
    if len(pts) < 3:
        return list(pts)
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        ax, ay = pts[i]
        bx, by = pts[j]
        dx, dy = bx - ax, by - ay
        den = math.hypot(dx, dy)
        best, bi = -1.0, -1
        for k in range(i + 1, j):
            px, py = pts[k]
            if den < 1e-9:
                dist = math.hypot(px - ax, py - ay)
            else:
                dist = abs(dy * px - dx * py + bx * ay - by * ax) / den
            if dist > best:
                best, bi = dist, k
        if best > tol:
            keep[bi] = True
            stack.append((i, bi))
            stack.append((bi, j))
    return [p for p, k in zip(pts, keep) if k]


def resample(pts, spacing):
    """Walk a polyline at fixed arclength, so the tiles come out uniform."""
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    carry = 0.0
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-9:
            continue
        t = spacing - carry
        while t <= seg:
            out.append((ax + (bx - ax) * t / seg, ay + (by - ay) * t / seg))
            t += spacing
        carry = (carry + seg) % spacing
    if math.hypot(out[-1][0] - pts[-1][0], out[-1][1] - pts[-1][1]) > spacing * 0.4:
        out.append(pts[-1])
    return out


def clip_poly(pts, x0, y0, x1, y1):
    """Sutherland-Hodgman against the display box (convex clip region)."""
    def inside(p, edge):
        if edge == 0:
            return p[0] >= x0
        if edge == 1:
            return p[0] <= x1
        if edge == 2:
            return p[1] >= y0
        return p[1] <= y1

    def cross(p, q, edge):
        px, py = p
        qx, qy = q
        if edge in (0, 1):
            xe = x0 if edge == 0 else x1
            t = (xe - px) / (qx - px)
            return (xe, py + (qy - py) * t)
        ye = y0 if edge == 2 else y1
        t = (ye - py) / (qy - py)
        return (px + (qx - px) * t, ye)

    out = list(pts)
    for edge in range(4):
        if not out:
            return []
        inp, out = out, []
        for i in range(len(inp)):
            cur = inp[i]
            prv = inp[i - 1]
            ci, pi = inside(cur, edge), inside(prv, edge)
            if ci:
                if not pi:
                    out.append(cross(prv, cur, edge))
                out.append(cur)
            elif pi:
                out.append(cross(prv, cur, edge))
    return out


def dedup(pts, eps=0.05):
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
            out.append(p)
    if len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= eps:
        out.pop()
    return out


def is_convex(pts):
    """Reproduce XYPolygon::determine_convexity() exactly.

    MOOS does not use the turn-direction test. For every edge it requires all
    remaining vertices to lie strictly on one side of that edge's line;
    vertices exactly on the line are ignored, and an edge with no off-line
    vertex at all marks the polygon non-convex. That last clause is what
    rejects degenerate zero-area quads, so the weaker turn test would let
    through polygons pObstacleMgr and BHV_AvoidObstacleV24 then refuse.
    """
    n = len(pts)
    if n < 3:
        return False
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        sign = 0
        for j in range(n):
            if j == i or j == (i + 1) % n:
                continue
            cx, cy = pts[j]
            cr = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
            if cr == 0:
                continue
            s = 1 if cr > 0 else -1
            if sign == 0:
                sign = s
            elif s != sign:
                return False
        if sign == 0:
            return False
    return True


def round_pts(pts, ndigits=1):
    """Snap to the precision the geometry is actually published at.

    Convexity has to hold for the coordinates that reach the MOOSDB, not the
    full-precision ones -- a thin quad can be convex before rounding and
    degenerate after.
    """
    return [(round(x, ndigits), round(y, ndigits)) for x, y in pts]


# --------------------------------------------------------------------------
# Output formatting

def pts_spec(pts, ndigits=1):
    return ":".join("%.*f,%.*f" % (ndigits, x, ndigits, y) for x, y in pts)


def png_write(path, rows, w, h):
    def chunk(t, data):
        c = t + data
        return (struct.pack(">I", len(data)) + c
                + struct.pack(">I", zlib.crc32(c) & 0xffffffff))
    raw = b"".join(b"\x00" + r for r in rows)
    open(path, "wb").write(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b""))


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tif", default=TIF)
    ap.add_argument("--info", default=INFO)
    ap.add_argument("--step", type=int, default=4,
                    help="backdrop downsample factor (default 4, ~0.86 m/sample)")
    ap.add_argument("--smooth-radius", type=int, default=2)
    ap.add_argument("--smooth-passes", type=int, default=3)
    ap.add_argument("--buffer", type=float, default=4.0,
                    help="meters to grow land into the water (default 4)")
    ap.add_argument("--simplify", type=float, default=5.0,
                    help="Douglas-Peucker tolerance for the COLLISION tiles, "
                         "meters (default 5). Coarser than the display: every "
                         "vertex here becomes another tile, and every tile in "
                         "range becomes another spawned behavior for the helm "
                         "to solve over.")
    ap.add_argument("--display-simplify", type=float, default=1.0,
                    help="Douglas-Peucker tolerance for the DRAWN shoreline, "
                         "meters (default 1). Costs nothing at run time, so "
                         "it stays fine.")
    ap.add_argument("--spacing", type=float, default=50.0,
                    help="max collision tile width along shore, meters (default 20)")
    ap.add_argument("--depth", type=float, default=25.0,
                    help="how far each tile extends inland, meters (default 25)")
    ap.add_argument("--overlap", type=float, default=0.35,
                    help="tile end extension, as a fraction of spacing (default 0.4)")
    ap.add_argument("--min-area", type=float, default=400.0,
                    help="drop land blobs smaller than this, sq meters")
    ap.add_argument("--area", default="-160,-210,275,80",
                    help="x0,y0,x1,y1 window to emit geometry for")
    ap.add_argument("--preview", action="store_true",
                    help="also write land_preview.png")
    ap.add_argument("--verify", action="store_true",
                    help="check the tile wall for gaps along the shoreline")
    ap.add_argument("--outdir", default=os.path.dirname(os.path.abspath(__file__)))
    args = ap.parse_args()

    ax0, ay0, ax1, ay1 = [float(v) for v in args.area.split(",")]
    ax0, ax1 = min(ax0, ax1), max(ax0, ax1)
    ay0, ay1 = min(ay0, ay1), max(ay0, ay1)

    info = read_info(args.info)
    xw, xe, ys, yn = geo_bounds(info)
    d, off, w, h = read_tiff_rgb(args.tif)

    m, ow, oh = classify(d, off, w, h, args.step)
    mpp_x = (xe - xw) / ow
    mpp_y = (yn - ys) / oh
    mpp = 0.5 * (mpp_x + mpp_y)

    sys.stderr.write("backdrop %dx%d -> mask %dx%d  (%.2f m/sample)\n"
                     % (w, h, ow, oh, mpp))
    sys.stderr.write("extent x %.1f..%.1f  y %.1f..%.1f\n" % (xw, xe, ys, yn))

    m = majority(m, ow, oh, args.smooth_radius, args.smooth_passes)

    # Two masks, deliberately. The operator is shown the real waterline; the
    # helm is walled off from a shoreline grown `buffer` meters into the lake.
    # Deriving both from one mask would put the tile wall's seaward face
    # exactly on the drawn shore, leaving the boats no margin at all -- and
    # coverage so knife-edge that rounding the published coordinates to one
    # decimal opened gaps in it.
    shore_mask = m
    buff_mask = dilate(m, ow, oh, int(round(args.buffer / mpp)))

    # Water border, so every contour closes. See pad_water().
    PAD = 2
    shore_mask, _, _ = pad_water(shore_mask, ow, oh, PAD)
    buff_mask, ow, oh = pad_water(buff_mask, ow, oh, PAD)

    def g2w(i, j):
        return (xw + (i - PAD) * mpp_x, yn - (j - PAD) * mpp_y)

    def w2g(x, y):
        return (int(round((x - xw) / mpp_x)) + PAD,
                int(round((yn - y) / mpp_y)) + PAD)

    min_cells = int(args.min_area / (mpp_x * mpp_y))

    def rings(mask, simplify):
        """Simplified world-space outlines of every land mass in a mask."""
        lab, comps = components(mask, ow, oh, min_cells)
        out = []
        for cid, _cells in comps:
            def mask_at(x, y, cid=cid):
                return 1 if lab[y][x] == cid else 0
            for loop in contours(mask_at, ow, oh):
                world = dedup([g2w(px, py) for px, py in loop])
                if len(world) < 4:
                    continue
                simp = dedup(dp_simplify(world, simplify))
                if len(simp) >= 4:
                    out.append(simp)
        return len(comps), out

    nblobs, shore_rings = rings(shore_mask, args.display_simplify)
    _, buff_rings = rings(buff_mask, args.simplify)
    sys.stderr.write("land blobs kept: %d (min %d cells)\n" % (nblobs, min_cells))

    # ---- Display layer: the true waterline, clipped to the window.
    display = []
    for simp in shore_rings:
        clipped = dedup(clip_poly(simp, ax0, ay0, ax1, ay1))
        if len(clipped) < 3:
            continue
        area = 0.0
        for i in range(len(clipped)):
            x1_, y1_ = clipped[i]
            x2_, y2_ = clipped[(i + 1) % len(clipped)]
            area += x1_ * y2_ - x2_ * y1_
        if abs(area) / 2.0 >= args.min_area:
            display.append(clipped)

    # ---- Collision layer: convex tiles along the buffered shore.
    tiles = []
    dropped_nonconvex = 0
    for simp in buff_rings:
        # Tiles are laid on the simplified polyline itself, subdividing only
        # the long edges. Resampling at a fixed arclength instead would place
        # tile bases on chords that cut inside every convex headland, leaving
        # the shoreline arc sticking out through the wall -- gaps of 10-20 m
        # in practice. Splitting an edge adds points along a straight line, so
        # it cannot cut a corner.
        walk = []
        ring = simp + [simp[0]]
        for i in range(len(ring) - 1):
            ax_, ay_ = ring[i]
            bx_, by_ = ring[i + 1]
            seg = math.hypot(bx_ - ax_, by_ - ay_)
            walk.append((ax_, ay_))
            parts = int(math.ceil(seg / args.spacing))
            for k in range(1, parts):
                t = k / float(parts)
                walk.append((ax_ + (bx_ - ax_) * t, ay_ + (by_ - ay_) * t))
        walk.append(ring[-1])

        for i in range(len(walk) - 1):
            px, py = walk[i]
            qx, qy = walk[i + 1]
            # Only tile the stretch the vehicles can actually reach.
            mx, my = 0.5 * (px + qx), 0.5 * (py + qy)
            if not (ax0 - args.spacing <= mx <= ax1 + args.spacing
                    and ay0 - args.spacing <= my <= ay1 + args.spacing):
                continue
            # Skip the ring running along the water padding. That border is an
            # artifact of closing the contours, not a shore, and its inward
            # normal points off the edge of the chart -- which would wall off
            # open water at the chart boundary.
            edge = 3 * mpp
            if not (xw + edge <= px <= xe - edge
                    and xw + edge <= qx <= xe - edge
                    and ys + edge <= py <= yn - edge
                    and ys + edge <= qy <= yn - edge):
                continue
            dx, dy = qx - px, qy - py
            L = math.hypot(dx, dy)
            if L < 1e-6:
                continue
            ux, uy = dx / L, dy / L
            # Overlap neighbours so convex corners leave no gap. A tile is a
            # parallelogram off one segment, so at a seaward-bulging corner two
            # neighbours splay apart; extending both ends of every base segment
            # closes the wedge between them.
            ext = args.spacing * args.overlap
            px, py = px - ux * ext, py - uy * ext
            qx, qy = qx + ux * ext, qy + uy * ext
            # Inland direction is fixed by the contour winding, not by probing.
            # contours() emits every ring -- outer shores and inland ponds
            # alike -- with land on the left in grid space; the grid-to-world
            # mapping flips y, so land ends up at (-uy, ux) of the world-space
            # heading. No sampling needed, and no way for a ragged notch to
            # flip a tile seaward.
            nx, ny = -uy, ux

            # Sanity check only. If the far side really does look like the land
            # side, this segment is a one-cell sliver where the winding is
            # meaningless, so drop it rather than wall off open water.
            def land_votes(sx, sy, mx=mx, my=my):
                v = 0
                for dist in (3.0, 6.0, 10.0):
                    ti, tj = w2g(mx + sx * dist, my + sy * dist)
                    if 0 <= ti < ow and 0 <= tj < oh and buff_mask[tj][ti] == 1:
                        v += 1
                return v

            if land_votes(-nx, -ny) > land_votes(nx, ny):
                continue
            D = args.depth
            quad = round_pts([(px, py), (qx, qy),
                              (qx + nx * D, qy + ny * D),
                              (px + nx * D, py + ny * D)])
            if not is_convex(quad):
                dropped_nonconvex += 1
                continue
            tiles.append(quad)

    sys.stderr.write("display polygons: %d   collision tiles: %d"
                     " (%d dropped non-convex)\n"
                     % (len(display), len(tiles), dropped_nonconvex))
    if not tiles:
        sys.exit("no land found in the requested area -- check --area")

    if args.verify:
        # Walk the shoreline at 1 m and confirm every point sits inside at
        # least one tile. A gap here is a doorway the helm would happily
        # steer through, so this is the check that matters.
        def in_quad(pt, quad):
            sign = 0
            for k in range(4):
                ax_, ay_ = quad[k]
                bx_, by_ = quad[(k + 1) % 4]
                cr = ((bx_ - ax_) * (pt[1] - ay_) - (by_ - ay_) * (pt[0] - ax_))
                if abs(cr) < 1e-9:
                    continue
                s = 1 if cr > 0 else -1
                if sign == 0:
                    sign = s
                elif s != sign:
                    return False
            return True

        checked = 0
        gaps = []
        for poly in display:
            walk = resample(poly + [poly[0]], 1.0)
            for pt in walk:
                # Skip anything running along the window clip or the chart
                # border: those edges are bookkeeping, not shoreline, and no
                # tile is expected to cover them.
                mgn = 4.0
                if not (ax0 + mgn <= pt[0] <= ax1 - mgn
                        and ay0 + mgn <= pt[1] <= ay1 - mgn):
                    continue
                if not (xw + mgn <= pt[0] <= xe - mgn
                        and ys + mgn <= pt[1] <= yn - mgn):
                    continue
                checked += 1
                if not any(in_quad(pt, q) for q in tiles):
                    gaps.append(pt)
        if checked:
            sys.stderr.write("verify: %d shoreline samples, %d uncovered (%.2f%%)\n"
                             % (checked, len(gaps), 100.0 * len(gaps) / checked))
            for pt in gaps[:10]:
                sys.stderr.write("  gap at %.1f,%.1f\n" % pt)

    od = args.outdir
    # Record the settings that shaped the geometry, not how the run was
    # invoked: --preview and --verify are diagnostics and would otherwise
    # churn the header between otherwise identical regenerations.
    cmdline = ("%s --buffer %g --simplify %g --spacing %g --depth %g "
               "--overlap %g --display-simplify %g --area %s"
               % (os.path.basename(sys.argv[0]), args.buffer, args.simplify,
                  args.spacing, args.depth, args.overlap,
                  args.display_simplify, args.area))
    hdr = ("// AUTOGENERATED by %s -- do not edit by hand.\n"
           "// source: %s\n"
           "// buffer=%.1fm simplify=%.1fm spacing=%.1fm depth=%.1fm area=%s\n"
           % (cmdline, os.path.basename(args.tif), args.buffer,
              args.simplify, args.spacing, args.depth, args.area))

    # The shoreline the operator sees traced as a line, as opposed to the
    # filled land body. Only the stretches that are genuinely waterline: the
    # runs lying along the clip window or the edge of the chart are bookkeeping
    # and would otherwise be drawn as if they were shore.
    def shore_runs(poly):
        eps = 0.25

        def on_border(p):
            return (abs(p[0] - ax0) < eps or abs(p[0] - ax1) < eps
                    or abs(p[1] - ay0) < eps or abs(p[1] - ay1) < eps
                    or abs(p[0] - xw) < 3 * mpp or abs(p[0] - xe) < 3 * mpp
                    or abs(p[1] - ys) < 3 * mpp or abs(p[1] - yn) < 3 * mpp)

        n = len(poly)
        flags = [on_border(p) for p in poly]
        if all(flags):
            return []
        if not any(flags):
            return [poly + [poly[0]]]
        start = flags.index(True)
        runs, cur = [], []
        for k in range(n + 1):
            p = poly[(start + k) % n]
            if on_border(p):
                if len(cur) >= 2:
                    runs.append(cur)
                cur = []
            else:
                cur.append(p)
        if len(cur) >= 2:
            runs.append(cur)
        return runs

    seglists = []
    for i, poly in enumerate(display):
        for j, run in enumerate(shore_runs(poly)):
            seglists.append(("shore_%d_%d" % (i, j), run))
    sys.stderr.write("shoreline seglists: %d\n" % len(seglists))

    # ---- land.txt: the canonical geometry, for tools and for reference.
    with open(os.path.join(od, "land.txt"), "w") as f:
        f.write(hdr)
        f.write("//\n// display: non-convex, drawn only.  tile: convex, collision only.\n\n")
        for i, poly in enumerate(display):
            f.write("display = pts={%s},label=land_%d\n" % (pts_spec(poly), i))
        f.write("\n")
        for i, quad in enumerate(tiles):
            f.write("tile = pts={%s},label=land_t%03d\n" % (pts_spec(quad), i))

    # ---- pObstacleMgr plug, included by each vehicle. Collision layer only.
    with open(os.path.join(od, "plug_land_obstacles.moos"), "w") as f:
        f.write(hdr)
        f.write("""//
// pObstacleMgr Config Block -- static land obstacles.
//
// The shoreline is given up front rather than sensed: it is charted terrain,
// not a contact. Hence given_obstacle rather than uFldObstacleSim, whose
// sensor_range and duration semantics would make the shore blink in and out.
//
// given_max_duration = off is required. The default is 60 seconds, after
// which the land would quietly expire and the vehicles would sail ashore.
//
// post_view_polys = false on purpose. These tiles are the collision
// decomposition -- roughly %d overlapping parallelograms. Drawing them
// produces a sawtooth mess, and three vehicles would each draw their own
// copy. The shoreline the operator sees is posted by the shoreside instead
// (plug_land_view.moos), from this same extraction run.

ProcessConfig = pObstacleMgr
{
  AppTick   = 2
  CommsTick = 2

  given_max_duration = off
  post_view_polys    = false
  post_dist_to_polys = false

  alert_range  = $(LAND_ALERT_RANGE=35)
  ignore_range = -1

""" % len(tiles))
        for i, quad in enumerate(tiles):
            f.write("  given_obstacle = pts={%s},label=land_t%03d\n"
                    % (pts_spec(quad), i))
        f.write("}\n")

    # ---- shoreside display plug. Independent of the vehicles, so the
    #      operator sees the shore before anything is deployed.
    with open(os.path.join(od, "plug_land_view.moos"), "w") as f:
        f.write(hdr)
        f.write("""//
// Shoreline display for the operator's viewer.
//
// Posted by the shoreside so the land is on screen from startup, before any
// vehicle is up and before the operator draws the mission region. That
// ordering is the whole point: the region is drawn by clicking on the chart,
// so the land has to be visible first. Nothing here is ever read by a
// behavior -- these are the true, non-convex waterlines, whereas the helm
// avoids the convex tiles in plug_land_obstacles.moos.
//
// pMarineViewer parses VIEW_POLYGON through stringStandard2Poly(), which adds
// vertices with the convexity check disabled, so non-convex polygons render
// fine.
//
// Three scripts, because the visibility toggle needs a seed:
//
//   LandInit   fires once, to give LAND_VISIBLE a value. Without it neither
//              of the conditional scripts below would ever run.
//   LandDraw   repaints while LAND_VISIBLE is true, on a slow loop so that a
//              viewer restarted mid-mission gets the land back.
//   LandErase  clears the same labels while LAND_VISIBLE is false. Erasing
//              needs its own script: pausing LandDraw would only stop the
//              repaint, leaving whatever is already on screen.

ProcessConfig = uTimerScript_LandInit
{
  AppTick   = 2
  CommsTick = 2

  paused    = false
  reset_max = 0            // once, at startup

  event = var=LAND_VISIBLE, val=true, time=0
}

ProcessConfig = uTimerScript_LandDraw
{
  AppTick   = 2
  CommsTick = 2

  paused        = false
  reset_max     = unlimited
  reset_time    = 30
  script_atomic = true

  condition = LAND_VISIBLE = true

""")
        for i, poly in enumerate(display):
            f.write('  event = var=VIEW_POLYGON, val="pts={%s},label=land_%d,'
                    'edge_color=tan,vertex_color=tan,fill_color=darkkhaki,'
                    'fill_transparency=0.45,vertex_size=1,edge_size=2", time=1\n'
                    % (pts_spec(poly), i))
        f.write("\n")
        for lbl, run in seglists:
            f.write('  event = var=VIEW_SEGLIST, val="pts={%s},label=%s,'
                    'edge_color=sandybrown,vertex_color=invisible,'
                    'edge_size=3", time=1\n' % (pts_spec(run), lbl))
        f.write("""}

ProcessConfig = uTimerScript_LandErase
{
  AppTick   = 2
  CommsTick = 2

  paused        = false
  reset_max     = unlimited
  reset_time    = 30
  script_atomic = true

  condition = LAND_VISIBLE = false

""")
        # get_spec_inactive() form: a throwaway triangle plus active=false,
        # matched by label. Short, and exactly what pObstacleMgr posts to
        # retract an obstacle.
        for i in range(len(display)):
            f.write('  event = var=VIEW_POLYGON, '
                    'val="pts={0,0:9,0:0,9},active=false,label=land_%d", '
                    'time=1\n' % i)
        for lbl, _run in seglists:
            f.write('  event = var=VIEW_SEGLIST, '
                    'val="pts={0,0:9,0:0,9},active=false,label=%s", '
                    'time=1\n' % lbl)
        f.write("}\n")

    if args.preview:
        rows = []
        for y in range(oh):
            row = bytearray()
            for x in range(ow):
                if buff_mask[y][x] and not shore_mask[y][x]:
                    row += b"\xc8\xb0\x60"          # buffer ring
                elif buff_mask[y][x]:
                    row += b"\x86\x6b\x3a"          # land
                else:
                    row += b"\x28\x60\xb8"          # water
            rows.append(bytes(row))

        def stamp(pts, col, closed=True):
            n = len(pts)
            rng = range(n) if closed else range(n - 1)
            for i in rng:
                x0_, y0_ = pts[i]
                x1_, y1_ = pts[(i + 1) % n]
                i0 = (x0_ - xw) / mpp_x + PAD
                j0 = (yn - y0_) / mpp_y + PAD
                i1 = (x1_ - xw) / mpp_x + PAD
                j1 = (yn - y1_) / mpp_y + PAD
                steps = int(max(abs(i1 - i0), abs(j1 - j0))) + 1
                for s in range(steps + 1):
                    t = s / steps
                    xi = int(i0 + (i1 - i0) * t)
                    yi = int(j0 + (j1 - j0) * t)
                    if 0 <= xi < ow and 0 <= yi < oh:
                        rows[yi][xi * 3:xi * 3 + 3] = col

        rows = [bytearray(r) for r in rows]
        for quad in tiles:
            stamp(quad, b"\xff\xd0\x00")
        for poly in display:
            stamp(poly, b"\xff\x30\x30")
        png_write(os.path.join(od, "land_preview.png"),
                  [bytes(r) for r in rows], ow, oh)
        sys.stderr.write("wrote land_preview.png\n")

    sys.stderr.write("wrote land.txt, plug_land_obstacles.moos, plug_land_view.moos\n")


if __name__ == "__main__":
    main()
