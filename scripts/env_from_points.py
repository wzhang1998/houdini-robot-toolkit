"""Touch-off points (probe_env.py) -> shapes in the env file.

    python scripts/env_from_points.py envs/volvox_lab_points.json [--env envs/volvox_lab.json] [--dry-run]

Points are grouped by name "object:kind" (probe_env.py's prompt):

    plane      >= 3 points -> halfspace, least-squares fit, oriented so the
               robot's shoulder is on the free side (any tilt)
    wall       >= 2 points (3 to check) -> a VERTICAL halfspace: a line fit
               seen from above -- few points, spread along the wall
    level      >= 1 point (3 to check) -> a HORIZONTAL halfspace at their
               mean height (a floor; a ceiling or table top above the shoulder)
    box        >= 3 top corners (+ optional "object:bottom" points; default
               the floor) -> box, yaw from the smallest-area rectangle
    cylinder   >= 3 points round its foot -> cylinder (height: an
               "object:top" point, else 2.0 m)
    point      kept as reference points only

The ceiling is out of the arm's reach: put its tape-measured height above
the floor in the points file as "ceiling_height_m" (probe_ui.py has a field)
and it becomes a plane parallel to the floor.

An object already in the env keeps its role, note and slow-zone speed; only
its geometry is replaced, and the change is printed (how far the estimate
was off). New objects are obstacles, keep_out when the name says operator /
person. The old env is kept as <env>.bak.

Pure Python. Tests: python scripts/env_from_points.py --self-test
"""

import argparse
import json
import math
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

SHOULDER = (0.0, 0.0, 0.5)          # a point that must be on the free side of every plane


def _mean(pts):
    return [sum(p[i] for p in pts) / len(pts) for i in range(3)]


def fit_plane(pts):
    """(normal, offset): n . x = offset, least squares, n towards SHOULDER."""
    c = _mean(pts)
    cov = [[sum((p[i] - c[i]) * (p[j] - c[j]) for p in pts) for j in range(3)] for i in range(3)]
    tr = cov[0][0] + cov[1][1] + cov[2][2] + 1e-12
    M = [[(tr if i == j else 0.0) - cov[i][j] for j in range(3)] for i in range(3)]   # smallest eigvec of cov
    v = (0.3, 0.5, 0.8)
    for _ in range(200):
        w = [sum(M[i][j] * v[j] for j in range(3)) for i in range(3)]
        n = math.sqrt(sum(x * x for x in w))
        v = tuple(x / n for x in w)
    off = sum(v[i] * c[i] for i in range(3))
    if sum(v[i] * SHOULDER[i] for i in range(3)) < off:
        v, off = tuple(-x for x in v), -off
    rms = math.sqrt(sum((sum(v[i] * p[i] for i in range(3)) - off) ** 2 for p in pts) / len(pts))
    return [round(x, 6) for x in v], round(off, 5), rms


def plane_width(pts):
    """How wide the points spread across their longest direction, m: the
    tallest triangle any three of them make. Points nearly in a line leave
    the plane's tilt about that line unmeasured."""
    best = 0.0
    n = len(pts)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                a, b, c = pts[i], pts[j], pts[k]
                u = [b[t] - a[t] for t in range(3)]
                v = [c[t] - a[t] for t in range(3)]
                cr = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
                longest = max(math.dist(a, b), math.dist(b, c), math.dist(a, c))
                if longest > 0:
                    best = max(best, math.sqrt(sum(x * x for x in cr)) / longest)
    return best


MIN_WIDTH_M = 0.3                   # plane points narrower than this: warn


def fit_wall(pts):
    """A vertical plane: a least-squares line through the points seen from
    above, normal towards SHOULDER. Returns (geo, horizontal spread m)."""
    c = _mean(pts)
    sxx = sum((p[0] - c[0]) ** 2 for p in pts)
    syy = sum((p[1] - c[1]) ** 2 for p in pts)
    sxy = sum((p[0] - c[0]) * (p[1] - c[1]) for p in pts)
    a = 0.5 * math.atan2(2 * sxy, sxx - syy)          # direction of the line
    d = (math.cos(a), math.sin(a))
    n = [-d[1], d[0], 0.0]
    off = n[0] * c[0] + n[1] * c[1]
    if n[0] * SHOULDER[0] + n[1] * SHOULDER[1] < off:
        n, off = [-n[0], -n[1], 0.0], -off
    rms = math.sqrt(sum((n[0] * p[0] + n[1] * p[1] - off) ** 2 for p in pts) / len(pts))
    along = [d[0] * p[0] + d[1] * p[1] for p in pts]
    return ({"type": "halfspace", "normal": [round(n[0], 6), round(n[1], 6), 0.0], "offset": round(off, 5),
             "_fit_rms_m": round(rms, 5), "_n": len(pts)}, max(along) - min(along))


def fit_level(pts):
    """A horizontal plane at the points' mean height, solid on the side away
    from SHOULDER (a floor below, a ceiling or table top above)."""
    z = sum(p[2] for p in pts) / len(pts)
    rms = math.sqrt(sum((p[2] - z) ** 2 for p in pts) / len(pts))
    up = SHOULDER[2] > z
    return {"type": "halfspace", "normal": [0.0, 0.0, 1.0 if up else -1.0], "offset": round(z if up else -z, 5),
            "_fit_rms_m": round(rms, 5), "_n": len(pts)}


def fit_box(top, bottom_z):
    """Smallest-area rectangle round the top points' footprint (0.25 deg steps)."""
    best = None
    for k in range(360):
        yaw = math.radians(k * 0.25)
        cy, sy = math.cos(yaw), math.sin(yaw)
        xs = [cy * p[0] + sy * p[1] for p in top]
        ys = [-sy * p[0] + cy * p[1] for p in top]
        area = (max(xs) - min(xs)) * (max(ys) - min(ys))
        if best is None or area < best[0] - 1e-12:
            best = (area, k * 0.25, (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
                    max(xs) - min(xs), max(ys) - min(ys))
    _, yaw_deg, lx, ly, sx, sy_ = best
    yaw = math.radians(yaw_deg)
    cx = math.cos(yaw) * lx - math.sin(yaw) * ly
    cy_ = math.sin(yaw) * lx + math.cos(yaw) * ly
    ztop = max(p[2] for p in top)
    return {"type": "box", "center": [round(cx, 4), round(cy_, 4), round((ztop + bottom_z) / 2, 4)],
            "size": [round(sx, 4), round(sy_, 4), round(ztop - bottom_z, 4)], "yaw_deg": round(yaw_deg, 2)}


def _circle(pts):
    """Least-squares circle through xy (Kasa): x^2 + y^2 + D x + E y + F = 0.
    Touch-off points are rarely spread evenly round a zone, and their mean
    is then off-centre."""
    S = [[0.0] * 3 for _ in range(3)]
    b = [0.0] * 3
    for p in pts:
        row = (p[0], p[1], 1.0)
        rhs = -(p[0] ** 2 + p[1] ** 2)
        for i in range(3):
            b[i] += row[i] * rhs
            for j in range(3):
                S[i][j] += row[i] * row[j]
    # Cramer's rule on the 3x3 normal equations
    det = lambda m: (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                     + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det(S)
    sol = []
    for k in range(3):
        m = [list(r) for r in S]
        for i in range(3):
            m[i][k] = b[i]
        sol.append(det(m) / d)
    D, E, F = sol
    cx, cy = -D / 2.0, -E / 2.0
    return cx, cy, math.sqrt(max(0.0, cx * cx + cy * cy - F))


def fit_cylinder(foot, top_z):
    cx, cy, r = _circle(foot)
    c = (cx, cy)
    r = max(r, max(math.hypot(p[0] - cx, p[1] - cy) for p in foot))
    z0 = min(p[2] for p in foot)
    return {"type": "cylinder", "center": [round(c[0], 4), round(c[1], 4), round(z0, 4)],
            "radius": round(r, 4), "height": round(top_z - z0, 4)}


def shapes(points, floor_z):
    groups = {}
    for p in points:
        name, _, kind = p["name"].partition(":")
        groups.setdefault((name, kind or "point"), []).append(p["tcp_m"])
    out = {}
    for (name, kind), pts in groups.items():
        if kind == "plane" and len(pts) >= 3:
            n, off, rms = fit_plane(pts)
            out[name] = {"type": "halfspace", "normal": n, "offset": off, "_fit_rms_m": round(rms, 5),
                         "_width_m": round(plane_width(pts), 4), "_n": len(pts)}
        elif kind == "wall" and len(pts) >= 2:
            geo, spread = fit_wall(pts)
            out[name] = dict(geo, _width_m=round(spread, 4))
        elif kind == "level" and len(pts) >= 1:
            out[name] = fit_level(pts)
        elif kind == "box" and len(pts) >= 3:
            bottom = groups.get((name, "bottom"))
            out[name] = fit_box(pts, min(p[2] for p in bottom) if bottom else floor_z)
        elif kind == "cylinder" and len(pts) >= 3:
            top = groups.get((name, "top"))
            out[name] = fit_cylinder(pts, max(p[2] for p in top) if top else 2.0)
    return out


def merge(env, new):
    """Replace geometry of named objects; add new ones. Returns change lines."""
    lines = []
    by_name = {o["name"]: o for o in env.get("objects", [])}
    for name, geo in new.items():
        o = by_name.get(name)
        if o is None:
            role = "keep_out" if any(k in name for k in ("operator", "person", "people")) else "obstacle"
            o = {"name": name, "role": role}
            env.setdefault("objects", []).append(o)
            lines.append("added %s (%s, %s)" % (name, geo["type"], role))
        else:
            if "center" in o and "center" in geo:
                lines.append("%s moved %.0f mm" % (name, 1000 * math.dist(o["center"], geo["center"])))
            elif "offset" in o and "offset" in geo:
                lines.append("%s plane offset %+.0f mm" % (name, 1000 * (geo["offset"] - o["offset"])))
            else:
                lines.append("%s replaced" % name)
            for k in ("center", "size", "yaw_deg", "radius", "height", "normal", "offset", "type"):
                o.pop(k, None)
        if geo.get("_width_m") is not None and geo["_width_m"] < MIN_WIDTH_M:
            lines[-1] += ("  -- WARNING: %s's points are nearly in a line (%.2f m wide), its tilt is not "
                          "measured; spread them out" % (name, geo["_width_m"]))
        elif geo.get("_n") == 3 and "_fit_rms_m" in geo and geo["_fit_rms_m"] < 1e-9:
            lines[-1] += "  (3 points: an exact fit, nothing to check it by -- a 4th point gives a residual)"
        elif "_fit_rms_m" in geo:
            lines[-1] += "  (fit rms %.1f mm)" % (1000 * geo["_fit_rms_m"])
        o.update({k: v for k, v in geo.items() if not k.startswith("_")})
        o["measured"] = True
    return lines


def set_ceiling(env, height_m):
    """The ceiling, out of the arm's reach, from a tape-measured height above
    the floor: a plane parallel to the env's floor, height_m above it, solid
    above. Returns the change line."""
    floor = next((o for o in env["objects"] if o["name"] == "floor" and o["type"] == "halfspace"), None)
    if floor is None:
        n, d = [0.0, 0.0, 1.0], 0.0
    else:
        L = math.sqrt(sum(x * x for x in floor["normal"]))
        n, d = [x / L for x in floor["normal"]], floor["offset"] / L
    geo = {"type": "halfspace", "normal": [round(-x, 6) for x in n], "offset": round(-(d + height_m), 5)}
    o = next((o for o in env["objects"] if o["name"] == "ceiling"), None)
    if o is None:
        o = {"name": "ceiling", "role": "obstacle"}
        env["objects"].append(o)
        line = "added ceiling %.2f m above the floor" % height_m
    else:
        line = "ceiling %.2f m above the floor (plane offset %+.0f mm)" % (
            height_m, 1000 * (geo["offset"] - o.get("offset", geo["offset"])))
        for k in ("center", "size", "yaw_deg", "radius", "height", "normal", "offset", "type"):
            o.pop(k, None)
    o.update(geo)
    o["measured"] = "tape"
    return line + ("" if floor else "  -- no floor in the env: assumed at the base (z = 0)")


def update_env(env, points, ceiling_height_m=None):
    """Fit the points and merge them into env (in place); then the ceiling
    from its tape height, when given (after the floor, which it follows).
    Returns (change lines, validation errors); the caller writes the file."""
    import collision as CL
    floor = next((o for o in env["objects"] if o["name"] == "floor" and o["type"] == "halfspace"), None)
    lines = merge(env, shapes(points, floor["offset"] if floor else 0.0))
    if ceiling_height_m:
        lines.append(set_ceiling(env, float(ceiling_height_m)))
    return lines, CL.validate_env(env)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("points", nargs="?")
    ap.add_argument("--env", default=os.path.join(ROOT, "envs", "volvox_lab.json"))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    env = json.load(open(a.env))
    data = json.load(open(a.points))
    lines, errs = update_env(env, data["points"], data.get("ceiling_height_m"))
    if errs:
        print("env would be invalid:", errs)
        return 1
    for line in lines:
        print(" ", line)
    if a.dry_run:
        return 0
    shutil.copyfile(a.env, a.env + ".bak")
    json.dump(env, open(a.env, "w"), indent=1)
    print("wrote", a.env, "(previous: .bak)")
    return 0


def self_test():
    import random
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    rnd = random.Random(5)
    # a wall 2.5 m out along -X, points with 2 mm noise
    wall = [(-2.5 + rnd.gauss(0, 0.002), rnd.uniform(-1, 1), rnd.uniform(0.2, 2.0)) for _ in range(5)]
    n, off, rms = fit_plane(wall)
    check("a wall's plane: normal towards the robot, 2.5 m out",
          abs(n[0] - 1.0) < 0.01 and abs(off + 2.5) < 0.005, "n %s offset %.4f rms %.4f" % (n, off, rms))
    # a 0.6 x 0.9 x 1.1 cart, yawed 30 deg, top corners only
    yaw = math.radians(30)
    corners = [(sx * 0.3, sy * 0.45) for sx in (-1, 1) for sy in (-1, 1)]
    top = [(0.8 + math.cos(yaw) * x - math.sin(yaw) * y, 1.1 + math.sin(yaw) * x + math.cos(yaw) * y, 1.08)
           for x, y in corners]
    b = fit_box(top, -0.02)
    check("a yawed box from its top corners", abs(b["yaw_deg"] % 90 - 30) < 0.3 and
          sorted(round(x, 3) for x in b["size"][:2]) == [0.6, 0.9] and abs(b["size"][2] - 1.1) < 1e-6,
          "%s" % b)
    c = fit_cylinder([(1.3 + 0.4 * math.cos(k), 1.0 + 0.4 * math.sin(k), 0.0) for k in range(0, 6)], 2.0)
    check("a cylinder from points round its foot", abs(c["radius"] - 0.4) < 1e-6 and c["height"] == 2.0, "%s" % c)
    env = {"schema": "motionlab.env/1", "objects": [
        {"name": "control_cart", "type": "box", "center": [0.75, 1.15, 0.55], "size": [0.65, 0.9, 1.1], "role": "obstacle", "note": "red"},
        {"name": "operator", "type": "cylinder", "center": [1.35, 1.0, 0.0], "radius": 0.4, "height": 2.0, "role": "keep_out"}]}
    lines = merge(env, {"control_cart": b, "wall_tv": {"type": "halfspace", "normal": n, "offset": off}})
    cart = env["objects"][0]
    check("merge keeps role and note, replaces geometry, adds new objects",
          cart["role"] == "obstacle" and cart["note"] == "red" and cart["yaw_deg"] == b["yaw_deg"]
          and any(o["name"] == "wall_tv" for o in env["objects"]), "; ".join(lines))
    # three floor points nearly in a line (a real take): the tilt about that
    # line is unmeasured -- the fit put the floor 15 cm up, tilted 6 deg
    line = [{"name": "floor:plane", "tcp_m": p} for p in
            ([0.0990, 1.2259, 0.0099], [0.7160, 1.0200, 0.0184], [-0.4026, 1.1803, 0.0198])]
    fl = shapes(line, 0.0)["floor"]
    check("plane points nearly in a line are measured as such", fl.get("_width_m", 9) < MIN_WIDTH_M, "%s" % fl.get("_width_m"))
    msg = merge({"schema": "motionlab.env/1", "objects": []}, shapes(line, 0.0))
    check("... and the change line warns", any("in a line" in m for m in msg), "; ".join(msg))
    spread = [{"name": "floor:plane", "tcp_m": p} for p in ([0.0, 1.2, 0.0], [0.8, -1.0, 0.0], [-1.0, 0.0, 0.0])]
    check("a triangle of floor points is wide", shapes(spread, 0.0)["floor"].get("_width_m", 0) > 1.0)
    # walls are vertical, floors level: fitting them so needs fewer points and
    # stops a narrow take tilting them (the real TV-wall take: 0.17 m across,
    # read as leaning 3.9 deg)
    tvw = [{"name": "wall_tv:wall", "tcp_m": p} for p in
           ([-1.71796, 0.35703, 0.77859], [-1.66866, 0.67874, 0.75272], [-1.83939, -0.18924, 0.29644])]
    w = shapes(tvw, 0.0)["wall_tv"]
    check("a wall fit is vertical and faces the robot", w["type"] == "halfspace" and w["normal"][2] == 0.0
          and w["normal"][0] > 0.9 and -1.9 < w["offset"] < -1.7, "%s" % w)
    check("... its spread is measured along the floor", w["_width_m"] > 0.8, "%.2f m" % w["_width_m"])
    lv = shapes([{"name": "floor:level", "tcp_m": p} for p in
                 ([0.09903, 1.22593, 0.00989], [0.71598, 1.02004, 0.01836], [-0.29773, 0.80754, -0.0048])], 0.0)["floor"]
    check("a level fit is horizontal, at the points' mean height, solid below",
          lv["normal"] == [0.0, 0.0, 1.0] and abs(lv["offset"] - 0.00782) < 1e-4, "%s" % lv)
    # the ceiling is out of reach: a tape-measured height above the measured
    # floor, parallel to it
    tilted = [{"name": "floor:plane", "tcp_m": p} for p in
              ([0.0, 1.2, -0.02], [0.8, -1.0, -0.02 + 0.018], [-1.0, 0.0, -0.02 - 0.01])]
    env3 = {"schema": "motionlab.env/1", "objects": []}
    msg3, errs3 = update_env(env3, tilted, ceiling_height_m=2.7)
    fl3 = next(o for o in env3["objects"] if o["name"] == "floor")
    ce = next((o for o in env3["objects"] if o["name"] == "ceiling"), None)
    ok = (ce is not None and not errs3 and ce["type"] == "halfspace"
          and all(abs(a + b) < 1e-9 for a, b in zip(ce["normal"], fl3["normal"]))
          and abs(-ce["offset"] - fl3["offset"] - 2.7) < 1e-6 and ce.get("measured") == "tape")
    check("ceiling = floor + tape height, parallel, solid side above", ok, "%s; %s" % (ce, msg3))
    import collision as CL
    check("... the robot's shoulder is on its free side, a point at 2.8 m inside it",
          CL.sdf(ce, (0.0, 0.0, 0.5)) > 2.0 and CL.sdf(ce, (0.0, 0.0, 2.8)) < 0, "")
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
