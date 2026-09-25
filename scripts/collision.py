"""Collision and safety zones for a joint trajectory -- the real2sim cell.

The robot is a chain of capsules, one per link, fitted to the URDF link meshes
so that every vertex is inside (conservative), plus an optional tool capsule.
The cell is an environment file (envs/*.json, schema motionlab.env/1) in the
robot base frame -- URDF, Z up, metres -- listing shapes with a role:

    obstacle   no link within the env's margin_m of it (walls, cart, floor)
    keep_out   no link inside it at all (where people stand)
    slow       inside it the TCP may not exceed tcp_speed_mps
               (ISO/TS 15066 style speed limiting near people)
    work       the TCP must stay inside it (the performance volume)

Shapes: box {center, size, yaw_deg}, cylinder {center (bottom), radius,
height}, sphere {center, radius}, halfspace {normal, offset}: the solid side
is n . x < offset (floor: normal [0,0,1], offset = floor height).

Distances are exact for capsule vs convex shape: the signed distance to a
convex set is convex along the capsule's segment, so a ternary search finds
its minimum. Self-collision: every pair of links that is not adjacent and
does not already touch at the zero pose (as MoveIt's SRDF "Adjacent" /
"Default" disables).

    model = load_model("fr20")                 capsules from the URDF meshes
    env = load_env("envs/volvox_lab.json")
    check(model, env, times, joints)  -> report {ok, min_clearance, closest,
                                                 violations[...]}
    capsules(model, q)                -> [(name, a, b, r)] in the base frame

Pure Python. Tests: python scripts/collision.py
"""

import json
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import urdf_rig as U  # noqa: E402

ENV_SCHEMA = "motionlab.env/1"
ROLES = ("obstacle", "keep_out", "slow", "work")
_MODELS = {}


# --------------------------------------------------------------------------
# robot capsules
# --------------------------------------------------------------------------

def _stl_vertices(path):
    with open(path, "rb") as f:
        d = f.read()
    n = struct.unpack_from("<I", d, 80)[0] if len(d) >= 84 else 0
    out = set()
    if len(d) == 84 + n * 50 and n > 0:
        for t in range(n):
            b = 84 + t * 50 + 12
            for k in range(3):
                out.add(struct.unpack_from("<3f", d, b + k * 12))
    else:
        for line in d.decode("ascii", "ignore").splitlines():
            p = line.split()
            if len(p) == 4 and p[0] == "vertex":
                out.add(tuple(float(x) for x in p[1:]))
    return list(out)


def _principal_axis(pts):
    c = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
    cov = [[sum((p[i] - c[i]) * (p[j] - c[j]) for p in pts) for j in range(3)] for i in range(3)]
    v = (1.0, 0.3, 0.1)
    for _ in range(60):                    # power iteration: largest eigenvector
        w = tuple(sum(cov[i][j] * v[j] for j in range(3)) for i in range(3))
        n = math.sqrt(sum(x * x for x in w)) or 1.0
        v = tuple(x / n for x in w)
    return c, v


def _seg_point_dist(a, b, p):
    ab = U._sub(b, a)
    L = U._dot(ab, ab)
    t = 0.0 if L < 1e-18 else max(0.0, min(1.0, U._dot(U._sub(p, a), ab) / L))
    return math.dist(p, (a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]))


def fit_capsule(pts):
    """(a, b, r) in the points' frame, containing every point."""
    c, ax = _principal_axis(pts)
    ts = [U._dot(U._sub(p, c), ax) for p in pts]
    # centre the line on the middle of the perpendicular extent, not the mean
    perp = [U._sub(U._sub(p, c), U._scale(ax, t)) for p, t in zip(pts, ts)]
    mid = [(min(q[i] for q in perp) + max(q[i] for q in perp)) / 2.0 for i in range(3)]
    c = U._add(c, tuple(mid))
    r = max(math.sqrt(max(0.0, U._dot(q, q) - 0.0)) for q in
            (U._sub(U._sub(p, c), U._scale(ax, U._dot(U._sub(p, c), ax))) for p in pts))
    t0, t1 = min(ts), max(ts)
    if t1 - t0 > 2 * r:
        t0, t1 = t0 + r, t1 - r
    else:
        t0 = t1 = (t0 + t1) / 2.0
    a, b = U._add(c, U._scale(ax, t0)), U._add(c, U._scale(ax, t1))
    r = max(_seg_point_dist(a, b, p) for p in pts)     # guarantee containment
    return a, b, r


def _capsule_volume(a, b, r):
    return math.pi * r * r * math.dist(a, b) + 4.0 / 3.0 * math.pi * r ** 3


def fit_capsules(pts, max_k=3):
    """1..max_k capsules covering pts, split into equal slabs along the
    principal axis; the split with the least total volume. One capsule
    round an arm link with sideways joint housings at its ends is 7 cm
    fatter than the tube (FR20 upper arm: r 0.217 vs 0.15)."""
    c, ax = _principal_axis(pts)
    ts = [U._dot(U._sub(p, c), ax) for p in pts]
    t0, t1 = min(ts), max(ts)
    best = None
    for k in range(1, max_k + 1):
        parts = [[] for _ in range(k)]
        for p, t in zip(pts, ts):
            parts[min(k - 1, int((t - t0) / (t1 - t0 + 1e-12) * k))].append(p)
        caps = [fit_capsule(q) for q in parts if len(q) >= 4]
        vol = sum(_capsule_volume(*cp) for cp in caps)
        if best is None or vol < best[0] * 0.97:          # a split must earn its keep
            best = (vol, caps)
    return best[1]


def load_model(robot="fr20", tool_len=0.0, tool_radius=0.04):
    """Capsules per link (link frame) + the chain, cached per robot."""
    key = (robot, tool_len, tool_radius)
    if key in _MODELS:
        return _MODELS[key]
    import robot_profile as RP
    prof = RP.load(robot)
    urdf = prof["rig"].get("urdf")
    if not urdf:
        raise ValueError("%s has no URDF: collision needs link meshes" % robot)
    parsed = U.parse_urdf(os.path.join(ROOT, urdf))
    pkg = os.path.join(ROOT, "assets")
    links = [(parsed["root_link"], -1)] + [(j["child"], i) for i, j in enumerate(parsed["chain"])]
    caps = []
    for name, idx in links:
        mesh = parsed["links"].get(name)
        if not mesh:
            continue
        pts = _stl_vertices(U.resolve_mesh(mesh, pkg))
        for a, b, r in fit_capsules(pts):
            caps.append({"name": name, "joint": idx, "a": a, "b": b, "r": r})
    fo = float(prof["rig"].get("flange_offset_m", 0.0))
    if tool_len > 0:
        caps.append({"name": "tool", "joint": len(parsed["chain"]) - 1,
                     "a": (0.0, 0.0, fo), "b": (0.0, 0.0, fo + tool_len), "r": tool_radius})
    model = {"robot": robot, "chain": parsed["chain"], "caps": caps, "flange_offset": fo,
             "tcp_local": (0.0, 0.0, fo + tool_len)}
    model["pairs"] = _self_pairs(model)
    _MODELS[key] = model
    return model


def capsules(model, q):
    """[(name, a, b, r)] in the base frame for joint angles q (degrees)."""
    fk = U.forward_kinematics(model["chain"], q)
    out = []
    for c in model["caps"]:
        if c["joint"] < 0:
            R, p = U.IDENTITY, (0.0, 0.0, 0.0)
        else:
            R, p = fk[c["joint"]]["link_R"], fk[c["joint"]]["link_p"]
        out.append((c["name"], U._add(p, U._mat_vec(R, c["a"])), U._add(p, U._mat_vec(R, c["b"])), c["r"]))
    last = fk[-1]
    tcp = U._add(last["link_p"], U._mat_vec(last["link_R"], model["tcp_local"]))
    return out, tcp


def _seg_seg_dist(p1, q1, p2, q2):
    """Closest distance between segments (Ericson, Real-Time Collision Detection 5.1.9)."""
    d1, d2, r = U._sub(q1, p1), U._sub(q2, p2), U._sub(p1, p2)
    a, e, f = U._dot(d1, d1), U._dot(d2, d2), U._dot(d2, r)
    if a <= 1e-12 and e <= 1e-12:
        return math.dist(p1, p2)
    if a <= 1e-12:
        s, t = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = U._dot(d1, r)
        if e <= 1e-12:
            t, s = 0.0, max(0.0, min(1.0, -c / a))
        else:
            b = U._dot(d1, d2)
            den = a * e - b * b
            s = max(0.0, min(1.0, (b * f - c * e) / den)) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, max(0.0, min(1.0, (b - c) / a))
    c1 = U._add(p1, U._scale(d1, s))
    c2 = U._add(p2, U._scale(d2, t))
    return math.dist(c1, c2)


def _self_pairs(model):
    """Link pairs to check: not adjacent, not touching at the zero pose."""
    caps0, _ = capsules(model, [0.0] * len(model["chain"]))
    pairs = []
    for i in range(len(caps0)):
        for j in range(i + 1, len(caps0)):
            ji, jj = model["caps"][i]["joint"], model["caps"][j]["joint"]
            if abs(ji - jj) <= 1:              # same link or neighbours (the tool rides on the last link)
                continue
            d = _seg_seg_dist(caps0[i][1], caps0[i][2], caps0[j][1], caps0[j][2]) - caps0[i][3] - caps0[j][3]
            if d > 0.0:
                pairs.append((i, j))
    return pairs


# --------------------------------------------------------------------------
# environment
# --------------------------------------------------------------------------

def load_env(path):
    with open(path) as f:
        env = json.load(f)
    errs = validate_env(env)
    if errs:
        raise ValueError("%s: %s" % (path, "; ".join(errs)))
    return env


def validate_env(env):
    errs = []
    if env.get("schema") != ENV_SCHEMA:
        errs.append("schema %r, expected %r" % (env.get("schema"), ENV_SCHEMA))
    for o in env.get("objects", []):
        n = o.get("name", "?")
        if o.get("role") not in ROLES:
            errs.append("%s: role %r not one of %s" % (n, o.get("role"), ROLES))
        t = o.get("type")
        need = {"box": ("center", "size"), "cylinder": ("center", "radius", "height"),
                "sphere": ("center", "radius"), "halfspace": ("normal", "offset")}.get(t)
        if need is None:
            errs.append("%s: type %r unknown" % (n, t))
            continue
        for k in need:
            if k not in o:
                errs.append("%s: %s needs %s" % (n, t, k))
        if o.get("role") == "slow" and "tcp_speed_mps" not in o:
            errs.append("%s: a slow zone needs tcp_speed_mps" % n)
    return errs


def sdf(o, p):
    """Signed distance from p to shape o (negative inside), metres."""
    t = o["type"]
    if t == "halfspace":
        n = U._normalize(o["normal"])
        return U._dot(n, p) - o["offset"]
    if t == "sphere":
        return math.dist(p, o["center"]) - o["radius"]
    if t == "cylinder":
        c = o["center"]
        dr = math.hypot(p[0] - c[0], p[1] - c[1]) - o["radius"]
        dz = max(c[2] - p[2], p[2] - (c[2] + o["height"]))
        if dr > 0 and dz > 0:
            return math.hypot(dr, dz)
        return max(dr, dz)
    if t == "box":
        c, s = o["center"], o["size"]
        yaw = math.radians(o.get("yaw_deg", 0.0))
        dx, dy, dz = p[0] - c[0], p[1] - c[1], p[2] - c[2]
        cy, sy = math.cos(yaw), math.sin(yaw)
        lx, ly = cy * dx + sy * dy, -sy * dx + cy * dy
        q = (abs(lx) - s[0] / 2.0, abs(ly) - s[1] / 2.0, abs(dz) - s[2] / 2.0)
        out = math.sqrt(sum(max(x, 0.0) ** 2 for x in q))
        return out + min(max(q), 0.0)
    raise ValueError(t)


def capsule_distance(o, a, b, r):
    """Signed clearance between a capsule and a shape (negative = overlap)."""
    f = lambda s: sdf(o, (a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2])))
    lo, hi = 0.0, 1.0
    for _ in range(40):                    # sdf of a convex set is convex along a line
        m1, m2 = lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0
        if f(m1) < f(m2):
            hi = m2
        else:
            lo = m1
    return min(f(0.0), f(1.0), f((lo + hi) / 2.0)) - r


# --------------------------------------------------------------------------
# checking a trajectory
# --------------------------------------------------------------------------

def check(model, env, times, joints, max_violations=20):
    """Collision / zone report for a trajectory (times s, joints deg).

    ok is False on any violation. min_clearance / closest: the smallest link
    clearance to an obstacle or keep_out shape, or between two links."""
    margin = float(env.get("margin_m", 0.05))
    objs = env.get("objects", [])
    hard = [o for o in objs if o["role"] in ("obstacle", "keep_out")]
    slow = [o for o in objs if o["role"] == "slow"]
    work = [o for o in objs if o["role"] == "work"]
    viol, best = [], (math.inf, None)
    prev_tcp = None
    for f, (t, q) in enumerate(zip(times, joints)):
        caps, tcp = capsules(model, q)
        for o in hard:
            for name, a, b, r in caps:
                if name == U_BASE:
                    continue                # the base is bolted down: what it stands on is its mount
                d = capsule_distance(o, a, b, r)
                if d < best[0]:
                    best = (d, {"frame": f, "t": round(t, 4), "link": name, "with": o["name"]})
                limit = margin if o["role"] == "obstacle" else 0.0
                if d < limit and len(viol) < max_violations:
                    viol.append({"frame": f, "t": round(t, 4), "kind": o["role"], "link": name,
                                 "with": o["name"], "clearance_m": round(d, 4)})
        for i, j in model["pairs"]:
            d = _seg_seg_dist(caps[i][1], caps[i][2], caps[j][1], caps[j][2]) - caps[i][3] - caps[j][3]
            if d < best[0]:
                best = (d, {"frame": f, "t": round(t, 4), "link": caps[i][0], "with": caps[j][0]})
            if d < 0.0 and len(viol) < max_violations:
                viol.append({"frame": f, "t": round(t, 4), "kind": "self", "link": caps[i][0],
                             "with": caps[j][0], "clearance_m": round(d, 4)})
        if prev_tcp is not None and f > 0:
            dt = t - times[f - 1]
            v = math.dist(tcp, prev_tcp) / dt if dt > 0 else 0.0
            for o in slow:
                if sdf(o, tcp) < 0 and v > o["tcp_speed_mps"] * 1.001 and len(viol) < max_violations:
                    viol.append({"frame": f, "t": round(t, 4), "kind": "slow", "link": "tcp", "with": o["name"],
                                 "speed_mps": round(v, 3), "limit_mps": o["tcp_speed_mps"]})
        for o in work:
            if sdf(o, tcp) > 0 and len(viol) < max_violations:
                viol.append({"frame": f, "t": round(t, 4), "kind": "work", "link": "tcp", "with": o["name"],
                             "outside_m": round(sdf(o, tcp), 4)})
        prev_tcp = tcp
    return {"ok": not viol, "min_clearance_m": round(best[0], 4) if best[1] else None,
            "closest": best[1], "violations": viol, "margin_m": margin}


U_BASE = "base_link"


def describe(report):
    """One line for Pre-Flight / manifests."""
    if report["ok"]:
        c = report["closest"] or {}
        return "clear: nearest %.0f mm (%s to %s, frame %s)" % (
            report["min_clearance_m"] * 1000, c.get("link"), c.get("with"), c.get("frame"))
    v = report["violations"][0]
    if v["kind"] == "slow":
        return "TCP %.2f m/s in slow zone %s (limit %.2f) at frame %d" % (v["speed_mps"], v["with"], v["limit_mps"], v["frame"])
    if v["kind"] == "work":
        return "TCP %.0f mm outside work zone %s at frame %d" % (v["outside_m"] * 1000, v["with"], v["frame"])
    return "%s: %s %s %s (%.0f mm) at frame %d" % (
        v["kind"], v["link"], "inside" if v["clearance_m"] < 0 else "within margin of", v["with"],
        v["clearance_m"] * 1000, v["frame"])


if __name__ == "__main__":
    import random
    import time

    fails = []

    def ok(label, cond, detail=""):
        print("%s  %s%s" % ("ok  " if cond else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not cond:
            fails.append(label)

    t0 = time.time()
    m = load_model("fr20")
    ok("FR20 capsules fitted from the URDF meshes", len({c["name"] for c in m["caps"]}) == 7,
       ", ".join("%s r=%.3f L=%.3f" % (c["name"], c["r"], math.dist(c["a"], c["b"])) for c in m["caps"])
       + "  (%.1f s)" % (time.time() - t0))
    # containment: every vertex of every mesh inside its capsule
    parsed = U.parse_urdf(os.path.join(ROOT, "assets/fairino_description/urdf/fairino20_v6.urdf"))
    worst = 0.0
    for name in {c["name"] for c in m["caps"]}:
        pts = _stl_vertices(U.resolve_mesh(parsed["links"][name], os.path.join(ROOT, "assets")))
        mine = [c for c in m["caps"] if c["name"] == name]
        worst = max(worst, max(min(_seg_point_dist(c["a"], c["b"], p) - c["r"] for c in mine) for p in pts))
    ok("every mesh vertex lies inside its capsule", worst <= 1e-9, "worst %.2e m outside" % worst)
    ok("self-collision pairs exclude neighbours", all(abs(m["caps"][i]["joint"] - m["caps"][j]["joint"]) > 1
                                                      for i, j in m["pairs"]), "%d pairs" % len(m["pairs"]))

    floor = {"schema": ENV_SCHEMA, "margin_m": 0.05,
             "objects": [{"name": "floor", "type": "halfspace", "normal": [0, 0, 1], "offset": -0.02, "role": "obstacle"}]}
    READY = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    r = check(m, floor, [0.0], [READY])
    ok("the ready pose clears the floor", r["ok"], describe(r))
    # reach down: shoulder lifted past horizontal, the arm goes through the floor
    r = check(m, floor, [0.0], [[0.0, 30.0, 10.0, -90.0, -90.0, 0.0]])
    ok("an arm reaching below the base hits the floor", not r["ok"] and r["violations"][0]["with"] == "floor", describe(r))
    # folding the elbow shut
    r = check(m, floor, [0.0], [[0.0, -90.0, 160.0, -90.0, -90.0, 0.0]])
    ok("folding the elbow shut is a self-collision", any(v["kind"] == "self" for v in r["violations"]), describe(r))
    # a box where the tool is
    caps, tcp = capsules(m, READY)
    box = dict(floor, objects=floor["objects"] + [{"name": "crate", "type": "box", "center": list(tcp),
                                                   "size": [0.2, 0.2, 0.2], "role": "obstacle"}])
    r = check(m, box, [0.0], [READY])
    ok("a box at the TCP is a collision with the wrist", not r["ok"] and any(
        v["with"] == "crate" and v["link"].startswith("wrist") for v in r["violations"]), describe(r))
    # capsule distance vs brute force on random boxes / capsules
    random.seed(4)
    err = 0.0
    for _ in range(200):
        o = {"type": "box", "center": [random.uniform(-1, 1) for _ in range(3)],
             "size": [random.uniform(0.1, 0.8) for _ in range(3)], "yaw_deg": random.uniform(0, 90)}
        a = tuple(random.uniform(-1.5, 1.5) for _ in range(3))
        b = tuple(random.uniform(-1.5, 1.5) for _ in range(3))
        exact = capsule_distance(o, a, b, 0.1)
        brute = min(sdf(o, tuple(a[k] + s / 2000.0 * (b[k] - a[k]) for k in range(3))) for s in range(2001)) - 0.1
        err = max(err, abs(exact - brute))
    ok("capsule-box clearance matches brute force", err < 1e-3, "worst %.1e m over 200 cases" % err)
    # slow zone: fast TCP inside is flagged, slow is not
    zone = {"name": "near_operator", "type": "sphere", "center": list(tcp), "radius": 0.3, "role": "slow", "tcp_speed_mps": 0.25}
    envz = dict(floor, objects=floor["objects"] + [zone])
    q2 = list(READY); q2[0] += 5.0                   # J1 5 deg: TCP moves ~7 cm
    r_fast = check(m, envz, [0.0, 0.1], [READY, q2])
    r_slow = check(m, envz, [0.0, 2.0], [READY, q2])
    ok("TCP over the speed limit inside a slow zone is flagged; under it is not",
       any(v["kind"] == "slow" for v in r_fast["violations"]) and r_slow["ok"],
       describe(r_fast) + " / " + describe(r_slow))
    workz = dict(floor, objects=floor["objects"] + [{"name": "stage", "type": "box", "center": [3, 0, 1], "size": [1, 1, 1], "role": "work"}])
    r = check(m, workz, [0.0], [READY])
    ok("TCP outside the work zone is flagged", not r["ok"] and r["violations"][0]["kind"] == "work", describe(r))
    env_path = os.path.join(ROOT, "envs", "volvox_lab.json")
    if os.path.exists(env_path):
        env = load_env(env_path)
        r = check(m, env, [0.0], [READY])
        ok("envs/volvox_lab.json loads, and the ready pose is clear in it", r["ok"], describe(r))

    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
