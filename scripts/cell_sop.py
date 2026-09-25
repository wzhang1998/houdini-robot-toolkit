"""Houdini side of the cell (scenes/FR20_cell.hiplc): the clip, the room, the
robot's collision capsules. Imported through the scene's hou.session, so
parameter expressions can call it: hou.session.joint(3).

    clip      a joint CSV (the asset's export) or a clip JSON (the factories),
              from /obj/CELL_CTRL's Clip; sampled at the current frame, 24 fps
    env_geo   Python SOP: the environment file as geometry, coloured by role
              (obstacle grey, keep_out red, slow orange, work green; zones
              see-through)
    capsule_geo  Python SOP: the robot's collision capsules at this frame,
              each coloured by its clearance (green far .. red at the margin),
              detail attributes min_clearance_m / nearest / violations
    fit_range the playbar to the clip

Frames: the env file and collision.py work in the robot base frame (URDF,
Z up); Houdini is Y up -- (x, y, z) -> (x, z, -y), as urdf_rig does.
"""

import json
import math
import os
import sys

import hou

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import collision as CL  # noqa: E402

FPS = 24.0
_CACHE = {}
ROLE_COLOUR = {"obstacle": (0.55, 0.57, 0.6), "keep_out": (0.95, 0.15, 0.1),
               "slow": (1.0, 0.6, 0.1), "work": (0.2, 0.85, 0.35)}
ROLE_ALPHA = {"obstacle": 1.0, "keep_out": 0.35, "slow": 0.12, "work": 0.06}


def _ctrl():
    return hou.node("/obj/CELL_CTRL")


def _h(p):
    return (p[0], p[2], -p[1])


def clip():
    """(times, joints) of the clip named on CELL_CTRL, cached by file time."""
    path = _ctrl().evalParm("clip")
    key = (path, os.path.getmtime(path) if os.path.exists(path) else 0)
    if _CACHE.get("clip_key") != key:
        if path.lower().endswith(".json"):
            with open(path) as f:
                c = json.load(f)
            t = [p["t"] for p in c["points"]]
            q = [p["q"] for p in c["points"]]
        else:
            import fairino_player
            t, q = fairino_player.load_csv(path)
        _CACHE["clip_key"], _CACHE["clip"] = key, (t, q)
    return _CACHE["clip"]


def q_at(frame=None):
    t, q = clip()
    if not t:
        return [0.0] * 6
    s = ((frame if frame is not None else hou.frame()) - 1.0) / FPS
    if s <= t[0]:
        return list(q[0])
    if s >= t[-1]:
        return list(q[-1])
    lo, hi = 0, len(t) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if t[mid] <= s:
            lo = mid
        else:
            hi = mid
    f = (s - t[lo]) / (t[hi] - t[lo])
    return [a + f * (b - a) for a, b in zip(q[lo], q[hi])]


def joint(j):
    """Robot-frame degrees of joint j (1-6) at the current frame."""
    return q_at()[j - 1]


def fit_range():
    t, _ = clip()
    end = 1 + int(math.ceil(t[-1] * FPS))
    hou.playbar.setFrameRange(1, end)
    hou.playbar.setPlaybackRange(1, end)
    hou.setFrame(1)


def env():
    path = _ctrl().evalParm("env")
    key = (path, os.path.getmtime(path) if os.path.exists(path) else 0)
    if _CACHE.get("env_key") != key:
        _CACHE["env_key"], _CACHE["env"] = key, CL.load_env(path)
    return _CACHE["env"]


def _model():
    tl = _ctrl().evalParm("tool_len")
    return CL.load_model("fr20", tool_len=round(tl, 3))


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------

def _box(geo, center, size, yaw, cd, alpha, name):
    c, s = center, size
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    corners = []
    for dz in (-1, 1):
        for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            lx, ly = dx * s[0] / 2, dy * s[1] / 2
            corners.append((c[0] + cy * lx - sy * ly, c[1] + sy * lx + cy * ly, c[2] + dz * s[2] / 2))
    pts = [geo.createPoint() for _ in corners]
    for p, x in zip(pts, corners):
        p.setPosition(_h(x))
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    for f in faces:
        poly = geo.createPolygon()
        for i in f:
            poly.addVertex(pts[i])
        _prim_attrs(geo, poly, cd, alpha, name)


def _cylinder(geo, center, radius, height, cd, alpha, name, sides=24):
    rings = []
    for z in (center[2], center[2] + height):
        ring = []
        for k in range(sides):
            a = 2 * math.pi * k / sides
            p = geo.createPoint()
            p.setPosition(_h((center[0] + radius * math.cos(a), center[1] + radius * math.sin(a), z)))
            ring.append(p)
        rings.append(ring)
    for k in range(sides):
        poly = geo.createPolygon()
        for p in (rings[0][k], rings[0][(k + 1) % sides], rings[1][(k + 1) % sides], rings[1][k]):
            poly.addVertex(p)
        _prim_attrs(geo, poly, cd, alpha, name)
    for ring, rev in ((rings[0], True), (rings[1], False)):
        poly = geo.createPolygon()
        for p in (reversed(ring) if rev else ring):
            poly.addVertex(p)
        _prim_attrs(geo, poly, cd, alpha, name)


def _prim_attrs(geo, prim, cd, alpha, name):
    prim.setAttribValue("Cd", cd)
    prim.setAttribValue("Alpha", alpha)
    prim.setAttribValue("name", name)


def env_geo(node):
    geo = node.geometry()
    geo.clear()
    geo.addAttrib(hou.attribType.Prim, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Prim, "Alpha", 1.0)
    geo.addAttrib(hou.attribType.Prim, "name", "")
    geo.addAttrib(hou.attribType.Prim, "role", "")
    for o in env().get("objects", []):
        cd, a = ROLE_COLOUR[o["role"]], ROLE_ALPHA[o["role"]]
        tall = (o.get("size") or [0, 0, 0])[2] > 1.5 or o.get("height", 0) > 1.5
        if o["role"] == "obstacle" and tall:
            a = 0.22                                          # walls and shelves: see-through, so the robot shows
        n0 = len(geo.prims())
        if o["type"] == "box":
            _box(geo, o["center"], o["size"], o.get("yaw_deg", 0.0), cd, a, o["name"])
        elif o["type"] == "cylinder":
            _cylinder(geo, o["center"], o["radius"], o["height"], cd, a, o["name"])
        elif o["type"] == "sphere":
            c, r = o["center"], o["radius"]
            _cylinder(geo, (c[0], c[1], c[2] - r), r, 2 * r, cd, a, o["name"])
        elif o["type"] == "halfspace":
            n = CL.U._normalize(o["normal"])
            if abs(n[2]) > 0.9 and n[2] > 0:                 # a floor: a slab under the plane
                _box(geo, (0.0, 0.0, o["offset"] - 0.01), (6.0, 6.0, 0.02), 0.0, (0.35, 0.3, 0.26), 1.0, o["name"])
            # walls / ceilings given as planes are left out of the picture
        for pr in geo.prims()[n0:]:
            pr.setAttribValue("role", o["role"])


def _capsule_mesh(geo, a, b, r, cd, name, sides=14, cap_rings=4):
    ax = CL.U._sub(b, a)
    L = math.sqrt(CL.U._dot(ax, ax))
    z = CL.U._normalize(ax) if L > 1e-9 else (0.0, 0.0, 1.0)
    ref = (1.0, 0.0, 0.0) if abs(z[0]) < 0.9 else (0.0, 1.0, 0.0)
    x = CL.U._normalize(CL.U._cross(ref, z))
    y = CL.U._cross(z, x)
    rings = []
    # bottom hemisphere, cylinder, top hemisphere: rings of (centre offset along z, radius)
    prof = [(-r * math.cos(math.pi / 2 * k / cap_rings), r * math.sin(math.pi / 2 * k / cap_rings))
            for k in range(cap_rings + 1)]
    prof += [(L + r * math.sin(math.pi / 2 * k / cap_rings), r * math.cos(math.pi / 2 * k / cap_rings))
             for k in range(cap_rings + 1)]
    for h, rr in prof:
        ring = []
        for k in range(sides):
            t = 2 * math.pi * k / sides
            p = tuple(a[i] + z[i] * h + rr * (math.cos(t) * x[i] + math.sin(t) * y[i]) for i in range(3))
            pt = geo.createPoint()
            pt.setPosition(_h(p))
            ring.append(pt)
        rings.append(ring)
    for r0, r1 in zip(rings, rings[1:]):
        for k in range(sides):
            poly = geo.createPolygon()
            for p in (r0[k], r0[(k + 1) % sides], r1[(k + 1) % sides], r1[k]):
                poly.addVertex(p)
            poly.setAttribValue("Cd", cd)
            poly.setAttribValue("name", name)


def _clear_colour(d, margin):
    t = max(0.0, min(1.0, (d - margin) / 0.4))
    c = hou.Color()
    c.setHSV((120.0 * t, 0.85, 1.0))
    return c.rgb()


def capsule_geo(node, frame=None):
    geo = node.geometry()
    geo.clear()
    geo.addAttrib(hou.attribType.Prim, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Prim, "name", "")
    model = _model()
    cell = env()
    margin = float(cell.get("margin_m", 0.05))
    q = q_at(frame)
    caps, tcp = CL.capsules(model, q)
    hard = [o for o in cell.get("objects", []) if o["role"] in ("obstacle", "keep_out")]
    worst, nearest = math.inf, ""
    for name, a, b, r in caps:
        if name in CL.FIXED_LINKS:
            d = math.inf
        else:
            d = min([CL.capsule_distance(o, a, b, r) for o in hard] or [math.inf])
            for o in hard:
                dd = CL.capsule_distance(o, a, b, r)
                if dd < worst:
                    worst, nearest = dd, "%s to %s" % (name, o["name"])
        _capsule_mesh(geo, a, b, r, _clear_colour(d, margin) if d < math.inf else (0.7, 0.7, 0.75), name)
    geo.addAttrib(hou.attribType.Global, "min_clearance_m", 0.0)
    geo.setGlobalAttribValue("min_clearance_m", worst if worst < math.inf else -1.0)
    geo.addAttrib(hou.attribType.Global, "nearest", "")
    geo.setGlobalAttribValue("nearest", nearest)


def ghost_geo(node):
    """Onion skin: the capsules at Count evenly spaced frames of the clip,
    coloured by time (blue = start .. red = end), and the TCP path."""
    geo = node.geometry()
    geo.clear()
    geo.addAttrib(hou.attribType.Prim, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Prim, "name", "")
    n = max(2, node.evalParm("count"))
    t, q = clip()
    model = _model()
    last = 1 + int(round(t[-1] * FPS))
    for k in range(n):
        fr = 1 + (last - 1) * k / float(n - 1)
        c = hou.Color()
        c.setHSV((240.0 * (1 - k / float(n - 1)), 0.75, 1.0))
        caps, _ = CL.capsules(model, q_at(fr))
        for name, a, b, r in caps:
            if name != CL.U_BASE or k == 0:
                _capsule_mesh(geo, a, b, r, c.rgb(), name, sides=10, cap_rings=2)
    poly = geo.createPolygon(is_closed=False)
    for fr in range(1, last + 1):
        _, tcp = CL.capsules(model, q_at(fr))
        p = geo.createPoint()
        p.setPosition(_h(tcp))
        poly.addVertex(p)
    poly.setAttribValue("Cd", (1.0, 1.0, 1.0))
    poly.setAttribValue("name", "tcp_path")
