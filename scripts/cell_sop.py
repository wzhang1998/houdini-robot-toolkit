"""Houdini side of the cell (scenes/FR20_cell.hiplc): the clip, the room, the
robot's collision capsules. Imported through the scene's hou.session, so
parameter expressions can call it: hou.session.joint(3).

    clip      a joint CSV (the asset's export) or a clip JSON (the factories),
              from /obj/CELL_CTRL's Clip; sampled at the current frame, 24 fps
    env_geo   Python SOP: the environment file as geometry, drawn like the
              Isaac scene (room_geom's looks): floor and objects solid, walls
              one face turned in (a cutaway with Remove Backfaces), zones as
              outlines coloured by role
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
    if not t:                                  # no motion (a clip rejected before any): HOME, not zero (flat)
        import robot_profile
        return list(robot_profile.home(robot_profile.load("fr20")))
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

def _outward(corners, center):
    """Houdini-frame direction from a solid's centre to a face's centre."""
    m = [sum(p[i] for p in corners) / len(corners) for i in range(3)]
    return _h([m[i] - center[i] for i in range(3)])


def _box(geo, center, size, yaw, cd, alpha, name):
    c, s = center, size
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    corners = []
    for dz in (-1, 1):
        for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            lx, ly = dx * s[0] / 2, dy * s[1] / 2
            corners.append((c[0] + cy * lx - sy * ly, c[1] + sy * lx + cy * ly, c[2] + dz * s[2] / 2))
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    for f in faces:
        fc = [corners[i] for i in f]
        poly = _face(geo, fc, cd, name, facing=_outward(fc, c))
        poly.setAttribValue("Alpha", alpha)


def _cylinder(geo, center, radius, height, cd, alpha, name, sides=24):
    mid = (center[0], center[1], center[2] + height / 2.0)
    rings = [[(center[0] + radius * math.cos(2 * math.pi * k / sides),
               center[1] + radius * math.sin(2 * math.pi * k / sides), z) for k in range(sides)]
             for z in (center[2], center[2] + height)]
    faces = [[rings[0][k], rings[0][(k + 1) % sides], rings[1][(k + 1) % sides], rings[1][k]]
             for k in range(sides)] + rings
    for fc in faces:
        poly = _face(geo, fc, cd, name, facing=_outward(fc, mid))
        poly.setAttribValue("Alpha", alpha)


def _polyline(geo, xs, cd, name, closed=False):
    """An opaque line (own points, so promoting Cd / Alpha to points does
    not blend it with a see-through face). A loop repeats its first point:
    a closed polygon would draw as a solid face."""
    xs = list(xs) + ([xs[0]] if closed else [])
    poly = geo.createPolygon(is_closed=False)
    for x in xs:
        p = geo.createPoint()
        p.setPosition(_h(x))
        poly.addVertex(p)
    _prim_attrs(geo, poly, cd, 1.0, name)


def _prim_attrs(geo, prim, cd, alpha, name):
    prim.setAttribValue("Cd", cd)
    prim.setAttribValue("Alpha", alpha)
    prim.setAttribValue("name", name)


def env_geo(node):
    env_geometry(node.geometry(), env())


def env_geometry(geo, cell, walls=True):
    """Draw an environment (collision.load_env) into geo: prim Cd / Alpha /
    name / role. Also used by the robot_arm asset's Show Cell toggle.

    The same look as the Isaac scene (room_geom's LOOKS and rules): floor and
    objects solid, each wall one face turned into the room -- with the
    viewport's Remove Backfaces on, the walls near the camera vanish (a
    cutaway; everything else faces out, so only walls vanish) -- and zones
    as outlines. walls=False leaves
    the walls out (renders from outside the room)."""
    import room_geom as RG
    geo.clear()
    geo.addAttrib(hou.attribType.Prim, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Prim, "Alpha", 1.0)
    geo.addAttrib(hou.attribType.Prim, "name", "")
    geo.addAttrib(hou.attribType.Prim, "role", "")
    fp = RG.footprint(cell)
    for o in cell.get("objects", []):
        n0 = len(geo.prims())
        if o["type"] == "halfspace":
            continue                                          # the room's planes: drawn closed, below
        if o["role"] != "obstacle":
            # a zone is a volume the checks use, not a thing in the room
            for line in RG.zone_lines(o, fp):
                _polyline(geo, line, RG.ZONE_RGB.get(o["role"], (1.0, 1.0, 1.0)), o["name"])
        else:
            cd = RG.LOOKS[RG.look_key(o["name"])][0]
            if o["type"] == "box":
                _box(geo, o["center"], o["size"], o.get("yaw_deg", 0.0), cd, 1.0, o["name"])
            elif o["type"] == "cylinder":
                _cylinder(geo, o["center"], o["radius"], o["height"], cd, 1.0, o["name"])
            elif o["type"] == "sphere":
                c, r = o["center"], o["radius"]
                _cylinder(geo, (c[0], c[1], c[2] - r), r, 2 * r, cd, 1.0, o["name"])
        for pr in geo.prims()[n0:]:
            pr.setAttribValue("role", o["role"])
    _room(geo, cell, walls)


def _face(geo, corners, cd, name, facing=None):
    """A polygon over corners (robot frame); turned so its normal points
    along `facing` (Houdini frame) when given."""
    poly = geo.createPolygon()
    pts = []
    for c in corners:
        pt = geo.createPoint()
        pt.setPosition(_h(c))
        poly.addVertex(pt)
        pts.append(pt)
    if facing is not None and poly.normal().dot(hou.Vector3(facing)) < 0.0:
        geo.deletePrims([poly], keep_points=False)
        return _face(geo, list(reversed(corners)), cd, name)
    _prim_attrs(geo, poly, cd, 1.0, name)
    return poly


def _room(geo, cell, walls=True):
    """Floor, walls and ceiling as a closed room (room_geom.py): the floor
    solid, each wall one face turned into the room, the ceiling grid's
    height as an outline."""
    import room_geom as RG
    fp = RG.footprint(cell)
    z0, z1 = RG.heights(cell)
    n0 = len(geo.prims())
    _face(geo, [(x, y, z0) for x, y in fp], RG.LOOKS["floor"][0], "floor", facing=(0.0, 1.0, 0.0))
    if walls:
        for name, corners, outward in RG.walls(cell):
            inward = _h([-x for x in outward])
            _face(geo, corners, RG.LOOKS[RG.wall_look(name)][0], name, facing=inward)
    _polyline(geo, [(x, y, z1) for x, y in fp], RG.ZONE_RGB["ceiling"], "ceiling", closed=True)
    for pr in geo.prims()[n0:]:
        pr.setAttribValue("role", "obstacle")


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
        slack = math.inf                      # clearance beyond each object's own margin
        if name not in CL.FIXED_LINKS:
            for o in hard:
                dd = CL.capsule_distance(o, a, b, r)
                m = o.get("margin_m", margin) if o["role"] == "obstacle" else 0.0
                slack = min(slack, dd - m)
                if dd < worst:
                    worst, nearest = dd, "%s to %s" % (name, o["name"])
        _capsule_mesh(geo, a, b, r, _clear_colour(slack, 0.0) if slack < math.inf else (0.7, 0.7, 0.75), name)
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
