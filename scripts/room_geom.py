"""The room as closed, finite geometry, from the env's measured planes.

The room (envs/*.usda, read by room_usd.py into collision.py's dict) stores walls, floor and ceiling as
halfspaces: infinite planes, which is right for collision checks and wrong
for pictures (drawn as planes they cut through each other). Here they are
turned into the room itself:

    footprint(env)   the floor outline: the vertical walls' half-planes
                     intersected (convex polygon, counter-clockwise, robot
                     frame XY), clipped to a limit square where a side is open
    heights(env)     floor and ceiling z (the horizontal halfspaces)
    walls(env)       one panel per wall: its edge of the footprint, floor to
                     ceiling -- (name, [4 corners], outward normal)
    edges(obj)       the 12 edges of a box / the rings of a cylinder, for
                     drawing a zone (work / keep-out / slow) as an outline
    zone_lines(o, fp)  how a zone is drawn: its outline, up to the walls

How the room looks is also here -- LOOKS (colour, roughness, metal,
opacity by name), look_key(name), SOLID_WALLS, ZONE_RGB -- so Isaac and
Houdini draw it alike: floor and objects solid, each wall one face turned
into the room (hidden from outside: a cutaway), zones as outlines.

Used by the Houdini cell display (cell_sop.py) and the USD export
(room_usd.py), so both show the same room. Pure Python.

    python scripts/room_geom.py        self-test
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

OPEN_HALF = 3.0          # where no wall closes a side, the room stops this far from the base

SOLID_WALLS = ("wall_tv",)          # every other measured wall is glass
# name keyword -> (diffuse rgb, roughness, metallic, opacity)
LOOKS = {
    "floor": ((0.36, 0.24, 0.15), 0.55, 0.0, 1.0),
    "wall": ((0.86, 0.85, 0.82), 0.9, 0.0, 1.0),
    "glass": ((0.8, 0.9, 0.95), 0.0, 0.0, 0.08),
    "glass_face": ((0.78, 0.86, 0.9), 0.15, 0.0, 1.0),   # a glass wall seen from inside: pale blue-grey, opaque
    "tv": ((0.02, 0.02, 0.025), 0.2, 0.0, 1.0),
    "cart": ((0.62, 0.08, 0.06), 0.45, 0.2, 1.0),
    "shelves": ((0.28, 0.28, 0.3), 0.4, 0.7, 1.0),
    "furniture": ((0.55, 0.4, 0.28), 0.6, 0.0, 1.0),
    "plant": ((0.2, 0.42, 0.18), 0.8, 0.0, 1.0),
    "base_plate": ((0.78, 0.64, 0.45), 0.7, 0.0, 1.0),
    "canvas": ((0.96, 0.96, 0.93), 0.9, 0.0, 1.0),
    "wood": ((0.62, 0.43, 0.24), 0.65, 0.0, 1.0),
    "obstacle": ((0.6, 0.6, 0.62), 0.7, 0.0, 1.0),
}
ZONE_RGB = {"work": (0.2, 0.85, 0.35), "keep_out": (0.95, 0.2, 0.15), "slow": (1.0, 0.65, 0.1),
            "ceiling": (0.7, 0.72, 0.75)}


def look_key(name):
    """The LOOKS entry for an object, by its name."""
    n = name.lower()
    if n.startswith("frame"):                     # the paper's frame (show.canvas_parts)
        return "wood"
    for k in ("glass", "canvas", "base_plate", "cart", "tv", "shelves", "furniture", "plant", "floor"):
        if k in n:
            return k
    if n.startswith("wall") or "partition" in n:
        return "wall"
    return "obstacle"


def wall_look(name):
    """The LOOKS entry for a wall's inward face."""
    return "wall" if name in SOLID_WALLS else "glass_face"


def _norm2(n):
    L = math.hypot(n[0], n[1])
    return (n[0] / L, n[1] / L)


def vertical_walls(env):
    return [o for o in env["objects"] if o["type"] == "halfspace" and abs(o["normal"][2]) < 0.1]


def heights(env):
    """(floor z, ceiling z): z where the horizontal halfspaces' planes are."""
    floor, ceil = 0.0, 2.7
    for o in env["objects"]:
        if o["type"] != "halfspace" or abs(o["normal"][2]) < 0.9:
            continue
        L = math.sqrt(sum(x * x for x in o["normal"]))
        nz = o["normal"][2] / L
        z = (o["offset"] / L) / nz
        if nz > 0:
            floor = z
        else:
            ceil = z
    return floor, ceil


def _clip(poly, n, d):
    """Keep the part of poly where n . p >= d (Sutherland-Hodgman)."""
    out = []
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        fa, fb = n[0] * a[0] + n[1] * a[1] - d, n[0] * b[0] + n[1] * b[1] - d
        if fa >= 0:
            out.append(a)
        if (fa >= 0) != (fb >= 0):
            t = fa / (fa - fb)
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def footprint(env, half=OPEN_HALF):
    """The floor outline [(x, y)], counter-clockwise: free space n.p >= offset
    for every vertical wall (collision.py's sign: sdf = n.p - offset)."""
    poly = [(-half, -half), (half, -half), (half, half), (-half, half)]
    for o in vertical_walls(env):
        L = math.hypot(o["normal"][0], o["normal"][1])
        poly = _clip(poly, _norm2(o["normal"]), o["offset"] / L)
    return poly


def walls(env, half=OPEN_HALF):
    """[(name, corners, outward normal)]: each wall's piece of the footprint's
    boundary, floor to ceiling. Outward = into the wall, away from the room."""
    fp = footprint(env, half)
    z0, z1 = heights(env)
    out = []
    for o in vertical_walls(env):
        L = math.hypot(o["normal"][0], o["normal"][1])
        n, d = _norm2(o["normal"]), o["offset"] / L
        on = [p for p in fp if abs(n[0] * p[0] + n[1] * p[1] - d) < 1e-6]
        if len(on) < 2:
            continue                                         # the wall does not bound the room
        a, b = on[0], on[-1]
        out.append((o["name"], [(a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1), (a[0], a[1], z1)],
                    (-n[0], -n[1], 0.0)))
    return out


def box_corners(center, size, yaw_deg):
    c, s = center, size
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [[(c[0] + cy * dx * s[0] / 2 - sy * dy * s[1] / 2, c[1] + sy * dx * s[0] / 2 + cy * dy * s[1] / 2,
              c[2] + dz * s[2] / 2) for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1))] for dz in (-1, 1)]


def edges(o, sides=48, uprights=8):
    """Polylines outlining a box or cylinder zone: [[(x, y, z), ...], ...]."""
    if o["type"] == "box":
        lo, hi = box_corners(o["center"], o["size"], o.get("yaw_deg", 0.0))
        return [lo + lo[:1], hi + hi[:1]] + [[a, b] for a, b in zip(lo, hi)]
    if o["type"] == "cylinder":
        c, r, h = o["center"], o["radius"], o["height"]
        at = lambda a, z: (c[0] + r * math.cos(a), c[1] + r * math.sin(a), z)
        rings = [[at(2 * math.pi * k / sides, z) for k in range(sides + 1)] for z in (c[2], c[2] + h)]
        return rings + [[at(2 * math.pi * k / uprights, c[2]), at(2 * math.pi * k / uprights, c[2] + h)]
                        for k in range(uprights)]
    raise ValueError("no outline for %r" % o["type"])


def zone_lines(o, fp):
    """A zone as drawn: its outline (edges), clipped to the footprint."""
    return clip_polylines(edges(o), fp)


def inside(fp, p, eps=1e-9):
    """Is p (x, y[, z]) inside the convex counter-clockwise footprint fp?"""
    return all((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= -eps
               for a, b in zip(fp, fp[1:] + fp[:1]))


def clip_polylines(polylines, fp):
    """The parts of polylines inside the footprint: a zone drawn up to the
    walls, not through them (split where a point falls outside)."""
    out = []
    for line in polylines:
        run = []
        for p in line:
            if inside(fp, p):
                run.append(p)
            else:
                if len(run) > 1:
                    out.append(run)
                run = []
        if len(run) > 1:
            out.append(run)
    return out


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    box = {"objects": [
        {"name": "floor", "type": "halfspace", "normal": [0, 0, 1], "offset": -0.01},
        {"name": "ceiling", "type": "halfspace", "normal": [0, 0, -1], "offset": -2.2},
        {"name": "w_front", "type": "halfspace", "normal": [1, 0, 0], "offset": -1.5},     # wall at x = -1.5
        {"name": "w_back", "type": "halfspace", "normal": [-1, 0, 0], "offset": -2.0},    # wall at x = +2.0
        {"name": "w_left", "type": "halfspace", "normal": [0, 1, 0], "offset": -1.0}]}    # wall at y = -1.0
    fp = footprint(box)
    xs, ys = [p[0] for p in fp], [p[1] for p in fp]
    check("footprint: the walls bound x and y, an open side stops at the limit",
          abs(min(xs) + 1.5) < 1e-9 and abs(max(xs) - 2.0) < 1e-9 and abs(min(ys) + 1.0) < 1e-9 and abs(max(ys) - OPEN_HALF) < 1e-9, fp)
    check("heights: floor and ceiling from the horizontal planes", heights(box) == (-0.01, 2.2), heights(box))
    ws = {n: (c, nn) for n, c, nn in walls(box)}
    c, nn = ws["w_front"]
    check("a wall panel spans its edge of the room, floor to ceiling, facing out",
          all(abs(p[0] + 1.5) < 1e-9 for p in c) and {p[2] for p in c} == {-0.01, 2.2} and nn == (-1.0, 0.0, 0.0), c)
    check("panels do not run past the room (no crossing walls)",
          all(min(xs) - 1e-9 <= p[0] <= max(xs) + 1e-9 and min(ys) - 1e-9 <= p[1] <= max(ys) + 1e-9
              for _, c, _ in walls(box) for p in c))
    import json
    import collision as CL
    env = CL.load_env(os.path.join(os.path.dirname(HERE), "envs", "volvox_lab.usda"))
    fp = footprint(env)
    check("the lab: four walls make a closed quad", len(fp) == 4 and len(walls(env)) == 4, (len(fp), [w[0] for w in walls(env)]))
    area = 0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(fp, fp[1:] + fp[:1])))
    check("the lab's floor area is room-sized", 8.0 < area < 25.0, round(area, 2))
    check("the base is inside the room", all(o["normal"][0] * 0 + o["normal"][1] * 0 - o["offset"] > 0 for o in vertical_walls(env)))
    ring = edges({"type": "cylinder", "center": [1.9, 0.0, 0.0], "radius": 1.0, "height": 1.0})
    clipped = clip_polylines(ring, footprint(box))
    check("a zone's outline stops at the walls", clipped and all(p[0] <= 2.0 + 1e-9 for l in clipped for p in l),
          max(p[0] for l in clipped for p in l) if clipped else None)
    e = edges({"type": "box", "center": [0, 0, 1], "size": [2, 2, 2]})
    check("a box zone outlines as 2 rings + 4 uprights", len(e) == 6 and len(e[0]) == 5)
    zl = zone_lines({"type": "box", "center": [0, 0, 1], "size": [1, 1, 1]}, footprint(box))
    check("a zone is drawn as its full outline (rings and uprights)", len(zl) == 6, len(zl))
    zc = zone_lines({"type": "cylinder", "center": [0, 0, 0], "radius": 0.5, "height": 1.0}, footprint(box))
    check("a cylinder zone too", len(zc) == 2 + 8, len(zc))
    check("looks by name", look_key("wall_right_glass") == "glass" and look_key("partition_left") == "wall"
          and look_key("control_cart") == "cart" and wall_look("wall_tv") == "wall"
          and wall_look("wall_back") == "glass_face")
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
