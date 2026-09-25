"""A RoomPlan scan (USDZ, e.g. the iOS "RoomPlan" app) -> the robot frame ->
objects in the env file.

    hython scripts/scan_to_env.py scan.usdz                       # list what the scan holds
    hython scripts/scan_to_env.py scan.usdz --front Wall2 --left Wall1 \\
        --name Storage1=control_cart --name Television0=tv --drop chairs [--write]

RoomPlan writes every wall, door and object as a box (Y up, metres). The
scan has its own frame; it is placed in the robot's by the planes the arm
itself measured (scripts/probe_ui.py): --front is the scan wall that is the
env's `wall_tv`, --left the one that is `partition_left`, and the scan's
floor goes to the env's `floor`. The two walls each give the rotation --
how far apart they are is the fit check -- and together the position.

Then: measured objects stay as they are; the two matched walls are not
added again; the other walls become planes (collinear pieces merged);
objects become boxes (named scan_<name>, or --name SCAN=ENV to replace an
env object, e.g. an estimate); --drop removes env objects that are gone.
Without --write it only prints the changes. The old env is kept as .bak.

Reading the USDZ needs pxr (hython has it); the rest is pure Python.
Tests: python scripts/scan_to_env.py --self-test
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

ENV = os.path.join(ROOT, "envs", "volvox_lab.json")


def read_roomplan(path):
    """Every mesh of a RoomPlan USDZ as a box in a Z-up scan frame:
    {name, yaw_deg, size [along its x, y, height], center [x, y, z]}."""
    from pxr import Gf, Usd, UsdGeom
    st = Usd.Stage.Open(path)
    if UsdGeom.GetStageUpAxis(st) != "Y":
        raise ValueError("expected a Y-up RoomPlan stage, got %s" % UsdGeom.GetStageUpAxis(st))
    k = UsdGeom.GetStageMetersPerUnit(st)
    out = []
    for p in st.Traverse():
        if p.GetTypeName() != "Mesh":
            continue
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pts = [m.Transform(Gf.Vec3d(*v)) for v in UsdGeom.Mesh(p).GetPointsAttr().Get()]
        P = [(v[0] * k, -v[2] * k, v[1] * k) for v in pts]            # Y up -> Z up, a proper rotation
        ax = m.GetRow3(0)
        yaw = math.degrees(math.atan2(-ax[2], ax[0]))
        out.append(box_from_points(p.GetName(), P, yaw))
    return out


def box_from_points(name, P, yaw):
    """The box of points P whose local x axis is at yaw (deg, from above)."""
    ux = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
    uy = (-ux[1], ux[0])
    a = [q[0] * ux[0] + q[1] * ux[1] for q in P]
    b = [q[0] * uy[0] + q[1] * uy[1] for q in P]
    ca, cb = (max(a) + min(a)) / 2, (max(b) + min(b)) / 2
    z0, z1 = min(q[2] for q in P), max(q[2] for q in P)
    return {"name": name, "yaw_deg": yaw, "size": [max(a) - min(a), max(b) - min(b), z1 - z0],
            "center": [ca * ux[0] + cb * uy[0], ca * ux[1] + cb * uy[1], (z0 + z1) / 2]}


def kind(b):
    n = b["name"].lower()
    for k in ("wall", "floor", "door", "window", "opening"):
        if n.startswith(k):
            return k
    return "object"


def inner_plane(wall, inside):
    """(normal, offset) of a wall box's face towards the point `inside`
    (from above): n . x = offset, n pointing into the room."""
    a = math.radians(wall["yaw_deg"])
    n = (-math.sin(a), math.cos(a))
    c = wall["center"]
    if n[0] * (inside[0] - c[0]) + n[1] * (inside[1] - c[1]) < 0:
        n = (-n[0], -n[1])
    return n, n[0] * c[0] + n[1] * c[1] + wall["size"][1] / 2


def _ang(v):
    return math.degrees(math.atan2(v[1], v[0]))


def _wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def align(scan, env, front, left):
    """The scan -> robot transform from two walls and the floor:
    {yaw_deg, t [x, y, z], yaw_disagreement_deg}. front / left: scan wall
    names matched to the env's wall_tv / partition_left (vertical planes)."""
    by = {b["name"]: b for b in scan}
    O = {o["name"]: o for o in env["objects"]}
    floor = next(b for b in scan if kind(b) == "floor")
    inside = floor["center"]
    pairs = []
    for sname, ename in ((front, "wall_tv"), (left, "partition_left")):
        o = O[ename]
        if o["type"] != "halfspace" or abs(o["normal"][2]) > 1e-6:
            raise ValueError("%s must be a measured vertical wall in the env" % ename)
        nr = o["normal"][:2]
        L = math.hypot(*nr)
        pairs.append((inner_plane(by[sname], inside), ((nr[0] / L, nr[1] / L), o["offset"] / L)))
    yaws = [_wrap(_ang(r[0]) - _ang(s[0])) for s, r in pairs]
    yaw = yaws[0] + _wrap(yaws[1] - yaws[0]) / 2
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    rows = []
    for (ns, offs), (nr, offr) in pairs:
        n = (c * ns[0] - s * ns[1], s * ns[0] + c * ns[1])             # the scan normal, rotated
        rows.append((n, offr - offs))
    (a, b), (cc, d) = rows[0][0], rows[1][0]
    det = a * d - b * cc
    if abs(det) < 0.2:
        raise ValueError("the two walls are nearly parallel: they cannot fix the position")
    tx = (rows[0][1] * d - b * rows[1][1]) / det
    ty = (a * rows[1][1] - cc * rows[0][1]) / det
    fl = O["floor"]
    tz = fl["offset"] / fl["normal"][2] - (floor["center"][2] + floor["size"][2] / 2)
    return {"yaw_deg": yaw, "t": [tx, ty, tz], "yaw_disagreement_deg": abs(_wrap(yaws[1] - yaws[0]))}


def transform(b, al):
    c, s = math.cos(math.radians(al["yaw_deg"])), math.sin(math.radians(al["yaw_deg"]))
    x, y, z = b["center"]
    t = al["t"]
    return dict(b, center=[c * x - s * y + t[0], s * x + c * y + t[1], z + t[2]],
                yaw_deg=_wrap(b["yaw_deg"] + al["yaw_deg"]))


def apply(env, scan, al, front, left, names=None, drop=(), source=""):
    """The aligned scan into env (in place). Returns change lines."""
    names = names or {}
    lines = []
    floor = next(b for b in scan if kind(b) == "floor")
    robot = [transform(b, al) for b in scan]
    inside = transform(floor, al)["center"]
    for n in drop:
        before = len(env["objects"])
        env["objects"] = [o for o in env["objects"] if o["name"] != n]
        lines.append(("removed %s" % n) if len(env["objects"]) < before else "no %s to remove" % n)
    # walls not matched to a measured one: planes, collinear pieces merged
    planes = []
    for b in robot:
        if kind(b) != "wall" or b["name"] in (front, left):
            continue
        n, off = inner_plane(b, inside)
        for p in planes:
            if abs(_wrap(_ang(n) - _ang(p["n"]))) < 2.0 and abs(off - p["off"]) < 0.05:
                p["from"].append(b["name"])
                break
        else:
            planes.append({"n": n, "off": off, "from": [b["name"]]})
    O = {o["name"]: o for o in env["objects"]}
    for i, p in enumerate(planes):
        nm = "scan_wall_%d" % i
        o = O.get(nm) or {"name": nm, "role": "obstacle"}
        if nm not in O:
            env["objects"].append(o)
        o.update(type="halfspace", normal=[round(p["n"][0], 6), round(p["n"][1], 6), 0.0], offset=round(p["off"], 5),
                 measured="scan", note="RoomPlan %s (%s)%s" % ("+".join(p["from"]), source, ""))
        lines.append("%s: plane from %s, %.2f m from the base" % (nm, "+".join(p["from"]), -p["off"]))
    for b in robot:
        if kind(b) != "object":
            continue
        nm = names.get(b["name"], "scan_" + b["name"].lower())
        geo = {"type": "box", "center": [round(x, 4) for x in b["center"]],
               "size": [round(x, 4) for x in b["size"]], "yaw_deg": round(b["yaw_deg"], 2)}
        o = O.get(nm)
        if o is None:
            o = {"name": nm, "role": "obstacle"}
            env["objects"].append(o)
            lines.append("added %s (%s): %.2f x %.2f x %.2f m at (%.2f, %.2f)" % (
                nm, b["name"], *geo["size"], *geo["center"][:2]))
        else:
            old = o.get("center")
            lines.append("%s <- %s%s" % (nm, b["name"], " (moved %.2f m)" % math.dist(old[:2], geo["center"][:2])
                                           if old else ""))
            for k in ("center", "size", "yaw_deg", "radius", "height", "normal", "offset", "type"):
                o.pop(k, None)
        o.update(geo)
        o["measured"] = "scan"
        o["note"] = ("%s; " % o["note"].split(" ESTIMATE")[0].split("; ESTIMATE")[0] if o.get("note") else "") + \
            "RoomPlan %s (%s)" % (b["name"], source)
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scan", nargs="?")
    ap.add_argument("--env", default=ENV)
    ap.add_argument("--front", help="the scan wall that is the env's wall_tv")
    ap.add_argument("--left", help="the scan wall that is the env's partition_left")
    ap.add_argument("--name", action="append", default=[], help="SCAN=ENV: give a scan object an env name")
    ap.add_argument("--drop", default="", help="comma-separated env objects to remove")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    scan = read_roomplan(a.scan)
    if not (a.front and a.left):
        for b in scan:
            print("%-14s %-7s %5.2f x %5.2f x %5.2f m  yaw %7.2f  at (%.2f, %.2f)" % (
                b["name"], kind(b), *b["size"], b["yaw_deg"], *b["center"][:2]))
        print("\nname the two measured walls: --front <wall_tv> --left <partition_left>")
        return 0
    import collision as CL
    env = json.load(open(a.env))
    al = align(scan, env, a.front, a.left)
    print("aligned: yaw %.2f deg, t (%.3f, %.3f, %.3f) m; the two walls agree to %.2f deg"
          % (al["yaw_deg"], *al["t"], al["yaw_disagreement_deg"]))
    src = "%s, aligned to the probed %s / %s / floor, yaw %.2f deg" % (
        os.path.basename(a.scan), a.front + "=wall_tv", a.left + "=partition_left", al["yaw_deg"])
    lines = apply(env, scan, al, a.front, a.left, dict(x.split("=", 1) for x in a.name),
                  [x for x in a.drop.split(",") if x], src)
    for line in lines:
        print("  " + line)
    errs = CL.validate_env(env)
    if errs:
        print("env would be invalid:", errs)
        return 1
    if a.write:
        shutil.copyfile(a.env, a.env + ".bak")
        json.dump(env, open(a.env, "w"), indent=1)
        print("wrote", a.env, "(previous: .bak)")
    return 0


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    def wall(name, yaw, cx, cy, length):
        return {"name": name, "yaw_deg": yaw, "size": [length, 0.16, 2.4], "center": [cx, cy, 1.2]}

    # a 4 x 3 m room in its scan frame; the robot base (unknown to the scan)
    # at scan (1.5, 1.0); the robot frame is the scan turned by 30 deg
    scan = [wall("Wall0", 0.0, 2.0, -0.08, 4.0), wall("Wall1", 90.0, 4.08, 1.5, 3.0),
            wall("Wall2", 0.0, 2.0, 3.08, 4.0), wall("Wall3", 90.0, -0.08, 1.5, 3.0),
            {"name": "Floor0", "yaw_deg": 0.0, "size": [4.0, 3.0, 0.1], "center": [2.0, 1.5, -0.05]},
            {"name": "Storage0", "yaw_deg": 0.0, "size": [0.6, 0.4, 1.0], "center": [3.5, 2.5, 0.5]}]
    th = 30.0
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    base = (1.5, 1.0)

    def to_robot(p):
        x, y = p[0] - base[0], p[1] - base[1]
        return (c * x - s * y, s * x + c * y)

    def robot_plane(n_scan, point_scan):
        n = (c * n_scan[0] - s * n_scan[1], s * n_scan[0] + c * n_scan[1])
        q = to_robot(point_scan)
        return [n[0], n[1], 0.0], n[0] * q[0] + n[1] * q[1]
    # the probe measured Wall0 (y = 0, facing +y) as "wall_tv" and Wall3
    # (x = 0, facing +x) as "partition_left"; the floor 2 cm below the base
    ntv, otv = robot_plane((0.0, 1.0), (1.0, 0.0))
    nl, ol = robot_plane((1.0, 0.0), (0.0, 1.0))
    env = {"schema": "motionlab.env/1", "objects": [
        {"name": "floor", "type": "halfspace", "normal": [0, 0, 1], "offset": -0.02, "role": "obstacle"},
        {"name": "wall_tv", "type": "halfspace", "normal": ntv, "offset": otv, "role": "obstacle"},
        {"name": "partition_left", "type": "halfspace", "normal": nl, "offset": ol, "role": "obstacle"},
        {"name": "cart", "type": "box", "center": [0, 0, 0.5], "size": [1, 1, 1], "role": "obstacle", "note": "red"},
        {"name": "chairs", "type": "box", "center": [0, 0, 0.5], "size": [1, 1, 1], "role": "obstacle"}]}
    al = align(scan, env, "Wall0", "Wall3")
    t_expect = to_robot((0.0, 0.0))
    check("rotation and position recovered from two walls and the floor",
          abs(al["yaw_deg"] - th) < 1e-6 and math.dist(al["t"][:2], t_expect) < 1e-6 and abs(al["t"][2] - (-0.02)) < 1e-9
          and al["yaw_disagreement_deg"] < 1e-6, al)
    lines = apply(env, scan, al, "Wall0", "Wall3", {"Storage0": "cart"}, ["chairs"], "test")
    O = {o["name"]: o for o in env["objects"]}
    st = O["cart"]
    check("a scan object replaces the named env object, in the robot frame",
          math.dist(st["center"][:2], to_robot((3.5, 2.5))) < 1e-4 and abs(st["center"][2] - 0.48) < 1e-4
          and abs(st["yaw_deg"] - th) < 1e-6 and st["note"].startswith("red"), st)
    check("dropped objects are gone", "chairs" not in O, lines)
    extra = [o for o in env["objects"] if o["name"].startswith("scan_wall")]
    check("the two unmatched walls become planes facing the room",
          len(extra) == 2 and all(o["normal"][0] * 0 + 1 for o in extra)
          and all((o["normal"][0] * 0.0 + o["normal"][1] * 0.0) > o["offset"] for o in extra), lines)
    import collision as CL
    check("... and the result is a valid env", not CL.validate_env(env), CL.validate_env(env))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
