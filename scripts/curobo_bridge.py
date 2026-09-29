"""cuRobo (NVIDIA, GPU) as an optional planner for the moves between poses.

safe_move.route asks it when the straight MoveJ is blocked, before its own
grid of folded postures: a collision-free joint path from cuRobo's
MotionGen, in about a second, 30-40 % less joint travel than the grid's
detours (the FR20 spike, 2026-09-28: curobo-spike-fr20/src/FR20_RESULTS.md).
Only the path is taken: it is thinned to a few waypoints and every leg is
checked again by safe_move (our capsules, the move margins, the work zones,
the joint limits); the moves are timed as before (Ruckig). When the service
is not running, or its path fails our check, route goes on as without it.

The service runs in the cuRobo Docker image (docker-curobo:latest, v0.7.7):

    uv run scripts/curobo_bridge.py up      robot config to .curobo/, start the service (port 8768)
    uv run scripts/curobo_bridge.py down    stop it
    uv run scripts/curobo_bridge.py status
    uv run scripts/curobo_bridge.py         self-test (no Docker needed)

ROBOT_CUROBO=0 turns it off; ROBOT_CUROBO_URL points elsewhere.

The robot for cuRobo: the vendor URDF's chain (no meshes) with J6 in the
tool cable's range, spheres filling the toolkit's own capsules (the LED
strip on a link of its own), self-collision per link pair only where the
toolkit checks every capsule pair of it; PLAN_BUFFER_M more room to the
world than our margins (a pose near a margin, as a hub can be, is a valid
start: its check is ours). The world: each obstacle and
keep-out zone grown by its margin, as cuboids (a halfspace a slab 1 m thick,
a cylinder its bounding box).
"""

import json
import math
import os
import subprocess
import sys
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

URL = os.environ.get("ROBOT_CUROBO_URL", "http://127.0.0.1:8768")
IMAGE = "docker-curobo:latest"
CONTAINER = "robot-curobo"
WORK = os.path.join(ROOT, ".curobo")
TOOL_LINK = "led_strip"                  # the tool on a link of its own (cuRobo extra_links)
THIN_DEG = (3.0, 1.0)                    # waypoint tolerances tried, coarse first
PLAN_BUFFER_M = 0.02                     # cuRobo keeps this much more than our margins: its spheres and
                                         # 1 cm activation skimmed them (the wipe's moves, 2026-09-28)


# --- the robot ---------------------------------------------------------------

def robot_config(prof, model, urdf_path):
    """cuRobo's robot config (a dict) for the FR20 of `prof` with the
    toolkit's collision `model`; writes its chain-only URDF to urdf_path."""
    import collision as C
    import robot_profile as RP
    lim = RP.motion_limits(prof)
    root = ET.parse(os.path.join(ROOT, prof["rig"]["urdf"])).getroot()
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for el in link.findall(tag):
                link.remove(el)
    joints = [j for j in root.findall("joint") if j.get("type") in ("revolute", "continuous")]
    for j, (lo, hi) in zip(joints, lim):
        lm = j.find("limit")
        lm.set("lower", "%.6f" % math.radians(lo))
        lm.set("upper", "%.6f" % math.radians(hi))
    ET.ElementTree(root).write(urdf_path)
    names = [j.get("name") for j in joints]
    links = [joints[0].find("parent").get("link")] + [j.find("child").get("link") for j in joints]

    def link_of(cap):
        return TOOL_LINK if cap["name"].startswith("tool_") else links[cap["joint"] + 1]

    caps = [c for c in model["caps"] if not c.get("root")]   # the upper arm at J2: contact-only, not for cuRobo
    spheres = {}
    for cap in caps:
        a, b, r = cap["a"], cap["b"], cap["r"]
        n = max(1, int(math.ceil(math.dist(a, b) / r)))        # spheres r apart along the capsule
        for k in range(n + 1):
            c = [a[i] + k / n * (b[i] - a[i]) for i in range(3)]
            spheres.setdefault(link_of(cap), []).append({"center": [round(x, 5) for x in c], "radius": round(r, 5)})
    # a link pair is checked only when the toolkit checks every capsule pair of it
    pairs = set(C._self_pairs(model))
    idx = {id(c): k for k, c in enumerate(model["caps"])}
    all_links = links + [TOOL_LINK]
    ignore = {}
    for a in all_links:
        ignore[a] = []
        for b in all_links:
            cp = [(idx[id(x)], idx[id(y)]) for x in caps for y in caps if link_of(x) == a and link_of(y) == b]
            if a != b and (not cp or not all((min(i, j), max(i, j)) in pairs for i, j in cp)):
                ignore[a].append(b)
    rest = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    return {"robot_cfg": {"kinematics": {
        "urdf_path": "/work/" + os.path.basename(urdf_path), "asset_root_path": "/work",
        "base_link": links[0], "ee_link": links[-1],
        # the base is bolted on the plate: never checked against the room
        "collision_link_names": [l for l in links[1:] + [TOOL_LINK] if l in spheres],
        "collision_spheres": {l: v for l, v in spheres.items() if l != links[0]}, "collision_sphere_buffer": PLAN_BUFFER_M,
        "extra_links": {TOOL_LINK: {"parent_link_name": links[-1], "link_name": TOOL_LINK,
                                    "joint_name": TOOL_LINK + "_joint", "joint_type": "FIXED",
                                    "fixed_transform": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]}},
        "self_collision_ignore": ignore, "self_collision_buffer": {l: 0.0 for l in all_links},
        "use_global_cumul": True,
        "cspace": {"joint_names": names, "retract_config": [math.radians(x) for x in rest],
                   "null_space_weight": [1.0] * 6, "cspace_distance_weight": [1.0] * 6,
                   "max_jerk": 500.0, "max_acceleration": 15.0}}}}


# --- the world ---------------------------------------------------------------

def _quat_z_to(n):
    """(w, x, y, z) of a rotation taking +z to the unit vector n."""
    ref = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    d = sum(ref[k] * n[k] for k in range(3))
    x = [ref[i] - n[i] * d for i in range(3)]
    ln = math.sqrt(sum(v * v for v in x))
    x = [v / ln for v in x]
    y = [n[1] * x[2] - n[2] * x[1], n[2] * x[0] - n[0] * x[2], n[0] * x[1] - n[1] * x[0]]
    R = [[x[0], y[0], n[0]], [x[1], y[1], n[1]], [x[2], y[2], n[2]]]
    w = math.sqrt(max(0.0, 1 + R[0][0] + R[1][1] + R[2][2])) / 2
    qx = math.copysign(math.sqrt(max(0.0, 1 + R[0][0] - R[1][1] - R[2][2])) / 2, R[2][1] - R[1][2])
    qy = math.copysign(math.sqrt(max(0.0, 1 - R[0][0] + R[1][1] - R[2][2])) / 2, R[0][2] - R[2][0])
    qz = math.copysign(math.sqrt(max(0.0, 1 - R[0][0] - R[1][1] + R[2][2])) / 2, R[1][0] - R[0][1])
    return [w, qx, qy, qz]


def world_config(menv):
    """cuRobo's world (cuboids) for an env with its margins applied
    (safe_move.move_env): obstacles grown by their margin, keep-out zones
    as they are; work and slow zones are not obstacles (safe_move checks
    the work zones on the path it gets back)."""
    base = float(menv.get("margin_m", 0.05))
    cub = {}
    for o in menv["objects"]:
        if o["role"] not in ("obstacle", "keep_out"):
            continue
        m = float(o.get("margin_m", base)) if o["role"] == "obstacle" else 0.0
        if o["type"] == "halfspace":                     # free side n.p >= d: a slab behind the plane
            n = o["normal"]
            ln = math.sqrt(sum(v * v for v in n))
            n = [v / ln for v in n]
            d = o["offset"] / ln
            cub[o["name"]] = {"dims": [20.0, 20.0, 1.0],
                              "pose": [n[i] * (d - 0.5 + m) for i in range(3)] + _quat_z_to(n)}
        elif o["type"] == "box":
            yaw = math.radians(o.get("yaw_deg", 0.0))
            cub[o["name"]] = {"dims": [s + 2 * m for s in o["size"]],
                              "pose": list(o["center"]) + [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]}
        elif o["type"] == "cylinder":                    # its bounding box, grown
            r, h, c = o["radius"] + m, o["height"] + 2 * m, o["center"]
            cub[o["name"]] = {"dims": [2 * r, 2 * r, h], "pose": [c[0], c[1], c[2] + o["height"] / 2, 1.0, 0.0, 0.0, 0.0]}
    return {"cuboid": cub}


# --- the path ------------------------------------------------------------------

def thin(qs, tol_deg):
    """Waypoints (after the first pose) that keep the joint path within
    tol_deg of qs (Douglas-Peucker in joint space, the largest joint's
    deviation); the last pose always."""
    if len(qs) < 3:
        return [list(q) for q in qs[1:]]

    def dev(p, a, b, f):
        return max(abs(p[j] - (a[j] + f * (b[j] - a[j]))) for j in range(len(p)))

    keep = {0, len(qs) - 1}
    stack = [(0, len(qs) - 1)]
    while stack:
        i, k = stack.pop()
        if k - i < 2:
            continue
        worst, at = 0.0, None
        for m in range(i + 1, k):
            d = dev(qs[m], qs[i], qs[k], (m - i) / (k - i))
            if d > worst:
                worst, at = d, m
        if worst > tol_deg:
            keep.add(at)
            stack += [(i, at), (at, k)]
    return [list(qs[m]) for m in sorted(keep) if m > 0]


# --- the service ---------------------------------------------------------------

_state = {"up": None}


def _call(path, body=None, timeout=10.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(URL + path, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def available():
    """The service answers and is ready (asked once a process)."""
    if os.environ.get("ROBOT_CUROBO", "1") == "0":
        return False
    if _state["up"] is None:
        try:
            _state["up"] = bool(_call("/health", timeout=0.5).get("ready"))
        except (OSError, ValueError, urllib.error.URLError):
            _state["up"] = False
    return _state["up"]


def plan(q_from, q_to, world, timeout=30.0):
    """cuRobo's joint path (degrees) q_from -> q_to in `world`, or (None, why)."""
    try:
        r = _call("/plan", {"start": list(q_from), "goal": list(q_to), "world": world}, timeout)
    except (OSError, ValueError, urllib.error.URLError) as e:
        _state["up"] = False                               # gone: don't ask again this run
        return None, "cuRobo unreachable (%s)" % e
    if not r.get("ok"):
        return None, "cuRobo: %s" % r.get("status", "no path")
    return r["q_deg"], "cuRobo %.2f s" % r.get("plan_s", 0.0)


def route(q_from, q_to, menv, model, limits, clear, within, planner=None):
    """A detour from cuRobo, checked leg by leg: (waypoints, why) or
    (None, why). clear(a, b) is None when the MoveJ a -> b is clear;
    within(q) keeps a waypoint inside the joint limits. planner (for tests)
    stands in for the service."""
    if planner is None:
        if not available():
            return None, "cuRobo not running"
        qs, why = plan(q_from, q_to, world_config(menv))
    else:
        qs, why = planner(q_from, q_to, world_config(menv))
    if qs is None:
        return None, why
    for tol in THIN_DEG:
        path = thin([list(q_from)] + [list(q) for q in qs[1:-1]] + [list(q_to)], tol)
        pts = [list(q_from)] + path
        if all(within(w) for w in path[:-1]) and all(clear(a, b) is None for a, b in zip(pts, pts[1:])):
            travel = sum(max(abs(x - y) for x, y in zip(a, b)) for a, b in zip(pts, pts[1:]))
            return path, "%s: detour through %d waypoint(s), %.0f deg of joint travel" % (why, len(path) - 1, travel)
    return None, why + ", its path not clear by our check"


# --- up / down -------------------------------------------------------------------

def up():
    import collision as C
    import robot_profile as RP
    os.makedirs(WORK, exist_ok=True)
    prof = RP.load("fr20")
    cfg = robot_config(prof, C.load_model("fr20"), os.path.join(WORK, "fr20.urdf"))
    with open(os.path.join(WORK, "fr20.json"), "w") as f:
        json.dump(cfg, f, indent=1)
    down(quiet=True)
    cmd = ["docker", "run", "-d", "--rm", "--name", CONTAINER, "--gpus", "all", "-p", "8768:8768",
           "-v", WORK + ":/work", "-v", HERE + ":/app/scripts", IMAGE, "python3", "/app/scripts/curobo_service.py"]
    out = subprocess.run(cmd, capture_output=True, text=True)
    print(out.stdout.strip() or out.stderr.strip())
    print("starting; `uv run scripts/curobo_bridge.py status` says when it is ready (~1 min: build and warm-up)")
    return out.returncode


def down(quiet=False):
    out = subprocess.run(["docker", "stop", CONTAINER], capture_output=True, text=True)
    if not quiet:
        print(out.stdout.strip() or out.stderr.strip())
    return 0


def status():
    try:
        print(json.dumps(_call("/health", timeout=2.0)))
        return 0
    except (OSError, ValueError, urllib.error.URLError) as e:
        print("not running (%s)" % e)
        return 1


# --- self-test -------------------------------------------------------------------

def self_test():
    import collision as C
    import robot_profile as RP
    import safe_move as SM
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    env = C.load_env(os.path.join(ROOT, "envs", "volvox_lab.usda"))
    menv = SM.move_env(env)
    w = world_config(menv)["cuboid"]
    ceil = next(o for o in menv["objects"] if o["name"] == "ceiling")
    z_ceiling = -ceil["offset"] / math.sqrt(sum(v * v for v in ceil["normal"]))
    c = w["ceiling"]
    check("the ceiling's slab starts its margin below it", abs((c["pose"][2] - 0.5) - (z_ceiling - SM.CEILING_MARGIN_M)) < 1e-9,
          (c["pose"][2] - 0.5, z_ceiling - SM.CEILING_MARGIN_M))
    box = next(o for o in menv["objects"] if o["type"] == "box" and o["role"] == "obstacle")
    check("a box is grown by its margin", all(abs(a - (b + 2 * box["margin_m"])) < 1e-9 for a, b in zip(w[box["name"]]["dims"], box["size"])))
    check("slow and work zones are not obstacles", all(o["role"] not in ("slow", "work") for o in menv["objects"] if o["name"] in w))

    qs = [[k * 1.0, 0, 0, 0, 0, 0] for k in range(11)] + [[10.0, k * 1.0, 0, 0, 0, 0] for k in range(1, 11)]
    got = thin(qs, 0.5)
    check("thinning keeps the corner and the end, nothing else", got == [[10.0, 0, 0, 0, 0, 0], [10.0, 10.0, 0, 0, 0, 0]], got)

    model = C.load_model("fr20", tool=False)
    lim = RP.motion_limits(RP.load("fr20"))
    home = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    goal = [-60.0, -90.0, 90.0, -90.0, -90.0, 0.0]

    def clear(a, b):
        return SM.segment_clear(model, menv, a, b)

    def within(q):
        return SM._within(q, lim)

    def straight(a, b, world):
        return [a, [(x + y) / 2 for x, y in zip(a, b)], b], "fake 0.01 s"
    path, why = route(home, goal, menv, model, lim, clear, within, planner=straight)
    check("a clear path from the planner is taken, thinned", path == [goal], (path, why))
    up_ = [0.0, -90.0, 0.0, -90.0, -90.0, 0.0]                  # the arm straight up: through the ceiling margin

    def through_ceiling(a, b, world):
        return [a, up_, b], "fake"
    path, why = route(home, goal, menv, model, lim, clear, within, planner=through_ceiling)
    check("a path our check refuses is not taken", path is None and "not clear" in why, why)
    path, why = route(home, goal, menv, model, lim, clear, within, planner=lambda a, b, w: (None, "cuRobo: no path"))
    check("no path from the planner: none", path is None, why)
    _state["up"] = False
    check("the service down: route says so", route(home, goal, menv, model, lim, clear, within)[1] == "cuRobo not running")
    _state["up"] = None
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    sys.exit({"up": up, "down": down, "status": status}.get(arg, self_test)())
