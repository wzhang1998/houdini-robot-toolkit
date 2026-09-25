"""Controller MoveJ moves (go to a clip's start, go HOME) checked against the
room with wider margins than a clip gets, and a detour when the straight
move is not clear.

A MoveJ is a straight line in joint space: every joint turns at once. Going
from one elbow configuration to the other (J3 through 0) straightens the arm
on the way; with the upper arm standing, a straight FR20 points at the
ceiling -- on 2026-09-25 such a move passed 6 cm under the ceiling grid.
Clips are timed and checked in Houdini; these moves are not, so:

    MOVE_MARGIN_M      every obstacle, at least (floor and base plate keep
                       their own)
    CEILING_MARGIN_M   the ceiling
    keep-out zones     no link inside
    work zones         the TCP stays WORK_INSET_M inside (the controller's
                       own work area among them: it stops the arm outside)

route(q_from, q_to, env) returns the straight move when it is clear, else
one or two waypoints: fold the elbow at the current J1 (the arm lowered, so
a straight arm is not pointing up), turn J1, unfold. The candidates are
searched in order of total joint travel; the first clear route wins.

    python scripts/safe_move.py        self-test
"""

import itertools
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import collision as C  # noqa: E402

MOVE_MARGIN_M = 0.10
CEILING_MARGIN_M = 0.30
STEP_DEG = 2.0                      # joint-space sampling of a move
KEEP_OWN = ("floor", "base_plate")  # contact-close by design: their own margins
# the upper arm's root turns in place about J2, a few cm over the plate and
# the floor whatever J2 is: only contact counts for it there
ROOT_LINK = "upperarm_link"
WORK_INSET_M = 0.03
SEARCH_STEP_DEG = 6.0               # coarse sampling while searching a detour; the winner is re-checked at STEP_DEG


def move_env(env):
    """The env's obstacles and keep-out zones with the move margins."""
    base = float(env.get("margin_m", 0.05))
    objs = []
    for o in env.get("objects", []):
        if o["role"] not in ("obstacle", "keep_out", "work"):
            continue
        o = dict(o)
        if o["role"] == "obstacle" and o["name"] not in KEEP_OWN:
            if o["type"] == "halfspace" and C.U._normalize(o["normal"])[2] < -0.9:
                o["margin_m"] = max(o.get("margin_m", base), CEILING_MARGIN_M)
            else:
                o["margin_m"] = max(o.get("margin_m", base), MOVE_MARGIN_M)
        objs.append(o)
    return dict(env, objects=objs)


def _samples(a, b, step=STEP_DEG):
    n = max(1, int(math.ceil(max(abs(x - y) for x, y in zip(a, b)) / step)))
    return [[x + (y - x) * k / float(n) for x, y in zip(a, b)] for k in range(n + 1)]


def blocked(model, menv, q):
    """(link, object, clearance m) of the first violation at pose q, or None."""
    caps, tcp = C.capsules(model, q)
    base = float(menv.get("margin_m", 0.05))
    for o in menv["objects"]:
        if o["role"] == "work":
            out = C.sdf(o, tcp) + WORK_INSET_M
            if out > 0:
                return "tcp", o["name"] + " (outside)", -out
            continue
        limit = o.get("margin_m", base) if o["role"] == "obstacle" else 0.0
        for name, a, b, r in caps:
            if name in C.FIXED_LINKS:
                continue
            d = C.capsule_distance(o, a, b, r)
            if name == ROOT_LINK and o["name"] in KEEP_OWN:
                d += limit
            if d < limit:
                return name, o["name"], d
    for i, j in model["pairs"]:
        d = C._seg_seg_dist(caps[i][1], caps[i][2], caps[j][1], caps[j][2]) - caps[i][3] - caps[j][3]
        if d < 0.0:
            return caps[i][0], caps[j][0], d
    return None


def segment_clear(model, menv, a, b, step=STEP_DEG):
    """None when the MoveJ a -> b is clear, else (fraction along, violation)."""
    qs = _samples(a, b, step)
    for k, q in enumerate(qs):
        hit = blocked(model, menv, q)
        if hit:
            return k / float(len(qs) - 1), hit
    return None


def _travel(path):
    return sum(max(abs(x - y) for x, y in zip(a, b)) for a, b in zip(path, path[1:]))


def _within(q, limits, pad=2.0):
    return all(lo + pad <= x <= hi - pad for x, (lo, hi) in zip(q, limits))


def route(q_from, q_to, env, model=None, limits=None):
    """(waypoints after q_from ending at q_to, description) or (None, why)."""
    model = model or C.load_model("fr20")
    menv = move_env(env)
    direct = segment_clear(model, menv, q_from, q_to)
    if direct is None:
        return [list(q_to)], "straight MoveJ, clear (margins %.2f m, ceiling %.2f m)" % (MOVE_MARGIN_M, CEILING_MARGIN_M)
    why = "straight MoveJ blocked %.0f%% of the way: %s near %s (%.3f m)" % (100 * direct[0], *direct[1])
    limits = limits or [(-175, 175), (-265, 85), (-162, 162), (-265, 85), (-175, 175), (-175, 175)]
    # folded postures: J2 / J3 on a grid, the wrist already at the goal's.
    # The fold happens facing J1 = t: where the arm is, where it goes, or
    # any other direction (a straightening arm needs open room around it)
    folds = [(j2, j3) for j2 in range(-180, 1, 15) for j3 in range(-150, 151, 30)]
    turns = sorted({round(q_from[0], 3), round(q_to[0], 3)} | set(range(-150, 151, 30)))
    cands = []
    for (j2, j3), t in itertools.product(folds, turns):
        fold = [t, j2, j3] + list(q_to[3:])
        lead = [] if t == round(q_from[0], 3) else [[t] + list(q_from[1:])]      # turn first, as the arm is
        cands.append(lead + [fold, list(q_to)])                                  # fold, then MoveJ to the goal
        if t != round(q_to[0], 3):
            cands.append(lead + [fold, [q_to[0], j2, j3] + list(q_to[3:]), list(q_to)])   # fold, turn, unfold
    cands.sort(key=lambda p: _travel([q_from] + p))
    for path in cands:
        if not all(_within(w, limits) for w in path[:-1]):
            continue
        if any(blocked(model, menv, w) for w in path[:-1]):
            continue
        pts = [list(q_from)] + path
        legs = list(zip(pts, pts[1:]))
        if all(segment_clear(model, menv, a, b, SEARCH_STEP_DEG) is None for a, b in legs) and                 all(segment_clear(model, menv, a, b) is None for a, b in legs):
            return path, why + "; detour through %d waypoint(s), %.0f deg of joint travel" % (len(path) - 1, _travel(pts))
    return None, why + "; no detour found -- move it by hand (WebApp jog) first"


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    root = os.path.dirname(HERE)
    env = C.load_env(os.path.join(root, "envs", "volvox_lab.json"))
    model = C.load_model("fr20")
    menv = move_env(env)
    ceil = [o for o in menv["objects"] if o["name"] == "ceiling"][0]
    check("the ceiling gets the ceiling margin", ceil["margin_m"] == CEILING_MARGIN_M, ceil["margin_m"])
    plate = [o for o in menv["objects"] if o["name"] == "base_plate"][0]
    check("the base plate keeps its own margin", plate.get("margin_m") == [o for o in env["objects"] if o["name"] == "base_plate"][0].get("margin_m"))
    check("slow zones are not move obstacles; work zones are kept", all(o["role"] != "slow" for o in menv["objects"])
          and any(o["role"] == "work" for o in menv["objects"]))
    zone = [o for o in env["objects"] if o["name"] == "controller_zone"]
    if zone:
        behind = [170.0, -90.0, 90.0, -90.0, -90.0, 0.0]         # HOME turned to face the cart: TCP behind the zone
        hit = blocked(model, menv, behind)
        check("a pose with the TCP outside the controller's work area is blocked", hit is not None and "outside" in hit[1], hit)
    # 2026-09-25: from the inside-wall test clip's pose to HOME passed 6 cm under the grid
    b = [102.869, -125.575, -29.215, -209.768, -63.033, 2.070]
    home = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    no_zone = move_env(dict(env, objects=[o for o in env["objects"] if o["role"] != "work"]))
    hit = segment_clear(model, no_zone, b, home)
    check("the elbow-flip MoveJ that nearly hit the ceiling is blocked by the ceiling margin", hit is not None and hit[1][1] == "ceiling", hit)
    check("... (and, with the controller's zone, by the TCP leaving it first)", segment_clear(model, menv, b, home) is not None)
    path, why = route(b, home, env, model)
    check("... and a detour is found, or the move refused", path is None or path[-1] == home, why)
    if path:
        pts = [b] + path
        check("the detour keeps the TCP inside the work zones", all(blocked(model, menv, w) is None for w in path))
        check("every leg of the detour is clear", all(segment_clear(model, menv, x, y) is None for x, y in zip(pts, pts[1:])))
    path, why = route(home, [-60.0, -90.0, 90.0, -90.0, -90.0, 0.0], env, model)
    check("turning J1 alone at HOME is a straight move", path is not None and len(path) == 1, why)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
