"""Gestures: expressive clips made from a hub pose towards interaction zones.

The dance phrases (choreo.py) move the whole arm between kinesphere poses;
seen in the room they read as the arm swinging at right angles, with the
wrist mostly along for the ride. People read a robot arm the way they read
a head and neck: where it LOOKS. Here every key is a tool position AND a
point the tool looks at, so the wrist (J4-J6, J5 above all) carries the
character, and small moves of the TCP read as leaning in or backing off.

Zones (robot base frame, boxes centre / size / yaw):

    audience   where people stand (outside the robot's reach) -- the points
               the arm looks at, greets, waves to
    greet      where the TCP performs for them (inside the stage)
    idle       the space around the rest hub, for looking around

A gesture starts and ends at its hub (at rest), so it chains in the show
graph like any clip. Families:

    look    glance from point to point in the audience, holds between,
            leaning a few cm towards each
    wave    the TCP swings sideways while the tool keeps aiming at one
            person and rolls with the swing
    nod     the gaze drops and comes back up (twice), the TCP dipping
    reach   the TCP moves out towards the audience (an offer), holds, returns
    tilt    the tool keeps aiming at someone and rolls to one side and the
            other, like a head tilting (J6, J5), holds
    trace   the TCP draws a loose path through the greet zone while the
            gaze drifts across the audience

    make(rig, hub_q, family, zones, rng, bpm, intensity) -> motion clip dict (or None)

Timing: keys at beat multiples (min-jerk between, holds); sampled at 24 fps
through the closed-form IK, nearest branch; slowed uniformly if the
player would slow it (fairino_player.limiting), never clipped. Pure Python.

    python scripts/gestures.py        self-test
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import capability as C  # noqa: E402
import fairino_player as P  # noqa: E402
import motion_clip as M  # noqa: E402
import robot_profile as RP  # noqa: E402
import ur_ik  # noqa: E402
import urdf_rig as U  # noqa: E402

FPS = 24.0
FAMILIES = ("look", "wave", "nod", "reach", "tilt", "trace")
LIMIT_MARGIN = 3.0
MAX_STEP_DEG = 30.0              # per 24 fps frame: more is a branch flip (~180), not a quick gesture; timing is fitted after
PLAN_SAFETY = 0.85


class Rig:
    def __init__(self):
        self.model, self.chain, self.fo, self.vel = C.load_fr20(ROOT)
        prof = RP.load("fr20")
        self.acc = RP.acceleration_limits(prof)
        self.vel = RP.velocity_limits(prof)
        self.limits = [tuple(x) for x in prof["robot"]["limits_deg"]]

    def tool(self, q):
        """(R, tcp, direction) of pose q."""
        R, p6 = ur_ik.pose_of(self.chain, q)
        tcp = U._add(p6, U._mat_vec(R, (0.0, 0.0, self.fo)))
        return R, tcp, (R[0][2], R[1][2], R[2][2])

    def solve(self, tcp, direction, roll, near, R=None):
        R = R or C.tool_frame(direction, roll)
        p6 = U._sub(tcp, U._mat_vec(R, (0.0, 0.0, self.fo)))
        sols = ur_ik.within_limits(self.model, ur_ik.solve(self.model, R, p6, q6_when_singular=near[5]))
        sols = [s for s in sols if all(lo + LIMIT_MARGIN < x < hi - LIMIT_MARGIN for x, (lo, hi) in zip(s["q"], self.limits))]
        best = ur_ik.nearest(sols, near)
        if best is None:
            return None
        return [b - 360.0 * round((b - a) / 360.0) for a, b in zip(near, best["q"])]


def roll_of(R, direction):
    """The roll r with capability.tool_frame(direction, r) == R (degrees)."""
    R0 = C.tool_frame(direction, 0.0)
    x0 = (R0[0][0], R0[1][0], R0[2][0])
    y0 = (R0[0][1], R0[1][1], R0[2][1])
    x = (R[0][0], R[1][0], R[2][0])
    return math.degrees(math.atan2(U._dot(x, y0), U._dot(x, x0)))


def transport(R, d, droll_deg=0.0):
    """R turned by the smallest rotation that takes its z axis onto d, then
    rolled droll about the new z: a tool frame carried along continuously
    (tool_frame(d, roll) switches its reference axis and jumps J6)."""
    z0 = (R[0][2], R[1][2], R[2][2])
    z1 = U._normalize(d)
    axis = U._cross(z0, z1)
    s, c = U._norm(axis), U._dot(z0, z1)
    cols = [(R[0][k], R[1][k], R[2][k]) for k in range(3)]
    if s > 1e-12:
        ang = math.atan2(s, c)
        M_ = U.axis_angle_matrix(U._normalize(axis), ang)
        cols = [U._mat_vec(M_, v) for v in cols]
    if droll_deg:
        Mr = U.axis_angle_matrix(cols[2], math.radians(droll_deg))
        cols = [U._mat_vec(Mr, cols[0]), U._mat_vec(Mr, cols[1]), cols[2]]
    return tuple((cols[0][i], cols[1][i], cols[2][i]) for i in range(3))


def _minjerk(u):
    u = max(0.0, min(1.0, u))
    return u * u * u * (10 - 15 * u + 6 * u * u)


def _lerp(a, b, u):
    return [x + (y - x) * u for x, y in zip(a, b)]


def box_point(box, rng, fx=(-0.5, 0.5), fy=(-0.5, 0.5), fz=(-0.5, 0.5)):
    """A random point in a (yawed) box, each axis within the given fractions."""
    c, s = box["center"], box["size"]
    yaw = math.radians(box.get("yaw_deg", 0.0))
    lx, ly, lz = s[0] * rng.uniform(*fx), s[1] * rng.uniform(*fy), s[2] * rng.uniform(*fz)
    return (c[0] + math.cos(yaw) * lx - math.sin(yaw) * ly, c[1] + math.sin(yaw) * lx + math.cos(yaw) * ly, c[2] + lz)


def inside(box, p, pad=0.0):
    c, s = box["center"], box["size"]
    yaw = math.radians(box.get("yaw_deg", 0.0))
    dx, dy, dz = p[0] - c[0], p[1] - c[1], p[2] - c[2]
    lx, ly = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
    return abs(lx) <= s[0] / 2 - pad and abs(ly) <= s[1] / 2 - pad and abs(dz) <= s[2] / 2 - pad


# --------------------------------------------------------------------------
# keys: (duration to reach, tcp, look point, roll) -- then sampled
# --------------------------------------------------------------------------

def _eyes(zones, rng):
    """A point people's faces are at: in the audience box, 1.3-1.7 m up."""
    a = zones["audience"]
    p = box_point(a, rng, fz=(-0.5, 0.5))
    return (p[0], p[1], min(max(1.3 + rng.random() * 0.4, a["center"][2] - a["size"][2] / 2), a["center"][2] + a["size"][2] / 2))


def _toward(p, target, dist):
    d = U._normalize(U._sub(target, p))
    return U._add(p, U._scale(d, dist))


def keys_for(family, home, zones, rng, beat, k):
    """Keys after the start: [(duration s, tcp, look, roll)], ending at home.
    home = (tcp, look, roll) of the hub; k = intensity 0..1."""
    p0, look0, r0 = home
    greet = zones.get("greet")
    keys = []
    hold = lambda: beat * rng.choice((0.5, 1.0, 1.0, 1.5))

    def clamp(p):
        if greet and not inside(greet, p, 0.02):
            return p0                                       # the greet zone is where the TCP may go
        return p
    if family == "look":
        for _ in range(rng.choice((2, 3))):
            e = _eyes(zones, rng)
            keys.append((beat * rng.choice((1, 1.5, 2)), clamp(_toward(p0, e, 0.04 + 0.06 * k)), e, r0 + rng.uniform(-15, 15) * k))
            keys.append((hold(), keys[-1][1], e, keys[-1][3]))
    elif family == "wave":
        e = _eyes(zones, rng)
        d = U._normalize(U._sub(e, p0))
        side = U._normalize(U._cross(d, (0.0, 0.0, 1.0)))
        amp = 0.06 + 0.08 * k
        keys.append((beat, clamp(_toward(p0, e, 0.05)), e, r0))
        for i in range(rng.choice((2, 3))):
            for sgn in (1, -1):
                keys.append((beat * 0.5, clamp(U._add(_toward(p0, e, 0.05), U._scale(side, sgn * amp))), e, r0 + sgn * 25 * k))
        keys.append((beat * 0.5, clamp(_toward(p0, e, 0.05)), e, r0))
        keys.append((hold(), keys[-1][1], e, r0))
    elif family == "nod":
        e = _eyes(zones, rng)
        down = (e[0], e[1], e[2] - 0.9)
        keys.append((beat, p0, e, r0))
        for _ in range(2):
            keys.append((beat * 0.5, clamp(U._add(_toward(p0, e, 0.03), (0.0, 0.0, -0.04 - 0.04 * k))), down, r0))
            keys.append((beat * 0.5, clamp(_toward(p0, e, 0.02)), e, r0))
        keys.append((hold(), keys[-1][1], e, r0))
    elif family == "reach":
        e = _eyes(zones, rng)
        out = _toward(p0, e, 0.15 + 0.2 * k)
        if greet and not inside(greet, out, 0.02):
            out = _toward(p0, e, 0.12)
        keys.append((beat * 1.5, clamp(out), e, r0 + rng.uniform(-20, 20)))
        keys.append((beat * 2, keys[-1][1], e, keys[-1][3]))
        keys.append((beat, clamp(_toward(p0, e, 0.05)), e, r0))
    elif family == "tilt":
        e = _eyes(zones, rng)
        keys.append((beat, clamp(_toward(p0, e, 0.03)), e, r0))
        for sgn in rng.sample((1, -1), 2):
            keys.append((beat, keys[-1][1], e, r0 + sgn * (25 + 20 * k)))
            keys.append((hold(), keys[-1][1], e, keys[-1][3]))
    elif family == "trace":
        zone = greet or zones.get("idle")
        e = _eyes(zones, rng)
        for _ in range(rng.choice((3, 4))):
            p = box_point(zone, rng, (-0.35, 0.35), (-0.35, 0.35), (-0.35, 0.35)) if zone else p0
            e = _eyes(zones, rng) if rng.random() < 0.5 else e
            keys.append((beat * rng.choice((1, 1.5)), p, e, r0 + rng.uniform(-20, 20) * k))
    else:
        raise ValueError("unknown family %r" % family)
    keys.append((beat * 1.5, p0, look0, r0))                  # home again
    keys.append((beat * 0.5, p0, look0, r0))                  # at rest
    return keys


def _frames(rig, hub_q, times, states, n, extra_roll=None):
    """[(t, p, R)] per 24 fps frame: the tool frame carried along (transport),
    plus extra_roll(t) degrees about z (the end correction)."""
    total = times[-1]
    out = []
    Rc, last = rig.tool(hub_q)[0], 0.0
    last_roll = states[0][2]
    for i in range(n + 1):
        t = min(i / FPS, total)
        j = max(0, min(len(times) - 2, next((k for k in range(len(times) - 1) if times[k + 1] >= t), len(times) - 2)))
        u = _minjerk((t - times[j]) / max(1e-9, times[j + 1] - times[j]))
        (pa, la, ra), (pb, lb, rb) = states[j], states[j + 1]
        p, look, roll = _lerp(pa, pb, u), _lerp(la, lb, u), ra + (rb - ra) * u
        ex = extra_roll(t) if extra_roll else 0.0
        if i:
            Rc = transport(Rc, U._sub(look, p), (roll - last_roll) + (ex - last))
        last_roll, last = roll, ex
        out.append((t, p, Rc))
    return out


def sample(rig, hub_q, keys, home):
    """24 fps joints through the keys (min-jerk between, tool aimed at the
    look point), starting exactly at hub_q. The frame is carried along, so
    a loop of directions can come back rolled (holonomy): the leftover roll
    is taken out over the way home. None when a key cannot be held or a
    step jumps (a branch flip)."""
    p0, look0, r0 = home
    times, states = [0.0], [(p0, look0, r0)]
    for dur, p, look, roll in keys:
        times.append(times[-1] + dur)
        states.append((p, look, roll))
    total = times[-1]
    n = int(math.ceil(total * FPS))
    first = _frames(rig, hub_q, times, states, n)
    R_end, R_hub = first[-1][2], rig.tool(hub_q)[0]
    x_end = (R_end[0][0], R_end[1][0], R_end[2][0])
    x_hub, y_hub = (R_hub[0][0], R_hub[1][0], R_hub[2][0]), (R_hub[0][1], R_hub[1][1], R_hub[2][1])
    left = -math.degrees(math.atan2(U._dot(x_end, y_hub), U._dot(x_end, x_hub)))
    t_home = times[-3] if len(times) >= 3 else 0.0            # the way home starts here
    fix = lambda t: left * _minjerk((t - t_home) / max(1e-9, total - t_home)) if t > t_home else 0.0
    frames = _frames(rig, hub_q, times, states, n, fix) if abs(left) > 1e-6 else first
    qs, prev = [], list(hub_q)
    for i, (t, p, R) in enumerate(frames):
        q = list(hub_q) if i == 0 else rig.solve(p, None, None, prev, R=R)
        if q is None or max(abs(a - b) for a, b in zip(q, prev)) > MAX_STEP_DEG:
            return None
        if abs(math.sin(math.radians(q[4]))) < 0.2:           # the wrist singularity: J5 near 0 / 180
            return None
        qs.append(q)
        prev = q
    if max(abs(a - b) for a, b in zip(qs[-1], hub_q)) > 0.5:
        return None
    qs[-1] = list(hub_q)
    return [i / FPS for i in range(n + 1)], qs


def hub_pose(rig, tcp, look, near, rolls=range(-180, 180, 10)):
    """A hub from where the tool tip is and what it looks at: the IK pose
    nearest `near` over rolls about the aim (so the arm keeps its elbow /
    wrist configuration). None when no roll reaches it clear of the wrist
    singularity."""
    d = U._sub(look, tcp)
    best = None
    for r in rolls:
        q = rig.solve(tcp, d, r, near)
        if q is None or abs(math.sin(math.radians(q[4]))) < 0.3:
            continue
        # a hub the wrist can move from: far from its singularity first
        # (near it the wrist spins for small changes of aim), then nearest
        cost = (abs(math.sin(math.radians(q[4]))) < 0.7, max(abs(a - b) for a, b in zip(q, near)))
        if best is None or cost < best[0]:
            best = (cost, q)
    return best and best[1]


def home_of(rig, hub_q):
    """(tcp, look point 1.5 m along the tool, roll) of a hub pose."""
    R, tcp, d = rig.tool(hub_q)
    return tcp, U._add(tcp, U._scale(d, 1.5)), roll_of(R, d)


def make(rig, hub_q, family, zones, rng, bpm=90, intensity=0.6, clip_id=None, env=None, safety=PLAN_SAFETY):
    """A gesture clip from hub_q, or None when this draw does not work
    (unreachable key, branch flip, too violent, hits the room)."""
    import collision as CL
    import motion_labels
    home = home_of(rig, hub_q)
    beat = 60.0 / bpm
    keys = keys_for(family, home, zones, rng, beat, intensity)
    # timing: the keys as designed; where the player would slow the clip,
    # stretch the KEY times (so the curve stays smooth) and sample again
    scale, s = 1.0, None
    for _ in range(4):
        got = sample(rig, hub_q, [(d * scale, p, l, r) for d, p, l, r in keys], home)
        if got is None:
            return None
        ts, qs = got
        s = P.limiting(ts, qs, 125.0, [v * safety for v in rig.vel], [a * safety for a in rig.acc])["scale_needed"]
        if s <= 1.0:
            break
        scale *= s * 1.03
        if scale > 2.5:
            return None                                      # would crawl: redraw rather than play it slowly
    else:
        return None
    clip = {"schema": M.SCHEMA, "id": clip_id or "gesture_%s" % family, "robot": "fr20",
            "joint_names": ["j%d" % i for i in range(1, 7)], "units": {"angle": "deg", "time": "s", "length": "m"},
            "points": [{"t": round(t, 6), "q": [round(x, 5) for x in q]} for t, q in zip(ts, qs)],
            "tcp": M._tcp_path("fr20", qs),
            "style": {"generator": "gestures.py", "primitive": family, "bpm": bpm, "intensity": intensity,
                      "slowed": round(scale, 3)},
            "meta": {"duration_s": round(ts[-1], 6), "tags": ["gesture", family], "source": {"generator": "gestures.py"}}}
    xs = list(zip(*clip["tcp"]))
    clip["meta"]["bounds"] = {"min": [min(a) for a in xs], "max": [max(a) for a in xs]}
    M.measure(clip, acc=rig.acc)
    if env is not None:
        rep = CL.check(CL.load_model("fr20"), env, ts, qs)
        clip["safety"]["collision"] = CL.describe(rep)
        clip["safety"]["min_clearance_m"] = rep["min_env_clearance_m"]
        clip["safety"]["min_self_clearance_m"] = rep["min_self_clearance_m"]
        if not rep["ok"]:
            return None
    clip["labels"] = motion_labels.label(clip)
    return clip


def wrist_share(qs):
    """Fraction of the joint travel done by J4-J6 (how much the wrist acts)."""
    travel = [sum(abs(b[j] - a[j]) for a, b in zip(qs, qs[1:])) for j in range(6)]
    return sum(travel[3:]) / max(1e-9, sum(travel))


def self_test():
    import json
    import random
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    rig = Rig()
    stage_home = [-60.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    R, tcp, d = rig.tool(stage_home)
    r = roll_of(R, d)
    q = rig.solve(tcp, d, r, stage_home)
    check("roll_of + solve give the hub pose back", q is not None and max(abs(a - b) for a, b in zip(q, stage_home)) < 1e-6, q)
    zones = {"audience": {"center": [-1.4, 1.2, 1.2], "size": [0.6, 1.2, 1.2]},
             "greet": {"center": [-0.7, 0.8, 1.1], "size": [0.6, 0.8, 0.6]}}
    hub = hub_pose(rig, (-0.62, 0.72, 1.15), zones["audience"]["center"], stage_home)
    check("a hub from a tool-tip position and a point to look at", hub is not None, hub and [round(x, 1) for x in hub])
    hub = [round(x, 3) for x in hub]
    env = json.load(open(os.path.join(ROOT, "envs", "volvox_lab.json")))
    made = {}
    for fam in FAMILIES:
        rng = random.Random(5)
        for attempt in range(12):
            c = make(rig, hub, fam, zones, rng, bpm=90, intensity=0.7, env=env)
            if c:
                made[fam] = c
                break
    check("every family makes a clip from the stage hub", len(made) == len(FAMILIES), sorted(made))
    ok_ends = all(c["points"][0]["q"] == hub and c["points"][-1]["q"] == hub for c in made.values())
    check("each starts and ends at the hub (chains in the show)", ok_ends)
    shares = {f: round(wrist_share([p["q"] for p in c["points"]]), 2) for f, c in made.items()}
    check("the wrist carries most of look / tilt / nod", all(shares.get(f, 0) > 0.5 for f in ("look", "tilt", "nod")), shares)
    j5 = {f: round(max(p["q"][4] for p in c["points"]) - min(p["q"][4] for p in c["points"]), 1) for f, c in made.items()}
    check("J5 moves (not a right-angle arm with a still wrist)", sum(v > 8 for v in j5.values()) >= 4, j5)
    check("every clip plays at its own speed", all(c["safety"]["playback_scale"] <= 1.0 + 1e-6 for c in made.values()),
          {f: c["safety"]["playback_scale"] for f, c in made.items()})
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
