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

The TCP stays in the zone its hub's tool tip is in (greet first; a 0.4 m
box around the tip when it is in none); keys outside are pulled onto the
zone's boundary. A hub that faces away from the audience looks at it turned
into its view (the audience's centre within GAZE_CONE_DEG * 0.6 of the
hub's aim, no look beyond GAZE_CONE_DEG).

A gesture starts and ends at its hub (at rest), so it chains in the show
graph like any clip: a quarter beat still at the start (the mirror of the
half beat at the end), then the first move eases out of rest (never
front-loaded), so the first and last 24 fps steps are zero and speed and
acceleration rise from zero at both joins. Families:

    look    glance from point to point in the audience, holds between,
            leaning a few cm towards each, the gaze leading the lean
    wave    the TCP swings sideways while the tool keeps aiming at one
            person and rolls with the swing, the roll trailing it
    nod     the gaze drops and comes back up (twice, the second smaller),
            the TCP dipping
    reach   leans back, then moves out towards the audience (an offer),
            holds, offers again a little further, returns
    tilt    the tool keeps aiming at someone and rolls to one side and the
            other, like a head tilting (J6, J5), holds
    trace   the TCP draws a loose path through the zone while the gaze
            drifts across the audience, trailing it
    peek    curious: creeps towards a person, cocks the wrist, pauses,
            darts back a little
    shy     the gaze drops away and the TCP retreats towards the robot,
            then it slowly looks back
    stretch a big slow reach up and out, the wrist rolling, like a yawn,
            then settles
    bounce  small up-down bobs on the beat, the gaze fixed on someone,
            like a head bob to music
    search  scans the audience side to side, quick look and hold, as if
            looking for someone; ends leaning towards one person

Animation inside the keys (all scaled by intensity k):

    anticipation   a small counter-move before a big move (leans back
                   before reaching out, dips before stretching up)
    overshoot      fast moves go past their target and settle back
    overlap        the gaze / roll has its own clock: it trails the TCP
                   (follow-through: the wrist catches up) or leads it (the
                   eyes turn first, the lean follows)
    ease           min-jerk on a warped clock per move: decisive (quick
                   start, long settle) or hesitant (slow start, late landing)
    breathing      a slow drift of the TCP (a few mm) and the gaze over the
                   whole clip, a breath a bar, faded in and out: holds are
                   never frozen

    make(rig, hub_q, family, zones, rng, bpm, intensity) -> motion clip dict (or None)

Timing: keys at beat multiples; sampled at 24 fps through IK that tracks
the previous frame (Newton from it, so the arm stays on its branch; the
closed form when that fails). Each move is then given the time its joints
need at the plan safety (fairino_player.need_profile, per key, rounded up
to a quarter beat) -- a move that would need more than MAX_STRETCH times
its beats, a key the arm cannot hold, a branch flip or the room drop the
gesture to a smaller size of itself (SIZES) before the draw is given up.
Pure Python.

    python scripts/gestures.py        self-test
"""

import bisect
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
FAMILIES = ("look", "wave", "nod", "reach", "tilt", "trace", "peek", "shy", "stretch", "bounce", "search")
LIMIT_MARGIN = 3.0
MAX_STEP_DEG = 30.0              # per 24 fps frame: more is a branch flip (~180), not a quick gesture; timing is fitted after
PLAN_SAFETY = 0.85
GAZE_CONE_DEG = 50.0             # the most the gaze turns from the hub's aim
SIZES = (1.0, 0.7, 0.45)         # a draw that does not work is tried smaller
MAX_STRETCH = 2.5                # a move needing more than this times its beats is too big for the tempo
BEATS = (9.0, 12.0)              # a gesture's length in beats (holds lengthened / shortened to fit)
RATE_HZ = 125.0                  # the player's rate (fairino_player)
UP = (0.0, 0.0, 1.0)


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

    def within(self, q):
        return all(lo + LIMIT_MARGIN < x < hi - LIMIT_MARGIN for x, (lo, hi) in zip(q, self.limits))

    def solve(self, tcp, direction, roll, near, R=None):
        R = R or C.tool_frame(direction, roll)
        p6 = U._sub(tcp, U._mat_vec(R, (0.0, 0.0, self.fo)))
        sols = ur_ik.within_limits(self.model, ur_ik.solve(self.model, R, p6, q6_when_singular=near[5]))
        sols = [s for s in sols if self.within(s["q"])]
        best = ur_ik.nearest(sols, near)
        if best is None:
            return None
        return [b - 360.0 * round((b - a) / 360.0) for a, b in zip(near, best["q"])]

    def track(self, tcp, R, prev):
        """The pose for (tcp, R) on prev's branch: Newton steps from prev
        (a frame away, so two or three), checked; the closed form nearest
        prev when they do not land (near a singularity, or past a limit)."""
        R = ur_ik.orthonormalize(R)
        p6 = U._sub(tcp, U._mat_vec(R, (0.0, 0.0, self.fo)))
        q = ur_ik._polish(self.chain, prev, R, p6, iterations=8)
        if max(abs(e) for e in ur_ik._pose_error(self.chain, q, R, p6)) < 1e-7 and self.within(q):
            return q
        return self.solve(tcp, None, None, prev, R=R)


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


def _ease(u, b=0.0):
    """Min-jerk from 0 to 1 on a warped clock u + b u (1 - u): b > 0 starts
    quick and settles long (decisive), b < 0 starts slow and lands late
    (hesitant). Zero speed and acceleration at both ends for |b| < 1."""
    u = max(0.0, min(1.0, u))
    return _minjerk(u + b * u * (1.0 - u))


def _bump(u):
    """0 -> 1 -> 0 over u in [0, 1], flat (speed and acceleration 0) at both ends."""
    u = max(0.0, min(1.0, u))
    return 64.0 * (u * (1.0 - u)) ** 3


def _lerp(a, b, u):
    return [x + (y - x) * u for x, y in zip(a, b)]


def _cap(v, most):
    """v shortened to at most `most` long."""
    n = U._norm(v)
    return v if n <= most or n < 1e-12 else U._scale(v, most / n)


def _angle(a, b):
    return math.degrees(math.atan2(U._norm(U._cross(a, b)), U._dot(a, b)))


def _turn(v, toward, deg):
    """v turned by deg towards `toward` (both directions; away when deg < 0)."""
    axis = U._cross(v, toward)
    if U._norm(axis) < 1e-9:
        return v
    return U._mat_vec(U.axis_angle_matrix(U._normalize(axis), math.radians(deg)), v)


def _side_of(d):
    """A horizontal unit vector across direction d (world x when d is vertical)."""
    s = U._cross(d, UP)
    return U._normalize(s) if U._norm(s) > 0.1 else (1.0, 0.0, 0.0)


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


def into(box, p, pad=0.0):
    """p moved onto the nearest point of the (yawed) box shrunk by pad."""
    c, s = box["center"], box["size"]
    yaw = math.radians(box.get("yaw_deg", 0.0))
    cy, sy = math.cos(yaw), math.sin(yaw)
    dx, dy, dz = p[0] - c[0], p[1] - c[1], p[2] - c[2]
    lx, ly = cy * dx + sy * dy, -sy * dx + cy * dy
    h = [max(0.0, x / 2.0 - pad) for x in s]
    lx, ly, dz = max(-h[0], min(h[0], lx)), max(-h[1], min(h[1], ly)), max(-h[2], min(h[2], dz))
    return (c[0] + cy * lx - sy * ly, c[1] + sy * lx + cy * ly, c[2] + dz)


def tcp_zone(zones, p):
    """The box the TCP performs in: the zone (not the audience) the hub's
    tool tip is in, greet first; a 0.4 m box around the tip when none is."""
    names = ["greet", "idle"] + sorted(k for k in zones if k not in ("greet", "idle", "audience"))
    for n in names:
        z = zones.get(n)
        if z and "center" in z and inside(z, p):
            return z
    return {"center": list(p), "size": [0.4, 0.4, 0.4], "yaw_deg": 0.0}


# --------------------------------------------------------------------------
# keys: (duration to reach, tcp, look point, roll, options) -- then sampled
# --------------------------------------------------------------------------

def _eyes(zones, rng):
    """A point people's faces are at: in the audience box, 1.3-1.7 m up."""
    a = zones["audience"]
    p = box_point(a, rng, fz=(-0.5, 0.5))
    return (p[0], p[1], min(max(1.3 + rng.random() * 0.4, a["center"][2] - a["size"][2] / 2), a["center"][2] + a["size"][2] / 2))


def _toward(p, target, dist):
    d = U._normalize(U._sub(target, p))
    return U._add(p, U._scale(d, dist))


class _View:
    """Where people are, seen from the hub: faces in the audience zone; the
    whole audience turned (about the hub's tool tip) into the hub's view
    when its centre lies more than 0.6 GAZE_CONE_DEG off the hub's aim, and
    no look further out than GAZE_CONE_DEG."""

    def __init__(self, zones, p0, look0):
        self.zones, self.p0 = zones, p0
        self.d0 = U._normalize(U._sub(look0, p0))
        self.turn = None
        a = zones.get("audience")
        if a:
            dc = U._normalize(U._sub(a["center"], p0))
            ang = _angle(self.d0, dc)
            if ang > 0.6 * GAZE_CONE_DEG and U._norm(U._cross(dc, self.d0)) > 1e-9:
                self.turn = U.axis_angle_matrix(U._normalize(U._cross(dc, self.d0)), math.radians(ang - 0.6 * GAZE_CONE_DEG))

    def in_view(self, e):
        v = U._sub(e, self.p0)
        ang = _angle(self.d0, v)
        if ang > GAZE_CONE_DEG:
            v = _turn(v, self.d0, ang - GAZE_CONE_DEG)
        return U._add(self.p0, v)

    def eyes(self, rng):
        if not self.zones.get("audience"):
            v = _turn(U._scale(self.d0, 1.5), _side_of(self.d0), rng.uniform(-0.6, 0.6) * GAZE_CONE_DEG)
            return U._add(self.p0, _turn(v, UP, rng.uniform(-0.3, 0.3) * GAZE_CONE_DEG))
        e = _eyes(self.zones, rng)
        if self.turn:
            e = U._add(self.p0, U._mat_vec(self.turn, U._sub(e, self.p0)))
        return self.in_view(e)


class _Draw:
    """The keys of one gesture as it is drawn: the current tool tip, look
    point and roll, and moves from them. Durations are in beats here."""

    def __init__(self, home, zones, rng, beat, k):
        self.p0, self.look0, self.r0 = home
        self.p, self.look, self.roll = home
        self.rng, self.beat, self.k = rng, beat, k
        self.zone = tcp_zone(zones, self.p0)
        self.view = _View(zones, self.p0, self.look0)
        self.keys = []
        self.over = 0.08 + 0.17 * k           # overshoot, fraction of the move
        self.anti = 0.08 + 0.17 * k           # anticipation, fraction of the move
        self.lag = beat * (0.1 + 0.25 * k)    # overlap of the gaze / roll, s
        # the still start, as the end's rest: a min-jerk move straight from
        # the first frame steps ~10 u^3 of the move at once (0.03-0.1 deg
        # for these); from rest here, the first move eases in from a frame at rest
        self.key(max(0.25, 2.0 / (FPS * beat)), ease=0.0)
        self.keys[-1][4]["lead"] = True

    def clamp(self, p):
        return p if inside(self.zone, p, 0.02) else into(self.zone, p, 0.02)

    def eyes(self):
        return self.view.eyes(self.rng)

    def gaze(self, p=None, look=None):
        p, look = p or self.p, look or self.look
        return U._normalize(U._sub(look, p))

    def ease(self):
        """A varied slow-in / slow-out for an ordinary move."""
        return self.rng.uniform(-0.25, 0.35) * (0.4 + 0.6 * self.k)

    def target(self, p=None, look=None, roll=None):
        """(tcp, look, roll) of a key: the TCP kept in its zone, the look in
        view; what is not given stays (the gaze keeps its direction)."""
        p = self.clamp(p) if p is not None else self.p
        look = self.view.in_view(look) if look is not None else U._add(p, U._sub(self.look, self.p))
        return p, look, self.roll if roll is None else roll

    def key(self, beats, p=None, look=None, roll=None, ease=None, lag=0.0, hold=False):
        p, look, roll = self.target(p, look, roll)
        if len(self.keys) == 1:                             # the first move: out of rest gently, never front-loaded
            ease = min(0.0, self.ease() if ease is None else ease)
        self.keys.append((beats, p, look, roll, {"ease": self.ease() if ease is None else ease, "lag": lag, "hold": hold}))
        self.p, self.look, self.roll = p, look, roll

    def hold(self, beats):
        self.key(beats, ease=0.0, hold=True)

    def _past(self, p, look, roll, frac, away=False):
        """A state past the target (overshoot) or before the start, away
        from the target (anticipation): frac of the move, at most 4 cm,
        10 degrees of gaze, 10 degrees of roll."""
        sgn = -1.0 if away else 1.0
        base_p, base_d, base_r = (self.p, self.gaze(), self.roll) if away else (p, self.gaze(p, look), roll)
        dp = _cap(U._scale(U._sub(p, self.p), sgn * frac), 0.04)
        q = self.clamp(U._add(base_p, dp))
        d_from, d_to = self.gaze(), self.gaze(p, look)
        ang = min(10.0, frac * _angle(d_from, d_to))
        # anticipation turns the gaze away from where it is going; overshoot
        # turns it on past, away from where it came from
        d = _turn(base_d, d_to, -ang) if away else _turn(base_d, d_from, -ang)
        dist = U._norm(U._sub(look, p))
        r = base_r + sgn * max(-10.0, min(10.0, frac * (roll - self.roll)))
        return q, U._add(q, U._scale(d, dist)), r

    def anticipate(self, beats, p=None, look=None, roll=None):
        """A small counter-move before a big one: away from the target."""
        q, lq, rq = self._past(*self.target(p, look, roll), frac=self.anti, away=True)
        self.key(beats, q, lq, rq, ease=-0.2)

    def arrive(self, beats, p=None, look=None, roll=None, lag=0.0, settle=0.5, ease=0.35):
        """A quick move that goes past its target and settles back."""
        p, look, roll = self.target(p, look, roll)
        q, lq, rq = self._past(p, look, roll, self.over)
        self.key(beats, q, lq, rq, ease=ease, lag=lag)
        self.key(settle, p, look, roll, ease=-0.15, lag=lag * 0.5)

    def home(self, lag):
        """Back to the hub (the wrist settling last), then at rest."""
        self.key(1.5, self.p0, self.look0, self.r0, ease=0.1, lag=lag)
        self.keys[-1][4]["home"] = True
        self.key(0.5, self.p0, self.look0, self.r0, ease=0.0, lag=0.0)
        self.keys[-1][4]["home"] = True


def _fit_beats(keys, lo, hi):
    """Holds lengthened (or shortened, not under half a beat) so the keys
    add up to lo..hi beats; a hold added before the way home when there are
    none to lengthen."""
    total = sum(k[0] for k in keys)
    holds = [i for i, k in enumerate(keys) if k[4].get("hold")]
    if total < lo:
        if not holds:
            i = next(i for i, k in enumerate(keys) if k[4].get("home"))
            keys.insert(i, (0.0, keys[i - 1][1], keys[i - 1][2], keys[i - 1][3], {"ease": 0.0, "lag": 0.0, "hold": True}))
            holds = [i]
        add = (lo - total) / len(holds)
        for i in holds:
            keys[i] = (keys[i][0] + add,) + keys[i][1:]
    elif total > hi and holds:
        room = sum(keys[i][0] - 0.5 for i in holds)
        cut = min(total - hi, room)
        for i in holds:
            share = (keys[i][0] - 0.5) / room if room > 1e-9 else 0.0
            keys[i] = (keys[i][0] - cut * share,) + keys[i][1:]
    return keys


def keys_for(family, home, zones, rng, beat, k):
    """Keys after the start: [(duration s, tcp, look, roll, options)], ending
    at home at rest. home = (tcp, look, roll) of the hub; k = intensity 0..1.
    options: ease (the warp of the min-jerk clock), lag (s the gaze / roll
    trails the TCP; < 0 leads it), hold (a hold: may be lengthened)."""
    g = _Draw(home, zones, rng, beat, k)
    p0, r0 = g.p0, g.r0
    hold = lambda: rng.choice((0.5, 1.0, 1.0, 1.5))
    if family == "look":
        for _ in range(rng.choice((2, 3))):
            e = g.eyes()
            p = _toward(p0, e, 0.04 + 0.06 * k)
            g.arrive(rng.choice((1, 1.5)), p, e, r0 + rng.uniform(-15, 15) * k, lag=-g.lag)   # the eyes lead
            g.hold(hold())
    elif family == "wave":
        e = g.eyes()
        base = _toward(p0, e, 0.05)
        side = _side_of(U._sub(e, base))
        amp, ramp = 0.03 + 0.04 * k, 15 + 15 * k
        first = rng.choice((1, -1))
        g.key(1, base, e, r0, lag=-g.lag * 0.5)
        g.anticipate(0.5, U._add(base, U._scale(side, first * amp)), e, r0 + first * ramp)
        for i in range(rng.choice((3, 4))):
            sgn = first if i % 2 == 0 else -first
            g.key(1, U._add(base, U._scale(side, sgn * amp)), e, r0 + sgn * ramp, ease=0.0, lag=g.lag)  # the roll trails the swing
        g.arrive(1, base, e, r0, lag=g.lag)
        g.hold(1)
    elif family == "nod":
        e = g.eyes()
        g.arrive(1, _toward(p0, e, 0.03), e, r0, lag=-g.lag)
        for depth in (1.0, 0.65):
            dip = (12.0 + 12.0 * k) * depth
            pd = U._add(_toward(p0, e, 0.03), (0.0, 0.0, -(0.02 + 0.03 * k) * depth))
            d = _turn(U._sub(e, pd), (0.0, 0.0, -1.0), dip)
            g.key(1, pd, U._add(pd, d), r0, ease=0.2, lag=g.lag * 0.5)
            g.arrive(1, _toward(p0, e, 0.02), e, r0, lag=g.lag * 0.5)
        g.hold(1.5)
    elif family == "reach":
        e = g.eyes()
        out = _toward(p0, e, 0.15 + 0.2 * k)
        roll = r0 + rng.uniform(-20, 20)
        g.anticipate(0.75, out, e, roll)                    # leans back first
        g.arrive(1.5, out, e, roll, lag=g.lag)              # the wrist follows through
        g.hold(1.5)
        further = _toward(g.p, e, 0.03 + 0.04 * k)
        g.key(0.75, further, e, roll + rng.choice((-1, 1)) * (8 + 10 * k), ease=0.2, lag=g.lag)   # "here"
        g.hold(1)
        g.key(1, _toward(p0, e, 0.05), e, r0, lag=g.lag)
    elif family == "tilt":
        e = g.eyes()
        g.arrive(1, _toward(p0, e, 0.03), e, r0, lag=-g.lag)
        base = g.p
        side = _side_of(U._sub(e, base))
        for sgn in rng.sample((1, -1), 2):
            g.arrive(1, U._add(base, U._scale(side, sgn * (0.01 + 0.015 * k))), e, r0 + sgn * (25 + 20 * k), lag=g.lag)
            g.hold(hold())
    elif family == "trace":
        e = g.eyes()
        for _ in range(rng.choice((3, 4))):
            p = U._add(p0, _cap(U._sub(box_point(g.zone, rng, (-0.35, 0.35), (-0.35, 0.35), (-0.35, 0.35)), p0), 0.25))
            e = g.eyes() if rng.random() < 0.5 else e
            g.key(rng.choice((1, 1.5)), p, e, r0 + rng.uniform(-20, 20) * k, lag=g.lag)
        g.hold(0.5)
    elif family == "peek":
        e = g.eyes()
        near = _toward(p0, e, 0.10 + 0.12 * k)
        g.key(2, near, e, r0, ease=-0.3, lag=g.lag)                 # creeps in
        cock = rng.choice((-1, 1)) * (20 + 20 * k)
        g.arrive(0.75, near, e, r0 + cock, lag=g.lag)               # cocks the wrist
        g.hold(1.5)
        g.arrive(0.5, _toward(p0, e, 0.03 + 0.04 * k), e, r0 + cock * 0.5, ease=0.5)   # darts back a little
        g.hold(1)
    elif family == "shy":
        e = g.eyes()
        g.key(1, _toward(p0, e, 0.03), e, r0, lag=-g.lag)
        d = U._normalize(U._sub(e, g.p))
        away = _turn(_turn(d, (0.0, 0.0, -1.0), 30 + 15 * k), _side_of(d), rng.choice((-1, 1)) * (12 + 12 * k))
        back = U._normalize((-p0[0], -p0[1], 0.0)) if math.hypot(p0[0], p0[1]) > 1e-6 else (0.0, 0.0, 0.0)
        pr = U._add(p0, U._add(U._scale(back, 0.05 + 0.06 * k), (0.0, 0.0, -0.02 - 0.02 * k)))
        g.key(1, g.p, U._add(g.p, U._scale(away, 1.3)), r0 - 10 * k, ease=0.3, lag=-g.lag)   # the gaze drops away first
        g.key(1.5, pr, U._add(pr, U._scale(away, 1.3)), r0 - 15 * k, ease=0.0, lag=g.lag)      # and it shrinks back
        g.hold(1.5)
        half = U._normalize(_lerp(away, U._normalize(U._sub(e, pr)), 0.5))
        g.key(1.5, pr, U._add(pr, U._scale(half, 1.3)), r0 - 5 * k, ease=-0.3)                 # a glance back
        g.hold(0.75)
        g.key(1.5, _toward(p0, e, 0.02), e, r0, ease=-0.3, lag=g.lag)
        g.hold(0.5)
    elif family == "stretch":
        e = g.eyes()
        d = U._normalize(U._sub(e, p0))
        out = 0.08 + 0.08 * k
        top = g.clamp(U._add(p0, (d[0] * out, d[1] * out, 0.12 + 0.12 * k)))    # up and out
        up = _turn(d, UP, 25 + 20 * k)
        roll = r0 + rng.choice((-1, 1)) * (30 + 25 * k)
        g.anticipate(0.75, top, U._add(top, U._scale(up, 1.5)), roll)       # dips first
        g.key(3, top, U._add(top, U._scale(up, 1.5)), roll, ease=-0.2, lag=g.lag * 2)   # the long yawn, the wrist rolling after
        g.hold(1)
        g.arrive(2, _toward(p0, e, 0.02), e, r0, lag=g.lag, ease=0.0, settle=0.75)            # settles, a little sag
    elif family == "bounce":
        e = g.eyes()
        base = _toward(p0, e, 0.03)
        g.key(1, base, e, r0, lag=-g.lag)
        depth = 0.015 + 0.02 * k
        sway = 3 + 5 * k
        for i in range(rng.choice((5, 6))):
            sgn = 1 if i % 2 == 0 else -1
            g.key(0.5, U._add(base, (0.0, 0.0, -depth)), e, r0 + sgn * sway, ease=0.3, lag=g.lag * 0.4)   # down on the beat
            g.key(0.5, base, e, r0, ease=-0.2, lag=g.lag * 0.4)
        g.hold(1)
    elif family == "search":
        es = sorted((g.eyes() for _ in range(rng.choice((3, 4)))),
                    key=lambda x: U._dot(U._sub(x, p0), _side_of(g.view.d0)))
        if rng.random() < 0.5:
            es.reverse()
        g.key(1.5, _toward(p0, es[0], 0.02), es[0], r0, lag=-g.lag)          # to one end of the room
        g.hold(0.5)
        for e in es[1:]:
            g.arrive(1, _toward(p0, e, 0.02), e, r0 + rng.uniform(-10, 10) * k, lag=-g.lag, ease=0.3)   # quick look
            g.hold(rng.choice((0.5, 0.75, 1.0)))
        e = rng.choice(es)
        g.key(1, _toward(p0, e, 0.05 + 0.05 * k), e, r0 + rng.choice((-1, 1)) * (10 + 10 * k), lag=-g.lag)   # found you
        g.hold(1.5)
    else:
        raise ValueError("unknown family %r" % family)
    g.home(min(g.lag, 0.25 * beat))
    keys = _fit_beats(g.keys, *BEATS)
    return [(b * beat, p, look, roll, opt) for b, p, look, roll, opt in keys]


def shrink(keys, home, size):
    """The same gesture smaller: every key's tool tip, aim and roll moved
    towards the hub's by 1 - size (the look points keep their distance)."""
    p0, look0, r0 = home
    d0 = U._normalize(U._sub(look0, p0))
    out = []
    for key in keys:
        dur, p, look, roll = key[:4]
        q = _lerp(p0, p, size)
        d = U._normalize(U._sub(look, p))
        d2 = U._normalize(_lerp(d0, d, size))
        out.append((dur, tuple(q), U._add(q, U._scale(d2, U._norm(U._sub(look, p)))), r0 + (roll - r0) * size) + tuple(key[4:]))
    return out


def style_for(rng, beat, k):
    """The breathing of one clip: a slow drift of the TCP and the gaze, a
    breath a bar, a few mm and under a degree, bigger with intensity."""
    return {"breath_m": 0.002 + 0.004 * k, "gaze_m": 0.008 + 0.014 * k, "hz": 1.0 / (4.0 * beat),
            "phase": [rng.uniform(0, 2 * math.pi) for _ in range(4)]}


def _gaze_times(tp, lags):
    """The gaze / roll clock: each key's time shifted by its lag (at most
    0.6 of the move before or after it, never backwards), the last at the end."""
    tg = [0.0]
    n = len(tp)
    for j in range(1, n):
        before = tp[j] - tp[j - 1]
        after = tp[j + 1] - tp[j] if j + 1 < n else 0.0
        lag = max(-0.6 * before, min(0.6 * after, lags[j]))
        tg.append(max(tg[-1] + 0.3 * before, tp[j] + lag))
    tg[-1] = tp[-1]
    if tg[-2] >= tg[-1]:
        tg[-2] = tp[-2]
    return tg


def _at(times, values, eases, t):
    """A channel at time t: min-jerk (eased) between keyed values."""
    j = max(0, min(len(times) - 2, bisect.bisect_right(times, t) - 1))
    u = _ease((t - times[j]) / max(1e-9, times[j + 1] - times[j]), eases[j + 1])
    a, b = values[j], values[j + 1]
    if isinstance(a, float):
        return a + (b - a) * u
    return _lerp(a, b, u)


def _frames(rig, hub_q, keys, home, style, n, total, extra_roll=None):
    """[(t, p, R)] per 24 fps frame: the TCP on its clock, the look point and
    roll on theirs (the lags), plus the breathing; the tool frame carried
    along (transport), plus extra_roll(t) degrees about z (the end correction)."""
    p0, look0, r0 = home
    tp, P_, L_, R_, E_, lags = [0.0], [list(p0)], [list(look0)], [float(r0)], [0.0], [0.0]
    for dur, p, look, roll, opt in keys:
        tp.append(tp[-1] + dur)
        P_.append(list(p))
        L_.append(list(look))
        R_.append(float(roll))
        E_.append(opt.get("ease", 0.0))
        lags.append(opt.get("lag", 0.0))
    tg = _gaze_times(tp, lags)
    t_still = tp[-2]                                           # the rest at the end
    t_lead = tp[1] if keys and keys[0][4].get("lead") else 0.0  # the still start
    d0 = U._normalize(U._sub(look0, p0))
    side, up = _side_of(d0), U._normalize(U._cross(_side_of(d0), d0))
    ramp = min(1.0, (t_still - t_lead) / 3.0)
    ph = style["phase"] if style else [0.0] * 4
    w2 = 2.0 * math.pi * (style["hz"] if style else 0.0)
    out = []
    Rc, last, last_roll = rig.tool(hub_q)[0], 0.0, float(r0)
    for i in range(n + 1):
        t = min(i / FPS, total)
        p = _at(tp, P_, E_, t)
        look = _at(tg, L_, E_, t)
        roll = _at(tg, R_, E_, t)
        if style and t_lead < t < t_still:
            w = _minjerk(min(t - t_lead, t_still - t) / ramp)
            bp, bl = style["breath_m"] * w, style["gaze_m"] * w
            p = U._add(p, U._add(U._scale(UP, bp * math.sin(w2 * t + ph[0])),
                                 U._scale(side, 0.5 * bp * math.sin(0.62 * w2 * t + ph[1]))))
            look = U._add(look, U._add(U._scale(side, bl * math.sin(0.8 * w2 * t + ph[2])),
                                       U._scale(up, 0.6 * bl * math.sin(0.53 * w2 * t + ph[3]))))
        ex = extra_roll(t) if extra_roll else 0.0
        if i:
            Rc = transport(Rc, U._sub(look, p), (roll - last_roll) + (ex - last))
        last_roll, last = roll, ex
        out.append((t, p, Rc))
    return out, tp, tg


def _as5(keys):
    return [tuple(k) if len(k) >= 5 else tuple(k) + ({},) for k in keys]


def sample(rig, hub_q, keys, home, style=None):
    """24 fps joints through the keys (tool aimed at the look point), starting
    exactly at hub_q. The frame is carried along, so a loop of directions
    can come back rolled (holonomy): the leftover roll is taken out over the
    way home. None when a key cannot be held or a step jumps (a branch flip)."""
    keys = _as5(keys)
    total = sum(k[0] for k in keys)
    n = int(math.ceil(total * FPS))
    first, _, tg = _frames(rig, hub_q, keys, home, style, n, total)
    R_end, R_hub = first[-1][2], rig.tool(hub_q)[0]
    x_end = (R_end[0][0], R_end[1][0], R_end[2][0])
    x_hub, y_hub = (R_hub[0][0], R_hub[1][0], R_hub[2][0]), (R_hub[0][1], R_hub[1][1], R_hub[2][1])
    left = -math.degrees(math.atan2(U._dot(x_end, y_hub), U._dot(x_end, x_hub)))
    i_home = next((i for i, k in enumerate(keys) if k[4].get("home")), len(keys) - 2)
    t_home, t_end = tg[max(0, i_home)], tg[-2] if tg[-2] > tg[max(0, i_home)] else total
    fix = lambda t: left * _minjerk((t - t_home) / max(1e-9, t_end - t_home)) if t > t_home else 0.0
    frames = _frames(rig, hub_q, keys, home, style, n, total, fix)[0] if abs(left) > 1e-6 else first
    qs, prev = [], list(hub_q)
    for i, (t, p, R) in enumerate(frames):
        q = list(hub_q) if i == 0 else rig.track(p, R, prev)
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


def fit(rig, hub_q, keys, home, style, beat, vel, acc, rounds=5):
    """(ts, qs, keys) with each move given the time its joints need: the
    player's need (v / limit, sqrt(a / limit)) over each key's span, that
    key's duration stretched by it and rounded up to a quarter beat, until
    nothing needs slowing. None when a key cannot be sampled or a move
    would need more than MAX_STRETCH times its beats."""
    keys = _as5(keys)
    base = [k[0] for k in keys]
    quarter = beat / 4.0
    for _ in range(rounds):
        got = sample(rig, hub_q, keys, home, style)
        if got is None:
            return None
        ts, qs = got
        prof = P.need_profile(ts, qs, RATE_HZ, vel, acc)
        worst = max(nd for _, nd in prof)
        if worst <= 1.0:
            return ts, qs, keys
        tp = [0.0]
        for k in keys:
            tp.append(tp[-1] + k[0])
        tg = _gaze_times(tp, [0.0] + [k[4].get("lag", 0.0) for k in keys])
        need = [1.0] * len(keys)
        for t, nd in prof:                                      # the key moving the TCP and the one turning the gaze
            for clock in (tp, tg):
                j = max(0, min(len(keys) - 1, bisect.bisect_right(clock, t) - 1))
                need[j] = max(need[j], nd)
        new = []
        for j, k in enumerate(keys):
            d = k[0]
            if need[j] > 1.0:
                d = math.ceil(d * need[j] * 1.04 / quarter - 1e-9) * quarter
                if d > MAX_STRETCH * base[j] + 1e-9:
                    return None
            new.append((d,) + tuple(k[1:]))
        keys = new
    return None


def room_check(model, env, ts, qs, far=0.3):
    """collision.check on the objects that can matter. A shape whose bound
    (the distance at each link capsule's middle less half its length and its
    radius -- distances change no faster than the point moves) stays more
    than its margin + far from every capsule in every frame can neither be
    hit nor be the closest, so it is left out; the full check when nothing
    left comes within far. The same report, much sooner."""
    import collision as CL
    margin = float(env.get("margin_m", 0.05))
    objs = env.get("objects", [])
    hard = [i for i, o in enumerate(objs) if o["role"] in ("obstacle", "keep_out")]
    bound = {i: math.inf for i in hard}
    for q in qs:
        caps, _ = CL.capsules(model, q)
        for i in hard:
            o = objs[i]
            for name, a, b, r in caps:
                if name in CL.FIXED_LINKS:
                    continue
                mid = tuple((x + y) / 2.0 for x, y in zip(a, b))
                bound[i] = min(bound[i], CL.sdf(o, mid) - math.dist(a, b) / 2.0 - r)
    m = lambda o: o.get("margin_m", margin) if o["role"] == "obstacle" else 0.0
    keep = [o for i, o in enumerate(objs) if i not in bound or bound[i] - m(o) < far]
    rep = CL.check(model, dict(env, objects=keep), ts, qs)
    c = rep.get("min_env_clearance_m")
    if c is None or c > far:
        rep = CL.check(model, env, ts, qs)
    return rep


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
    """A gesture clip from hub_q, or None when this draw does not work at
    any size (unreachable key, branch flip, too violent, hits the room)."""
    import collision as CL
    import motion_labels
    home = home_of(rig, hub_q)
    beat = 60.0 / bpm
    keys = keys_for(family, home, zones, rng, beat, intensity)
    style = style_for(rng, beat, intensity)
    vel, acc = [v * safety for v in rig.vel], [a * safety for a in rig.acc]
    nominal = sum(k[0] for k in keys)
    model = CL.load_model("fr20") if env is not None else None
    for size in SIZES:
        got = fit(rig, hub_q, keys if size == 1.0 else shrink(keys, home, size), home, style, beat, vel, acc)
        if got is None:
            continue
        ts, qs, fitted = got
        rep = room_check(model, env, ts, qs) if env is not None else None
        if rep is None or rep["ok"]:
            break
    else:
        return None
    clip = {"schema": M.SCHEMA, "id": clip_id or "gesture_%s" % family, "robot": "fr20",
            "joint_names": ["j%d" % i for i in range(1, 7)], "units": {"angle": "deg", "time": "s", "length": "m"},
            "points": [{"t": round(t, 6), "q": [round(x, 5) for x in q]} for t, q in zip(ts, qs)],
            "tcp": M._tcp_path("fr20", qs),
            "style": {"generator": "gestures.py", "primitive": family, "bpm": bpm, "intensity": intensity,
                      "slowed": round(ts[-1] / nominal, 3), "size": size},
            "meta": {"duration_s": round(ts[-1], 6), "tags": ["gesture", family], "source": {"generator": "gestures.py"}}}
    xs = list(zip(*clip["tcp"]))
    clip["meta"]["bounds"] = {"min": [min(a) for a in xs], "max": [max(a) for a in xs]}
    M.measure(clip, acc=rig.acc)
    if rep is not None:
        clip["safety"]["collision"] = CL.describe(rep)
        clip["safety"]["min_clearance_m"] = rep["min_env_clearance_m"]
        clip["safety"]["min_self_clearance_m"] = rep["min_self_clearance_m"]
    clip["labels"] = motion_labels.label(clip)
    return clip


def wrist_share(qs):
    """Fraction of the joint travel done by J4-J6 (how much the wrist acts)."""
    travel = [sum(abs(b[j] - a[j]) for a, b in zip(qs, qs[1:])) for j in range(6)]
    return sum(travel[3:]) / max(1e-9, sum(travel))


# --------------------------------------------------------------------------
# self-test: the hubs of shows/party.json, every family, a sweep of seeds
# --------------------------------------------------------------------------

SEEDS = 10                       # draws per family and hub in the sweep
TRIES = 3                        # a seed's draws before it counts as not made
STILL_DEG = 0.005                # a frame step under this on every joint is "frozen"
REST_STEP_DEG = 0.005            # the first and last frame steps, at most (clips join at rest)


def _room():
    """The lab: envs/volvox_lab.usda (OpenUSD), or the .json it came from."""
    import collision as CL
    errs = []
    for name in ("volvox_lab.usda", "volvox_lab.json"):
        path = os.path.join(ROOT, "envs", name)
        if not os.path.exists(path):
            continue
        try:
            return CL.load_env(path)
        except (ValueError, ImportError) as e:
            errs.append("%s: %s" % (name, e))
    raise SystemExit("no room to check against: %s" % ("; ".join(errs) or "envs/volvox_lab.* missing"))


def _show_hubs(rig):
    """(zones, {name: q}) of shows/party.json, the hubs resolved as
    show.resolve_hubs does (a tool tip + look point through hub_pose)."""
    import json
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    hubs = {}
    for name, h in cfg["hubs"].items():
        if h.get("tcp") and h.get("look"):
            near = h.get("near") or h.get("q") or [-60.0, -90.0, 90.0, -90.0, -90.0, 0.0]
            q = hub_pose(rig, h["tcp"], h["look"], near)
            if q is None:
                raise SystemExit("hub %s: no pose puts the tool at %s looking at %s" % (name, h["tcp"], h["look"]))
            hubs[name] = [round(x, 4) for x in q]
        else:
            hubs[name] = list(h["q"])
    return cfg["zones"], hubs


_W = {}


def _worker_init():
    import collision as CL
    rig = Rig()
    zones, hubs = _show_hubs(rig)
    _W.update(rig=rig, zones=zones, hubs=hubs, env=_room(), model=CL.load_model("fr20"))


def _still_run(ts, qs, t_from, t_to):
    """The longest stretch (s) in t_from..t_to where no joint moves more than STILL_DEG a frame."""
    best = run = 0.0
    for (ta, a), (tb, b) in zip(zip(ts, qs), zip(ts[1:], qs[1:])):
        if ta < t_from or tb > t_to:
            run = 0.0
            continue
        run = run + (tb - ta) if max(abs(x - y) for x, y in zip(a, b)) < STILL_DEG else 0.0
        best = max(best, run)
    return best


def _trial(job):
    """One seed of one family from one hub: up to TRIES draws (show.py's
    bpm and intensity ranges), the first clip checked. A summary, not the clip."""
    import random
    fam, hub_name, seed = job
    rig, hub = _W["rig"], _W["hubs"][hub_name]
    rng = random.Random(seed)
    out = {"fam": fam, "hub": hub_name, "seed": seed, "first": False, "made": False, "problems": []}
    for i in range(TRIES):
        bpm, k = rng.randint(80, 125), rng.uniform(0.4, 0.9)
        c = make(rig, hub, fam, _W["zones"], rng, bpm=bpm, intensity=k, env=_W["env"])
        if c is None:
            continue
        ts, qs = [p["t"] for p in c["points"]], [p["q"] for p in c["points"]]
        bad = out["problems"]
        if max(abs(a - b) for a, b in zip(qs[0], hub)) > 1e-3 or max(abs(a - b) for a, b in zip(qs[-1], hub)) > 1e-3:
            bad.append("does not start and end at the hub")
        # at rest at both ends (the stream joins clips at the hubs): the first
        # and last frame steps under REST_STEP_DEG, and out of rest (into it)
        # smoothly: over the first and last half second no step changes by
        # more than a joint's acceleration limit allows in a frame
        first, last = (max(abs(a - b) for a, b in zip(qs[1], qs[0])), max(abs(a - b) for a, b in zip(qs[-1], qs[-2])))
        out["first_step"], out["last_step"] = first, last
        if first > REST_STEP_DEG or last > REST_STEP_DEG:
            bad.append("not at rest at the ends (steps %.4f, %.4f deg)" % (first, last))
        n_half = int(0.5 * FPS)
        for f in list(range(1, n_half)) + list(range(len(qs) - n_half, len(qs) - 1)):
            if any(abs(qs[f + 1][j] - 2 * qs[f][j] + qs[f - 1][j]) * FPS ** 2 > rig.acc[j] * PLAN_SAFETY * 1.05 for j in range(6)):
                bad.append("jumps out of (into) rest at frame %d" % f)
                break
        if any(not lo <= x <= hi for q in qs for x, (lo, hi) in zip(q, rig.limits)):
            bad.append("a joint past its limit")
        lim = P.limiting(ts, qs, RATE_HZ, [v * PLAN_SAFETY for v in rig.vel], [a * PLAN_SAFETY for a in rig.acc])
        if lim["scale_needed"] > 1.0 + 1e-6:
            bad.append("needs slowing x%.3f (J%d %s)" % (lim["scale_needed"], lim["joint"], lim["kind"]))
        rep = room_check(_W["model"], _W["env"], ts, qs)
        if not rep["ok"]:
            bad.append("hits the room")
        beat = 60.0 / bpm
        out.update(first=i == 0, made=True, dur=ts[-1], size=c["style"]["size"], slowed=c["style"]["slowed"],
                   still=_still_run(ts, qs, 0.25, ts[-1] - 0.5 * beat), wrist=wrist_share(qs),
                   j5=max(q[4] for q in qs) - min(q[4] for q in qs), clearance=rep["min_env_clearance_m"])
        break
    return out


def _sweep(fams, hub_names, seeds):
    """_trial over families x hubs x seeds, in worker processes when it can."""
    jobs = [(f, h, s) for h in hub_names for f in fams for s in range(seeds)]
    try:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=max(1, min(16, (os.cpu_count() or 2) - 1)), initializer=_worker_init) as ex:
            return list(ex.map(_trial, jobs, chunksize=2))
    except (OSError, ImportError, RuntimeError) as e:
        print("note: no worker processes (%s); the sweep runs here, 3 seeds" % e)
        _worker_init()
        return [_trial(j) for j in jobs if j[2] < 3]


def self_test():
    import random
    import time
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    t0 = time.time()
    rig = Rig()
    stage_home = [-60.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    R, tcp, d = rig.tool(stage_home)
    r = roll_of(R, d)
    q = rig.solve(tcp, d, r, stage_home)
    check("roll_of + solve give the hub pose back", q is not None and max(abs(a - b) for a, b in zip(q, stage_home)) < 1e-6, q)
    zones, hubs = _show_hubs(rig)
    check("the greet hub from its tool tip and look point", hubs.get("greet") is not None, hubs.get("greet"))

    # the tracking IK: on the branch it starts from, the closed form's answer
    q1 = [x + dx for x, dx in zip(hubs["greet"], (2.0, -1.5, 1.0, 3.0, -2.0, 4.0))]
    R1, tcp1, _ = rig.tool(q1)
    qt = rig.track(tcp1, R1, hubs["greet"])
    qc = rig.solve(tcp1, None, None, hubs["greet"], R=R1)
    check("tracking IK = the nearest closed-form branch", qt is not None and qc is not None and max(abs(a - b) for a, b in zip(qt, qc)) < 1e-5,
          qt and qc and [round(a - b, 7) for a, b in zip(qt, qc)])

    # the eases: 0 -> 1, flat at both ends, monotone, decisive and hesitant
    h = 1e-4
    ease_ok = all(abs(_ease(0.0, b)) < 1e-12 and abs(_ease(1.0, b) - 1.0) < 1e-12 and _ease(h, b) < 1e-8
                  and 1.0 - _ease(1.0 - h, b) < 1e-8 and all(_ease((i + 1) / 50.0, b) >= _ease(i / 50.0, b) for i in range(50))
                  for b in (-0.5, -0.2, 0.0, 0.35, 0.5))
    check("eases start and land at rest, never turn back", ease_ok)

    # keys: every family from each hub ends at the hub at rest, in its beats;
    # reach leans back first (anticipation); tilt goes past its tilt and
    # settles (overshoot); the gaze trails or leads the TCP (overlap)
    beat = 60.0 / 100
    shapes_ok, lengths = True, {}
    for name, hq in sorted(hubs.items()):
        home = home_of(rig, hq)
        for fam in FAMILIES:
            ks = keys_for(fam, home, zones, random.Random(3), beat, 0.7)
            shapes_ok &= all(math.dist(k[1], home[0]) < 1e-9 and abs(k[3] - home[2]) < 1e-9 for k in (ks[0], ks[-2], ks[-1]))
            lengths[fam] = round(sum(k[0] for k in ks) / beat, 2)
    check("keys start and end at the hub, at rest", shapes_ok)
    check("every family is %g-%g beats" % BEATS, all(BEATS[0] - 1e-6 <= b <= BEATS[1] + 1e-6 for b in lengths.values()), lengths)
    home = home_of(rig, hubs["greet"])
    ks = keys_for("reach", home, zones, random.Random(3), beat, 0.7)
    check("reach leans back before it reaches out (anticipation)",
          U._dot(U._sub(ks[1][1], home[0]), U._sub(ks[2][1], home[0])) < 0.0)
    ks = keys_for("tilt", home, zones, random.Random(3), beat, 0.7)
    rolls = [k[3] - home[2] for k in ks]
    check("tilt rolls past and settles back (overshoot)", any(abs(a) > abs(b) + 1.0 and a * b > 0 for a, b in zip(rolls, rolls[1:])),
          [round(x, 1) for x in rolls])
    lags = [k[4]["lag"] for f in FAMILIES for k in keys_for(f, home, zones, random.Random(3), beat, 0.7)]
    check("the gaze trails and leads the TCP (overlap)", min(lags) < 0.0 < max(lags))
    wave = keys_for("wave", home, zones, random.Random(3), beat, 0.7)
    check("a smaller draw stays nearer the hub", max(math.dist(k[1], home[0]) for k in shrink(wave, home, 0.5)) <
          max(math.dist(k[1], home[0]) for k in wave))

    # the sweep: every family from greet and rest, SEEDS seeds, TRIES draws each
    names = [n for n in ("greet", "rest") if n in hubs]
    res = _sweep(FAMILIES, names, SEEDS)
    print("\n%-8s %-6s %8s %8s %8s  %-12s %s" % ("family", "hub", "1st draw", "in %d" % TRIES, "4-12 s", "length", "size / slowed"))
    for name in names:
        for fam in FAMILIES:
            rs = [x for x in res if x["fam"] == fam and x["hub"] == name]
            made = [x for x in rs if x["made"]]
            ds = [x["dur"] for x in made]
            print("%-8s %-6s %5d/%-2d %5d/%-2d %5d/%-2d  %-12s %s" % (
                fam, name, sum(x["first"] for x in rs), len(rs), len(made), len(rs),
                sum(4.0 <= x <= 12.0 for x in ds), len(rs), "%.1f-%.1f s" % (min(ds), max(ds)) if ds else "-",
                "%.2f / %.2f" % (sum(x["size"] for x in made) / len(made), sum(x["slowed"] for x in made) / len(made)) if made else ""))
    made = [x for x in res if x["made"]]
    per = {(x["fam"], x["hub"]) for x in made}
    print()
    check("every family makes a clip from every hub", len(per) == len(FAMILIES) * len(names),
          sorted({(f, n) for f in FAMILIES for n in names} - per))
    check("most seeds make a clip within %d draws" % TRIES, len(made) >= 0.9 * len(res), "%d/%d" % (len(made), len(res)))
    probs = sorted({"%s/%s: %s" % (x["fam"], x["hub"], p) for x in made for p in x["problems"]})
    check("each starts and ends at the hub at rest, within limits and speed, clear of the room", not probs, probs[:6])
    inside_ = sum(4.0 <= x["dur"] <= 12.0 for x in made)
    check("most are 4-12 s long (the show's range)", inside_ >= 0.9 * len(made),
          "%d/%d, %.1f-%.1f s" % (inside_, len(made), min(x["dur"] for x in made), max(x["dur"] for x in made)))
    still = max(made, key=lambda x: x["still"])
    steps = (max(x["first_step"] for x in made), max(x["last_step"] for x in made))
    check("first and last steps under %g deg (starts and ends at rest)" % REST_STEP_DEG, max(steps) <= REST_STEP_DEG,
          "first %.4f, last %.4f deg" % steps)
    check("never frozen for 0.4 s (holds breathe)", still["still"] < 0.4,
          "longest %.2f s (%s from %s)" % (still["still"], still["fam"], still["hub"]))
    greet = [x for x in made if x["hub"] == "greet"]
    shares = {f: round(sum(x["wrist"] for x in greet if x["fam"] == f) / max(1, sum(x["fam"] == f for x in greet)), 2) for f in FAMILIES}
    check("the wrist carries most of look / tilt / nod", all(shares[f] > 0.5 for f in ("look", "tilt", "nod")), shares)
    j5 = {f: round(max([x["j5"] for x in greet if x["fam"] == f] or [0]), 1) for f in FAMILIES}
    check("J5 moves (not a right-angle arm with a still wrist)", sum(v > 8 for v in j5.values()) >= 4, j5)
    print("\n%.0f s" % (time.time() - t0))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
