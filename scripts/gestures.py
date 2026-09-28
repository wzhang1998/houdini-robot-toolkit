"""Gestures: expressive clips made from a hub pose towards interaction zones.

The dance phrases (choreo.py) move the whole arm between kinesphere poses;
seen in the room they read as the arm swinging at right angles, with the
wrist mostly along for the ride. People read a robot arm the way they read
a head and neck: where it LOOKS. Here every key is a tool position AND a
point the tool looks at, so the wrist (J4-J6, J5 above all) carries the
character -- and the body goes with it: it sinks, rises, dives and sweeps
through heights, the more the higher the intensity.

Zones (robot base frame, boxes centre / size / yaw):

    audience   where people stand (outside the robot's reach) -- the points
               the arm looks at, greets, waves to
    greet      where the TCP performs for them (inside the stage)
    idle       the space around the rest hub, for looking around

The space a gesture uses (Space): ACROSS, the footprint of the zone its
hub's tool tip stands in (greet first; a FALLBACK_M square around the tip
when it stands in none); UP AND DOWN, the whole operating band TCP_Z (the
show's range.tcp_z) -- the zone is design space, not a wall. Hard limits on
top, from the room (when make() has one): inside every work zone (the
stage, the controller's work area: z 0.1-1.6 m) and out of the slow zones
the hub is not in (the operator's), by HARD_PAD. Keys outside are pulled
onto its boundary. A hub that faces away from the audience looks at it
turned into its view (the audience's centre within GAZE_CONE_DEG * 0.6 of
the hub's aim, no look beyond GAZE_CONE_DEG).

A gesture starts and ends at its hub (at rest), so it chains in the show
graph like any clip: a quarter beat still at the start (the mirror of the
half beat at the end), then the first move eases out of rest (never
front-loaded), so the first and last 24 fps steps are zero and speed and
acceleration rise from zero at both joins. Families:

    look    glance from point to point in the audience (and above their
            heads, and down at their feet), holds between, leaning and
            bobbing towards each, the gaze leading the lean
    wave    the "hand" goes up, the TCP swings sideways while the tool
            keeps aiming at one person and rolls with the swing
    nod     the gaze drops and comes back up (two or three, smaller each
            time), the TCP dipping
    reach   leans back, then moves out and up (or down) towards the
            audience (an offer), holds, offers again a little further
    tilt    the tool keeps aiming at someone and rolls to one side and the
            other, like a head tilting (J6, J5), holds -- the calm one
    trace   the TCP draws a loose, flowing path through its space while the
            gaze drifts across the audience, trailing it
    peek    ducks low and creeps towards a person, still, then pops up to
            peek, cocks the wrist, pauses, darts back a little
    shy     the gaze drops away and the TCP shrinks back and down towards
            the robot, then it slowly looks back
    stretch a big slow reach up and out (out and across when there is no
            room above), the wrist rolling, like a yawn, then settles
    bounce  bobs on the beat, the gaze fixed on someone, the groove
            sinking or lifting halfway, like a head bob to music
    search  scans the audience side to side, travelling and bending low
            and high, quick look and hold; ends leaning towards one person
    rise    gathers low (the gaze on the floor), still, then rises high,
            looking up and out over the audience, holds, settles back
    dive    a small lift, then swoops down towards the floor in front and
            back up past the hub, like a bird dipping, the gaze leading
    sweep   a big arc across the stage and through heights (a rainbow or a
            swing), the gaze leading the way, and back
    pop     an accent, 3-4 s: a flick the other way, a snap to a pose (up,
            down, out, across) that stops dead, a freeze, back

The LED strip on the flange (assets/tools/led_strip.urdf: 1 m, centred as a
T, its length along the tool frame's y, the LEDs facing out along z) is the
character's prop. Its families aim and roll the tool so the strip lies where
the gesture needs it (roll_for / lay: the strip is symmetric, so a roll
within +-90 degrees lays it any way across the aim), keep its lower end
STRIP_FLOOR_M over the floor, and draw a few candidate designs (headings,
heights, pane tilts, aims), performing the first whose poses and the ways
between -- the way home included -- the probe finds the tool can take (make's
Probe: the joint limits, the wrist, the room and the arm, the strip
included):

    twirl   the wrist spins the strip like a propeller / baton, face on to
            the audience (else level ahead, its own aim, aside; out, further
            out or lower: the roomiest): winds up (further back where the
            room ahead is short), spins 160-300 degrees (J6: its limits, the
            room and the arm scanned first, roll_room), speeding up and
            braking past its mark, settles; spins back past the start (long)
            or unwinds; the tool tip drifting a little. Under TWIRL_MIN_DEG
            of room anywhere, the draw is refused
    wipe    a squeegee on a pane in front (facing the audience; turned, or
            leaning back like a windscreen, where the arm cannot face it
            square): the face flat to the glass, the strip the blade, 3-5
            overlapping strokes (as many as the length leaves room for, quicker
            and shorter when short of it) -- side to side with the blade
            upright, rows stepping down, or top to bottom with it level --
            pressed on for each stroke and lifted off between, the blade
            trailing; the pane moved in where the arm's reach cuts it; a shake
    scoop   the strip as a shovel (sideways first, read in profile): takes
            it up, tips it down and digs in steep (its lower end 0.35-0.65 m
            under the hub's tool tip, not under STRIP_FLOOR_M), scoops low
            and ahead levelling the blade, lifts with a toss, settles
    broom   the strip near level and low, the face down and ahead (the
            bristles), brushing side to side, the head angled into each
            stroke and its face trailing, creeping ahead; a flick
    salute  raised upright like a sabre before the face, held; a flourish
            (spun end over end when J6 has the room, or a cut across and
            back); lowered across and forward, a bow

The other families keep the strip clear by its roll: a key whose pose would
put the strip (or the arm) into the room or the arm is turned by the least
roll that clears it (strip_safe). STRIP_REFUSED: what the strip makes
impossible from a hub.

Animation inside the keys (all scaled by intensity k):

    anticipation   a counter-move before a big move (leans back before
                   reaching out, dips before stretching up)
    overshoot      fast moves go past their target and settle back
    overlap        the gaze / roll has its own clock: it trails the TCP
                   (follow-through: the wrist catches up) or leads it (the
                   eyes turn first, the lean follows)
    ease           min-jerk on a warped clock per move: decisive (quick
                   start, long settle) or hesitant (slow start, late landing)
    flow           a key marked via is passed through without stopping: one
                   eased clock over the run, a cubic through its keys (arcs,
                   swoops, loops)
    breathing      a slow drift of the TCP (a few mm) and the gaze over the
                   whole clip, a breath a bar, faded in and out: holds are
                   never frozen

Rhythm: the tempo (bpm) sets the beat, the intensity how many beats a move
takes (TEMPO: a calm move up to twice its beats, a lively one 0.6 of them,
no quicker than V_DRAW / A_DRAW as drawn) and how big it is (sizes from
calm to lively, +-15 %). A clip's length is drawn first: a pop is 3-4.2 s, a
short accent form (likelier the livelier; quick even when calm, parts left
out) 3.2-4.8 s, the rest a long form of 5-12 s (the calmer, the longer);
repeats (glances, swings, bobs, nods) fill it, counted again from an
estimate of the joints' time when they would not. Within a clip:
accelerando / ritardando over repeats, a sudden stop into a held pose,
stillness then a burst.

    make(rig, hub_q, family, zones, rng, bpm, intensity, clip_id=None, env=None,
         safety=PLAN_SAFETY) -> motion clip dict (or None); labels["family"],
         labels["params"] (form, length, tempo, accents, space, size, slowed;
         a strip family's own: spin_deg, pane_normal, heading, flourish, ...)

Timing: keys at beat multiples; sampled at 24 fps through IK that tracks
the previous frame (Newton from it, so the arm stays on its branch; the
closed form when that fails). A key whose pose -- or the way to it --
would take the wrist near its singularity has its gaze turned back
towards the hub's aim (wrist_safe: a hub with J5 at 25-35 degrees cannot
look as far about). Each move is then given the time its joints need at
the plan safety (fairino_player.need_profile, per key, rounded up to an
eighth of a beat): a big move takes more beats, up to MAX_STRETCH times its
own and a clip of MAX_CLIP_S (the holds give time back first). Only a move
that would crawl, a clip that would run over MAX_CLIP_S, a key the arm
cannot hold, a branch flip or the room drop the gesture to a smaller size of
itself (SIZES) -- the last resort -- before the draw is given up.

hub_pose (show.resolve_hubs) solves a hub from its tool tip and look point:
the IK pose nearest its seed, away from the wrist singularity and, as far
as a roll allows, the joint limits; when that pose is not clear of the
floor and itself (a low hub from a high hub's seed is elbow-down), every
branch is weighed instead.
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
HEAD_FAMILIES = ("look", "wave", "nod", "reach", "tilt", "trace", "peek", "shy", "stretch", "bounce", "search",
                 "rise", "dive", "sweep", "pop")
STRIP_FAMILIES = ("twirl", "wipe", "scoop", "broom", "salute")   # the LED strip as the prop
FAMILIES = HEAD_FAMILIES + STRIP_FAMILIES
LIMIT_MARGIN = 3.0
MAX_STEP_DEG = 30.0              # per 24 fps frame: more is a branch flip (~180), not a quick gesture; timing is fitted after
PLAN_SAFETY = 0.85
GAZE_CONE_DEG = 50.0             # the most the gaze turns from the hub's aim
SIZES = (1.0, 0.8, 0.6, 0.45)    # the last resort: a draw that does not work is tried smaller
MAX_STRETCH = 4.0                # a move needing more than this times its beats would crawl: smaller instead
MIN_CLIP_S, MAX_CLIP_S = 3.0, 12.0   # a clip's length (the show's library.duration_s)
TEMPO = (2.0, 0.6)               # a move's beats times this, calm (k 0) .. lively (k 1)
TCP_Z = (0.35, 1.9)              # the show's operating band for the TCP (shows/*.json range.tcp_z)
Z_PAD = 0.05                     # m inside the operating band
ZONE_PAD = 0.02                  # m inside a zone's footprint
HARD_PAD = 0.03                  # m inside the room's work zones, outside its slow zones
FALLBACK_M = 0.8                 # the footprint around a hub that stands in no zone
SHOULDER = (0.0, 0.0, 0.215)     # the FR20's J2 axis (ur_ik model p2)
REACH_M = 1.6                    # the TCP this near the shoulder (or as near as its hub): beyond, a tool that
                                 # must look somewhere runs out of arm (the wrist centre reaches 1.72 m)
RATE_HZ = 125.0                  # the player's rate (fairino_player)
V_DRAW, W_DRAW = 1.5, 240.0      # a move is drawn no quicker than these peaks (m/s of the TCP, deg/s of gaze and roll)
A_DRAW, AW_DRAW = 4.0, 600.0     # nor more violent than these (m/s^2, deg/s^2): quicker would step like a branch
                                 # flip, or need its time many times over, before the joints' own time is fitted
A_EST, AW_EST = 1.4, 220.0       # what the joints usually allow a move (m/s^2 of the TCP, deg/s^2 of the gaze):
                                 # the estimate a clip's repeats are counted by
UP = (0.0, 0.0, 1.0)
DOWN = (0.0, 0.0, -1.0)
STRIP_DEFAULT = (0.06, 0.5, (0.0, 1.0, 0.0), 0.016)   # the LED strip (assets/tools/led_strip.urdf) when none is mounted
STRIP_FLOOR_M = 0.2              # a strip family keeps the strip's lower end this high (the checks: 5 cm margin)
STRIP_TOP_M = 1.95               # and its upper end this low (the ceiling at 2.16 m, less its margin)
STRIP_ROLLS = (15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0, 75.0, -75.0, 90.0)   # strip_safe's turns, least first
TWIRL_MIN_DEG = 120.0            # a twirl spins the strip at least this far (J6)
# (family, hub) the strip makes impossible, checked as refused: the high hub's
# wrist (J5 26 deg) cannot turn the face down to the floor without passing
# its singularity or folding onto the forearm (every heading, face angle and
# approach tried)
STRIP_REFUSED = (("broom", "high"),)
_STRIP = []


def strip_geometry():
    """(offset, half, axis, radius) of the strip on the flange, in the tool
    (TCP) frame: its centre `offset` m out along the tool axis, `half` its
    half length, `axis` the way its length runs (the tool frame's y for the
    LED strip), `radius` its capsule's -- from the tool capsule named
    tool_strip (collision.load_model reads the tool's URDF), STRIP_DEFAULT
    when the profile mounts none. Cached."""
    if not _STRIP:
        import collision as CL
        m = CL.load_model("fr20")
        cap = next((c for c in m["caps"] if c["name"] == "tool_strip"), None)
        if cap is None:
            _STRIP.append(STRIP_DEFAULT)
        else:
            a, b = cap["a"], cap["b"]
            mid = [(x + y) / 2.0 for x, y in zip(a, b)]
            _STRIP.append((mid[2] - m["tcp_local"][2], math.dist(a, b) / 2.0, U._normalize(U._sub(b, a)), cap["r"]))
    return _STRIP[0]


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

    def solutions(self, tcp, direction, roll, near, R=None):
        """Every IK pose for the tool at tcp (within the limits and their
        margin), each unwrapped towards near."""
        R = R or C.tool_frame(direction, roll)
        p6 = U._sub(tcp, U._mat_vec(R, (0.0, 0.0, self.fo)))
        sols = ur_ik.within_limits(self.model, ur_ik.solve(self.model, R, p6, q6_when_singular=near[5]))
        return [s for s in sols if self.within(s["q"])]

    def solve(self, tcp, direction, roll, near, R=None):
        best = ur_ik.nearest(self.solutions(tcp, direction, roll, near, R), near)
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


def _mix(a, b, u):
    return a + (b - a) * u


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


def _flat(v):
    """v's horizontal direction (None when v is vertical)."""
    h = (v[0], v[1], 0.0)
    return U._normalize(h) if U._norm(h) > 1e-6 else None


def _pick(rng, weighted):
    r = rng.random() * sum(w for _, w in weighted)
    for x, w in weighted:
        r -= w
        if r <= 0:
            return x
    return weighted[-1][0]


def box_point(box, rng, fx=(-0.5, 0.5), fy=(-0.5, 0.5), fz=(-0.5, 0.5)):
    """A random point in a (yawed) box, each axis within the given fractions."""
    c, s = box["center"], box["size"]
    yaw = math.radians(box.get("yaw_deg", 0.0))
    lx, ly, lz = s[0] * rng.uniform(*fx), s[1] * rng.uniform(*fy), s[2] * rng.uniform(*fz)
    return (c[0] + math.cos(yaw) * lx - math.sin(yaw) * ly, c[1] + math.sin(yaw) * lx + math.cos(yaw) * ly, c[2] + lz)


def _local(box, p):
    """p in a (yawed) box's frame, from its centre."""
    c = box["center"]
    yaw = math.radians(box.get("yaw_deg", 0.0))
    dx, dy = p[0] - c[0], p[1] - c[1]
    return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy, p[2] - c[2])


def inside(box, p, pad=0.0):
    lx, ly, dz = _local(box, p)
    s = box["size"]
    return abs(lx) <= s[0] / 2 - pad and abs(ly) <= s[1] / 2 - pad and abs(dz) <= s[2] / 2 - pad


def into(box, p, pad=0.0):
    """p moved onto the nearest point of the (yawed) box shrunk by pad."""
    c, s = box["center"], box["size"]
    yaw = math.radians(box.get("yaw_deg", 0.0))
    cy, sy = math.cos(yaw), math.sin(yaw)
    lx, ly, dz = _local(box, p)
    h = [max(0.0, x / 2.0 - pad) for x in s]
    lx, ly, dz = max(-h[0], min(h[0], lx)), max(-h[1], min(h[1], ly)), max(-h[2], min(h[2], dz))
    return (c[0] + cy * lx - sy * ly, c[1] + sy * lx + cy * ly, c[2] + dz)


# --------------------------------------------------------------------------
# the space a gesture (or a path) may use
# --------------------------------------------------------------------------

def footprint_zone(zones, p, name=None):
    """(name, box) of the zone a hub's tool tip performs in: the named one
    (when its footprint holds p), else the zone that holds p, else the one
    whose footprint (across, any height) holds p -- greet, idle, the others
    in turn, never the audience; (None, None) when none does."""
    names = [name] if name else ["greet", "idle"] + sorted(k for k in zones if k not in ("greet", "idle", "audience"))
    boxes = [(n, zones[n]) for n in names if zones.get(n) and "center" in zones[n]]
    for n, z in boxes:
        if inside(z, p):
            return n, z
    for n, z in boxes:
        lx, ly, _ = _local(z, p)
        if abs(lx) <= z["size"][0] / 2 and abs(ly) <= z["size"][1] / 2:
            return n, z
    return None, None


class Space:
    """Where the TCP may go from a hub at p0: the zone's footprint (ZONE_PAD
    inside) across, the operating band (Z_PAD inside) up and down, inside
    the room's work zones and out of the slow zones p0 is not in (HARD_PAD),
    within REACH_M of the shoulder; the hub's own point always inside.

        box              the soft space as a yawed box (footprint x band)
        bottom, top      its height band (m)
        clamp(p)         the nearest point of the space
        contains(p, pad) p inside it"""

    def __init__(self, zones, p0, env=None, name=None, band=TCP_Z):
        self.p0 = tuple(p0)
        self.name, zone = footprint_zone(zones or {}, p0, name)
        if zone is None:
            zone = {"center": list(p0), "size": [FALLBACK_M, FALLBACK_M, 1.0], "yaw_deg": 0.0}
        lx, ly, _ = _local(zone, p0)
        hx, hy = zone["size"][0] / 2 - ZONE_PAD, zone["size"][1] / 2 - ZONE_PAD
        xlo, xhi, ylo, yhi = min(-hx, lx), max(hx, lx), min(-hy, ly), max(hy, ly)
        zlo, zhi = band[0] + Z_PAD, band[1] - Z_PAD
        objs = (env or {}).get("objects", [])
        self.work = [o for o in objs if o.get("role") == "work" and o.get("type") == "box"]
        for w in self.work:
            zlo = max(zlo, w["center"][2] - w["size"][2] / 2 + HARD_PAD)
            zhi = min(zhi, w["center"][2] + w["size"][2] / 2 - HARD_PAD)
        zlo, zhi = min(zlo, p0[2]), max(zhi, p0[2])
        yaw = math.radians(zone.get("yaw_deg", 0.0))
        cx, cy = (xlo + xhi) / 2, (ylo + yhi) / 2
        c = zone["center"]
        self.box = {"center": [c[0] + math.cos(yaw) * cx - math.sin(yaw) * cy,
                               c[1] + math.sin(yaw) * cx + math.cos(yaw) * cy, (zlo + zhi) / 2],
                    "size": [xhi - xlo, yhi - ylo, zhi - zlo], "yaw_deg": zone.get("yaw_deg", 0.0)}
        self.bottom, self.top = zlo, zhi
        self.reach = max(REACH_M, math.dist(p0, SHOULDER) + 0.02)
        # the slow zones (the operator's) the hub is not in: kept out of, across
        self.slow = []
        for o in objs:
            if o.get("role") == "slow" and o.get("type") in ("cylinder", "sphere"):
                r = o["radius"] + HARD_PAD
                if math.hypot(p0[0] - o["center"][0], p0[1] - o["center"][1]) > r:
                    self.slow.append(((o["center"][0], o["center"][1]), r))

    def clamp(self, p):
        p = tuple(p)
        for _ in range(2):
            p = into(self.box, p)
            for w in self.work:
                if inside(w, self.p0, HARD_PAD):
                    p = into(w, p, HARD_PAD)
            for (cx, cy), r in self.slow:
                d = math.hypot(p[0] - cx, p[1] - cy)
                if d < r:
                    s = r / d if d > 1e-9 else 1.0
                    p = (cx + (p[0] - cx) * s, cy + (p[1] - cy) * s, p[2])
            d = math.dist(p, SHOULDER)
            if d > self.reach:
                p = U._add(SHOULDER, U._scale(U._sub(p, SHOULDER), self.reach / d))
        return p

    def contains(self, p, pad=0.0):
        return (inside(self.box, p, pad) and all(inside(w, p, HARD_PAD + pad) for w in self.work if inside(w, self.p0, HARD_PAD))
                and all(math.hypot(p[0] - cx, p[1] - cy) >= r + pad for (cx, cy), r in self.slow)
                and math.dist(p, SHOULDER) <= self.reach - pad)


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


def clip_length(rng, k, family):
    """(form, (lo, hi) s) a clip is drawn to last: a pop 3-4.2 s; a wipe,
    broom or salute always long (6.5-11 s, round a length drawn); else a
    short accent form (likelier the livelier) 3.2-5 s, or a long form round
    a length drawn from 7-11 s (calm) .. 5-8.5 s (lively), within 5-12 s."""
    if family == "pop":
        return "pop", (MIN_CLIP_S, 4.2)
    if family in ("wipe", "broom", "salute"):          # a routine with the prop: its strokes, its ceremony
        mid = rng.uniform(_mix(8.0, 6.5, k), _mix(11.0, 9.5, k))
        return "long", (max(5.0, mid - 1.5), min(MAX_CLIP_S, mid + 1.0))
    if rng.random() < 0.35 + 0.4 * k:
        return "short", (3.2, 4.8)
    mid = rng.uniform(_mix(7.0, 5.0, k), _mix(11.0, 8.5, k))
    return "long", (max(5.0, mid - 1.5), min(MAX_CLIP_S, mid + 1.0))


class _Draw:
    """The keys of one gesture as it is drawn: the current tool tip, look
    point and roll, and moves from them. Durations are in beats here; a
    move's are multiplied by the intensity's tempo (holds and the still
    start are not)."""

    def __init__(self, home, zones, rng, beat, k, space, budget, form="long"):
        self.p0, self.look0, self.r0 = home
        self.p, self.look, self.roll = home
        self.rng, self.beat, self.k = rng, beat, k
        self.space = space
        self.zone = space.box
        self.view = _View(zones, self.p0, self.look0)
        self.budget = budget                  # beats the clip is drawn to last
        self.keys = []
        self.repeats = False
        self.params = {"tempo": "steady", "accents": []}
        self.tempo = _mix(TEMPO[0], TEMPO[1], k)
        self.short = form != "long"
        if self.short:                        # an accent is quick even when calm
            self.tempo = min(self.tempo, _mix(1.0, TEMPO[1], k))
        self.over = 0.08 + 0.17 * k           # overshoot, fraction of the move
        self.anti = 0.08 + 0.17 * k           # anticipation, fraction of the move
        self.lag = beat * (0.1 + 0.25 * k)    # overlap of the gaze / roll, s
        a = zones.get("audience")
        f = _flat(U._sub(a["center"], self.p0)) if a else None
        f = f or _flat(self.view.d0) or _flat((-self.p0[0], -self.p0[1], 0.0)) or (1.0, 0.0, 0.0)
        self.fwd = f                                          # across the floor, towards the audience
        self.side = U._normalize(U._cross(UP, f))             # across, to its left
        self.back = _flat((-self.p0[0], -self.p0[1], 0.0)) or U._scale(f, -1.0)   # towards the robot
        self.room_up = max(0.0, space.top - self.p0[2])
        self.room_down = max(0.0, self.p0[2] - space.bottom)
        # the tool frame as the sampler will carry it from key to key
        # (transport), for the strip families to lay the strip where they
        # want it; probe: can the tool stand at a pose (Probe, from make)
        self.R = C.tool_frame(U._sub(self.look0, self.p0), self.r0)
        self.probe = None
        self.free = False                     # the strip families aim where the prop needs (no gaze cone)
        self.why = None                       # a list: why a design could not be laid (lay)
        self.strip = strip_geometry()
        # the still start, as the end's rest: a min-jerk move straight from
        # the first frame steps ~10 u^3 of the move at once (0.03-0.1 deg
        # for these); from rest here, the first move eases in from a frame at rest
        self.key(max(0.25, 2.0 / (FPS * beat)), ease=0.0, raw=True)
        self.keys[-1][4]["lead"] = True

    # -- where
    def clamp(self, p):
        return self.space.clamp(p)

    def at(self, fwd=0.0, side=0.0, up=0.0, base=None):
        """A point fwd towards the audience, side across, up from base (the hub)."""
        b = base or self.p0
        return (b[0] + self.fwd[0] * fwd + self.side[0] * side, b[1] + self.fwd[1] * fwd + self.side[1] * side, b[2] + up)

    def amp(self, lo, hi):
        """A size from lo (calm) to hi (lively), varied +-15 %."""
        return _mix(lo, hi, self.k) * self.rng.uniform(0.85, 1.15)

    def vert(self, want, prefer=0):
        """A signed height change of `want` from the hub: up or down (prefer
        +1 / -1 when both fit), where the space has room; less when neither
        side has it."""
        up, dn = max(0.0, self.room_up - 0.01), max(0.0, self.room_down - 0.01)
        fits = [s for s, r in ((1, up), (-1, dn)) if r >= want]
        if prefer and prefer in fits:
            return prefer * want
        if fits:
            return self.rng.choice(fits) * want
        return min(want, up) if up >= dn else -min(want, dn)

    def eyes(self):
        return self.view.eyes(self.rng)

    def azimuth(self, p):
        """p's bearing about the robot's base axis (degrees)."""
        return math.degrees(math.atan2(p[1], p[0]))

    def swung(self, p, lead_deg=0.0, pitch_deg=0.0):
        """A look point for the tool at p that turns with the body: the
        hub's aim turned about the vertical by p's change of bearing from the
        hub (as J1 turns it) and lead_deg more, pitched up by pitch_deg. The
        wrist stays quiet, so the arm can swing at its quickest."""
        turn = self.azimuth(p) - self.azimuth(self.p0) + lead_deg
        d = U._mat_vec(U.axis_angle_matrix(UP, math.radians(turn)), self.view.d0)
        if pitch_deg:
            d = _turn(d, UP, pitch_deg)
        return U._add(p, U._scale(d, 1.5))

    def gaze(self, p=None, look=None):
        p, look = p or self.p, look or self.look
        return U._normalize(U._sub(look, p))

    # -- when
    def left(self):
        """Beats still free in the budget (the way home kept aside)."""
        return self.budget - sum(k[0] for k in self.keys) - 2.0 * self.tempo - 0.5

    def reps(self, per, lo, hi):
        """How many repeats of `per` beats (a move's, before the tempo) fit, lo..hi."""
        self.repeats = True
        return max(lo, min(hi, int(self.left() / max(1e-6, per * self.tempo))))

    def curve(self, n):
        """Duration factors for n repeated moves (mean 1): steady, accelerando
        (each quicker, the last about half the first) or ritardando (each
        slower); the livelier, the likelier an accelerando."""
        kind = _pick(self.rng, (("steady", 1.0), ("accel", 0.6 + 0.8 * self.k), ("rit", 0.8)))
        if n < 2 or kind == "steady":
            return [1.0] * n
        self.params["tempo"] = kind
        r = (0.5 if kind == "accel" else 1.8) ** (1.0 / (n - 1))
        ms = [r ** i for i in range(n)]
        mean = sum(ms) / n
        return [m / mean for m in ms]

    def ease(self):
        """A varied slow-in / slow-out for an ordinary move."""
        return self.rng.uniform(-0.25, 0.35) * (0.4 + 0.6 * self.k)

    def target(self, p=None, look=None, roll=None):
        """(tcp, look, roll) of a key: the TCP kept in its space, the look in
        view; what is not given stays (the gaze keeps its direction)."""
        p = self.clamp(p) if p is not None else self.p
        if look is None:
            look = U._add(p, U._sub(self.look, self.p))
        elif not self.free:
            look = self.view.in_view(look)
        return p, look, self.roll if roll is None else roll

    def key(self, beats, p=None, look=None, roll=None, ease=None, lag=0.0, hold=False, via=False, least=0.5, raw=False):
        """A key `beats` after the last (times the tempo unless a hold or raw).
        via: passed through without stopping (the next key must move)."""
        p, look, roll = self.target(p, look, roll)
        if not (hold or raw):
            beats *= self.tempo
            # no quicker than V_DRAW / W_DRAW, A_DRAW / AW_DRAW at the min-jerk
            # peaks (1.875 d / T, 5.77 d / T^2)
            ang, dist = max(_angle(self.gaze(), self.gaze(p, look)), abs(roll - self.roll)), math.dist(p, self.p)
            least = max(1.875 * dist / V_DRAW, 1.875 * ang / W_DRAW, math.sqrt(5.77 * dist / A_DRAW),
                        math.sqrt(5.77 * ang / AW_DRAW), 0.15)
            beats = max(beats, least / self.beat)
        if len(self.keys) == 1:                             # the first move: out of rest gently, never front-loaded
            ease = min(0.0, self.ease() if ease is None else ease)
        if hold and self.keys and self.keys[-1][4].get("via"):
            del self.keys[-1][4]["via"]                    # a run cannot flow into a standstill
        opt = {"ease": self.ease() if ease is None else ease, "lag": lag, "hold": hold}
        if hold:
            opt["least"] = least
        if via:
            opt["via"] = True
        self.keys.append((beats, p, look, roll, opt))
        self.R = transport(self.R, U._sub(look, p), roll - self.roll)
        self.p, self.look, self.roll = p, look, roll

    def hold(self, beats, least=None):
        """A hold of `beats` (not shortened under least: half a beat, a third in an accent)."""
        self.key(beats, ease=0.0, hold=True, least=least if least is not None else (0.35 if self.short else 0.5))

    def _past(self, p, look, roll, frac, away=False):
        """A state past the target (overshoot) or before the start, away
        from the target (anticipation): frac of the move, at most 3-9 cm
        (with intensity), 10 degrees of gaze, 10 degrees of roll."""
        sgn = -1.0 if away else 1.0
        base_p, base_d, base_r = (self.p, self.gaze(), self.roll) if away else (p, self.gaze(p, look), roll)
        dp = _cap(U._scale(U._sub(p, self.p), sgn * frac), 0.03 + 0.06 * self.k)
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

    def arrive(self, beats, p=None, look=None, roll=None, lag=0.0, settle=0.5, ease=0.35, via=False):
        """A quick move that goes past its target and settles back."""
        p, look, roll = self.target(p, look, roll)
        q, lq, rq = self._past(p, look, roll, self.over)
        self.key(beats, q, lq, rq, ease=ease, lag=lag)
        self.key(settle, p, look, roll, ease=-0.15, lag=lag * 0.5)

    def stop(self, beats, p=None, look=None, roll=None, lag=0.0):
        """A sudden stop into a held pose: gathering speed to the very end
        (a late landing), a small overshoot, settled in a quarter beat."""
        p, look, roll = self.target(p, look, roll)
        q, lq, rq = self._past(p, look, roll, 0.5 * self.over)
        self.key(beats, q, lq, rq, ease=-0.35, lag=lag)
        self.key(0.25, p, look, roll, ease=0.0, lag=0.0)
        self.params["accents"].append("stop")

    def burst(self, still, beats, p=None, look=None, roll=None, lag=0.0, via=False):
        """Stillness, then a burst: a hold of `still` beats (it breathes),
        then a move that starts at once and settles long."""
        self.hold(still, least=max(0.5, still * 0.6))
        self.key(beats, p, look, roll, ease=0.45, lag=lag, via=via)
        self.params["accents"].append("burst")

    def home(self, lag):
        """Back to the hub (the wrist settling last), then at rest."""
        self.key(1.5, self.p0, self.look0, self.r0, ease=0.1, lag=lag)
        self.keys[-1][4]["home"] = True
        self.key(0.5, self.p0, self.look0, self.r0, ease=0.0, lag=0.0, raw=True)
        self.keys[-1][4]["home"] = True

    # -- the strip
    def frame_at(self, p, look, roll):
        """The tool frame of a key (p, look, roll) after the last: its frame carried to the new aim, rolled."""
        return transport(self.R, U._sub(look, p), roll - self.roll)

    def _turn_to(self, R, s):
        """The roll (degrees, within +-90: the strip is symmetric) that
        turns frame R's strip onto s about R's aim (0 when s lies along it)."""
        d = (R[0][2], R[1][2], R[2][2])
        y = U._mat_vec(R, self.strip[2])
        sp = U._sub(s, U._scale(d, U._dot(s, d)))
        if U._norm(sp) < 1e-6:
            return 0.0
        phi = math.degrees(math.atan2(U._dot(d, U._cross(y, sp)), U._dot(y, sp)))
        return (phi + 90.0) % 180.0 - 90.0

    def roll_for(self, p, look, s):
        """The key roll that lays the strip along s (as near as it can lie
        across the aim), turned from the current roll by at most 90 degrees."""
        return self.roll + self._turn_to(self.frame_at(p, look, self.roll), s)

    def lay(self, targets, check=True):
        """[(p, look, roll)] of keys for targets [(p, look, way)] in turn from
        the last key: the tool tip in its space, raised where the strip's
        lower end would come under STRIP_FLOOR_M; the roll laying the strip
        along `way` (a vector; turned at most 90 degrees), or the roll `way`
        (a number), or as it is (None). With check (and a probe), None when
        a pose or the way to it -- in steps of 15 degrees and 8 cm -- is one
        the tool cannot take: a joint limit, the wrist singularity, a branch
        flip, the room, the arm itself, the strip included."""
        R, roll, p_, l_ = self.R, self.roll, self.p, self.look
        probe = self.probe if check else None
        q = probe.pose(p_, R) if probe else None
        if probe and q is None:
            return self._why("the start")
        out = []
        for tgt in targets:
            p, look, way = tgt[:3]
            p = self.clamp(p)
            for _ in range(2 if len(tgt) < 4 or tgt[3] else 1):   # raise the tool tip over the strip's lower end
                Rt = transport(R, U._sub(look, p), 0.0)
                phi = (way - roll if isinstance(way, (int, float)) else
                       self._turn_to(Rt, way) if way is not None else 0.0)
                Rt = transport(R, U._sub(look, p), phi)
                c = U._add(p, U._scale((Rt[0][2], Rt[1][2], Rt[2][2]), self.strip[0]))
                low = c[2] - self.strip[1] * abs(U._mat_vec(Rt, self.strip[2])[2])
                if low >= STRIP_FLOOR_M - 1e-6 or (len(tgt) > 3 and not tgt[3]):
                    break
                shift = STRIP_FLOOR_M - low
                p, look = self.clamp(U._add(p, (0.0, 0.0, shift))), U._add(look, (0.0, 0.0, shift))
            if probe:
                ang = max(_angle(U._sub(l_, p_), U._sub(look, p)), abs(phi))
                steps = max(1, int(math.ceil(max(ang / 15.0, math.dist(p, p_) / 0.08))))
                for j in range(1, steps + 1):
                    u = j / float(steps)
                    pu, lu = _lerp(p_, p, u), _lerp(l_, look, u)
                    qn = probe.pose(pu, transport(R, U._sub(lu, pu), phi * u), near=q)
                    if qn is None or max(abs(a - b) for a, b in zip(qn, q)) > 60.0:
                        return self._why("target %d at %.2f: %s" % (len(out), u, "a flip" if qn else probe.last))
                    q = qn
            R, roll, p_, l_ = Rt, roll + phi, p, look
            out.append((p, look, roll))
        return out

    def _why(self, reason):
        """None, the reason noted in self.why (a list) when there is one."""
        if self.why is not None:
            self.why.append(reason)
        return None

    def perform(self, designs, fallback=-1):
        """Keys from the first of the candidate designs the tool can take
        (lay), the way home included: a design is a list of moves (kind,
        beats, p, look, way, opts) -- kind key, arrive or stop (a hold:
        ("hold", beats)). designs[fallback], unchecked, when none is clear
        (make's checks then refuse it or not). The index of the design used."""
        home = (self.p0, self.look0, float(self.r0), False)
        chosen, laid = None, None
        for i, moves in enumerate(designs):
            laid = self.lay([m[2:5] for m in moves if m[0] != "hold"] + [home])
            if laid is not None:
                chosen = i
                break
        if laid is None:
            chosen = fallback % len(designs)
            laid = self.lay([m[2:5] for m in designs[chosen] if m[0] != "hold"], check=False)
        it = iter(laid)
        for m in designs[chosen]:
            if m[0] == "hold":
                self.hold(m[1])
                continue
            p, look, roll = next(it)
            opts = dict(m[5]) if len(m) > 5 else {}
            getattr(self, m[0])(m[1], p, look, roll, **opts)
        return chosen

    def strip_z(self, z, down, up, reach=0.0):
        """z moved to where a strip reaching `down` below and `up` above
        it keeps STRIP_FLOOR_M over the floor and under STRIP_TOP_M, and
        the TCP moving `reach` up and down from it stays in its space
        (between, when it cannot)."""
        lo = max(self.space.bottom + reach, STRIP_FLOOR_M + down)
        hi = min(self.space.top - reach, STRIP_TOP_M - up)
        return max(lo, min(hi, z)) if lo <= hi else 0.5 * (lo + hi)

    def roll_room(self, p, look, roll, most=330.0, step=10.0):
        """(down, up): how far (degrees) the roll may turn each way from
        `roll` with the tool at p looking at look -- the joint limits (J6
        above all), the wrist, the room and the arm itself, the strip
        included -- scanned in steps from the pose nearest the hub; None
        without a probe."""
        if self.probe is None:
            return None
        p = self.clamp(p)
        R = self.frame_at(p, look, roll)
        d = (R[0][2], R[1][2], R[2][2])
        q0 = self.probe.pose(p, R)
        if q0 is None:
            return (0.0, 0.0)
        out = []
        for sgn in (-1.0, 1.0):
            q, a = q0, 0.0
            while a < most:
                qn = self.probe.pose(p, transport(R, d, sgn * (a + step)), near=q)
                if qn is None or max(abs(x - y) for x, y in zip(qn, q)) > 2.0 * step + 5.0:
                    break
                q, a = qn, a + step
            out.append(a)
        return tuple(out)


def estimate(keys, beat):
    """About how long (s) the joints will make keys (durations in beats):
    each move at least what A_EST / AW_EST allow its distance and turn of
    gaze and roll (min-jerk: T = sqrt(5.77 d / a)), holds as drawn."""
    total, prev = 0.0, None
    for b, p, look, roll, opt in keys:
        t = b * beat
        if prev is not None and not opt.get("hold"):
            d0, d1 = U._normalize(U._sub(prev[1], prev[0])), U._normalize(U._sub(look, p))
            ang = max(_angle(d0, d1), abs(roll - prev[2]))
            t = max(t, math.sqrt(5.77 * math.dist(p, prev[0]) / A_EST), math.sqrt(5.77 * ang / AW_EST))
        total += t
        prev = (p, look, roll)
    return total


def _fit_beats(keys, lo, hi):
    """Holds lengthened (or shortened, not under their least) so the keys
    add up to lo..hi beats; a hold added before the way home when there are
    none to lengthen."""
    total = sum(k[0] for k in keys)
    holds = [i for i, k in enumerate(keys) if k[4].get("hold")]
    if total < lo:
        if not holds:
            i = next(i for i, k in enumerate(keys) if k[4].get("home"))
            if keys[i - 1][4].get("via"):
                del keys[i - 1][4]["via"]
            keys.insert(i, (0.0, keys[i - 1][1], keys[i - 1][2], keys[i - 1][3], {"ease": 0.0, "lag": 0.0, "hold": True, "least": 0.0}))
            holds = [i]
        add = (lo - total) / len(holds)
        for i in holds:
            keys[i] = (keys[i][0] + add,) + keys[i][1:]
    elif total > hi and holds:
        room = sum(max(0.0, keys[i][0] - keys[i][4].get("least", 0.5)) for i in holds)
        cut = min(total - hi, room)
        for i in holds:
            share = max(0.0, keys[i][0] - keys[i][4].get("least", 0.5)) / room if room > 1e-9 else 0.0
            keys[i] = (keys[i][0] - cut * share,) + keys[i][1:]
    return keys


def _moves(g, family, form):
    """A family's keys drawn into g (all but the way home)."""
    rng, k, p0, r0 = g.rng, g.k, g.p0, g.r0
    short = form != "long"
    def hold():
        return 0.5 if short else rng.choice((0.5, 1.0, 1.0, 1.5))
    lively = rng.random() < k                                 # accents: likelier the livelier
    if family == "look":
        n = g.reps(2.2, 1 if form == "short" else 2, 4)
        ts = g.curve(n)
        for i in range(n):
            e = g.eyes()
            x = rng.random()
            if x < 0.1 + 0.2 * k:
                e = (e[0], e[1], e[2] + g.amp(0.3, 0.8))                        # above their heads
            elif x < 0.2 + 0.4 * k:
                e = (e[0], e[1], max(0.1, e[2] - g.amp(0.5, 1.1)))              # down at their feet
            dz = (e[2] - p0[2]) * 0.12 + rng.uniform(-1, 1) * g.amp(0.03, 0.16)  # the body bobs with the gaze
            p = U._add(_toward(p0, e, g.amp(0.06, 0.26)), (0.0, 0.0, dz))
            roll = r0 + rng.uniform(-15, 15) * k
            if lively and i == n - 1:
                g.stop(0.75 * ts[i], p, e, roll, lag=-g.lag)                    # the eyes lead, then it freezes
            else:
                g.arrive(rng.choice((0.75, 1.0)) * ts[i], p, e, roll, lag=-g.lag)
            g.hold(hold())
    elif family == "wave":
        e = g.eyes()
        base = U._add(_toward(p0, e, g.amp(0.03, 0.1)), (0.0, 0.0, g.vert(g.amp(0.03, 0.22), prefer=1)))   # the hand goes up
        side = _side_of(U._sub(e, base))
        amp, ramp = g.amp(0.08, 0.32), 15 + 20 * k
        first = rng.choice((1, -1))
        g.key(1, base, e, r0, lag=-g.lag * 0.5)
        if not short:
            g.anticipate(0.5, U._add(base, U._scale(side, first * amp)), e, r0 + first * ramp)
        n = g.reps(1.0, 2 if form == "short" else 3, 6)
        ts = g.curve(n)
        for i in range(n):
            sgn = first if i % 2 == 0 else -first
            g.key(ts[i], U._add(base, U._scale(side, sgn * amp)), e, r0 + sgn * ramp, ease=0.0, lag=g.lag)  # the roll trails the swing
        g.arrive(1, base, e, r0, lag=g.lag)
        g.hold(1)
    elif family == "nod":
        e = g.eyes()
        g.arrive(1, _toward(p0, e, g.amp(0.02, 0.06)), e, r0, lag=-g.lag)
        base = g.p
        n = g.reps(2.0, 1 if form == "short" else 2, 3)
        ts = g.curve(n)
        for i, depth in enumerate((1.0, 0.65, 0.4)[:n]):
            dip = (12.0 + 18.0 * k) * depth
            pd = U._add(_toward(base, e, g.amp(0.04, 0.16) * depth), (0.0, 0.0, -g.amp(0.07, 0.26) * depth))   # a bow
            d = _turn(U._sub(e, pd), DOWN, dip)
            g.key(ts[i], pd, U._add(pd, d), r0, ease=0.2, lag=g.lag * 0.5)
            g.arrive(ts[i], _toward(base, e, 0.01), e, r0, lag=g.lag * 0.5)
        g.hold(1.5)
    elif family == "reach":
        e = g.eyes()
        out = U._add(_toward(p0, e, g.amp(0.12, 0.32)), (0.0, 0.0, g.vert(g.amp(0.08, 0.38))))   # an offer, high or low
        roll = r0 + rng.uniform(-20, 20)
        g.anticipate(0.75, out, e, roll)                    # leans back first
        g.arrive(1.5, out, e, roll, lag=g.lag)              # the wrist follows through
        g.hold(hold() + 0.5)
        if not short:
            further = U._add(_toward(g.p, e, g.amp(0.03, 0.08)), (0.0, 0.0, 0.15 * (g.p[2] - p0[2])))
            g.key(0.75, further, e, roll + rng.choice((-1, 1)) * (8 + 10 * k), ease=0.2, lag=g.lag)   # "here"
            g.hold(1)
            g.key(1, _toward(p0, e, 0.05), e, r0, lag=g.lag)
    elif family == "tilt":
        e = g.eyes()
        g.arrive(1, _toward(p0, e, 0.03), e, r0, lag=-g.lag)
        base = g.p
        side = _side_of(U._sub(e, base))
        for sgn in rng.sample((1, -1), 1 if short else 2):
            p = U._add(U._add(base, U._scale(side, sgn * g.amp(0.02, 0.08))), (0.0, 0.0, -g.amp(0.01, 0.06)))
            g.arrive(1, p, e, r0 + sgn * (25 + 20 * k), lag=g.lag)
            g.hold(hold())
    elif family == "trace":
        e = g.eyes()
        rad = g.amp(0.2, 0.55)
        n = g.reps(1.25, 2 if form == "short" else 3, 5)
        ts = g.curve(n)
        for i in range(n):
            q = box_point(g.space.box, rng, (-0.4, 0.4), (-0.4, 0.4), (-0.45, 0.45))
            p = U._add(p0, _cap(U._sub(q, p0), rad))
            e = g.eyes() if rng.random() < 0.5 else e
            g.key(rng.choice((1, 1.5)) * ts[i], p, e, r0 + rng.uniform(-20, 20) * k, lag=g.lag,
                  via=i < n - 1 and rng.random() < 0.75)   # flowing through most points
        g.hold(0.5)
    elif family == "peek":
        e = g.eyes()
        low = U._add(_toward(p0, e, g.amp(0.04, 0.12)), (0.0, 0.0, -g.amp(0.05, 0.26)))
        g.key(2, low, e, r0, ease=-0.3, lag=g.lag)                          # ducks and creeps in
        top = U._add(_toward(p0, e, g.amp(0.08, 0.22)), (0.0, 0.0, min(g.room_up, g.amp(0.0, 0.14))))
        cock = rng.choice((-1, 1)) * (20 + 20 * k)
        g.burst(rng.choice((0.75, 1.0, 1.5)), 0.75, top, e, r0 + cock * 0.3, lag=-g.lag * 0.5)   # still, then pops up to peek
        g.arrive(0.5, top, e, r0 + cock, lag=g.lag)                         # cocks the wrist
        g.hold(hold() + 0.5)
        if not short:
            g.arrive(0.5, _toward(p0, e, 0.03 + 0.04 * k), e, r0 + cock * 0.5, ease=0.5)   # darts back a little
            g.hold(1)
    elif family == "shy":
        e = g.eyes()
        g.key(1, _toward(p0, e, 0.03), e, r0, lag=-g.lag)
        d = U._normalize(U._sub(e, g.p))
        away = _turn(_turn(d, DOWN, 30 + 15 * k), _side_of(d), rng.choice((-1, 1)) * (12 + 12 * k))
        pr = U._add(U._add(p0, U._scale(g.back, g.amp(0.08, 0.22))), (0.0, 0.0, -g.amp(0.08, 0.32)))
        pr = U._add(pr, U._scale(_side_of(d), rng.choice((-1, 1)) * g.amp(0.03, 0.18)))      # and turns aside
        g.key(1, g.p, U._add(g.p, U._scale(away, 1.3)), r0 - 10 * k, ease=0.3, lag=-g.lag)   # the gaze drops away first
        g.key(1.5, pr, U._add(pr, U._scale(away, 1.3)), r0 - 15 * k, ease=0.0, lag=g.lag)      # and it shrinks back and down
        pr = g.p
        g.hold(hold() + 0.5)
        if not short:
            half = U._normalize(_lerp(away, U._normalize(U._sub(e, pr)), 0.5))
            g.key(1.5, pr, U._add(pr, U._scale(half, 1.3)), r0 - 5 * k, ease=-0.3)             # a glance back
            g.hold(0.75)
        g.key(2 if form == "long" else 1.5, _toward(p0, e, 0.02), e, r0, ease=-0.3, lag=g.lag)
        g.hold(0.5)
    elif family == "stretch":
        e = g.eyes()
        d = _flat(U._sub(e, p0)) or g.fwd
        rise = min(g.room_up - 0.01, g.amp(0.15, 0.45))
        if rise >= 0.12:
            top = U._add(g.at(up=rise), U._scale(d, g.amp(0.05, 0.18)))        # up and out
            up = _turn(d, UP, 25 + 20 * k)
        else:                                                               # no room above: long, out and across
            top = g.at(fwd=g.amp(0.08, 0.2), side=rng.choice((-1, 1)) * g.amp(0.1, 0.3), up=max(0.0, rise))
            up = _turn(d, UP, 10 + 10 * k)
        roll = r0 + rng.choice((-1, 1)) * (30 + 25 * k)
        g.anticipate(0.75, top, U._add(top, U._scale(up, 1.5)), roll)       # dips first
        g.key(2 if short else 3, top, U._add(top, U._scale(up, 1.5)), roll, ease=-0.2, lag=g.lag * 2)   # the long yawn, the wrist rolling after
        g.hold(hold() + 0.5)
        sag = U._add(_toward(p0, e, 0.02), (0.0, 0.0, -g.amp(0.01, 0.06)))
        g.arrive(1.5 if short else 2, sag, e, r0, lag=g.lag, ease=0.0, settle=0.75)   # settles, a little sag
    elif family == "bounce":
        e = g.eyes()
        base = _toward(p0, e, 0.03)
        g.key(1, base, e, r0, lag=-g.lag)
        depth, sway = g.amp(0.05, 0.18), 3 + 5 * k
        across = _side_of(U._sub(e, base))
        n = g.reps(1.0, 2 if form == "short" else 4, 8)
        ts = g.curve(n)
        groove = g.vert(g.amp(0.0, 0.12)) if rng.random() < 0.6 else 0.0   # halfway it sinks (or lifts) into the groove,
        step = U._scale(across, rng.choice((-1, 1)) * g.amp(0.02, 0.2))       # stepping aside
        for i in range(n):
            sgn = 1 if i % 2 == 0 else -1
            b = U._add(base, (0.0, 0.0, groove if i >= n // 2 else 0.0))
            b = U._add(b, step) if i >= n // 2 else b
            b = U._add(b, U._scale(across, sgn * g.amp(0.01, 0.08)))
            g.key(0.5 * ts[i], U._add(b, (0.0, 0.0, -depth)), e, r0 + sgn * sway, ease=0.3, lag=g.lag * 0.4)   # down on the beat
            g.key(0.5 * ts[i], b, e, r0, ease=-0.2, lag=g.lag * 0.4)
        g.hold(1)
    elif family == "search":
        es = sorted((g.eyes() for _ in range(g.reps(1.5, 2 if form == "short" else 3, 4))),
                    key=lambda x: U._dot(U._sub(x, p0), _side_of(g.view.d0)))
        if rng.random() < 0.5:
            es.reverse()
        travel = g.amp(0.08, 0.36)
        across = _side_of(g.view.d0)
        spread = max(1e-6, max(abs(U._dot(U._sub(x, p0), across)) for x in es))

        def spot(e):
            s = U._dot(U._sub(e, p0), across) / spread
            return U._add(U._add(_toward(p0, e, 0.02), U._scale(across, s * travel)), (0.0, 0.0, rng.uniform(-1, 1) * g.amp(0.04, 0.2)))
        g.key(1.5, spot(es[0]), es[0], r0, lag=-g.lag)                    # to one end of the room
        g.hold(0.5)
        ts = g.curve(len(es) - 1)
        for i, e in enumerate(es[1:]):
            g.arrive(ts[i], spot(e), e, r0 + rng.uniform(-10, 10) * k, lag=-g.lag, ease=0.3)   # quick look
            g.hold(rng.choice((0.5, 0.75, 1.0)))
        e = rng.choice(es)
        found = _toward(p0, e, g.amp(0.05, 0.15))
        if lively:
            g.stop(1, found, e, r0 + rng.choice((-1, 1)) * (10 + 10 * k), lag=-g.lag)   # there you are
        else:
            g.key(1, found, e, r0 + rng.choice((-1, 1)) * (10 + 10 * k), lag=-g.lag)
        g.hold(1.5)
    elif family == "rise":
        e = g.eyes()
        want = g.amp(0.15, 0.6)                                           # the height it travels
        up_r, dn_r = g.room_up - 0.01, g.room_down - 0.01
        top = max(0.0, min(up_r, want * rng.uniform(0.5, 0.75)))
        low = max(0.0, min(dn_r, max(want - top, g.amp(0.05, 0.2))))
        top = max(0.0, min(up_r, max(top, want - low)))
        gather = g.at(fwd=-g.amp(0.02, 0.08), up=-low)
        floor = U._add(gather, U._add(U._scale(g.fwd, 2.0), (0.0, 0.0, -0.8)))   # the eyes down ahead (not at its feet:
                                                                                 # the wrist would whip back up)
        g.key(2, gather, floor, r0 + rng.uniform(-10, 10) * k, ease=-0.2, lag=-g.lag)   # sinks, the eyes first
        peak = g.at(fwd=g.amp(0.02, 0.12), up=top)
        over = U._add(e, (0.0, 0.0, g.amp(0.2, 0.6)))                       # up and out, over their heads
        roll = r0 + rng.choice((-1, 1)) * (15 + 25 * k)
        mid = U._add(_lerp(gather, peak, 0.55), U._scale(g.fwd, 0.03))
        g.burst(0.75 if short else rng.choice((1.0, 1.5)), 0.75, mid, _lerp(floor, over, 0.6), _mix(r0, roll, 0.5), lag=-g.lag, via=True)
        g.arrive(1, peak, over, roll, lag=-g.lag * 0.5)
        g.hold(0.75 if short else rng.choice((1.0, 1.5, 2.0)))
        if not short:
            g.key(2, g.at(fwd=0.02, up=-0.02), e, r0, ease=-0.25, lag=g.lag)   # settles back, slowing
            g.hold(0.5)
    elif family == "dive":
        e = g.eyes()
        depth = max(0.05, min(g.room_down - 0.01, g.amp(0.15, 0.6)))
        out = g.amp(0.06, 0.25)
        lift = min(g.room_up, 0.03 + 0.07 * k)
        if not short:
            g.key(1, g.at(fwd=-0.02 - 0.04 * k, up=lift), U._add(e, (0.0, 0.0, 0.15)), r0, ease=-0.2, lag=-g.lag)   # a lift, the bird rises before it dips
        mid = g.at(fwd=out * 0.6, up=-depth * 0.55)
        bottom = g.at(fwd=out, up=-depth)
        # the gaze leads down and ahead, along the swoop (not at the floor
        # below: the wrist would whip down and back up)
        def ahead(p, drop):
            return g.at(fwd=out + 2.0, up=max(0.2, p[2] - drop) - p0[2])
        swing = rng.choice((-1, 1)) * (10 + 20 * k)
        g.burst(0.35 if short else rng.choice((0.5, 0.75)), 0.75, mid, ahead(mid, 0.7), r0 + swing * 0.5, lag=-g.lag * 0.5, via=True)   # swoops
        g.key(0.5, bottom, ahead(bottom, 0.5), r0 + swing, lag=-g.lag * 0.5)          # dips (a stop: the swoop turns there)
        g.arrive(1, g.at(fwd=out * 0.3, up=min(g.room_up, g.amp(0.03, 0.12))), U._add(e, (0.0, 0.0, 0.1)), r0, ease=0.3, lag=g.lag)   # and back up past the hub
        g.hold(1)
    elif family == "sweep":
        wide = g.amp(0.15, 0.6)
        h = g.amp(0.08, 0.3)
        rainbow = g.room_up > g.room_down * 0.6 if rng.random() < 0.7 else rng.random() < 0.5
        ends, crest = (-0.5 * h, 0.5 * h) if rainbow else (0.5 * h, -0.5 * h)   # low ends, a high middle (or a swing)
        sgn = rng.choice((1, -1))
        a_ = g.clamp(g.at(side=sgn * wide, up=ends))
        m_ = g.clamp(g.at(fwd=g.amp(0.03, 0.12), up=crest))
        b_ = g.clamp(g.at(side=-sgn * wide, up=ends))
        # the gaze turns with the body (a quiet wrist: the arm swings at its
        # quickest), leading it: turned on towards where it is going
        span = g.azimuth(b_) - g.azimuth(a_)
        tilt = (8 + 12 * k) * (1 if rainbow else -1)
        if not short:
            g.anticipate(0.5, a_, g.swung(a_, -0.1 * span), r0)
        g.key(1.5, a_, g.swung(a_, 0.15 * span, -0.5 * tilt), r0 + sgn * 10 * k, lag=-g.lag)   # out to one end, looking across
        g.hold(hold())
        g.key(1.25, m_, g.swung(m_, 0.3 * span, tilt), r0, ease=0.2, lag=-g.lag, via=True)     # the arc across, the gaze leading
        g.key(1.25, b_, g.swung(b_, 0.05 * span, -0.5 * tilt), r0 - sgn * 10 * k, lag=-g.lag)
        g.hold(hold())
        if not short:
            back = g.at(fwd=0.02, side=-sgn * wide * 0.3, up=-0.3 * crest)
            g.key(1.25, back, g.swung(back, -0.2 * span), r0, lag=-g.lag, via=True)   # back through the other height
    elif family == "pop":
        e = g.eyes()
        kind = _pick(rng, (("up", 3), ("down", 2), ("out", 2), ("across", 2)))
        mag = g.amp(0.12, 0.58)
        if kind == "up":
            p, look = g.at(fwd=0.02, up=min(g.room_up, mag)), U._add(e, (0.0, 0.0, 0.6))
        elif kind == "down":
            p, look = g.at(fwd=0.04, up=-min(g.room_down, mag)), U._add(e, (0.0, 0.0, -0.8))
        elif kind == "out":
            p, look = U._add(_toward(p0, e, mag * 0.6), (0.0, 0.0, g.vert(mag * 0.7))), e
        else:
            side = rng.choice((-1, 1))
            p = g.clamp(g.at(side=side * mag, up=g.vert(mag * 0.5)))
            look = g.swung(p, 0.3 * (g.azimuth(p) - g.azimuth(p0)))        # the head whips round with it
        roll = r0 + rng.choice((-1, 1)) * (10 + 25 * k)
        g.params["pose"] = kind
        g.anticipate(0.5, p, look, roll)                                   # a flick the other way
        g.stop(0.5, p, look, roll, lag=-g.lag * 0.5)                       # snaps there, stops dead
        g.hold(1.5, least=1.0)                                             # freeze (it breathes)
    elif family in STRIP_FAMILIES:
        _strip_moves(g, family, form, hold, lively)
    else:
        raise ValueError("unknown family %r" % family)


def _rad(deg):
    return math.radians(deg)


def _strip_moves(g, family, form, hold, lively):
    """The strip families' keys (the LED strip as the prop). Each draws a
    few candidate designs -- other headings, heights, pane tilts, aims --
    and performs the first the probe finds the tool can take (the poses and
    the ways between, the way home included: g.perform); the tool is aimed
    and rolled so the strip lies where the gesture needs it (g.lay), the
    tool tip raised where the strip would come near the floor."""
    rng, k, p0 = g.rng, g.k, g.p0
    short = form != "long"
    off, half = g.strip[0], g.strip[1]
    f, sd = g.fwd, g.side
    g.free = True

    def headings(first):
        """Across-the-floor directions to try: `first`, then turned from it."""
        out = [first]
        for a in (35.0, -35.0, 70.0, -70.0):
            out.append(U._normalize(U._mat_vec(U.axis_angle_matrix(UP, _rad(a)), first)))
        return out

    def ahead(p, d):
        return U._add(p, U._scale(U._normalize(d), 1.5))

    if family == "twirl":
        # the wrist spins the strip like a propeller / a baton: winds up,
        # spins up and brakes past its mark, settles; (long) spins back past
        # where it started; the tool tip drifting a little. Where: face on
        # to a face in the audience, else level ahead, else the hub's aim --
        # the one with the most room to spin (roll_room: J6's limits, the
        # room and the arm, the strip included)
        e = g.eyes()
        out = g.amp(0.04, 0.16)
        want = g.amp(160.0, 300.0)
        drift = U._add(U._scale(sd, rng.uniform(-1.0, 1.0) * g.amp(0.02, 0.06)), (0.0, 0.0, g.amp(0.01, 0.05)))
        dz = rng.uniform(-1.0, 1.0) * g.amp(0.0, 0.08)
        wind0 = 15.0 + 20.0 * k
        designs, rooms = [], []
        aims = ("audience", "level", "hub", "side", "side")
        # out a little, further out, or lower down (under what is close by)
        for target, out_, low_ in [(t_, o_, z_) for o_, z_ in ((out, 0.0), (out + 0.25, 0.0), (out, 0.3)) for t_ in
                                   (e, g.at(fwd=2.5), U._add(p0, U._scale(g.view.d0, 2.0)), g.at(fwd=1.5, side=1.5), g.at(fwd=1.5, side=-1.5))]:
            d = U._normalize(U._sub(target, _toward(p0, target, out_)))
            P = U._add(_toward(p0, target, out_), (0.0, 0.0, -low_))
            r = half * math.sqrt(max(0.0, 1.0 - d[2] ** 2)) + 0.03        # the disc's half height
            P = g.clamp((P[0], P[1], g.strip_z(P[2] + dz, r - off * d[2], r + off * d[2])))
            look = ahead(P, d)
            room = g.roll_room(P, look, g.roll)
            if room is None:
                room = (90.0, 360.0) if rng.random() < 0.5 else (360.0, 90.0)
            sgn = 1.0 if room[1] >= room[0] else -1.0
            spin = max(0.0, min(want, max(room) - 15.0))            # (the overshoot's 10 degrees and 5 spare)
            back = min(room)
            # a longer wind-up where the spin is short of room: the spin
            # starts from further back (J6 travels wind + spin)
            wind = min(max(wind0, TWIRL_MIN_DEG + 5.0 - spin), max(0.0, back - 8.0))
            second = min(g.amp(90.0, 200.0), max(0.0, back - 20.0), max(0.0, 380.0 - spin)) if not short else 0.0
            P2 = U._add(P, drift)
            r_s = g.roll
            moves = [("key", 1.25, P, look, r_s, {"lag": -g.lag}),                                  # out to where it twirls
                     ("key", 0.75, U._add(P, (0.0, 0.0, -0.01 - 0.02 * k)), look, r_s - sgn * wind, {"ease": -0.2}),   # winds up
                     ("arrive", 1.5 if lively else 2.0, P2, ahead(P2, d), r_s + sgn * spin, {"ease": -0.3}),   # spins up, brakes past, settles
                     ("hold", hold())]
            if second > 0.0:
                moves += [("arrive", 1.5, P, look, r_s - sgn * second, {"ease": -0.2}), ("hold", 0.5)]   # back the other way
            else:
                moves += [("key", 1.25, P, look, r_s, {"ease": -0.2})]                            # unwinds
            designs.append(moves)
            rooms.append((spin, sgn, second, room, aims[len(rooms) % len(aims)], wind))
            if spin >= want:                            # all the spin it wants: no need to look further
                break
        # the roomiest first; too little room anywhere (a twirl under
        # TWIRL_MIN_DEG, its wind-up included) and the draw is refused
        order = sorted(range(len(designs)), key=lambda i: -min(rooms[i][0] + rooms[i][5], TWIRL_MIN_DEG + 60.0))
        used = order[g.perform([designs[i] for i in order], fallback=0)]
        spin, sgn, second, room, aim_, wind = rooms[used]
        if spin + wind < TWIRL_MIN_DEG and g.probe is not None:
            g.params["refused"] = "twirl: %.0f degrees of room to spin" % (spin + wind)
        g.params.update(spin_deg=round(sgn * spin, 1), wind_deg=round(-sgn * wind, 1), back_deg=round(-sgn * second, 1),
                        room_deg=[round(x) for x in room], aim=aim_)
    elif family == "wipe":
        # a squeegee on a pane in front of it: the LED face flat to the
        # glass, the strip the blade, overlapping strokes -- side to side
        # with the blade upright, rows stepping down, or top to bottom with
        # it level, columns stepping across -- pressed on for the stroke,
        # lifted off between, the blade trailing it; a shake to finish.
        # The pane faces the audience (or turned, or leaning back like a
        # windscreen, where the arm cannot face it square)
        L = g.amp(0.22, 0.45)                                   # a stroke
        step = g.amp(0.08, 0.16)                                # between strokes (the 1 m blade overlaps them)
        # (as many strokes as the length leaves room for, 3-5; not redrawn
        # fewer when the joints need longer: 3 is the least, the pace below)
        count = max(3, min(5, int(g.left() / (4.0 * g.tempo))))
        ts = g.curve(count)
        press, lift = 0.012 + 0.01 * k, 0.03 + 0.02 * k
        lean = 3.0 + 5.0 * k                                    # the blade trails its stroke (a squeegee's lean)
        span = (count - 1) * step
        order = rng.choice((1.0, -1.0))
        shake = 10.0 + 10.0 * k
        out_ = g.amp(0.04, 0.12)
        across = rng.uniform(-1.0, 1.0) * g.amp(0.0, 0.1)
        upright0 = rng.random() < 0.65                          # (side to side is the elbow's lighter work)
        el0 = math.degrees(math.asin(max(-1.0, min(1.0, g.view.d0[2]))))    # the hub's aim above level
        # the strokes in the time the clip leaves them: quicker when short
        # of it, and then smaller too
        free = g.left() - (1.5 + 0.4 + 0.7) * g.tempo - 0.5

        def pace(upright):
            fct = max(0.6, min(1.0, free / (count * (2.6 if upright else 3.3) * g.tempo)))
            return fct, max(0.18, L * fct * (1.0 if upright else 0.7))
        cands = []
        for tilt in sorted((0.0, 20.0, 35.0, -15.0), key=lambda t: abs(t - el0)):   # nearest the hub's aim first
            for upright in (upright0, not upright0):
                for h in headings(f)[:3]:
                    cands.append((tilt, h, upright))
        designs, panes = [], []
        for tilt, h, upright in cands[:15]:
            fct, Ls = pace(upright)
            n = _turn(h, UP, tilt)                              # the pane's normal, into the glass
            u = U._normalize(U._cross(UP, n))                   # across the pane
            v = U._cross(n, u)                                  # up the pane
            reach = (span if upright else Ls) / 2.0 * abs(v[2]) + 0.01     # the strokes' own height
            ext = reach + (half * abs(v[2]) + 0.02 if upright else 0.04)
            cz = g.strip_z(p0[2] + rng.uniform(-0.05, 0.1), ext, ext, reach)
            c = U._add(g.at(side=across, up=cz - p0[2]), U._scale(h, out_))
            blade = v if upright else u

            def at_(a, b, into):
                return U._add(c, U._add(U._scale(u, a), U._add(U._scale(v, b), U._scale(n, into))))

            def put(beats, p, travel, **kw):
                d = n if travel is None else U._sub(U._scale(n, math.cos(_rad(lean))), U._scale(travel, math.sin(_rad(lean))))
                return ("key", beats, p, ahead(p, d), blade, kw)
            strokes = []
            for i in range(count):
                if upright:                                      # side to side, row by row down
                    s_ = order * (1.0 if i % 2 == 0 else -1.0)
                    b = span / 2.0 - i * step
                    strokes.append(((-s_ * Ls / 2.0, b), (s_ * Ls / 2.0, b), U._scale(u, s_)))
                else:                                            # top to bottom, column by column across
                    a = order * (-span / 2.0 + i * step)
                    strokes.append(((a, Ls / 2.0), (a, -Ls / 2.0), U._scale(v, -1.0)))
            for _ in range(8):                                   # the pane moved in where the space (the arm's reach) cuts it
                pts = [at_(a, b, into) for s0, s1, _ in strokes for a, b in (s0, s1) for into in (press, -lift)]
                cut = max((U._sub(g.clamp(q_), q_) for q_ in pts), key=U._norm)
                if U._norm(cut) < 0.002:
                    break
                c = U._add(c, U._scale(cut, 1.0 + 0.01 / U._norm(cut)))
            (a0, b0), _, _ = strokes[0]
            moves = [put(1.5, at_(a0, b0, -lift - 0.03), None, lag=-g.lag)]   # turns its face to the pane
            for i, (s0, s1, travel) in enumerate(strokes):
                moves.append(put(0.5 * fct, at_(s0[0], s0[1], press), travel, ease=0.2))            # presses on
                moves.append(put(1.5 * fct * ts[i], at_(s1[0], s1[1], press), travel, ease=-0.1))   # the stroke
                if i + 1 < count:
                    n0 = strokes[i + 1][0]
                    if upright:                                  # lifts off, steps down
                        moves.append(put(0.6 * fct, at_((s1[0] + n0[0]) / 2.0, (s1[1] + n0[1]) / 2.0, -lift), None, ease=0.3))
                    else:                                        # lifts off, back up to the next column
                        moves.append(put(0.4 * fct, at_(s1[0], s1[1], -lift), None, ease=0.2, via=True))
                        moves.append(put(0.9 * fct, at_(n0[0], n0[1], -lift), None, ease=0.0))
            last = at_(strokes[-1][1][0], strokes[-1][1][1], -lift - 0.02)
            moves.append(put(0.4, last, None, ease=0.3))                            # lets go
            designs.append(moves)
            panes.append((n, u if upright else v, upright, tilt, Ls, fct))
        used = g.perform(designs)
        g.key(0.35, g.p, g.look, g.roll + shake, ease=0.2)                          # shakes the blade off
        g.key(0.35, g.p, g.look, g.roll - shake, ease=-0.2)
        g.hold(0.5)
        n, axis, upright, tilt, Ls, fct = panes[used]
        g.params.update(pane_normal=[round(x, 4) for x in n], stroke_axis=[round(x, 4) for x in axis],
                        blade="upright" if upright else "level", pane_tilt_deg=tilt, strokes=count,
                        stroke_m=round(Ls, 3), step_m=round(step, 3), stroke_pace=round(fct, 3))
    elif family == "scoop":
        # the strip as a shovel: the aim up and ahead (the face up, the
        # load's side), the strip upright in the plane of the scoop, its
        # lower end ahead and down; takes it up, tips it down and digs in
        # steep, scoops low and ahead levelling the blade, lifts with a
        # toss, settles. Sideways first (read in profile), then turned
        # towards the audience
        # the aim's elevation as it digs, scoops and tosses: up and ahead
        # (the face up, the load on it); else from a little under level --
        # the lower end swinging from under the tool, ahead and up
        angles = [(rng.uniform(22.0, 34.0), rng.uniform(46.0, 56.0), rng.uniform(56.0, 64.0)),
                  (rng.uniform(-30.0, -20.0), rng.uniform(15.0, 25.0), rng.uniform(32.0, 40.0)),
                  (rng.uniform(-40.0, -32.0), rng.uniform(0.0, 8.0), rng.uniform(15.0, 22.0))]
        depth = g.amp(0.35, 0.65)                               # the lower end below the hub's tool tip
        end_z = max(STRIP_FLOOR_M, p0[2] - depth)
        back = g.amp(0.0, 0.06)
        dig_f, top_f, top_up = g.amp(0.1, 0.28), g.amp(0.12, 0.3), min(g.room_up, g.amp(0.05, 0.3))
        up0 = min(g.room_up, g.amp(0.02, 0.06))
        flick_f, flick_up = 0.02 + 0.04 * k, 0.02 + 0.03 * k
        side0 = rng.choice((1.0, -1.0))
        hs = [U._scale(sd, side0), U._scale(sd, -side0), U._normalize(U._add(f, U._scale(sd, side0))),
              U._normalize(U._add(f, U._scale(sd, -side0))), f]

        def drop(gam):                                          # the lower end below the tool tip
            return half * math.cos(_rad(gam)) - off * math.sin(_rad(gam))
        designs, used_as = [], []
        for g_dig, g_lvl, g_toss in angles:
            z_dig = max(g.space.bottom, end_z + drop(g_dig))
            for h in hs:
                h = _flat(h) or f

                def aim(gam):
                    return U._add(U._scale(h, math.cos(_rad(gam))), U._scale(UP, math.sin(_rad(gam))))

                def blade(gam):
                    return U._sub(U._scale(h, math.sin(_rad(gam))), U._scale(UP, math.cos(_rad(gam))))

                def pt(fwd, up):
                    return U._add(p0, U._add(U._scale(h, fwd), (0.0, 0.0, up)))

                def mv(kind, beats, p, gam, **kw):
                    return (kind, beats, p, ahead(p, aim(gam)), blade(gam), kw)
                top = pt(top_f, top_up)
                moves = [mv("key", 1.0, pt(-back - 0.02, up0), g_dig + 10.0, ease=-0.1, lag=-g.lag),     # takes it up
                         mv("key", 1.0, pt(-back, z_dig - p0[2]), g_dig, ease=0.35, lag=g.lag * 0.5),   # tips it down, digs in
                         mv("key", 1.25, pt(dig_f, z_dig - p0[2] + 0.03), g_lvl, ease=-0.1, via=True, lag=g.lag),   # scoops, levelling
                         mv("arrive", 1.0, top, g_toss, ease=0.3, lag=g.lag),                          # lifts it
                         mv("key", 0.5, U._add(top, U._add(U._scale(h, flick_f), (0.0, 0.0, flick_up))), g_lvl - 8.0, ease=0.45),   # the toss
                         ("hold", hold())]
                if not short:
                    moves += [mv("key", 1.25, pt(0.03, -0.02), g_lvl - 12.0, ease=-0.2, lag=g.lag), ("hold", 0.5)]   # settles
                designs.append(moves)
                used_as.append((h, g_dig, g_lvl, g_toss))
        h, g_dig, g_lvl, g_toss = used_as[g.perform(designs)]
        g.params.update(dig_deg=round(g_dig, 1), level_deg=round(g_lvl, 1), toss_deg=round(g_toss, 1),
                        end_z=round(end_z, 3), heading=[round(x, 3) for x in h])
    elif family == "broom":
        # the strip near level, low, brushing side to side: the face down
        # and ahead (the bristles), the head angled into its stroke, the
        # face trailing it, creeping ahead; a flick to finish. Ahead is the
        # audience first, then turned; the face less steep where the wrist
        # cannot point it down
        yaw0, drag = 12.0 + 14.0 * k, 6.0 + 8.0 * k
        drop_ = g.amp(0.35, 0.9)
        out0 = g.amp(0.05, 0.15)
        w = g.amp(0.12, 0.32)
        count = g.reps(1.6, 2 if short else 3, 6)
        ts = g.curve(count)
        first = rng.choice((1.0, -1.0))
        flick_up = g.amp(0.04, 0.12)
        beta0 = rng.uniform(40.0, 60.0)
        designs, looks = [], []
        # the face down and ahead, less steep, a little up (a hub that aims
        # up cannot turn it down past its wrist); ahead, turned; pulled back
        # with the head less angled (a wall close ahead); half as low (the
        # folded arm would meet the strip)
        for beta, h, out_, yaw, dz in [(b_, h_, o_, y_, z_) for b_ in (beta0, 25.0, -15.0) for h_ in headings(f)[:3]
                                       for o_, y_, z_ in ((out0, yaw0, drop_), (-0.08, 0.3 * yaw0, drop_),
                                                          (out0, yaw0, 0.5 * drop_))]:
            a = U._normalize(U._cross(UP, h))                   # across the heading
            z_low = g.strip_z(p0[2] - dz, off * max(0.0, math.sin(_rad(beta))) + 0.03, 0.1)
            base = U._add(g.at(up=z_low - p0[2]), U._scale(h, out_))

            def put(kind, beats, x, travel, fwd=0.0, up=0.0, **kw):
                p = U._add(base, U._add(U._scale(a, x), U._add(U._scale(h, fwd), (0.0, 0.0, up))))
                s = _turn(a, h, travel * yaw) if travel else a  # the head's leading end ahead
                d = U._sub(U._scale(h, math.cos(_rad(beta))), U._scale(UP, math.sin(_rad(beta))))
                if travel:
                    d = _turn(d, U._scale(a, -travel), drag)     # the face trailing
                d = U._normalize(U._sub(d, U._scale(s, U._dot(d, s))))
                return (kind, beats, p, ahead(p, d), s, kw)
            for turn_first in (False, True):                # (the head turned level up here, then down: past the arm)
                moves = []
                if not short:
                    moves.append(("key", 0.75, g.at(up=min(g.room_up, 0.02 + 0.04 * k)), None, None, {"ease": -0.2}))   # a breath in
                if turn_first:
                    moves.append(put("key", 1.0, 0.0, 0.0, fwd=-out_, up=p0[2] - z_low, ease=-0.1))
                moves.append(put("key", 1.0 if turn_first else 2.0, 0.0, 0.0, ease=-0.15, lag=-g.lag))   # down to the floor, the head level
                sg, push = first, 0.0
                for i in range(count):
                    sg = first * (1.0 if i % 2 == 0 else -1.0)
                    push = (0.015 + 0.015 * k) * i                                          # creeping ahead
                    moves.append(put("key", 0.5 * ts[i], 0.0, sg, fwd=push, up=-0.015, via=True, ease=0.0, lag=g.lag * 0.5))
                    moves.append(put("key", 0.5 * ts[i], sg * w, sg, fwd=push + 0.01, up=0.01, ease=0.1, lag=g.lag * 0.5))
                moves.append(put("arrive", 0.75, -sg * w * 0.5, -sg, fwd=push + 0.05, up=flick_up, ease=0.4))   # flicks the dust away
                moves.append(("hold", hold()))
                designs.append(moves)
                looks.append((beta, h, z_low, out_))
        # the first move of the long form keeps its gaze (None): lay it as it is
        for moves in designs:
            for i, m in enumerate(moves):
                if m[0] != "hold" and m[3] is None:
                    moves[i] = (m[0], m[1], m[2], U._add(m[2], U._sub(g.look, g.p)), None, m[5])
        used = g.perform(designs)
        beta, h, z_low, out_ = looks[used]
        g.params.update(face_deg=round(beta, 1), heading=[round(x, 3) for x in h], strokes=count,
                        stroke_m=round(2.0 * w, 3), low_z=round(z_low, 3))
    elif family == "salute":
        # the strip raised upright like a sabre before the face, held; a
        # flourish (spun end over end, or a cut across and back); lowered
        # across and forward, a bow; back
        e = g.eyes()
        rise = g.vert(g.amp(0.08, 0.3), prefer=1)
        fwd_ = g.amp(0.03, 0.12)
        dip_up = -min(g.room_down, g.amp(0.02, 0.07))
        low_f, low_up = g.amp(0.03, 0.1), -min(g.room_down, g.amp(0.0, 0.08))
        bow = 25.0 + 15.0 * k
        cut0 = 70.0 + 30.0 * k
        spin_ok = rng.random() < 0.6
        designs = []
        for target in (e, g.at(fwd=2.5, up=0.3), U._add(p0, U._scale(g.view.d0, 2.0))):
            P = U._add(_toward(p0, target, fwd_), (0.0, 0.0, rise))
            d = U._normalize(U._sub(target, P))
            sz = math.sqrt(max(0.0, 1.0 - d[2] ** 2))
            P = g.clamp((P[0], P[1], g.strip_z(P[2], half * sz - off * d[2] + 0.02, half * sz + off * d[2] + 0.02)))
            look = ahead(P, d)
            dip = g.at(up=dip_up)
            L = g.at(fwd=low_f, up=low_up)
            lo = ahead(L, _turn(d, DOWN, bow))
            moves = [("key", 0.75, dip, ahead(dip, _turn(g.gaze(), DOWN, 8.0 + 8.0 * k)), None, {"ease": -0.2}),   # draws: dips
                     ("arrive", 1.25, P, look, UP, {"lag": g.lag}),                                             # the sabre raised
                     ("hold", hold() + 0.5)]                                                                      # the salute
            # the flourish: the frame there is not known before it is laid;
            # a spin needs 200 degrees of roll room one way (roll_room, as laid)
            designs.append((moves, P, look, L, lo))
        chosen = None
        for moves, P, look, L, lo in designs:
            laid = g.lay([m[2:5] for m in moves if m[0] != "hold"])
            if laid is not None:
                chosen = (moves, P, look, L, lo)
                break
        chosen = chosen or designs[0]
        moves, P, look, L, lo = chosen
        for m in moves:
            if m[0] == "hold":
                g.hold(m[1])
            else:
                p_, l_, r_ = g.lay([m[2:5]], check=False)[0]
                getattr(g, m[0])(m[1], p_, l_, r_, **m[5])
        room = g.roll_room(g.p, g.look, g.roll)
        sgn = rng.choice((1.0, -1.0)) if room is None else (1.0 if room[1] >= room[0] else -1.0)
        r_up = g.roll
        kind = "spin" if spin_ok and (room is None or max(room) >= 200.0) else "cut"
        tail = []
        if kind == "spin":
            tail.append(("stop", 1.0, g.p, g.look, r_up + sgn * 180.0, {}))                    # spun end over end, stops dead
        else:
            cut = min(cut0, max(20.0, (max(room) if room else 90.0) - 15.0))
            tail += [("key", 0.75, U._add(g.p, U._scale(sd, 0.04 * sgn)), g.look, r_up + sgn * cut, {"ease": 0.4}),   # a cut across
                     ("arrive", 0.75, g.p, g.look, r_up, {"ease": 0.3})]                                                # and back up
        lo_half = ahead(L, _turn(U._normalize(U._sub(look, P)), DOWN, 0.5 * bow))
        ends = [[("hold", 0.75), ("key", 1.5, L, lo, sd, {"ease": -0.2, "lag": g.lag}), ("hold", 0.75)],   # lowered across, a bow
                [("hold", 0.75), ("key", 1.5, L, lo, None, {"ease": -0.2, "lag": g.lag}), ("hold", 0.75)],  # (not turned)
                [("hold", 0.75), ("key", 1.5, L, lo_half, None, {"ease": -0.2, "lag": g.lag}), ("hold", 0.75)],   # (a nod)
                [("hold", 1.0)]]                                                                          # (held, then home)
        g.perform([tail + e_ for e_ in ends])
        g.params.update(flourish=kind, aim=("audience", "level", "hub")[designs.index(chosen)])
    else:
        raise ValueError("unknown family %r" % family)


def draw_keys(family, home, zones, rng, beat, k, space=None, length=None, probe=None):
    """(keys, params): the keys after the start [(duration s, tcp, look,
    roll, options)], ending at home at rest, and what was drawn. home =
    (tcp, look, roll) of the hub; k = intensity 0..1; space: a Space (the
    zones' when None); length: (form, (lo, hi) s), drawn when None; probe:
    a Probe (make's), which the strip families ask where the strip may go.
    options: ease (the warp of the min-jerk clock), lag (s the gaze / roll
    trails the TCP; < 0 leads it), hold (a hold: may be lengthened or
    shortened, not under least_s), via (passed through without stopping),
    strip (a strip family's key: its aim and roll are the prop's, kept by
    wrist_safe, shrink and strip_safe).
    Repeats are counted from the beats the length leaves; when the joints
    will likely need longer than it (estimate), the same draw is made again
    with fewer."""
    space = space or Space(zones, home[0])
    form, (lo_s, hi_s) = length or clip_length(rng, k, family)
    state, budget = rng.getstate(), hi_s / beat
    for attempt in range(4):
        rng.setstate(state)
        g = _Draw(home, zones, rng, beat, k, space, budget, form)
        g.probe = probe
        _moves(g, family, form)
        g.home(min(g.lag, 0.25 * beat))
        if family in STRIP_FAMILIES:
            for kk in g.keys:
                kk[4]["strip"] = True
        est = estimate(g.keys, beat)
        if est <= hi_s or not g.repeats:
            break
        budget *= max(0.5, 0.95 * hi_s / est)
    keys = _fit_beats(g.keys, lo_s / beat, hi_s / beat)
    out = []
    for b, p, look, roll, opt in keys:
        opt = dict(opt)
        if "least" in opt:
            opt["least_s"] = opt.pop("least") * beat
        out.append((b * beat, p, look, roll, opt))
    prm = dict(g.params, family=family, form=form, length_s=[round(lo_s, 3), round(hi_s, 3)],
               tempo_factor=round(g.tempo, 3), space={"zone": space.name, "z": [round(space.bottom, 3), round(space.top, 3)]})
    return out, prm


def keys_for(family, home, zones, rng, beat, k, space=None, length=None):
    """draw_keys' keys alone."""
    return draw_keys(family, home, zones, rng, beat, k, space, length)[0]


WRIST_KEY_MIN = 0.35             # |sin J5| a key's pose keeps (the sampler refuses 0.2: the wrist singularity)


def wrist_safe(rig, hub_q, keys, home):
    """keys with the gaze of any whose pose -- or the way to it -- would bring
    the wrist near its singularity (|sin J5| < WRIST_KEY_MIN, or no pose)
    turned back towards the hub's aim, a third at a time (a hub already
    near it -- J5 of 25-35 degrees -- cannot look as far about)."""
    p0, look0, r0 = home
    d0 = U._normalize(U._sub(look0, p0))
    out, prev, last = [], list(hub_q), (p0, d0, r0)

    def clear(p, d, roll, near):
        """The pose at a key and on the way to it (a third, two thirds) with
        the wrist clear of its singularity, or None."""
        q = near
        for u in (1.0 / 3.0, 2.0 / 3.0, 1.0):
            pu = _lerp(last[0], p, u)
            du = U._normalize(_lerp(last[1], d, u))
            q = rig.solve(pu, du, _mix(last[2], roll, u), q)
            if q is None or abs(math.sin(math.radians(q[4]))) < WRIST_KEY_MIN:
                return None
        return q
    for key in keys:
        dur, p, look, roll = key[:4]
        d = U._normalize(U._sub(look, p))
        dist = U._norm(U._sub(look, p))
        if len(key) > 4 and key[4].get("strip"):           # the prop's aim: as drawn (the sampler refuses a singular wrist)
            out.append(tuple(key))
            last = (p, d, roll)
            continue
        best = None
        for blend in (0.0, 0.35, 0.7, 1.0):
            db = U._normalize(_lerp(d, d0, blend))
            q = clear(p, db, _mix(roll, r0, blend), prev)
            if q is not None:
                best = (blend, db, q)
                break
        if best is None or best[0] == 0.0:
            out.append(tuple(key))
            if best:
                prev = best[2]
            last = (p, d, roll)
            continue
        blend, db, q = best
        out.append((dur, p, U._add(p, U._scale(db, dist)), _mix(roll, r0, blend)) + tuple(key[4:]))
        prev, last = q, (p, db, _mix(roll, r0, blend))
    return out


def shrink(keys, home, size):
    """The same gesture smaller: every key's tool tip, aim and roll moved
    towards the hub's by 1 - size (the look points keep their distance);
    a strip key's tool tip only (its aim and roll are the prop's)."""
    p0, look0, r0 = home
    d0 = U._normalize(U._sub(look0, p0))
    out = []
    for key in keys:
        dur, p, look, roll = key[:4]
        q = _lerp(p0, p, size)
        if len(key) > 4 and key[4].get("strip"):
            out.append((dur, tuple(q), U._add(q, U._sub(look, p)), roll) + tuple(key[4:]))
            continue
        d = U._normalize(U._sub(look, p))
        d2 = U._normalize(_lerp(d0, d, size))
        out.append((dur, tuple(q), U._add(q, U._scale(d2, U._norm(U._sub(look, p)))), r0 + (roll - r0) * size) + tuple(key[4:]))
    return out


PROBE_PAD_M = 0.006              # a pose the strip families design with keeps this much beyond every margin


class Probe:
    """Can the tool stand at a pose (p, R)? The IK pose nearest a seed (the
    hub's) within the limits and their margin, the wrist clear of its
    singularity (WRIST_KEY_MIN), PROBE_PAD_M clear of the room (beyond every
    obstacle's margin) and of the arm itself -- the strip included
    (collision.load_model carries the tool); the way between two poses
    strays a little from both. pose(p, R, near) -> joints, or None
    (last: why)."""

    def __init__(self, rig, hub_q, env=None, pad=PROBE_PAD_M):
        import collision as CL
        self.rig, self.hub_q, self.env, self.pad = rig, list(hub_q), env, pad
        self.model = CL.load_model("fr20")
        self.last = None

    def pose(self, p, R, near=None):
        import collision as CL
        q = self.rig.solve(p, None, None, near or self.hub_q, R=R)
        self.last = "out of reach or past a limit" if q is None else "the wrist singularity"
        if q is None or abs(math.sin(math.radians(q[4]))) < WRIST_KEY_MIN:
            return None
        self.last = "the room or itself"
        if self.env is not None and CL.pose_clearance(self.model, self.env, q, cap=self.pad)[0] < self.pad:
            return None
        caps, _ = CL.capsules(self.model, q)
        clear = all(CL._seg_seg_dist(caps[i][1], caps[i][2], caps[j][1], caps[j][2]) - caps[i][3] - caps[j][3] >= self.pad
                    for i, j in self.model["pairs"])
        return q if clear else None


def strip_safe(rig, hub_q, keys, home, probe):
    """keys with the roll of any whose pose -- or the way to it, at a third
    and two thirds -- would put the strip (or the arm) into the room or into
    the arm turned by the least of STRIP_ROLLS that clears them, the frames
    as the sampler carries them; the strip is symmetric, so +-90 degrees
    reach every way it can lie across the aim. The keys after it keep their
    own roll. A strip family's keys and the way home are left as drawn;
    nothing changes without a probe."""
    if probe is None:
        return keys
    R, last, prev = rig.tool(hub_q)[0], home[2], (home[0], home[1])
    q = list(hub_q)
    out = []

    def way(p, look, droll):
        """The joints at the key and on the way to it (None: blocked)."""
        qa = q
        for u in (1.0 / 3.0, 2.0 / 3.0, 1.0):
            pu, lu = _lerp(prev[0], p, u), _lerp(prev[1], look, u)
            qa = probe.pose(pu, transport(R, U._sub(lu, pu), droll * u), near=qa)
            if qa is None:
                return None
        return qa
    for key in _as5(keys):
        dur, p, look, roll, opt = key[:5]
        qk = way(p, look, roll - last)
        if qk is None and not (opt.get("strip") or opt.get("home")):
            for dr in STRIP_ROLLS:
                qk = way(p, look, roll + dr - last)
                if qk is not None:
                    roll += dr
                    break
        out.append((dur, p, look, roll) + tuple(key[4:]))
        R, last, prev = transport(R, U._sub(look, p), roll - last), roll, (p, look)
        q = qk if qk is not None else probe.pose(p, R, near=q) or q
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


class _Track:
    """A channel through keyed values (lists) at times: min-jerk on a warped
    clock (the reached key's ease) from key to key; a run of keys marked via
    is passed through without stopping -- one eased clock (the run's first
    key's ease) over the whole run, a cubic Hermite through its keys
    (Catmull-Rom tangents), so speed and acceleration are zero only where
    the run starts and stops."""

    def __init__(self, times, values, eases, via):
        self.t, self.v, self.e = times, [list(x) for x in values], eases
        n = len(times)
        self.stops = [i for i in range(n) if i in (0, n - 1) or not via[i]]
        self.st = [times[i] for i in self.stops]
        self.m = [None] * n
        for a, b in zip(self.stops, self.stops[1:]):
            if b - a < 2:
                continue
            for i in range(a, b + 1):
                i0, i1 = max(a, i - 1), min(b, i + 1)
                dt = max(1e-9, times[i1] - times[i0])
                self.m[i] = [(y - x) / dt for x, y in zip(self.v[i0], self.v[i1])]

    def at(self, t):
        k = max(0, min(len(self.stops) - 2, bisect.bisect_right(self.st, t) - 1))
        a, b = self.stops[k], self.stops[k + 1]
        ta, tb = self.t[a], self.t[b]
        u = (t - ta) / max(1e-9, tb - ta)
        if b == a + 1:
            return _lerp(self.v[a], self.v[b], _ease(u, self.e[b]))
        tau = ta + _ease(u, self.e[a + 1]) * (tb - ta)
        j = max(a, min(b - 1, bisect.bisect_right(self.t, tau) - 1))
        h = self.t[j + 1] - self.t[j]
        if h < 1e-9:
            return list(self.v[j + 1])
        s = (tau - self.t[j]) / h
        h00, h10, h01, h11 = 2 * s ** 3 - 3 * s * s + 1, s ** 3 - 2 * s * s + s, -2 * s ** 3 + 3 * s * s, s ** 3 - s * s
        return [h00 * p + h10 * h * m0 + h01 * q + h11 * h * m1
                for p, q, m0, m1 in zip(self.v[j], self.v[j + 1], self.m[j], self.m[j + 1])]


def _frames(rig, hub_q, keys, home, style, n, total, extra_roll=None):
    """[(t, p, R)] per 24 fps frame: the TCP on its clock, the look point and
    roll on theirs (the lags), plus the breathing; the tool frame carried
    along (transport), plus extra_roll(t) degrees about z (the end correction)."""
    p0, look0, r0 = home
    tp, P_, L_, R_, E_, V_, lags = [0.0], [list(p0)], [list(look0)], [[float(r0)]], [0.0], [False], [0.0]
    for dur, p, look, roll, opt in keys:
        tp.append(tp[-1] + dur)
        P_.append(list(p))
        L_.append(list(look))
        R_.append([float(roll)])
        E_.append(opt.get("ease", 0.0))
        V_.append(bool(opt.get("via")))
        lags.append(opt.get("lag", 0.0))
    tg = _gaze_times(tp, lags)
    tr_p, tr_l, tr_r = _Track(tp, P_, E_, V_), _Track(tg, L_, E_, V_), _Track(tg, R_, E_, V_)
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
        p = tr_p.at(t)
        look = tr_l.at(t)
        roll = tr_r.at(t)[0]
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
    def fix(t):
        return left * _minjerk((t - t_home) / max(1e-9, t_end - t_home)) if t > t_home else 0.0
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


def _squeeze(keys, over):
    """keys with their holds shortened (not under least_s) by up to `over` s in all."""
    room = [(i, max(0.0, k[0] - k[4].get("least_s", 0.0))) for i, k in enumerate(keys) if k[4].get("hold")]
    total = sum(r for _, r in room)
    if total <= 1e-9 or over <= 0.0:
        return keys
    cut = min(over, total)
    out = list(keys)
    for i, r in room:
        out[i] = (keys[i][0] - cut * r / total,) + tuple(keys[i][1:])
    return out


def fit(rig, hub_q, keys, home, style, beat, vel, acc, rounds=6, soft_s=MAX_CLIP_S):
    """(ts, qs, keys) with each move given the time its joints need: the
    player's need (v / limit, sqrt(a / limit)) over each key's span, that
    key's duration stretched by it and rounded up to an eighth of a beat
    (a flowing run's keys together, by the most any of them needs), until
    nothing needs slowing; when the clip runs over soft_s (the length it
    was drawn to) the holds give time back (not under their least). None
    when a key cannot be sampled, a move would need more than MAX_STRETCH
    times its beats, or the clip would run over MAX_CLIP_S."""
    keys = _as5(keys)
    base = [k[0] for k in keys]
    eighth = beat / 8.0
    for _ in range(rounds):
        total = sum(k[0] for k in keys)
        if total > min(soft_s, MAX_CLIP_S) + 1e-9:
            keys = _squeeze(keys, total - min(soft_s, MAX_CLIP_S))
        if sum(k[0] for k in keys) > MAX_CLIP_S + 1e-6:
            return None
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
        # a flowing run is one move: its keys slow together
        run, runs = [], []
        for j, k in enumerate(keys):
            run.append(j)
            if not k[4].get("via"):
                runs.append(run)
                run = []
        if run:
            runs.append(run)
        for r in runs:
            top = max(need[j] for j in r)
            for j in r:
                need[j] = top
        new = []
        for j, k in enumerate(keys):
            d = k[0]
            if need[j] > 1.0:
                d = math.ceil(d * need[j] * 1.02 / eighth - 1e-9) * eighth
                if d > MAX_STRETCH * max(base[j], beat) + 1e-9:     # a crawl (a short key may take a few beats)
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
    def m(o):
        return o.get("margin_m", margin) if o["role"] == "obstacle" else 0.0
    keep = [o for i, o in enumerate(objs) if i not in bound or bound[i] - m(o) < far]
    rep = CL.check(model, dict(env, objects=keep), ts, qs)
    c = rep.get("min_env_clearance_m")
    if c is None or c > far:
        rep = CL.check(model, env, ts, qs)
    return rep


FLOOR_CLEAR_M = 0.03             # a hub's links (but the base's) at least this above the floor
HUB_HEADROOM_DEG = 30.0          # a hub's joints this far from their limits, when a roll allows


def _robot_clear(q):
    """No link near the floor, no two links touching (the robot alone, no room)."""
    import collision as CL
    model = CL.load_model("fr20")
    caps, _ = CL.capsules(model, q)
    if any(min(a[2], b[2]) - r < FLOOR_CLEAR_M for name, a, b, r in caps if name not in CL.FIXED_LINKS):
        return False
    return all(CL._seg_seg_dist(caps[i][1], caps[i][2], caps[j][1], caps[j][2]) - caps[i][3] - caps[j][3] >= 0.0
               for i, j in model["pairs"])


def hub_pose(rig, tcp, look, near, rolls=range(-180, 180, 10)):
    """A hub from where the tool tip is and what it looks at: the IK pose
    nearest `near` over rolls about the aim (so the arm keeps its elbow /
    wrist configuration). When that pose puts a link into the floor or into
    another link (a low hub from a high one's seed: the nearest branch is
    elbow-down), every branch at every roll is weighed instead, the clear
    ones first. None when no roll reaches it clear of the wrist singularity."""
    d = U._sub(look, tcp)

    def cost(q):
        # a hub the wrist can move from: far from its singularity first
        # (near it the wrist spins for small changes of aim), then as far
        # from the joint limits as it can be, up to HUB_HEADROOM_DEG (a J6
        # at its stop cannot roll one way), then nearest (the largest
        # change, then all of them)
        room = min(min(x - lo, hi - x) for x, (lo, hi) in zip(q, rig.limits))
        diff = [abs(a - b) for a, b in zip(q, near)]
        return (abs(math.sin(math.radians(q[4]))) < 0.7, -min(room, HUB_HEADROOM_DEG), round(max(diff), 6), sum(diff))
    best = None
    for r in rolls:
        q = rig.solve(tcp, d, r, near)
        if q is None or abs(math.sin(math.radians(q[4]))) < 0.3:
            continue
        if best is None or cost(q) < best[0]:
            best = (cost(q), q)
    if best is not None and _robot_clear(best[1]):
        return best[1]
    clear = None
    for r in rolls:
        for s in rig.solutions(tcp, d, r, near):
            q = [b - 360.0 * round((b - a) / 360.0) for a, b in zip(near, s["q"])]
            q = q if rig.within(q) else list(s["q"])
            if not rig.within(q) or abs(math.sin(math.radians(q[4]))) < 0.3 or not _robot_clear(q):
                continue
            if clear is None or cost(q) < clear[0]:
                clear = (cost(q), q)
    return clear[1] if clear else (best and best[1])


def home_of(rig, hub_q):
    """(tcp, look point 1.5 m along the tool, roll) of a hub pose."""
    R, tcp, d = rig.tool(hub_q)
    return tcp, U._add(tcp, U._scale(d, 1.5)), roll_of(R, d)


def make(rig, hub_q, family, zones, rng, bpm=90, intensity=0.6, clip_id=None, env=None, safety=PLAN_SAFETY):
    """A gesture clip from hub_q, or None when this draw does not work at
    any size (unreachable key, branch flip, too violent, too long, hits the
    room). labels["family"], labels["params"]: what was drawn and fitted."""
    import collision as CL
    import motion_labels
    home = home_of(rig, hub_q)
    beat = 60.0 / bpm
    space = Space(zones, home[0], env)
    probe = Probe(rig, hub_q, env)
    keys, prm = draw_keys(family, home, zones, rng, beat, intensity, space, probe=probe)
    if prm.get("refused"):                              # a strip family found no room for itself
        return None
    keys = wrist_safe(rig, hub_q, keys, home)
    style = style_for(rng, beat, intensity)
    vel, acc = [v * safety for v in rig.vel], [a * safety for a in rig.acc]
    nominal = sum(k[0] for k in keys)
    model = CL.load_model("fr20") if env is not None else None
    for size in SIZES:
        ks = strip_safe(rig, hub_q, keys if size == 1.0 else shrink(keys, home, size), home, probe)
        got = fit(rig, hub_q, ks, home, style, beat, vel, acc, soft_s=prm["length_s"][1])
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
    clip["labels"]["family"] = family
    prm.update(bpm=bpm, intensity=round(intensity, 3), size=size, slowed=clip["style"]["slowed"],
               reach_m=round(max(math.dist(k[1], home[0]) for k in fitted), 3))
    clip["labels"]["params"] = prm
    return clip


def wrist_share(qs):
    """Fraction of the joint travel done by J4-J6 (how much the wrist acts)."""
    travel = [sum(abs(b[j] - a[j]) for a, b in zip(qs, qs[1:])) for j in range(6)]
    return sum(travel[3:]) / max(1e-9, sum(travel))


def tcp_stats(tcp):
    """{zspan, extent (largest axis span), travel (bounding-box diagonal),
    vmean, vpeak (m/s at 24 fps)} of a TCP path."""
    xs = list(zip(*tcp))
    spans = [max(a) - min(a) for a in xs]
    sp = [math.dist(a, b) * FPS for a, b in zip(tcp, tcp[1:])] or [0.0]
    return {"zspan": spans[2], "extent": max(spans), "travel": math.sqrt(sum(s * s for s in spans)),
            "vmean": sum(sp) / len(sp), "vpeak": max(sp)}


WIPE_FACE_DEG = 20.0             # a wipe's stroke: the LED face this near the pane's normal


def count_strokes(tcps, aims, normal, axis, least=0.1, face=WIPE_FACE_DEG):
    """How many wiping strokes a TCP path makes: runs of frames pressed on
    the pane (the tool tip within 1 cm of its deepest along the normal) with
    the face (the tool's aim) within `face` degrees of the normal, each run
    split where it turns back along the stroke axis; a stroke at least
    `least` m long."""
    depth = [U._dot(p, normal) for p in tcps]
    top = max(depth)
    on = [dp >= top - 0.01 and _angle(d, normal) <= face for d, dp in zip(aims, depth)]
    count, run = 0, []
    for i, ok in enumerate(on + [False]):
        if ok:
            run.append(U._dot(tcps[i], axis))
            continue
        if run:
            count += sum(x >= least for x in _pieces(run))
        run = []
    return count


def _pieces(xs, turn=0.01):
    """The lengths of the monotone pieces of xs (a turn back of more than `turn` ends one)."""
    out, start, ext, sgn = [], xs[0], xs[0], 0
    for x in xs[1:]:
        if sgn == 0:
            if abs(x - start) > turn:
                sgn, ext = (1 if x > start else -1), x
            continue
        if (x - ext) * sgn >= 0:
            ext = x
        elif abs(x - ext) > turn:
            out.append(abs(ext - start))
            start, sgn, ext = ext, -sgn, x
    out.append(abs(ext - start))
    return out


def strip_stats(rig, clip):
    """What the strip did in a clip (the hub is its first pose): J6's range
    (deg), the strip's lowest end (m) and how far under the hub's tool tip it
    went, and for a wipe its strokes (count_strokes on the pane of its params)."""
    import collision as CL
    model = CL.load_model("fr20")
    qs = [p["q"] for p in clip["points"]]
    lows = []
    for q in qs:
        caps, _ = CL.capsules(model, q)
        s = next((c for c in caps if c[0] == "tool_strip"), None)
        lows.append(min(s[1][2], s[2][2]) if s else math.nan)
    hub_z = rig.tool(qs[0])[1][2]
    out = {"s_j6": max(q[5] for q in qs) - min(q[5] for q in qs), "s_low": min(lows), "s_drop": hub_z - min(lows),
           "s_back": abs(lows[-1] - lows[0]), "s_hub_low": lows[0]}
    prm = (clip.get("labels") or {}).get("params") or {}
    if prm.get("pane_normal"):
        tools = [rig.tool(q) for q in qs]
        out["s_strokes"] = count_strokes([t[1] for t in tools], [t[2] for t in tools], prm["pane_normal"],
                                         prm["stroke_axis"], least=min(0.1, 0.5 * prm["stroke_m"] * clip["style"]["size"]))
    return out


# --------------------------------------------------------------------------
# self-test: the hubs of shows/party.json and a low and a high one, every
# family, a sweep of seeds, tempi and intensities
# --------------------------------------------------------------------------

SEEDS = 10                       # draws per family and hub in the sweep
TRIES = 3                        # a seed's draws before it counts as not made
STILL_DEG = 0.005                # a frame step under this on every joint is "frozen"
REST_STEP_DEG = 0.005            # the first and last frame steps, at most (clips join at rest)
SWEEP_BPM = (70, 140)
SWEEP_K = (0.2, 1.0)
SYNTHETIC = (("synth_low", 0.55), ("synth_high", 1.5))   # hubs at the greet hub's x, y at these heights


def _room(cfg=None):
    """The lab as the show sees it: envs/volvox_lab.usda (OpenUSD), or the
    .json it came from, with the show's stage and paper (show.show_env)."""
    import collision as CL
    errs = []
    for name in ("volvox_lab.usda", "volvox_lab.json"):
        path = os.path.join(ROOT, "envs", name)
        if not os.path.exists(path):
            continue
        try:
            env = CL.load_env(path)
        except (ValueError, ImportError) as e:
            errs.append("%s: %s" % (name, e))
            continue
        if cfg:
            import show
            env = show.show_env(env, cfg, cfg["margins"]["idle_canvas_m"])
        return env
    raise SystemExit("no room to check against: %s" % ("; ".join(errs) or "envs/volvox_lab.* missing"))


def synthetic_hubs(rig, cfg, env=None):
    """{name: (q, tcp)}: hubs at the greet hub's x, y and SYNTHETIC heights,
    looking at the audience's centre (hub_pose, the greet hub's seed); when
    that spot has no clear pose, the nearest that does, searched in 5 cm
    steps up / down and 10 cm across (inside the stage)."""
    import collision as CL
    g = cfg["hubs"]["greet"]
    look = cfg["zones"]["audience"]["center"]
    model = CL.load_model("fr20") if env is not None else None
    space = Space(cfg["zones"], g["tcp"], env)
    out = {}
    for name, z in SYNTHETIC:
        tries = [(0.0, 0.0, 0.0)] + [(dx, dy, dz) for dz in (0.0, 0.05, -0.05, 0.1, -0.1) for dx in (0.0, 0.1, -0.1)
                                      for dy in (0.0, 0.1, -0.1) if (dx, dy, dz) != (0.0, 0.0, 0.0)]
        for dx, dy, dz in tries:
            tcp = (g["tcp"][0] + dx, g["tcp"][1] + dy, z + dz)
            if env is not None and not space.contains((tcp[0], tcp[1], space.box["center"][2])):
                continue
            q = hub_pose(rig, tcp, look, g.get("near") or g.get("q"))
            if q is None or not _robot_clear(q):
                continue
            if env is not None and not CL.pose_clearance(model, env, q)[1]:
                continue
            out[name] = ([round(x, 4) for x in q], [round(x, 4) for x in tcp])
            break
    return out


def _show_hubs(rig, synthetic=False, env=None):
    """(zones, {name: q}) of shows/party.json, the hubs resolved as
    show.resolve_hubs does (a tool tip + look point through hub_pose); with
    synthetic, the low and high hubs added (synthetic_hubs)."""
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
    if synthetic:
        for name, (q, _) in synthetic_hubs(rig, cfg, env).items():
            hubs[name] = q
    if env is not None:                                  # a hub not clear itself: its stand-in (stand_in)
        for name in list(hubs):
            q = stand_in(rig, hubs[name], env)
            if q is not None:
                hubs[name] = q
    return cfg["zones"], hubs


def hub_clear(q, env):
    """None when a hub pose is clear of the room and of the arm itself, the
    strip included; else what it hits (collision.describe)."""
    import collision as CL
    rep = CL.check(CL.load_model("fr20"), env, [0.0], [q])
    return None if rep["ok"] else CL.describe(rep)


def stand_in(rig, q, env, most=90):
    """What the self-tests use for a hub that is not clear itself (the
    show's config must move it): the hub turned about its tool axis (J6,
    10-degree steps, up to `most`) by the least that clears it with 10
    degrees to spare either way; None when it is clear, or nothing is."""
    if hub_clear(q, env) is None:
        return None
    for a in range(10, most + 1, 10):
        for sgn in (-1, 1):
            turned = [[x + (sgn * b if j == 5 else 0.0) for j, x in enumerate(q)] for b in (a, a + 10, a + 20)]
            if all(rig.within(t) and hub_clear(t, env) is None for t in turned):
                return [round(x, 4) for x in turned[1]]
    return None


_W = {}


def _worker_init():
    import json
    import collision as CL
    rig = Rig()
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    env = _room(cfg)
    zones, hubs = _show_hubs(rig, True, env)
    _W.update(rig=rig, zones=zones, hubs=hubs, env=env, model=CL.load_model("fr20"))


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
    """One seed of one family from one hub: up to TRIES draws (bpm and
    intensity drawn from SWEEP_BPM, SWEEP_K, or the fixed k given), the first
    clip checked. A summary, not the clip."""
    import random
    fam, hub_name, seed = job[:3]
    k_fixed = job[3] if len(job) > 3 else None
    rig, hub = _W["rig"], _W["hubs"][hub_name]
    rng = random.Random(seed * 7919 + FAMILIES.index(fam) * 131 + sum(map(ord, hub_name)))
    out = {"fam": fam, "hub": hub_name, "seed": seed, "first": False, "made": False, "problems": []}
    for i in range(TRIES):
        bpm, k = rng.randint(*SWEEP_BPM), rng.uniform(*SWEEP_K) if k_fixed is None else k_fixed
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
        z = [p[2] for p in c["tcp"]]
        if min(z) < TCP_Z[0] or max(z) > TCP_Z[1]:
            bad.append("TCP outside the operating band (%.2f..%.2f m)" % (min(z), max(z)))
        beat = 60.0 / bpm
        out.update(first=i == 0, made=True, dur=ts[-1], size=c["style"]["size"], slowed=c["style"]["slowed"], k=k,
                   still=_still_run(ts, qs, 0.25, ts[-1] - 0.5 * beat), wrist=wrist_share(qs),
                   j5=max(q[4] for q in qs) - min(q[4] for q in qs), clearance=rep["min_env_clearance_m"],
                   labels=c["labels"].get("family") == fam and "form" in c["labels"].get("params", {}), **tcp_stats(c["tcp"]))
        if fam in STRIP_FAMILIES:
            out.update(strip_stats(rig, c))
        break
    return out


def _sweep(jobs):
    """_trial over the jobs, in worker processes when it can."""
    try:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=max(1, min(30, (os.cpu_count() or 2) - 1)), initializer=_worker_init) as ex:
            return list(ex.map(_trial, jobs, chunksize=1))
    except (OSError, ImportError, RuntimeError) as e:
        print("note: no worker processes (%s); the sweep runs here, 3 seeds" % e)
        _worker_init()
        return [_trial(j) for j in jobs if j[2] < 3]


def _median(xs):
    s = sorted(xs)
    return s[len(s) // 2] if s else 0.0


def self_test():
    import json
    import random
    import time
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    t0 = time.time()
    rig = Rig()
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    env = _room(cfg)
    stage_home = [-60.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    R, tcp, d = rig.tool(stage_home)
    r = roll_of(R, d)
    q = rig.solve(tcp, d, r, stage_home)
    check("roll_of + solve give the hub pose back", q is not None and max(abs(a - b) for a, b in zip(q, stage_home)) < 1e-6, q)
    zones, hubs = _show_hubs(rig)
    check("the greet hub from its tool tip and look point", hubs.get("greet") is not None, hubs.get("greet"))
    greet_before = [62.6059, -102.2701, 107.725, -194.967, -87.1174, 4.7785]
    check("hub_pose keeps a clear hub where it was", max(abs(a - b) for a, b in zip(hubs["greet"], greet_before)) < 1e-3,
          hubs["greet"])
    synth = synthetic_hubs(rig, cfg, env)
    check("low and high hubs at the greet hub's x, y (hub_pose: clear of the floor)", len(synth) == len(SYNTHETIC),
          {n: t for n, (q, t) in synth.items()})
    for n in sorted(hubs):
        print("      hub %-10s J5 %6.1f  tcp %s" % (n, hubs[n][4], [round(x, 3) for x in rig.tool(hubs[n])[1]]))
    blocked = {n: hub_clear(q, env) for n, q in list(hubs.items()) + [(n, q) for n, (q, _) in synth.items()]}
    blocked = {n: w for n, w in blocked.items() if w}
    for n, w in sorted(blocked.items()):
        q = stand_in(rig, hubs.get(n) or synth[n][0], env)
        print("      hub %s is not clear with the tool (%s); the sweep uses it turned to J6 %s" % (
            n, w, "%.0f" % q[5] if q else "-- none clears it"))
    check("the show's hubs are clear of the room and the arm, the tool included", not blocked, blocked)
    g = cfg["hubs"]["greet"]
    q_low = hub_pose(rig, (g["tcp"][0], g["tcp"][1], 0.55), zones["audience"]["center"], g["near"])
    check("a low hub from the greet hub's seed is not elbow-down into the floor", q_low is not None and _robot_clear(q_low),
          q_low and [round(x, 1) for x in q_low])

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
    # a flowing run: through its via key without stopping, at rest where it starts and stops
    tr = _Track([0.0, 1.0, 2.0], [[0.0], [1.0], [0.0]], [0.0, 0.2, 0.0], [False, True, False])
    v_mid_dir = (tr.at(0.6)[0] - tr.at(0.5)[0])
    tr2 = _Track([0.0, 1.0, 2.0], [[0.0], [1.0], [2.0]], [0.0, 0.2, 0.0], [False, True, False])
    v2 = (tr2.at(1.0 + 1e-4)[0] - tr2.at(1.0 - 1e-4)[0]) / 2e-4
    check("a via key is passed through, the run starts and stops at rest",
          abs(tr.at(0.0)[0]) < 1e-12 and abs(tr.at(2.0)[0]) < 1e-12 and abs(tr.at(1e-3)[0]) < 1e-6 and v2 > 0.5 and v_mid_dir > 0,
          "speed through the via %.2f (straight run)" % v2)

    # the space: the zone across, the operating band up and down, the room's hard limits
    home = home_of(rig, hubs["greet"])
    sp = Space(zones, home[0], env)
    check("the greet space: its zone across, the band %.2f-%.2f m up and down (controller cap)" % (sp.bottom, sp.top),
          sp.name == "greet" and sp.bottom <= TCP_Z[0] + Z_PAD + 1e-9 and 1.5 < sp.top <= 1.6 - HARD_PAD + 1e-9, (sp.bottom, sp.top))
    far = sp.clamp((home[0][0] + 3.0, home[0][1], 3.0))
    check("the space keeps out of the operator's slow zone and under the controller's cap",
          all(math.hypot(far[0] - c[0], far[1] - c[1]) >= r - 1e-9 for c, r in sp.slow) and far[2] <= sp.top + 1e-9 and sp.slow,
          [round(x, 3) for x in far])

    # keys: every family from each hub ends at the hub at rest, in its length;
    # reach leans back first (anticipation); tilt goes past its tilt and
    # settles (overshoot); the gaze trails or leads the TCP (overlap)
    beat = 60.0 / 100
    shapes_ok, lengths = True, {}
    for name, hq in sorted(hubs.items()):
        home_h = home_of(rig, hq)
        for fam in FAMILIES:
            for seed in range(4):
                ks, prm = draw_keys(fam, home_h, zones, random.Random(seed), beat, 0.25 + 0.25 * seed, Space(zones, home_h[0], env))
                shapes_ok &= all(math.dist(kk[1], home_h[0]) < 1e-9 and abs(kk[3] - home_h[2]) < 1e-9 for kk in (ks[0], ks[-2], ks[-1]))
                s = sum(kk[0] for kk in ks)
                lengths.setdefault(fam, []).append(s <= MAX_CLIP_S + 1e-6 and (s >= prm["length_s"][0] - 1e-3 or prm["form"] != "long"))
    check("keys start and end at the hub, at rest", shapes_ok)
    check("keys are drawn to their length (a long form at least its bottom, none over %g s)" % MAX_CLIP_S,
          all(all(v) for v in lengths.values()), {f: sum(v) for f, v in lengths.items()})
    ks = keys_for("reach", home, zones, random.Random(3), beat, 0.7)
    check("reach leans back before it reaches out (anticipation)",
          U._dot(U._sub(ks[1][1], home[0]), U._sub(ks[2][1], home[0])) < 0.0)
    ks = keys_for("tilt", home, zones, random.Random(3), beat, 0.7)
    rolls = [kk[3] - home[2] for kk in ks]
    check("tilt rolls past and settles back (overshoot)", any(abs(a) > abs(b) + 1.0 and a * b > 0 for a, b in zip(rolls, rolls[1:])),
          [round(x, 1) for x in rolls])
    lags = [kk[4]["lag"] for f in FAMILIES for kk in keys_for(f, home, zones, random.Random(3), beat, 0.7)]
    check("the gaze trails and leads the TCP (overlap)", min(lags) < 0.0 < max(lags))
    wave = keys_for("wave", home, zones, random.Random(3), beat, 0.7)
    check("a smaller draw stays nearer the hub", max(math.dist(kk[1], home[0]) for kk in shrink(wave, home, 0.5)) <
          max(math.dist(kk[1], home[0]) for kk in wave))
    def pace(k_):                                   # s of moving a metre, over a family's moves
        out = []
        for f in HEAD_FAMILIES:
            ks = keys_for(f, home, zones, random.Random(5), beat, k_, length=("long", (4.0, 12.0)))
            t = sum(b[0] for a, b in zip(ks, ks[1:]) if not b[4].get("hold"))
            d = sum(math.dist(a[1], b[1]) for a, b in zip(ks, ks[1:]) if not b[4].get("hold"))
            out.append(t / max(1e-6, d))
        return _median(out)
    check("the calm move slower than the lively (%.0f vs %.0f s a metre, as drawn)" % (pace(0.2), pace(1.0)),
          pace(0.2) > 2.5 * pace(1.0))

    # the sweep: every family from greet, rest, low and high, SEEDS seeds,
    # TRIES draws each, bpm and intensity over SWEEP_BPM, SWEEP_K; and reach,
    # stretch, rise, dive, sweep at full intensity from greet
    names = [n for n in ("rest", "greet") if n in hubs] + sorted(n for n in hubs if n not in ("rest", "greet"))
    names += [n for n, _ in SYNTHETIC if n in synth]
    jobs = [(f, h, s) for h in names for f in FAMILIES for s in range(SEEDS)]
    full = ("reach", "stretch", "rise", "dive", "sweep")
    jobs += [(f, "greet", 100 + s, 1.0) for f in full for s in range(5)]
    allres = _sweep(jobs)
    res = [x for x in allres if x["seed"] < 100]
    top = [x for x in allres if x["seed"] >= 100]
    print("\n%-8s %-6s %8s %8s  %-11s %6s %6s %6s %6s  %s" % ("family", "hub", "1st draw", "in %d" % TRIES, "length",
                                                          "z m", "ext m", "peak", "mean", "size / slowed"))
    for name in names:
        for fam in FAMILIES:
            rs = [x for x in res if x["fam"] == fam and x["hub"] == name]
            made = [x for x in rs if x["made"]]
            ds = [x["dur"] for x in made]
            print("%-8s %-6s %5d/%-2d %5d/%-2d  %-11s %6.2f %6.2f %6.2f %6.2f  %s" % (
                fam, name, sum(x["first"] for x in rs), len(rs), len(made), len(rs),
                "%.1f-%.1f s" % (min(ds), max(ds)) if ds else "-",
                _median([x["zspan"] for x in made]), _median([x["extent"] for x in made]),
                _median([x["vpeak"] for x in made]), _median([x["vmean"] for x in made]),
                "%.2f / %.2f" % (sum(x["size"] for x in made) / len(made), sum(x["slowed"] for x in made) / len(made)) if made else ""))
    made = [x for x in res if x["made"]]
    per = {(x["fam"], x["hub"]) for x in made}
    print()
    refused = sorted((f, n) for f, n in STRIP_REFUSED if n in names)
    if refused:
        print("     the strip refuses: %s" % "; ".join("%s from %s (%d/%d made)" % (
            f, n, sum(x["made"] for x in res if x["fam"] == f and x["hub"] == n), SEEDS) for f, n in refused))
    want = {(f, n) for f in FAMILIES for n in names} - set(refused)
    check("every family makes a clip from every hub (but what the strip refuses)", not want - per, sorted(want - per))
    cells = {(f, n): sum(x["made"] for x in res if x["fam"] == f and x["hub"] == n) / float(SEEDS) for f, n in want}
    low_cells = {"%s/%s" % c: v for c, v in cells.items() if v < 0.8}
    check("at least 80%% of seeds make a clip within %d draws, every family from every hub (but what the strip refuses)" % TRIES,
          not low_cells, low_cells or "%d/%d overall" % (len(made), len(res)))
    probs = sorted({"%s/%s: %s" % (x["fam"], x["hub"], p) for x in made for p in x["problems"]})
    check("each starts and ends at the hub at rest, within limits and speed, clear of the room, in the band", not probs, probs[:6])
    ds = [x["dur"] for x in made]
    check("%g-%g s long, at least 20%% of them 5 s or less (accents)" % (MIN_CLIP_S, MAX_CLIP_S),
          all(MIN_CLIP_S - 1e-6 <= d <= MAX_CLIP_S + 1e-6 for d in ds) and sum(d <= 5.0 for d in ds) >= 0.2 * len(ds),
          "%.1f-%.1f s, %d%% <= 5 s" % (min(ds), max(ds), 100 * sum(d <= 5.0 for d in ds) / len(ds)))
    steps = (max(x["first_step"] for x in made), max(x["last_step"] for x in made))
    check("first and last steps under %g deg (starts and ends at rest)" % REST_STEP_DEG, max(steps) <= REST_STEP_DEG,
          "first %.4f, last %.4f deg" % steps)
    still = max(made, key=lambda x: x["still"])
    check("never frozen for 0.4 s (holds breathe)", still["still"] < 0.4,
          "longest %.2f s (%s from %s)" % (still["still"], still["fam"], still["hub"]))
    greet = [x for x in made if x["hub"] == "greet"]
    shares = {f: round(sum(x["wrist"] for x in greet if x["fam"] == f) / max(1, sum(x["fam"] == f for x in greet)), 2) for f in FAMILIES}
    check("the wrist carries most of look / tilt", all(shares[f] > 0.5 for f in ("look", "tilt")), shares)
    j5 = {f: round(max([x["j5"] for x in greet if x["fam"] == f] or [0]), 1) for f in FAMILIES}
    check("J5 moves (not a right-angle arm with a still wrist)", sum(v > 8 for v in j5.values()) >= 4, j5)
    check("labels: family and typed params", all(x["labels"] for x in made))
    # the strip families: what each did with the strip
    strip = [x for x in made if x["fam"] in STRIP_FAMILIES]
    print()
    print("%-8s %-10s %6s %8s %9s %9s %8s" % ("family", "hub", "made", "J6 deg", "low end m", "under m", "strokes"))
    for fam in STRIP_FAMILIES:
        for name in names:
            rs = [x for x in strip if x["fam"] == fam and x["hub"] == name]
            print("%-8s %-10s %3d/%-2d %8s %9s %9s %8s" % (
                fam, name, len(rs), SEEDS, "%.0f" % min(x["s_j6"] for x in rs) if rs else "-",
                "%.2f" % min(x["s_low"] for x in rs) if rs else "-", "%.2f" % min(x["s_drop"] for x in rs) if rs else "-",
                "%d" % min(x["s_strokes"] for x in rs) if rs and "s_strokes" in rs[0] else "-"))
    print()
    tw = [x for x in strip if x["fam"] == "twirl"]
    check("twirl spins the strip: J6 travels at least %g deg" % TWIRL_MIN_DEG, tw and all(x["s_j6"] >= TWIRL_MIN_DEG for x in tw),
          "least %.0f deg, median %.0f" % (min(x["s_j6"] for x in tw), _median([x["s_j6"] for x in tw])) if tw else "none made")
    wp = [x for x in strip if x["fam"] == "wipe"]
    check("wipe: at least 3 pressed strokes with the face within %g deg of the pane" % WIPE_FACE_DEG,
          wp and all(x["s_strokes"] >= 3 for x in wp),
          "least %d, median %d" % (min(x["s_strokes"] for x in wp), _median([x["s_strokes"] for x in wp])) if wp else "none made")
    sc = [x for x in strip if x["fam"] == "scoop"]
    check("scoop: the strip's lower end goes 0.3 m under the hub's tool tip and comes back",
          sc and all(x["s_drop"] >= 0.3 and x["s_back"] < 1e-3 for x in sc),
          "least %.2f m under, median %.2f" % (min(x["s_drop"] for x in sc), _median([x["s_drop"] for x in sc])) if sc else "none made")
    under = [min(x["s_hub_low"], STRIP_FLOOR_M) - x["s_low"] for x in strip]
    check("the strip families keep the strip's lower end at STRIP_FLOOR_M (or the hub's own, when lower) but for 5 cm",
          strip and max(under) <= 0.05, "lowest %s m, at most %.3f m under" % (
              {f: round(min([x["s_low"] for x in strip if x["fam"] == f] or [9.0]), 2) for f in STRIP_FAMILIES}, max(under)))
    # the gestures (the head and neck: the families before the strip) alone
    made = [x for x in made if x["fam"] in HEAD_FAMILIES]
    zs, ext = [x["zspan"] for x in made], [x["extent"] for x in made]
    check("through heights: median TCP height span >= 0.15 m", _median(zs) >= 0.15, "median %.3f m" % _median(zs))
    # the gestures alone, from every hub (the high ones have little room
    # above and little arm beyond): the paths' figures lift the library's
    # median past 0.3 m (paths.py checks theirs)
    check("big: median TCP extent >= 0.28 m (gestures alone)", _median(ext) >= 0.28, "median %.3f m" % _median(ext))
    peaks = [x["vpeak"] for x in made]
    fast, calm_ = sum(v >= 1.0 for v in peaks), sum(v < 0.3 for v in peaks)
    check("quick and calm: some peak at >= 1 m/s, some stay under 0.3 m/s", fast >= max(5, 0.02 * len(peaks)) and calm_ >= 0.05 * len(peaks),
          "%d%% >= 1 m/s, %d%% < 0.3 m/s" % (100 * fast / len(peaks), 100 * calm_ / len(peaks)))
    lo_k = [x for x in made if x["k"] <= 0.45]
    hi_k = [x for x in made if x["k"] >= 0.75]
    check("intensity scales size and speed (k >= 0.75 vs <= 0.45: extent x%.1f, mean speed x%.1f)" % (
          _median([x["extent"] for x in hi_k]) / max(1e-6, _median([x["extent"] for x in lo_k])),
          _median([x["vmean"] for x in hi_k]) / max(1e-6, _median([x["vmean"] for x in lo_k]))),
          _median([x["extent"] for x in hi_k]) >= 1.5 * _median([x["extent"] for x in lo_k])
          and _median([x["vmean"] for x in hi_k]) >= 1.6 * _median([x["vmean"] for x in lo_k]))
    for f in full:
        rs = [x for x in top if x["fam"] == f and x["made"]]
        tv, zz = _median([x["travel"] for x in rs]), _median([x["zspan"] for x in rs])
        most = 0.7 if f in ("reach", "stretch") else 1.0
        check("%s at full intensity from greet: travels 0.4-%g m, >= 0.3 m in height" % (f, most),
              len(rs) >= 4 and 0.4 <= tv <= most and zz >= 0.3, "%d/5 made, travel %.2f m, height %.2f m" % (len(rs), tv, zz))
    print("\n%.0f s" % (time.time() - t0))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
