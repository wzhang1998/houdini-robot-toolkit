"""Paths: lively spatial figures the tool draws around a show hub.

gestures.py makes the arm look and lean; choreo.py makes it dance in joint
space. Here the TCP draws a readable FIGURE in the air at human to body
scale (0.2-0.6 m, the livelier the bigger) -- a Lissajous knot, a figure
eight, a spiral, a coil, a rose, a loose loop -- the kind of shape a person
traces with a hand, mostly in upright or leaning planes so it passes through
heights, while the wrist carries a character of its own (looking at the
audience, leaning like a brush, or keeping the hub's aim).

The figure stays in its space (gestures.Space): the hub's zone across, the
show's operating band up and down (the zone is design space, not a wall),
inside the room's work zones (the controller's caps the TCP at 1.6 m), out
of the operator's slow zone, within reach of the shoulder. Its centre moves
off the hub point across, towards the audience and up or down; when it does
not fit it is nudged inwards first, made smaller last.

Every figure is closed: it starts and ends at the hub's TCP, at rest, so it
chains in the show graph like any clip. The figure blooms out of the hub
point (its amplitude ramps up from 0 while its centre drifts out to an
offset) and folds back into it. Families:

    lissajous   sin(a t + phase), sin(b t) for a:b in 1:2, 2:1, 2:3, 3:2,
                1:3, 3:4, with a small swing out of the plane
    figure8     an eight (standing or lying), bowed into a saddle
    spiral      in the plane: grows turn by turn, then shrinks, domed
                towards the audience
    helix       a coil round the vertical (or a leaning axis): climbs above
                the hub and sinks below it (0.3-0.55 m in all, as the room
                allows), the radius opening and closing
    rose        r = cos(k t): 3, 4 or 5 petals, the petals bowed
    spline      a smooth closed loop (periodic C2 cubic) through the hub
                and 3-5 random points in the hub's space

Typed parameters per clip, drawn from an rng with ranges scaled by an
intensity 0..1 (and returned in labels["params"]): extent (m), aspect,
depth (out of plane), plane (a named plane -- audience: facing the
audience; tilted: facing it and leaning back; floor: horizontal; side:
edge-on to it -- and its normal), centre offset from the hub TCP (with a
height), cycles, form (short: a 3-5 s flourish, simpler and 3/4 the size,
likelier the livelier; long: 5-11 s), tempo (steady, accel, rit), speed
(v_mps, a_mps2: calm stays calm, v ~ k^1.5), bpm / beats, and the tool
orientation mode:

    look      the tool aims at a point in the audience zone that drifts
              slowly from face to face (the swing from the hub's aim is
              capped, so a hub facing away still turns towards people)
    tangent   the tool leans along the direction of travel, trailing
              like a brush -- the lean grows with the speed
    fixed     the hub's tool frame throughout (the wrist only compensates)

In look and tangent modes the tool also rolls with the figure's rhythm.

The LED strip on the flange (1 m across the tool's aim, collision.load_model
carries it) is kept clear by a roll about the aim: the figure's frames are
probed with the strip turned 0, +-30, +-60 or 90 degrees (strip_rolls,
gestures.Probe: the room and the arm, the strip included), and the turn most
of them clear fades in (and out) sooner than the look and the roll (half
their time, or what J6 needs for the turn), strip_roll_deg in the params;
when the clip still hits the room the next is tried. (No ribbon or brush
mode -- the strip kept along or across the path: a loop's tangent turns a
full turn a loop, more than J6's range over a few loops.)

Timing: the figure is paced along the arc length of the figure at full
size (so a spiral's small turns take as long as its big ones, as a hand
draws them) with the two-thirds power law of human drawing (slower in the
tight turns, faster on the straights: v ~ curvature^-1/3), capped where a
turn would ask the TCP for more than a_mps2 (v <= sqrt(a / curvature)),
surging and easing twice over the figure, speeding up (accel) or slowing
down (rit) if drawn so; the cruise takes a whole number of beats; min-jerk
ease in and out at both ends (velocity and acceleration 0), STILL_S at the
hub before and after (the first and last 24 fps steps are nil). The look /
lean / roll fade in and out with the same kind of ramp, so the tool frame
is the hub's at both ends. Sampled at 24 fps through the closed-form IK
(nearest branch, a step over gestures.MAX_STEP_DEG is a branch flip), then
slowed where the player would need it (fairino_player.need_profile at the
plan safety: Pace.ease_off lowers the speed along the figure there, so it
stays quick on the open stretches), then uniformly if still needed -- never
clipped, never over MAX_SLOW. A wrist that runs into its singularity gets
the same figure with half its look / lean / roll, then none (WRIST_SOFTEN).

    make(rig, hub_q, family, zones, rng, bpm=90, intensity=0.6, clip_id=None,
         env=None, safety=PLAN_SAFETY, orient=None, zone=None, why=None)
        -> motion clip dict (as gestures.make) or None; labels["family"],
           labels["params"] (the typed parameters, as drawn and fitted)
    draw(family, home, zones, rng, bpm, intensity, orient, zone, space) -> params
    near_room(env, model, qs) -> (env with only what the clip can come
                                  near, bound of the rest): the same room
                                  check, several times sooner
    FAMILIES, ORIENTS, PLANES

Pure Python.

    python scripts/paths.py        self-test
"""

import bisect
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import fairino_player as P  # noqa: E402
import gestures as G  # noqa: E402
import motion_clip as M  # noqa: E402
import urdf_rig as U  # noqa: E402

FPS = G.FPS
PLAN_SAFETY = G.PLAN_SAFETY
FAMILIES = ("lissajous", "figure8", "spiral", "helix", "rose", "spline")
ORIENTS = ("look", "tangent", "fixed")
PLANES = ("audience", "tilted", "floor", "side")
DENSE = 2400                     # samples of a figure for pacing
POWER_LAW = 1.0 / 3.0            # v ~ curvature^-1/3 (Lacquaniti, Terzuolo & Viviani 1983)
MAX_SLOW = 2.5                   # slower than this: redraw rather than crawl
ZONE_PAD = 0.02
NEAR_ROOM_M = 0.3                # room objects farther than their margin + this cannot be touched
STILL_S = 2.0 / G.FPS            # still at the hub before and after the figure
LOOK_FAR = 1.4                   # m: the look point's least distance from the hub
Z = (0.0, 0.0, 1.0)


# --------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------

def _mj(u):
    u = max(0.0, min(1.0, u))
    return u * u * u * (10 - 15 * u + 6 * u * u)


def _window(s, rho):
    """1 in the middle, min-jerk ramps over the first and last rho of 0..1."""
    return _mj(s / rho) * _mj((1.0 - s) / rho)


def _lerp(a, b, u):
    return a + (b - a) * u


def _vsum(*terms):
    """Sum of (scale, vector) pairs."""
    out = [0.0, 0.0, 0.0]
    for s, v in terms:
        out[0] += s * v[0]
        out[1] += s * v[1]
        out[2] += s * v[2]
    return tuple(out)


def _swing(d0, d):
    """The smallest rotation taking unit d0 onto unit d."""
    axis = U._cross(d0, d)
    s = U._norm(axis)
    if s < 1e-12:
        return U.IDENTITY
    return U.axis_angle_matrix(U._scale(axis, 1.0 / s), math.atan2(s, U._dot(d0, d)))


def _turn_towards(d0, target, rad, soft_cap=None):
    """d0 turned towards target (unit vectors) by rad; with soft_cap, by
    fraction rad of the angle between them, that angle capped smoothly at
    soft_cap (cap * tanh(angle / cap): no kink where the cap sets in)."""
    axis = U._cross(d0, target)
    s = U._norm(axis)
    if s < 1e-12 or (soft_cap is not None and soft_cap < 1e-9):
        return d0
    if soft_cap is not None:
        rad = rad * soft_cap * math.tanh(math.atan2(s, U._dot(d0, target)) / soft_cap)
    return U._mat_vec(U.axis_angle_matrix(U._scale(axis, 1.0 / s), rad), d0)


def _pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, int(round(p * (len(s) - 1)))))] if s else 0.0


# --------------------------------------------------------------------------
# where: the zone, the plane
# --------------------------------------------------------------------------

def zone_of(zones, tcp, name=None):
    """(name, box) of the zone the figure stays in: the named one, else the
    zone that holds the hub's TCP (not the audience), else (None, None)."""
    if name:
        return name, zones.get(name)
    for k, box in zones.items():
        if k != "audience" and G.inside(box, tcp):
            return k, box
    return None, None


def audience_dir(zones, p0):
    """Horizontal unit vector from p0 towards the audience (-X without one)."""
    a = zones.get("audience")
    h = (a["center"][0] - p0[0], a["center"][1] - p0[1], 0.0) if a else (-1.0, 0.0, 0.0)
    return U._normalize(h) if U._norm(h) > 1e-6 else (-1.0, 0.0, 0.0)


def plane_axes(plane, h):
    """(normal, u, v) of a named plane (or a normal vector) given the
    horizontal audience direction h: u in-plane and horizontal, v in-plane
    and up (towards the audience for a horizontal plane), the normal
    towards the audience (up for a horizontal plane)."""
    if not isinstance(plane, str):
        n = U._normalize(plane)
    elif plane == "audience":
        n = h
    elif plane == "tilted":
        n = U._normalize(_vsum((math.cos(math.radians(35)), h), (math.sin(math.radians(35)), Z)))
    elif plane == "floor":
        n = Z
    elif plane == "side":
        n = U._normalize(U._cross(Z, h))
    else:
        raise ValueError("unknown plane %r" % plane)
    u = U._cross(Z, n) if abs(n[2]) < 0.95 else U._cross(Z, h)
    u = U._normalize(u)
    v = U._cross(n, u)
    if abs(n[2]) >= 0.95:
        if U._dot(v, h) < 0:
            v = U._scale(v, -1.0)
    elif v[2] < 0:
        v = U._scale(v, -1.0)
    return n, u, v


# --------------------------------------------------------------------------
# figures: shape at full size, amplitude envelope, centre drift, lift
# --------------------------------------------------------------------------

def _periodic_spline(pts):
    """Closed C2 cubic through pts (chord-length knots): f(s), s in 0..1."""
    n = len(pts)
    h = [max(1e-6, math.dist(pts[i], pts[(i + 1) % n])) for i in range(n)]
    total = sum(h)
    knots = [0.0]
    for x in h[:-1]:
        knots.append(knots[-1] + x)
    # second derivatives Mi: h[i-1] M[i-1] + 2 (h[i-1] + h[i]) M[i] + h[i] M[i+1] = rhs, cyclic
    A = [[0.0] * n for _ in range(n)]
    for i in range(n):
        A[i][(i - 1) % n] += h[i - 1]
        A[i][i] += 2.0 * (h[i - 1] + h[i])
        A[i][(i + 1) % n] += h[i]
    Ms = []
    for c in range(3):
        y = [p[c] for p in pts]
        rhs = [6.0 * ((y[(i + 1) % n] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]) for i in range(n)]
        Ms.append(_solve(A, rhs))

    def f(s):
        x = (s % 1.0) * total
        i = max(0, min(n - 1, bisect.bisect_right(knots, x) - 1))
        a, b, hi = pts[i], pts[(i + 1) % n], h[i]
        t = x - knots[i]
        out = []
        for c in range(3):
            m0, m1 = Ms[c][i], Ms[c][(i + 1) % n]
            out.append(m0 * (hi - t) ** 3 / (6 * hi) + m1 * t ** 3 / (6 * hi)
                       + (a[c] / hi - m0 * hi / 6) * (hi - t) + (b[c] / hi - m1 * hi / 6) * t)
        return tuple(out)
    return f


def _solve(A, b):
    """Gaussian elimination with partial pivoting (small dense systems)."""
    n = len(b)
    m = [list(row) + [x] for row, x in zip(A, b)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


class Figure:
    """A closed path from p0 back to p0 as a function of s in 0..1.

    pos(s)   = p0 + drift(s) * offset + lift(s) * normal
               + env(s) * A * (x u + aspect y v + w normal)
    ref(s)   = the same at full amplitude (env = 1): what the pacing follows
    phase(s) = the figure's angle (for the roll's rhythm)"""

    def __init__(self, family, prm, p0, axes):
        self.family, self.prm, self.p0 = family, prm, tuple(p0)
        self.n, self.u, self.v = axes
        self.scale = 1.0                      # shrinks the whole figure into its zone
        if family == "spline":
            self.curve = _periodic_spline([tuple(p0)] + [tuple(x) for x in prm["points"]])

    def _local(self, s):
        """(x, y, w, theta, env, drift, lift) at s."""
        f, k = self.family, self.prm
        cyc = k["cycles"]
        if f == "lissajous":
            a, b = k["ratio"]
            th = 2 * math.pi * cyc * s
            ph = math.radians(k["phase_deg"])
            return (math.sin(a * th + ph), math.sin(b * th), k["depth"] * math.sin(th + ph), th,
                    _window(s, k["ramp"]), _window(s, k["ramp"]), 0.0)
        if f == "figure8":
            th = 2 * math.pi * cyc * s
            x, y = math.sin(th), 0.6 * math.sin(2 * th)      # lying: an infinity sign
            if not k["lying"]:
                x, y = y, x                                  # standing: an eight
            return (x, y, k["depth"] * math.cos(th), th, _window(s, k["ramp"]), _window(s, k["ramp"]), 0.0)
        if f == "spiral":
            th = 2 * math.pi * k["turns"] * s * k["sense"]
            env = math.sin(math.pi * s) ** 2
            return (math.cos(th), math.sin(th), k["depth"], th, env, _window(s, 0.25), 0.0)
        if f == "helix":
            th = 2 * math.pi * k["turns"] * s * k["sense"]
            # up to rise_m * split above the start and the rest below it,
            # smoothly (B (1 - cos) + A sin: max U, min -D)
            up = k["rise_m"] * k.get("rise_split", 1.0)
            down = k["rise_m"] - up
            lift = 0.5 * (up - down) * (1.0 - math.cos(2 * math.pi * s)) + math.sqrt(max(0.0, up * down)) * math.sin(2 * math.pi * s)
            return (math.cos(th), math.sin(th), 0.0, th, _window(s, k["ramp"]), _window(s, 0.3), lift)
        if f == "rose":
            kk = k["k"]
            span = math.pi if kk % 2 else 2 * math.pi
            th = span * cyc * s
            r = math.cos(kk * th)
            rot = math.radians(k["rot_deg"])
            return (r * math.cos(th + rot), r * math.sin(th + rot), k["depth"] * r * r, th * kk,
                    _window(s, k["ramp"]), _window(s, k["ramp"]), 0.0)
        raise ValueError("unknown family %r" % f)

    def at(self, s, full=False):
        """(position, phase) at s; full: the figure at full amplitude."""
        if self.family == "spline":
            c = self.curve(s * self.prm["cycles"])
            # the loop shrinks towards the hub point when it must fit its zone
            return (_vsum((1.0, self.p0), (self.scale, U._sub(c, self.p0))),
                    2 * math.pi * self.prm["cycles"] * s)
        x, y, w, th, env, drift, lift = self._local(s)
        e = 1.0 if full else env
        A = self.prm["extent_m"] / 2.0 * self.scale
        k = self.prm
        pos = _vsum((1.0, self.p0), (drift * self.scale, k["offset"]), (lift * self.scale, self.n),
                    (e * A * x, self.u), (e * A * k["aspect"] * y, self.v), (e * A * w, self.n))
        return pos, th

    def fits(self, space, n=240):
        """Every point inside the space (a gestures.Space)."""
        return all(space.contains(self.at(i / float(n))[0]) for i in range(n + 1))


# --------------------------------------------------------------------------
# drawing the parameters
# --------------------------------------------------------------------------

def _smooth(xs, w):
    """Moving average over +-w samples (ends clamped)."""
    n = len(xs)
    c = [0.0]
    for x in xs:
        c.append(c[-1] + x)
    return [(c[min(n, i + w + 1)] - c[max(0, i - w)]) / (min(n, i + w + 1) - max(0, i - w)) for i in range(n)]


def _smooth_vectors(vs, w):
    """Each vector averaged with a triangular kernel over +-2w samples."""
    out = []
    n = len(vs)
    for i in range(n):
        acc, tot = [0.0, 0.0, 0.0], 0.0
        for j in range(max(0, i - 2 * w), min(n, i + 2 * w + 1)):
            k = 2 * w + 1 - abs(i - j)
            acc = [a + k * b for a, b in zip(acc, vs[j])]
            tot += k
        out.append(tuple(a / tot for a in acc))
    return out


def _pick(rng, weighted):
    r = rng.random() * sum(w for _, w in weighted)
    for x, w in weighted:
        r -= w
        if r <= 0:
            return x
    return weighted[-1][0]


# upright and leaning planes most (they carry the figure through heights),
# the floor least; a helix coils round the vertical (or a leaning axis)
PLANE_WEIGHTS = {
    "lissajous": (("audience", 4), ("tilted", 3), ("side", 2), ("floor", 1)),
    "figure8": (("audience", 4), ("tilted", 3), ("side", 2), ("floor", 1)),
    "spiral": (("audience", 4), ("tilted", 3), ("side", 1.5), ("floor", 1)),
    "helix": (("floor", 8), ("tilted", 2)),
    "rose": (("audience", 4), ("tilted", 3), ("side", 1.5), ("floor", 1)),
    "spline": (("audience", 1),),
}
COUNTS = {"lissajous": (1, 2, 3), "figure8": (1, 2, 3), "spiral": (2, 3, 4, 5, 6), "helix": (2, 3, 4, 5, 6),
          "rose": (1, 2), "spline": (1, 2)}
EXTENT_M = (0.18, 0.6)           # a figure's size, calm .. lively (+-15 %)
HELIX_RISE_M = (0.34, 0.58)      # how high a helix climbs (in z), calm .. lively
V_MPS = (0.06, 0.8)              # drawing speed, calm .. lively
A_MPS2 = (0.6, 3.0)              # the most the turns may ask of the TCP, calm .. lively


def _resize(prm, p0, s):
    """The figure's sizes scaled by s about the hub point (in place)."""
    prm["extent_m"] *= s
    prm["offset"] = U._scale(prm["offset"], s)
    if "rise_m" in prm:
        prm["rise_m"] *= s
    if "points" in prm:
        prm["points"] = [[round(p0[i] + s * (q[i] - p0[i]), 4) for i in range(3)] for q in prm["points"]]


def _eyes(zones, rng):
    """A face in the audience: in its box, 1.3-1.7 m up (clamped to the box)."""
    a = zones["audience"]
    p = G.box_point(a, rng)
    zlo, zhi = a["center"][2] - a["size"][2] / 2, a["center"][2] + a["size"][2] / 2
    return (p[0], p[1], min(max(1.3 + rng.random() * 0.4, zlo), zhi))


def draw(family, home, zones, rng, bpm, k, orient=None, zone=None, space=None):
    """The typed parameters of one clip (a dict), or None when the figure
    cannot be fitted into its space. home = (tcp, look, roll) of the hub;
    space: a gestures.Space (the zone's footprint, the operating band, the
    room's hard limits), made from zones and zone when None."""
    p0 = home[0]
    beat = 60.0 / bpm
    h = audience_dir(zones, p0)
    space = space or G.Space(zones, p0, None, zone)
    zname = space.name
    orient = orient or _pick(rng, (("look", 4.5), ("tangent", 3.5), ("fixed", 1.5)))
    plane = _pick(rng, PLANE_WEIGHTS[family])
    n, u, v = plane_axes(plane, h)
    extent = max(0.15, min(0.62, _lerp(EXTENT_M[0], EXTENT_M[1], k) * rng.uniform(0.85, 1.15)))
    short = rng.random() < 0.15 + 0.55 * k           # a short accent (the simpler figures), likelier the livelier
    if short:
        extent *= 0.75                                # a flourish: the arm draws a big figure no quicker than ~5 s
    prm = {"family": family, "orient": orient, "plane": plane, "normal": [round(x, 4) for x in n],
           "zone": zname, "extent_m": extent, "aspect": rng.uniform(0.65, 1.0),
           "depth": rng.uniform(0.0, 0.25 + 0.3 * k), "bpm": bpm, "intensity": round(k, 3),
           "ramp": rng.uniform(0.18, 0.3)}
    # the centre drifts off the hub point: across the plane, in towards the
    # audience, and up or down (within the band, the figure's own height kept)
    ou, ov, oh = rng.uniform(-0.3, 0.3) * extent, rng.uniform(-0.2, 0.3) * extent, rng.uniform(0.0, 0.05 + 0.1 * k)
    half_z = extent / 2.0 * (abs(u[2]) + abs(v[2]) + 0.5 * abs(n[2]))
    oz = rng.uniform(-1.0, 1.0) * _lerp(0.05, 0.3, k)
    lo_z, hi_z = space.bottom + half_z - p0[2], space.top - half_z - p0[2]
    oz = max(lo_z, min(hi_z, oz)) if lo_z <= hi_z else 0.5 * (lo_z + hi_z)
    prm["offset"] = _vsum((ou, u), (ov, v), (oh, h), (oz, Z))
    if family == "lissajous":
        prm["ratio"] = list(rng.choice(((1, 2), (2, 1)) if short else ((1, 2), (2, 1), (2, 3), (3, 2), (1, 3), (3, 4))))
        prm["phase_deg"] = rng.choice((30.0, 45.0, 60.0, 90.0)) * rng.choice((1, -1))
    elif family == "figure8":
        prm["lying"] = rng.random() < 0.5
        prm["depth"] = rng.uniform(0.2, 0.35 + 0.35 * k)
    elif family == "spiral":
        prm["sense"] = rng.choice((1, -1))
        prm["depth"] = rng.uniform(0.1, 0.3 + 0.4 * k)
        prm["aspect"] = rng.uniform(0.8, 1.0)
    elif family == "helix":
        # a real climb: HELIX_RISE_M in z (along a leaning axis, as much in
        # z), up where the space has room above, else down; the climb is its
        # height offset
        prm["sense"] = rng.choice((1, -1))
        rise = _lerp(HELIX_RISE_M[0], HELIX_RISE_M[1], k) * rng.uniform(1.0, 1.1)
        prm["offset"] = _vsum((1.0, prm["offset"]), (-prm["offset"][2], Z))
        # as much above the hub as below where there is room (a coil sunk
        # far below a high hub folds the arm onto its wrist)
        up_room = max(0.0, space.top - p0[2] - extent * 0.1)
        down_room = max(0.0, p0[2] - space.bottom - extent * 0.1)
        up = min(up_room, rise / 2.0 + max(0.0, rise / 2.0 - down_room))
        down = min(down_room, rise - up)
        prm["rise_m"] = max(0.05, up + down) / max(0.3, abs(n[2]))
        prm["rise_split"] = round(up / max(1e-6, up + down), 4)
        prm["aspect"] = rng.uniform(0.85, 1.0)
        prm["ramp"] = rng.uniform(0.12, 0.2)
    elif family == "rose":
        petals = rng.choice((3, 4) if short else (3, 4, 5))
        prm["petals"] = petals
        prm["k"] = petals if petals % 2 else petals // 2
        prm["rot_deg"] = rng.uniform(0, 360)
        prm["ramp"] = rng.uniform(0.12, 0.22)
        prm["aspect"] = rng.uniform(0.85, 1.0)
    elif family == "spline":
        centre = _vsum((1.0, p0), (1.0, prm["offset"]))
        pts = []
        for _ in range(rng.choice((3, 4, 5))):
            for _try in range(40):
                q = _vsum((1.0, centre), (rng.uniform(-1, 1) * extent * 0.6, u),
                          (rng.uniform(-1, 1) * extent * 0.6, v), (rng.uniform(-0.3, 0.6) * extent * 0.5, h))
                last = pts[-1] if pts else p0
                if space.contains(q, 0.03) and math.dist(q, last) > 0.35 * extent:
                    pts.append(q)
                    break
        if len(pts) < 3:
            return None
        prm["points"] = [[round(c, 4) for c in q] for q in pts]
        for key in ("aspect", "depth", "ramp"):                 # a loop through points has none of these
            del prm[key]
    else:
        raise ValueError("unknown family %r" % family)
    # the orientation
    if orient == "look":
        # faces, seen from the hub: at least LOOK_FAR away along the same
        # line (a near face would make the wrist whip as the TCP moves)
        faces = [_eyes(zones, rng) for _ in range(rng.choice((2, 3)))] if zones.get("audience") else [home[1]]
        prm["look_points"] = [[round(c, 3) for c in _vsum((1.0, p0), (max(LOOK_FAR, math.dist(f, p0)),
                                                                           U._normalize(U._sub(f, p0))))]
                              for f in faces]
        prm["max_look_deg"] = _lerp(25.0, 45.0, k) * rng.uniform(0.85, 1.1)
    if orient == "tangent":
        prm["lean_deg"] = _lerp(12.0, 28.0, k) * rng.uniform(0.85, 1.15)
        prm["lean_lag_s"] = round(rng.uniform(0.25, 0.45), 3)
    prm["roll_deg"] = 0.0 if orient == "fixed" else _lerp(8.0, 22.0, k) * rng.uniform(0.7, 1.2) * rng.choice((1, -1))
    prm["roll_phase_deg"] = rng.uniform(0, 360)
    prm["roll_cycles"] = rng.choice((1.0, 1.5, 2.0))
    # the timing: a drawing speed from calm to lively, slower in the turns
    # (power law, and a cap on the TCP's acceleration), steady or speeding up
    # / slowing down over the figure, a whole number of beats; as many
    # cycles as come closest to a clip length: a short accent (3-5 s,
    # likelier the livelier) or a long figure (7-11 s calm .. 5-8.5 s lively)
    prm["v_mps"] = round(_lerp(V_MPS[0], V_MPS[1], k ** 1.5) * rng.uniform(0.85, 1.15), 4)   # calm stays calm longer
    prm["a_mps2"] = round(_lerp(A_MPS2[0], A_MPS2[1], k), 4)
    prm["power_law"] = round(rng.uniform(0.6, 1.0), 3)
    prm["tempo"] = _pick(rng, (("steady", 1.2), ("accel", 0.5 + 0.8 * k), ("rit", 0.8)))
    prm["ease_s"] = round(min(1.6, max(_lerp(0.6, 0.35, k), beat * rng.choice((1.0, 1.5, 2.0)))), 3)
    if short:
        prm["form"], target_s = "short", rng.uniform(2.4, 3.4)        # as drawn: the joints add a third or so
        prm["v_mps"] = round(max(prm["v_mps"] * 1.25, 0.35 + 0.45 * k), 4)   # an accent: quick, eased in and out sooner
        prm["a_mps2"] = round(prm["a_mps2"] * 1.2, 4)
        prm["ease_s"] = round(min(prm["ease_s"], max(0.35, beat)), 3)
    else:
        prm["form"], target_s = "long", rng.uniform(_lerp(7.0, 5.0, k), _lerp(11.0, 8.5, k))
    while True:
        best = None
        for c in COUNTS[family]:
            if family in ("spiral", "helix"):
                prm["turns"], prm["cycles"] = c, 1
            else:
                prm["cycles"] = c
            dur = Pace(Figure(family, prm, p0, (n, u, v)), prm, 500).duration
            off = abs(dur - target_s) + (100.0 if dur < G.MIN_CLIP_S + 0.1 else 0.0)   # never under a clip's least
            if best is None or off < best[2]:
                best = (dur, c, off)
        if best[0] < G.MAX_CLIP_S - 1.5 or prm["extent_m"] < 0.2:
            break
        _resize(prm, p0, 0.88)                         # too long even once round: a smaller figure
    if family in ("spiral", "helix"):
        prm["turns"], prm["cycles"] = best[1], 1
    else:
        prm["cycles"] = best[1]
    # into the space: the centre nudged inwards first (a hub at the edge of
    # its space, or of the arm's reach), then the figure smaller
    fig = Figure(family, prm, p0, (n, u, v))
    inward = U._normalize(_vsum((1.0, space.box["center"]), (0.5, G.SHOULDER), (-1.5, p0)))
    nudges = 0 if family == "spline" else 5
    while not fig.fits(space):
        if nudges:
            nudges -= 1
            prm["offset"] = _vsum((1.0, prm["offset"]), (0.04, inward))
            continue
        fig.scale *= 0.85
        if prm["extent_m"] * fig.scale < 0.12:
            return None
    if fig.scale < 1.0:
        _resize(prm, p0, fig.scale)
        fig = Figure(family, prm, p0, (n, u, v))
        if not fig.fits(space):
            return None
    prm["offset"] = [round(x, 4) for x in prm["offset"]]
    for key in ("extent_m", "aspect", "depth", "ramp", "max_look_deg", "lean_deg", "roll_deg", "roll_phase_deg",
                "rise_m", "rot_deg"):
        if key in prm:
            prm[key] = round(prm[key], 4)
    return prm


# --------------------------------------------------------------------------
# timing: pace along the figure, ease in and out
# --------------------------------------------------------------------------

class Pace:
    """s (figure parameter) as a function of time t in 0..duration.

    Along the figure at full size: speed v_mps shaped by the two-thirds
    power law (normalised to its mean along the arc), capped where the
    turn would ask the TCP for more than a_mps2 (v <= sqrt(a / curvature));
    the cruise rounded to whole beats; ease_s of min-jerk speed ramps at
    both ends. ease_off() then slows it only where the joints need it."""

    def __init__(self, fig, prm, n=DENSE):
        ref = [fig.at(i / float(n), True)[0] for i in range(n + 1)]
        ds = [max(1e-9, math.dist(ref[i], ref[i + 1])) for i in range(n)]
        kap = []
        for i in range(n + 1):
            a, b, c = ref[max(0, i - 1)], ref[i], ref[min(n, i + 1)]
            h = max(1e-9, 0.5 * math.dist(a, c))
            kap.append(U._norm(_vsum((1.0, a), (-2.0, b), (1.0, c))) / (h * h))
        w = max(2, n // 200)
        kap = _smooth(kap, w)
        k0 = 1.0 / max(0.05, prm.get("extent_m", 0.3))
        beta = POWER_LAW * prm.get("power_law", 1.0)
        f = [(x + k0) ** -beta for x in kap]
        mean = sum(0.5 * (f[i] + f[i + 1]) * ds[i] for i in range(n)) / sum(ds)
        v = [prm["v_mps"] * max(0.55, min(1.6, x / mean)) for x in f]
        surge = 0.25 if fig.family == "helix" else 0.15   # it surges and eases twice (a coil's curve is even)
        v = [x * (1.0 + surge * math.cos(4.0 * math.pi * i / float(n))) for i, x in enumerate(v)]
        tempo = prm.get("tempo", "steady")
        if tempo in ("accel", "rit"):                  # the end twice (half) the start's speed
            g = math.log(2.0) * (1.0 if tempo == "accel" else -1.0)
            v = [x * math.exp(g * (i / float(n) - 0.5)) for i, x in enumerate(v)]
        v = [max(0.02, min(x, math.sqrt(prm["a_mps2"] / max(1e-6, c)))) for x, c in zip(v, kap)]
        self.v, self.ds, self.w = _smooth(v, w), ds, w
        self.beat, self.ease = 60.0 / prm["bpm"], prm["ease_s"]
        self.s = [i / float(n) for i in range(n + 1)]
        self._build()

    def _build(self):
        tau = [0.0]
        for i in range(len(self.ds)):
            tau.append(tau[-1] + self.ds[i] / (0.5 * (self.v[i] + self.v[i + 1])))
        self.beats = max(2, int(math.ceil(tau[-1] / self.beat - 0.2)))
        cruise = self.beats * self.beat
        self.tau = [x * cruise / tau[-1] for x in tau]
        self.cruise = cruise
        self.duration = cruise + self.ease

    def ease_off(self, prof, slow=1.0, margin=1.05, spread_s=0.3):
        """Slower where the joints need it: prof = [(t, need)] of the clip as
        played (`slow` times the design); the speed along the figure divided
        by the need there (and margin), spread over spread_s either side and
        smoothed, so the figure keeps its pace elsewhere -- quick on the
        open stretches, eased in the tight ones."""
        n = len(self.s)
        f = [1.0] * n
        for t, nd in prof:
            if nd <= 1.0:
                continue
            i = max(0, min(n - 1, bisect.bisect_right(self.tau, self.tau_of((t - STILL_S) / slow)) - 1))
            f[i] = max(f[i], nd * margin)
        w = max(self.w, int(spread_s * n / max(1e-6, self.cruise)))
        grown = [max(f[max(0, i - w):i + w + 1]) for i in range(n)]
        smooth = _smooth(grown, w)
        self.v = [v / max(1.0, a, b) for v, a, b in zip(self.v, grown, smooth)]
        self.v = _smooth(self.v, self.w)
        self._build()

    def tau_of(self, t):
        """Cruise time reached at t: min-jerk speed ramps of `ease` s at both ends."""
        Ta, T = self.ease, self.duration
        if t <= 0:
            return 0.0
        if t >= T:
            return self.cruise
        if t < Ta:
            u = t / Ta
            return Ta * (2.5 * u ** 4 - 3 * u ** 5 + u ** 6)
        if t > T - Ta:
            return self.cruise - self.tau_of(T - t)
        return Ta / 2.0 + (t - Ta)

    def s_of(self, t):
        x = self.tau_of(t)
        i = max(0, min(len(self.tau) - 2, bisect.bisect_right(self.tau, x) - 1))
        a, b = self.tau[i], self.tau[i + 1]
        return self.s[i] + (self.s[i + 1] - self.s[i]) * ((x - a) / (b - a) if b > a else 0.0)


# --------------------------------------------------------------------------
# frames: position and tool frame per 24 fps frame
# --------------------------------------------------------------------------

def _look_at(prm, u):
    """The drifting look point at normalised time u: min-jerk between faces."""
    pts = prm["look_points"]
    if len(pts) == 1:
        return tuple(pts[0])
    x = u * (len(pts) - 1)
    i = min(len(pts) - 2, int(x))
    return tuple(_lerp(a, b, _mj((x - i) * 1.25 - 0.125)) for a, b in zip(pts[i], pts[i + 1]))


def frames(rig, hub_q, fig, prm, pace, slow=1.0):
    """[(t, tcp, R)] at 24 fps for the clip played `slow` times slower,
    STILL_S at the hub before and after (the first and last steps are nil,
    as a gesture's)."""
    R0, p0, d0 = rig.tool(hub_q)
    T = pace.duration
    n = int(math.ceil((T * slow + 2.0 * STILL_S) * FPS))
    ts = [i / FPS for i in range(n + 1)]
    td = [min(T, max(0.0, t - STILL_S) / slow) for t in ts]   # design time of each frame
    orient = prm["orient"]
    tw = max(pace.ease, 0.2 * T)                          # the look / roll fade in and out
    r_s = abs(prm.get("strip_roll_deg", 0.0))            # the strip's turn: sooner, in the time J6 needs for it
    tws = min(T / 3.0, max(0.5 * tw, 1.2 * max(0.0123 * r_s, math.sqrt(0.0113 * r_s))))

    def pos(t):
        return fig.at(pace.s_of(min(T, max(0.0, t))))

    samples = [pos(t) for t in td]
    if orient == "tangent":
        # the lean follows the velocity across the tool, smoothed over a
        # fraction of a second (a brush's handle lags its tip, and a sharp
        # turn passes the lean through upright instead of flipping it)
        g = 1.0 / (2 * FPS)
        m = int(math.ceil(T / g))
        grid = [pos(i * g)[0] for i in range(m + 1)]
        vel = [U._scale(U._sub(grid[min(m, i + 1)], grid[max(0, i - 1)]), 1.0 / ((min(m, i + 1) - max(0, i - 1)) * g))
               for i in range(m + 1)]
        vel = [_vsum((1.0, x), (-U._dot(x, d0), d0)) for x in vel]
        vel = _smooth_vectors(vel, max(1, int(prm["lean_lag_s"] / g)))
        vref = max(1e-6, _pct([U._norm(x) for x in vel], 0.85))
    out = []
    for i, (t, (p, th)) in enumerate(zip(ts, samples)):
        w = _mj(td[i] / tw) * _mj((T - td[i]) / tw)
        ws = _mj(td[i] / tws) * _mj((T - td[i]) / tws)       # the strip turns clear sooner, back later
        d = d0
        if orient == "look":
            aim = U._normalize(U._sub(_look_at(prm, td[i] / T), p))
            d = _turn_towards(d0, aim, w, math.radians(prm["max_look_deg"]))
        elif orient == "tangent":
            x = td[i] / g
            j = min(m - 1, int(x))
            vp = _vsum((1 - (x - j), vel[j]), (x - j, vel[j + 1]))
            if U._norm(vp) > 1e-9:
                back = U._scale(U._normalize(vp), -1.0)            # the tip trails, like a brush
                d = _turn_towards(d0, back, math.radians(prm["lean_deg"]) * w * math.tanh(1.2 * U._norm(vp) / vref))
        R = U._mat_mul(_swing(d0, d), R0)
        roll = prm["roll_deg"] * w * math.sin(2 * math.pi * prm["roll_cycles"] * td[i] / T
                                              + math.radians(prm["roll_phase_deg"]))
        roll += prm.get("strip_roll_deg", 0.0) * ws          # the strip turned clear (strip_rolls)
        if roll:
            R = U._mat_mul(U.axis_angle_matrix(d, math.radians(roll)), R)
        out.append((t, p, R))
    return out


def solve_frames(rig, hub_q, frs):
    """Joints per frame through the IK, nearest branch, from hub_q exactly;
    (qs, None) or (None, why)."""
    qs, prev = [list(hub_q)], list(hub_q)
    for i, (t, p, R) in enumerate(frs[1:], 1):
        q = rig.solve(p, None, None, prev, R=R)
        if q is None:
            return None, "unreachable at %.2f s" % t
        step = max(abs(a - b) for a, b in zip(q, prev))
        if step > G.MAX_STEP_DEG:
            return None, "branch flip at %.2f s (%.0f deg in a frame)" % (t, step)
        if abs(math.sin(math.radians(q[4]))) < 0.2:
            return None, "wrist singularity at %.2f s" % t
        qs.append(q)
        prev = q
    if max(abs(a - b) for a, b in zip(qs[-1], hub_q)) > 0.5:
        return None, "does not come back to the hub"
    qs[-1] = list(hub_q)
    return qs, None


# --------------------------------------------------------------------------
# the strip on the flange: turned clear of the room and the arm
# --------------------------------------------------------------------------

STRIP_ROLLS = (0.0, 30.0, -30.0, 60.0, -60.0, 90.0)   # the strip's turns to try (it is symmetric: +-90 reach every way)
STRIP_EVERY = 4                  # frames: every this many checked when choosing
STRIP_TRIES = 3                  # rolls tried in turn when the clip hits the room


def strip_rolls(rig, hub_q, fig, prm, pace, env=None):
    """The rolls (degrees about the aim, faded in and out a little sooner
    than the look and the roll) that keep the strip on the flange clear along the figure,
    likeliest first: STRIP_ROLLS ordered by how many of the figure's frames
    (every STRIP_EVERY-th, as designed) the tool can take with it -- clear
    of the room and of the arm, the strip included (gestures.Probe) -- the
    least turn first among equals. The tool frame at the hub is the hub's:
    the strip turns while the figure blooms and back while it folds."""
    probe = G.Probe(rig, hub_q, env)
    score = []
    for r in STRIP_ROLLS:
        prm["strip_roll_deg"] = r
        q, good, n = list(hub_q), 0, 0
        for t, p, R in frames(rig, hub_q, fig, prm, pace, 1.0)[::STRIP_EVERY]:
            n += 1
            qn = probe.pose(p, R, near=q)
            if qn is not None:
                good, q = good + 1, qn
        score.append((-good, abs(r), r))
        if good == n and r == 0.0:                   # the figure as drawn is clear: the others in turn after it
            prm["strip_roll_deg"] = 0.0
            return list(STRIP_ROLLS)
    prm["strip_roll_deg"] = 0.0
    return [r for _, _, r in sorted(score)]


# --------------------------------------------------------------------------
# the room: only what the clip can come near
# --------------------------------------------------------------------------

def near_room(env, model, qs, reach=NEAR_ROOM_M):
    """env without the obstacles no link can come within its margin + reach
    of (a lower bound per link from the box around its capsule over the
    whole clip: an sdf changes no faster than the distance). The check is
    then the same, only cheaper. Also the smallest bound of what was left out."""
    import collision as CL
    lo = hi = None
    for q in qs:
        caps, _ = CL.capsules(model, q)
        if lo is None:
            lo = [[min(a[c], b[c]) - r for c in range(3)] for _, a, b, r in caps]
            hi = [[max(a[c], b[c]) + r for c in range(3)] for _, a, b, r in caps]
            names = [c[0] for c in caps]
            continue
        for k, (_, a, b, r) in enumerate(caps):
            for c in range(3):
                lo[k][c] = min(lo[k][c], a[c] - r, b[c] - r)
                hi[k][c] = max(hi[k][c], a[c] + r, b[c] + r)
    margin = float(env.get("margin_m", 0.05))
    keep, left_out = [], math.inf
    for o in env.get("objects", []):
        if o["role"] not in ("obstacle", "keep_out"):
            keep.append(o)
            continue
        bound = math.inf
        for k, name in enumerate(names):
            if name in CL.FIXED_LINKS:
                continue
            centre = [(lo[k][c] + hi[k][c]) / 2 for c in range(3)]
            half = math.dist(lo[k], hi[k]) / 2
            bound = min(bound, CL.sdf(o, centre) - half)
        if bound < o.get("margin_m", margin) + reach:
            keep.append(o)
        else:
            left_out = min(left_out, bound)
    return dict(env, objects=keep), left_out


# --------------------------------------------------------------------------
# a clip
# --------------------------------------------------------------------------

WRIST_SOFTEN = (1.0, 0.5, 0.0)   # the orientation's swing, as drawn, then quieter when the wrist cannot follow


def _soften(prm, f):
    """The orientation's swing (look, lean, roll) scaled to f of as drawn (in place)."""
    base = prm.setdefault("_drawn", {x: prm[x] for x in ("max_look_deg", "lean_deg", "roll_deg") if x in prm})
    for x, val in base.items():
        prm[x] = round(val * f, 4)
    prm["wrist_soften"] = f


def _timed(rig, hub_q, fig, prm, pace, designed, vel, acc):
    """(ts, qs, slow) of the figure through the IK, slowed where the joints
    need it (pace.ease_off), then, if still needed, all of it; or (None, why)."""
    slow = 1.0
    for it in range(10):
        frs = frames(rig, hub_q, fig, prm, pace, slow)
        qs, reason = solve_frames(rig, hub_q, frs)
        if qs is None:
            return None, reason
        ts = [f[0] for f in frs]
        s = P.limiting(ts, qs, 125.0, vel, acc)["scale_needed"]
        if s <= 1.0:
            return ts, qs, slow
        if it < 6:                                     # slower where the joints need it
            pace.ease_off(P.need_profile(ts, qs, 125.0, vel, acc), slow)
        else:                                          # then, if still needed, all of it
            slow *= s * 1.03
        if pace.duration * slow > MAX_SLOW * designed:
            return None, "would play %.1fx slower than designed" % (pace.duration * slow / designed)
    return None, "timing did not settle"


def make(rig, hub_q, family, zones, rng, bpm=90, intensity=0.6, clip_id=None, env=None, safety=PLAN_SAFETY,
         orient=None, zone=None, why=None):
    """A path clip from hub_q (gestures.make's shape; labels["family"] and
    labels["params"] added), or None when this draw does not work (does not
    fit its zone, unreachable, branch flip, would crawl, hits the room).
    orient: look / tangent / fixed, or None to draw one; zone: the zone's
    name (default: the one holding the hub's TCP); why: a list the reason
    for a None is appended to."""
    import collision as CL
    import motion_labels

    def fail(reason):
        if why is not None:
            why.append("%s: %s" % (family, reason))
        return None
    if family not in FAMILIES:
        raise ValueError("unknown family %r" % family)
    home = G.home_of(rig, hub_q)
    space = G.Space(zones, home[0], env, zone)
    prm = draw(family, home, zones, rng, bpm, intensity, orient, zone, space)
    if prm is None:
        return fail("does not fit its space")
    fig = Figure(family, prm, home[0], plane_axes(prm["plane"], audience_dir(zones, home[0])))
    pace = Pace(fig, prm)
    vel, acc = [v * safety for v in rig.vel], [a * safety for a in rig.acc]
    model = CL.load_model("fr20") if env is not None else None
    reason = None
    # the strip turned clear (strip_rolls), the likeliest first; the next
    # when the clip still hits the room (the checks see every frame)
    for roll in strip_rolls(rig, hub_q, fig, prm, pace, env)[:STRIP_TRIES]:
        prm["strip_roll_deg"] = roll
        if "_drawn" in prm:
            _soften(prm, 1.0)
        pace = Pace(fig, prm)
        designed = pace.duration
        # a wrist that runs into its singularity (or flips) gets a quieter
        # orientation for the same figure: half the look / lean / roll, then none
        for soft in WRIST_SOFTEN:
            if soft < 1.0:
                _soften(prm, soft)
                pace = Pace(fig, prm)
                designed = pace.duration
            got = _timed(rig, hub_q, fig, prm, pace, designed, vel, acc)
            if got[0] is not None or not any(w in got[1] for w in ("wrist", "branch flip", "unreachable")):
                break
        if got[0] is None:
            reason = got[1]
            continue
        ts, qs, slow = got
        prm["beats"], prm["cruise_s"] = pace.beats, round(pace.cruise, 4)
        slow_all, slow = slow, ts[-1] / designed
        if not G.MIN_CLIP_S <= ts[-1] <= G.MAX_CLIP_S + 1e-6:
            return fail("%.1f s, outside %g-%g s" % (ts[-1], G.MIN_CLIP_S, G.MAX_CLIP_S))
        clip = {"schema": M.SCHEMA, "id": clip_id or "path_%s" % family, "robot": "fr20",
                "joint_names": ["j%d" % i for i in range(1, 7)], "units": {"angle": "deg", "time": "s", "length": "m"},
                "points": [{"t": round(t, 6), "q": [round(x, 5) for x in q]} for t, q in zip(ts, qs)],
                "tcp": M._tcp_path("fr20", qs),
                "style": {"generator": "paths.py", "primitive": family, "orient": prm["orient"], "bpm": bpm,
                          "intensity": intensity, "slowed": round(slow, 3)},
                "meta": {"duration_s": round(ts[-1], 6), "tags": ["path", family, prm["orient"]],
                         "source": {"generator": "paths.py"}}}
        clip["points"][0]["q"], clip["points"][-1]["q"] = list(hub_q), list(hub_q)
        xs = list(zip(*clip["tcp"]))
        clip["meta"]["bounds"] = {"min": [min(a) for a in xs], "max": [max(a) for a in xs]}
        M.measure(clip, acc=rig.acc)
        if env is None:
            break
        near, left_out = near_room(env, model, qs)
        rep = CL.check(model, near, ts, qs)
        clip["safety"]["collision"] = CL.describe(rep)
        clear = rep["min_env_clearance_m"]
        clip["safety"]["min_clearance_m"] = clear if clear is not None else round(left_out, 4)
        clip["safety"]["min_self_clearance_m"] = rep["min_self_clearance_m"]
        if rep["ok"]:
            break
        reason = "hits the room: %s" % CL.describe(rep)[:80]
    else:
        return fail(reason)
    clip["labels"] = motion_labels.label(clip)
    clip["labels"]["family"] = family
    prm["slowed"], prm["slowed_all"] = round(slow, 3), round(slow_all, 3)
    prm.pop("_drawn", None)
    clip["labels"]["params"] = prm
    return clip


def wrist_share(qs):
    return G.wrist_share(qs)


# --------------------------------------------------------------------------
# self-test
# --------------------------------------------------------------------------

SEEDS = 8                        # draws per family and hub in the sweep
TRIES = 3                        # a seed's draws before it counts as not made
MATRIX_TRIES = 8                 # draws for one family x mode from one hub
_W = {}


def _worker_init():
    import json
    import collision as CL
    import show
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    rig = G.Rig()
    hubs = show.resolve_hubs(cfg, rig)
    env = show.show_env(CL.load_env(os.path.join(ROOT, cfg["env"])), cfg, cfg["margins"]["idle_canvas_m"])
    for name, (q, _) in G.synthetic_hubs(rig, cfg, env).items():
        hubs[name] = q
    for name in list(hubs):                              # a hub not clear itself: its stand-in (gestures.stand_in)
        hubs[name] = G.stand_in(rig, hubs[name], env) or hubs[name]
    _W.update(cfg=cfg, rig=rig, hubs=hubs, env=env, zones=cfg["zones"], model=CL.load_model("fr20"),
              safety=(cfg.get("range") or {}).get("speed", PLAN_SAFETY))


def _summary(c, hq, room_full=False):
    """What the checks need of a clip (not the clip)."""
    import collision as CL
    import show
    rig, cfg = _W["rig"], _W["cfg"]
    ts, qs = [p["t"] for p in c["points"]], [p["q"] for p in c["points"]]
    vel, acc = [v * _W["safety"] for v in rig.vel], [a * _W["safety"] for a in rig.acc]
    d = c["labels"]["descriptors"]
    out = {"end": max(max(abs(a - b) for a, b in zip(qs[0], hq)), max(abs(a - b) for a, b in zip(qs[-1], hq))),
           "rest_v": max(max(abs(a - b) for a, b in zip(qs[1], qs[0])), max(abs(a - b) for a, b in zip(qs[-1], qs[-2]))) * FPS,
           "step": max(max(abs(a - b) for a, b in zip(x, y)) for x, y in zip(qs, qs[1:])),
           "need": P.limiting(ts, qs, 125.0, vel, acc)["scale_needed"], "clear": c["safety"]["min_clearance_m"],
           "dur": c["meta"]["duration_s"], "in_range": show.out_of_range(cfg, qs, c["tcp"]) is None,
           "safe": bool(c["safety"]["ok"]), "wrist": wrist_share(qs), "ratio": d["tcp_peak_mps"] / max(1e-6, d["tcp_mean_mps"]),
           "labels": c["labels"]["family"] and "measured" in c["labels"] and "form" in c["labels"]["params"],
           "orient": c["labels"]["params"]["orient"], "plane": c["labels"]["params"]["plane"],
           "form": c["labels"]["params"]["form"], "slowed": c["labels"]["params"]["slowed"],
           "action": c["labels"]["measured"]["action"]}
    out.update(G.tcp_stats(c["tcp"]))
    if room_full:                                   # the near-room check against the whole room
        rep = CL.check(_W["model"], _W["env"], ts, qs)
        a, b = rep["min_env_clearance_m"], c["safety"]["min_clearance_m"]
        far = _W["env"].get("margin_m", 0.05) + NEAR_ROOM_M
        out["room_same"] = rep["ok"] and (abs(a - b) < 1e-6 or (a >= far and b >= far))
    return out


def _job(job):
    """One draw set: (kind, hub, family, orient, seed, k). 'matrix': up to
    MATRIX_TRIES draws in one orientation mode; 'sweep': up to TRIES draws,
    the mode drawn. bpm and intensity from SWEEP_BPM, SWEEP_K (or k)."""
    import random
    kind, hub, fam, orient, seed, k_fixed = job
    hq = _W["hubs"][hub]
    rng = random.Random(seed * 7919 + FAMILIES.index(fam) * 131 + sum(map(ord, hub)) + (ORIENTS.index(orient) if orient else 7))
    whys = []
    tries = MATRIX_TRIES if kind == "matrix" else TRIES
    for i in range(tries):
        k = rng.uniform(*G.SWEEP_K) if k_fixed is None else k_fixed
        c = make(_W["rig"], hq, fam, _W["zones"], rng, bpm=rng.randint(*G.SWEEP_BPM), intensity=k, env=_W["env"],
                 safety=_W["safety"], orient=orient, why=whys)
        if c is not None:
            out = {"kind": kind, "hub": hub, "fam": fam, "mode": orient, "seed": seed, "k": k, "tries": i + 1, "made": True,
                   "whys": whys}
            out.update(_summary(c, hq, room_full=seed % 7 == 0))
            return out
    return {"kind": kind, "hub": hub, "fam": fam, "mode": orient, "seed": seed, "tries": tries, "made": False, "whys": whys}


def _run(jobs):
    try:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=max(1, min(30, (os.cpu_count() or 2) - 1)), initializer=_worker_init) as ex:
            return list(ex.map(_job, jobs, chunksize=1))
    except (OSError, ImportError, RuntimeError) as e:
        print("note: no worker processes (%s); run here, 3 seeds" % e)
        _worker_init()
        return [_job(j) for j in jobs if j[4] < 3]


def self_test():
    import json
    import time
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    t_start = time.time()
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    zones = cfg["zones"]

    # the pieces
    f = _periodic_spline([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0.5)])
    check("the closed spline passes its points and closes", math.dist(f(0.0), (0, 0, 0)) < 1e-9
          and math.dist(f(1.0), (0, 0, 0)) < 1e-9 and min(math.dist(f(i / 400.0), (1, 1, 0)) for i in range(401)) < 0.01)
    n, u, v = plane_axes("audience", (0.0, -1.0, 0.0))
    check("the audience plane faces the audience, v up", math.dist(n, (0, -1, 0)) < 1e-9 and v[2] > 0.99, (n, u, v))
    import random
    rig = G.Rig()
    home = G.home_of(rig, G._show_hubs(rig)[1]["greet"])
    sp = G.Space(zones, home[0])
    prm = draw("helix", home, zones, random.Random(4), 100, 0.2, "fixed", space=sp)
    if prm is None:
        return check("a calm helix is drawn from greet", False) or 1
    fig = Figure("helix", prm, home[0], plane_axes(prm["plane"], audience_dir(zones, home[0])))
    zs = [fig.at(i / 200.0)[0][2] for i in range(201)]
    check("a calm helix still climbs >= 0.3 m (in its figure)", max(zs) - min(zs) >= 0.3 - 1e-6, "%.2f m" % (max(zs) - min(zs)))
    fast = dict(prm, tempo="accel")
    pa, pf = Pace(fig, dict(prm, tempo="steady")), Pace(fig, fast)
    check("an accelerando figure ends quicker than it starts", pf.v[-10] > 1.4 * pf.v[10] and abs(pa.v[-10] / pa.v[10] - 1.0) < 0.5,
          "%.2f -> %.2f m/s" % (pf.v[10], pf.v[-10]))

    # the hubs themselves, with the tool (a hub that is not clear is swept
    # turned about its tool axis: gestures.stand_in)
    import collision as CL
    import show
    env = show.show_env(CL.load_env(os.path.join(ROOT, cfg["env"])), cfg, cfg["margins"]["idle_canvas_m"])
    blocked = {n: G.hub_clear(q, env) for n, q in show.resolve_hubs(cfg, rig).items()}
    blocked = {n: w for n, w in blocked.items() if w}
    check("the show's hubs are clear of the room and the arm, the tool included", not blocked, blocked)

    # the matrix: every family x mode from every hub; the sweep: every family
    # from every hub, SEEDS seeds, TRIES draws, the mode drawn
    hubs = ["rest", "greet"] + sorted(h for h in cfg["hubs"] if h not in ("rest", "greet"))
    jobs = [("matrix", h, fm, m, 0, None) for h in hubs for fm in FAMILIES for m in ORIENTS]
    hubs += [name for name, _ in G.SYNTHETIC]
    jobs += [("sweep", h, fm, None, s, None) for h in hubs for fm in FAMILIES for s in range(SEEDS)]
    jobs += [("top", "greet", fm, None, 100 + s, 1.0) for fm in FAMILIES for s in range(4)]
    res = _run(jobs)
    hubs = [h for h in hubs if any(r["hub"] == h for r in res)]
    mat = [r for r in res if r["kind"] == "matrix"]
    sweep = [r for r in res if r["kind"] == "sweep"]
    top = [r for r in res if r["kind"] == "top" and r["made"]]
    print("\n%-6s %-10s %-8s %5s %6s %7s %6s %6s %6s  %s" % ("hub", "family", "mode", "tries", "dur s", "ext m", "z m",
                                                         "peak", "slow", "plane / form / action"))
    for r in mat:
        if not r["made"]:
            print("%-6s %-10s %-8s %5d   --- none" % (r["hub"], r["fam"], r["mode"], r["tries"]))
            continue
        print("%-6s %-10s %-8s %5d %6.2f %7.3f %6.2f %6.2f %6.2f  %s / %s / %s" % (
            r["hub"], r["fam"], r["mode"], r["tries"], r["dur"], r["extent"], r["zspan"], r["vpeak"], r["slowed"],
            r["plane"], r["form"], r["action"]))
    print()
    check("every family x mode makes a clip from every show hub (%s)" % ", ".join(sorted({r["hub"] for r in mat})),
          all(r["made"] for r in mat),
          [(r["hub"], r["fam"], r["mode"]) for r in mat if not r["made"]])
    cells = {}
    for r in sweep:
        cells.setdefault((r["fam"], r["hub"]), []).append(r["made"])
    print("     sweep, clips within %d draws per family and hub: %s" % (TRIES, {"%s/%s" % c: "%d/%d" % (sum(v), len(v))
                                                                          for c, v in sorted(cells.items())}))
    low = {"%s/%s" % c: "%d/%d" % (sum(v), len(v)) for c, v in cells.items() if sum(v) < 0.8 * len(v)}
    check("at least 80%% of seeds make a clip within %d draws, every family from every hub" % TRIES, not low, low)
    made = [r for r in res if r["made"]]
    check("each starts and ends at its hub (1e-3 deg)", max(r["end"] for r in made) < 1e-3, max(r["end"] for r in made))
    check("at rest at both ends (first / last frame steps under %g deg, as a gesture's)" % G.REST_STEP_DEG,
          max(r["rest_v"] for r in made) <= G.REST_STEP_DEG * FPS, "max %.4f deg" % (max(r["rest_v"] for r in made) / FPS))
    check("no step near a branch flip", max(r["step"] for r in made) < G.MAX_STEP_DEG, "max %.2f deg a frame" % max(r["step"] for r in made))
    check("joint velocity / acceleration within the limits at the plan safety %.2f" % _plan_safety(cfg),
          max(r["need"] for r in made) <= 1.0 + 1e-6, "worst %.3f" % max(r["need"] for r in made))
    check("clear of the room (every clip checked, min clearance)", all(r["clear"] is not None for r in made),
          "min %.3f m" % min(r["clear"] for r in made))
    same = [r["room_same"] for r in made if "room_same" in r]
    check("the near-room check agrees with the whole room", same and all(same), "%d/%d" % (sum(same), len(same)))
    ds = [r["dur"] for r in made]
    check("%g-%g s long, some short accents (<= 5 s)" % (G.MIN_CLIP_S, G.MAX_CLIP_S),
          all(G.MIN_CLIP_S - 1e-6 <= d <= G.MAX_CLIP_S + 1e-6 for d in ds) and sum(d <= 5.0 for d in ds) >= 0.08 * len(ds),
          "%.1f-%.1f s, %d%% <= 5 s" % (min(ds), max(ds), 100 * sum(d <= 5.0 for d in ds) / len(ds)))
    lo_d, hi_d = cfg["library"]["duration_s"]
    if lo_d > G.MIN_CLIP_S or hi_d < G.MAX_CLIP_S:
        print("     note: shows/party.json library.duration_s %s drops the clips outside it (%d/%d here)" % (
            [lo_d, hi_d], sum(not lo_d <= d <= hi_d for d in ds), len(ds)))
    in_r = sum(r["in_range"] for r in made)
    check("inside the show's operating range", in_r == len(made), "%d/%d" % (in_r, len(made)))
    ext = [r["extent"] for r in made]
    check("human- to body-scale figures (largest TCP extent 0.12-0.9 m)", all(0.12 <= e <= 0.9 for e in ext),
          "%.2f-%.2f m" % (min(ext), max(ext)))
    ok_play = sum(r["safe"] for r in made)
    check("every clip plays at its own speed (safety ok)", ok_play == len(made), "%d/%d" % (ok_play, len(made)))
    avg = {m: round(sum(r["wrist"] for r in mat if r["made"] and r["mode"] == m) /
                    max(1, sum(r["made"] and r["mode"] == m for r in mat)), 2) for m in ORIENTS}
    check("the wrist acts more when it looks or leans than when fixed",
          avg.get("look", 0) > avg.get("fixed", 1) and avg.get("tangent", 0) > avg.get("fixed", 1), avg)
    check("labels: measured, family, typed params", all(r["labels"] for r in made))
    flat = min(made, key=lambda r: r["ratio"])
    check("not constant speed (peak / mean TCP speed > 1.3)", flat["ratio"] > 1.3,
          "min %.2f (%s from %s, %s, %s)" % (flat["ratio"], flat["fam"], flat["hub"], flat["plane"], flat["form"]))
    sw = [r for r in sweep if r["made"]]
    def med(xs):
        return sorted(xs)[len(xs) // 2] if xs else 0.0
    check("through heights: median TCP height span >= 0.15 m", med([r["zspan"] for r in sw]) >= 0.15,
          "median %.3f m" % med([r["zspan"] for r in sw]))
    check("big: median TCP extent >= 0.3 m", med([r["extent"] for r in sw]) >= 0.3, "median %.3f m" % med([r["extent"] for r in sw]))
    hx = [r["zspan"] for r in sw if r["fam"] == "helix"]
    check("helices climb (median height span >= 0.3 m)", med(hx) >= 0.3, "median %.2f m, min %.2f" % (med(hx), min(hx)))
    te = [r["extent"] for r in top]
    check("full intensity from greet: figures 0.3-0.6 m or more (median)", len(top) >= 0.8 * 4 * len(FAMILIES) and med(te) >= 0.3,
          "%d made, median %.2f m, %.2f-%.2f" % (len(top), med(te), min(te), max(te)))
    lo_k = [r for r in sw if r["k"] <= 0.45]
    hi_k = [r for r in sw if r["k"] >= 0.75]
    check("intensity scales size and speed (k >= 0.75 vs <= 0.45: extent x%.2f, mean speed x%.2f)" % (
          med([r["extent"] for r in hi_k]) / max(1e-6, med([r["extent"] for r in lo_k])),
          med([r["vmean"] for r in hi_k]) / max(1e-6, med([r["vmean"] for r in lo_k]))),
          med([r["extent"] for r in hi_k]) > 1.2 * med([r["extent"] for r in lo_k])
          and med([r["vmean"] for r in hi_k]) > 1.5 * med([r["vmean"] for r in lo_k]))
    planes = {}
    for r in sw:
        planes[r["plane"]] = planes.get(r["plane"], 0) + 1
    upright = sum(v for p, v in planes.items() if p != "floor")
    check("mostly upright or leaning planes (through heights)", upright >= 0.75 * len(sw), planes)
    whys = [w for r in res for w in r["whys"]]
    if whys:
        counts = {}
        for w in whys:
            key = w.split(" at ")[0][:50]
            counts[key] = counts.get(key, 0) + 1
        print("     redraws: %s" % dict(sorted(counts.items(), key=lambda x: -x[1])[:8]))
    print("     %d clips from %d tries in %.1f s" % (len(made), sum(r["tries"] for r in res), time.time() - t_start))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def _plan_safety(cfg):
    return (cfg.get("range") or {}).get("speed", PLAN_SAFETY)


if __name__ == "__main__":
    sys.exit(self_test())
