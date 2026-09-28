"""Paths: lively spatial figures the tool draws around a show hub.

gestures.py makes the arm look and lean; choreo.py makes it dance in joint
space. Here the TCP draws a readable FIGURE in the air at human scale
(0.15-0.5 m) -- a Lissajous knot, a figure eight, a spiral, a coil, a rose,
a loose loop -- the kind of shape a person traces with a hand, while the
wrist carries a character of its own (looking at the audience, leaning
like a brush, or keeping the hub's aim).

Every figure is closed: it starts and ends at the hub's TCP, at rest, so it
chains in the show graph like any clip. The figure blooms out of the hub
point (its amplitude ramps up from 0 while its centre drifts out to an
offset) and folds back into it. Families:

    lissajous   sin(a t + phase), sin(b t) for a:b in 1:2, 2:1, 2:3, 3:2,
                1:3, 3:4, with a small swing out of the plane
    figure8     an eight (standing or lying), bowed into a saddle
    spiral      in the plane: grows turn by turn, then shrinks, domed
                towards the audience
    helix       a coil whose axis is the plane normal: rises, then falls
                back, the radius opening and closing
    rose        r = cos(k t): 3, 4 or 5 petals, the petals bowed
    spline      a smooth closed loop (periodic C2 cubic) through the hub
                and 3-5 random points in the hub's zone

Typed parameters per clip, drawn from an rng with ranges scaled by an
intensity 0..1 (and returned in labels["params"]): extent (m), aspect,
depth (out of plane), plane (a named plane -- audience: facing the
audience; tilted: facing it and leaning back; floor: horizontal; side:
edge-on to it -- and its normal), centre offset from the hub TCP, cycles,
bpm / beats, and the tool orientation mode:

    look      the tool aims at a point in the audience zone that drifts
              slowly from face to face (the swing from the hub's aim is
              capped, so a hub facing away still turns towards people)
    tangent   the tool leans along the direction of travel, trailing
              like a brush -- the lean grows with the speed
    fixed     the hub's tool frame throughout (the wrist only compensates)

In look and tangent modes the tool also rolls with the figure's rhythm.

Timing: the figure is paced along the arc length of the figure at full
size (so a spiral's small turns take as long as its big ones, as a hand
draws them) with the two-thirds power law of human drawing (slower in the
tight turns, faster on the straights: v ~ curvature^-1/3), capped where a
turn would ask the TCP for more than a_mps2 (v <= sqrt(a / curvature));
the cruise takes a whole number of beats; min-jerk ease in and out at
both ends (velocity and acceleration 0). The look / lean / roll fade in
and out with the same kind of ramp, so the tool frame is the hub's at
both ends. Sampled at 24 fps through the closed-form
IK (nearest branch, a step over gestures.MAX_STEP_DEG is a branch flip),
then slowed uniformly if the player would slow it (fairino_player.limiting
at the plan safety) -- never clipped.

    make(rig, hub_q, family, zones, rng, bpm=90, intensity=0.6, clip_id=None,
         env=None, safety=PLAN_SAFETY, orient=None, zone=None, why=None)
        -> motion clip dict (as gestures.make) or None; labels["family"],
           labels["params"] (the typed parameters, as drawn and fitted)
    draw(family, home, zones, rng, bpm, intensity, orient, zone) -> params
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
NEAR_ROOM_M = 0.3
LOOK_FAR = 1.4                   # m: the look point's least distance from the hub                # room objects farther than their margin + this cannot be touched
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
    if s < 1e-12:
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
            lift = k["rise_m"] * (0.5 - 0.5 * math.cos(2 * math.pi * s))
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

    def fits(self, box, n=240):
        return all(G.inside(box, self.at(i / float(n))[0], ZONE_PAD) for i in range(n + 1))


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


PLANE_WEIGHTS = {
    "lissajous": (("audience", 5), ("tilted", 2.5), ("floor", 1.5), ("side", 1)),
    "figure8": (("audience", 4.5), ("tilted", 2.5), ("floor", 2), ("side", 1)),
    "spiral": (("audience", 5), ("tilted", 3), ("floor", 2)),
    "helix": (("floor", 6), ("audience", 4)),
    "rose": (("audience", 5), ("tilted", 3), ("floor", 2)),
    "spline": (("audience", 1),),
}
COUNTS = {"lissajous": (1, 2, 3), "figure8": (1, 2, 3), "spiral": (3, 4, 5, 6), "helix": (3, 4, 5, 6),
          "rose": (1, 2), "spline": (1, 2)}


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


def draw(family, home, zones, rng, bpm, k, orient=None, zone=None):
    """The typed parameters of one clip (a dict), or None when the figure
    cannot be fitted into its zone. home = (tcp, look, roll) of the hub."""
    p0 = home[0]
    beat = 60.0 / bpm
    h = audience_dir(zones, p0)
    zname, box = zone_of(zones, p0, zone)
    if box is not None and not G.inside(box, p0, ZONE_PAD):
        zname, box = None, None                       # a hub outside its zone: the room check still holds
    orient = orient or _pick(rng, (("look", 4.5), ("tangent", 3.5), ("fixed", 2)))
    plane = _pick(rng, PLANE_WEIGHTS[family])
    n, u, v = plane_axes(plane, h)
    extent = max(0.15, min(0.5, _lerp(0.2, 0.45, k) * rng.uniform(0.85, 1.15)))
    prm = {"family": family, "orient": orient, "plane": plane, "normal": [round(x, 4) for x in n],
           "zone": zname, "extent_m": extent, "aspect": rng.uniform(0.65, 1.0),
           "depth": rng.uniform(0.0, 0.25 + 0.3 * k), "bpm": bpm, "intensity": round(k, 3),
           "ramp": rng.uniform(0.18, 0.3)}
    # the centre drifts off the hub point: across the plane, and in towards the audience
    ou, ov, oh = rng.uniform(-0.3, 0.3) * extent, rng.uniform(-0.2, 0.3) * extent, rng.uniform(0.0, 0.05 + 0.1 * k)
    prm["offset"] = _vsum((ou, u), (ov, v), (oh, h))
    if family == "lissajous":
        prm["ratio"] = list(rng.choice(((1, 2), (2, 1), (2, 3), (3, 2), (1, 3), (3, 4))))
        prm["phase_deg"] = rng.choice((30.0, 45.0, 60.0, 90.0)) * rng.choice((1, -1))
    elif family == "figure8":
        prm["lying"] = rng.random() < 0.5
        prm["depth"] = rng.uniform(0.2, 0.35 + 0.35 * k)
    elif family == "spiral":
        prm["sense"] = rng.choice((1, -1))
        prm["depth"] = rng.uniform(0.1, 0.3 + 0.4 * k)
        prm["aspect"] = rng.uniform(0.8, 1.0)
    elif family == "helix":
        prm["sense"] = rng.choice((1, -1))
        prm["rise_m"] = extent * rng.uniform(0.5, 0.9) * rng.choice((1, -1) if plane == "floor" else (1,))
        prm["aspect"] = rng.uniform(0.85, 1.0)
        prm["ramp"] = rng.uniform(0.12, 0.2)
    elif family == "rose":
        petals = rng.choice((3, 4, 5))
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
                          (rng.uniform(-1, 1) * extent * 0.5, v), (rng.uniform(-0.3, 0.6) * extent * 0.5, h))
                last = pts[-1] if pts else p0
                if (box is None or G.inside(box, q, ZONE_PAD + 0.03)) and math.dist(q, last) > 0.35 * extent:
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
    # the timing: a human drawing speed, slower in the turns (power law, and
    # a cap on the TCP's acceleration), a whole number of beats; as many
    # cycles as come closest to a clip length drawn from 4.5-9 s
    prm["v_mps"] = round(_lerp(0.2, 0.42, k) * rng.uniform(0.85, 1.15), 4)
    prm["a_mps2"] = round(_lerp(0.8, 1.3, k), 4)
    prm["power_law"] = round(rng.uniform(0.6, 1.0), 3)
    prm["ease_s"] = round(min(1.6, max(0.6, beat * rng.choice((1.0, 1.5, 2.0)))), 3)
    target_s = rng.uniform(4.5, 9.0)
    while True:
        best = None
        for c in COUNTS[family]:
            if family in ("spiral", "helix"):
                prm["turns"], prm["cycles"] = c, 1
            else:
                prm["cycles"] = c
            dur = Pace(Figure(family, prm, p0, (n, u, v)), prm, 500).duration
            if best is None or abs(dur - target_s) < abs(best[0] - target_s):
                best = (dur, c)
        if best[0] < 10.0 or prm["extent_m"] < 0.2:
            break
        _resize(prm, p0, 0.88)                         # too long even once round: a smaller figure
    if family in ("spiral", "helix"):
        prm["turns"], prm["cycles"] = best[1], 1
    else:
        prm["cycles"] = best[1]
    # into the zone
    fig = Figure(family, prm, p0, (n, u, v))
    if box is not None:
        while not fig.fits(box):
            fig.scale *= 0.85
            if prm["extent_m"] * fig.scale < 0.12:
                return None
        if fig.scale < 1.0:
            _resize(prm, p0, fig.scale)
            fig = Figure(family, prm, p0, (n, u, v))
            if not fig.fits(box):
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
    both ends."""

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
        v = [max(0.02, min(x, math.sqrt(prm["a_mps2"] / max(1e-6, c)))) for x, c in zip(v, kap)]
        v = _smooth(v, w)
        tau = [0.0]
        for i in range(n):
            tau.append(tau[-1] + ds[i] / (0.5 * (v[i] + v[i + 1])))
        beat = 60.0 / prm["bpm"]
        self.beats = max(2, int(math.ceil(tau[-1] / beat - 0.2)))
        cruise = self.beats * beat
        self.tau = [x * cruise / tau[-1] for x in tau]
        self.s = [i / float(n) for i in range(n + 1)]
        self.cruise, self.ease = cruise, prm["ease_s"]
        self.duration = cruise + self.ease

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
    """[(t, tcp, R)] at 24 fps for the clip played `slow` times slower."""
    R0, p0, d0 = rig.tool(hub_q)
    T = pace.duration
    n = int(math.ceil(T * slow * FPS))
    ts = [i / FPS for i in range(n + 1)]
    td = [min(T, t / slow) for t in ts]                   # design time of each frame
    orient = prm["orient"]
    tw = max(pace.ease, 0.2 * T)                          # the look / roll fade in and out

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
    prm = draw(family, home, zones, rng, bpm, intensity, orient, zone)
    if prm is None:
        return fail("does not fit its zone")
    fig = Figure(family, prm, home[0], plane_axes(prm["plane"], audience_dir(zones, home[0])))
    pace = Pace(fig, prm)
    prm["beats"], prm["cruise_s"] = pace.beats, round(pace.cruise, 4)
    vel, acc = [v * safety for v in rig.vel], [a * safety for a in rig.acc]
    slow = 1.0
    for _ in range(4):
        frs = frames(rig, hub_q, fig, prm, pace, slow)
        qs, reason = solve_frames(rig, hub_q, frs)
        if qs is None:
            return fail(reason)
        ts = [f[0] for f in frs]
        s = P.limiting(ts, qs, 125.0, vel, acc)["scale_needed"]
        if s <= 1.0:
            break
        slow *= s * 1.03
        if slow > MAX_SLOW:
            return fail("would play %.1fx slower than designed" % slow)
    else:
        return fail("timing did not settle")
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
    if env is not None:
        model = CL.load_model("fr20")
        near, left_out = near_room(env, model, qs)
        rep = CL.check(model, near, ts, qs)
        clip["safety"]["collision"] = CL.describe(rep)
        clear = rep["min_env_clearance_m"]
        clip["safety"]["min_clearance_m"] = clear if clear is not None else round(left_out, 4)
        clip["safety"]["min_self_clearance_m"] = rep["min_self_clearance_m"]
        if not rep["ok"]:
            return fail("hits the room: %s" % CL.describe(rep)[:80])
    clip["labels"] = motion_labels.label(clip)
    clip["labels"]["family"] = family
    prm["slowed"] = round(slow, 3)
    clip["labels"]["params"] = prm
    return clip


def wrist_share(qs):
    return G.wrist_share(qs)


# --------------------------------------------------------------------------
# self-test
# --------------------------------------------------------------------------

def self_test():
    import json
    import random
    import time
    import collision as CL
    import show
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    t_start = time.time()
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    rig = G.Rig()
    hubs = show.resolve_hubs(cfg, rig)
    env = show.show_env(CL.load_env(os.path.join(ROOT, cfg["env"])), cfg, cfg["margins"]["idle_canvas_m"])
    zones = cfg["zones"]
    safety = (cfg.get("range") or {}).get("speed", PLAN_SAFETY)
    lo_d, hi_d = cfg["library"]["duration_s"]

    # the pieces
    f = _periodic_spline([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0.5)])
    check("the closed spline passes its points and closes", math.dist(f(0.0), (0, 0, 0)) < 1e-9
          and math.dist(f(1.0), (0, 0, 0)) < 1e-9 and min(math.dist(f(i / 400.0), (1, 1, 0)) for i in range(401)) < 0.01)
    n, u, v = plane_axes("audience", (0.0, -1.0, 0.0))
    check("the audience plane faces the audience, v up", math.dist(n, (0, -1, 0)) < 1e-9 and v[2] > 0.99, (n, u, v))

    rows, made, tries_total, whys = [], [], 0, []
    for hi, hub in enumerate(("rest", "greet")):
        hq = hubs[hub]
        zone = cfg["hubs"][hub].get("zone")
        for fi, fam in enumerate(FAMILIES):
            for mi, mode in enumerate(ORIENTS):
                rng = random.Random(1000 * hi + 10 * fi + mi + 7)
                clip, tries = None, 0
                while clip is None and tries < 8:
                    tries += 1
                    clip = make(rig, hq, fam, zones, rng, bpm=rng.randint(*cfg["library"]["bpm"]),
                                intensity=rng.uniform(*cfg["library"]["intensity"]), env=env, safety=safety,
                                orient=mode, zone=zone, why=whys)
                tries_total += tries
                rows.append((hub, fam, mode, tries, clip))
                if clip:
                    made.append((hub, fam, mode, hq, clip))
    print("\n%-6s %-10s %-8s %5s %6s %7s %6s %6s  %s" % ("hub", "family", "mode", "tries", "dur s", "ext m",
                                                        "wrist", "slow", "plane / action"))
    for hub, fam, mode, tries, c in rows:
        if c is None:
            print("%-6s %-10s %-8s %5d   --- none" % (hub, fam, mode, tries))
            continue
        ext = max(c["labels"]["descriptors"]["extent_m"])
        print("%-6s %-10s %-8s %5d %6.2f %7.3f %6.2f %6.2f  %s / %s" % (
            hub, fam, mode, tries, c["meta"]["duration_s"], ext, wrist_share([p["q"] for p in c["points"]]),
            c["labels"]["params"]["slowed"], c["labels"]["params"]["plane"], c["labels"]["measured"]["action"]))
    print()
    check("every family x mode makes a clip from both hubs", len(made) == len(rows),
          [(h, f, m) for h, f, m, _, c in rows if c is None])
    per_fam = {}
    for hub, fam, mode, tries, c in rows:
        a = per_fam.setdefault(fam, [0, 0])
        a[0] += 1 if c else 0
        a[1] += tries
    print("     acceptance per family (clips / tries): %s" % {f: "%d/%d" % tuple(x) for f, x in per_fam.items()})
    ends = [max(max(abs(a - b) for a, b in zip(c["points"][0]["q"], hq)),
                max(abs(a - b) for a, b in zip(c["points"][-1]["q"], hq))) for _, _, _, hq, c in made]
    check("each starts and ends at its hub (1e-3 deg)", ends and max(ends) < 1e-3, max(ends) if ends else None)
    rest_v = []
    steps = []
    for _, _, _, hq, c in made:
        q = [p["q"] for p in c["points"]]
        rest_v.append(max(max(abs(a - b) for a, b in zip(q[1], q[0])), max(abs(a - b) for a, b in zip(q[-1], q[-2]))) * FPS)
        steps.append(max(max(abs(a - b) for a, b in zip(x, y)) for x, y in zip(q, q[1:])))
    check("at rest at both ends (first / last frame joint speed < 1 deg/s)", rest_v and max(rest_v) < 1.0,
          "max %.3f deg/s" % max(rest_v))
    check("no step near a branch flip", steps and max(steps) < G.MAX_STEP_DEG, "max %.2f deg a frame" % max(steps))
    vel, acc = [v * safety for v in rig.vel], [a * safety for a in rig.acc]
    need = [P.limiting([p["t"] for p in c["points"]], [p["q"] for p in c["points"]], 125.0, vel, acc)["scale_needed"]
            for _, _, _, _, c in made]
    check("joint velocity / acceleration within the limits at the plan safety %.2f" % safety,
          need and max(need) <= 1.0 + 1e-6, "worst %.3f" % max(need))
    clear = [c["safety"]["min_clearance_m"] for *_, c in made]
    check("clear of the room (every clip checked, min clearance)", all(x is not None for x in clear),
          "min %.3f m" % min(clear))
    # the room check on the pruned room is the check on the whole room
    model = CL.load_model("fr20")
    same = []
    for *_, c in made[::17]:
        rep = CL.check(model, env, [p["t"] for p in c["points"]], [p["q"] for p in c["points"]])
        a, b = rep["min_env_clearance_m"], c["safety"]["min_clearance_m"]
        far = env.get("margin_m", 0.05) + NEAR_ROOM_M
        same.append(rep["ok"] and (abs(a - b) < 1e-6 or (a >= far and b >= far)))
    check("the near-room check agrees with the whole room", all(same), same)
    in_d = sum(lo_d <= c["meta"]["duration_s"] <= hi_d for *_, c in made)
    check("duration within %g-%g s for most" % (lo_d, hi_d), in_d >= 0.85 * len(made), "%d/%d" % (in_d, len(made)))
    in_r = sum(show.out_of_range(cfg, [p["q"] for p in c["points"]], c["tcp"]) is None for *_, c in made)
    check("inside the show's operating range for most", in_r >= 0.85 * len(made), "%d/%d" % (in_r, len(made)))
    ext = [max(c["labels"]["descriptors"]["extent_m"]) for *_, c in made]
    check("human-scale figures (largest TCP extent 0.12-0.6 m)", all(0.12 <= e <= 0.6 for e in ext),
          "%.2f-%.2f m" % (min(ext), max(ext)))
    ok_play = sum(bool(c["safety"]["ok"]) for *_, c in made)
    check("every clip plays at its own speed (safety ok)", ok_play == len(made), "%d/%d" % (ok_play, len(made)))
    ws = {m: [wrist_share([p["q"] for p in c["points"]]) for _, _, mm, _, c in made if mm == m] for m in ORIENTS}
    avg = {m: round(sum(x) / len(x), 2) for m, x in ws.items() if x}
    check("the wrist acts more when it looks or leans than when fixed",
          avg.get("look", 0) > avg.get("fixed", 1) and avg.get("tangent", 0) > avg.get("fixed", 1), avg)
    labs = all(c["labels"]["family"] == f and c["labels"]["params"]["orient"] == m and "measured" in c["labels"]
               for _, f, m, _, c in made)
    check("labels: measured, family, typed params", labs)
    speeds = [c["labels"]["descriptors"]["tcp_peak_mps"] / max(1e-6, c["labels"]["descriptors"]["tcp_mean_mps"])
              for *_, c in made]
    check("not constant speed (peak / mean TCP speed > 1.3)", min(speeds) > 1.3, "min %.2f" % min(speeds))
    if whys:
        counts = {}
        for w in whys:
            key = w.split(" at ")[0][:50]
            counts[key] = counts.get(key, 0) + 1
        print("     redraws: %s" % dict(sorted(counts.items(), key=lambda x: -x[1])))
    el = time.time() - t_start
    print("     %d clips from %d tries in %.1f s" % (len(made), tries_total, el))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
