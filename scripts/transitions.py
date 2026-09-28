"""Jerk-limited joint moves for the show (Ruckig): hub to hub through the
safe_move waypoints without stopping at each one, a stop-and-go from a
moving state to a pose, and the shortest smooth stop.

Why jerk-limited: show.timed_move times the straight joint-space legs with
retime_topp -- time-optimal under velocity and acceleration limits, so the
acceleration switches from +max to -max in one sample (an unbounded jerk:
a knock the arm and its plywood base feel), and it comes to a full stop at
every waypoint corner. Ruckig (the `ruckig` package, community version: one
state-to-state trajectory per call) limits jerk too, so acceleration ramps
over JERK_RISE_S; and a piece can end at a chosen velocity, so the arm
flows past an intermediate waypoint instead of halting there.

    move(waypoints, vel, acc, jerk=None, safety=0.5, dt=1/125)       -> (t, q)
        rest at waypoints[0] to rest at waypoints[-1], blending past the others
    plan_move(...)                                                    -> MovePlan
        the same, kept as Ruckig pieces: at(s) -> (q, v, a), the velocity
        at each waypoint, the measured deviation from the straight legs
    exit_from(q, v, a, target_q, vel, acc, jerk=None, safety=0.5, dt) -> (t, q)
        from a moving state to target_q at rest (leave a clip early)
    ramp_stop(q, v, a, acc, jerk=None, dt)                            -> (t, q)
        the shortest jerk-limited stop, position free (a smooth soft stop)
    compare(cfg_path)                                                 -> rows
        the show's hub moves timed both ways (TOPP vs Ruckig), printed

(t, q) is the shape show.timed_move returns and show.Segment takes: times
in s from 0, joints in degrees. Samples are uniform, at most dt apart: at
dt = 1/125 as dense as the 125 Hz stream, so Segment.at's PCHIP (only C1)
has little to fill in between them.

Past a waypoint without stopping: a piece whose start velocity, end
velocity and displacement all point the same way can run with every joint
on one profile (phase-synchronised: exactly straight in joint space; done
here as one Ruckig profile along the piece). So each leg runs straight
between blend points P_in / P_out on it, entered and left at speed f along
the leg (f as a fraction of the leg's full speed), and one time-
synchronised piece blends from P_in (on the leg in) to P_out (on the leg
out) across the corner W, like the controller's MoveJ blend radius. A curve tangent to both legs
cannot pass exactly through their corner, so it passes near it: a
parabolic blend of half-width tau cuts the corner by tau |dU| / 4 per joint
(dU the change of full-speed velocity), which sets tau for max_dev_deg,
whatever the speed. f starts at the length of the average of the two
legs' directions (joint space measured in seconds at the velocity limit:
1 straight on, 0.71 at a right angle, 0 turning right back, where the arm
stops at W, straight in and out) and is capped so the blend's acceleration
fits. Every piece is then measured -- its per-joint distance from the
nearest straight leg, and any backing up along a leg -- and a corner whose
pieces stray shrinks (f and tau) until they fit, at worst to a stop; and
when stopping at every corner is quicker (tiny blends are slow), the move
does that. The legs are the ones safe_move checked; the caller still
re-checks the collision of what it gets.

Limits: vel and acc per joint (robot_profile.velocity_limits /
acceleration_limits), times safety as timed_move does. The FR20's jerk
limit is unknown (robot_profile.jerk_limits is None: Fairino publishes
none), so jerk defaults to acc / JERK_RISE_S: the acceleration takes 0.2 s
to build up, 25 ticks at 125 Hz -- gentle for a base screwed to plywood,
where J1-J3 already shook at high acceleration. Pass jerk to override
(safety scales it too, so the rise time stays 0.2 s at any safety).

    python scripts/transitions.py            self-test, with the TOPP / Ruckig table
    python scripts/transitions.py --compare  the table, every row collision-checked (slow)
"""

import bisect
import json
import math
import os
import sys

from ruckig import ControlInterface, InputParameter, Result, Ruckig, Synchronization, Trajectory

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

DT = 1.0 / 125.0                  # the ServoJ stream rate
JERK_RISE_S = 0.2                 # default jerk = acc / this (FR20: no published jerk limit)
MAX_DEV_DEG = 2.0                 # per joint, from the straight legs: ~5 cm at the FR20's full reach
BLEND = 0.25                      # a blend reaches at most this fraction of the shorter leg (at full speed) from the corner
SHRINK = 0.6                      # a corner's blend (speed and half-width) shrinks by this while a piece strays
MAX_SHRINKS = 6                   # ... this many times, then the arm stops there (straight in and out)
BACKTRACK_TOL_DEG = 1e-3          # backing up along a leg by more than this is bowing too
ENDPOINT_TOL_DEG = 1e-6
UNBOUNDED = 1e9                   # a limit Ruckig must be given but that does not apply


# --------------------------------------------------------------------------
# limits and one Ruckig trajectory
# --------------------------------------------------------------------------

def default_jerk(acc, rise_s=JERK_RISE_S):
    """Per-joint jerk, deg/s^3, for acceleration acc built up over rise_s."""
    return [a / rise_s for a in acc]


def _limits(vel, acc, jerk, safety):
    jerk = jerk or default_jerk(acc)
    if not (len(vel) == len(acc) == len(jerk)):
        raise ValueError("limits disagree on the joint count: vel %d, acc %d, jerk %d" % (len(vel), len(acc), len(jerk)))
    return [v * safety for v in vel], [a * safety for a in acc], [j * safety for j in jerk]


def _solve(q0, v0, a0, q1, v1, vmax, amax, jmax, interface=ControlInterface.Position):
    """One synchronised Ruckig trajectory from (q0, v0, a0) to (q1, v1, a = 0).
    Phase synchronisation keeps it a straight line when it can be (rest to
    rest), else Ruckig falls back to time synchronisation: every joint
    arrives at once."""
    n = len(q0)
    inp = InputParameter(n)
    inp.current_position, inp.current_velocity, inp.current_acceleration = list(q0), list(v0), list(a0)
    inp.target_position, inp.target_velocity, inp.target_acceleration = list(q1), list(v1), [0.0] * n
    inp.max_velocity, inp.max_acceleration, inp.max_jerk = list(vmax), list(amax), list(jmax)
    inp.control_interface = interface
    inp.synchronization = Synchronization.Phase
    traj = Trajectory(n)
    res = Ruckig(n).calculate(inp, traj)
    if res not in (Result.Working, Result.Finished):
        raise ValueError("ruckig: %s from q %s v %s a %s to q %s v %s (vmax %s amax %s jmax %s)"
                         % (res, _r(q0), _r(v0), _r(a0), _r(q1), _r(v1), _r(vmax), _r(amax), _r(jmax)))
    return traj


def _r(x):
    return [round(float(v), 3) for v in x]


def _sample(duration, at, dt, start=None, end=None):
    """(t, q): uniform samples at most dt apart from 0 to duration, both
    ends included. start / end, when given, are where the curve must begin
    and end: the samples there are set to them exactly (after checking
    Ruckig got within ENDPOINT_TOL_DEG)."""
    if duration <= 0.0:
        q = list(start if start is not None else at(0.0))
        return [0.0], [q]
    n = max(1, int(math.ceil(duration / dt - 1e-9)))
    t = [duration * i / n for i in range(n + 1)]
    q = [list(at(s)) for s in t]
    for i, want in ((0, start), (-1, end)):
        if want is None:
            continue
        off = max(abs(a - b) for a, b in zip(q[i], want))
        if off > ENDPOINT_TOL_DEG:
            raise ValueError("ruckig ended %.2e deg from the %s pose %s" % (off, "start" if i == 0 else "end", _r(want)))
        q[i] = [float(x) for x in want]
    return t, q


# --------------------------------------------------------------------------
# move: rest to rest through waypoints
# --------------------------------------------------------------------------

def _full_speed(d, vmax):
    """The joint velocity along d with its limiting joint at its limit
    (d divided by the leg's time at full speed)."""
    k = max(abs(x) / v for x, v in zip(d, vmax))
    return [x / k for x in d] if k > 0 else [0.0] * len(d)


def _full_time(d, vmax):
    return max(abs(x) / v for x, v in zip(d, vmax))


def corner_speeds(pts, vmax):
    """f per waypoint: 0 at the ends; at a corner the length of the average
    of the two legs' directions (in joint space scaled by the velocity
    limits: seconds), sqrt((1 + cos) / 2) -- 1 straight on, 0.71 at a right
    angle, 0 turning right back."""
    out = [0.0]
    for i in range(1, len(pts) - 1):
        a = [(y - x) / v for x, y, v in zip(pts[i - 1], pts[i], vmax)]
        b = [(y - x) / v for x, y, v in zip(pts[i], pts[i + 1], vmax)]
        cos = sum(x * y for x, y in zip(a, b)) / (math.sqrt(sum(x * x for x in a) * sum(y * y for y in b)) or 1.0)
        out.append(math.sqrt(max(0.0, (1.0 + cos) / 2.0)))
    out.append(0.0)
    return out


def _blend(pts, i, f, tau, vmax, amax, jmax, max_dev_deg):
    """The blend at corner i: (tau, f) cut so that the
    parabolic blend cuts the corner by at most 0.8 max_dev_deg per joint
    (tau |dU| / 4), and every joint can change its velocity by f |dU| in
    the blend's time 2 tau / f at 0.8 of its acceleration limit, ramps
    included (f |dU| / a + a / j): Ruckig then need not stretch the blend
    and slow the other joints in it."""
    din = [y - x for x, y in zip(pts[i - 1], pts[i])]
    dout = [y - x for x, y in zip(pts[i], pts[i + 1])]
    uin, uout = _full_speed(din, vmax), _full_speed(dout, vmax)
    du = [b - a for a, b in zip(uin, uout)]
    worst = max(abs(x) for x in du)
    if worst > 0:
        tau = min(tau, 0.8 * 4.0 * max_dev_deg / worst)
        for x, a, j in zip(du, amax, jmax):
            if x:
                a, x = 0.8 * a, abs(x)
                r = a / j                                   # the acceleration ramp's time
                f = min(f, (-r + math.sqrt(r * r + 8.0 * tau * x / a)) / (2.0 * x / a))
    return tau, f


class _Line:
    """A straight piece a -> b entered at velocity v0 and left at v1 (both
    along it): one Ruckig profile in the fraction s along the piece, every
    joint on it -- phase synchronisation by construction (Ruckig's own
    phase mode drops to time synchronisation on some collinear inputs,
    rounding, and the piece bows). Limits: the tightest joint's, per unit s.
    at_time as a Ruckig Trajectory's."""

    def __init__(self, a, v0, b, v1, vmax, amax, jmax):
        self.a, self.d = list(a), [y - x for x, y in zip(a, b)]
        dd = sum(x * x for x in self.d)
        sd0, sd1 = (sum(x * y for x, y in zip(v, self.d)) / dd for v in (v0, v1))

        def lim(limits):
            return min(m / abs(x) for m, x in zip(limits, self.d) if x)

        self.s = _solve([0.0], [sd0], [0.0], [1.0], [sd1], [lim(vmax)], [lim(amax)], [lim(jmax)])
        self.duration = self.s.duration

    def at_time(self, t):
        (s,), (sd,), (sdd,) = self.s.at_time(t)
        return ([x + s * y for x, y in zip(self.a, self.d)], [sd * y for y in self.d], [sdd * y for y in self.d])


def _pieces(pts, f, tau, vmax):
    """The Ruckig pieces: [(q0, v0, q1, v1, legs)] with legs the straight
    legs the piece runs along (one, or the two either side of a blend); and
    per waypoint (piece index, fraction of it) where the arm is at or
    nearest to it."""
    n = len(pts[0])
    zero = [0.0] * n
    legs = [[y - x for x, y in zip(a, b)] for a, b in zip(pts, pts[1:])]
    U = [_full_speed(d, vmax) for d in legs]
    h = [_full_time(d, vmax) for d in legs]
    # the points in order: (q, v, leg it continues on, a blend to the next)
    seq = [(pts[0], zero, 0, False)]
    for i in range(1, len(pts) - 1):
        if f[i] <= 0.0:
            seq.append((pts[i], zero, i, False))
            continue
        p_in = [w - tau[i] / h[i - 1] * d for w, d in zip(pts[i], legs[i - 1])]      # tau at full speed from W
        p_out = [w + tau[i] / h[i] * d for w, d in zip(pts[i], legs[i])]
        seq += [(p_in, [f[i] * x for x in U[i - 1]], i - 1, True), (p_out, [f[i] * x for x in U[i]], i, False)]
    seq.append((pts[-1], zero, len(legs) - 1, False))
    pieces, marks = [], [(0, 0.0)]
    for (q0, v0, leg, blend), (q1, v1, _, _) in zip(seq, seq[1:]):
        if blend:
            marks.append((len(pieces), 0.5))                   # the blend's middle: nearest the corner
            pieces.append((q0, v0, q1, v1, (leg, leg + 1)))
        else:
            pieces.append((q0, v0, q1, v1, (leg,)))
            if q1 is not pts[-1] and not any(v1):
                marks.append((len(pieces), 0.0))               # a stop at a corner
    marks.append((len(pieces), 0.0))
    return pieces, marks


def _deviation(traj, pts, legs, dt):
    """(per joint, the largest distance (deg) of a piece's curve from its
    straight leg(s) -- each sample against its nearest point of the nearest
    leg; how far (deg) it backs up along a single leg)."""
    segs = []
    for k in legs:
        a, b = pts[k], pts[k + 1]
        d = [y - x for x, y in zip(a, b)]
        segs.append((a, d, sum(x * x for x in d), max(abs(x) for x in d)))
    dur = traj.duration
    m = max(2, int(math.ceil(dur / dt)))
    worst, back, ahead = [0.0] * len(pts[0]), 0.0, -1.0
    for i in range(m + 1):
        q = traj.at_time(dur * i / m)[0]
        best = None
        for a, d, dd, span in segs:
            u = min(1.0, max(0.0, sum((x - y) * z for x, y, z in zip(q, a, d)) / dd)) if dd > 0 else 0.0
            off = [abs(x - y - u * z) for x, y, z in zip(q, a, d)]
            if best is None or max(off) < max(best[0]):
                best = (off, u, span)
        worst = [max(x, y) for x, y in zip(worst, best[0])]
        if len(segs) == 1:
            ahead = max(ahead, best[1])
            back = max(back, (ahead - best[1]) * best[2])
    return worst, back


class MovePlan:
    """A move as Ruckig pieces end to end: straight legs, and a blend
    across each corner the arm does not stop at."""

    def __init__(self, pts, trajs, marks, f, tau, deviation):
        self.waypoints, self.trajs, self.f, self.tau, self.deviation = pts, trajs, f, tau, deviation
        self.starts = [0.0]
        for tr in trajs:
            self.starts.append(self.starts[-1] + tr.duration)
        # when the arm is at each waypoint (or, blending past it, nearest to it)
        self.waypoint_times = [self.starts[k] + (frac * trajs[k].duration if frac else 0.0) for k, frac in marks]

    @property
    def duration(self):
        return self.starts[-1]

    @property
    def max_deviation_deg(self):
        """The most any joint strays from the straight legs (deg)."""
        return max([max(d) for d in self.deviation] or [0.0])

    @property
    def pass_vel(self):
        """The joint velocity at (or nearest) each waypoint, deg/s."""
        return [self.at(s)[1] for s in self.waypoint_times]

    def at(self, s):
        """(q, v, a) at time s, held at the ends."""
        n = len(self.waypoints[0])
        if not self.trajs or s <= 0.0:
            return list(self.waypoints[0]), [0.0] * n, [0.0] * n
        if s >= self.duration:
            return list(self.waypoints[-1]), [0.0] * n, [0.0] * n
        k = min(bisect.bisect_right(self.starts, s) - 1, len(self.trajs) - 1)
        q, v, a = self.trajs[k].at_time(s - self.starts[k])
        return list(q), list(v), list(a)

    def sample(self, dt=DT):
        return _sample(self.duration, lambda s: self.at(s)[0], dt, self.waypoints[0], self.waypoints[-1])


def _plan(waypoints, vel, acc, jerk, safety, max_dev_deg, dt, blend):
    pts = [[float(x) for x in waypoints[0]]]
    for w in waypoints[1:]:
        if max(abs(float(x) - y) for x, y in zip(w, pts[-1])) > 1e-9:     # a repeated waypoint is no leg
            pts.append([float(x) for x in w])
    vmax, amax, jmax = _limits(vel, acc, jerk, safety)
    if len(pts) == 1:
        return MovePlan(pts, [], [], [0.0], [0.0], [])
    h = [_full_time([y - x for x, y in zip(a, b)], vmax) for a, b in zip(pts, pts[1:])]
    f, tau = corner_speeds(pts, vmax) if blend else [0.0] * len(pts), [0.0] * len(pts)
    for i in range(1, len(pts) - 1):
        if f[i] > 0.0:
            tau[i], f[i] = _blend(pts, i, f[i], BLEND * min(h[i - 1], h[i]), vmax, amax, jmax, max_dev_deg)
    shrinks = [0] * len(pts)
    zero = [0.0] * len(pts[0])
    while True:
        pieces, marks = _pieces(pts, f, tau, vmax)
        trajs = [_Line(q0, v0, q1, v1, vmax, amax, jmax) if len(legs) == 1 else
                 _solve(q0, v0, zero, q1, v1, vmax, amax, jmax) for q0, v0, q1, v1, legs in pieces]
        dev = [[0.0] * len(zero) for _ in range(len(pts) - 1)]
        bad = set()
        for tr, (_, _, _, _, legs) in zip(trajs, pieces):
            worst, back = _deviation(tr, pts, legs, dt)
            for leg in legs:
                dev[leg] = [max(x, y) for x, y in zip(dev[leg], worst)]
            if max(worst) > max_dev_deg or back > BACKTRACK_TOL_DEG:
                # the corners either end of this piece (a blend is its own corner)
                ends = {legs[0], legs[-1] + 1} if len(legs) == 1 else {legs[1]}
                bad.update(c for c in ends if 0 < c < len(pts) - 1 and f[c] > 0.0)
        if not bad:
            return MovePlan(pts, trajs, marks, f, tau, dev)
        for c in bad:
            shrinks[c] += 1
            if shrinks[c] > MAX_SHRINKS:
                f[c] = 0.0                                    # stop there: rest to rest, straight
            else:
                f[c], tau[c] = f[c] * SHRINK, tau[c] * SHRINK
        # every piece between stops is rest to rest and in phase: straight, so this ends


def plan_move(waypoints, vel, acc, jerk=None, safety=0.5, max_dev_deg=MAX_DEV_DEG, dt=DT):
    """The move as a MovePlan (see move()): blending past the corners, or
    stopping at each when that is as quick (tiny blends are slower)."""
    blend = _plan(waypoints, vel, acc, jerk, safety, max_dev_deg, dt, blend=True)
    if not any(blend.f):
        return blend
    stop = _plan(waypoints, vel, acc, jerk, safety, max_dev_deg, dt, blend=False)
    return blend if blend.duration < stop.duration else stop


def move(waypoints, vel, acc, jerk=None, safety=0.5, dt=DT, max_dev_deg=MAX_DEV_DEG):
    """(t, q): a jerk-limited joint move from rest at waypoints[0] to rest at
    waypoints[-1], blending past the waypoints between without stopping
    (every joint synchronised; within max_dev_deg per joint of the straight
    legs). vel, acc (and jerk, default acc / JERK_RISE_S) are per-joint
    limits in deg/s, deg/s^2, deg/s^3, scaled by safety."""
    return plan_move(waypoints, vel, acc, jerk, safety, max_dev_deg, dt).sample(dt)


# --------------------------------------------------------------------------
# leaving a moving state
# --------------------------------------------------------------------------

def exit_from(state_q, state_v, state_a, target_q, vel, acc, jerk=None, safety=0.5, dt=DT):
    """(t, q): from a moving state (joints, velocities, accelerations, deg
    units) to target_q at rest, jerk-limited, all joints arriving together.
    A state faster than the limits is braked into them first (Ruckig). The
    path is not straight -- it carries on in the direction it was moving
    before turning for the target -- so check its collision."""
    vmax, amax, jmax = _limits(vel, acc, jerk, safety)
    n = len(state_q)
    tr = _solve(state_q, state_v, state_a, target_q, [0.0] * n, vmax, amax, jmax)
    return _sample(tr.duration, lambda s: tr.at_time(s)[0], dt, state_q, target_q)


def ramp_stop(state_q, state_v, state_a, acc, jerk=None, dt=DT):
    """(t, q): the shortest jerk-limited stop from a moving state, where it
    ends free. acc and jerk are the limits to use as given (no safety
    factor: pass what the stop may use); jerk defaults to acc / JERK_RISE_S.
    The joints stop together, in the time the slowest one needs, and in
    phase when they were moving in phase (the path keeps its direction)."""
    jerk = jerk or default_jerk(acc)
    n = len(state_q)
    tr = _solve(state_q, state_v, state_a, state_q, [0.0] * n, [UNBOUNDED] * n, acc, jerk,
                interface=ControlInterface.Velocity)
    return _sample(tr.duration, lambda s: tr.at_time(s)[0], dt, state_q, None)


# --------------------------------------------------------------------------
# measuring: what the 125 Hz stream sees, and TOPP vs Ruckig
# --------------------------------------------------------------------------

def peaks(t, q):
    """Per joint, the largest |velocity|, |acceleration| and |jerk| by
    divided differences of the samples (each level placed at the midpoints
    of the one before): {"vel": [...], "acc": [...], "jerk": [...]}."""
    n = len(q[0])
    tt, x = list(t), [list(v) for v in q]
    out = {}
    for name in ("vel", "acc", "jerk"):
        x = [[(b - a) / (t1 - t0) for a, b in zip(x0, x1)] for x0, x1, t0, t1 in zip(x, x[1:], tt, tt[1:])]
        tt = [(a + b) / 2.0 for a, b in zip(tt, tt[1:])]
        out[name] = [max([abs(v[j]) for v in x] or [0.0]) for j in range(n)]
    return out


def stream(t, q, dt=DT, phase=0.0):
    """The samples the show streams from a (t, q) segment: show.Segment.at
    (the player's PCHIP) every dt from phase (the last tick at or before
    the end: after it the stream holds, at rest)."""
    import show
    seg = show.Segment("x", "move", t, q, "a", "b")
    n = int(math.floor((t[-1] - phase) / dt))
    ts = [phase + k * dt for k in range(n + 1)]
    return ts, [seg.at(s) for s in ts]


def polyline_deviation(q, pts):
    """Largest per-joint distance (deg) of samples q from the polyline
    through pts (each sample against its nearest leg)."""
    worst = 0.0
    for x in q:
        best = math.inf
        for a, b in zip(pts, pts[1:]):
            d = [y - z for z, y in zip(a, b)]
            dd = sum(v * v for v in d)
            u = min(1.0, max(0.0, sum((p - z) * v for p, z, v in zip(x, a, d)) / dd)) if dd > 0 else 0.0
            best = min(best, max(abs(p - z - u * v) for p, z, v in zip(x, a, d)))
        worst = max(worst, best)
    return worst


def _load_show_env(cfg):
    """The show config's env through collision.load_env: the named file, or
    its .usda / .json sibling while the room moves to OpenUSD."""
    import collision as C
    path = os.path.join(ROOT, cfg["env"])
    stem = os.path.splitext(path)[0]
    errors = []
    for p in (path, stem + ".usda", stem + ".json"):
        if not os.path.exists(p):
            continue
        try:
            return C.load_env(p), p
        except (ValueError, KeyError, json.JSONDecodeError) as e:     # e.g. a .usda load_env cannot read yet
            errors.append("%s: %s" % (os.path.basename(p), str(e)[:80]))
    raise SystemExit("no env for %s: %s" % (cfg["env"], "; ".join(errors) or "no file"))


def show_moves(cfg_path=None):
    """The show's real hub moves, built as show.build builds them: [(label,
    [waypoints], env)] for rest <-> greet in the show's idle env, plus
    safe_move's self-test detour (a clip pose back to HOME round the
    controller's work area, three legs) in the lab env."""
    import collision as C
    import safe_move
    import show
    cfg_path = cfg_path or os.path.join(ROOT, "shows", "party.json")
    cfg = json.load(open(cfg_path))
    model = C.load_model("fr20")
    env, _ = _load_show_env(cfg)
    idle_env = show.show_env(env, cfg, cfg["margins"]["idle_canvas_m"])
    import gestures as G
    hubs = show.resolve_hubs(cfg, G.Rig())
    out = []
    for a, b in (("rest", "greet"), ("greet", "rest")):
        path, why = safe_move.route(hubs[a], hubs[b], idle_env, model)
        if path is None:
            raise SystemExit("no move %s -> %s: %s" % (a, b, why))
        out.append(("%s -> %s" % (a, b), [hubs[a]] + path, idle_env))
    clip_pose = [102.869, -125.575, -29.215, -209.768, -63.033, 2.070]
    home = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    path, why = safe_move.route(clip_pose, home, env, model)
    if path is not None:
        out.append(("detour -> HOME (%d legs)" % len(path), [clip_pose] + path, env))
    return cfg, model, out


def compare(cfg_path=None, log=print, collide="show"):
    """TOPP (show.timed_move) vs Ruckig (move) on the show's hub moves. Rows:
    move, method, duration s, samples, peak |jerk| deg/s^3 of the samples
    and of the 125 Hz stream (show.Segment.at), peak |acc| of the stream,
    max per-joint deviation from the straight legs (deg), and collision.check
    (ok, min env clearance m) for: collide="all" every row, "show" the
    Ruckig rest <-> greet moves (the self-test's; the check is the slow
    part), "none" none."""
    import collision as C
    import robot_profile as RP
    import show
    cfg, model, moves = show_moves(cfg_path)
    prof = RP.load("fr20")
    vel, acc = RP.velocity_limits(prof), RP.acceleration_limits(prof)
    safety = cfg["transition_safety"]
    rows = []
    for label, pts, env in moves:
        for method in ("TOPP", "Ruckig"):
            if method == "TOPP":
                t, q = show.timed_move(pts, safety, vel, acc)
            else:
                t, q = move(pts, vel, acc, RP.jerk_limits(prof), safety)
            ts, qs = stream(t, q)
            p = peaks(ts, qs)
            row = {"move": label, "method": method, "duration_s": t[-1], "samples": len(t),
                   "jerk_samples": max(peaks(t, q)["jerk"]), "peak_jerk": max(p["jerk"]), "peak_acc": max(p["acc"]),
                   "peak_vel": max(p["vel"]), "dev_deg": polyline_deviation(qs, pts), "ok": None, "clearance_m": None,
                   "t": t, "q": q, "pts": pts}
            if collide == "all" or (collide == "show" and method == "Ruckig" and not label.startswith("detour")):
                r = C.check(model, env, t, q)
                row["ok"], row["clearance_m"] = r["ok"], r["min_env_clearance_m"]
            rows.append(row)
    log("safety %.2f: vel <= %s deg/s, acc <= %s deg/s^2, jerk <= %s deg/s^3 (default: acc / %.2f s)"
        % (safety, _r([v * safety for v in vel]), _r([a * safety for a in acc]),
           _r([j * safety for j in (RP.jerk_limits(prof) or default_jerk(acc))]), JERK_RISE_S))
    log("%-24s %-7s %7s %8s %11s %11s %9s %8s %6s %8s" % (
        "move", "method", "time s", "samples", "jerk (smp)", "jerk @125", "acc @125", "dev deg", "ok", "clear m"))
    for r in rows:
        log("%-24s %-7s %7.2f %8d %11.0f %11.0f %9.1f %8.3f %6s %8s" % (
            r["move"], r["method"], r["duration_s"], r["samples"], r["jerk_samples"], r["peak_jerk"], r["peak_acc"],
            r["dev_deg"], "-" if r["ok"] is None else r["ok"], "-" if r["clearance_m"] is None else r["clearance_m"]))
    log("(jerk deg/s^3 and acc deg/s^2 by finite differences: of the move's own samples, and of the 125 Hz\n"
        " stream show.Segment.at makes from them -- its PCHIP flattens the slope at the sample nearest a\n"
        " joint's turn-around, a ~0.0005 deg ripple that shows as jerk)")
    return rows


# --------------------------------------------------------------------------
# self-test
# --------------------------------------------------------------------------

def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    import robot_profile as RP
    prof = RP.load("fr20")
    vel, acc = RP.velocity_limits(prof), RP.acceleration_limits(prof)
    jerk = default_jerk(acc)
    safety = 0.5
    vmax, amax, jmax = _limits(vel, acc, jerk, safety)
    tol = 1.01                                              # finite differences of an exact curve stay under its peak

    def within(t, q, label):
        p = peaks(t, q)
        worst = max(max(p[k][j] / lim[j] for j in range(6)) for k, lim in (("vel", vmax), ("acc", amax), ("jerk", jmax)))
        check(label, worst <= tol, "peak / limit %.3f" % worst)

    A = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    B = [40.0, -80.0, 80.0, -70.0, -90.0, 10.0]
    C_ = [80.0, -75.0, 60.0, -40.0, -80.0, 30.0]            # B -> C bends ~20 deg off A -> B: a gentle corner

    # rest to rest
    t, q = move([A, B, C_], vel, acc, jerk, safety)
    off = max(max(abs(x - y) for x, y in zip(q[0], A)), max(abs(x - y) for x, y in zip(q[-1], C_)))
    check("endpoints exact", off <= 1e-6, "%.1e deg" % off)
    v0 = max(abs(x - y) / (t[1] - t[0]) for x, y in zip(q[1], q[0]))
    v1 = max(abs(x - y) / (t[-1] - t[-2]) for x, y in zip(q[-1], q[-2]))
    check("... at rest (first / last step under 0.1 deg/s)", v0 < 0.1 and v1 < 0.1, "%.3f / %.3f deg/s" % (v0, v1))
    check("samples at most dt apart", max(b - a for a, b in zip(t, t[1:])) <= DT + 1e-12)
    within(t, q, "velocity, acceleration and jerk within limits x safety")

    # past a corner without stopping
    plan = plan_move([A, B, C_], vel, acc, jerk, safety)
    at_b = plan.at(plan.waypoint_times[1])
    speed = max(abs(v) for v in at_b[1])
    check("a gentle corner is passed, not stopped at", speed > 0.2 * max(vmax), "%.1f deg/s at B" % speed)
    near = max(abs(x - y) for x, y in zip(at_b[0], B))
    check("... within %.1f deg of the waypoint" % MAX_DEV_DEG, near <= MAX_DEV_DEG, "%.3f deg" % near)
    check("... and of the straight legs throughout", plan.max_deviation_deg <= MAX_DEV_DEG,
          "%.3f deg" % plan.max_deviation_deg)
    check("... measured the same way on the samples",
          polyline_deviation(plan.sample()[1], [A, B, C_]) <= MAX_DEV_DEG + 1e-6)
    stops = plan_move([A, B], vel, acc, jerk, safety).duration + plan_move([B, C_], vel, acc, jerk, safety).duration
    check("... and quicker than stopping there", plan.duration < stops, "%.2f s vs %.2f s" % (plan.duration, stops))
    t, q = plan.sample()
    within(t, q, "... within the limits through the blend")
    one = plan_move([A, C_], vel, acc, jerk, safety)
    check("rest to rest in one leg is a straight line", one.max_deviation_deg < 1e-9, "%.1e deg" % one.max_deviation_deg)

    # a tight bound shrinks the blend, never breaks it
    tight = plan_move([A, B, C_], vel, acc, jerk, safety, max_dev_deg=0.3)
    ts = max(abs(v) for v in tight.pass_vel[1])
    check("a tighter deviation bound slows the corner and holds",
          tight.max_deviation_deg <= 0.3 and ts <= speed and tight.duration >= plan.duration,
          "%.3f deg, %.1f deg/s, %.2f s" % (tight.max_deviation_deg, ts, tight.duration))

    # a sharp reversal: back nearly the way it came
    R = [5.0, -88.0, 88.0, -85.0, -90.0, 2.0]
    plan = plan_move([A, B, R], vel, acc, jerk, safety)
    speed = max(abs(v) for v in plan.at(plan.waypoint_times[1])[1])
    check("a sharp reversal stops (or all but) at the corner", speed <= 0.05 * max(vmax), "%.2f deg/s" % speed)
    t, q = plan.sample()
    within(t, q, "... within the limits")
    check("... and stays on the legs", plan.max_deviation_deg <= MAX_DEV_DEG, "%.3f deg" % plan.max_deviation_deg)

    # a right angle, as safe_move's detours turn: fold (J2, J3), then turn (J1)
    F = [0.0, -130.0, 120.0, -90.0, -90.0, 0.0]
    G_ = [60.0, -130.0, 120.0, -90.0, -90.0, 0.0]
    plan = plan_move([A, F, G_], vel, acc, jerk, safety)
    stops = plan_move([A, F], vel, acc, jerk, safety).duration + plan_move([F, G_], vel, acc, jerk, safety).duration
    t, q = plan.sample()
    within(t, q, "fold then turn: within the limits")
    check("... within the bound of its legs", plan.max_deviation_deg <= MAX_DEV_DEG, "%.3f deg" % plan.max_deviation_deg)
    check("... and no slower than stopping at the corner", plan.duration <= stops + 1e-9,
          "%.2f s vs %.2f s" % (plan.duration, stops))
    check("a repeated waypoint is no leg", abs(plan_move([A, F, F, G_], vel, acc, jerk, safety).duration - plan.duration) < 1e-9)

    # leaving a moving state
    sq, sv, sa = [10.0, -85.0, 85.0, -80.0, -90.0, 5.0], [30.0, -10.0, 20.0, 40.0, 0.0, -30.0], [50.0, 0.0, -30.0, 0.0, 20.0, 0.0]
    t, q = exit_from(sq, sv, sa, A, vel, acc, jerk, safety)
    off = max(abs(x - y) for x, y in zip(q[-1], A))
    vend = max(abs(x - y) / (t[-1] - t[-2]) for x, y in zip(q[-1], q[-2]))
    check("exit_from ends at the target at rest", off <= 1e-6 and vend < 0.1, "%.1e deg, %.3f deg/s" % (off, vend))
    check("... starting from the state", q[0] == sq)
    p = peaks(t, q)
    check("... jerk within the limits", max(p["jerk"][j] / jmax[j] for j in range(6)) <= tol)

    # the soft stop
    t, q = ramp_stop(sq, sv, sa, amax, jmax)
    vend = max(abs(x - y) / (t[-1] - t[-2]) for x, y in zip(q[-1], q[-2]))
    p = peaks(t, q)
    worst = max(max(p[k][j] / lim[j] for j in range(6)) for k, lim in (("acc", amax), ("jerk", jmax)))
    check("ramp_stop ends at zero velocity", vend < 0.1, "%.3f deg/s after %.2f s" % (vend, t[-1]))
    check("... within the acceleration and jerk limits", worst <= tol, "peak / limit %.3f" % worst)
    check("... stopping from rest takes no time", ramp_stop(A, [0.0] * 6, [0.0] * 6, amax, jmax)[0] == [0.0])

    # the show's real hub moves: the table, and the room
    print()
    rows = compare()
    print()
    for r in rows:
        if r["method"] != "Ruckig":
            continue
        if r["ok"] is not None:
            check("%s passes collision.check" % r["move"], r["ok"], "clearance %s m" % r["clearance_m"])
        within(r["t"], r["q"], "%s within the limits" % r["move"])
        check("%s within %.1f deg of its legs" % (r["move"], MAX_DEV_DEG), r["dev_deg"] <= MAX_DEV_DEG + 0.01,
              "%.3f deg" % r["dev_deg"])
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--compare" in sys.argv:
        compare(collide="all")
        sys.exit(0)
    sys.exit(self_test())
