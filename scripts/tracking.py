"""The tracking layer: a person followed on top of the clips, safely,
whatever the tracking data does (Stage 11; docs/next_steps_2026-09-28.md).

Mode A, gaze: while a clip plays, small J1 and J5 offsets turn the tool
(the LED strip's face) towards a person's head. Per tick (125 Hz):

    TargetInput   /track/target in: drop low confidence; drop a point that
                  would mean moving faster than a person can (a depth
                  outlier, a one-frame glitch) -- unless the next frames
                  agree with it (the detector moved to another person: taken
                  after SWITCH_FRAMES); a One Euro filter on what is taken;
                  lost after LOST_AFTER_S without a point, or on /track/lost
    Gaze          the offsets that would aim the tool at the target, from the
                  clip's pose this tick, bounded (MAX_OFFSET_DEG); lost: the
                  last ones held HOLD_S, then zero (back to the clip)
    safety        the target offsets shrunk until the pose they give is clear
                  of the room (collision.check with the clip's margins)
    Ruckig        the offsets move to their target within SHARE of the
                  joints' velocity, acceleration and jerk limits -- a jump in
                  the data becomes a smooth, bounded turn; lost, the way back
                  takes ~1.5 s at these limits

The clip's own motion is untouched: the offsets are added to it.

    uv run scripts/tracking.py        self-test
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

MIN_CONF = 0.5
MAX_SPEED_MPS = 3.0            # faster than a person's head: not taken (a glitch) ...
SWITCH_FRAMES = 3              # ... unless this many frames in a row agree (another person)
LOST_AFTER_S = 0.35
ONE_EURO = (0.8, 0.6, 1.0)     # min cutoff Hz, beta, derivative cutoff Hz (Casiez et al. 2012)
GAZE_JOINTS = (0, 4)           # J1 turns towards the person, J5 tilts the tool up / down
MAX_OFFSET_DEG = (20.0, 10.0)
SHARE = 0.3                    # of the joints' velocity, acceleration and jerk limits
HOLD_S = 0.7
SHRINK = (1.0, 0.75, 0.5, 0.25, 0.0)
AIM_ROOM_M = 0.02              # the aimed pose keeps this much more than the room's margins ...
AIM_SELF_M = 0.01              # ... and this much between the links
AHEAD_S = (0.2, 0.4, 0.6, 1.0, 1.4)   # the clip's poses this far ahead are checked with the offsets too:
                                     # taking back a full J1 offset at SHARE takes ~1.2 s (a crowd's edge)
SLOW_KEEP_M = 0.2              # J1's offset towards a slow zone stops where the anchor's tool point is this far from it
SLOW_NEAR_M = 0.15             # the tool point this near a slow zone, now or AHEAD_S ahead: the offsets hold still
CHECK_EVERY = 4                # the aim and look-ahead checks every 4th tick (~31 Hz, the data's rate)


class OneEuro:
    """The One Euro filter (Casiez, Roussel, Vogel 2012), per coordinate:
    little smoothing when the value moves fast (little lag), much when it
    is still (little jitter)."""

    def __init__(self, min_cutoff=ONE_EURO[0], beta=ONE_EURO[1], d_cutoff=ONE_EURO[2]):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x = self.dx = self.t = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, t, x):
        if self.t is None or t <= self.t:
            self.x, self.dx, self.t = list(x), [0.0] * len(x), t
            return list(self.x)
        dt = t - self.t
        dx = [(a - b) / dt for a, b in zip(x, self.x)]
        ad = self._alpha(self.d_cutoff, dt)
        self.dx = [ad * a + (1 - ad) * b for a, b in zip(dx, self.dx)]
        speed = math.sqrt(sum(v * v for v in self.dx))
        a = self._alpha(self.min_cutoff + self.beta * speed, dt)
        self.x = [a * p + (1 - a) * q for p, q in zip(x, self.x)]
        self.t = t
        return list(self.x)


class TargetInput:
    """Tracking messages in, one filtered target out (or None: nobody)."""

    def __init__(self, min_conf=MIN_CONF, max_speed=MAX_SPEED_MPS, switch_frames=SWITCH_FRAMES,
                 lost_after=LOST_AFTER_S):
        self.min_conf, self.max_speed, self.switch_frames, self.lost_after = min_conf, max_speed, switch_frames, lost_after
        self.filter = OneEuro()
        self.last = None               # (t measured, raw point) last taken
        self.pos = None                # filtered
        self.seen = None               # arrival time of the last taken point
        self.cand = []                 # points not taken, agreeing with each other (another person?)
        self.taken = self.dropped = self.switches = 0

    def target(self, x, y, z, conf, t, pid=0, now=None):
        now = t if now is None else now
        p = (x, y, z)
        if conf < self.min_conf:
            self.dropped += 1
            return
        if self.last is not None and self.seen is not None and now - self.seen < self.lost_after:
            dt = max(t - self.last[0], 1e-3)
            if math.dist(p, self.last[1]) / dt > self.max_speed:
                ok = self.cand and math.dist(p, self.cand[-1][1]) / max(t - self.cand[-1][0], 1e-3) <= self.max_speed
                self.cand = self.cand + [(t, p)] if ok else [(t, p)]
                if len(self.cand) < self.switch_frames:
                    self.dropped += 1
                    return
                self.filter = OneEuro()                  # another person: start again from them
                for tc, pc in self.cand[:-1]:
                    self.filter(tc, pc)
                self.switches += 1
        self.cand = []
        self.last, self.seen = (t, p), now
        self.pos = self.filter(t, p)
        self.taken += 1

    def lost(self):
        self.seen = None

    def now(self, now):
        """The target at `now`, or None when lost."""
        if self.seen is None or now - self.seen > self.lost_after:
            if self.seen is not None:
                self.filter = OneEuro()
            self.seen, self.cand = None, []
            return None
        return self.pos


def _tool(rig, q):
    R, tcp, _ = rig.tool(q)
    return tcp, (R[0][2], R[1][2], R[2][2])


def _aim_error(rig, q, target):
    """The tool axis at q less the unit direction from its tool point to target."""
    tcp, d = _tool(rig, q)
    v = [a - b for a, b in zip(target, tcp)]
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [a - b / n for a, b in zip(d, v)]


def aim_offsets(rig, q, target, iterations=4):
    """The gaze joints' offsets (deg) that turn the tool axis at pose q
    towards target: Gauss-Newton on the aim error, the Jacobian by finite
    differences at each step (the wrist's own twist decides which joint
    tilts and which turns, so no joint is assumed to do either)."""
    off = [0.0] * len(GAZE_JOINTS)
    h = 0.5
    for _ in range(iterations):
        qo = list(q)
        for k, j in enumerate(GAZE_JOINTS):
            qo[j] += off[k]
        e = _aim_error(rig, qo, target)
        cols = []
        for k, j in enumerate(GAZE_JOINTS):
            qh = list(qo)
            qh[j] += h
            cols.append([(a - b) / h for a, b in zip(_aim_error(rig, qh, target), e)])
        # (J^T J + lambda I) delta = -J^T e, 2 x 2
        a11 = sum(x * x for x in cols[0]) + 1e-6
        a22 = sum(x * x for x in cols[1]) + 1e-6
        a12 = sum(x * y for x, y in zip(cols[0], cols[1]))
        b1 = -sum(x * y for x, y in zip(cols[0], e))
        b2 = -sum(x * y for x, y in zip(cols[1], e))
        det = a11 * a22 - a12 * a12
        if abs(det) < 1e-12:
            break
        off[0] += (a22 * b1 - a12 * b2) / det
        off[1] += (a11 * b2 - a12 * b1) / det
    return off


class Gaze:
    """Mode A: the gaze offsets on top of a clip, tick by tick.

    The offsets are aimed from the hub's own pose (anchor), not the clip's
    of the moment: the whole performance turns towards the person and the
    clip's gestures go on around that direction -- aimed from the moving
    clip, the offsets chased its every look at full speed and made the gaze
    worse (track_eval, 2026-09-29). Safety, each tick:
    - the aimed offsets are shrunk until the clip's pose with them keeps
      AIM_ROOM_M more than the room's margins and AIM_SELF_M between links
      (the offsets lag behind the aim: that room is for the way there),
      now and at the clip's poses AHEAD_S ahead (the clip is known: a
      gesture heading for the strip is seen before it gets there);
    - when the offsets as they are would not be clear at a pose ahead, the
      aim goes to zero now: 0.6 s is room enough to take them back;
    - the pose sent is checked; when it is not clear the aim goes to zero
      at once (and the tick is counted: it should never happen);
    - near a slow zone (the tool point within SLOW_NEAR_M of it, now or at
      a pose ahead) the offsets hold still: the arm does not turn to anyone
      there, so the tool keeps the clip's own (checked) speed -- a turn from
      one person to the next in the zone took it to twice the zone's limit
      (track_eval crowd, 2026-09-29); and the speed limit still comes down
      while the tool point is near the zone's speed."""

    def __init__(self, anchor=None, dt=0.008, share=SHARE, max_offset=MAX_OFFSET_DEG, hold_s=HOLD_S, env=None,
                 model=None, rig=None):
        import collision as C
        import gestures as G
        import robot_profile as RP
        import transitions as T
        from ruckig import InputParameter, OutputParameter, Ruckig
        self.C = C
        self.rig = rig or G.Rig()
        self.model = model or C.load_model("fr20")
        self.env = env
        self.aim_env = None if env is None else dict(env, objects=[
            dict(o, margin_m=o.get("margin_m", env.get("margin_m", 0.05)) + AIM_ROOM_M) if o["role"] == "obstacle"
            else o for o in env["objects"]])
        self.slow = [o for o in (env or {}).get("objects", []) if o["role"] == "slow"]
        self.bounds = [(-m, m) for m in max_offset]
        if self.slow and anchor is not None:
            self.bounds[0] = self._j1_bounds(anchor, max_offset[0])
        self.anchor = anchor
        prof = RP.load("fr20")
        vel, acc = RP.velocity_limits(prof), RP.acceleration_limits(prof)
        jerk = T.default_jerk(acc)
        self.lim = RP.motion_limits(prof)
        self.max_offset, self.hold_s, self.dt = max_offset, hold_s, dt
        n = len(GAZE_JOINTS)
        self.otg, self.inp, self.out = Ruckig(n, dt), InputParameter(n), OutputParameter(n)
        self.inp.current_position = [0.0] * n
        self.inp.current_velocity = [0.0] * n
        self.inp.current_acceleration = [0.0] * n
        self.vmax = [vel[j] * share for j in GAZE_JOINTS]
        self.inp.max_velocity = list(self.vmax)
        self.inp.max_acceleration = [acc[j] * share for j in GAZE_JOINTS]
        self.inp.max_jerk = [jerk[j] * share for j in GAZE_JOINTS]
        self.want = [0.0] * n          # the offsets aimed at
        self.aim_for = None            # (target, offsets) of the last aim
        self.lost_at = None
        self.prev_tcp = None
        self.unsafe = 0                # ticks whose commanded pose was not clear (should stay 0)
        self.shrunk = 0                # ticks whose aim was shrunk to stay clear
        self.governed = 0              # ticks with the speed limit lowered in a slow zone

    def _clear(self, q, env=None, self_room=0.0):
        if self.env is None:
            return True
        if not all(lo <= x <= hi for x, (lo, hi) in zip(q, self.lim)):
            return False
        rep = self.C.check(self.model, env or self.env, [0.0], [q])
        return rep["ok"] and (rep["min_self_clearance_m"] is None or rep["min_self_clearance_m"] >= self_room)

    def _with(self, q, off):
        q = list(q)
        for k, j in enumerate(GAZE_JOINTS):
            q[j] += off[k]
        return q

    def _aim(self, q_base, target):
        anchor = self.anchor or q_base
        if self.aim_for is None or math.dist(self.aim_for[0], target) > 0.01:     # aimed again as the person moves
            off = aim_offsets(self.rig, anchor, target)
            self.aim_for = (tuple(target), [max(lo, min(hi, x)) for x, (lo, hi) in zip(off, self.bounds)])
        return self.aim_for[1]

    def _j1_bounds(self, anchor, m):
        """J1's offset range: up to m either way, but towards a slow zone only as far as keeps the
        anchor's tool point SLOW_KEEP_M from it -- a J1 offset turns the whole clip, and a fast
        part of it turned into the zone cannot be slowed (the crowd, 2026-09-29)."""
        out = []
        for sign in (-1.0, 1.0):
            k = 0.0
            while k < m:
                q = list(anchor)
                q[GAZE_JOINTS[0]] += sign * (k + 1.0)
                tcp = self.rig.tool(q)[1]
                if any(self.C.sdf(o, tcp) < SLOW_KEEP_M for o in self.slow):
                    break
                k += 1.0
            out.append(sign * k)
        return tuple(out)

    def _near_slow(self, q):
        tcp = self.rig.tool(q)[1]
        return any(self.C.sdf(o, tcp) < SLOW_NEAR_M for o in self.slow)

    def _govern(self, q):
        """The offsets' speed limit down while the tool point is fast in a slow zone."""
        tcp = self.rig.tool(q)[1]
        if self.prev_tcp is not None and self.slow:
            v = math.dist(tcp, self.prev_tcp) / self.dt
            near = [o for o in self.slow if self.C.sdf(o, tcp) < 0.1]
            if near and v > 0.8 * min(o["tcp_speed_mps"] for o in near):
                self.inp.max_velocity = [max(x * 0.5, 0.05 * m) for x, m in zip(self.inp.max_velocity, self.vmax)]
                self.governed += 1
            else:
                self.inp.max_velocity = [min(x * 1.02, m) for x, m in zip(self.inp.max_velocity, self.vmax)]
        self.prev_tcp = tcp

    def step(self, q_base, target, now, ahead=()):
        """The pose to send this tick: the clip's q_base with the offsets.
        ahead: the clip's poses AHEAD_S ahead (what the runner knows)."""
        self.ticks = getattr(self, "ticks", -1) + 1
        heavy = self.env is not None and self.ticks % CHECK_EVERY == 0
        poses = [q_base] + list(ahead)
        if target is not None:
            self.lost_at = None
            want = self._aim(q_base, target)
            if heavy or self.env is None:
                for f in SHRINK:                           # the aim shrunk until its poses keep room
                    w = [x * f for x in want]
                    if f == 0.0 or all(self._clear(self._with(p, w), self.aim_env, AIM_SELF_M) for p in poses):
                        self.shrunk += f < 1.0
                        self.want = w
                        break
        else:
            self.aim_for = None
            if self.lost_at is None:
                self.lost_at = now
            if now - self.lost_at >= self.hold_s:
                self.want = [0.0] * len(GAZE_JOINTS)       # back to the clip
        if self.slow and any(self._near_slow(p) for p in poses):
            self.want = list(self.offsets)             # no turning in or on the way into a slow zone
            self.held_slow = getattr(self, "held_slow", 0) + 1
        if heavy and ahead and any(abs(x) > 1e-6 for x in self.offsets):
            if self.slow and any(self._near_slow(self._with(p, self.offsets)) and not self._near_slow(p) for p in ahead):
                self.want = [0.0] * len(GAZE_JOINTS)       # the offsets would turn a fast part of the clip into
                self.retreats = getattr(self, "retreats", 0) + 1   # a slow zone: take them back first
            if not all(self._clear(self._with(p, self.offsets), None, 0.0) for p in ahead):
                self.want = [0.0] * len(GAZE_JOINTS)       # trouble ahead: take them back now
                self.retreats = getattr(self, "retreats", 0) + 1
        self.inp.target_position = list(self.want)
        self.inp.target_velocity = [0.0] * len(GAZE_JOINTS)
        if all(abs(t - c) < 1e-6 and abs(v) < 1e-6 and abs(a) < 1e-6 for t, c, v, a in
               zip(self.want, self.inp.current_position, self.inp.current_velocity, self.inp.current_acceleration)):
            # there and still: snapped (Ruckig finds no step in a round-off, 2026-09-29)
            self.inp.current_position = list(self.want)
            self.inp.current_velocity = [0.0] * len(GAZE_JOINTS)
            self.inp.current_acceleration = [0.0] * len(GAZE_JOINTS)
            pos = list(self.want)
        else:
            self.otg.update(self.inp, self.out)
            self.out.pass_to_input(self.inp)
            pos = self.out.new_position
        q = self._with(q_base, pos)
        if self.env is not None and not self._clear(q):
            self.unsafe += 1
            self.want = [0.0] * len(GAZE_JOINTS)           # back towards the clip at once
        self._govern(q)
        return q

    @property
    def offsets(self):
        return list(self.inp.current_position)


NEAR_TIE_M = 0.3              # people this much further from the arm than the nearest are as near: they take turns
ATTEND_MIN_S = 5.0            # a person is looked at this long at least before the arm turns to another ...
ATTEND_MAX_S = 12.0           # ... and at most this long while somebody else waits (they take turns)
DWELL_S = 1.0                 # in view this long before being looked at
PASSER_MPS = 0.6              # moving faster than this (smoothed): walking by, not looked at
GROUP_M = 0.6                 # people this close to the one looked at: one group, its middle is the target
FORGET_S = 0.5                # a person not seen this long is gone
SPEED_WINDOW_S = 0.6          # walking speed: over this long (frame to frame, the depth noise alone reads ~1 m/s)


def _walking_speed(hist, t):
    """How fast a person moves along the floor (m/s): the middle of their
    last few points against that of the points SPEED_WINDOW_S before --
    frame to frame, a standing person's depth noise reads as walking. None
    until the window is there."""
    old = [p for tm, p in hist if tm <= t - SPEED_WINDOW_S]
    if not old:
        return None
    t_old = max(tm for tm, _ in hist if tm <= t - SPEED_WINDOW_S)
    a = [p for tm, p in hist if t_old - 0.1 <= tm <= t_old]
    b = [p for tm, p in hist if tm >= t - 0.1]
    ma = [sum(q[i] for q in a) / len(a) for i in range(2)]
    mb = [sum(q[i] for q in b) / len(b) for i in range(2)]
    tb = sum(tm for tm, _ in hist if tm >= t - 0.1) / len(b)
    ta = sum(tm for tm, _ in hist if t_old - 0.1 <= tm <= t_old) / len(a)
    return math.dist(ma, mb) / max(tb - ta, 1e-3)


class Attention:
    """Whom to look at when several people are there (/track/people): only
    those in view DWELL_S, not walking by, confident, inside the zone; the
    ones nearest the arm first (the user) -- those within NEAR_TIE_M of the
    nearest -- one at a time, at least ATTEND_MIN_S each; among them someone
    not looked at yet, or after ATTEND_MAX_S whoever waited longest (they
    take turns); someone clearly nearer is turned to once the one looked at
    has had ATTEND_MIN_S; the arm does not flit; people close together are
    one group (their middle). Its target goes to TargetInput as the one person's
    would (TargetInput's gate and filter, Gaze's limits: a turn from one to
    the next is as smooth as any)."""

    def __init__(self, inside=None, min_conf=MIN_CONF, origin=(0.0, 0.0)):
        self.inside, self.min_conf, self.origin = inside, min_conf, origin
        self.tracks = {}              # pid -> {"pos", "t", "first", "last", "speed"}
        self.last_end = {}            # pid -> when the arm last turned away from them
        self.current, self.since = None, 0.0
        self.turns = 0

    def update(self, people, now):
        """people: [(pid, x, y, z, conf, t measured)] -- one frame's detections."""
        for pid, x, y, z, conf, t in people:
            if conf < self.min_conf:
                continue
            p = (x, y, z)
            tr = self.tracks.get(pid)
            if tr is None or now - tr["last"] > FORGET_S:
                tr = self.tracks[pid] = {"pos": p, "t": t, "first": now, "last": now, "speed": None, "hist": []}
            tr.update(pos=p, t=t, last=now)
            h = tr["hist"]
            h.append((t, p))
            while h and h[0][0] < t - 2.0 * SPEED_WINDOW_S:
                del h[0]
            tr["speed"] = _walking_speed(h, t)
        for pid in [k for k, tr in self.tracks.items() if now - tr["last"] > FORGET_S]:
            del self.tracks[pid]

    def _eligible(self, now):
        return [pid for pid, tr in self.tracks.items()
                if now - tr["first"] >= DWELL_S and tr["speed"] is not None and tr["speed"] <= PASSER_MPS
                and (self.inside is None or self.inside(tr["pos"]))]

    def _waited(self, pid, now):
        return now - max(self.tracks[pid]["first"], self.last_end.get(pid, -1e9))

    def _turn_to(self, pid, now):
        if self.current is not None:
            self.last_end[self.current] = now
        self.current, self.since = pid, now
        self.turns += 1

    def choose(self, now):
        """(the point to look at, whose) now, or None: nobody to look at."""
        el = self._eligible(now)
        if self.current not in el:
            if self.current is not None:
                self.last_end[self.current] = now
            self.current = None
        cur = self.tracks[self.current]["pos"] if self.current is not None else None
        dist = lambda p: math.dist(self.tracks[p]["pos"][:2], self.origin)                         # noqa: E731
        front = [p for p in el if dist(p) <= min(dist(q) for q in el) + NEAR_TIE_M] if el else []
        cands = [p for p in front if cur is None or math.dist(self.tracks[p]["pos"], cur) > GROUP_M]
        fresh = [p for p in cands if p not in self.last_end]
        best = lambda ps: max(ps, key=lambda p: (p not in self.last_end, self._waited(p, now), -p))  # noqa: E731
        stint = now - self.since
        if self.current is None and cands:
            self._turn_to(best(cands), now)
        elif self.current is not None and self.current not in front and stint >= ATTEND_MIN_S and cands:
            self._turn_to(min(cands, key=dist), now)              # someone clearly nearer the arm
        elif self.current is not None and stint >= ATTEND_MAX_S and cands:
            self._turn_to(best(cands), now)
        elif self.current is not None and stint >= ATTEND_MIN_S and fresh:
            self._turn_to(best(fresh), now)
        if self.current is None:
            return None
        c = self.tracks[self.current]["pos"]
        grp = [self.tracks[p]["pos"] for p in el if math.dist(self.tracks[p]["pos"], c) <= GROUP_M]
        return tuple(sum(q[i] for q in grp) / len(grp) for i in range(3)), self.current


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    import random
    f = OneEuro()
    rng = random.Random(1)
    raw = [(i / 30.0, [1.0 + rng.gauss(0, 0.02)]) for i in range(90)]
    out = [f(t, x)[0] for t, x in raw]
    jit_in = max(x[0] for _, x in raw[45:]) - min(x[0] for _, x in raw[45:])
    jit_out = max(out[45:]) - min(out[45:])
    check("One Euro: a still target's jitter cut to a quarter or less", jit_out < jit_in / 4, "%.3f -> %.3f m" % (jit_in, jit_out))
    f = OneEuro()
    lag = max(abs(f(i / 30.0, [1.0 * i / 30.0])[0] - i / 30.0) for i in range(60))
    check("... and a target moving 1 m/s followed within 10 cm", lag < 0.1, "%.3f m" % lag)

    ti = TargetInput()
    for i in range(30):
        ti.target(0.0, -1.8, 1.6, 0.9, i / 30.0)
    ti.target(0.0, -1.8, 1.6, 0.3, 1.0)
    check("low confidence is not taken", ti.dropped == 1)
    ti.target(0.0, 1.2, 1.6, 0.9, 1.033)
    check("a one-frame outlier 3 m off is not taken", ti.dropped == 2 and abs(ti.now(1.04)[1] + 1.8) < 0.05, ti.now(1.04))
    for k in range(SWITCH_FRAMES):
        ti.target(1.3, -1.8, 1.7, 0.9, 1.066 + k / 30.0)
    check("another person, seen %d frames in a row: taken, the filter from them" % SWITCH_FRAMES,
          ti.switches == 1 and abs(ti.now(1.2)[0] - 1.3) < 0.05, ti.now(1.2))
    check("lost when no point came for %.2f s" % LOST_AFTER_S, ti.now(1.2 + 0.066 + LOST_AFTER_S + 0.01) is None)
    ti.target(1.3, -1.8, 1.7, 0.9, 2.0)
    ti.lost()
    check("/track/lost: lost at once", ti.now(2.01) is None)

    # attention: whom to look at when several are there
    def feed(att, people_at, t0, t1, fps=30.0):
        """Runs att over [t0, t1): people_at(t) -> [(pid, x, y, z, conf)]; the chosen pid per frame."""
        out = []
        for i in range(int(round((t1 - t0) * fps))):
            t = t0 + i / fps
            att.update([(pid, x, y, z, c, t) for pid, x, y, z, c in people_at(t)], t)
            ch = att.choose(t)
            out.append((t, ch[1] if ch else None, ch[0] if ch else None))
        return out

    def spans(seq):
        runs, cur, start = [], None, 0.0
        for t, pid, _ in seq:
            if pid != cur:
                if cur is not None:
                    runs.append((cur, t - start))
                cur, start = pid, t
        if cur is not None:
            runs.append((cur, seq[-1][0] - start))
        return runs

    two = lambda t: [(1, -0.5, -1.8, 1.6, 0.9), (2, 0.5, -1.8, 1.6, 0.9)]
    seq = feed(Attention(), two, 0.0, 40.0)
    runs = spans(seq)
    inner = [d for pid, d in runs[1:-1] if pid is not None]
    check("two people standing: each looked at in turn, never less than %.0f s at a time" % ATTEND_MIN_S,
          {p for p, _ in runs} >= {1, 2} and inner and min(inner) >= ATTEND_MIN_S - 0.05, runs)
    check("... nobody before they have been there %.1f s" % DWELL_S,
          all(pid is None for t, pid, _ in seq if t < DWELL_S - 0.05) and seq[int(DWELL_S * 30) + 3][1] is not None)

    def passer(t):
        out = [(1, 0.2, -1.8, 1.6, 0.9)]
        if 4.0 <= t < 6.5:
            out.append((2, -1.0 + 1.2 * (t - 4.0), -1.8, 1.65, 0.9))
        return out
    seq = feed(Attention(), passer, 0.0, 12.0)
    check("someone walking by at 1.2 m/s is never looked at", all(pid != 2 for _, pid, _ in seq))

    def leaver(t):
        out = [(2, 0.6, -1.8, 1.6, 0.9)]
        if t < 5.0:
            out.append((1, -0.6, -1.8, 1.6, 0.9))
        return out
    att = Attention()
    feed(att, leaver, 0.0, 2.0)
    att.current, att.since = 1, 2.0                # looking at 1 (for 3 s, short of the minimum) when they leave
    seq = feed(att, leaver, 2.0, 8.0)
    held = all(pid == 1 for t, pid, _ in seq if t < 5.0)
    after = [t for t, pid, _ in seq if t >= 5.0 and pid == 2]
    check("the one looked at leaves before their turn is up: the next one within %.1f s" % (FORGET_S + 0.2),
          held and after and after[0] - 5.0 <= FORGET_S + 0.2, after[:1])

    near_far = lambda t: [(1, 0.0, -2.6, 1.6, 0.9), (2, 0.3, -1.6, 1.6, 0.9)]
    seq = feed(Attention(), near_far, 0.0, 40.0)
    check("the one nearest the arm first: two people 1 m apart in distance, only the nearer is looked at",
          {pid for _, pid, _ in seq if pid is not None} == {2})
    arrive = lambda t: [(1, 0.0, -2.4, 1.6, 0.9)] + ([(2, 0.8, -1.5, 1.6, 0.9)] if t >= 3.0 else [])
    seq = feed(Attention(), arrive, 0.0, 30.0)
    runs = spans(seq)
    check("someone nearer comes: after the one looked at had %.0f s, the arm turns to them, and stays" % ATTEND_MIN_S,
          [p for p, _ in runs if p is not None] == [1, 2] and abs(runs[0][1] - ATTEND_MIN_S) < 0.2, runs)
    group = lambda t: [(1, 0.0, -1.8, 1.6, 0.9), (2, 0.3, -1.8, 1.62, 0.9), (3, 0.15, -1.9, 1.55, 0.9)]
    seq = feed(Attention(), group, 0.0, 20.0)
    pts = [p for _, pid, p in seq if p is not None]
    check("three people within %.1f m: looked at as one, their middle, never jumping between them" % GROUP_M,
          pts and max(math.dist(p, (0.15, -1.833, 1.59)) for p in pts) < 0.02, pts[-1] if pts else None)
    low = lambda t: [(1, 0.0, -1.8, 1.6, 0.3)]
    check("a low-confidence person is not looked at", all(pid is None for _, pid, _ in feed(Attention(), low, 0.0, 5.0)))

    import show as S
    import json
    import collision as C
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    g = S.Graph.load(S.compiled_path(os.path.join(ROOT, "shows", "party.json")))
    hub = g.hubs["greet"]
    gz = Gaze(anchor=hub, env=env)
    person = (-0.9, -1.75, 1.6)
    tcp, d = _tool(gz.rig, hub)
    def angle(q, p):
        t_, d_ = _tool(gz.rig, q)
        v = [a - b for a, b in zip(p, t_)]
        n = math.sqrt(sum(x * x for x in v))
        return math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b / n for a, b in zip(d_, v))))))
    before = angle(hub, person)
    qs = [gz.step(hub, person, k * gz.dt) for k in range(400)]
    after = angle(qs[-1], person)
    check("aimed at a person from the greet hub: the tool turns towards them", after < before / 2 or after < 3.0,
          "%.1f -> %.1f deg, offsets %s" % (before, after, [round(x, 1) for x in gz.offsets]))

    def derivs(qs, dt):
        v = [[(b - a) / dt for a, b in zip(x, y)] for x, y in zip(qs, qs[1:])]
        a = [[(b - c) / dt for c, b in zip(x, y)] for x, y in zip(v, v[1:])]
        return (max(abs(x) for r in v for x in r), max(abs(x) for r in a for x in r))
    import robot_profile as RP
    prof = RP.load("fr20")
    vmax, amax = RP.velocity_limits(prof), RP.acceleration_limits(prof)
    gz = Gaze(anchor=hub, env=env)
    qs = [gz.step(hub, (-0.9, -1.75, 1.6) if k < 200 else (0.6, -1.9, 1.7), k * gz.dt) for k in range(600)]
    ratio = max(max(abs(b - a) / gz.dt / (SHARE * vmax[j]) for a, b in zip(col, col[1:])) for j, col in
                ((j, [q[j] for q in qs]) for j in GAZE_JOINTS))
    racc = max(max(abs(c - 2 * b + a) / gz.dt ** 2 / (SHARE * amax[j]) for a, b, c in zip(col, col[1:], col[2:]))
               for j, col in ((j, [q[j] for q in qs]) for j in GAZE_JOINTS))
    check("a jump of the target 1.5 m: each joint's turn within %.0f %% of its velocity and acceleration limits"
          % (100 * SHARE), ratio <= 1.01 and racc <= 1.02, "velocity %.2f, acceleration %.2f of the share" % (ratio, racc))
    gz = Gaze(anchor=hub, env=env)
    qs = [gz.step(hub, person if k < 300 else None, k * gz.dt) for k in range(900)]
    held = qs[300 + int(0.5 / gz.dt)]
    check("lost: the offsets held %.1f s, then back to the clip (zero), smoothly" % HOLD_S,
          max(abs(a - b) for a, b in zip(held, qs[299])) < 0.05 and max(abs(x) for x in gz.offsets) < 1e-6
          and derivs(qs, gz.dt)[1] <= SHARE * 600 * 1.01, [round(x, 2) for x in gz.offsets])
    check("every commanded pose clear of the room", gz.unsafe == 0)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
