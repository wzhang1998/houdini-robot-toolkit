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
CLIP_LOOK_S = 0.5              # the offsets get what the clip leaves of the joints' limits over this long ahead
                               # (more than the 0.2 s the offsets' jerk needs to take their acceleration back) ...
CLIP_BUDGET = 0.95             # ... of this much of each limit: a clip at 0.8 of J5's acceleration and the offsets'
                               # own share on top reached 1.07 (the real walk, 2026-09-30)
CLIP_FLOOR = 0.02              # ... and at least this much (Ruckig needs some; only a clip at the limit itself)


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
        self.vgov = list(self.vmax)    # the velocity limit as governed near a slow zone (_govern)
        self.amax = [acc[j] * share for j in GAZE_JOINTS]
        self.v_lim, self.a_lim = [vel[j] for j in GAZE_JOINTS], [acc[j] for j in GAZE_JOINTS]
        self.clip_rates = None         # the clip's peak |velocity|, |acceleration| per gaze joint, CLIP_LOOK_S ahead
        self.inp.max_velocity = list(self.vmax)
        self.inp.max_acceleration = list(self.amax)
        self.inp.max_jerk = [jerk[j] * share for j in GAZE_JOINTS]
        self.want = [0.0] * n          # the offsets aimed at
        self.aim_for = None            # (target, offsets) of the last aim
        self.lost_at = None
        self.prev_tcp = None
        self.unsafe = 0                # ticks whose commanded pose was not clear (should stay 0)
        self.shrunk = 0                # ticks whose aim was shrunk to stay clear
        self.governed = 0              # ticks with the speed limit lowered in a slow zone
        self.trouble = False           # the offsets must go back (trouble ahead, or a pose not clear): every tick,
                                       # over the slow zone's hold, until a look-ahead check finds none

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
                self.vgov = [max(x * 0.5, 0.05 * m) for x, m in zip(self.vgov, self.vmax)]
                self.governed += 1
            else:
                self.vgov = [min(x * 1.02, m) for x, m in zip(self.vgov, self.vmax)]
        self.prev_tcp = tcp

    def _clip_rates(self, clip, now):
        """The clip's peak |velocity| and |acceleration| (deg/s, deg/s^2) per gaze joint from now to
        CLIP_LOOK_S ahead: clip(t) -> its pose; differences over two ticks."""
        h = 2.0 * self.dt
        qs = [clip(now + (k - 1) * h) for k in range(int(CLIP_LOOK_S / h) + 3)]
        v, a = [], []
        for j in GAZE_JOINTS:
            col = [q[j] for q in qs]
            v.append(max(abs(col[i + 1] - col[i - 1]) / (2.0 * h) for i in range(1, len(col) - 1)))
            a.append(max(abs(col[i + 1] - 2.0 * col[i] + col[i - 1]) / h ** 2 for i in range(1, len(col) - 1)))
        return v, a

    def _limits(self):
        """The offsets' limits this tick: their share, less what the clip ahead uses of the joints' limits."""
        vmax, amax = list(self.vgov), list(self.amax)
        if self.clip_rates is not None:
            vc, ac = self.clip_rates
            vmax = [min(g, max(CLIP_FLOOR * L, CLIP_BUDGET * L - c)) for g, L, c in zip(vmax, self.v_lim, vc)]
            amax = [min(s, max(CLIP_FLOOR * L, CLIP_BUDGET * L - c)) for s, L, c in zip(amax, self.a_lim, ac)]
        return vmax, amax

    def step(self, q_base, target, now, ahead=(), clip=None):
        """The pose to send this tick: the clip's q_base with the offsets.
        ahead: the clip's poses AHEAD_S ahead (what the runner knows). clip: clip(t) -> its pose, when
        known -- the offsets then get only what it leaves of the joints' limits (_limits)."""
        self.ticks = getattr(self, "ticks", -1) + 1
        heavy = self.env is not None and self.ticks % CHECK_EVERY == 0
        if clip is not None and (self.clip_rates is None or self.ticks % CHECK_EVERY == 0):
            self.clip_rates = self._clip_rates(clip, now)
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
        if heavy:
            self.trouble = False
            if ahead and any(abs(x) > 1e-6 for x in self.offsets):
                # the offsets would turn a fast part of the clip into a slow zone, or the clip ahead with them is
                # not clear: take them back now -- over the hold (the retreat's speed governed in a slow zone)
                self.trouble = ((self.slow and any(self._near_slow(self._with(p, self.offsets))
                                                   and not self._near_slow(p) for p in ahead))
                                or not all(self._clear(self._with(p, self.offsets), None, 0.0) for p in ahead))
                self.retreats = getattr(self, "retreats", 0) + self.trouble
        if self.trouble:
            self.want = [0.0] * len(GAZE_JOINTS)
        self.inp.target_position = list(self.want)
        self.inp.target_velocity = [0.0] * len(GAZE_JOINTS)
        self.inp.max_velocity, self.inp.max_acceleration = self._limits()
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
            self.trouble = True
        self._govern(q)
        return q

    @property
    def offsets(self):
        return list(self.inp.current_position)


BAND_M = 1.2                  # only people within this of the glass count -- looked at, or waving (the user)
BAND_KEEP_M = 0.15            # ... and, once in, until this beyond it (the depth noise at the edge: no flicker)
ACT_WINDOW_S = 1.5            # activity: a hand's movement about the head over this long ...
ACT_BIN_S = 0.25
ACT_FLOOR_MPS = 0.1           # ... less this (the noise of a still hand) ...
ACT_FULL_MPS = 0.8            # ... this and more counts fully (capped: the wildest does not rule)
WAVE_MIN_S = 1.0              # waving: the hand up (above head - WAVE_UP_M) this long, swinging to and fro
WAVE_UP_M = 0.45
WAVE_AMP_M = 0.08             # ... each swing at least this far
WAVE_SWINGS = 2               # ... this many turns in the window
SCORE_WAVE, SCORE_ACTIVE, SCORE_NEAR = 2.0, 1.0, 1.5   # whom to look at: waving, then active, then near the arm
SCORE_TIE = 0.25              # scores this close to the best: as good -- they take turns
ATTEND_MIN_S = 5.0            # a person is looked at this long at least before the arm turns to another ...
ATTEND_MAX_S = 12.0           # ... and at most this long while somebody else waits (they take turns)
DWELL_S = 1.0                 # in view this long before being looked at
PASSER_MPS = 0.6              # moving faster than this (smoothed): walking by, not looked at
GROUP_M = 0.6                 # people this close to the one looked at: one group, its middle is the target
FORGET_S = 0.5                # a person not seen this long is gone ...
DROPOUT_S = 2.0               # ... but the one looked at only after this: the camera lost them, they are still
                              # there (the real Femto from above loses a walker for 0.5-3 s, 2026-09-30) -- held
                              # at their last point, and back within it, looked at again at once (no new dwell)
SPEED_WINDOW_S = 1.2          # walking speed: over this long -- frame to frame the depth noise alone reads ~1 m/s,
                              # and over 0.6 s a real person waiting (shifting, looking round) reads 0.7 (CMU 141_20)
SPEED_FIRST_S = 0.6           # ... over as long as there is, this long at least, while someone is new


def _walking_speed(hist, t):
    """How fast a person moves along the floor (m/s): the middle of their
    last few points against that of the points SPEED_WINDOW_S before (as
    far back as there is, SPEED_FIRST_S at least, while they are new) --
    frame to frame, a standing person's depth noise reads as walking. None
    until SPEED_FIRST_S is there."""
    old = [tm for tm, _ in hist if tm <= t - SPEED_FIRST_S]
    if not old:
        return None
    within = [tm for tm in old if tm >= t - SPEED_WINDOW_S]
    t_old = min(within) if within else max(old)
    a = [p for tm, p in hist if t_old - 0.1 <= tm <= t_old]
    b = [p for tm, p in hist if tm >= t - 0.1]
    ma = [sum(q[i] for q in a) / len(a) for i in range(2)]
    mb = [sum(q[i] for q in b) / len(b) for i in range(2)]
    tb = sum(tm for tm, _ in hist if tm >= t - 0.1) / len(b)
    ta = sum(tm for tm, _ in hist if t_old - 0.1 <= tm <= t_old) / len(a)
    return math.dist(ma, mb) / max(tb - ta, 1e-3)


def glass_distance(glass, p):
    """How far p is in front of the glass (glass: (normal, offset) of the room's halfspace, the guests
    on its far side): > 0 on the guests' side."""
    n, off = glass
    return off - sum(a * b for a, b in zip(n, p))


def hand_activity(samples):
    """0..1: how much a hand moves about the head -- the path of (hand - head), its mean every
    ACT_BIN_S (frame to frame, a still hand's 1-2 cm of noise reads as 0.6 m/s), per second, less
    ACT_FLOOR_MPS, over ACT_FULL_MPS; samples [(t, rel)]."""
    if len(samples) < 3 or samples[-1][0] - samples[0][0] < 0.3:
        return 0.0
    bins = {}
    for t, r in samples:
        bins.setdefault(int(t / ACT_BIN_S), []).append(r)
    means = [[sum(r[i] for r in rs) / len(rs) for i in range(3)] for _, rs in sorted(bins.items())]
    if len(means) < 2:
        return 0.0
    v = sum(math.dist(a, b) for a, b in zip(means, means[1:])) / ((len(means) - 1) * ACT_BIN_S)
    return max(0.0, min(1.0, (v - ACT_FLOOR_MPS) / (ACT_FULL_MPS - ACT_FLOOR_MPS)))


def is_waving(samples, now):
    """The hand up over the last WAVE_MIN_S and swinging to and fro (WAVE_SWINGS turns of WAVE_AMP_M
    or more, along its widest horizontal direction); samples [(t, rel)] (hand - head)."""
    last = [(t, r) for t, r in samples if t >= now - WAVE_MIN_S - 0.2]
    if len(last) < 8 or last[-1][0] - last[0][0] < WAVE_MIN_S - 0.1 or any(r[2] < -WAVE_UP_M for _, r in last):
        return False
    xs, ys = [r[0] for _, r in last], [r[1] for _, r in last]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
    h = [(x - mx) * math.cos(ang) + (y - my) * math.sin(ang) for x, y in zip(xs, ys)]
    turns, ext, up = 0, h[0], None
    for v in h[1:]:
        if up is None:
            if abs(v - ext) >= WAVE_AMP_M:
                up, ext = v > ext, v
        elif (v > ext) == up:
            ext = v
        elif abs(v - ext) >= WAVE_AMP_M:
            turns, up, ext = turns + 1, not up, v
    return turns >= WAVE_SWINGS


class Attention:
    """Whom to look at when several people are there (/track/people): only
    those in view DWELL_S, not walking by, confident, inside the zone and
    within BAND_M of the glass; scored -- waving (SCORE_WAVE), a hand's
    activity (SCORE_ACTIVE), nearness to the arm (SCORE_NEAR: the nearest 1)
    -- the best first (the user: the nearest, then those moving; waving calls
    it), those within SCORE_TIE of the best as good; one at a time, at least
    ATTEND_MIN_S each; among them someone
    not looked at yet, or after ATTEND_MAX_S whoever waited longest (they
    take turns); someone clearly better (above the tie: waving, more active,
    nearer) is turned to once the one looked at has had ATTEND_MIN_S; the
    arm does not flit; people close together are
    one group (their middle). Its target goes to TargetInput as the one person's
    would (TargetInput's gate and filter, Gaze's limits: a turn from one to
    the next is as smooth as any)."""

    def __init__(self, inside=None, min_conf=MIN_CONF, origin=(0.0, 0.0), glass=None):
        self.inside, self.min_conf, self.origin, self.glass = inside, min_conf, origin, glass
        self.tracks = {}              # pid -> {"pos", "t", "first", "last", "speed"}
        self.last_end = {}            # pid -> when the arm last turned away from them
        self.current, self.since = None, 0.0
        self.turns = 0

    def update(self, people, now, hands=()):
        """people: [(pid, x, y, z, conf, t measured)] -- one frame's detections; hands likewise, a hand
        a person (its height and swing: activity, waving)."""
        keep = lambda pid: DROPOUT_S if pid == self.current else FORGET_S          # noqa: E731
        for pid, x, y, z, conf, t in people:
            if conf < self.min_conf:
                continue
            p = (x, y, z)
            tr = self.tracks.get(pid)
            if tr is None or now - tr["last"] > keep(pid):
                tr = self.tracks[pid] = {"pos": p, "t": t, "first": now, "last": now, "speed": None, "hist": []}
            tr.update(pos=p, t=t, last=now)
            h = tr["hist"]
            h.append((t, p))
            while h and h[0][0] < t - 2.0 * SPEED_WINDOW_S:
                del h[0]
            tr["speed"] = _walking_speed(h, t)
            if self.glass is not None:
                d = glass_distance(self.glass, p)
                tr["in_band"] = -0.2 <= d <= BAND_M + (BAND_KEEP_M if tr.get("in_band") else 0.0)
        for pid, x, y, z, conf, t in hands:
            tr = self.tracks.get(pid)
            if conf < self.min_conf or tr is None:
                continue
            hs = tr.setdefault("hands", [])
            hs.append((t, (x - tr["pos"][0], y - tr["pos"][1], z - tr["pos"][2])))
            while hs and hs[0][0] < t - ACT_WINDOW_S:
                del hs[0]
        for pid in [k for k, tr in self.tracks.items() if now - tr["last"] > keep(k)]:
            del self.tracks[pid]

    def activity(self, pid, now):
        """0..1: their hand's movement (0 while they walk: arms swing then too)."""
        tr = self.tracks.get(pid)
        if tr is None or (tr["speed"] or 0.0) > PASSER_MPS:
            return 0.0
        return hand_activity([x for x in tr.get("hands", []) if x[0] >= now - ACT_WINDOW_S])

    def waving(self, pid, now):
        tr = self.tracks.get(pid)
        return tr is not None and (tr["speed"] or 0.0) <= PASSER_MPS and is_waving(tr.get("hands", []), now)

    def _eligible(self, now):
        return [pid for pid, tr in self.tracks.items()
                if now - tr["first"] >= DWELL_S and tr["speed"] is not None and tr["speed"] <= PASSER_MPS
                and (self.inside is None or self.inside(tr["pos"]))
                and (self.glass is None or tr.get("in_band"))]

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
        dist = lambda p: max(0.3, math.dist(self.tracks[p]["pos"][:2], self.origin))               # noqa: E731
        dmin = min(dist(q) for q in el) if el else 1.0
        score = lambda p: (SCORE_WAVE * self.waving(p, now) + SCORE_ACTIVE * self.activity(p, now)   # noqa: E731
                           + SCORE_NEAR * dmin / dist(p))
        top = max(score(q) for q in el) if el else 0.0
        front = [p for p in el if score(p) >= top - SCORE_TIE]
        cands = [p for p in front if cur is None or math.dist(self.tracks[p]["pos"], cur) > GROUP_M]
        fresh = [p for p in cands if p not in self.last_end]
        best = lambda ps: max(ps, key=lambda p: (p not in self.last_end, self._waited(p, now), -p))  # noqa: E731
        stint = now - self.since
        if self.current is None and cands:
            self._turn_to(best(cands), now)
        elif self.current is not None and self.current not in front and stint >= ATTEND_MIN_S and cands:
            self._turn_to(max(cands, key=score), now)             # someone clearly more: waving, active, nearer
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
    def feed(att, people_at, t0, t1, fps=30.0, hands_at=None):
        """Runs att over [t0, t1): people_at(t) -> [(pid, x, y, z, conf)], hands_at(t) likewise; the chosen
        pid per frame."""
        out = []
        for i in range(int(round((t1 - t0) * fps))):
            t = t0 + i / fps
            att.update([(pid, x, y, z, c, t) for pid, x, y, z, c in people_at(t)], t,
                       [(pid, x, y, z, c, t) for pid, x, y, z, c in (hands_at(t) if hands_at else [])])
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
    check("the one looked at leaves before their turn is up: the next one within %.1f s (held as a dropout first)"
          % (DROPOUT_S + 0.2), held and after and after[0] - 5.0 <= DROPOUT_S + 0.2, after[:1])

    def dropout(gap):
        return lambda t: [] if 5.0 <= t < 5.0 + gap else [(1, -0.2 + 0.05 * t, -1.8, 1.6, 0.9)]
    seq = feed(Attention(), dropout(1.5), 0.0, 9.0)
    during = [(pid, pt) for t, pid, pt in seq if 5.0 <= t < 6.5]
    last = next(pt for t, pid, pt in reversed(seq) if t < 5.0)
    check("the one looked at, unseen 1.5 s (the camera lost them): still looked at, at their last point",
          during and all(pid == 1 and pt == last for pid, pt in during), during[-1:])
    check("... back: looked at again at once (no new dwell)",
          all(pid == 1 for t, pid, _ in seq if 6.5 <= t < 9.0), [(round(t, 2), pid) for t, pid, _ in seq if pid != 1][-3:])
    seq = feed(Attention(), dropout(3.0), 0.0, 9.0)
    gone = [t for t, pid, _ in seq if t >= 5.0 and pid is None]
    check("... unseen %.1f s: let go (gone)" % DROPOUT_S,
          gone and 5.0 + DROPOUT_S - 0.05 <= gone[0] <= 5.0 + DROPOUT_S + 0.1, gone[:1])

    near_far = lambda t: [(1, 0.0, -2.6, 1.6, 0.9), (2, 0.3, -1.6, 1.6, 0.9)]
    seq = feed(Attention(), near_far, 0.0, 40.0)
    check("the one nearest the arm first: two people 1 m apart in distance, only the nearer is looked at",
          {pid for _, pid, _ in seq if pid is not None} == {2})
    arrive = lambda t: [(1, 0.0, -2.4, 1.6, 0.9)] + ([(2, 0.8, -1.5, 1.6, 0.9)] if t >= 3.0 else [])
    seq = feed(Attention(), arrive, 0.0, 30.0)
    runs = spans(seq)
    check("someone nearer comes: after the one looked at had %.0f s, the arm turns to them, and stays" % ATTEND_MIN_S,
          [p for p, _ in runs if p is not None] == [1, 2] and abs(runs[0][1] - ATTEND_MIN_S) < 0.2, runs)
    # waving and moving (the glass: y = -1.2, the guests at y < -1.2)
    glass = ((0.0, 1.0, 0.0), -1.2)

    def wave(pid, x, y, t, t0=0.0, t1=1e9, hz=1.2, amp=0.15):
        """A hand: waving above the head from t0 to t1, else down by the side."""
        if t0 <= t < t1:
            return (pid, x + amp * math.sin(2 * math.pi * hz * t), y, 1.75, 0.9)
        return (pid, x + 0.25, y, 0.95, 0.9)
    stand2 = lambda t: [(1, 0.0, -1.6, 1.6, 0.9), (2, 0.9, -2.1, 1.6, 0.9)]
    seq = feed(Attention(glass=glass), stand2, 0.0, 16.0,
               hands_at=lambda t: [wave(1, 0.0, -1.6, t, 1e9), wave(2, 0.9, -2.1, t, 4.0)])
    first_b = next((t for t, pid, _ in seq if pid == 2), None)
    check("the nearer one looked at; then the further one waves (from 4 s): the arm turns to the waver once the "
          "near one has had %.0f s, and stays" % ATTEND_MIN_S,
          seq[45][1] == 1 and first_b is not None and first_b <= 1.0 + ATTEND_MIN_S + 0.3
          and all(pid == 2 for t, pid, _ in seq if t > first_b), first_b)
    beyond = lambda t: [(3, 0.0, -2.6, 1.6, 0.9)]
    seq = feed(Attention(glass=glass), beyond, 0.0, 8.0, hands_at=lambda t: [wave(3, 0.0, -2.6, t)])
    check("someone waving %.1f m from the glass (beyond %.1f m): not looked at" % (1.4, BAND_M),
          all(pid is None for _, pid, _ in seq))
    rng3 = random.Random(5)
    edge = lambda t: [(1, 0.0, -2.35 + rng3.gauss(0, 0.04), 1.6, 0.9)]
    seq = feed(Attention(glass=glass), edge, 0.0, 10.0)
    runs = [r for r in spans(seq) if r[0] is not None]
    check("someone standing at the band's edge (1.15 m from the glass, 4 cm depth noise): looked at without a break",
          len(runs) == 1 and runs[0][1] > 8.5, runs)
    kid = lambda t: [(1, 0.4, -1.7, 1.6, 0.9), (4, -0.9 + 1.5 * (abs((t % 2.4) - 1.2)), -1.5, 1.2, 0.9)]
    seq = feed(Attention(glass=glass), kid, 0.0, 16.0,
               hands_at=lambda t: [wave(1, 0.4, -1.7, t, 1e9), wave(4, -0.9 + 1.5 * abs((t % 2.4) - 1.2), -1.5, t,
                                                                  hz=2.0, amp=0.25)])
    check("a child running to and fro at the glass, arms flying: not followed (walking, not waving at it)",
          sum(pid == 4 for _, pid, _ in seq) < 0.1 * len(seq), sum(pid == 4 for _, pid, _ in seq))
    att = Attention(glass=glass)
    rng2 = random.Random(3)
    noisy_hand = lambda t: [(1, 0.25 + rng2.gauss(0, 0.01), -1.6 + rng2.gauss(0, 0.02), 0.95 + rng2.gauss(0, 0.01), 0.9)]
    feed(att, lambda t: [(1, 0.0, -1.6, 1.6, 0.9)], 0.0, 4.0, hands_at=noisy_hand)
    check("a still hand's noise (1-2 cm a frame) is not activity", att.activity(1, 4.0) < 0.1,
          round(att.activity(1, 4.0), 3))
    check("... nor waving", not att.waving(1, 4.0))
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
    def busy(t):                    # a clip swinging J1 and J5 at 0.8 of their acceleration limits (0.5 Hz)
        w = 2 * math.pi * 0.5
        q = list(hub)
        for j in GAZE_JOINTS:
            q[j] += 0.8 * amax[j] / w ** 2 * math.sin(w * t + j)
        return q
    gz = Gaze(anchor=hub, dt=0.008)
    people = ((-0.9, -1.75, 1.6), (0.6, -1.9, 1.7))
    qs = [gz.step(busy(k * gz.dt), people[(k // 150) % 2], k * gz.dt, clip=busy) for k in range(1500)]
    arm = max(max(abs(c - 2 * b + a) / gz.dt ** 2 / amax[j] for a, b, c in zip(col, col[1:], col[2:]))
              for j, col in ((j, [q[j] for q in qs]) for j in GAZE_JOINTS))
    armv = max(max(abs(b - a) / gz.dt / vmax[j] for a, b in zip(col, col[1:]))
               for j, col in ((j, [q[j] for q in qs]) for j in GAZE_JOINTS))
    check("a clip already at 0.8 of the acceleration limits, the target jumping every 1.2 s: the arm (clip + "
          "offsets) within the limits -- the offsets get what the clip leaves", arm <= 1.0 and armv <= 1.0,
          "acceleration %.2f, velocity %.2f of the limits" % (arm, armv))
    check("... and the offsets still turn to them", max(abs(x) for x in gz.offsets) > 2.0,
          [round(x, 1) for x in gz.offsets])
    gz = Gaze(anchor=hub, env=env)
    qs = [gz.step(hub, person if k < 300 else None, k * gz.dt) for k in range(900)]
    held = qs[300 + int(0.5 / gz.dt)]
    check("lost: the offsets held %.1f s, then back to the clip (zero), smoothly" % HOLD_S,
          max(abs(a - b) for a, b in zip(held, qs[299])) < 0.05 and max(abs(x) for x in gz.offsets) < 1e-6
          and derivs(qs, gz.dt)[1] <= SHARE * 600 * 1.01, [round(x, 2) for x in gz.offsets])
    check("every commanded pose clear of the room", gz.unsafe == 0)
    gz = Gaze(anchor=hub, env=env)
    for k in range(400):                                   # turned to the person
        gz.step(hub, person, k * gz.dt, [hub] * len(AHEAD_S))
    turned = max(abs(x) for x in gz.offsets)
    gz._near_slow = lambda q: True                         # near a slow zone (the offsets would hold) ...
    real_clear = gz._clear
    gz._clear = lambda q, env=None, self_room=0.0: (env is not None or self_room > 0.0) and real_clear(q, env, self_room)
    for k in range(400, 700):                              # ... and the clip ahead, with them, not clear
        gz.step(hub, person, k * gz.dt, [hub] * len(AHEAD_S))
    check("trouble ahead near a slow zone: the offsets still go back (clearance before the hold; the speed "
          "governed)", turned > 3.0 and max(abs(x) for x in gz.offsets) < 0.2 * turned,
          (round(turned, 1), [round(x, 2) for x in gz.offsets]))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
