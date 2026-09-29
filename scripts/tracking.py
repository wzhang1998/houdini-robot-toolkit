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
    """Mode A: the gaze offsets on top of a clip, tick by tick."""

    def __init__(self, dt=0.008, share=SHARE, max_offset=MAX_OFFSET_DEG, hold_s=HOLD_S, env=None, model=None, rig=None):
        import collision as C
        import gestures as G
        import robot_profile as RP
        import transitions as T
        from ruckig import InputParameter, OutputParameter, Ruckig
        self.C = C
        self.rig = rig or G.Rig()
        self.model = model or C.load_model("fr20")
        self.env = env
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
        self.inp.max_velocity = [vel[j] * share for j in GAZE_JOINTS]
        self.inp.max_acceleration = [acc[j] * share for j in GAZE_JOINTS]
        self.inp.max_jerk = [jerk[j] * share for j in GAZE_JOINTS]
        self.want = [0.0] * n          # the offsets aimed at
        self.lost_at = None
        self.unsafe = 0                # ticks whose commanded pose was not clear (should stay 0)
        self.shrunk = 0                # ticks whose aim was shrunk to stay clear

    def _clear(self, q):
        if self.env is None:
            return True
        if not all(lo <= x <= hi for x, (lo, hi) in zip(q, self.lim)):
            return False
        return self.C.check(self.model, self.env, [0.0], [q])["ok"]

    def _with(self, q, off):
        q = list(q)
        for k, j in enumerate(GAZE_JOINTS):
            q[j] += off[k]
        return q

    def step(self, q_base, target, now):
        """The pose to send this tick: the clip's q_base with the offsets."""
        if target is not None:
            self.lost_at = None
            want = [max(-m, min(m, x)) for x, m in zip(aim_offsets(self.rig, q_base, target), self.max_offset)]
            for f in SHRINK:                               # the aim shrunk until its pose is clear
                w = [x * f for x in want]
                if f == 0.0 or self._clear(self._with(q_base, w)):
                    self.shrunk += f < 1.0
                    self.want = w
                    break
        else:
            if self.lost_at is None:
                self.lost_at = now
            if now - self.lost_at >= self.hold_s:
                self.want = [0.0] * len(GAZE_JOINTS)       # back to the clip
        self.inp.target_position = list(self.want)
        self.inp.target_velocity = [0.0] * len(GAZE_JOINTS)
        self.otg.update(self.inp, self.out)
        self.out.pass_to_input(self.inp)
        q = self._with(q_base, self.out.new_position)
        if self.env is not None and not self._clear(q):
            self.unsafe += 1
        return q

    @property
    def offsets(self):
        return list(self.inp.current_position)


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

    import show as S
    import json
    import collision as C
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    g = S.Graph.load(S.compiled_path(os.path.join(ROOT, "shows", "party.json")))
    hub = g.hubs["greet"]
    gz = Gaze(env=env)
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
    gz = Gaze(env=env)
    qs = [gz.step(hub, (-0.9, -1.75, 1.6) if k < 200 else (0.6, -1.9, 1.7), k * gz.dt) for k in range(600)]
    ratio = max(max(abs(b - a) / gz.dt / (SHARE * vmax[j]) for a, b in zip(col, col[1:])) for j, col in
                ((j, [q[j] for q in qs]) for j in GAZE_JOINTS))
    racc = max(max(abs(c - 2 * b + a) / gz.dt ** 2 / (SHARE * amax[j]) for a, b, c in zip(col, col[1:], col[2:]))
               for j, col in ((j, [q[j] for q in qs]) for j in GAZE_JOINTS))
    check("a jump of the target 1.5 m: each joint's turn within %.0f %% of its velocity and acceleration limits"
          % (100 * SHARE), ratio <= 1.01 and racc <= 1.02, "velocity %.2f, acceleration %.2f of the share" % (ratio, racc))
    gz = Gaze(env=env)
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
