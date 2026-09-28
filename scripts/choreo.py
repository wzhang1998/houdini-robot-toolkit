"""Dance-like phrases for the FR20, driven by Laban Movement Analysis.

Laban's Effort describes HOW a body moves on four axes, each -1 .. +1 here:

    weight   light (-1)     .. strong (+1)      which joints lead: strong moves are
                                                whole-arm (J1-J3, far reaches), light
                                                ones wrist-led (J4-J6, near the body)
    time     sustained (-1) .. sudden (+1)      attack sharpness, tempo
    space    indirect (-1)  .. direct (+1)      detours / wandering harmonics
    flow     free (+1)      .. bound (-1)       overlap and blending vs holds

Weight x Time x Space give Laban's eight effort actions -- the vocabulary a
phrase is written in (and the one motion_labels.py measures back):

    punch  strong sudden direct     press  strong sustained direct
    slash  strong sudden indirect   wring  strong sustained indirect
    dab    light  sudden direct     glide  light  sustained direct
    flick  light  sudden indirect   float  light  sustained indirect

A phrase is bars of 4 beats at a tempo; each bar names an action (a phrase of
different actions is where the contrast comes from). Each bar is filled with
moves chosen for its action:

    travel   to a kinesphere pose: a TCP target at a level (low / mid / high),
             a direction from the robot's own view (front, left, right,
             diagonals), a reach, with the tool aimed (out, down, up, at the
             audience), solved by the closed-form IK nearest the current pose
    wave     a travelling wave through J2-J3-J5 (vertical) or J1-J4-J6
             (horizontal), phase-lagged towards the tool: follow-through
    sway     J1 swings, J6 counters it (the "head" keeps facing)
    bounce   a dip on every beat
    twist    J4 against J6 (wringing)
    look     the wrist turns the tool to a gaze point and back
    hold     stillness (with a slow breath when flow is free)

Animation principles (spec["principles"]: True by default, False for the
plain phrase, or a 0..1 strength), scaled by the effort:

    anticipation   a sudden travel (punch, slash, dab, flick) winds up
                   against its strike first -- 10-15 % of the distance, just
                   before the beat, slower than the strike
    overshoot      ... strikes 7-12 % past its target (more when flow is
                   free) and settles back inside its segment; every
                   reversal comes to rest, so a direct path stays direct
    follow-through J2-J6 run on delayed clocks (DELAY_SHAPE: the elbow a
                   little, the wrist most) -- successive flow: the wrist
                   trails the arm into a move and through its end. Under a
                   frame for punch / dab (their path must stay straight),
                   a few frames for press / glide, most for slash / flick
                   (a whip -- the lag costs a joint no acceleration, a detour
                   in a sudden strike would) and wring / float; twist and
                   sway also phase the hand behind the forearm / base
    breathing      a slow (3.6 s), tiny (< 0.4 deg) breath over the whole
                   phrase, faded in and out at the ends -- holds never
                   freeze. It stays under motion_labels' stillness floors.

With them a phrase is held to PLAN_SAFETY of the speed / acceleration
limits and LIMIT_MARGIN inside the joint limits; the labels were
calibrated with them on (motion_labels.CAL, from calibration_set()).

Dynamics (on by default; each switched off by a spec key, and with all
four off -- levels False, size False, tempo "steady", syncopate False,
what random_spec(dynamics=False) writes -- a phrase is the one it was
before them, frame for frame):

    levels     the phrase travels through the kinesphere's levels: each
               bar has a level (LEVEL_Z: low 0.72, mid 1.1, high 1.45 m TCP
               height), different from the one before -- a sink to low
               then a rise to high. A strong bar gets there with the whole
               arm, riding its travels (kinesphere poses at the level's
               height; a punch or slash in strikes kept to an arm jab of
               _jab_max, 20 deg, so they stay sudden); a glide rides its
               travels too, slowly enough to stay light
               (LIGHT_LEVEL_DEG_S); a float or flick has it carried under
               the whole bar (a slow "travel_level", wandering in the
               wrist); a dab stays where it is (a level change under it
               read flick or glide). Poses up high reach less far; a
               phrase whose TCP leaves TCP_Z (0.4-1.58 m: the lab
               controller's zone ends at 1.6) is planned again.
    size       each bar's amplitude from its effort (_size): strong and
               sudden big (~1.5), light and sustained small (~0.5), with
               a spread -- oscillations, wrist moves and jabs scale by it
               (a jab only up to _jab_max: a longer strike reads press)
    tempo      a tempo curve over the moves: steady, accelerando,
               ritardando, or a freeze (two still beats) then a burst; a
               sustained move is never quicker than the beat, nor shorter
               than SUSTAIN_S (3 s a wave, 2 s a travel) -- quicker, a
               float read flick
    syncopate  one sudden move pushed off the beat by half a beat
    accents    a bar with "moves": 1 is one move and home (random_spec
               makes most one-bar phrases accents: 3-5 s when sudden,
               half-beat rests at the ends)

A phrase longer than MAX_S (12 s per two bars) is played up to 0.85x
faster, or planned again -- after two tries with one move in each
sustained bar, then in every bar; one shorter than MIN_S (3 s) ends in
stillness; one where no travel found a target is planned again.

Every phrase starts and ends at rest in HOME, so clips chain -- or at the
pose in spec["start"] (a show's hub pose). A clip must
play at its designed speed (fairino_player's measure) and clear the cell
(collision.py): intensity is reduced first -- smaller oscillations and
detours, softer attacks, so the beat is kept -- and only then the tempo.

    phrase(spec, seed)            -> (times, joints, info)     24 fps frames
    make_clip(spec, seed, env)    -> motion_clip dict with labels
    random_spec(rng, ...)         -> a phrase spec
    calibration_set(kin, seeds)   -> single-action phrases for motion_labels.fit_cal
    hub_sweep(kin, env, ...)      -> phrases as show.hub_clips makes them, measured

Pure Python. Tests: python scripts/choreo.py
"""

import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import capability as CAP  # noqa: E402
import collision as CL  # noqa: E402
import fairino_player as P  # noqa: E402
import motion_clip as M  # noqa: E402
import robot_profile as RP  # noqa: E402
import ur_ik  # noqa: E402
import urdf_rig as U  # noqa: E402

FPS = 24.0
HOME = RP.home(RP.load("fr20")) or [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]   # profile robot.home_deg
ACTIONS = {                                           # (weight, time, space)
    "punch": (1, 1, 1), "slash": (1, 1, -1), "press": (1, -1, 1), "wring": (1, -1, -1),
    "dab": (-1, 1, 1), "flick": (-1, 1, -1), "glide": (-1, -1, 1), "float": (-1, -1, -1),
}
# robot's own view: it faces -X; its left is -Y
DIRECTIONS = {"front": 0.0, "front_left": 35.0, "left": 70.0, "front_right": -35.0, "right": -70.0}
LEVELS = {"low": 0.7, "mid": 1.1, "high": 1.6}
PLAN_SAFETY = 0.85                                    # fraction of the joint limits a plan may use
AUDIENCE = (-3.0, 0.0, 1.4)
LIMIT_MARGIN = 3.0                                    # degrees inside the joint limits
# the level contour's TCP heights: inside the controller's work zone (TCP z
# 0.1-1.6 m in the lab) and the show's operating range, with room for the
# moves on top (LEVELS["high"] is a kinesphere point, often past 1.6 m)
LEVEL_Z = {"low": 0.72, "mid": 1.1, "high": 1.45}
# how far a bar may change level (m), by the elbow (Kin.level_shift): the
# whole arm sinks and rises for a strong bar (its kinesphere travels aim at
# the level's height anyway); a light one does it slowly (sustained) or a
# little (sudden: a flick; a dab not at all)
LEVEL_DZ = {"strong": 0.8, "light_sustained": 0.38, "light_sudden": 0.16}
PROX_MASS = (4.0, 4.0, 2.5)                           # motion_labels.MASS[:3]: its weight measure
LIGHT_LEVEL_DEG_S = 5.0                               # peak mass-weighted J1-J3 speed of a light level change
                                                      # (motion_labels reads ~6.5 deg/s as strong)
TEMPOS = ("steady", "accel", "rit", "freeze")
TCP_Z = (0.4, 1.58)                                   # m: a phrase with levels stays in this band (spec["tcp_z"])
MAX_S = 12.0                                          # s per 2 bars: ... played faster (to 0.85x) or re-planned to fit
                                                      # (spec["max_s"]; the show's window)
MIN_S = 3.0                                           # s: ... and lasts at least this (a still end)
SUSTAIN_S = 3.0                                       # s: a sustained wave lasts at least this, a travel 2/3 of it


def efforts_of(action, flow=0.0):
    w, t, s = ACTIONS[action]
    return {"weight": float(w), "time": float(t), "space": float(s), "flow": float(flow)}


# --------------------------------------------------------------------------
# shaping functions
# --------------------------------------------------------------------------

def _minjerk(u):
    u = max(0.0, min(1.0, u))
    return u * u * u * (10 - 15 * u + 6 * u * u)


def _attack(time_effort, k):
    """Fraction of a move's duration spent getting there: sustained moves
    use all of it, sudden ones a third (softer as intensity k drops)."""
    sudden = (time_effort + 1) / 2.0 * k
    return 1.0 - 0.65 * sudden


def _envelope(u):
    """Hann window: 0 at both ends with zero slope, 1 in the middle. A fast
    min-jerk ramp at the ends added more acceleration than the oscillation
    it faded, and the fit then shrank every oscillation to nothing."""
    return 0.5 - 0.5 * math.cos(2 * math.pi * max(0.0, min(1.0, u)))


# --------------------------------------------------------------------------
# animation principles (spec["principles"]: True / False / 0..1 strength)
# --------------------------------------------------------------------------

# successive flow: how much of a segment's lag each joint takes -- the base
# and shoulder lead, the elbow follows a little, the wrist most
DELAY_SHAPE = (0.0, 0.0, 0.3, 0.6, 0.8, 1.0)
BREATH_S = 3.6                                        # a breath, seconds
BREATH_DEG = (0.14, 0.1, -0.2, 0.2, 0.35, 0.0)        # its depth per joint, mostly distal


def _principles(spec):
    """Strength of the animation principles, 0 (off: the plain phrase, with
    the checks it had before them) .. 1 (default)."""
    p = spec.get("principles", True)
    if p is True:
        return 1.0
    if not p:
        return 0.0
    return max(0.0, min(1.0, float(p)))


def _anticipation(eff, p):
    """(wind-up, overshoot) as fractions of a sudden travel's distance:
    strong actions wind up more; free flow overshoots more, bound less.
    Sustained travels get neither."""
    if eff["time"] <= 0 or p <= 0:
        return 0.0, 0.0
    return (p * (0.10 + 0.05 * (eff["weight"] + 1) / 2.0),
            p * (0.07 + 0.05 * (eff["flow"] + 1) / 2.0))


def _lag(eff, p):
    """Follow-through: how far (s) the wrist trails the base, a few frames --
    more when indirect (wandering), sustained (gentle drag) or free; under
    a frame for a direct sudden action (punch, dab), whose path must stay
    straight -- their life is the anticipation and overshoot instead. An
    indirect sudden action (slash, flick) whips: its strike is too short for
    a detour at the acceleration limits, but a lag is only a delay, and
    costs a joint no acceleration at all."""
    indirect, sudden = (1 - eff["space"]) / 2.0, (eff["time"] + 1) / 2.0
    frames = 0.5 + indirect * (2.5 + 3.0 * sudden) + 1.0 * (1 - sudden) + 0.5 * (eff["flow"] + 1) / 2.0
    return p * frames / FPS


def _strike(u, a, ant, over, w, z):
    """Progress of a sudden travel with anticipation and overshoot: a
    counter-move to -ant over [-w, 0] (before the beat), the strike to
    1 + over over [0, a], then settling back to 1 over [a, a + z]. Min-jerk
    pieces: every reversal comes to rest, so a direct path stays direct."""
    if u <= -w:
        return 0.0
    if u < 0.0:
        return -ant * _minjerk((u + w) / w)
    if u < a:
        return -ant + (1.0 + ant + over) * _minjerk(u / a)
    if z > 0.0 and u < a + z:
        return 1.0 + over - over * _minjerk((u - a) / z)
    return 1.0


def _breath(tt, total, beat, p, flow):
    """A slow, tiny breath over the whole phrase so it never freezes: faded
    in over the first beat (a second at least) and out over the last, so
    the phrase still starts and ends at rest in its pose; shallower when
    bound."""
    fade = min(max(beat, 1.0), total / 2.0)
    ramp = min(_minjerk(tt / fade), _minjerk((total - tt) / fade))
    if ramp <= 0.0 or p <= 0.0:
        return None
    w = 2 * math.pi / BREATH_S
    amp = p * ramp * (0.8 + 0.2 * flow)
    phase = (0.7, 0.0, 0.5, 2.1, 1.3, 0.0)
    rate = (0.5, 1.0, 1.0, 0.5, 1.0, 1.0)            # base and forearm drift at half the rate
    return [amp * d * math.sin(r * w * tt + ph) for d, r, ph in zip(BREATH_DEG, rate, phase)]


# --------------------------------------------------------------------------
# poses in the kinesphere
# --------------------------------------------------------------------------

class Kin:
    def __init__(self, acc=None):
        """acc: joint acceleration limit(s) to plan with instead of the
        profile's (deg/s^2) -- e.g. what scripts/accel_probe.py measured.
        Phrases, their checks and their labels all follow it."""
        self.model, self.chain, self.fo, self.vel = CAP.load_fr20(ROOT)
        prof = RP.load("fr20")
        self.acc = RP.acceleration_limits(prof)
        if acc is not None:
            self.acc = list(acc) if isinstance(acc, (list, tuple)) else [float(acc)] * 6
        self.limits = RP.motion_limits(prof)
        self.col = CL.load_model("fr20")

    def tcp(self, q):
        _, t = CL.capsules(self.col, q)
        return t

    def pose(self, level, direction, reach, aim, near, rng, z=None):
        """Joint pose putting the TCP at the kinesphere point, tool aimed;
        the IK branch nearest `near`. None when unreachable. z: the TCP
        height to use instead of the level's (LEVELS)."""
        az = math.radians(DIRECTIONS[direction] + rng.uniform(-10, 10))
        r = reach
        z = (LEVELS[level] if z is None else z) + rng.uniform(-0.1, 0.1)
        p = (-r * math.cos(az), -r * math.sin(az), z)       # front = -X, left = -Y
        if aim == "down":
            d = (0.0, 0.0, -1.0)
        elif aim == "up":
            d = (-0.3 * math.cos(az), -0.3 * math.sin(az), 1.0)
        elif aim == "audience":
            d = tuple(a - b for a, b in zip(AUDIENCE, p))
        else:                                                 # out: along the reach, a little down
            d = (-math.cos(az), -math.sin(az), -0.35)
        R = CAP.tool_frame(d, rng.uniform(-30, 30))
        p6 = U._sub(p, U._mat_vec(R, (0.0, 0.0, self.fo)))
        sols = ur_ik.within_limits(self.model, ur_ik.solve(self.model, R, p6, q6_when_singular=near[5]))
        sols = [s for s in sols if all(lo + LIMIT_MARGIN < x < hi - LIMIT_MARGIN
                                       for x, (lo, hi) in zip(s["q"], self.limits))
                and abs(math.sin(math.radians(s["q"][4]))) > 0.25]          # clear of the wrist singularity
        best = ur_ik.nearest(sols, near)
        if best is None:
            return None
        q = list(best["q"])
        return [b - 360.0 * round((b - a) / 360.0) for a, b in zip(near, q)]

    def level_shift(self, q, dz, keep_reach=True):
        """q with the TCP raised (dz > 0) or lowered by dz metres: the elbow
        (J3) sinks or lifts the forearm, the shoulder (J2) keeps the
        horizontal reach when keep_reach (a whole-arm move; without, the
        reach drifts a little and the shoulder stays), J4 turns back what
        J2 + J3 turned so the tool keeps its pitch (the three axes are
        parallel). Newton steps on the TCP; None when it does not get
        within 2 cm (out of reach from this pose)."""
        t0 = self.tcp(q)
        zg, rg = t0[2] + dz, math.hypot(t0[0], t0[1])
        q = list(q)
        h = 0.5
        for _ in range(5):
            t = self.tcp(q)
            r = math.hypot(t[0], t[1])
            ez, er = zg - t[2], rg - r
            if abs(ez) < 0.003 and (not keep_reach or abs(er) < 0.01):
                break
            cols = []
            for j in (1, 2):
                qq = list(q)
                qq[j] += h
                qq[3] -= h
                tt = self.tcp(qq)
                cols.append(((tt[2] - t[2]) / h, (math.hypot(tt[0], tt[1]) - r) / h))
            if keep_reach:
                (a, c), (b, d) = cols
                det = a * d - b * c
                if abs(det) < 1e-9:
                    return None
                d2, d3 = (d * ez - b * er) / det, (a * er - c * ez) / det
            else:
                if abs(cols[1][0]) < 1e-6:
                    return None
                d2, d3 = 0.0, ez / cols[1][0]
            if max(abs(d2), abs(d3)) > 60.0:
                return None
            q[1] += d2
            q[2] += d3
            q[3] -= d2 + d3
        return q if abs(self.tcp(q)[2] - zg) < 0.02 else None


# --------------------------------------------------------------------------
# a phrase: segments on a timeline
# --------------------------------------------------------------------------

# travel_far: a kinesphere pose by IK (whole arm); travel_near: a small change
# led by the wrist; wave_p / wave_d: proximal (J1-J3) / distal (J4-J6) waves
MENU = {
    "punch": ["travel_far", "travel_far", "bounce"],
    "slash": ["travel_curve", "sway", "travel_curve"],
    "press": ["travel_far", "hold", "travel_far"],
    "wring": ["wave_p", "twist", "travel_curve"],
    "dab":   ["travel_near", "look", "travel_near"],
    "flick": ["wave_d", "look", "travel_near"],
    "glide": ["travel_near", "look", "travel_near"],
    "float": ["wave_d", "travel_near", "wave_d"],
}
AIMS = {"punch": "out", "slash": "out", "press": "down", "wring": "out",
        "dab": "audience", "flick": "audience", "glide": "out", "float": "up"}
OSCILLATORS = ("wave_p", "wave_d", "sway", "twist", "bounce", "look")


def random_spec(rng, bars=None, actions=None, flow=None, bpm=None, dynamics=True):
    """A phrase: bars of one action each (repeats and contrasts), a tempo
    that suits the actions, a flow. With dynamics (default): 1-3 bars (a
    one-bar accent as often as a longer phrase), 70-140 bpm (sudden actions
    faster), a tempo curve (TEMPOS: steady, accelerando, ritardando, a
    freeze then a burst) and, when a bar is sudden, maybe a syncopated
    accent; levels and sizes are drawn by the planner (it knows the start
    pose). dynamics=False: the phrase as before -- 2-4 bars, 60-125 bpm,
    and the tempo, levels, sizes and syncopation switched off."""
    n = bars or rng.choice((1, 1, 2, 2, 3) if dynamics else (2, 3, 3, 4))
    names = list(ACTIONS)
    acts = actions or [rng.choice(names) for _ in range(n)]
    sudden = sum(ACTIONS[a][1] > 0 for a in acts) / float(len(acts))
    lo, hi = (70, 95) if dynamics else (60, 80)
    spec = {"bars": [{"action": a} for a in acts],
            "bpm": bpm or int(round(rng.uniform(lo, hi) + 45 * sudden)),
            "flow": flow if flow is not None else round(rng.uniform(-1, 1), 2)}
    if not dynamics:
        spec.update(levels=False, size=False, tempo="steady", syncopate=False)
        return spec
    # a freeze-and-burst suits a phrase with a sudden bar; the curves any
    spec["tempo"] = rng.choice(("steady", "accel", "rit", "freeze", "freeze") if sudden
                               else ("steady", "steady", "accel", "rit"))
    spec["syncopate"] = bool(sudden) and rng.random() < 0.5
    if len(acts) == 1 and rng.random() < (0.9 if sudden else 0.5):
        spec["bars"][0]["moves"] = 1                  # an accent: one move and home, 3-5 s when sudden
    return spec


FLOOR = {"schema": CL.ENV_SCHEMA, "margin_m": 0.05, "objects": [
    {"name": "floor", "type": "halfspace", "normal": [0, 0, 1], "offset": -0.02, "role": "obstacle"},
    {"name": "base_plate", "type": "box", "center": [0.0, 0.0, -0.01], "size": [1.2, 1.2, 0.02], "role": "obstacle"}]}


def _pose_ok(kin, q):
    """A planned pose alone: inside the joint limits by LIMIT_MARGIN, clear
    of the wrist singularity, of itself and of the floor."""
    return (all(lo + LIMIT_MARGIN < x < hi - LIMIT_MARGIN for x, (lo, hi) in zip(q, kin.limits))
            and abs(math.sin(math.radians(q[4]))) >= 0.25 and CL.check(kin.col, FLOOR, [0.0], [q])["ok"])


def _target(kind, eff, act, cur, kin, rng, tries=16, home=None, size=1.0, level_z=None):
    """Where a travel goes. Strong actions (travel_far / travel_curve): a
    kinesphere pose by IK -- the whole arm moves. Light ones (travel_near):
    the wrist leads -- J4-J6 turn, J2/J3 follow a little, J1 hardly. Every
    candidate is checked alone against self-collision and the floor.
    size scales a light move's span and a strong sudden one's jab (the
    effort's amplitude); level_z: kinesphere poses at that TCP height (the
    bar's level) instead of a random level."""
    for i in range(tries):
        sudden = eff["time"] > 0
        if kind == "travel_near":
            # drift back towards HOME as well, so light phrases stay centred
            span = [3, 6, 10, 30, 25, 45]
            if sudden:
                span = [x * 0.4 for x in span]
            span = [x * size for x in span]
            q = [c + 0.25 * (h - c) + rng.uniform(-s_, s_) for c, h, s_ in zip(cur, home or HOME, span)]
        else:
            reach = 1.25 + 0.15 * eff["weight"] + rng.uniform(-0.15, 0.1)
            lv = rng.choice(("low", "mid", "mid", "high"))
            z = None
            if level_z is not None:
                # at the bar's level (up high the arm cannot reach as far);
                # the last tries at the height it is at
                z = level_z - 0.04 if i < 2 * tries // 3 else kin.tcp(cur)[2]
                reach = min(reach, 1.1) if z > 1.3 else reach
            q = kin.pose(lv, rng.choice(list(DIRECTIONS)), reach, AIMS[act], cur, rng, z=z)
            if q is None or max(abs(a - b) for a, b in zip(q, cur)) > 150:
                continue
            # a strong move is a whole-arm move: skip poses reached mostly by
            # turning the wrist (a jab towards one moved J1-J3 a few degrees
            # and read light)
            if max(abs(a - b) for a, b in zip(q[:3], cur[:3])) < 0.4 * max(abs(a - b) for a, b in zip(q, cur)):
                continue
            if sudden:
                # a jab towards that pose: at 150 deg/s^2 a heavy arm cannot
                # be quick over a big distance, so a sudden move is a short one
                # in the same time a move covers distance in proportion to the
                # acceleration: 7-10 deg at FR20's 150 deg/s^2, up to 3x more
                # size: up to _jab_max (a longer strike reads sustained)
                jab = (7.0 + 3.0 * eff["weight"]) * min(3.0, max(1.0, min(kin.acc) / 150.0))
                jab = min(jab * size, max(jab, _jab_max(kin, eff)))
                d = max(abs(a - b) for a, b in zip(q, cur))
                if d > jab:
                    q = [c + (x - c) * jab / d for c, x in zip(cur, q)]
        if not _pose_ok(kin, q):
            continue
        return q
    return None


def _travel_time(kin, qa, qb):
    """Shortest minimum-jerk move qa -> qb within PLAN_SAFETY of the joint
    limits: peak acceleration 5.77 d / T^2, peak speed 1.875 d / T."""
    t = 0.0
    for a, b, v, acc in zip(qa, qb, kin.vel, kin.acc):
        d = abs(b - a)
        t = max(t, math.sqrt(5.7735 * d / (PLAN_SAFETY * acc)), 1.875 * d / (PLAN_SAFETY * v))
    return t


def _caps(seg, kin, beat):
    """Amplitude caps (deg) that keep a move's own acceleration within half
    the limits -- the other half is for the travels it rides on."""
    dur = seg["t1"] - seg["t0"]
    e = seg["eff"]
    kind = seg["kind"]
    if kind in ("wave_p", "wave_d"):
        w = 2 * math.pi * (2.0 if e["time"] > 0 else 1.0) / dur
        # the second harmonic of an indirect wave doubles the frequency
        w = w * (1.6 if e["space"] < 0 else 1.0)
    elif kind in ("sway", "twist"):
        # one cycle under a Hann window of the same length: the window's own
        # curvature adds up to ~2x the sine's acceleration (it overran the
        # limits, and the fit softened the whole phrase, attacks included)
        w = 1.5 * 2 * math.pi / dur
    elif kind == "bounce":
        ta = max(0.05, _attack(e["time"], 1.0) * 0.5 * beat)
        return [0.4 * a * ta * ta / 5.7735 for a in kin.acc]
    elif kind == "look":
        w = 2 * math.pi / dur
    else:
        return [1e9] * 6
    return [0.45 * a / (w * w) for a in kin.acc]


def _level_name(z):
    return "low" if z < 0.9 else "high" if z > 1.3 else "mid"


def _size(eff, rng):
    """A bar's amplitude from its effort, with a spread: strong and sudden
    big (punch ~1.5), light and sustained small (float, glide ~0.5)."""
    base = 1.0 + 0.3 * eff["weight"] + 0.2 * eff["time"]
    return round(max(0.35, min(1.8, base * rng.uniform(0.75, 1.3))), 3)


def _dynamics(spec, kin, home, rng):
    """The phrase's dynamics, resolved from the spec (drawing from rng only
    what is switched on and not given):

        levels     spec["levels"]: True (default: a contour, a level per bar
                   that differs from the one before -- from the start
                   pose's level, low <-> high jumps preferred), False (off:
                   the phrase stays at the start's level), or a list of
                   level names; bar["level"] overrides
        sizes      spec["size"]: True (default: _size, from the effort),
                   False (off: 1.0), or a number; bar["size"] overrides
        tempo      spec["tempo"]: one of TEMPOS, True (default: drawn),
                   False (steady)
        syncopate  spec["syncopate"]: True / False, or absent (default:
                   half the phrases with a sudden bar); one sudden move is
                   pushed off the beat by half a beat
    """
    bars = spec["bars"]
    n = len(bars)
    out = {"levels": [None] * n, "sizes": [1.0] * n, "tempo": "steady", "freeze_at": None, "sync_at": None}
    lv = spec.get("levels", True)
    if lv is not False:
        given = list(lv) if isinstance(lv, (list, tuple)) else [None] * n
        prev = _level_name(kin.tcp(home)[2])
        for i, bar in enumerate(bars):
            g = bar.get("level") or (given[i] if i < len(given) else None)
            if g is None:
                opts = [x for x in LEVEL_Z if x != prev]
                far = [x for x in opts if abs(LEVEL_Z[x] - LEVEL_Z[prev]) > 0.5]
                g = rng.choice(far) if far and rng.random() < 0.7 else rng.choice(opts)
            out["levels"][i] = prev = g
    sz = spec.get("size", True)
    for i, bar in enumerate(bars):
        if bar.get("size") is not None:
            out["sizes"][i] = float(bar["size"])
        elif sz is True:
            out["sizes"][i] = _size(efforts_of(bar["action"]), rng)
        elif sz not in (False, None):
            out["sizes"][i] = float(sz)
    bar_of = [i for i, bar in enumerate(bars) for _ in range(_n_moves(bar))]
    moves = len(bar_of)
    tempo = spec.get("tempo", True)
    if tempo is True:
        tempo = rng.choice(TEMPOS)
    out["tempo"] = tempo if tempo in TEMPOS else "steady"
    if out["tempo"] == "freeze" and moves > 1:
        out["freeze_at"] = rng.randrange(1, moves)
    sync = spec.get("syncopate")
    if sync is not False:
        cands = [i for i in range(moves) if ACTIONS[bars[bar_of[i]]["action"]][1] > 0]
        if cands and (sync is True or rng.random() < 0.5):
            out["sync_at"] = rng.choice(cands)
    return out


def _n_moves(bar):
    """Moves in a bar: two, or one for an accent (bar["moves"] = 1)."""
    return 1 if bar.get("moves") == 1 else 2


def _tempo_factor(dyn, i, n, sustained=False):
    """Beat length of move i of n, as a factor of the phrase's beat:
    accelerando 1.25 -> 0.8, ritardando 0.8 -> 1.3, a burst (0.75) after a
    freeze. A sustained move is never quicker than the beat (its waves
    at 0.8 of it read sudden): an accelerando up to the beat, a
    ritardando from it."""
    u = i / float(max(1, n - 1))
    f = 1.0
    if dyn["tempo"] == "accel":
        f = 1.25 - 0.45 * min(1.0, u)
    elif dyn["tempo"] == "rit":
        f = 0.8 + 0.5 * min(1.0, u)
    elif dyn["tempo"] == "freeze" and dyn["freeze_at"] is not None and i >= dyn["freeze_at"]:
        f = 0.75
    return max(f, 1.0) if sustained else f


def _prox(dq):
    """Mass-weighted J1-J3 change (motion_labels' weight measure)."""
    return sum(m * abs(x) for m, x in zip(PROX_MASS, dq[:3])) / sum(PROX_MASS)


def _level_delta(kin, cur, level, eff, size, rng):
    """(joint change, height it reaches, the level's height) taking the
    TCP from cur towards the level: as far as the effort allows (LEVEL_DZ,
    scaled by the bar's size against the effort's own), a whole-arm move
    for a strong bar, the elbow alone for a light one. A zero change when
    the pose cannot (a kinesphere travel may still get there: the level's
    height); for a dab, no level at all."""
    z = kin.tcp(cur)[2]
    goal = LEVEL_Z[level] + rng.uniform(-0.05, 0.05)
    strong, sudden = eff["weight"] > 0, eff["time"] > 0
    if not strong and sudden and eff["space"] > 0:
        # a dab is a touch where the arm already is: a level change under
        # it (whole-bar or riding its travels) reads flick or glide
        return [0.0] * 6, z, None
    cap = LEVEL_DZ["strong" if strong else "light_sudden" if sudden else "light_sustained"]
    nominal = 1.0 + 0.3 * eff["weight"] + 0.2 * eff["time"]
    cap *= max(0.75, min(1.25, size / nominal))
    dz = max(-cap, min(cap, goal - z))
    for f in (1.0, 0.6, 0.35):
        if abs(dz * f) < 0.04:
            break
        q = kin.level_shift(cur, dz * f, keep_reach=strong)
        if q is not None and _pose_ok(kin, q):
            return [b - a for a, b in zip(cur, q)], z + dz * f, goal
    return [0.0] * 6, z, goal


def _jab_max(kin, eff):
    """The longest sudden move of the arm (deg, the one of J1-J3 moving
    most) that still reads sudden: 20 at FR20's 300 deg/s^2 for a strong
    one (at 28 a punch read press), in proportion to the acceleration up
    to 3x."""
    return (7.0 + 3.0 * eff["weight"]) * min(3.0, max(1.0, min(kin.acc) / 150.0))


def _jab(kin, cur, target, eff):
    """A strike carrying the level: target pulled in towards cur until no
    arm joint (J1-J3) moves more than _jab_max, each wrist joint then
    clipped to what it does in the same time (sqrt of its faster
    acceleration). The arm keeps the whole budget: scaled by the joint
    moving most of all six, a wrist re-aiming the tool shrank the strike
    to 0.2 m; arm-first it is ~0.4 m and ~1 m/s. None if that pose is not
    clear."""
    lim = _jab_max(kin, eff)
    d = max(abs(a - b) for a, b in zip(target[:3], cur[:3]))
    f = min(1.0, lim / d) if d > 0 else 1.0
    wl = lim * math.sqrt(max(kin.acc[3:]) / min(kin.acc))
    q = [c + (x - c) * f for c, x in zip(cur, target)]
    q = q[:3] + [c + max(-wl, min(wl, x - c)) for c, x in zip(cur[3:], q[3:])]
    return q if _pose_ok(kin, q) else None


def _hold(t0, t1, eff, act, bi, beat):
    """A still beat (the breath goes on): a freeze, a syncopation's rest."""
    return {"t0": t0, "t1": t1, "kind": "hold", "eff": eff, "act": act, "bar": bi, "quiet": True,
            "phase": 0.0, "beat": beat, "cap": [1e9] * 6, "lag": 0.0}


LEVEL_BY_TRAVEL = ("punch", "press", "slash", "glide", "wring")        # the level rides on the travels


def _plan(spec, kin, rng, out=None):
    """Segments: dicts with t0, t1, kind, params. Returns (segments, total);
    out (a dict) receives the dynamics drawn (_dynamics).
    A travel takes the fewest whole beats its distance allows at the joint
    limits (and at its attack), so the phrase stays on the beat grid.

    Dynamics (_dynamics; each switchable in the spec): a bar goes to its
    level -- for a punch, press, slash, wring or glide the change rides on
    the bar's travels (split between them; a strike stays one straight
    move, _jab); for a float or flick a slow "travel_level" carries the
    whole bar, the moves on top; a dab keeps its level. A light bar
    changes level slowly enough to stay light (LIGHT_LEVEL_DEG_S). Sizes
    scale the moves; the tempo curve sets each move's beat; a freeze is
    two still beats before a burst; a syncopated move is framed by
    half-beat rests; an accent (one move) has half-beat rests at the ends."""
    beat = 60.0 / spec["bpm"]
    flow = spec.get("flow", 0.0)
    p = _principles(spec)
    home = list(spec.get("start") or HOME)
    # a beat of stillness first (half a beat before an accent's one move)
    accent = sum(_n_moves(bar) for bar in spec["bars"]) == 1
    rest = 0.5 * beat if accent else beat
    t = rest
    dyn = _dynamics(spec, kin, home, rng)
    if out is not None:
        out.update(dyn)
    dyn_on = dyn["levels"][0] is not None
    cur = list(home)
    anchor = list(home)                                       # home, carried to the current level
    segs = []
    nmoves = sum(_n_moves(bar) for bar in spec["bars"])
    mi = 0
    for bi, bar in enumerate(spec["bars"]):
        act = bar["action"]
        eff = efforts_of(act, flow)
        size = dyn["sizes"][bi]
        moves = list(MENU[act])
        rng.shuffle(moves)
        if _n_moves(bar) == 1:
            # an accent: one move, a travel when the bar has one
            moves = [next((m for m in moves if m.startswith("travel")), moves[0])]
        moves = moves[:2]
        bar_t0, bar_segs = t, []
        D, goal = [0.0] * 6, None
        if dyn["levels"][bi]:
            D, _, goal = _level_delta(kin, cur, dyn["levels"][bi], eff, size, rng)
        n_tr = sum(m.startswith("travel") for m in moves)
        if any(D) and n_tr == 0 and eff["weight"] > 0:
            # a strong bar going somewhere travels (a wring of two
            # oscillators over a slow carriage read slash)
            moves[-1] = next(m for m in MENU[act] if m.startswith("travel"))
            n_tr = 1
        ride = any(D) and act in LEVEL_BY_TRAVEL and n_tr > 0
        carry = any(D) and not ride
        # kinesphere travels aim at the level's height when the level rides
        # on them (even if the elbow alone could not get there), else stay
        # at the height the arm is at (a carriage takes it)
        goal = goal if goal is not None and act in LEVEL_BY_TRAVEL and n_tr > 0 else None
        light = eff["weight"] < 0
        for kind in moves:
            b = beat * _tempo_factor(dyn, mi, nmoves, eff["time"] < 0)
            if b != beat:
                b = min(max(b, 0.4), 1.0)                      # 60-150 bpm
            if mi == dyn["freeze_at"]:
                segs.append(_hold(t, t + 2 * beat, eff, act, bi, beat))    # the freeze
                t += 2 * beat
            if mi == dyn["sync_at"]:
                segs.append(_hold(t, t + 0.5 * b, eff, act, bi, b))        # off the beat
                t += 0.5 * b
            # oscillators: a whole bar when sustained, half when sudden;
            # travels: two beats, more if the distance needs it
            nb = (4 if eff["time"] < 0 else 2) if kind in OSCILLATORS else 2
            if _n_moves(bar) == 1 and eff["time"] < 0 and light:
                nb = 4                                         # a light sustained bar of one move takes the whole bar
            if dyn_on and eff["time"] < 0:
                # a sustained move stays slow however quick the tempo: under
                # 3 s at 90+ bpm a float's wave read flick (a travel: 2 s)
                need_s = SUSTAIN_S if kind in OSCILLATORS else SUSTAIN_S * 2.0 / 3.0
                nb = max(nb, int(math.ceil(need_s / b - 1e-9)))
            t0, t1 = t, t + nb * b
            seg = {"t0": t0, "t1": t1, "kind": kind, "eff": eff, "act": act, "bar": bi, "size": size}
            if kind.startswith("travel"):
                # riding: this travel's share of the level change, and the
                # home a light move drifts back to, carried with it
                base = [c + d / n_tr for c, d in zip(cur, D)] if ride else cur
                anc = [c + d / n_tr for c, d in zip(anchor, D)] if ride else anchor
                strike = ride and eff["time"] > 0
                target = _target(kind, eff, act, base, kin, rng, home=anc, size=size,
                                 level_z=(goal if goal is not None else kin.tcp(cur)[2]) if dyn_on else None)
                if target is not None and strike:
                    # a strike carrying the level stays a jab (a longer one
                    # is slower at the acceleration limits, and reads press)
                    target = _jab(kin, cur, target, eff)
                if target is None:
                    seg["kind"] = "hold"
                else:
                    anchor = anc
                    seg["from"], seg["to"] = list(cur), target
                    seg["detour"] = [rng.uniform(-1, 1) for _ in range(6)]
                    if accent and eff["time"] > 0:
                        seg["attack_min"] = 0.55                 # an accent's strike rests less after
                    a = max(_attack(eff["time"], 1.0), seg.get("attack_min", 0.0))
                    # + room for detour / overshoot: the principles' wind-up and
                    # overshoot (<= 0.27 of the distance) fit in it, so a strike
                    # stays as quick as a plain travel
                    need = _travel_time(kin, cur, target) * 1.25 / a
                    if ride and light:
                        # a light bar's level change stays slow: light
                        need = max(need, 1.875 * _prox([x - y for x, y in zip(base, cur)]) / LIGHT_LEVEL_DEG_S / a)
                    seg["ant"], seg["over"] = _anticipation(eff, p)
                    nb = max(nb, int(math.ceil(need / b - 1e-9)))
                    seg["t1"] = t1 = t0 + nb * b
                    seg["acc_min"] = min(kin.acc)
                    if dyn_on:
                        seg["lead_max"] = 0.5 * t0                 # a long move leads, not before the phrase
                    cur = target
            elif kind == "look":
                seg["dir"] = [rng.uniform(-1, 1), rng.uniform(-1, 1)]
            seg["phase"] = rng.uniform(0, 2 * math.pi)
            seg["beat"] = b
            seg["cap"] = _caps(seg, kin, b)
            seg["lag"] = _lag(eff, p)
            if kind in ("twist", "sway"):
                seg["succ"] = p * math.pi / 3.0                # phase the hand trails by
            segs.append(seg)
            bar_segs.append(seg)
            t = t1
            if mi == dyn["sync_at"]:
                segs.append(_hold(t, t + 0.5 * b, eff, act, bi, b))        # back on the beat
                t += 0.5 * b
            if flow < -0.3:                                    # bound: a held beat after every move
                segs.append({"t0": t, "t1": t + b, "kind": "hold", "eff": eff, "act": act, "bar": bi,
                             "phase": 0.0, "beat": b, "cap": [1e9] * 6, "lag": 0.0})
                t += b
            mi += 1
        if carry:
            seg = _carriage(kin, cur, D, bar_segs, bar_t0, t, eff, act, bi, p, rng)
            if seg:
                segs.append(seg)
                anchor = [a + y - x for a, x, y in zip(anchor, seg["from"], seg["to"])]
                cur = seg["to"]
    # home again, then a beat of stillness
    # bound flow arrives in 0.8 of the travel (see _travel): plan for that
    a_home = 0.8 if flow < -0.3 else 1.0
    b = beat * _tempo_factor(dyn, nmoves - 1, nmoves)
    if b != beat:
        b = min(max(b, 0.4), 1.0)
    need = _travel_time(kin, cur, home) * 1.1 / a_home
    if dyn_on and all(ACTIONS[x["action"]][0] < 0 for x in spec["bars"]):
        # a light phrase comes home as lightly as it left
        need = max(need, 1.875 * _prox([x - y for x, y in zip(home, cur)]) / LIGHT_LEVEL_DEG_S / a_home)
    nb = max(2, int(math.ceil(need / b - 1e-9)))
    glide = efforts_of("glide", flow)
    segs.append({"t0": t, "t1": t + nb * b, "kind": "travel_home", "from": list(cur), "to": list(home),
                 "eff": glide, "act": "glide", "detour": [0.0] * 6, "phase": 0.0,
                 "beat": b, "cap": [1e9] * 6, "lag": _lag(glide, p)})
    total = t + nb * b + rest
    if dyn_on:
        total = max(total, MIN_S)                             # an accent is still a phrase: 3 s at least
    return segs, total


def _carriage(kin, cur, D, bar_segs, t0, t1, eff, act, bi, p, rng):
    """A slow level change under a whole bar ("travel_level"): sustained
    over the bar, at most what a light bar may do (LIGHT_LEVEL_DEG_S) and
    what the joints can in that time, wandering (a detour) when the bar is
    indirect -- for a light bar in the wrist only; every pose the bar
    travels to is checked with it added. None when nothing fits."""
    span = t1 - t0
    light = eff["weight"] < 0
    f = 1.0
    if light:
        f = min(f, LIGHT_LEVEL_DEG_S * span / 1.875 / max(1e-9, _prox(D)))
    T = _travel_time(kin, [0.0] * 6, D) * 1.1
    if T > span:
        f = min(f, (span / T) ** 2)
    ends = [s["to"] for s in bar_segs if "to" in s] or [cur]
    for _ in range(3):
        d = [x * f for x in D]
        if max(abs(x) for x in d) < 1.0:
            return None
        if all(_pose_ok(kin, [a + x for a, x in zip(q, d)]) for q in ends):
            sust = dict(eff, time=-1.0)
            # (half a travel's: under a whole bar the full one lifted the TCP past 2 m)
            det = [0.0] * 6
            if eff["space"] < 0:
                det = [0.5 * rng.uniform(-1, 1) * (j >= 3 or not light) for j in range(6)]
            return {"t0": t0, "t1": t1, "kind": "travel_level", "from": list(cur),
                    "to": [a + x for a, x in zip(cur, d)], "eff": sust, "act": act, "bar": bi,
                    "detour": det, "phase": 0.0, "beat": span, "cap": [1e9] * 6,
                    "lag": _lag(sust, p), "acc_min": min(kin.acc), "lead_max": 0.5 * t0}
        f *= 0.5
    return None


def _offsets(seg, tt, k):
    """Joint offsets (degrees) an oscillating move adds at time tt. Each
    joint's amplitude is the design one, capped by what the joint can
    accelerate at this move's frequency (seg["cap"], from _caps)."""
    u = (tt - seg["t0"]) / (seg["t1"] - seg["t0"])
    if u <= 0.0 or u >= 1.0:
        return None
    e = seg["eff"]
    dur = seg["t1"] - seg["t0"]
    amp = (0.65 + 0.35 * e["weight"]) * k * seg.get("size", 1.0)   # light 0.3 .. strong 1.0, x size
    cap = seg.get("cap", [1e9] * 6)
    A = lambda j, design: min(design * amp, cap[j] * k)
    env = _envelope(u)
    kind = seg["kind"]
    off = [0.0] * 6
    ph = seg["phase"]
    if kind in ("wave_p", "wave_d"):
        cycles = 2.0 if e["time"] > 0 else 1.0
        joints = (0, 1, 2) if kind == "wave_p" else (3, 4, 5)
        ratio = (0.6, 0.8, 1.0) if kind == "wave_p" else (0.7, 0.85, 1.0)
        for n, (j, rr) in enumerate(zip(joints, ratio)):
            s = math.sin(2 * math.pi * cycles * u + ph - n * math.pi / 3)
            if e["space"] < 0:                                 # indirect: a second harmonic wanders
                s += 0.35 * math.sin(4 * math.pi * cycles * u + 1.7 * ph - n * 0.9)
            off[j] += A(j, 14.0 * rr) * s * env
    elif kind == "sway":
        s = math.sin(2 * math.pi * u + ph)
        # successive flow (principles): the hand's counter-turn trails the base
        off[0] += A(0, 16.0) * s * env
        off[5] -= A(5, 13.0) * math.sin(2 * math.pi * u + ph - seg.get("succ", 0.0)) * env
        # twice the frequency: a quarter of the cap (it was capped for the
        # sway's own frequency, needed 2-4x J4's acceleration limit, and the
        # fit then softened every slash phrase to its minimum intensity)
        off[3] += min(5.0 * amp, cap[3] * k / 4.0) * math.sin(4 * math.pi * u + ph) * env
    elif kind == "twist":
        s = math.sin(2 * math.pi * u + ph)
        off[3] += A(3, 22.0) * s * env
        # successive (principles): the hand trails the forearm -- a spiral,
        # not a straight back-and-forth in the joints
        off[5] -= A(5, 30.0) * math.sin(2 * math.pi * u + ph - seg.get("succ", 0.0)) * env
    elif kind == "bounce":
        beats = max(1, int(round(dur / seg.get("beat", dur))))  # a dip on every beat
        ub = (u * beats) % 1.0
        a = _attack(e["time"], k) * 0.5
        dip = _minjerk(ub / a) if ub < a else 1.0 - _minjerk((ub - a) / (1 - a))
        off[1] += A(1, 4.5) * dip * env
        off[2] -= A(2, 6.0) * dip * env
        off[4] += A(4, 3.0) * dip * env
    elif kind == "look":
        s = math.sin(math.pi * u) ** 2
        off[3] += A(3, 25.0) * seg["dir"][0] * s
        off[4] += A(4, 20.0) * seg["dir"][1] * s
    elif kind == "hold":
        if e["flow"] > 0.2 and not seg.get("quiet"):          # free: breathe (a freeze does not)
            off[1] += 1.2 * math.sin(2 * math.pi * u) * env
            off[2] -= 1.5 * math.sin(2 * math.pi * u) * env
    else:
        return None
    return off


def _strike_timing(seg, a, k, lead):
    """(wind-up, overshoot, w, z) of a sudden travel at intensity k, w and z
    as fractions of the travel's duration: the wind-up before the beat
    (at most 3/4 of a beat, never before the phrase's first beat), the
    settle inside the segment after the strike. Squeezed timings shrink
    the amplitudes with the square, so no piece is sharper than the strike."""
    ant, over = seg.get("ant", 0.0), seg.get("over", 0.0)
    if ant <= 0.0 and over <= 0.0:
        return 0.0, 0.0, 0.0, 0.0
    span = seg["t1"] - seg["t0"] + lead
    beat = seg.get("beat", span)
    w = max(0.0, min(0.6 * a, 0.75 * beat / span, (seg["t0"] - lead - 0.25 * beat) / span))
    z = max(0.0, min(0.9 * a, 0.96 - a))
    ant *= k * min(1.0, w / (0.6 * a)) ** 2
    over *= k * min(1.0, z / (0.9 * a)) ** 2
    return (ant if w > 0 else 0.0), (over if z > 0 else 0.0), w, z


def _travel(seg, tt, k, flow, lags=None):
    """(progress of the pose change, detour offsets) for a travel at tt.
    lags (per joint, s): the principles' version -- progress per joint,
    each joint's clock delayed by its lag, sudden travels with anticipation
    and overshoot; without, exactly the plain travel."""
    e = seg["eff"]
    dur = seg["t1"] - seg["t0"]
    # free flow starts the move a quarter-beat early and lets it overlap
    lead = min(0.12 * dur * max(0.0, flow), seg.get("lead_max", 1e9))
    a = max(_attack(e["time"], k), seg.get("attack_min", 0.0))
    if flow < -0.3:
        a = min(a, 0.8)                                       # bound: arrive, then be still
    if lags is not None:
        ant, over, w, z = _strike_timing(seg, a, k, lead)

        def at(t_):
            u_ = (t_ - seg["t0"] + lead) / (dur + lead)
            return [_strike(u_, a, ant, over, w, z)] * 6, _detour(seg, u_, a, k, dur + lead)
        both = _lagged(lambda t_: list(zip(*at(t_))), tt, lags)   # per joint: (progress, detour)
        return [x[0] for x in both], [x[1] for x in both]
    u = (tt - seg["t0"] + lead) / (dur + lead)
    return _minjerk(u / a), _detour(seg, u, a, k, dur + lead)


def _detour(seg, u, a, k, span):
    """Detour offsets (deg) of a travel at progress time u (attack a)."""
    e = seg["eff"]
    det = [0.0] * 6
    if 0.0 < u < a:
        # sin^2: the detour starts and ends at rest, as the travel does (a
        # half-sine ended with a slope where the travel stopped -- a velocity
        # kink that read as 9x the acceleration limit)
        bump = math.sin(math.pi * u / a) ** 2
        size = (1 - e["space"]) / 2.0 * 14.0 * k * (0.7 + 0.3 * e["weight"])
        if seg["kind"] == "travel_curve":
            size = max(size, 10.0 * k)
        ta = a * span
        size = min(size, 0.3 * seg.get("acc_min", 135.0) * ta * ta / (2 * math.pi ** 2))
        det = [size * d * bump for d in seg["detour"][:5]] + [0.0]
    return det


def _lagged(fn, tt, lags):
    """fn(t) -> 6 values or None, evaluated per joint at t - lag[j] (each
    distinct lag once): the wrist runs a few frames behind the base."""
    out, cache = [0.0] * 6, {}
    for j, lg in enumerate(lags):
        if lg not in cache:
            cache[lg] = fn(tt - lg)
        if cache[lg]:
            out[j] = cache[lg][j]
    return out


def _sample(segs, total, k, flow, scale=1.0, home=None, p=0.0):
    """24 fps frames of the phrase played `scale` times slower: frame i is at
    i / FPS, and shows the plan at i / FPS / scale. (Stretching the frame
    times instead gave a slowed phrase frames at 17 fps -- fine for the
    player, which goes by time, but not what an exported clip should be.)
    p > 0: with the animation principles (follow-through lags, anticipation
    and overshoot, breathing)."""
    ts = [i / FPS for i in range(int(math.ceil(total * scale * FPS)) + 1)]
    qs = []
    beat = segs[0]["beat"] if segs else 0.5
    lags = {id(s): [s.get("lag", 0.0) * d for d in DELAY_SHAPE] for s in segs} if p > 0 else {}
    for tt in (t / scale for t in ts):
        q = list(home or HOME)
        for seg in segs:
            if "to" in seg:
                s, det = _travel(seg, tt, k, flow, lags.get(id(seg)))
                if not isinstance(s, list):
                    s = [s] * 6
                q = [a + sj * (b - c) + d for a, sj, b, c, d in zip(q, s, seg["to"], seg["from"], det)]
        for seg in segs:
            if p > 0:
                if tt < seg["t0"] or tt > seg["t1"] + seg.get("lag", 0.0):
                    continue
                off = _lagged(lambda t_, sg=seg: _offsets(sg, t_, k), tt, lags[id(seg)])
            else:
                off = _offsets(seg, tt, k)
            if off:
                q = [a + b for a, b in zip(q, off)]
        if p > 0:
            br = _breath(tt, total, beat, p, flow)
            if br:
                q = [a + b for a, b in zip(q, br)]
        qs.append(q)
    return ts, qs


def phrase(spec, seed=0, env=None, kin=None, safety=0.9, max_tries=6):
    """24 fps frames for a phrase spec: (times, joints, info). info: the
    intensity kept, tempo, attempts, and why a try was dropped."""
    kin = kin or Kin()
    rng = random.Random(seed)
    p = _principles(spec)
    info = {"tries": [], "bpm": spec["bpm"], "principles": p}
    # with the principles, the result is held to the planning margins
    # (PLAN_SAFETY of the speed / acceleration limits, LIMIT_MARGIN inside
    # the joint limits); without, exactly the checks the phrases had before
    margin = LIMIT_MARGIN if p > 0 else 1.0
    if p > 0:
        safety = min(safety, PLAN_SAFETY)
    sp = spec
    for attempt in range(max_tries):
        dyn = {}
        segs, total = _plan(sp, kin, rng, out=dyn)
        levels = dyn["levels"][0] is not None
        if levels and all(s["kind"] in ("hold", "travel_home") for s in segs) and attempt < max_tries - 1:
            info["tries"].append("nothing moves (no target reached)")
            continue
        band = spec.get("tcp_z", TCP_Z if levels else None)
        max_s = spec.get("max_s", MAX_S * max(1.0, len(spec["bars"]) / 2.0) if levels else None)
        k, bpm_scale = 1.0, 1.0
        if max_s and total > max_s:
            if total * 0.85 > max_s and attempt < max_tries - 1:
                info["tries"].append("%.1f s, longer than %g s" % (total, max_s))
                # too long twice: one move in each sustained bar (a whole
                # bar each), then in every bar -- rather than playing a
                # sustained phrase so fast it reads sudden
                if attempt >= 1:
                    sust = attempt < 3
                    sp = dict(sp, bars=[dict(b, moves=1) if "moves" not in spec["bars"][i]
                                        and (not sust or ACTIONS[b["action"]][1] < 0) else b
                                        for i, b in enumerate(sp["bars"])])
                continue
            bpm_scale = max(0.85, max_s / total)              # a long phrase, a little faster
        for _ in range(8):
            ts, qs = _sample(segs, total, k, spec.get("flow", 0.0), bpm_scale, spec.get("start"), p)
            if any(not lo + margin < x < hi - margin for q in qs for x, (lo, hi) in zip(q, kin.limits)):
                k *= 0.7
                continue
            lim = P.limiting(ts, qs, 125.0, [v * safety for v in kin.vel], [a * safety for a in kin.acc])
            need = lim["scale_needed"]
            if need <= 1.0:
                break
            if k > 0.35:
                k = max(0.35, k / (need * need))             # acceleration ~ amplitude: keep the beat
            else:
                bpm_scale *= need * 1.02                      # last resort: slower
        else:
            info["tries"].append("could not fit the limits")
            continue
        if band:
            z = [c[2] for c in M._tcp_path("fr20", qs[::2])]
            if min(z) < band[0] or max(z) > band[1]:
                info["tries"].append("TCP height %.2f..%.2f m outside %g..%g" % (min(z), max(z), band[0], band[1]))
                continue
        if env is not None:
            rep = CL.check(kin.col, env, ts, qs)
            if not rep["ok"]:
                info["tries"].append(CL.describe(rep))
                continue
            info["collision"] = rep
        info.update(dynamics=dyn, intensity=round(k, 3), tempo_scale=round(bpm_scale, 3),
                    bpm_played=round(spec["bpm"] / bpm_scale, 1), segments=[
                        {"t0": round(s["t0"] * bpm_scale, 3), "t1": round(s["t1"] * bpm_scale, 3), "kind": s["kind"],
                         "action": s["act"], "bar": s.get("bar")} for s in segs], attempt=attempt)
        return ts, qs, info
    return None, None, info


def make_clip(spec, seed=0, env=None, clip_id=None, kin=None, tags=()):
    """A motion_clip dict (safety, labels) or a rejected one with reasons."""
    import motion_labels as L
    kin = kin or Kin()
    ts, qs, info = phrase(spec, seed, env, kin)
    cid = clip_id or "dance_%s_%d" % ("-".join(b["action"] for b in spec["bars"]), seed)
    clip = {"schema": M.SCHEMA, "id": cid, "robot": "fr20", "joint_names": ["j%d" % i for i in range(1, 7)],
            "units": {"angle": "deg", "time": "s", "length": "m"}, "points": [], "tcp": None,
            "style": {"generator": "choreo.py", "spec": spec, "seed": seed},
            "meta": {"duration_s": 0.0, "bounds": None, "tags": list(tags), "source": {"generator": "choreo.py"}}}
    if ts is None:
        clip["safety"] = {"ok": False, "reasons": ["no feasible phrase: " + "; ".join(info["tries"][-2:])],
                          "playback_scale": None}
        return clip
    clip["points"] = [{"t": round(t, 6), "q": [round(x, 5) for x in q]} for t, q in zip(ts, qs)]
    clip["tcp"] = M._tcp_path("fr20", qs)
    xs = list(zip(*clip["tcp"]))
    clip["meta"]["bounds"] = {"min": [min(a) for a in xs], "max": [max(a) for a in xs]}
    clip["meta"]["duration_s"] = round(ts[-1], 6)
    clip["style"].update({k: info[k] for k in ("intensity", "tempo_scale", "bpm_played", "segments", "principles",
                                               "dynamics")})
    clip["style"]["acc_limit"] = kin.acc
    M.measure(clip, acc=kin.acc)
    if env is not None:
        clip["safety"]["collision"] = CL.describe(info["collision"])
        clip["safety"]["min_clearance_m"] = info["collision"]["min_env_clearance_m"]
        clip["safety"]["min_self_clearance_m"] = info["collision"]["min_self_clearance_m"]
    clip["labels"] = L.label(clip, intent=spec)
    # one label per bar too: a mixed phrase is a sequence, not an average
    bars = {}
    for sg in info["segments"]:
        if sg.get("bar") is not None:
            b = bars.setdefault(sg["bar"], [sg["t0"], sg["t1"], sg["action"]])
            b[0], b[1] = min(b[0], sg["t0"]), max(b[1], sg["t1"])
    clip["labels"]["bars"] = [dict(L.label_span(clip, t0, t1), intent=a) for _, (t0, t1, a) in sorted(bars.items())]
    clip["labels"]["bars_agree"] = sum(b["action"] == b["intent"] for b in clip["labels"]["bars"])
    clip["labels"]["sequence"] = [b["action"] for b in clip["labels"]["bars"]]
    return clip


OPPOSITES = [("punch", "float"), ("slash", "glide"), ("press", "flick"), ("wring", "dab")]
# start poses the labels are calibrated from: HOME (the rest hub as show.py
# makes its clips, J1 turned to 0), the party's rest hub itself, a low hub
CAL_STARTS = {"home": HOME, "rest": [-60.0, -90.0, 90.0, -90.0, -90.0, 0.0],
              "low": [0.0, -75.0, 110.0, -125.0, -90.0, 0.0]}


def calibration_set(kin=None, seeds=range(4), starts=None, principles=None, log=None):
    """Single-action phrases for motion_labels.fit_cal: every action x seeds
    x start poses, 1 or 2 bars, tempo and flow from random_spec (every other
    seed clamped to a show's 80-125 bpm). Rows: {"action", "flow", "raw"
    (motion_labels.raw_measures), "seed", "start", "spec", "ts", "qs",
    "info"}; a phrase that cannot be made is left out (and logged)."""
    import motion_labels as L
    kin = kin or Kin()
    rows = []
    for act in ACTIONS:
        for seed in seeds:
            for sname, start in (starts or CAL_STARTS).items():
                rng = random.Random(7919 * seed + 31 * list(ACTIONS).index(act) + 1)
                spec = random_spec(rng, actions=[act] * (1 + seed % 2))
                if seed % 2:
                    spec["bpm"] = int(min(max(spec["bpm"], 80), 125))
                spec["start"] = list(start)
                if principles is not None:
                    spec["principles"] = principles
                ts, qs, info = phrase(spec, seed, kin=kin)
                if ts is None:
                    if log:
                        log("  %s seed %d from %s: no phrase" % (act, seed, sname))
                    continue
                tcp = M._tcp_path("fr20", qs)
                raw = L.raw_measures(ts, qs, tcp)
                rows.append({"action": act, "flow": spec["flow"], "seed": seed, "start": sname, "spec": spec,
                             "raw": {k: raw[k] for k in ("weight", "time", "space", "still")},
                             "ts": ts, "qs": qs, "info": info})
    return rows


def dance_wedge(n=48, seed=7):
    """Variants for the dance factory, from a fixed seed: every action alone
    (twice), contrasting pairs (A B and A B A -- dynamics come from contrast),
    then random 2-4 bar phrases. Each: {"id", "spec", "seed", "tags"}."""
    rng = random.Random(seed)
    out = []
    for a in ACTIONS:
        for k in range(2):
            out.append({"spec": {"bars": [{"action": a}] * 2, "bpm": 70 if ACTIONS[a][1] < 0 else 110,
                                 "flow": round(rng.uniform(-0.8, 0.8), 2)}, "tags": ["single", a]})
    for a, b in OPPOSITES:
        out.append({"spec": random_spec(rng, actions=[a, b]), "tags": ["contrast", a, b]})
        out.append({"spec": random_spec(rng, actions=[a, b, a]), "tags": ["contrast", "aba", a, b]})
    while len(out) < n:
        out.append({"spec": random_spec(rng), "tags": ["mix"]})
    for i, v in enumerate(out[:n]):
        v["seed"] = i
        v["id"] = "d%02d_%s" % (i, "-".join(b["action"] for b in v["spec"]["bars"]))
    return out[:n]


REST_HUB = CAL_STARTS["rest"]


def hub_sweep(kin, env=None, hubs=None, n=16, seed=11, dynamics=True, bars=(1, 2), duration_s=(3.0, 12.0),
              tcp_z=(0.35, 1.6), every=3):
    """Phrases as show.hub_clips makes them for its hubs (default: the
    party's rest hub and HOME): random_spec (bars 1-2, its own tempo),
    started at the hub with J1 = 0, then turned by the hub's J1. Kept when
    made, within duration_s, the TCP within tcp_z (the show's range) and,
    with env, clear of the room (every `every`-th frame: a cheap check;
    the show checks every frame). Rows: {"hub", "spec", "ok", "why", "dur",
    "zspan", "extent" (the TCP's largest box side), "v_mean", "v_peak"
    (TCP, m/s), "action" (measured), "ts", "qs" (turned), "hub_q", "info"}."""
    import motion_labels as L
    rows = []
    for hi, (name, hub) in enumerate((hubs or {"rest": REST_HUB, "home": HOME}).items()):
        rng = random.Random(seed + hi)
        local = [0.0] + list(hub[1:])
        for _ in range(n):
            spec = random_spec(rng, bars=rng.choice(bars), dynamics=dynamics)
            spec["start"] = local
            ts, qs, info = phrase(spec, seed=rng.randrange(10 ** 9), kin=kin)
            row = {"hub": name, "hub_q": list(hub), "spec": spec, "info": info, "ok": False, "why": None}
            rows.append(row)
            if ts is None:
                row["why"] = "no phrase: " + "; ".join(info["tries"][-1:])
                continue
            qs = [[q[0] + hub[0]] + list(q[1:]) for q in qs]
            tcp = M._tcp_path("fr20", qs)
            box = [max(a) - min(a) for a in zip(*tcp)]
            v = [math.dist(tcp[i + 1], tcp[i]) / (ts[i + 1] - ts[i]) for i in range(len(ts) - 1)]
            z = [c[2] for c in tcp]
            row.update(ts=ts, qs=qs, dur=ts[-1], zspan=box[2], extent=max(box), v_mean=sum(v) / len(v), v_peak=max(v),
                       action=L.efforts(L.raw_measures(ts, qs, tcp))["action"])
            if not duration_s[0] <= ts[-1] <= duration_s[1]:
                row["why"] = "%.1f s" % ts[-1]
            elif min(z) < tcp_z[0] or max(z) > tcp_z[1]:
                row["why"] = "TCP height %.2f..%.2f m" % (min(z), max(z))
            elif env is not None:
                rep = CL.check(kin.col, env, ts[::every] + ts[-1:], qs[::every] + qs[-1:])
                row["why"] = None if rep["ok"] else CL.describe(rep)[:80]
            row["ok"] = row["why"] is None
    return rows


def sweep_stats(rows):
    """Medians and shares of hub_sweep's kept rows."""
    kept = [r for r in rows if r["ok"]]
    med = lambda xs: sorted(xs)[len(xs) // 2] if xs else float("nan")  # noqa: E731
    share = lambda f: sum(1 for r in kept if f(r)) / float(max(1, len(kept)))  # noqa: E731
    return {"kept": len(kept), "tried": len(rows), "zspan": med([r["zspan"] for r in kept]),
            "extent": med([r["extent"] for r in kept]), "dur": med([r["dur"] for r in kept]),
            "short": share(lambda r: r["dur"] <= 5.0), "fast": share(lambda r: r["v_peak"] >= 1.2),
            "calm": share(lambda r: r["v_peak"] < 0.4), "zspan_03": share(lambda r: r["zspan"] >= 0.3),
            "dur_range": (min([r["dur"] for r in kept] or [0]), max([r["dur"] for r in kept] or [0])),
            "bpm_range": (min(r["spec"]["bpm"] for r in rows), max(r["spec"]["bpm"] for r in rows))}


if __name__ == "__main__":
    import time
    import motion_labels as L

    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    kin = Kin()
    env = CL.load_env(os.path.join(ROOT, "envs", "volvox_lab.usda"))
    t0 = time.time()
    clips = {}
    for act in ACTIONS:
        spec = {"bars": [{"action": act}] * 2, "bpm": 70 if ACTIONS[act][1] < 0 else 110, "flow": 0.0}
        clips[act] = make_clip(spec, seed=1, env=env, kin=kin)
    made = [a for a, c in clips.items() if c["safety"].get("ok")]
    check("every Laban action makes a clip that plays at its own speed and clears the cell",
          len(made) == 8, "%d/8 in %.0f s; %s" % (len(made), time.time() - t0,
                                                    "; ".join("%s: %s" % (a, c["safety"]["reasons"]) for a, c in clips.items()
                                                              if not c["safety"].get("ok"))))
    c = clips["punch"]
    if c["points"]:
        q0, q1 = c["points"][0]["q"], c["points"][-1]["q"]
        check("a phrase starts and ends at rest in HOME (clips chain)",
              max(abs(a - b) for a, b in zip(q0, HOME)) < 1e-6 and max(abs(a - b) for a, b in zip(q1, HOME)) < 1e-3)
    hub = [0.0, -75.0, 110.0, -125.0, -90.0, 0.0]                 # a lower pose, as a show's hub
    ts_h, qs_h, _ = phrase({"bars": [{"action": "dab"}], "bpm": 100, "flow": 0.0, "start": hub}, seed=3, kin=kin)
    check("a phrase given a start pose starts and ends there (show hubs)",
          qs_h is not None and max(abs(a - b) for a, b in zip(qs_h[0], hub)) < 1e-6
          and max(abs(a - b) for a, b in zip(qs_h[-1], hub)) < 1e-3, str(qs_h and [round(x, 2) for x in qs_h[-1]]))
    # measured efforts follow the intent (same seed, one axis flipped)
    def measured(act, flow=0.0, seed=2):
        spec = {"bars": [{"action": act}] * 2, "bpm": 90, "flow": flow}
        cl = make_clip(spec, seed=seed, env=None, kin=kin)
        return cl["labels"]["measured"] if cl.get("labels") else None
    pairs = [("time", "punch", "press"), ("time", "dab", "glide"), ("weight", "punch", "dab"),
             ("weight", "press", "glide"), ("space", "glide", "float"), ("space", "punch", "slash")]
    ok_n = 0
    for axis, hi_act, lo_act in pairs:
        a, b = measured(hi_act), measured(lo_act)
        good = a and b and a[axis] > b[axis]
        ok_n += bool(good)
        print("       %-6s %s %.2f vs %s %.2f %s" % (axis, hi_act, a[axis] if a else float("nan"), lo_act,
                                                  b[axis] if b else float("nan"), "" if good else "  <-- wrong way"))
    check("measured Weight / Time / Space order as the intent says", ok_n >= 5, "%d/6 pairs" % ok_n)
    fb, ff = measured("glide", flow=-1.0), measured("glide", flow=1.0)
    check("bound flow measures more bound than free flow", fb and ff and fb["flow"] < ff["flow"],
          "%.2f vs %.2f" % (fb["flow"], ff["flow"]))
    agree = sum(1 for a, c in clips.items() if c.get("labels") and c["labels"]["measured"]["action"] == a)
    check("the measured Laban action matches the intended one (8 clips from HOME, at acc %s)" % kin.acc, agree >= 7,
          "%d/8; %s" % (agree, ", ".join("%s->%s" % (a, c["labels"]["measured"]["action"]) for a, c in clips.items()
                                         if c.get("labels") and c["labels"]["measured"]["action"] != a)))
    rows = calibration_set(kin, seeds=range(20, 24))
    conf, n = L.confusion(rows)
    L.print_confusion(conf)
    check("... and single-action phrases (with dynamics) from HOME, the rest hub and a low hub, seeds the "
          "labels were not fitted on: >= 7/8 actions by majority, >= 85% of phrases",
          L.majority(conf)[1] >= 7 and n >= 0.85 * len(rows),
          "%d/8 actions, %d/%d phrases" % (L.majority(conf)[1], n, len(rows)))
    mix = make_clip({"bars": [{"action": "float"}, {"action": "punch"}, {"action": "glide"}], "bpm": 90, "flow": 0.3},
                    seed=4, env=env, kin=kin)
    check("a mixed phrase (float -> punch -> glide) is made and labelled",
          mix["safety"].get("ok") and mix.get("labels"), str(mix["safety"].get("reasons")))

    # ---- the animation principles -------------------------------------------
    rest = CAL_STARTS["rest"]
    on, off = {}, {}
    for act in ACTIONS:
        for pr, out in ((True, on), (False, off)):
            spec = {"bars": [{"action": act}] * 2, "bpm": 80 if ACTIONS[act][1] < 0 else 115, "flow": 0.0,
                    "start": rest, "principles": pr}
            out[act] = phrase(spec, seed=5, kin=kin)
    bad = []
    for act, (ts, qs, info) in on.items():
        if ts is None:
            bad.append("%s: no phrase" % act)
            continue
        pose = max(max(abs(a - b) for a, b in zip(qs[0], rest)), max(abs(a - b) for a, b in zip(qs[-1], rest)))
        still = max(max(abs(a - b) for a, b in zip(qs[1], qs[0])), max(abs(a - b) for a, b in zip(qs[-1], qs[-2])))
        if pose > 1e-3 or still > 1e-3:
            bad.append("%s: %.1e deg from the pose, %.1e deg/frame at an end" % (act, pose, still))
    check("with the principles, every phrase starts and ends at rest at its start pose (1e-3 deg)", not bad,
          "; ".join(bad))
    bad = []
    for act, (ts, qs, info) in on.items():
        if ts is None:
            continue
        lim = P.limiting(ts, qs, 125.0, [v * PLAN_SAFETY for v in kin.vel], [a * PLAN_SAFETY for a in kin.acc])
        margin = min(min(x - lo, hi - x) for q in qs for x, (lo, hi) in zip(q, kin.limits))
        if lim["scale_needed"] > 1.0 or margin < LIMIT_MARGIN:
            bad.append("%s: x%.3f (J%d %s), %.1f deg inside the joint limits" % (
                act, lim["scale_needed"], lim["joint"], lim["kind"], margin))
    check("... within the speed / acceleration limits at PLAN_SAFETY and LIMIT_MARGIN inside the joint limits",
          not bad, "; ".join(bad))
    # anticipation and overshoot: the same plan with and without (the rng is
    # drawn alike), J1-J3 projected on the first sudden travel's direction
    spec = {"bars": [{"action": "punch"}] * 2, "bpm": 110, "flow": 0.0, "start": rest}
    segs, total = _plan(spec, kin, random.Random(3))
    ts_a, qs_a = _sample(segs, total, 1.0, 0.0, 1.0, rest, 1.0)
    _, qs_b = _sample(segs, total, 1.0, 0.0, 1.0, rest, 0.0)
    tr = next((s for s in segs if s.get("ant")), None)
    if tr:
        dv = [b - a for a, b in zip(tr["from"][:3], tr["to"][:3])]
        nd = math.sqrt(sum(x * x for x in dv)) or 1e-9
        proj = [sum((x - y) * d for x, y, d in zip(qa[:3], qb[:3], dv)) / nd for qa, qb in zip(qs_a, qs_b)]
        before = [p_ for t, p_ in zip(ts_a, proj) if tr["t0"] - tr["beat"] <= t <= tr["t0"]]
        during = [p_ for t, p_ in zip(ts_a, proj) if tr["t0"] <= t <= tr["t1"]]
        check("a punch winds up against its strike and overshoots past it (vs the same plan without)",
              min(before) < -0.05 * nd and max(during) > 0.04 * nd,
              "wind-up %.1f deg, overshoot %.1f deg of a %.1f deg strike" % (-min(before), max(during), nd))
    else:
        check("a punch winds up against its strike and overshoots past it", False, "no sudden travel in the plan")
    # follow-through: in every travel (where the arm and the wrist share one
    # move) the J4-J6 speed profile trails J1-J3's -- and by more than in the
    # same plan without the principles (detours alone shift it a little)
    lags, gain = {}, []
    for act in ACTIONS:
        spec = {"bars": [{"action": act}] * 2, "bpm": 80 if ACTIONS[act][1] < 0 else 115, "flow": 0.0, "start": rest}
        segs, total = _plan(spec, kin, random.Random(5))
        ts_a, qs_a = _sample(segs, total, 1.0, 0.0, 1.0, rest, 1.0)
        _, qs_b = _sample(segs, total, 1.0, 0.0, 1.0, rest, 0.0)
        # (not under a level carriage: that is a second move at the same time)
        carried = [s for s in segs if s["kind"] == "travel_level"]
        for sg in segs:
            if "to" in sg and not any(c["t0"] - 0.3 < sg["t1"] and sg["t0"] - 0.3 < c["t1"] for c in carried):
                idx = [i for i, t in enumerate(ts_a) if sg["t0"] - 0.3 <= t <= sg["t1"]]
                la = L.successive_lag([ts_a[i] for i in idx], [qs_a[i] for i in idx])
                lb = L.successive_lag([ts_a[i] for i in idx], [qs_b[i] for i in idx])
                if la != 0.0 and lb != 0.0:                    # 0.0: one of the groups hardly moves
                    lags.setdefault(act, []).append(la)
                    gain.append(la - lb)
    flat = [x for v in lags.values() for x in v]
    check("follow-through: in every travel J4-J6 trail J1-J3, more than without the principles",
          len(flat) >= 8 and min(flat) > 0.0 and min(gain) >= 0.0 and sorted(gain)[len(gain) // 2] > 0.02,
          "%d travels, lag %.3f-%.3f s, %.3f-%.3f s more than without; median per action %s" % (
              len(flat), min(flat), max(flat), min(gain), max(gain),
              ", ".join("%s %.3f" % (a, sorted(v)[len(v) // 2]) for a, v in lags.items())))

    # breathing: no frame between the ends is fully still
    def still_frames(ts, qs, eps=0.02):
        """Frames (0.25 s from either end) where no joint moves eps deg/s."""
        return sum(1 for i in range(len(qs) - 1) if 0.25 <= ts[i] <= ts[-1] - 0.25
                   and max(abs(b - a) for a, b in zip(qs[i], qs[i + 1])) * FPS < eps)
    s_on = {a: still_frames(v[0], v[1]) for a, v in on.items() if v[0]}
    s_off = {a: still_frames(v[0], v[1]) for a, v in off.items() if v[0]}
    check("breathing: with the principles no frame is still (> 0.02 deg/s) in the holds; without, the holds freeze",
          all(x == 0 for x in s_on.values()) and sum(s_off.values()) > 0,
          "still frames with %d, without %d" % (sum(s_on.values()), sum(s_off.values())))
    labels_on = {a: L.efforts(L.raw_measures(v[0], v[1], M._tcp_path("fr20", v[1])))["action"]
                 for a, v in on.items() if v[0]}
    check("... and the principles keep the labels: the 8 phrases above still measure as intended (>= 7/8)",
          sum(a == b for a, b in labels_on.items()) >= 7,
          ", ".join("%s->%s" % (a, b) for a, b in labels_on.items() if a != b))

    # ---- dynamics: levels, sizes, tempo, accents -------------------------------
    # a sweep as the show makes a hub's phrases (the rest hub and HOME), in the room
    t1 = time.time()
    sw = hub_sweep(kin, env, n=20)
    st = sweep_stats(sw)
    print("       sweep: %d/%d kept, z span median %.2f (%.0f%% >= 0.3 m), extent median %.2f, %.1f-%.1f s "
          "(%.0f%% <= 5 s), peak TCP speed %.0f%% >= 1.2 m/s, %.0f%% < 0.4, %d-%d bpm, %.0f s" % (
              st["kept"], st["tried"], st["zspan"], 100 * st["zspan_03"], st["extent"], st["dur_range"][0],
              st["dur_range"][1], 100 * st["short"], 100 * st["fast"], 100 * st["calm"], st["bpm_range"][0],
              st["bpm_range"][1], time.time() - t1))
    for r in sw:
        if not r["ok"]:
            print("       dropped %s %s: %s" % (r["hub"], "-".join(b["action"] for b in r["spec"]["bars"]), r["why"]))
    check("hub sweep: >= 80% of the phrases are made, 3-12 s, TCP 0.35-1.6 m high and clear of the room",
          st["kept"] >= 0.8 * st["tried"], "%d/%d" % (st["kept"], st["tried"]))
    bad = []
    for r in sw:
        if "qs" not in r:
            continue
        ts, qs = r["ts"], r["qs"]
        start = r["hub_q"]
        lim = P.limiting(ts, qs, 125.0, [v * PLAN_SAFETY for v in kin.vel], [a * PLAN_SAFETY for a in kin.acc])
        margin = min(min(x - lo, hi - x) for q in qs for x, (lo, hi) in zip(q, kin.limits))
        pose = max(max(abs(a - b) for a, b in zip(qs[0], start)), max(abs(a - b) for a, b in zip(qs[-1], start)))
        still = max(max(abs(a - b) for a, b in zip(qs[1], qs[0])), max(abs(a - b) for a, b in zip(qs[-1], qs[-2])))
        if lim["scale_needed"] > 1.0 or margin < LIMIT_MARGIN or pose > 1e-3 or still > 1e-3:
            bad.append("%s: x%.3f, %.1f deg inside, %.1e deg from the pose, %.1e deg/frame at an end" % (
                "-".join(b["action"] for b in r["spec"]["bars"]), lim["scale_needed"], margin, pose, still))
    check("... every one within the limits at PLAN_SAFETY, LIMIT_MARGIN inside, at rest at its hub at both ends "
          "(1e-3 deg)", not bad, "; ".join(bad[:4]))
    check("levels: the TCP height changes >= 0.2 m in the median phrase, >= 0.3 m in >= 30% of them",
          st["zspan"] >= 0.2 and st["zspan_03"] >= 0.3, "median %.2f m, %.0f%%" % (st["zspan"], 100 * st["zspan_03"]))
    check("size: the median phrase spans >= 0.35 m", st["extent"] >= 0.35, "%.2f m" % st["extent"])
    check("rhythm: durations spread over 3-12 s, >= 20% of them accents of <= 5 s, 70-140 bpm",
          st["dur_range"][0] >= 3.0 - 1e-9 and st["dur_range"][1] <= 12.0 + 1e-9 and st["short"] >= 0.2
          and st["bpm_range"][0] <= 80 and st["bpm_range"][1] >= 125,
          "%.1f-%.1f s, %.0f%% <= 5 s, %d-%d bpm" % (st["dur_range"] + (100 * st["short"],) + st["bpm_range"]))
    check("energy: >= 10% of the phrases peak >= 1.2 m/s at the tool, >= 10% stay under 0.4 m/s",
          st["fast"] >= 0.1 and st["calm"] >= 0.1, "%.0f%% / %.0f%%" % (100 * st["fast"], 100 * st["calm"]))
    kept = [r for r in sw if r["ok"]]
    strong_sudden = [r["v_peak"] for r in kept if all(ACTIONS[b["action"]][:2] == (1, 1) for b in r["spec"]["bars"])]
    light_sust = [r["v_peak"] for r in kept if all(ACTIONS[b["action"]][:2] == (-1, -1) for b in r["spec"]["bars"])]
    mid = lambda xs: sorted(xs)[len(xs) // 2] if xs else float("nan")  # noqa: E731
    check("amplitude follows the effort: strong sudden phrases peak >= 2x faster than light sustained ones",
          strong_sudden and light_sust and mid(strong_sudden) >= 2.0 * mid(light_sust),
          "median %.2f vs %.2f m/s (%d / %d phrases)" % (mid(strong_sudden), mid(light_sust), len(strong_sudden),
                                                        len(light_sust)))
    single = [r for r in kept if len(set(b["action"] for b in r["spec"]["bars"])) == 1]
    agree = sum(r["action"] == r["spec"]["bars"][0]["action"] for r in single)
    check("... and the sweep's single-action phrases measure as intended (>= 75%)",
          single and agree >= 0.75 * len(single), "%d/%d" % (agree, len(single)))
    off = sweep_stats(hub_sweep(kin, None, n=12, dynamics=False))
    check("dynamics=False: the phrases as before -- they change level half as much or less",
          off["zspan"] <= 0.5 * st["zspan"], "z span median %.2f vs %.2f m" % (off["zspan"], st["zspan"]))

    # a level contour: press sinks low then rises high; a glide sinks and stays light
    rest_l = [0.0] + REST_HUB[1:]
    ts, qs, info = phrase({"bars": [{"action": "press"}, {"action": "press"}], "bpm": 90, "flow": 0.0, "start": rest_l,
                           "levels": ["low", "high"], "tempo": "steady", "syncopate": False}, seed=2, kin=kin)
    z = [c[2] for c in M._tcp_path("fr20", qs)] if qs else [1.1]
    check("levels: press low -> high goes below 0.85 m and above 1.3 m", min(z) < 0.85 and max(z) > 1.3,
          "TCP %.2f..%.2f m" % (min(z), max(z)))
    ts, qs, info = phrase({"bars": [{"action": "glide"}], "bpm": 80, "flow": 0.0, "start": rest_l, "levels": ["low"]},
                          seed=2, kin=kin)
    tcp = M._tcp_path("fr20", qs) if qs else [[0, 0, 1.1]]
    z = [c[2] for c in tcp]
    act = L.efforts(L.raw_measures(ts, qs, tcp))["action"] if qs else None
    check("... a glide sinks >= 0.2 m and still reads glide", max(z) - min(z) >= 0.2 and act == "glide",
          "%.2f m, %s" % (max(z) - min(z), act))
    # tempo curves, the freeze, syncopation (plans only)
    beats = {}
    for tempo in ("accel", "rit"):
        segs, _ = _plan({"bars": [{"action": "dab"}] * 2, "bpm": 90, "flow": 0.0, "tempo": tempo, "syncopate": False},
                        kin, random.Random(1))
        mv = [s["beat"] for s in segs if s["kind"] not in ("travel_level", "travel_home", "hold")]
        beats[tempo] = mv
    check("tempo: an accelerando's beats shorten, a ritardando's lengthen",
          beats["accel"][0] > beats["accel"][-1] * 1.3 and beats["rit"][0] * 1.3 < beats["rit"][-1],
          "accel %s, rit %s" % ([round(b, 2) for b in beats["accel"]], [round(b, 2) for b in beats["rit"]]))
    beat = 60.0 / 110
    segs, total = _plan({"bars": [{"action": "punch"}] * 2, "bpm": 110, "flow": 0.0, "tempo": "freeze",
                         "syncopate": False, "start": rest_l}, kin, random.Random(4))
    frz = [s for s in segs if s.get("quiet")]
    after = [s for s in segs if frz and s["t0"] >= frz[0]["t1"] - 1e-9 and s["kind"] not in ("travel_level", "hold")]
    ts, qs = _sample(segs, total, 1.0, 0.0, 1.0, rest_l, 1.0)
    tcp = M._tcp_path("fr20", qs)
    held = [math.dist(tcp[i + 1], tcp[i]) * FPS for i, t in enumerate(ts[:-1])
            if frz and frz[0]["t0"] + 0.3 <= t <= frz[0]["t1"] - 0.05]
    check("... a freeze holds two beats (the TCP under 2 cm/s) and the moves after it burst at 0.75 of the beat",
          frz and abs(frz[0]["t1"] - frz[0]["t0"] - 2 * beat) < 1e-9 and held and max(held) < 0.02
          and after and all(abs(s["beat"] - 0.75 * beat) < 1e-9 for s in after),
          "hold %.2f s, TCP up to %.3f m/s, %d moves after" % (frz[0]["t1"] - frz[0]["t0"] if frz else 0,
                                                              max(held) if held else -1, len(after)))
    segs, _ = _plan({"bars": [{"action": "punch"}] * 2, "bpm": 110, "flow": 0.0, "tempo": "steady", "syncopate": True},
                    kin, random.Random(4))
    moves = [s for s in segs if s["kind"] not in ("travel_level", "hold", "travel_home")]
    off_beat = [s for s in moves if abs(((s["t0"] / beat) % 1.0) - 0.5) < 1e-6]
    check("... syncopation puts one move on the off-beat (half a beat late), the others on the beat",
          len(off_beat) == 1 and all(abs(((s["t0"] / beat) + 1e-9) % 1.0) < 1e-6 for s in moves if s not in off_beat),
          "%d off-beat of %d" % (len(off_beat), len(moves)))
    print("\n%.0f s" % (time.time() - t0))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
