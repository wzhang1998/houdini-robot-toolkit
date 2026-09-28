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

Every phrase starts and ends at rest in HOME, so clips chain -- or at the
pose in spec["start"] (a show's hub pose). A clip must
play at its designed speed (fairino_player's measure) and clear the cell
(collision.py): intensity is reduced first -- smaller oscillations and
detours, softer attacks, so the beat is kept -- and only then the tempo.

    phrase(spec, seed)            -> (times, joints, info)     24 fps frames
    make_clip(spec, seed, env)    -> motion_clip dict with labels
    random_spec(rng, ...)         -> a phrase spec
    calibration_set(kin, seeds)   -> single-action phrases for motion_labels.fit_cal

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
        self.limits = [tuple(x) for x in prof["robot"]["limits_deg"]]
        self.col = CL.load_model("fr20")

    def tcp(self, q):
        _, t = CL.capsules(self.col, q)
        return t

    def pose(self, level, direction, reach, aim, near, rng):
        """Joint pose putting the TCP at the kinesphere point, tool aimed;
        the IK branch nearest `near`. None when unreachable."""
        az = math.radians(DIRECTIONS[direction] + rng.uniform(-10, 10))
        r = reach
        z = LEVELS[level] + rng.uniform(-0.1, 0.1)
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


def random_spec(rng, bars=None, actions=None, flow=None, bpm=None):
    """A phrase: 2-4 bars, one action each (repeats and contrasts), a tempo
    that suits the actions, a flow."""
    n = bars or rng.choice((2, 3, 3, 4))
    names = list(ACTIONS)
    acts = actions or [rng.choice(names) for _ in range(n)]
    sudden = sum(ACTIONS[a][1] > 0 for a in acts) / float(len(acts))
    return {"bars": [{"action": a} for a in acts],
            "bpm": bpm or int(round(rng.uniform(60, 80) + 45 * sudden)),
            "flow": flow if flow is not None else round(rng.uniform(-1, 1), 2)}


def _target(kind, eff, act, cur, kin, rng, tries=16, home=None):
    """Where a travel goes. Strong actions (travel_far / travel_curve): a
    kinesphere pose by IK -- the whole arm moves. Light ones (travel_near):
    the wrist leads -- J4-J6 turn, J2/J3 follow a little, J1 hardly. Every
    candidate is checked alone against self-collision and the floor."""
    floor = {"schema": CL.ENV_SCHEMA, "margin_m": 0.05, "objects": [
        {"name": "floor", "type": "halfspace", "normal": [0, 0, 1], "offset": -0.02, "role": "obstacle"},
        {"name": "base_plate", "type": "box", "center": [0.0, 0.0, -0.01], "size": [1.2, 1.2, 0.02], "role": "obstacle"}]}
    for _ in range(tries):
        sudden = eff["time"] > 0
        if kind == "travel_near":
            # drift back towards HOME as well, so light phrases stay centred
            span = [3, 6, 10, 30, 25, 45]
            if sudden:
                span = [x * 0.4 for x in span]
            q = [c + 0.25 * (h - c) + rng.uniform(-s_, s_) for c, h, s_ in zip(cur, home or HOME, span)]
        else:
            reach = 1.25 + 0.15 * eff["weight"] + rng.uniform(-0.15, 0.1)
            q = kin.pose(rng.choice(("low", "mid", "mid", "high")), rng.choice(list(DIRECTIONS)),
                         reach, AIMS[act], cur, rng)
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
                jab = (7.0 + 3.0 * eff["weight"]) * min(3.0, max(1.0, min(kin.acc) / 150.0))
                d = max(abs(a - b) for a, b in zip(q, cur))
                if d > jab:
                    q = [c + (x - c) * jab / d for c, x in zip(cur, q)]
        if not all(lo + LIMIT_MARGIN < x < hi - LIMIT_MARGIN for x, (lo, hi) in zip(q, kin.limits)):
            continue
        if abs(math.sin(math.radians(q[4]))) < 0.25:
            continue
        if not CL.check(kin.col, floor, [0.0], [q])["ok"]:
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


def _plan(spec, kin, rng):
    """Segments: dicts with t0, t1, kind, params. Returns (segments, poses).
    A travel takes the fewest whole beats its distance allows at the joint
    limits (and at its attack), so the phrase stays on the beat grid."""
    beat = 60.0 / spec["bpm"]
    flow = spec.get("flow", 0.0)
    p = _principles(spec)
    t = beat                                                  # a beat of stillness first
    home = list(spec.get("start") or HOME)
    cur = list(home)
    segs = []
    for bi, bar in enumerate(spec["bars"]):
        act = bar["action"]
        eff = efforts_of(act, flow)
        moves = list(MENU[act])
        rng.shuffle(moves)
        for kind in moves[:2]:
            # oscillators: a whole bar when sustained, half when sudden;
            # travels: two beats, more if the distance needs it
            nb = (4 if eff["time"] < 0 else 2) if kind in OSCILLATORS else 2
            t0, t1 = t, t + nb * beat
            seg = {"t0": t0, "t1": t1, "kind": kind, "eff": eff, "act": act, "bar": bi}
            if kind.startswith("travel"):
                target = _target(kind, eff, act, cur, kin, rng, home=home)
                if target is None:
                    seg["kind"] = "hold"
                else:
                    seg["from"], seg["to"] = list(cur), target
                    seg["detour"] = [rng.uniform(-1, 1) for _ in range(6)]
                    a = _attack(eff["time"], 1.0)
                    # + room for detour / overshoot: the principles' wind-up and
                    # overshoot (<= 0.27 of the distance) fit in it, so a strike
                    # stays as quick as a plain travel
                    need = _travel_time(kin, cur, target) * 1.25 / a
                    seg["ant"], seg["over"] = _anticipation(eff, p)
                    nb = max(nb, int(math.ceil(need / beat - 1e-9)))
                    seg["t1"] = t1 = t0 + nb * beat
                    seg["acc_min"] = min(kin.acc)
                    cur = target
            elif kind == "look":
                seg["dir"] = [rng.uniform(-1, 1), rng.uniform(-1, 1)]
            seg["phase"] = rng.uniform(0, 2 * math.pi)
            seg["beat"] = beat
            seg["cap"] = _caps(seg, kin, beat)
            seg["lag"] = _lag(eff, p)
            if kind in ("twist", "sway"):
                seg["succ"] = p * math.pi / 3.0                # phase the hand trails by
            segs.append(seg)
            t = t1
            if flow < -0.3:                                    # bound: a held beat after every move
                segs.append({"t0": t, "t1": t + beat, "kind": "hold", "eff": eff, "act": act, "bar": bi,
                             "phase": 0.0, "beat": beat, "cap": [1e9] * 6, "lag": 0.0})
                t += beat
    # home again, then a beat of stillness
    # bound flow arrives in 0.8 of the travel (see _travel): plan for that
    a_home = 0.8 if flow < -0.3 else 1.0
    nb = max(2, int(math.ceil(_travel_time(kin, cur, home) * 1.1 / a_home / beat - 1e-9)))
    glide = efforts_of("glide", flow)
    segs.append({"t0": t, "t1": t + nb * beat, "kind": "travel_home", "from": list(cur), "to": list(home),
                 "eff": glide, "act": "glide", "detour": [0.0] * 6, "phase": 0.0,
                 "beat": beat, "cap": [1e9] * 6, "lag": _lag(glide, p)})
    return segs, t + (nb + 1) * beat


def _offsets(seg, tt, k):
    """Joint offsets (degrees) an oscillating move adds at time tt. Each
    joint's amplitude is the design one, capped by what the joint can
    accelerate at this move's frequency (seg["cap"], from _caps)."""
    u = (tt - seg["t0"]) / (seg["t1"] - seg["t0"])
    if u <= 0.0 or u >= 1.0:
        return None
    e = seg["eff"]
    dur = seg["t1"] - seg["t0"]
    amp = (0.65 + 0.35 * e["weight"]) * k                     # light 0.3 .. strong 1.0
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
        if e["flow"] > 0.2:                                   # free: breathe
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
    lead = 0.12 * dur * max(0.0, flow)
    a = _attack(e["time"], k)
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
    for attempt in range(max_tries):
        segs, total = _plan(spec, kin, rng)
        k, bpm_scale = 1.0, 1.0
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
        if env is not None:
            rep = CL.check(kin.col, env, ts, qs)
            if not rep["ok"]:
                info["tries"].append(CL.describe(rep))
                continue
            info["collision"] = rep
        info.update(intensity=round(k, 3), tempo_scale=round(bpm_scale, 3),
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
    clip["style"].update({k: info[k] for k in ("intensity", "tempo_scale", "bpm_played", "segments", "principles")})
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
    rows = calibration_set(kin, seeds=range(20, 24), starts={"rest": CAL_STARTS["rest"]})
    conf, n = L.confusion(rows)
    L.print_confusion(conf)
    check("... and from the party's rest hub, several seeds: >= 7/8 actions by majority",
          L.majority(conf)[1] >= 7, "%d/8 actions, %d/%d phrases" % (L.majority(conf)[1], n, len(rows)))
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
        for sg in segs:
            if "to" in sg:
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
    print("\n%.0f s" % (time.time() - t0))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
