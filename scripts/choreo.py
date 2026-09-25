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

Every phrase starts and ends at rest in HOME, so clips chain. A clip must
play at its designed speed (fairino_player's measure) and clear the cell
(collision.py): intensity is reduced first -- smaller oscillations and
detours, softer attacks, so the beat is kept -- and only then the tempo.

    phrase(spec, seed)            -> (times, joints, info)     24 fps frames
    make_clip(spec, seed, env)    -> motion_clip dict with labels
    random_spec(rng, ...)         -> a phrase spec

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
HOME = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]        # upper arm up, forearm forward, tool down
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
# poses in the kinesphere
# --------------------------------------------------------------------------

class Kin:
    def __init__(self):
        self.model, self.chain, self.fo, self.vel = CAP.load_fr20(ROOT)
        prof = RP.load("fr20")
        self.acc = RP.acceleration_limits(prof)
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


def _target(kind, eff, act, cur, kin, rng, tries=16):
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
            q = [c + 0.25 * (h - c) + rng.uniform(-s_, s_) for c, h, s_ in zip(cur, HOME, span)]
        else:
            reach = 1.25 + 0.15 * eff["weight"] + rng.uniform(-0.15, 0.1)
            q = kin.pose(rng.choice(("low", "mid", "mid", "high")), rng.choice(list(DIRECTIONS)),
                         reach, AIMS[act], cur, rng)
            if q is None or max(abs(a - b) for a, b in zip(q, cur)) > 150:
                continue
            if sudden:
                # a jab towards that pose: at 150 deg/s^2 a heavy arm cannot
                # be quick over a big distance, so a sudden move is a short one
                jab = 7.0 + 3.0 * eff["weight"]
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
        w = 2 * math.pi / dur
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
    t = beat                                                  # a beat of stillness first
    cur = list(HOME)
    segs = []
    for bar in spec["bars"]:
        act = bar["action"]
        eff = efforts_of(act, flow)
        moves = list(MENU[act])
        rng.shuffle(moves)
        for kind in moves[:2]:
            # oscillators: a whole bar when sustained, half when sudden;
            # travels: two beats, more if the distance needs it
            nb = (4 if eff["time"] < 0 else 2) if kind in OSCILLATORS else 2
            t0, t1 = t, t + nb * beat
            seg = {"t0": t0, "t1": t1, "kind": kind, "eff": eff, "act": act}
            if kind.startswith("travel"):
                target = _target(kind, eff, act, cur, kin, rng)
                if target is None:
                    seg["kind"] = "hold"
                else:
                    seg["from"], seg["to"] = list(cur), target
                    seg["detour"] = [rng.uniform(-1, 1) for _ in range(6)]
                    a = _attack(eff["time"], 1.0)
                    need = _travel_time(kin, cur, target) * 1.25 / a      # + room for detour / overshoot
                    nb = max(nb, int(math.ceil(need / beat - 1e-9)))
                    seg["t1"] = t1 = t0 + nb * beat
                    seg["acc_min"] = min(kin.acc)
                    cur = target
            elif kind == "look":
                seg["dir"] = [rng.uniform(-1, 1), rng.uniform(-1, 1)]
            seg["phase"] = rng.uniform(0, 2 * math.pi)
            seg["beat"] = beat
            seg["cap"] = _caps(seg, kin, beat)
            segs.append(seg)
            t = t1
            if flow < -0.3:                                    # bound: a held beat after every move
                segs.append({"t0": t, "t1": t + beat, "kind": "hold", "eff": eff, "act": act,
                             "phase": 0.0, "beat": beat, "cap": [1e9] * 6})
                t += beat
    # home again, then a beat of stillness
    nb = max(2, int(math.ceil(_travel_time(kin, cur, HOME) * 1.1 / beat - 1e-9)))
    segs.append({"t0": t, "t1": t + nb * beat, "kind": "travel_home", "from": list(cur), "to": list(HOME),
                 "eff": efforts_of("glide", flow), "act": "glide", "detour": [0.0] * 6, "phase": 0.0,
                 "beat": beat, "cap": [1e9] * 6})
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
        off[0] += A(0, 16.0) * s * env
        off[5] -= A(5, 13.0) * s * env
        off[3] += A(3, 5.0) * math.sin(4 * math.pi * u + ph) * env
    elif kind == "twist":
        s = math.sin(2 * math.pi * u + ph)
        off[3] += A(3, 22.0) * s * env
        off[5] -= A(5, 30.0) * s * env
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


def _travel(seg, tt, k, flow):
    """(progress of the pose change, detour offsets) for a travel at tt."""
    e = seg["eff"]
    dur = seg["t1"] - seg["t0"]
    # free flow starts the move a quarter-beat early and lets it overlap
    lead = 0.12 * dur * max(0.0, flow)
    u = (tt - seg["t0"] + lead) / (dur + lead)
    a = _attack(e["time"], k)
    if flow < -0.3:
        a = min(a, 0.8)                                       # bound: arrive, then be still
    s = _minjerk(u / a)
    det = [0.0] * 6
    if 0.0 < u < a:
        # sin^2: the detour starts and ends at rest, as the travel does (a
        # half-sine ended with a slope where the travel stopped -- a velocity
        # kink that read as 9x the acceleration limit)
        bump = math.sin(math.pi * u / a) ** 2
        size = (1 - e["space"]) / 2.0 * 14.0 * k * (0.7 + 0.3 * e["weight"])
        if seg["kind"] == "travel_curve":
            size = max(size, 10.0 * k)
        ta = a * (dur + lead)
        size = min(size, 0.3 * seg.get("acc_min", 135.0) * ta * ta / (2 * math.pi ** 2))
        det = [size * d * bump for d in seg["detour"][:5]] + [0.0]
    return s, det


def _sample(segs, total, k, flow):
    ts = [i / FPS for i in range(int(math.ceil(total * FPS)) + 1)]
    qs = []
    for tt in ts:
        q = list(HOME)
        for seg in segs:
            if "to" in seg:
                s, det = _travel(seg, tt, k, flow)
                q = [a + s * (b - c) + d for a, b, c, d in zip(q, seg["to"], seg["from"], det)]
        for seg in segs:
            off = _offsets(seg, tt, k)
            if off:
                q = [a + b for a, b in zip(q, off)]
        qs.append(q)
    return ts, qs


def phrase(spec, seed=0, env=None, kin=None, safety=0.9, max_tries=6):
    """24 fps frames for a phrase spec: (times, joints, info). info: the
    intensity kept, tempo, attempts, and why a try was dropped."""
    kin = kin or Kin()
    rng = random.Random(seed)
    info = {"tries": [], "bpm": spec["bpm"]}
    for attempt in range(max_tries):
        segs, total = _plan(spec, kin, rng)
        k, bpm_scale = 1.0, 1.0
        for _ in range(8):
            ts, qs = _sample(segs, total, k, spec.get("flow", 0.0))
            ts = [x * bpm_scale for x in ts]
            if any(not lo + 1.0 < x < hi - 1.0 for q in qs for x, (lo, hi) in zip(q, kin.limits)):
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
                         "action": s["act"]} for s in segs], attempt=attempt)
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
    clip["style"].update({k: info[k] for k in ("intensity", "tempo_scale", "bpm_played", "segments")})
    M.measure(clip)
    if env is not None:
        clip["safety"]["collision"] = CL.describe(info["collision"])
        clip["safety"]["min_clearance_m"] = info["collision"]["min_clearance_m"]
    clip["labels"] = L.label(clip, intent=spec)
    return clip


if __name__ == "__main__":
    import time
    import motion_labels as L

    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    kin = Kin()
    env = CL.load_env(os.path.join(ROOT, "envs", "volvox_lab.json"))
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
    check("the measured Laban action matches the intended one on most actions", agree >= 5, "%d/8" % agree)
    mix = make_clip({"bars": [{"action": "float"}, {"action": "punch"}, {"action": "glide"}], "bpm": 90, "flow": 0.3},
                    seed=4, env=env, kin=kin)
    check("a mixed phrase (float -> punch -> glide) is made and labelled",
          mix["safety"].get("ok") and mix.get("labels"), str(mix["safety"].get("reasons")))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
