"""Labels for a motion clip: what it is, measured from the motion itself.

A library is only searchable if every clip is described the same way,
whether it came from choreo.py, the primitive factory or a Houdini export.
So the labels are MEASURED from the joints and TCP path; a generator's own
intent is kept beside them, and whether the two agree is itself a label.

    measured     Laban Effort, each -1 .. +1, and the nearest effort action
      weight     strong (+) .. light (-): speed of the proximal joints
                 (J1-J3, mass-weighted) -- the whole arm moving reads strong,
                 a quick wrist does not
      time       sudden (+) .. sustained (-): acceleration over speed while
                 moving, i.e. how quickly speed changes relative to how
                 fast it goes; judged per weight class
      space      direct (+) .. indirect (-): chord / arc between stops, of
                 the TCP path or of the (mass-weighted) joint path,
                 whichever is straighter; judged per weight class
      flow       free (+) .. bound (-): how much of the clip is spent still
      action     nearest of punch / slash / press / wring / dab / flick /
                 glide / float by (weight, time, space)
    descriptors  duration, TCP speed / path / extent, level (low / mid /
                 high share), direction from the robot's own view, dominant
                 joints, accents per minute, successive lag (how far the
                 wrist trails the arm), loopable, start / end pose
    tags         short words for search: the action, level, fast / slow,
                 whole-arm / wrist-led, loop
    intent       what made it (the phrase spec), when known

Below STILL_DEG_S / STILL_TCP_M_S the arm counts as still (a breath is not a
move). The centres (CAL) are fitted, not hand-set: fit_cal() on
choreo.calibration_set(); `--calibrate` refits and prints them.

label(clip, intent=None) -> dict. Pure Python.
Tests: python scripts/motion_labels.py (calibration, measures);
python scripts/choreo.py (labels against the generator's intent).
Refit:  python scripts/motion_labels.py --calibrate
"""

import math

try:
    import robot_profile as _RP
    HOME = _RP.home(_RP.load("fr20")) or [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
except Exception:                               # labels still work without the profiles
    HOME = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
MASS = [4.0, 4.0, 2.5, 1.0, 1.0, 0.6]          # rough share of the arm each joint moves
# below these the arm reads as still, whatever the clip's own peak: a slow
# breath (choreo.py's idle, ~0.3 deg/s mass-weighted, < 1 cm/s at the tool)
# is life, not a movement -- it must not split a path or fill a hold
STILL_DEG_S = 0.5                               # mass-weighted joint speed
STILL_TCP_M_S = 0.012                           # tool speed
ACTIONS = {
    "punch": (1, 1, 1), "slash": (1, 1, -1), "press": (1, -1, 1), "wring": (1, -1, -1),
    "dab": (-1, 1, 1), "flick": (-1, 1, -1), "glide": (-1, -1, 1), "float": (-1, -1, -1),
}
# Centres and spreads of the raw measures: (centre, spread); a raw value at
# the centre is 0, one spread away +-0.76 (tanh). Weight and time are judged
# on log scales: they span an order of magnitude. Time and space have a
# centre per weight class, (light, strong, spread): a whole-arm move is
# acceleration-limited, so its moves are longer and its accel / speed lower
# than a wrist's -- sudden is judged against what that part of the body can
# do -- and a whole-arm move straight in its joints is judged on how
# straight it stays. The nearest action is the sign on each axis, so the
# centres decide the action; the spreads only scale the effort values (the
# show's energy reads "time").
#
# Fitted by fit_cal() on choreo.calibration_set() -- single-action phrases,
# every action, seeds 0-7, from HOME, the rest hub and a low hub, with the
# animation principles and the dynamics on, at the profile's acceleration
# limits (J1-J3 300, J4-J6 600 deg/s^2). Refit after changing choreo.py or
# the limits with
#   python scripts/motion_labels.py --calibrate
# and paste the printed CAL here -- or keep this one when a refit does not
# read held-out seeds better. 2026-09-27 (b), after choreo's dynamics
# (levels, sizes, tempo curves, accents), fitted mid-way through them and
# kept: on the final phrases 179/192 agree, 92/96 held out, 8/8 actions by
# majority; on seeds 12-27 (384 phrases) 360 agree, every action >= 43/48
# (punch -> press, slash / wring -> each other, flick -> float the main
# confusions). A refit then read 365/384 but dab and wring 41/48. The
# previous CAL read the new phrases 83/96, flick 5/12. The light class's
# space centre moved up (0.92 -> 0.99): a flick or float now rides a slow
# level carriage, which straightens its joint path, so only a dab (no
# level change under it) stays above it; the flow centre moved down
# (0.42 -> 0.16): fewer still frames. Both shift the labels of clips from
# other generators (gestures, paths) towards indirect and bound.
# History: 2026-09-27 (a) 185/192, 91/96 (principles on, no dynamics).
# 2026-09-24 hand-set at 150 deg/s^2 (weight 5, time 3.4, space 0.85, flow
# 0.4); at 300/600 that read float as flick / slash and slash as punch
# (109/144 phrases, float 7/18, slash 11/18).
CAL = {"weight": (1.8838, 0.871), "time": (1.3705, 1.0661, 0.3751), "space": (0.989, 0.9933, 0.0105),
       "flow": (0.1551, 0.147)}


def _pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, int(round(p * (len(s) - 1)))))] if s else 0.0


def raw_measures(ts, qs, tcp):
    n = len(ts)
    dt = [(ts[i + 1] - ts[i]) for i in range(n - 1)]
    qd = [[(qs[i + 1][j] - qs[i][j]) / dt[i] for j in range(6)] for i in range(n - 1)]
    w = sum(MASS)
    speed = [sum(m * abs(x) for m, x in zip(MASS, v)) / w for v in qd]
    # weight: the proximal joints only -- the whole arm moving reads strong;
    # a quick wrist (flick, float) does not, however fast it turns
    wp = sum(MASS[:3])
    prox = [sum(m * abs(x) for m, x in zip(MASS[:3], v[:3])) / wp for v in qd]
    acc = [sum(m * abs(qd[i + 1][j] - qd[i][j]) / dt[i] for j, m in enumerate(MASS)) / w for i in range(n - 2)]
    peak = _pct(speed, 0.9) or 1e-9
    moving = [s > max(0.1 * peak, STILL_DEG_S) for s in speed]
    # skip the rest at both ends: a beat of stillness is part of every phrase
    first = next((i for i, m in enumerate(moving) if m), 0)
    last = len(moving) - 1 - next((i for i, m in enumerate(reversed(moving)) if m), 0)
    inner = moving[first:last + 1] or [True]
    still = 1.0 - sum(inner) / float(len(inner))
    # time: over the moving frames only -- over all frames, a phrase with
    # holds had a low speed percentile and read sudden however slow its moves
    mv = [s for s, m in zip(speed, moving) if m]
    time_raw = _pct([a for a, m in zip(acc, moving) if m] or [0.0], 0.9) / (_pct(mv, 0.9) or peak)
    # space: directness between stops, of the TCP path and of the joint path
    # (mass-weighted); the straighter of the two -- a whole-arm move straight
    # in its joints bends the tool's path, and still reads single-minded
    vt = [math.dist(tcp[i + 1], tcp[i]) / dt[i] for i in range(n - 1)] if tcp else []
    direct = _directness(tcp, vt, max(0.08 * (_pct(vt, 0.9) or 1e-9), STILL_TCP_M_S), math.dist) if tcp else 0.0
    jdist = lambda a, b: math.sqrt(sum((m * (x - y)) ** 2 for m, x, y in zip(MASS, a, b)))  # noqa: E731
    jdirect = _directness(qs, speed, max(0.08 * peak, STILL_DEG_S), jdist)
    return {"weight": _pct(prox, 0.9), "time": time_raw, "space": max(direct, jdirect), "still": still,
            "tcp_directness": direct, "joint_directness": jdirect, "speed": speed, "tcp_speed": vt}


def _directness(pts, speed, floor, dist):
    """Chord / arc of a path, summed over its moves between stops (speed at
    or below floor)."""
    chord, arc, start, seg_arc = 0.0, 0.0, None, 0.0
    for i, v in enumerate(speed):
        if v > floor:
            if start is None:
                start, seg_arc = i, 0.0
            seg_arc += dist(pts[i + 1], pts[i])
        elif start is not None:
            chord += dist(pts[i], pts[start])
            arc += seg_arc
            start = None
    if start is not None:
        chord += dist(pts[-1], pts[start])
        arc += seg_arc
    return chord / arc if arc > 1e-6 else 1.0


def successive_lag(ts, qs, max_lag_s=0.3):
    """Seconds the distal joints (J4-J6) trail the proximal ones (J1-J3):
    the shift that best lines up their speed profiles (cross-correlation,
    refined between frames by a parabola through the peak). > 0: successive
    flow, the wrist following the arm through; ~0: every joint at once;
    0.0 when either group hardly moves."""
    n = len(ts)
    if n < 8:
        return 0.0
    dt = (ts[-1] - ts[0]) / (n - 1)
    qd = [[abs(qs[i + 1][j] - qs[i][j]) / dt for j in range(6)] for i in range(n - 1)]
    prox = [sum(v[:3]) for v in qd]
    dist = [sum(v[3:]) for v in qd]
    if max(prox) < 1.0 or max(dist) < 1.0:                    # deg/s
        return 0.0
    mp, md = sum(prox) / len(prox), sum(dist) / len(dist)
    p = [x - mp for x in prox]
    d = [x - md for x in dist]
    m = max(1, int(round(max_lag_s / dt)))
    corr = {}
    for lag in range(-m - 1, m + 2):
        lo, hi = max(0, -lag), min(len(p), len(d) - lag)
        corr[lag] = sum(p[i] * d[i + lag] for i in range(lo, hi)) / max(1, hi - lo)
    best = max(range(-m, m + 1), key=lambda k: corr[k])
    a, b, c = corr[best - 1], corr[best], corr[best + 1]
    den = a - 2 * b + c
    frac = 0.5 * (a - c) / den if den < 0 else 0.0
    return (best + max(-0.5, min(0.5, frac))) * dt


def _map(x, key, cal=None):
    c, s = (cal or CAL)[key]
    return max(-1.0, min(1.0, math.tanh((x - c) / s)))


def efforts(raw, cal=None):
    """Raw measures -> Laban efforts, each -1 .. +1, and the nearest action.
    Time and space are judged against the centre of the measured weight
    class when CAL gives one per class: (light centre, strong centre, spread)."""
    cal = cal or CAL
    w = _map(math.log(max(raw["weight"], 1e-6)), "weight", cal)

    def by_class(key):
        c = cal[key]
        return {key: (c[1] if w > 0 else c[0], c[2]) if len(c) == 3 else c}
    m = {"weight": round(w, 3),
         "time": round(_map(math.log(max(raw["time"], 1e-6)), "time", by_class("time")), 3),
         "space": round(_map(raw["space"], "space", by_class("space")), 3),
         "flow": round(-_map(raw["still"], "flow", cal), 3)}
    m["action"] = nearest_action(m["weight"], m["time"], m["space"])
    return m


def nearest_action(weight, time, space):
    return min(ACTIONS, key=lambda a: sum((x - y) ** 2 for x, y in zip(ACTIONS[a], (weight, time, space))))


# --------------------------------------------------------------------------
# calibration
# --------------------------------------------------------------------------

def _cut(pos, neg):
    """The cut between two groups of a raw measure (pos above it): fewest
    misclassified, each group weighted by its size (balanced); among equal
    cuts, the middle of the widest gap between neighbouring samples."""
    xs = sorted(set(pos) | set(neg))
    if len(xs) < 2:
        return xs[0] if xs else 0.0
    best = None
    for a, b in zip(xs, xs[1:]):
        c = (a + b) / 2.0
        err = sum(x < c for x in pos) / float(len(pos)) + sum(x > c for x in neg) / float(len(neg))
        key = (round(err, 9), -(b - a))
        if best is None or key < best[0]:
            best = (key, c)
    return best[1]


def _spread(xs, c, floor):
    """Median distance from the centre: a typical sample maps to +-0.76."""
    d = sorted(abs(x - c) for x in xs)
    return max(floor, d[len(d) // 2]) if d else floor


def fit_cal(rows, flow_split=0.3):
    """CAL from calibration rows [{"raw": raw_measures-like dict (weight,
    time, space, still), "action": intended action, "flow": intended flow}].
    Weight: the best cut between the intended groups. Time and space: a cut
    per weight class (rows split by their INTENDED weight). Flow: the cut
    between bound (flow < -flow_split) and free (> flow_split) phrases."""
    def split(key, axis, transform=lambda x: x, subset=None):
        rr = subset if subset is not None else rows
        pos = [transform(r["raw"][key]) for r in rr if ACTIONS[r["action"]][axis] > 0]
        neg = [transform(r["raw"][key]) for r in rr if ACTIONS[r["action"]][axis] < 0]
        return pos, neg

    def per_class(key, axis, transform, floor):
        centres, dev = [], []
        for sign in (-1, 1):
            sub = [r for r in rows if ACTIONS[r["action"]][0] == sign]
            pos, neg = split(key, axis, transform, sub)
            c = _cut(pos, neg)
            centres.append(round(c, 4))
            dev += [abs(x - c) for x in pos + neg]
        return (centres[0], centres[1], round(_spread(dev, 0.0, floor), 4))
    lg = lambda x: math.log(max(x, 1e-6))  # noqa: E731
    cal = {}
    pos, neg = split("weight", 0, lg)
    c = _cut(pos, neg)
    cal["weight"] = (round(c, 4), round(_spread(pos + neg, c, 0.1), 4))
    cal["time"] = per_class("time", 1, lg, 0.1)
    cal["space"] = per_class("space", 2, lambda x: x, 0.005)
    bound = [r["raw"]["still"] for r in rows if r.get("flow", 0.0) < -flow_split]
    free = [r["raw"]["still"] for r in rows if r.get("flow", 0.0) > flow_split]
    if bound and free:
        c = _cut(bound, free)                                      # bound is the stiller group
        cal["flow"] = (round(c, 4), round(_spread(bound + free, c, 0.05), 4))
    else:
        cal["flow"] = CAL["flow"]
    return cal


def confusion(rows, cal=None):
    """{intended: {measured: count}} and the number that agree."""
    out = {a: {b: 0 for b in ACTIONS} for a in ACTIONS}
    for r in rows:
        out[r["action"]][efforts(r["raw"], cal)["action"]] += 1
    return out, sum(out[a][a] for a in ACTIONS)


def print_confusion(conf, log=print):
    names = list(ACTIONS)
    log("  intended \\ measured " + " ".join("%6s" % a for a in names) + "   agree")
    for a in names:
        n = sum(conf[a].values())
        log("  %-20s" % a + " ".join("%6d" % conf[a][b] for b in names) + "   %d/%d" % (conf[a][a], n))


def label(clip, intent=None):
    ts = [p["t"] for p in clip["points"]]
    qs = [p["q"] for p in clip["points"]]
    tcp = clip.get("tcp")
    raw = raw_measures(ts, qs, tcp)
    meas = efforts(raw)
    meas["raw"] = {"proximal_speed_deg_s": round(raw["weight"], 2), "accel_over_speed_1_s": round(raw["time"], 3),
                   "directness": round(raw["space"], 3), "still_fraction": round(raw["still"], 3),
                   "tcp_directness": round(raw["tcp_directness"], 3),
                   "joint_directness": round(raw["joint_directness"], 3)}
    # descriptors
    travel = [sum(abs(qs[i + 1][j] - qs[i][j]) for i in range(len(qs) - 1)) for j in range(6)]
    tot = sum(travel) or 1e-9
    share = [round(x / tot, 3) for x in travel]
    d = {"duration_s": round(ts[-1] - ts[0], 3), "joint_share": share,
         "dominant_joints": ["J%d" % (j + 1) for j in sorted(range(6), key=lambda j: -share[j])[:2]],
         "loopable": max(abs(a - b) for a, b in zip(qs[0], qs[-1])) < 1.0,
         "start_pose": "HOME" if max(abs(a - b) for a, b in zip(qs[0], HOME)) < 2.0 else None,
         "end_pose": "HOME" if max(abs(a - b) for a, b in zip(qs[-1], HOME)) < 2.0 else None}
    sp = raw["speed"]
    pk = _pct(sp, 0.9) or 1e-9
    accents = sum(1 for i in range(1, len(sp) - 1) if sp[i] < 0.25 * pk and sp[i] <= sp[i - 1] and sp[i] < sp[i + 1])
    d["accents_per_min"] = round(accents / max(1e-6, d["duration_s"]) * 60.0, 1)
    d["successive_lag_s"] = round(successive_lag(ts, qs), 3)
    if tcp:
        vt = raw["tcp_speed"]
        d["tcp_peak_mps"] = round(max(vt), 3)
        d["tcp_mean_mps"] = round(sum(vt) / len(vt), 3)
        d["tcp_path_m"] = round(sum(math.dist(tcp[i + 1], tcp[i]) for i in range(len(tcp) - 1)), 3)
        xs = list(zip(*tcp))
        d["extent_m"] = [round(max(a) - min(a), 3) for a in xs]
        lv = {"low": 0, "mid": 0, "high": 0}
        dirs = {"front": 0, "left": 0, "right": 0, "back": 0}
        for p in tcp:
            lv["low" if p[2] < 0.75 else "mid" if p[2] < 1.35 else "high"] += 1
            az = math.degrees(math.atan2(-p[1], -p[0]))           # 0 = front (-X), + = robot's left (-Y)
            dirs["front" if abs(az) < 30 else "back" if abs(az) > 120 else "left" if az > 0 else "right"] += 1
        d["level"] = {k: round(v / len(tcp), 3) for k, v in lv.items()}
        d["direction"] = {k: round(v / len(tcp), 3) for k, v in dirs.items()}
    tags = [meas["action"]]
    if tcp:
        tags.append(max(d["level"], key=d["level"].get))
        tags.append("fast" if d["tcp_peak_mps"] > 0.8 else "slow" if d["tcp_peak_mps"] < 0.3 else "moderate")
    tags.append("whole-arm" if sum(share[:3]) > 0.5 else "wrist-led")
    if d["loopable"]:
        tags.append("loop")
    out = {"measured": meas, "descriptors": d, "tags": tags}
    if intent:
        acts = [b["action"] for b in intent.get("bars", [])]
        out["intent"] = {"actions": acts, "bpm": intent.get("bpm"), "flow": intent.get("flow")}
        main = max(set(acts), key=acts.count) if acts else None
        out["agrees_with_intent"] = main == meas["action"] if main else None
    return out


def label_span(clip, t0, t1):
    """Measured effort and action of the part of a clip between t0 and t1
    (a bar of a phrase): {"t0", "t1", "action", weight, time, space, flow}."""
    pts = [(p["t"], p["q"], tc) for p, tc in zip(clip["points"], clip.get("tcp") or [None] * len(clip["points"]))
           if t0 - 1e-9 <= p["t"] <= t1 + 1e-9]
    if len(pts) < 4:
        return {"t0": t0, "t1": t1, "action": None}
    ts, qs, tcp = [p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]
    raw = raw_measures(ts, qs, tcp if tcp[0] is not None else None)
    return dict(efforts(raw), t0=round(t0, 3), t1=round(t1, 3))


def majority(conf):
    """{intended: the action measured most often} and how many actions agree
    -- strictly: a tie with another action does not count."""
    top = {a: max(conf[a], key=lambda b: (conf[a][b], b == a)) for a in conf}
    agree = sum(all(conf[a][a] > conf[a][b] for b in conf[a] if b != a) for a in conf)
    return top, agree


if __name__ == "__main__":
    import os
    import sys
    import time
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import choreo

    if "--calibrate" in sys.argv:
        # refit CAL: single-action phrases (choreo.calibration_set), fitted on
        # seeds 0-7, checked on seeds 8-11 it has not seen; paste the line
        t0 = time.time()
        kin = choreo.Kin()
        fit_rows = choreo.calibration_set(kin, seeds=range(8), log=print)
        hold = choreo.calibration_set(kin, seeds=range(8, 12), log=print)
        cal = fit_cal(fit_rows)
        for name, cc in (("current CAL", CAL), ("refitted", cal)):
            for part, rr in (("fit", fit_rows), ("held out", hold)):
                conf, n = confusion(rr, cc)
                print("%s on the %s phrases: %d/%d agree, %d/8 actions by majority"
                      % (name, part, n, len(rr), majority(conf)[1]))
                print_confusion(conf)
        print("\n%d + %d phrases in %.0f s at acc %s. Paste into motion_labels.py:\nCAL = %r"
              % (len(fit_rows), len(hold), time.time() - t0, kin.acc, cal))
        sys.exit(0)

    fails = []

    def check(label_, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label_, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label_)

    t_start = time.time()
    check("each effort action is the nearest action to its own corner",
          all(nearest_action(*ACTIONS[a]) == a for a in ACTIONS))
    # time is judged per weight class: the same accel / speed between the two
    # centres reads sudden for the whole arm, sustained for the wrist
    tc = CAL["time"]
    mid = math.exp((tc[0] + tc[1]) / 2.0)
    strong = efforts({"weight": math.exp(CAL["weight"][0] + 1.0), "time": mid, "space": 1.0, "still": 0.3})
    light = efforts({"weight": math.exp(CAL["weight"][0] - 1.0), "time": mid, "space": 1.0, "still": 0.3})
    check("time is judged against the weight class (heavy arm vs quick wrist)",
          len(tc) == 3 and (strong["time"] > 0) == (tc[1] < tc[0]) and (light["time"] > 0) == (tc[0] < tc[1]),
          "strong %.2f light %.2f" % (strong["time"], light["time"]))

    # a straight move between holds that breathe: the breath is not movement
    fps = 24.0
    ts, qs = [], []
    for i in range(int(5.0 * fps) + 1):
        t = i / fps
        u = max(0.0, min(1.0, (t - 2.0) / 1.0))
        s = u * u * u * (10 - 15 * u + 6 * u * u)
        br = 0.3 * math.sin(2 * math.pi * t / 3.6)
        ts.append(t)
        qs.append([0.0, -90.0 + 25.0 * s + 0.1 * br, 90.0 - 30.0 * s - 0.2 * br, -90.0 + 20.0 * s, -90.0 + 0.35 * br, 0.0])
    raw = raw_measures(ts, qs, None)
    check("a slow breath neither splits a straight move nor fills a hold",
          raw["space"] > 0.99 and raw["still"] < 0.05, "directness %.3f, still %.2f" % (raw["space"], raw["still"]))
    # successive flow: the wrist delayed by 3 frames reads ~0.125 s
    lagged = [[q[0], q[1], q[2], qs[max(0, i - 3)][3], qs[max(0, i - 3)][1] + 0.0, q[5]] for i, q in enumerate(qs)]
    together = [[q[0], q[1], q[2], q[3], q[1], q[5]] for q in qs]
    la, lb = successive_lag(ts, lagged), successive_lag(ts, together)
    check("successive_lag reads a 3-frame wrist delay, and none when the joints move together",
          abs(la - 3 / fps) < 0.02 and abs(lb) < 0.01, "%.3f s / %.3f s" % (la, lb))

    # the calibration holds at the current limits: single-action phrases
    kin = choreo.Kin()
    rows = choreo.calibration_set(kin, seeds=range(12, 16))
    conf, n = confusion(rows)
    top, agree = majority(conf)
    print_confusion(conf)
    check("single-action phrases (principles on) measure as intended: >= 7/8 actions, >= 85% of phrases",
          agree >= 7 and n >= 0.85 * len(rows), "%d/8 actions, %d/%d phrases at acc %s" % (agree, n, len(rows), kin.acc))
    rows_off = choreo.calibration_set(kin, seeds=range(12, 16), principles=False)
    conf, n = confusion(rows_off)
    top, agree = majority(conf)
    check("... and the plain phrases (principles off) too: >= 7/8 actions",
          agree >= 7, "%d/8 actions, %d/%d phrases; %s" % (
              agree, n, len(rows_off), ", ".join("%s->%s" % (a, b) for a, b in top.items() if a != b)))
    clip = {"points": [{"t": t, "q": q} for t, q in zip(rows[0]["ts"], rows[0]["qs"])], "tcp": None}
    lab = label(clip, intent=rows[0]["spec"])
    sp = label_span(clip, 0.0, rows[0]["ts"][-1] / 2.0)
    check("label() / label_span() keep their keys",
          set(lab["measured"]) >= {"weight", "time", "space", "flow", "action", "raw"}
          and {"descriptors", "tags", "intent", "agrees_with_intent"} <= set(lab)
          and set(sp) >= {"t0", "t1", "action", "weight", "time", "space", "flow"})
    print("\n%.0f s" % (time.time() - t_start))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
