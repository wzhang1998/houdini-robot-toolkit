"""Labels for a motion clip: what it is, measured from the motion itself.

A library is only searchable if every clip is described the same way,
whether it came from choreo.py, the primitive factory or a Houdini export.
So the labels are MEASURED from the joints and TCP path; a generator's own
intent is kept beside them, and whether the two agree is itself a label.

    measured     Laban Effort, each -1 .. +1, and the nearest effort action
      weight     strong (+) .. light (-): speed of the proximal joints
                 (J1-J3, mass-weighted) -- the whole arm moving reads strong,
                 a quick wrist does not
      time       sudden (+) .. sustained (-): acceleration over speed, i.e.
                 how quickly speed changes relative to how fast it goes
      space      direct (+) .. indirect (-): chord / arc of the TCP path
                 between stops
      flow       free (+) .. bound (-): how much of the clip is spent still
      action     nearest of punch / slash / press / wring / dab / flick /
                 glide / float by (weight, time, space)
    descriptors  duration, TCP speed / path / extent, level (low / mid /
                 high share), direction from the robot's own view, dominant
                 joints, accents per minute, loopable, start / end pose
    tags         short words for search: the action, level, fast / slow,
                 whole-arm / wrist-led, loop
    intent       what made it (the phrase spec), when known

label(clip, intent=None) -> dict. Pure Python. Tests: python scripts/choreo.py
(labels are checked against the generator's intent there).
"""

import math

HOME = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
MASS = [4.0, 4.0, 2.5, 1.0, 1.0, 0.6]          # rough share of the arm each joint moves
ACTIONS = {
    "punch": (1, 1, 1), "slash": (1, 1, -1), "press": (1, -1, 1), "wring": (1, -1, -1),
    "dab": (-1, 1, 1), "flick": (-1, 1, -1), "glide": (-1, -1, 1), "float": (-1, -1, -1),
}
# Centres and spreads of the raw measures, calibrated on choreo.py phrases
# (8 actions x 3 seeds, 2026-09-24): the middle between the groups.
#   proximal speed  strong 4.6-29 deg/s, light 1.4-5.7    -> log, centre 5
#   accel / speed   sudden 2.9-6.0 1/s, sustained 1.2-3.7  -> log, centre 3.4
#   directness      direct 0.90-1.00, indirect 0.16-0.81   -> centre 0.85
#   still fraction  bound phrases ~0.6, free ~0.2          -> centre 0.4
# Weight and time are judged on log scales: they span an order of magnitude.
CAL = {"weight": (math.log(5.0), 0.45), "time": (math.log(3.4), 0.35), "space": (0.85, 0.07), "flow": (0.4, 0.15)}


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
    moving = [s > 0.1 * peak for s in speed]
    # skip the rest at both ends: a beat of stillness is part of every phrase
    first = next((i for i, m in enumerate(moving) if m), 0)
    last = len(moving) - 1 - next((i for i, m in enumerate(reversed(moving)) if m), 0)
    inner = moving[first:last + 1] or [True]
    still = 1.0 - sum(inner) / float(len(inner))
    time_raw = _pct([a for a, m in zip(acc, moving) if m] or [0.0], 0.9) / peak
    # TCP directness between stops
    vt = [math.dist(tcp[i + 1], tcp[i]) / dt[i] for i in range(n - 1)] if tcp else []
    chord, arc, seg_start, seg_arc = 0.0, 0.0, None, 0.0
    vpk = _pct(vt, 0.9) or 1e-9
    for i, v in enumerate(vt):
        if v > 0.08 * vpk:
            if seg_start is None:
                seg_start, seg_arc = i, 0.0
            seg_arc += math.dist(tcp[i + 1], tcp[i])
        elif seg_start is not None:
            chord += math.dist(tcp[i], tcp[seg_start])
            arc += seg_arc
            seg_start = None
    if seg_start is not None:
        chord += math.dist(tcp[-1], tcp[seg_start])
        arc += seg_arc
    direct = chord / arc if arc > 1e-6 else 1.0
    return {"weight": _pct(prox, 0.9), "time": time_raw, "space": direct, "still": still,
            "speed": speed, "tcp_speed": vt}


def _map(x, key):
    c, s = CAL[key]
    return max(-1.0, min(1.0, math.tanh((x - c) / s)))


def nearest_action(weight, time, space):
    return min(ACTIONS, key=lambda a: sum((x - y) ** 2 for x, y in zip(ACTIONS[a], (weight, time, space))))


def label(clip, intent=None):
    ts = [p["t"] for p in clip["points"]]
    qs = [p["q"] for p in clip["points"]]
    tcp = clip.get("tcp")
    raw = raw_measures(ts, qs, tcp)
    meas = {"weight": round(_map(math.log(max(raw["weight"], 1e-6)), "weight"), 3),
            "time": round(_map(math.log(max(raw["time"], 1e-6)), "time"), 3),
            "space": round(_map(raw["space"], "space"), 3),
            "flow": round(-_map(raw["still"], "flow"), 3)}
    meas["action"] = nearest_action(meas["weight"], meas["time"], meas["space"])
    meas["raw"] = {"proximal_speed_deg_s": round(raw["weight"], 2), "accel_over_speed_1_s": round(raw["time"], 3),
                   "directness": round(raw["space"], 3), "still_fraction": round(raw["still"], 3)}
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
    m = {"weight": round(_map(math.log(max(raw["weight"], 1e-6)), "weight"), 3),
         "time": round(_map(math.log(max(raw["time"], 1e-6)), "time"), 3),
         "space": round(_map(raw["space"], "space"), 3),
         "flow": round(-_map(raw["still"], "flow"), 3)}
    m["action"] = nearest_action(m["weight"], m["time"], m["space"])
    return dict(m, t0=round(t0, 3), t1=round(t1, 3))
