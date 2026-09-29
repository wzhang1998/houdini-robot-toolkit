"""The tracking layer against every simulated scenario, offline: the gaze on
top of the greet clips, tick by tick as show_stream would play it, and a
pass / fail per scenario.

    uv run scripts/track_eval.py                 every scenario, the table
    uv run scripts/track_eval.py jump outlier    some
    uv run scripts/track_eval.py --json out.json
    uv run scripts/track_eval.py --export geo/tracking      each scenario's motion for Isaac (isaac/run_tracking.py)

Per scenario (track_sim.py): the greet hub's clips back to back (the show's
own, from the compiled graph), the scenario's messages delivered at their
arrival times to tracking.TargetInput, tracking.Gaze each 8 ms tick. Measured:
- the offsets' velocity and acceleration against their share of the limits
  (tracking.SHARE), and their largest step in a tick;
- the arm's (clip + offsets) velocity and acceleration against the limits;
- collision.check of the whole motion with the clips' margins;
- the gaze error (the tool axis against the person's real head) while
  somebody is there, with the tracking against the clip alone;
- lost: how long until the offsets are back to zero.
Crowd scenarios (track_sim.PEOPLE, /track/people): tracking.Attention picks
whom to look at and its target goes on as one person's; measured too: who
was looked at, how long each time (at least ATTEND_MIN_S, but at the ends),
never a passer-by, everyone in a crowd in turn, the next at once when the
one looked at leaves, a group as one.
Passes when the offsets and the arm stay within their limits, the motion is
clear, and, where the scenario has one, its own condition holds.
"""

import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import collision as C  # noqa: E402
import robot_profile as RP  # noqa: E402
import show as S  # noqa: E402
import track_sim as TS  # noqa: E402
import tracking as TR  # noqa: E402

DT = 0.008
BACK_S = 2.5                 # lost for good: the offsets back to zero this long after the hold


def base_motion(graph, hub, duration, seed=1):
    """The hub's idle clips back to back (a seeded order), as one function of time."""
    clips = sorted(graph.idle(hub), key=lambda s: s.name)
    rng = random.Random(seed)
    seq, t = [], 0.0
    while t < duration + 1.0:
        c = rng.choice(clips)
        seq.append((t, c))
        t += c.duration

    def at(now):
        for t0, c in reversed(seq):
            if now >= t0:
                return c.at(now - t0)
        return seq[0][1].at(0.0)
    return at, [c.name for _, c in seq]


def _angle(rig, q, p):
    tcp, d = TR._tool(rig, q)
    v = [a - b for a, b in zip(p, tcp)]
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b / n for a, b in zip(d, v))))))


def run(name, graph, env, model, hub="greet", seed=1):
    events, truth, dur = TS.scenario(name, seed=seed)
    base, clips = base_motion(graph, hub, dur, seed)
    ti = TR.TargetInput()
    att = TR.Attention()
    gz = TR.Gaze(anchor=graph.hubs[hub], dt=DT, env=env, model=model)
    ts, qs, qb, offs, tgt, who = [], [], [], [], [], []
    k, n = 0, int(dur / DT)
    for i in range(n):
        now = i * DT
        while k < len(events) and events[k]["t"] <= now:
            e = events[k]
            if e["addr"] == "/track/target":
                x, y, z, conf, tm, pid = e["args"]
                ti.target(x, y, z, conf, tm, pid, now=now)
            elif e["addr"] == "/track/people":
                tm = e["args"][0]
                att.update([p + (tm,) for p in TS.people_of(e["args"])], now)
                ch = att.choose(now)
                if ch is not None:
                    ti.target(ch[0][0], ch[0][1], ch[0][2], 0.9, tm, ch[1], now=now)
                elif ti.seen is not None:
                    ti.lost()                          # nobody to look at (passers-by only): as lost
            else:
                ti.lost()
            k += 1
        target = ti.now(now)
        b = base(now)
        q = gz.step(b, target, now, [base(now + d) for d in TR.AHEAD_S])
        ts.append(now)
        qs.append(q)
        qb.append(b)
        offs.append(gz.offsets)
        tgt.append(target)
        who.append(att.current if target is not None else None)
    return {"name": name, "t": ts, "q": qs, "base": qb, "offsets": offs, "target": tgt, "truth": truth, "dur": dur,
            "gaze": gz, "input": ti, "clips": clips, "who": who, "attention": att, "people": name in TS.PEOPLE}


def truth_at(r, i):
    """The real head the arm should be looking at, tick i: the person's, or
    in a crowd the one Attention picked (None: nobody)."""
    t = r["truth"](r["t"][i])
    if not r["people"]:
        return t
    return t.get(r["who"][i]) if r["who"][i] is not None else None


def stints(who, ts):
    """[(pid, seconds)] of the runs of `who` (None included)."""
    out, cur, t0 = [], "start", 0.0
    for p, t in zip(who, ts):
        if p != cur:
            if cur != "start":
                out.append((cur, t - t0))
            cur, t0 = p, t
    out.append((cur, ts[-1] - t0))
    return out


def measure(r, rig, env, model):
    prof = RP.load("fr20")
    vel, acc = RP.velocity_limits(prof), RP.acceleration_limits(prof)
    offs, qs, ts = r["offsets"], r["q"], r["t"]
    J = TR.GAZE_JOINTS

    def ratios(series, lim_v, lim_a, joints):
        v = a = 0.0
        for k, j in enumerate(joints):
            col = [x[k] for x in series] if series is offs else [x[j] for x in series]
            v = max(v, max(abs(y - x) / DT / lim_v[j] for x, y in zip(col, col[1:])))
            a = max(a, max(abs(z - 2 * y + x) / DT ** 2 / lim_a[j] for x, y, z in zip(col, col[1:], col[2:])))
        return v, a
    ov, oa = ratios(offs, [x * TR.SHARE for x in vel], [x * TR.SHARE for x in acc], J)
    av, aa = ratios(qs, vel, acc, range(6))
    step = max(max(abs(b - a) for a, b in zip(x, y)) for x, y in zip(offs, offs[1:]))
    rep = C.check(model, env, ts[::2], qs[::2])
    err_t, err_b = [], []
    for i in range(0, len(ts), 5):
        p = truth_at(r, i)
        if p is not None and r["target"][i] is not None:
            err_t.append(_angle(rig, qs[i], p))
            err_b.append(_angle(rig, r["base"][i], p))
    med = lambda xs: sorted(xs)[len(xs) // 2] if xs else None
    out = {"offset_vel": round(ov, 3), "offset_acc": round(oa, 3), "offset_step_deg": round(step, 3),
           "arm_vel": round(av, 3), "arm_acc": round(aa, 3), "clear": rep["ok"],
           "clearance_m": rep["min_env_clearance_m"], "unsafe_ticks": r["gaze"].unsafe, "shrunk_ticks": r["gaze"].shrunk,
           "gaze_err_deg": round(med(err_t), 1) if err_t else None,
           "clip_err_deg": round(med(err_b), 1) if err_b else None,
           "taken": r["input"].taken, "dropped": r["input"].dropped, "switches": r["input"].switches}
    # lost for good / gone: back to zero after the last target
    last = max((i for i, x in enumerate(r["target"]) if x is not None), default=None)
    if last is not None and last < len(ts) - 1:
        zero = next((i for i in range(last, len(ts)) if max(abs(x) for x in offs[i]) < 0.05), None)
        out["back_s"] = round((ts[zero] - ts[last]) if zero is not None else float("inf"), 2)
    ok = ov <= 1.01 and oa <= 1.02 and av <= 1.0 and aa <= 1.0 and rep["ok"] and r["gaze"].unsafe == 0
    why = []
    if not rep["ok"]:
        why.append(C.describe(rep))
    if r["name"] in ("lost_for_good", "in_and_out") and out.get("back_s", 0) > TR.HOLD_S + BACK_S:
        ok = False
        why.append("not back to the clip in %.1f s" % (TR.HOLD_S + BACK_S))
    if r["people"]:
        runs = [(p, d) for p, d in stints(r["who"], ts) if p is not None]
        looked = sorted({p for p, _ in runs})
        inner = [d for _, d in runs[1:-1]]
        out["attention"] = {"looked_at": looked, "turns": r["attention"].turns,
                            "shortest_s": round(min(inner), 2) if inner else None,
                            "runs": [[p, round(d, 1)] for p, d in runs]}
        if inner and min(inner) < TR.ATTEND_MIN_S - 0.2:
            ok = False
            why.append("looked at someone only %.1f s" % min(inner))
        if r["name"] == "two_standing" and looked != [1, 2]:
            ok = False
            why.append("looked at %s, not both (as near as each other)" % looked)
        if r["name"] == "crowd":                           # the nearest the arm first: the nearest yes, the furthest never
            everyone = r["truth"](r["dur"] - 1.0)
            dist = {pid: math.dist(p[:2], (0.0, 0.0)) for pid, p in everyone.items()}
            nearest, furthest = min(dist, key=dist.get), max(dist, key=dist.get)
            late = {p for t, p in zip(ts, r["who"]) if p is not None and t > 0.5 + 0.7 * 5 + TR.ATTEND_MIN_S}
            out["attention"]["nearest"], out["attention"]["furthest"] = nearest, furthest
            if nearest not in looked or furthest in late:
                ok = False
                why.append("not the nearest first (nearest %s, furthest %s, looked at %s)" % (nearest, furthest, looked))
        if r["name"] == "passer_by" and 2 in looked:
            ok = False
            why.append("looked at the passer-by")
        if r["name"] == "handover":
            nxt = next((t for t, p in zip(ts, r["who"]) if t >= 8.0 and p == 2), None)
            out["attention"]["next_s"] = round(nxt - 8.0, 2) if nxt is not None else None
            if nxt is None or nxt - 8.0 > TR.FORGET_S + 0.5:
                ok = False
                why.append("B not looked at within %.1f s of A leaving" % (TR.FORGET_S + 0.5))
        if r["name"] == "group":
            mid = [sum(c) / 3.0 for c in zip(*r["truth"](10.0).values())]
            far = max(math.dist(p, mid) for t, p in zip(ts, r["target"]) if p is not None and t >= 3.0)  # settled
            out["attention"]["off_middle_m"] = round(far, 3)
            if len(looked) > 1 and r["attention"].turns > 1 or far > 0.15:
                ok = False
                why.append("the group not looked at as one (%.2f m off its middle)" % far)
    if r["name"] == "outlier":
        worst = max(math.dist(p, r["truth"](t)) for t, p in zip(ts, r["target"]) if p is not None)
        out["target_err_m"] = round(worst, 3)
        if worst > 0.3:
            ok = False
            why.append("the outlier reached the target (%.2f m off)" % worst)
    out["pass"], out["why"] = ok, "; ".join(why)
    return out


def export(r, m, out_dir):
    """The scenario's motion for Isaac: every tick's commanded joints, the
    person's real head (or null), the tracked target (or null), the result."""
    os.makedirs(out_dir, exist_ok=True)
    rnd = lambda p: None if p is None else [round(x, 4) for x in p]
    with open(os.path.join(out_dir, "%s.json" % r["name"]), "w") as f:
        json.dump({"name": r["name"], "dt": DT, "hub": "greet", "clips": r["clips"], "result": m,
                   "q": [[round(x, 4) for x in q] for q in r["q"]], "base": [[round(x, 4) for x in q] for q in r["base"]],
                   "truth": [rnd(truth_at(r, i)) for i in range(len(r["t"]))], "target": [rnd(p) for p in r["target"]],
                   "people": ([[[pid] + rnd(p) for pid, p in sorted(r["truth"](t).items())] for t in r["t"]]
                              if r["people"] else None), "who": r["who"]}, f)


def main(argv):
    names = [a for a in argv if not a.startswith("--") and a in TS.SCENARIOS] or list(TS.SCENARIOS)
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    model = C.load_model("fr20")
    graph = S.Graph.load(S.compiled_path(os.path.join(ROOT, "shows", "party.json")))
    import gestures as G
    rig = G.Rig()
    rows = {}
    print("%-14s %-5s %-24s %-18s %-8s %-15s %-8s %s" % ("scenario", "pass", "offsets v/a of share, step",
                                                         "arm v/a of limits", "clear", "gaze / clip err", "back s",
                                                         "taken/dropped/switch"))
    for name in names:
        r = run(name, graph, env, model)
        m = measure(r, rig, env, model)
        rows[name] = m
        if "--export" in argv:
            export(r, m, argv[argv.index("--export") + 1])
        print("%-14s %-5s %4.2f / %4.2f, %5.3f deg   %4.2f / %4.2f        %-8s %5s / %-5s    %-8s %d/%d/%d  %s"
              % (name, "PASS" if m["pass"] else "FAIL", m["offset_vel"], m["offset_acc"], m["offset_step_deg"],
                 m["arm_vel"], m["arm_acc"], "%.3f" % m["clearance_m"] if m["clear"] else "NO",
                 m["gaze_err_deg"], m["clip_err_deg"], m.get("back_s", "-"), m["taken"], m["dropped"], m["switches"],
                 m["why"]), flush=True)
        if "attention" in m:
            a = m["attention"]
            print("%14s looked at %s, %d turns, shortest %s s, runs %s%s" % (
                "", a["looked_at"], a["turns"], a["shortest_s"], a["runs"],
                "".join(", %s %s" % (k, a[k]) for k in ("next_s", "off_middle_m", "nearest", "furthest") if k in a)),
                flush=True)
    if "--json" in argv:
        json.dump(rows, open(argv[argv.index("--json") + 1], "w"), indent=1)
    bad = [k for k, m in rows.items() if not m["pass"]]
    print("\n%s" % ("FAILED: " + ", ".join(bad) if bad else "all %d scenarios pass" % len(rows)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
