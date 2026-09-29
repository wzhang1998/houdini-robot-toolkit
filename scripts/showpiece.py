"""Showpieces: the show's big moves, made the way the scan is made.

The idle library is gestures round a hub -- the wrist does most of it, the
whole arm travels little. A showpiece is a process path instead, like the
scan: a raster wiped across a pane in front of the audience's wall with the
LED strip as the squeegee (the user, 2026-09-28: "sweeps along the wall
facing the audience, big, safe").

    "showpieces": [{"id": "wipe_rows", "hub": "greet", "wall": "partition_left",
                    "pattern": "rows", "lines": 3, "speed_mps": 0.35,
                    "width_frac": 0.8}, ...]

The pane is a vertical plane parallel to the wall, just inside the stage (the
work zone the TCP must stay in) on the wall's side; the tool points at the
wall. rows: horizontal passes stepping down, the strip upright (a squeegee
across its stroke); columns: vertical passes stepping sideways, the strip
level. The span is what the arm reaches with that tool attitude, clear of
the room, up to width_frac of the stage. Timed at an even speed along the
path (sine ramps at the ends), slowed until the joints keep their limits at
the library's safety; the hub's checked moves in and out make it one idle
clip from the hub back to it.

    python scripts/showpiece.py --self-test
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)


def pane_from_wall(wall, stage, inset=0.03, standoff=0.02):
    """The pane before a wall (a halfspace: normal into the room, offset) in
    the stage (a work box, yaw about z): {origin, u, v, n, tool, u_range,
    v_range}. u runs along the wall (left to right seen from the room), v up;
    origin is on the plane at u = 0, z = 0, nearest the robot's base; the
    plane is inset + standoff inside the stage's face nearest the wall."""
    n = [wall["normal"][0], wall["normal"][1], 0.0]
    ln = math.hypot(n[0], n[1])
    n = [n[0] / ln, n[1] / ln, 0.0]
    u = [-n[1], n[0], 0.0]                                 # right, facing the wall from the room
    yaw = math.radians(stage.get("yaw_deg", 0.0))
    ax = (math.cos(yaw), math.sin(yaw))
    ay = (-math.sin(yaw), math.cos(yaw))
    c, s = stage["center"], stage["size"]
    corners = [(c[0] + i * s[0] / 2 * ax[0] + j * s[1] / 2 * ay[0], c[1] + i * s[0] / 2 * ax[1] + j * s[1] / 2 * ay[1])
               for i in (-1, 1) for j in (-1, 1)]
    near_wall = min(n[0] * x + n[1] * y for x, y in corners)
    d = near_wall + inset + standoff                       # the plane: n . p = d
    us = [u[0] * x + u[1] * y for x, y in corners]
    return {"origin": [n[0] * d, n[1] * d, 0.0], "u": u, "v": [0.0, 0.0, 1.0], "n": n,
            "tool": [-n[0], -n[1], 0.0], "u_range": [min(us) + inset, max(us) - inset],
            "v_range": [c[2] - s[2] / 2 + inset, c[2] + s[2] / 2 - inset]}


def pane_point(pane, a, b):
    """The 3D point at (a along u, b along v) on the pane."""
    o, u, v = pane["origin"], pane["u"], pane["v"]
    return [o[i] + a * u[i] + b * v[i] for i in range(3)]


TURN_R_MAX = 0.15       # m: wider steps turn on quarter circles joined by a straight run (rounded corners)


def raster(a0, a1, b0, b1, lines, step=0.01):
    """A boustrophedon over [a0, a1] x [b0, b1]: `lines` strokes along a,
    stepping from b0 to b1, joined by turns inside the rectangle: a half
    circle (radius half the step), or for a step wider than 2 TURN_R_MAX two
    quarter circles and the straight run between (the strokes stay long
    when few are spread wide, 2026-09-28). [(a, b)] every ~step metres."""
    if lines < 2:
        raise ValueError("a raster needs two strokes or more")
    gap = (b1 - b0) / (lines - 1)
    r = min(abs(gap) / 2.0, TURN_R_MAX)
    s = math.copysign(1.0, gap)
    lo, hi = a0 + r, a1 - r
    if hi <= lo:
        raise ValueError("the strokes (%.2f m) are shorter than the turns" % (a1 - a0))
    pts = []
    for k in range(lines):
        b = b0 + k * gap
        fwd = k % 2 == 0
        n = max(2, int(math.ceil((hi - lo) / step)))
        xs = [lo + (hi - lo) * i / n for i in range(n + 1)]
        pts += [(x, b) for x in (xs if fwd else xs[::-1])]
        if k < lines - 1:                                  # the turn to the next stroke, bulging outward
            cx, sgn = (hi, 1.0) if fwd else (lo, -1.0)
            m = max(2, int(math.ceil(math.pi * r / 2 / step)))
            pts += [(cx + sgn * r * math.sin(math.pi / 2 * i / m), b + s * r * (1 - math.cos(math.pi / 2 * i / m)))
                    for i in range(1, m + 1)]
            run = abs(gap) - 2 * r
            nr = int(math.ceil(run / step))
            pts += [(cx + sgn * r, b + s * (r + run * i / nr)) for i in range(1, nr)]
            pts += [(cx + sgn * r * math.sin(math.pi / 2 * (1 + i / m)), b + gap - s * r * (1 + math.cos(math.pi / 2 * (1 + i / m))))
                    for i in range(0, m)]
    return pts


def path_length(pts):
    return sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))


def _at(pts, cum, s):
    """The point s metres along a polyline (cum: its cumulative lengths)."""
    import bisect
    k = min(max(bisect.bisect_right(cum, s) - 1, 0), len(pts) - 2)
    f = (s - cum[k]) / (cum[k + 1] - cum[k]) if cum[k + 1] > cum[k] else 0.0
    return [pts[k][i] + f * (pts[k + 1][i] - pts[k][i]) for i in range(3)]


def _span(pane, b, R, ik, cmodel, env, seed, step=0.05):
    """The widest run of a along the pane at height b where the tool, at
    attitude R, is reachable (IK near seed) and clear of the room: (a0, a1)
    or None."""
    import clip_factory as CF
    import collision as C
    lo, hi = pane["u_range"]
    n = int((hi - lo) / step)
    ok = []
    for i in range(n + 1):
        a = lo + i * step
        try:
            q = CF._solve_along(ik[0], R, [pane_point(pane, a, b)], ik[2], list(seed), 360.0)[0]
            good = C.check(cmodel, env, [0.0], [q])["ok"]
        except CF.Rejected:
            good = False
        ok.append((a, good))
    best, run = None, []
    for a, g in ok + [(None, False)]:
        if g:
            run.append(a)
        elif run:
            if best is None or run[-1] - run[0] > best[1] - best[0]:
                best = (run[0], run[-1])
            run = []
    return best


def make(sp, hubs, env, cfg, log=print):
    """One showpiece (see the module) as an idle Segment from its hub back to
    it, or None (said why)."""
    import capability as CAP
    import clip_factory as CF
    import collision as C
    import fairino_player as P
    import robot_profile as RP
    import show as S
    prof = RP.load("fr20")
    vel, acc = RP.velocity_limits(prof), RP.acceleration_limits(prof)
    cmodel = C.load_model("fr20")
    ik = CF._model()
    hub = hubs[sp["hub"]]
    wall = next(o for o in env["objects"] if o["name"] == sp.get("wall", "partition_left"))
    stage = next(o for o in env["objects"] if o["name"] == sp.get("zone", "stage"))
    pane = pane_from_wall(wall, stage)
    rows = sp.get("pattern", "rows") == "rows"
    R = CAP.tool_frame(pane["tool"], 0.0 if rows else 90.0)   # rows: the strip upright; columns: level
    strip = C.strip_box(C.tool_def(prof))
    half = max(strip["size"]) / 2.0 if strip else 0.0
    ceiling = next(o for o in env["objects"] if o["type"] == "halfspace" and o["normal"][2] < -0.9)
    if rows:                                              # the upright strip's ends clear of the floor and ceiling
        top = min(pane["v_range"][1], -ceiling["offset"] - float(ceiling.get("margin_m", 0.3)) - half) - 0.05
        bottom = max(pane["v_range"][0], 0.1 + half) + 0.05
    else:                                                 # the tool level at the wall: the arm's own band
        top, bottom = 1.6, 0.6
    b_top, b_bot = sp.get("top_m", top), sp.get("bottom_m", bottom)
    lines = int(sp.get("lines", 3))
    spans = [_span(pane, b, R, ik, cmodel, env, hub) for b in (b_top, (b_top + b_bot) / 2, b_bot)]
    if any(s is None for s in spans):
        log("  showpiece %s: no reach along the pane at some height" % sp["id"])
        return None
    a0, a1 = max(s[0] for s in spans), min(s[1] for s in spans)
    if a1 - a0 < 0.4:
        log("  showpiece %s: the heights reach no common 0.4 m along the pane (%.2f .. %.2f)" % (sp["id"], a0, a1))
        return None
    want = sp.get("width_frac", 0.8) * (pane["u_range"][1] - pane["u_range"][0])
    if a1 - a0 > want:                                    # centred in what is reachable
        mid = (a0 + a1) / 2
        a0, a1 = mid - want / 2, mid + want / 2
    safety = cfg.get("library", {}).get("safety", 0.5)
    v = float(sp.get("speed_mps", 0.35))
    for attempt in range(8):
        if a1 - a0 < 0.4:
            break
        if rows:
            uv = raster(a0, a1, b_top, b_bot, lines)
        else:
            uv = [(a, b) for b, a in raster(min(b_top, b_bot), max(b_top, b_bot), a0, a1, lines)]   # strokes bottom up
        pts = [pane_point(pane, a, b) for a, b in uv]
        cum = [0.0]
        for p, q in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.dist(p, q))
        L = cum[-1]
        acc_c = float(sp.get("accel_mps2", 0.6))
        ramp = v * (math.pi * v / (2 * acc_c)) / 2
        ts, ss, _ = S.scan_profile(max(0.0, L - 2 * ramp), v, acc_c, 0.02)
        try:
            qs = CF._solve_along(ik[0], R, [_at(pts, cum, s) for s in ss], ik[2], list(hub), 20.0)
        except CF.Rejected as e:
            log("  showpiece %s: %s -- narrower" % (sp["id"], e))
            a0, a1 = a0 + 0.05 * (a1 - a0), a1 - 0.05 * (a1 - a0)
            continue
        for k in range(1, len(qs)):
            qs[k] = [b - 360.0 * round((b - a) / 360.0) for a, b in zip(qs[k - 1], qs[k])]
        lim = P.limiting(ts, qs, 125.0, [x * safety for x in vel], [x * safety for x in acc])
        if lim["scale_needed"] > 1.0 + 1e-3:
            v /= lim["scale_needed"] * 1.05
            log("  showpiece %s: J%d asks %.2fx its %s -- %.2f m/s" % (sp["id"], lim["joint"], lim["scale_needed"],
                                                                        lim["kind"], v))
            continue
        rep = C.check(cmodel, env, ts, qs)
        slow = [x for x in rep["violations"] if x["kind"] == "slow"]
        if slow and len(slow) == len(rep["violations"]):
            # only too fast in a slow zone: slower (pulling the ends in did not help, 2026-09-28)
            v = 0.95 * min(x["limit_mps"] for x in slow)
            log("  showpiece %s: %s -- %.2f m/s" % (sp["id"], C.describe(rep), v))
            continue
        if not rep["ok"]:
            # pull in the end the obstacle is at, by a tenth
            f = min(rep["violations"][0].get("frame", 0), len(ss) - 1)
            p_bad = _at(pts, cum, ss[f])
            a_bad = sum((p_bad[i] - pane["origin"][i]) * pane["u"][i] for i in range(3))
            cut = 0.1 * (a1 - a0)
            if a_bad > (a0 + a1) / 2:
                a1 -= cut
            else:
                a0 += cut
            log("  showpiece %s: %s -- pulled in at a = %.2f" % (sp["id"], C.describe(rep), a_bad))
            continue
        break
    else:
        log("  showpiece %s: gave up" % sp["id"])
        return None
    if a1 - a0 < 0.4:
        log("  showpiece %s: narrowed below 0.4 m" % sp["id"])
        return None
    # in and out as the scan: a pose backed off the pane (lower, towards the
    # middle when needed) with a clear route from the hub, then straight on
    import gestures as G
    rig = G.Rig()
    mid = pane_point(pane, (a0 + a1) / 2, (b_top + b_bot) / 2)
    ends = {}
    for key, q, out in (("in", qs[0], False), ("out", qs[-1], True)):
        tcp = rig.tool(q)[1]
        inward = [mid[i] - tcp[i] for i in range(3)]
        ln = math.sqrt(sum(x * x for x in inward)) or 1.0
        got = S.scan_way(rig, q, pane["tool"], [0.2, 0.3, 0.4], [x / ln for x in inward], cmodel, env, hub, out,
                         grid=False)                     # straight or cuRobo: fail fast, the range then narrows
        if got is None:
            log("  showpiece %s: no approach with a clear route %s the hub" % (sp["id"], "to" if out else "from"))
            return None
        ends[key] = got
    pa, _, path_in, _ = ends["in"]
    pe, _, path_out, _ = ends["out"]
    move_safety = cfg["transition_safety"]
    for _ in range(4):                                    # through a slow zone: timed slower
        ti, qi, ri = S.checked_move([hub] + path_in + [qs[0]], move_safety, vel, acc, cmodel, env)
        to, qo, ro = S.checked_move([qs[-1], pe] + path_out, move_safety, vel, acc, cmodel, env)
        bad = [x for r in (ri, ro) for x in r["violations"]]
        if not bad or any(x["kind"] != "slow" for x in bad):
            break
        move_safety *= 0.95 * min(x["limit_mps"] / x["speed_mps"] for x in bad if x["kind"] == "slow")
        log("  showpiece %s: its moves through the slow zone -- timed at %.2f" % (sp["id"], move_safety))
    if not (ri["ok"] and ro["ok"]):
        log("  showpiece %s: its moves in / out refused: %s" % (sp["id"], C.describe(ri if not ri["ok"] else ro)))
        return None
    hold = 0.3
    t_all = list(ti) + [ti[-1] + hold + x for x in ts]
    t_all += [t_all[-1] + hold + x for x in to]
    q_all = [list(x) for x in qi] + [list(x) for x in qs] + [list(x) for x in qo]
    rep = C.check(cmodel, env, t_all, q_all)
    if not rep["ok"]:
        log("  showpiece %s refused: %s" % (sp["id"], C.describe(rep)))
        return None
    width = a1 - a0
    labels = {"family": sp.get("family", "wipe_rows" if rows else "wipe_cols"), "intent": ["showpiece"],
              "width_m": round(width, 3), "height_m": round(abs(b_top - b_bot), 3), "speed_mps": round(v, 3),
              "clearance_m": rep["min_env_clearance_m"]}
    log("  showpiece %s: %.2f x %.2f m on %s, %d strokes at %.2f m/s, %.1f s with its moves"
        % (sp["id"], width, abs(b_top - b_bot), wall["name"], lines, v, t_all[-1]))
    return S.Segment("%s_%s" % (sp["hub"], sp["id"]), "idle", t_all, q_all, sp["hub"], sp["hub"], labels)


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    wall = {"normal": [0.0, 1.0, 0.0], "offset": -2.0}      # the wall at y = -2, the room at y > -2
    stage = {"center": [0.0, 0.0, 1.0], "size": [2.0, 3.0, 2.0], "yaw_deg": 0.0}
    p = pane_from_wall(wall, stage)
    check("the pane stands inset + standoff inside the stage's face by the wall, u to the right facing the "
          "wall, the tool at it",
          abs(p["origin"][1] - (-1.5 + 0.05)) < 1e-9 and p["tool"] == [-0.0, -1.0, 0.0] and p["u"][0] == -1.0
          and abs(p["u_range"][0] + 0.97) < 1e-9 and abs(p["u_range"][1] - 0.97) < 1e-9
          and abs(p["v_range"][0] - 0.03) < 1e-9, p)
    pts = raster(-0.8, 0.8, 1.4, 0.8, 3)
    inside = all(-0.8 - 1e-9 <= a <= 0.8 + 1e-9 and 0.8 - 1e-9 <= b <= 1.4 + 1e-9 for a, b in pts)
    steps = [math.dist(x, y) for x, y in zip(pts, pts[1:])]
    check("a 3-stroke raster stays in its rectangle, starts top left and ends bottom right, evenly sampled",
          inside and math.dist(pts[0], (-0.65, 1.4)) < 1e-9 and math.dist(pts[-1], (0.65, 0.8)) < 1e-9
          and max(steps) < 0.012, (pts[0], pts[-1], max(steps)))
    L = path_length(pts)
    check("its length: three strokes of 1.3 m and two half turns of radius 0.15", abs(L - (3 * 1.3 + 2 * math.pi * 0.15)) < 0.01, L)
    cols = [(a, b) for b, a in raster(0.8, 1.4, -0.8, 0.8, 4)]          # as make() calls it for columns
    steps = [math.dist(x, y) for x, y in zip(cols, cols[1:])]
    check("columns spread wide: strokes up and down the height 0.3 m long (turns capped at %g m), in the "
          "rectangle, evenly sampled" % TURN_R_MAX, abs(cols[0][1] - (0.8 + TURN_R_MAX)) < 1e-9
          and abs(cols[0][0] + 0.8) < 1e-9 and abs(cols[-1][0] - 0.8) < 1e-9
          and all(-0.8 - 1e-9 <= a <= 0.8 + 1e-9 and 0.8 - 1e-9 <= b <= 1.4 + 1e-9 for a, b in cols)
          and max(steps) < 0.012 and min(steps) > 1e-6, (cols[0], cols[-1], max(steps), min(steps)))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
