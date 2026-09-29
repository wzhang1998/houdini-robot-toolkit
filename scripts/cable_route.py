"""Where to clip the LED strip's cable to the arm: a survey over every motion
the built shows can play (the idle library, the moves, the scan, the big
wipes). The cable runs from the strip's bracket on the flange, clipped to
the arm, down to the laptop on the red cart (the user, 2026-09-29).

    uv run scripts/cable_route.py                        party + party_bigwipe
    uv run scripts/cable_route.py --shows shows/party.json --dt 0.1
    uv run scripts/cable_route.py --self-test

Between two clips the cable spans a chord that changes as the joints
between them turn: its longest is the cable the span needs (shorter, it is
pulled tight), its range (longest - shortest) is the slack that swings as
a loop (the strip, a link, a pinch). A clip on a joint's axis keeps its
span the same while that joint turns -- the dress-pack rule, found here by
search: candidate clips on each link (the joints' axis caps, points along
the link's body, just outside its collision capsules), every motion
sampled, the layouts with 2..5 clips that keep the largest loop smallest
(then the least cable). Chords through a link (the cable pressed round the
arm) are counted for the chosen layouts. Geometry only: the cable's weight,
stiffness and swing are Stage 13's Isaac step.

Writes geo/cable/cable_route.json (the layouts, each clip as link + point
in the link's frame + words, per span the cable and the loop, the motions
at the extremes).
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import collision as C  # noqa: E402
import urdf_rig as U  # noqa: E402

CLEAR_M = 0.015                 # a clip's point this far outside its link's capsules
SPARE = 1.1                     # cable per span: its longest chord x this + CABLE_EXTRA_M
CABLE_EXTRA_M = 0.05
# the links in the cable's order, from the strip to the cart; index into forward_kinematics (-1: the base)
LINKS = [("wrist2_link", 4), ("wrist1_link", 3), ("forearm_link", 2), ("upperarm_link", 1), ("shoulder_link", 0),
         ("base_link", -1)]
WORDS = {0: "J1", 1: "J2", 2: "J3", 3: "J4", 4: "J5", 5: "J6"}
JOINT_NAMES = {0: "base turn (J1)", 1: "shoulder (J2)", 2: "elbow (J3)", 3: "wrist 1 (J4)", 4: "wrist 2 (J5)",
               5: "wrist 3 (J6)"}
FLOOR_CLEAR_M = 0.03            # a clip this far above the floor at least (the base plate, the cable under it)


# ---------------------------------------------------------------- geometry
def _link_frames(model, q):
    """{link index: (R, p)} for -1 (the base, fixed) .. 5 (wrist3, the flange's)."""
    fk = U.forward_kinematics(model["chain"], q)
    out = {-1: (U.IDENTITY, (0.0, 0.0, 0.0))}
    for k, f in enumerate(fk):
        out[k] = (f["link_R"], f["link_p"])
    return out, fk


def _to_world(frame, p):
    R, t = frame
    return tuple(t[i] + R[i][0] * p[0] + R[i][1] * p[1] + R[i][2] * p[2] for i in range(3))


def _to_local(frame, p):
    R, t = frame
    d = [p[i] - t[i] for i in range(3)]
    return tuple(R[0][i] * d[0] + R[1][i] * d[1] + R[2][i] * d[2] for i in range(3))


def _dir_local(frame, v):
    R = frame[0]
    return tuple(R[0][i] * v[0] + R[1][i] * v[1] + R[2][i] * v[2] for i in range(3))


def _seg_dist(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    L2 = sum(x * x for x in ab)
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, sum((p[i] - a[i]) * ab[i] for i in range(3)) / L2))
    return math.dist(p, [a[i] + ab[i] * t for i in range(3)])


def _clearance(p, caps):
    return min((_seg_dist(p, c["a"], c["b"]) - c["r"]) for c in caps) if caps else math.inf


def _push_out(c, d, caps, clear=CLEAR_M):
    """The point from c along unit d where it first clears caps by clear, or None."""
    for k in range(0, 121):
        p = tuple(c[i] + d[i] * 0.005 * k for i in range(3))
        if _clearance(p, caps) >= clear:
            return p
    return None


def _norm(v):
    L = math.sqrt(sum(x * x for x in v))
    return tuple(x / L for x in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def candidates(model, tool_caps=None):
    """The clip points: [{id, link, k (frame index), p (in the link's frame), words}].
    On each link: its joints' axis caps (both sides) and 4 points round its
    body at 25/50/75 % between its joints; the bracket's 4 sides (the cable
    leaves the strip there)."""
    frames, fk = _link_frames(model, [0.0, -90.0, 90.0, -90.0, -90.0, 0.0])
    out = []
    for name, k in LINKS:
        caps = [c for c in model["caps"] if c.get("joint") == k and not c["name"].startswith("tool")]
        fr = frames[k]
        joints = [j for j in (k, k + 1) if 0 <= j < len(fk)]
        for j in joints:                                      # the axis caps: clear of both links at the joint
            o = _to_local(fr, fk[j]["position"])
            a = _norm(_dir_local(fr, fk[j]["axis"]))
            other = j - 1 if j == k else j                    # the link on the joint's other side
            both = caps + [dict(c, a=_to_local(fr, _to_world(frames[other], c["a"])),
                                b=_to_local(fr, _to_world(frames[other], c["b"])))
                           for c in model["caps"] if c.get("joint", -1) == other and not c["name"].startswith("tool")]
            for s, side in ((1, "+"), (-1, "-")):
                p = _push_out(o, tuple(s * x for x in a), both)
                if p:
                    out.append({"id": "%s/%s_axis%s" % (name, WORDS[j], side), "link": name, "k": k, "p": p,
                                "words": "%s, on the %s joint's cap, its %s side"
                                         % (name, JOINT_NAMES[j], "+" if s > 0 else "-")})
        if len(joints) == 2:                                  # along the body, between its joints
            o0, o1 = _to_local(fr, fk[joints[0]]["position"]), _to_local(fr, fk[joints[1]]["position"])
            L = math.dist(o0, o1)
            if L > 0.15:
                along = _norm([o1[i] - o0[i] for i in range(3)])
                ax = _norm(_dir_local(fr, fk[joints[0]]["axis"]))
                side = _cross(along, ax)
                if math.sqrt(sum(x * x for x in side)) < 1e-6:
                    ax = _norm(_dir_local(fr, fk[joints[1]]["axis"]))
                    side = _cross(along, ax)
                side = _norm(side)
                for t in (0.25, 0.5, 0.75):
                    c = tuple(o0[i] + (o1[i] - o0[i]) * t for i in range(3))
                    for d, dn in ((ax, "axis+"), (tuple(-x for x in ax), "axis-"), (side, "side+"),
                                  (tuple(-x for x in side), "side-")):
                        p = _push_out(c, d, caps)
                        if p:
                            out.append({"id": "%s/body%02d_%s" % (name, int(t * 100), dn), "link": name, "k": k,
                                        "p": p, "words": "%s, %d%% of the way from the %s to the %s, %s face"
                                                         % (name, int(t * 100), JOINT_NAMES[joints[0]],
                                                            JOINT_NAMES[joints[1]], dn)})
    for c in (tool_caps or []):                               # the bracket's sides
        if c["name"] != "tool_bracket":
            continue
        mid = tuple((c["a"][i] + c["b"][i]) / 2 for i in range(3))
        for d, dn in (((1, 0, 0), "x+"), ((-1, 0, 0), "x-"), ((0, 1, 0), "y+"), ((0, -1, 0), "y-")):
            p = _push_out(mid, d, [c], 0.005)
            out.append({"id": "tool/bracket_%s" % dn, "link": "tool", "k": 5, "p": p,
                        "words": "the strip's bracket, %s side" % dn})
    return out


def cart_point(env):
    """Where the cable ends: the red cart's top, its edge nearest the robot's base."""
    c = next(o for o in env["objects"] if o["name"] == "control_cart")
    yaw = math.radians(c.get("yaw_deg", 0.0))
    u, v = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
    rel = (-c["center"][0], -c["center"][1])                  # towards the base, in the box's axes
    lu = max(-c["size"][0] / 2, min(c["size"][0] / 2, rel[0] * u[0] + rel[1] * u[1]))
    lv = max(-c["size"][1] / 2, min(c["size"][1] / 2, rel[0] * v[0] + rel[1] * v[1]))
    return (c["center"][0] + u[0] * lu + v[0] * lv, c["center"][1] + u[1] * lu + v[1] * lv,
            c["center"][2] + c["size"][2] / 2)


# ---------------------------------------------------------------- motions
def motions(show_paths, dt):
    """[(segment name, [q ...])]: every segment of the shows, sampled every dt
    (the same segment in two shows once)."""
    import show as S
    out, seen = [], set()
    for sp in show_paths:
        g = S.Graph.load(S.compiled_path(os.path.abspath(sp)))
        for s in g.segments:
            key = (s.name, len(s.q), tuple(round(x, 3) for x in s.q[0]), tuple(round(x, 3) for x in s.q[-1]))
            if key in seen:
                continue
            seen.add(key)
            n = max(1, int(s.duration / dt))
            out.append((s.name, [s.at(s.duration * i / n) for i in range(n + 1)]))
    return out


def track(model, cands, moves):
    """World positions of every candidate at every frame: ({id: [xyz ...]},
    [(segment, frame index in it)] per frame, [q per frame])."""
    pos = {c["id"]: [] for c in cands}
    where, qs = [], []
    for name, frames in moves:
        for i, q in enumerate(frames):
            fr, _ = _link_frames(model, q)
            for c in cands:
                pos[c["id"]].append(_to_world(fr[c["k"]], c["p"]))
            where.append((name, i))
            qs.append(q)
    return pos, where, qs


def usable(model, cands, pos, qs, clear=0.005):
    """The candidates no other link comes within clear of in any frame (a
    clip another link runs into is no clip) and above the floor by
    FLOOR_CLEAR_M: (kept, {id: (worst clearance, frame)})."""
    caps_f = [_world_caps(model, q) for q in qs]
    kept, dropped = [], {}
    for c in cands:
        own = (c["link"],) if c["link"] != "tool" else ("tool_bracket", "tool_strip", "wrist3_link")
        worst, fw = math.inf, 0
        for f, (p, caps) in enumerate(zip(pos[c["id"]], caps_f)):
            d = min(_clearance(p, [x for x in caps if x["name"] not in own]), p[2] - FLOOR_CLEAR_M + clear)
            if d < worst:
                worst, fw = d, f
        if worst < clear:
            dropped[c["id"]] = (round(worst, 3), fw)
        else:
            kept.append(c)
    return kept, dropped


def span_stats(pa, pb):
    """(shortest, longest, frame of shortest, frame of longest) of a chord over the frames."""
    lo, hi, flo, fhi = math.inf, -math.inf, 0, 0
    for f, (a, b) in enumerate(zip(pa, pb)):
        d = math.dist(a, b)
        if d < lo:
            lo, flo = d, f
        if d > hi:
            hi, fhi = d, f
    return lo, hi, flo, fhi


# ---------------------------------------------------------------- search
def best_layouts(cands, pos, end, counts=(2, 3, 4, 5)):
    """{clips: (largest loop, cable, [ids from the strip to the cart])}: per
    number of clips on the arm (the base's counts), the layout whose largest
    span loop is smallest, then the least cable. The strip's bracket starts
    it (one of its sides), the cart ends it; clips go link by link towards
    the base, a link used once."""
    order = ["tool"] + [n for n, _ in LINKS]
    level = {n: i for i, n in enumerate(order)}
    nodes = [c for c in cands]
    n_frames = len(next(iter(pos.values())))
    pos = dict(pos, cart=[end] * n_frames)
    cache = {}

    def stats(a, b):
        if (a, b) not in cache:
            cache[(a, b)] = span_stats(pos[a], pos[b])
        return cache[(a, b)]
    # best[(id, clips)] = (loop, cable, path) from the strip's bracket to this clip
    best = {}
    for c in nodes:
        if c["link"] == "tool":
            best[(c["id"], 0)] = (0.0, 0.0, [c["id"]])
    by_level = sorted(nodes, key=lambda c: level[c["link"]])
    for c in by_level:
        if c["link"] == "tool":
            continue
        for (pid, m), (loop, cable, path) in list(best.items()):
            prev = next(x for x in nodes if x["id"] == pid)
            if level[prev["link"]] >= level[c["link"]]:
                continue
            lo, hi, _, _ = stats(pid, c["id"])
            cand = (max(loop, hi - lo), cable + hi, path + [c["id"]])
            key = (c["id"], m + 1)
            if key not in best or cand[:2] < best[key][:2]:
                best[key] = cand
    out = {}
    for (cid, m), (loop, cable, path) in best.items():
        if m not in counts:
            continue
        lo, hi, _, _ = stats(cid, "cart")
        total = (max(loop, hi - lo), cable + hi, path + ["cart"])
        if m not in out or total[:2] < out[m][:2]:
            out[m] = total
    return out


def through_arm(model, pa, pb, qs, own=()):
    """Frames where the chord's middle 70 % passes more than 1 cm inside a
    link's capsule (the cable pressed round the arm): [frame index]."""
    hits = []
    for f, (a, b, q) in enumerate(zip(pa, pb, qs)):
        caps = _world_caps(model, q)
        pts = [tuple(a[i] + (b[i] - a[i]) * t for i in range(3)) for t in (0.15, 0.3, 0.5, 0.7, 0.85)]
        if any(_clearance(p, [c for c in caps if c["name"] not in own]) < -0.01 for p in pts):
            hits.append(f)
    return hits


def _world_caps(model, q):
    fr, _ = _link_frames(model, q)
    out = []
    for c in model["caps"]:
        k = c.get("joint", -1)
        out.append({"name": c["name"], "a": _to_world(fr[k], c["a"]), "b": _to_world(fr[k], c["b"]), "r": c["r"]})
    return out


# ---------------------------------------------------------------- report
def report(model, cands, pos, where, qs, layouts, end):
    byid = {c["id"]: c for c in cands}
    pos = dict(pos, cart=[end] * len(where))
    out = []
    for m in sorted(layouts):
        loop, cable, path = layouts[m]
        spans = []
        for a, b in zip(path, path[1:]):
            lo, hi, flo, fhi = span_stats(pos[a], pos[b])
            wrap = through_arm(model, pos[a], pos[b], qs, own=["tool_strip", "tool_bracket"])
            wrap_by = {}
            for f in wrap:
                wrap_by[where[f][0]] = wrap_by.get(where[f][0], 0) + 1
            spans.append({"from": a, "to": b, "cable_m": round(hi * SPARE + CABLE_EXTRA_M, 2),
                          "chord_m": [round(lo, 3), round(hi, 3)], "loop_m": round(hi - lo, 3),
                          "longest_in": where[fhi][0], "shortest_in": where[flo][0],
                          "through_arm_frames": len(wrap), "through_arm_in": sorted(wrap_by.items(),
                                                                                     key=lambda x: -x[1])[:5]})
        out.append({"clips": m, "largest_loop_m": round(loop, 3), "cable_m": round(sum(s["cable_m"] for s in spans), 2),
                    "points": [{"id": i, "link": byid[i]["link"], "p_link_m": [round(x, 4) for x in byid[i]["p"]],
                                "words": byid[i]["words"]} if i in byid else {"id": "cart", "p_world_m":
                                [round(x, 3) for x in end], "words": "the red cart's top, the laptop"}
                               for i in path],
                    "spans": spans})
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shows", nargs="*", default=["shows/party.json", "shows/party_bigwipe.json"])
    ap.add_argument("--dt", type=float, default=0.2, help="seconds between the frames sampled")
    ap.add_argument("--out", default=os.path.join(ROOT, "geo", "cable", "cable_route.json"))
    a = ap.parse_args(argv)
    import robot_profile as RP
    model = C.load_model("fr20")
    tool = [c for c in C.tool_capsules(C.tool_def(RP.load("fr20")), model["flange_offset"])]
    cands = candidates(model, tool)
    env = C.load_env(os.path.join(ROOT, json.load(open(os.path.join(ROOT, a.shows[0])))["env"]))
    end = cart_point(env)
    moves = motions([os.path.join(ROOT, s) for s in a.shows], a.dt)
    pos, where, qs = track(model, cands, moves)
    cands, dropped = usable(model, cands, pos, qs)
    print("%d clip candidates (%d others: another link runs into them), %d motions, %d frames (every %.2f s); "
          "the cart's end at %s" % (len(cands), len(dropped), len(moves), len(where), a.dt, [round(x, 2) for x in end]))
    layouts = best_layouts(cands, pos, end)
    rep = report(model, cands, pos, where, qs, layouts, end)
    j1 = [q[0] for q in qs]
    doc = {"shows": a.shows, "dt_s": a.dt, "frames": len(where), "motions": len(moves),
           "j1_range_deg": [round(min(j1), 1), round(max(j1), 1)], "layouts": rep}
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(doc, open(a.out, "w"), indent=1)
    for L in rep:
        print("\n%d clips: largest loop %.2f m, cable %.2f m" % (L["clips"], L["largest_loop_m"], L["cable_m"]))
        for p in L["points"]:
            print("   %-34s %s" % (p["id"], p["words"]))
        for s in L["spans"]:
            print("   span %-30s -> %-30s cable %.2f m, loop %.2f m (longest in %s, shortest in %s)%s"
                  % (s["from"], s["to"], s["cable_m"], s["loop_m"], s["longest_in"], s["shortest_in"],
                     ("; through the arm in %d frames: %s" % (s["through_arm_frames"], s["through_arm_in"][:3]))
                     if s["through_arm_frames"] else ""))
    print("\nJ1 over the shows: %s deg (the cable twists round the base by this much)" % doc["j1_range_deg"])
    print("wrote %s" % os.path.relpath(a.out, ROOT))
    return 0


def self_test():
    fails = []

    def check(name, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", name, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(name)

    model = C.load_model("fr20")
    cands = candidates(model)
    ids = {c["id"] for c in cands}
    check("candidates on every link: axis caps and body points, each clear of its link",
          {"forearm_link/J3_axis+", "upperarm_link/J3_axis+", "forearm_link/body50_side+"} <= ids
          and all(_clearance(c["p"], [x for x in model["caps"] if x.get("joint") == c["k"]
                                      and not x["name"].startswith("tool")]) >= CLEAR_M - 1e-9 for c in cands),
          len(cands))
    # only the elbow turns
    frames = [[0.0, -90.0, 20.0 + 5.0 * i, -90.0, -90.0, 0.0] for i in range(26)]
    sub = [c for c in cands if c["link"] in ("forearm_link", "upperarm_link")]
    pos, where, qs = track(model, sub, [("elbow", frames)])
    lo, hi, _, _ = span_stats(pos["upperarm_link/J3_axis+"], pos["forearm_link/J3_axis+"])
    lo2, hi2, _, _ = span_stats(pos["upperarm_link/body50_side+"], pos["forearm_link/body50_side+"])
    check("a span between two clips on the elbow's axis stays the same while the elbow turns; one between "
          "the links' middles swings", hi - lo < 1e-6 and hi2 - lo2 > 0.3, (hi - lo, hi2 - lo2))
    # the search: from the forearm's middle to a fixed end past the upper arm; only the elbow turns
    start = next(c for c in sub if c["id"] == "forearm_link/body75_side+")
    tool_like = dict(start, link="tool", id="tool/x")
    pos2 = dict(pos, **{"tool/x": pos[start["id"]]})
    end = _to_world(_link_frames(model, frames[0])[0][1], next(c for c in sub if c["id"] ==
                                                               "upperarm_link/body25_side+")["p"])
    got = best_layouts([tool_like] + [c for c in sub if c["link"] == "upperarm_link"]
                       + [dict(c, link="wrist1_link") for c in sub if c["link"] == "forearm_link"
                          and "J3_axis" in c["id"]],
                       dict(pos2, **{c["id"]: pos[c["id"]] for c in sub}), end, counts=(2,))
    path = got.get(2, (None, None, []))[2]
    check("the search puts the clips on the elbow's axis caps", any("J3_axis" in p for p in path), path)
    fa0 = next(c for c in _world_caps(model, frames[0]) if c["name"] == "forearm_link")
    inside = {"id": "upperarm_link/in_forearm", "link": "upperarm_link", "k": 1,
              "p": _to_local(_link_frames(model, frames[0])[0][1], fa0["a"])}   # where the forearm is, at first
    fine = next(c for c in sub if c["id"] == "upperarm_link/J3_axis+")
    pos3 = dict(pos, **{inside["id"]: [_to_world(_link_frames(model, f)[0][1], inside["p"]) for f in frames]})
    kept, dropped = usable(model, [inside, fine], pos3, frames)
    check("a clip another link runs into is dropped, one clear of them kept",
          list(dropped) == [inside["id"]] and kept == [fine], (dropped, [c["id"] for c in kept]))
    q = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    wc = _world_caps(model, q)
    fa = next(c for c in wc if c["name"] == "forearm_link")
    mid = tuple((fa["a"][i] + fa["b"][i]) / 2 for i in range(3))
    far = (mid[0], mid[1] + 1.0, mid[2])
    check("a chord through a link is caught, one beside it is not",
          through_arm(model, [tuple(mid[i] - (far[i] - mid[i]) for i in range(3))], [far], [q]) == [0]
          and through_arm(model, [(mid[0], mid[1] - 1.0, mid[2] + 0.6)], [(mid[0], mid[1] + 1.0, mid[2] + 0.6)],
                          [q]) == [])
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
