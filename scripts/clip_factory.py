"""Clip factory (Stage 3): primitive x style -> a checked, timed clip.

One variant at a time, so a TOP network can run many in parallel (one work
item each) -- scenes/FR20_clip_factory.hiplc, built by
scripts/build_factory_scene.py. Pure Python; FR20 (closed-form IK).

A variant is a dict:

    {"id": "circle_03", "primitive": "circle" | "line" | "figure8",
     "center": [x, y, z],        TCP path centre, robot base frame (m, Z up)
     "size": 0.3,                radius / half-length / lobe size (m)
     "plane": "xy" | "xz" | "yz",
     "tool": [dx, dy, dz],       fixed tool direction (default straight down)
     "safety": 0.8,              fraction of the velocity / acceleration limits
     "tags": [...]}

make(variant, out_dir) runs the pipeline and writes <id>.json (always --
rejected clips too, so the manifest can say why):

    1. sample the TCP path (samples along the primitive)
    2. Stage 2 filter: every sample reachable with the tool direction, and
       none within wrist_min of the wrist singularity (|sin q5|), measured
       with capability.measure -- the same numbers the atlas bakes
    3. IK along the path, each sample on the branch nearest the previous;
       a joint step over max_step_deg between samples is a branch flip
    4. timing: retime_topp.plan (velocity + acceleration x safety), then
       fit() on the 24 fps frames against the robot's full limits -- the
       Retime button's pipeline, with the frames' IK solved exactly
    5. frames -> motion_clip (TCP by FK, bounds, safety as the player plays)
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import capability as C  # noqa: E402
import fairino_player as P  # noqa: E402
import motion_clip as M  # noqa: E402
import retime_topp as T  # noqa: E402
import robot_profile as RP  # noqa: E402
import ur_ik  # noqa: E402
import urdf_rig as U  # noqa: E402

FPS = 24.0
REFERENCE = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]     # a common FR20 ready pose


def _axes(plane):
    return {"xy": ((1, 0, 0), (0, 1, 0)), "xz": ((1, 0, 0), (0, 0, 1)), "yz": ((0, 1, 0), (0, 0, 1))}[plane]


def path_points(v, n):
    """n + 1 TCP points along the variant's primitive."""
    c, s = v["center"], v["size"]
    a, b = _axes(v.get("plane", "xy"))
    out = []
    for i in range(n + 1):
        u = i / float(n)
        if v["primitive"] == "line":
            x, y = (2.0 * u - 1.0) * s, 0.0
        elif v["primitive"] == "circle":
            x, y = s * math.cos(2 * math.pi * u), s * math.sin(2 * math.pi * u)
        elif v["primitive"] == "figure8":
            x, y = s * math.sin(2 * math.pi * u), 0.5 * s * math.sin(4 * math.pi * u)
        else:
            raise ValueError("unknown primitive %r" % v["primitive"])
        out.append(tuple(c[k] + x * a[k] + y * b[k] for k in range(3)))
    return out


class Rejected(Exception):
    pass


def _model():
    model, chain, fo, vel = C.load_fr20(ROOT)
    prof = RP.load("fr20")
    return model, chain, fo, vel, RP.acceleration_limits(prof)


def reach_along(v, n=120, wrist_min=0.1):
    """The variant's target path and, per point, whether the arm can hold
    the tool direction there -- make()'s atlas filter, on every point:
    [(xyz, ok)]. For showing a clip rejected before any motion was made."""
    model, chain, fo, vel, _ = _model()
    R = C.tool_frame(v.get("tool", (0.0, 0.0, -1.0)), v.get("roll", 0.0))
    out = []
    for p in path_points(v, n):
        m = C.measure(model, chain, p, None, fo, vel, frame=R)
        out.append((p, bool(m["reachable"]) and m["wrist"] >= wrist_min))
    return out


def _solve_along(model, R, pts, fo, prev, max_step):
    """IK per point, nearest branch to the previous; Rejected on no solution
    or a branch flip."""
    qs = []
    for i, p in enumerate(pts):
        p6 = U._sub(p, U._mat_vec(R, (0.0, 0.0, fo)))
        sols = ur_ik.within_limits(model, ur_ik.solve(model, R, p6, q6_when_singular=prev[5]))
        best = ur_ik.nearest(sols, prev)
        if best is None:
            raise Rejected("no IK solution at sample %d of %d" % (i, len(pts) - 1))
        q = list(best["q"])
        step = max(abs(a - b) for a, b in zip(q, prev))
        if qs and step > max_step:
            raise Rejected("branch flip at sample %d: a joint jumps %.1f deg" % (i, step))
        qs.append(q)
        prev = q
    return qs


def make(v, out_dir=None, samples=600, wrist_min=0.1, max_step_deg=20.0, env=None):
    """Run the pipeline for one variant. Returns the clip (written to
    out_dir/<id>.json when out_dir is given). env: a collision.py cell
    (dict or path); the clip is rejected when it hits it or breaks a zone."""
    model, chain, fo, vel, acc = _model()
    safety = float(v.get("safety", 0.8))
    tool = v.get("tool", (0.0, 0.0, -1.0))
    R = C.tool_frame(tool, v.get("roll", 0.0))
    style = {k: v[k] for k in v if k not in ("id", "tags")}
    clip = {"schema": M.SCHEMA, "id": v["id"], "robot": "fr20", "joint_names": ["j%d" % i for i in range(1, 7)],
            "units": {"angle": "deg", "time": "s", "length": "m"}, "points": [], "tcp": None,
            "style": style, "meta": {"duration_s": 0.0, "bounds": None, "tags": list(v.get("tags", [])),
                                     "source": {"factory": "clip_factory.py"}}}
    try:
        pts = path_points(v, samples)
        # 2. Stage 2 filter, on every 10th sample (the measure is ~3 ms)
        for i in range(0, len(pts), 10):
            m = C.measure(model, chain, pts[i], None, fo, vel, frame=R)
            if not m["reachable"]:
                raise Rejected("atlas: unreachable with this tool direction at sample %d %s"
                               % (i, [round(x, 3) for x in pts[i]]))
            if m["wrist"] < wrist_min:
                raise Rejected("atlas: within %.0f deg of the wrist singularity at sample %d"
                               % (math.degrees(math.asin(min(1.0, m["wrist"]))), i))
        # 3. IK along the path
        qs = _solve_along(model, R, pts, fo, list(REFERENCE), max_step_deg)
        # unwrap (solutions come back in any +-360 representation)
        for i in range(1, len(qs)):
            qs[i] = [b - 360.0 * round((b - a) / 360.0) for a, b in zip(qs[i - 1], qs[i])]
        # 4. timing
        cum = T.plan(qs, [x * safety for x in vel], [x * safety for x in acc])
        need_of = lambda ts, qq: P.need_profile(ts, qq, 125.0, vel, acc)

        def frames_exact(c):
            ts, fq = [], []
            prev = qs[0]
            for fi, x in enumerate(T.frame_u(c, FPS)):
                k = min(int(x), len(pts) - 2)
                f = x - k
                p = tuple(a + f * (b - a) for a, b in zip(pts[k], pts[k + 1]))
                q = _solve_along(model, R, [p], fo, prev, max_step_deg)[0]
                q = [b - 360.0 * round((b - a) / 360.0) for a, b in zip(prev, q)]
                ts.append(fi / FPS)
                fq.append(q)
                prev = q
            return ts, fq

        cum, _, _ = T.fit(cum, lambda c: T.frames(c, qs, FPS), need_of)
        cum, rounds, ok = T.fit(cum, frames_exact, need_of, max_iter=4)
        ts, fq = frames_exact(cum)
        clip["points"] = [{"t": round(t, 6), "q": [round(x, 6) for x in q]} for t, q in zip(ts, fq)]
        clip["tcp"] = M._tcp_path("fr20", fq)
        xs = list(zip(*clip["tcp"]))
        clip["meta"]["bounds"] = {"min": [min(a) for a in xs], "max": [max(a) for a in xs]}
        clip["meta"]["duration_s"] = round(ts[-1], 6)
        M.measure(clip)
        if env is not None:
            import collision as CL
            cell = CL.load_env(env) if isinstance(env, str) else env
            rep = CL.check(CL.load_model("fr20"), cell, ts, fq)
            clip["safety"]["collision"] = CL.describe(rep)
            clip["safety"]["min_clearance_m"] = rep["min_env_clearance_m"]
            clip["safety"]["min_self_clearance_m"] = rep["min_self_clearance_m"]
            if not rep["ok"]:
                clip["safety"]["ok"] = False
                clip["safety"]["reasons"] = ["cell: " + CL.describe(rep)] + clip["safety"]["reasons"]
        import motion_labels
        clip["labels"] = motion_labels.label(clip)
    except Rejected as e:
        clip["safety"] = {"ok": False, "reasons": [str(e)], "playback_scale": None}
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        M.save(clip, os.path.join(out_dir, v["id"] + ".json"))
    return clip


def wedge(n=50, seed=3):
    """n variants: primitive x size x centre x tool tilt x safety, from a
    fixed seed, so a run is repeatable. Some are meant to fail (too far,
    through the J1 axis, near the wrist singularity) -- the manifest says
    which and why."""
    import random
    rnd = random.Random(seed)
    prims = ("circle", "line", "figure8")
    out = []
    for i in range(n):
        tilt = rnd.choice((0.0, 0.0, 20.0, 45.0))
        az = rnd.uniform(0, 2 * math.pi)
        tool = (math.sin(math.radians(tilt)) * math.cos(az), math.sin(math.radians(tilt)) * math.sin(az),
                -math.cos(math.radians(tilt)))
        # the stage: in front of the robot (-X, +-100 deg), 0.4 - 1.6 m up --
        # scattered all round, half the variants hit the floor or the cart
        r = rnd.uniform(0.4, 1.6)
        th = math.pi + rnd.uniform(-1.75, 1.75)
        out.append({"id": "v%02d_%s" % (i, prims[i % 3]), "primitive": prims[i % 3],
                    "center": [round(r * math.cos(th), 3), round(r * math.sin(th), 3), round(rnd.uniform(0.4, 1.6), 3)],
                    "size": round(rnd.uniform(0.05, 0.35), 3), "plane": rnd.choice(("xy", "xz", "yz")),
                    "tool": [round(x, 4) for x in tool], "safety": round(rnd.uniform(0.5, 0.9), 2),
                    "tags": [prims[i % 3], "tilt%d" % int(tilt)]})
    return out


if __name__ == "__main__":
    import shutil
    import tempfile
    import time

    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    tmp = tempfile.mkdtemp()
    try:
        t0 = time.time()
        good = make({"id": "circle_front", "primitive": "circle", "center": [0.8, 0.0, 0.5], "size": 0.15,
                     "plane": "xy", "safety": 0.8, "tags": ["test"]}, tmp)
        s = good["safety"]
        check("a small circle in front of the robot makes a valid clip", M.validate(good) == [], "; ".join(M.validate(good)))
        check("... that plays at its designed speed", s.get("ok") and s.get("playback_scale") == 1.0,
              "scale %s, %.1f s, %s (%.1f s to make)" % (s.get("playback_scale"), good["meta"]["duration_s"],
                                                       s.get("reasons"), time.time() - t0))
        # the TCP follows the circle: every frame within 1 mm of radius 0.15 about the centre
        dev = max(abs(math.hypot(p[0] - 0.8, p[1] - 0.0) - 0.15) + abs(p[2] - 0.5) for p in good["tcp"])
        check("... and its TCP stays on the circle", dev < 1e-3, "worst %.2e m" % dev)

        far = make({"id": "too_far", "primitive": "line", "center": [2.2, 0.0, 0.4], "size": 0.1}, tmp)
        check("a path beyond the reach is rejected by the atlas filter",
              not far["safety"]["ok"] and "unreachable" in far["safety"]["reasons"][0], far["safety"]["reasons"][0])
        ra = reach_along(far["style"], 20)
        check("a rejected clip's target path can be redrawn, its unreachable points marked",
              len(ra) == 21 and not any(ok for _, ok in ra), "%d of %d reachable" % (sum(ok for _, ok in ra), len(ra)))
        axis = make({"id": "over_axis", "primitive": "line", "center": [0.0, 0.0, 0.8], "size": 0.3}, tmp)
        check("a line through the J1 axis, tool down, is rejected (the inner cylinder)",
              not axis["safety"]["ok"] and "atlas" in axis["safety"]["reasons"][0], axis["safety"]["reasons"][0])

        man = M.write_manifest(tmp)
        check("manifest: 1 ok, 2 rejected, each with a reason",
              man["ok"] == 1 and man["rejected"] == 2 and all(e["reasons"] for e in man["clips"] if not e["ok"]),
              "%d ok, %d rejected" % (man["ok"], man["rejected"]))
        w = wedge(50)
        check("wedge(50) is repeatable and has unique ids", w == wedge(50) and len({v["id"] for v in w}) == 50)
    finally:
        shutil.rmtree(tmp)

    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
