"""Motion clip contract (Stage 3): one JSON file per clip, a library manifest.

A clip is shaped like ROS trajectory_msgs/JointTrajectory, so Stage 4's
MoveIt service can read it with little translation, plus what a library of
clips needs to be searched and trusted:

    {
      "schema": "motionlab.clip/1",
      "id": "fr20_test",
      "robot": "fr20",                    profile id
      "joint_names": ["j1", ..., "j6"],
      "units": {"angle": "deg", "time": "s", "length": "m"},
      "points": [{"t": 0.0, "q": [6 floats]}, ...],     strictly increasing t
      "tcp": [[x, y, z], ...] | null,     TCP per point, robot base frame
                                          (URDF, Z up); null without a URDF
      "style": {...},                     free-form: what made the clip
      "meta": {"duration_s", "bounds": {"min": [x,y,z], "max": [x,y,z]},
               "tags": [...], "source": {...}},
      "safety": {"vel_limit", "acc_limit", "jerk_limit" (or null),
                 "peak_vel", "peak_acc", "peak_jerk"   per joint, as played,
                 "playback_scale",        1.0 = plays at its own speed
                 "ok", "reasons": [...]}
    }

safety is measured the way fairino_player.py plays the clip (its own
conditioning), so a clip marked ok plays at its designed speed -- the same
guarantee as the asset's Pre-Flight. Jerk is reported, and checked when the
profile has a limit (UF850: UFACTORY's 28647 deg/s^3; FR20: none published).

    from_csv(path, robot, ...)  -> clip       the asset's CSV export
    to_csv(clip, path)                        what fairino_player.py plays
    validate(clip)              -> [errors]   structure, times, limits
    measure(clip)               -> safety     fills clip["safety"] too
    save(clip, path) / load(path)
    write_manifest(directory)   -> manifest   every clip: id, duration,
                                              bounds, tags, ok / reasons

Pure Python. Tests: python scripts/motion_clip.py
"""

import csv
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import fairino_player as P  # noqa: E402
import robot_profile as RP  # noqa: E402

SCHEMA = "motionlab.clip/1"


def _profile(robot):
    return RP.load(robot)


def _tcp_path(robot, qs):
    """TCP per point from URDF FK, or None when the profile has no URDF."""
    prof = _profile(robot)
    urdf = prof["rig"].get("urdf")
    if not urdf:
        return None
    import ur_ik
    import urdf_rig as U
    chain = U.parse_urdf(os.path.join(ROOT, urdf))["chain"]
    fo = float(prof["rig"].get("flange_offset_m", 0.0))
    out = []
    for q in qs:
        R, p6 = ur_ik.pose_of(chain, q)
        out.append([round(x, 6) for x in U._add(p6, U._mat_vec(R, (0.0, 0.0, fo)))])
    return out


def from_csv(path, robot, clip_id=None, tags=(), style=None, source=None):
    t, q = P.load_csv(path)
    t0 = t[0]
    clip = {
        "schema": SCHEMA,
        "id": clip_id or os.path.splitext(os.path.basename(path))[0],
        "robot": robot,
        "joint_names": ["j%d" % i for i in range(1, 7)],
        "units": {"angle": "deg", "time": "s", "length": "m"},
        "points": [{"t": round(ti - t0, 6), "q": [round(x, 6) for x in qi]} for ti, qi in zip(t, q)],
        "tcp": _tcp_path(robot, q),
        "style": dict(style or {}),
        "meta": {"duration_s": round(t[-1] - t0, 6), "tags": list(tags),
                 "source": dict(source or {"csv": os.path.abspath(path)})},
    }
    if clip["tcp"]:
        xs = list(zip(*clip["tcp"]))
        clip["meta"]["bounds"] = {"min": [min(a) for a in xs], "max": [max(a) for a in xs]}
    else:
        clip["meta"]["bounds"] = None
    measure(clip)
    return clip


def to_csv(clip, path):
    rows = []
    for i, pt in enumerate(clip["points"]):
        rows.append([i + 1, "%.6f" % pt["t"]] + ["%.6f" % x for x in pt["q"]])
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "time_s"] + ["j%d_deg" % i for i in range(1, 7)])
        w.writerows(rows)


def validate(clip):
    """Structural errors, as strings; [] when the clip is well-formed."""
    errs = []
    for k in ("schema", "id", "robot", "joint_names", "points", "meta", "safety"):
        if k not in clip:
            errs.append("missing %s" % k)
    if errs:
        return errs
    if clip["schema"] != SCHEMA:
        errs.append("schema %r, expected %r" % (clip["schema"], SCHEMA))
    n = len(clip["joint_names"])
    pts = clip["points"]
    if len(pts) < 2:
        errs.append("needs at least two points")
    try:
        prof = _profile(clip["robot"])
        limits = prof["robot"]["limits_deg"]
    except Exception as e:
        return errs + ["unknown robot %r: %s" % (clip["robot"], e)]
    for i, pt in enumerate(pts):
        if len(pt.get("q", [])) != n:
            errs.append("point %d: %d joints, expected %d" % (i, len(pt.get("q", [])), n))
            continue
        if not all(math.isfinite(x) for x in pt["q"]) or not math.isfinite(pt.get("t", float("nan"))):
            errs.append("point %d: non-finite value" % i)
            continue
        for j, (x, (lo, hi)) in enumerate(zip(pt["q"], limits)):
            if not lo - 1e-6 <= x <= hi + 1e-6:
                errs.append("point %d: j%d = %.3f outside [%g, %g]" % (i, j + 1, x, lo, hi))
        if i and not pt["t"] > pts[i - 1]["t"]:
            errs.append("point %d: t not strictly increasing" % i)
    if clip.get("tcp") is not None and len(clip["tcp"]) != len(pts):
        errs.append("tcp has %d entries for %d points" % (len(clip["tcp"]), len(pts)))
    return errs


def _jerk_peaks(samples, dt):
    """Per-joint max |jerk| over equal-interval samples, at rest outside."""
    pad = [samples[0]] * 2 + samples + [samples[-1]] * 2
    out = [0.0] * 6
    for k in range(2, len(pad) - 2):
        for j in range(6):
            d3 = (pad[k + 2][j] - 2 * pad[k + 1][j] + 2 * pad[k - 1][j] - pad[k - 2][j]) / (2 * dt ** 3)
            out[j] = max(out[j], abs(d3))
    return out


def measure(clip, rate_hz=125.0, acc=None):
    """clip["safety"], measured as the player plays the clip at speed 1.0.
    acc: acceleration limits to measure against instead of the profile's."""
    prof = _profile(clip["robot"])
    vel = RP.velocity_limits(prof)
    acc = (list(acc) if isinstance(acc, (list, tuple)) else [float(acc)] * 6) if acc is not None         else (RP.acceleration_limits(prof) or [300.0] * 6)
    jerk = RP.jerk_limits(prof)
    t = [p["t"] for p in clip["points"]]
    q = [p["q"] for p in clip["points"]]
    lim = [tuple(x) for x in prof["robot"]["limits_deg"]]
    reasons = []
    try:
        samples, dt, rep = P.condition(t, q, rate_hz, vel, acc, lim)
    except P.TrajectoryError as e:
        clip["safety"] = {"vel_limit": vel, "acc_limit": acc, "jerk_limit": jerk,
                          "peak_vel": None, "peak_acc": None, "peak_jerk": None,
                          "playback_scale": None, "ok": False, "reasons": ["does not condition: %s" % e]}
        return clip["safety"]
    scale = rep["time_scale"]
    # peaks at the clip's OWN timing (scale 1): what the design asks for
    s1, dt1 = samples, dt
    if scale > 1.0:
        path = P.Path(t, q)
        n = max(1, int(math.ceil(path.duration * rate_hz)))
        dt1 = path.duration / n
        s1 = [path.at(k * dt1) for k in range(n + 1)]
    vmax, amax = P._peaks(s1, dt1)
    jmax = _jerk_peaks(s1, dt1)
    if scale > 1.0:
        lb = P.limiting(t, q, rate_hz, vel, acc, lim)
        reasons.append("plays %.2fx slower than designed: J%d %s %.1fx its limit at %.2f s"
                       % (scale, lb["joint"], lb["kind"], lb["ratio"], lb["time_s"]))
    if jerk:
        worst = max(range(6), key=lambda j: jmax[j] / jerk[j])
        if jmax[worst] > jerk[worst] * 1.001:
            reasons.append("J%d jerk %.0f deg/s^3 over its %.0f" % (worst + 1, jmax[worst], jerk[worst]))
    clip["safety"] = {
        "vel_limit": vel, "acc_limit": acc, "jerk_limit": jerk,
        "peak_vel": [round(x, 3) for x in vmax], "peak_acc": [round(x, 3) for x in amax],
        "peak_jerk": [round(x, 1) for x in jmax],
        "playback_scale": round(scale, 4), "ok": not reasons, "reasons": reasons,
    }
    return clip["safety"]


def save(clip, path):
    with open(path, "w") as f:
        json.dump(clip, f, indent=1)


def load(path):
    with open(path) as f:
        return json.load(f)


def write_manifest(directory):
    """manifest.json beside the clips: one entry per *.json clip."""
    entries = []
    for f in sorted(glob.glob(os.path.join(directory, "*.json"))):
        if os.path.basename(f) == "manifest.json":
            continue
        try:
            c = load(f)
        except ValueError as e:
            entries.append({"file": os.path.basename(f), "ok": False, "reasons": ["unreadable: %s" % e]})
            continue
        s = c.get("safety", {})
        # a clip rejected before it was timed has no points: its own reason
        # says why, and "needs at least two points" would only hide it
        errs = validate(c) if c.get("points") else []
        e = {"file": os.path.basename(f), "id": c.get("id"), "robot": c.get("robot"),
             "duration_s": c.get("meta", {}).get("duration_s"),
             "bounds": c.get("meta", {}).get("bounds"), "tags": c.get("meta", {}).get("tags", []),
             "ok": not errs and bool(s.get("ok")),
             "reasons": list(s.get("reasons", [])) + errs}
        lab = c.get("labels")
        if lab:
            m = lab["measured"]
            e["labels"] = {"action": m["action"], "effort": {k: m[k] for k in ("weight", "time", "space", "flow")},
                           "tags": lab.get("tags", []), "agrees_with_intent": lab.get("agrees_with_intent")}
            if lab.get("sequence"):
                e["labels"]["sequence"] = lab["sequence"]
                e["labels"]["intent_sequence"] = [b["intent"] for b in lab.get("bars", [])]
        if "min_clearance_m" in s:
            e["min_clearance_m"] = s["min_clearance_m"]
        entries.append(e)
    man = {"schema": "motionlab.manifest/1", "clips": entries,
           "ok": sum(1 for e in entries if e["ok"]), "rejected": sum(1 for e in entries if not e["ok"])}
    with open(os.path.join(directory, "manifest.json"), "w") as f:
        json.dump(man, f, indent=1)
    return man


if __name__ == "__main__":
    import copy
    import shutil
    import tempfile

    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    good = from_csv(os.path.join(ROOT, "tests", "csv", "fr20_test.csv"), "fr20", tags=["test", "curve"])
    check("the retimed FR20 export is a valid clip", validate(good) == [], "; ".join(validate(good)))
    check("... and plays at its own speed (ok, scale 1.0)",
          good["safety"]["ok"] and good["safety"]["playback_scale"] == 1.0,
          "scale %s, %s" % (good["safety"]["playback_scale"], good["safety"]["reasons"]))
    check("duration from the CSV", abs(good["meta"]["duration_s"] - 18.0) < 1e-6, "%s s" % good["meta"]["duration_s"])
    b = good["meta"]["bounds"]
    check("TCP bounds lie inside the FR20's reach",
          b and all(math.hypot(x, y) < 1.95 for x, y in ((b["min"][0], b["min"][1]), (b["max"][0], b["max"][1]))),
          "min %s max %s" % ([round(x, 3) for x in b["min"]], [round(x, 3) for x in b["max"]]))

    old = from_csv(os.path.join(ROOT, "tests", "csv", "fr20_curve_240.csv"), "fr20")
    check("a clip the player must slow is marked not ok, with the reason",
          not old["safety"]["ok"] and old["safety"]["reasons"] and "slower than designed" in old["safety"]["reasons"][0],
          old["safety"]["reasons"][0] if old["safety"]["reasons"] else "no reason")

    bad = copy.deepcopy(good)
    bad["points"][5]["t"] = bad["points"][4]["t"]
    bad["points"][7]["q"][2] = 400.0
    bad["points"][9]["q"] = bad["points"][9]["q"][:5]
    errs = validate(bad)
    check("validate catches a repeated time, a joint past its limit and a short row",
          any("strictly" in e for e in errs) and any("outside" in e for e in errs) and any("5 joints" in e for e in errs),
          "; ".join(errs[:3]))

    tmp = tempfile.mkdtemp()
    try:
        to_csv(good, os.path.join(tmp, "rt.csv"))
        t2, q2 = P.load_csv(os.path.join(tmp, "rt.csv"))
        same = max(abs(a - b) for r1, r2 in zip(q2, [p["q"] for p in good["points"]]) for a, b in zip(r1, r2))
        check("clip -> CSV -> joints round-trips", same < 1e-5 and len(t2) == len(good["points"]), "max %.1e deg" % same)
        save(good, os.path.join(tmp, "good.json"))
        save(old, os.path.join(tmp, "old.json"))
        check("save / load round-trips", load(os.path.join(tmp, "good.json")) == json.loads(json.dumps(good)))
        man = write_manifest(tmp)
        rej = [e for e in man["clips"] if not e["ok"]]
        check("manifest: 1 ok, 1 rejected with its reason",
              man["ok"] == 1 and man["rejected"] == 1 and rej[0]["id"] == "fr20_curve_240" and rej[0]["reasons"],
              "%d ok, %d rejected" % (man["ok"], man["rejected"]))
    finally:
        shutil.rmtree(tmp)

    # UF850: a J1 swing of 60 deg in 1 s is inside velocity and acceleration
    # but its raised-cosine ends ask for more jerk than UFACTORY allows
    ulim = [tuple(x) for x in _profile("uf850")["robot"]["limits_deg"]]
    tw, qw = P.wiggle_clip([0.0, 0.0, -90.0, 0.0, 30.0, 0.0], 1, 30.0, 1.0, 1, ulim)
    uf = {"schema": SCHEMA, "id": "uf850_swing", "robot": "uf850", "joint_names": good["joint_names"],
          "units": good["units"], "points": [{"t": a, "q": b} for a, b in zip(tw, qw)], "tcp": None,
          "style": {}, "meta": {"duration_s": tw[-1], "tags": [], "bounds": None}}
    s = measure(uf)
    check("jerk is checked where the profile has a limit (UF850)",
          s["jerk_limit"] is not None and s["peak_jerk"] and max(s["peak_jerk"]) > 0,
          "peak %.0f vs %.0f deg/s^3, ok=%s %s" % (max(s["peak_jerk"]), s["jerk_limit"][0], s["ok"], s["reasons"]))
    s2 = measure(dict(uf, robot="fr20"))
    check("... and not where it has none (FR20)", s2["jerk_limit"] is None and not any("jerk" in r for r in s2["reasons"]))

    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
