"""Find clips and chain them into a show.

    python scripts/clip_library.py search --action punch --level high
    python scripts/clip_library.py search --sequence float-punch --max-s 20
    python scripts/clip_library.py sequence d01_punch-punch d17_punch-float-punch --out geo/show.csv

Clips come from geo/dance, geo/clips (the factories) and tests/clips. A
search matches the MEASURED labels (motion_labels.py): the clip's action,
any of its bars, its level, its tags.

A sequence joins clips end to start. Dance phrases all start and end at
rest in HOME, so they join directly; any other join gets a transition -- a
minimum-jerk move in joint space, as long as the joint limits need
(choreo._travel_time), checked against the cell like everything else. The
whole show is then measured as the player plays it, checked against the
cell, labelled, and written as the player's CSV + the clip JSON.

    load_library(dirs)              -> [clip entries]
    search(entries, ...)            -> [entries]
    sequence(clips, env=None)       -> clip

Pure Python. Tests: python scripts/clip_library.py --self-test
"""

import argparse
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import motion_clip as M  # noqa: E402

DIRS = [os.path.join(ROOT, d) for d in ("geo/dance", "geo/clips", "tests/clips")]


def load_library(dirs=None):
    out = []
    for d in dirs or DIRS:
        for f in sorted(glob.glob(os.path.join(d, "*.json"))):
            if os.path.basename(f) == "manifest.json":
                continue
            try:
                c = M.load(f)
            except ValueError:
                continue
            if c.get("schema") != M.SCHEMA:
                continue
            lab = c.get("labels") or {}
            out.append({"id": c["id"], "file": f, "ok": bool(c.get("safety", {}).get("ok")),
                        "duration_s": c.get("meta", {}).get("duration_s", 0.0),
                        "action": lab.get("measured", {}).get("action"),
                        "sequence": lab.get("sequence") or [],
                        "level": max(lab.get("descriptors", {}).get("level", {"?": 1}).items(), key=lambda kv: kv[1])[0],
                        "tags": list(lab.get("tags", [])) + list(c.get("meta", {}).get("tags", []))})
    return out


def search(entries, action=None, sequence=None, level=None, tag=None, min_s=None, max_s=None, ok_only=True):
    seq = sequence.split("-") if isinstance(sequence, str) else sequence
    res = []
    for e in entries:
        if ok_only and not e["ok"]:
            continue
        if action and action != e["action"] and action not in e["sequence"]:
            continue
        if seq and not any(e["sequence"][i:i + len(seq)] == seq for i in range(len(e["sequence"]))):
            continue
        if level and level != e["level"]:
            continue
        if tag and tag not in e["tags"]:
            continue
        if min_s is not None and e["duration_s"] < min_s:
            continue
        if max_s is not None and e["duration_s"] > max_s:
            continue
        res.append(e)
    return res


def _transition(qa, qb, fps=24.0):
    import choreo
    kin = choreo.Kin()
    T = max(1.0, choreo._travel_time(kin, qa, qb) * 1.3)
    n = int(math.ceil(T * fps))
    return [[a + (b - a) * choreo._minjerk(k / float(n)) for a, b in zip(qa, qb)] for k in range(1, n + 1)]


def sequence(clips, env=None, clip_id="show", fps=24.0, join_tol_deg=0.5):
    """One clip from several, joined end to start (transitions where they
    do not meet). env: a collision env (dict) or path; None skips the cell."""
    import collision
    import motion_labels
    qs, parts, transitions = [], [], []
    for c in clips:
        cq = [p["q"] for p in c["points"]]
        if not cq:
            raise ValueError("clip %s has no points (rejected?)" % c.get("id"))
        if qs:
            if max(abs(a - b) for a, b in zip(qs[-1], cq[0])) > join_tol_deg:
                tr = _transition(qs[-1], cq[0], fps)
                transitions.append({"before": c["id"], "frames": len(tr), "s": round(len(tr) / fps, 3)})
                qs.extend(tr)
            else:
                cq = cq[1:]                           # the same pose: do not hold a frame twice
        parts.append({"id": c["id"], "t0": round(len(qs) / fps, 3)})
        qs.extend(cq)
    ts = [i / fps for i in range(len(qs))]
    show = {"schema": M.SCHEMA, "id": clip_id, "robot": clips[0].get("robot", "fr20"),
            "joint_names": clips[0]["joint_names"], "units": clips[0]["units"],
            "points": [{"t": round(t, 6), "q": [round(x, 5) for x in q]} for t, q in zip(ts, qs)],
            "tcp": M._tcp_path(clips[0].get("robot", "fr20"), qs),
            "style": {"generator": "clip_library.sequence", "parts": parts, "transitions": transitions},
            "meta": {"duration_s": round(ts[-1], 6), "tags": ["sequence"], "source": {"clips": [c["id"] for c in clips]}}}
    xs = list(zip(*show["tcp"]))
    show["meta"]["bounds"] = {"min": [min(a) for a in xs], "max": [max(a) for a in xs]}
    M.measure(show)
    if env is not None:
        cell = collision.load_env(env) if isinstance(env, str) else env
        rep = collision.check(collision.load_model(show["robot"]), cell, ts, qs)
        show["safety"]["collision"] = collision.describe(rep)
        show["safety"]["min_clearance_m"] = rep["min_env_clearance_m"]
        if not rep["ok"]:
            show["safety"]["ok"] = False
            show["safety"]["reasons"] = ["cell: " + collision.describe(rep)] + show["safety"]["reasons"]
    show["labels"] = motion_labels.label(show)
    return show


def self_test():
    import clip_factory
    import collision
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    lib = load_library([os.path.join(ROOT, "tests", "clips")])
    check("the sample clips load with their labels", len(lib) >= 3 and all(e["action"] for e in lib),
          ", ".join("%s: %s %s" % (e["id"], e["action"], "-".join(e["sequence"])) for e in lib))
    punch = search(lib, action="punch")
    check("search by action finds the punch phrases", punch and all("punch" in [e["action"]] + e["sequence"] for e in punch),
          ", ".join(e["id"] for e in punch))
    env = collision.load_env(os.path.join(ROOT, "envs", "volvox_lab.json"))
    a, b = (M.load(e["file"]) for e in lib[:2])
    s = sequence([a, b], env)
    check("two HOME-to-HOME phrases join without a transition",
          not s["style"]["transitions"] and abs(s["meta"]["duration_s"] - (a["meta"]["duration_s"] + b["meta"]["duration_s"])) < 0.05,
          "%.2f s = %.2f + %.2f" % (s["meta"]["duration_s"], a["meta"]["duration_s"], b["meta"]["duration_s"]))
    check("... and the show plays at its own speed and clears the cell", s["safety"]["ok"],
          "%s; %s" % (s["safety"].get("playback_scale"), s["safety"].get("collision")))
    circle = clip_factory.make({"id": "circle_front", "primitive": "circle", "center": [-0.8, 0.0, 0.9], "size": 0.15,
                                "plane": "xy", "safety": 0.8})
    s2 = sequence([a, circle, b], env)
    tr = s2["style"]["transitions"]
    check("a clip that does not start where the last ended gets a transition, both ways", len(tr) == 2,
          "; ".join("%s s before %s" % (t["s"], t["before"]) for t in tr))
    check("... and that show still plays at its own speed and clears the cell", s2["safety"]["ok"],
          "%s; %s" % (s2["safety"].get("reasons"), s2["safety"].get("collision")))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", nargs="?", choices=("search", "sequence"))
    ap.add_argument("ids", nargs="*")
    ap.add_argument("--action")
    ap.add_argument("--sequence")
    ap.add_argument("--level", choices=("low", "mid", "high"))
    ap.add_argument("--tag")
    ap.add_argument("--min-s", type=float)
    ap.add_argument("--max-s", type=float)
    ap.add_argument("--env", default=os.path.join(ROOT, "envs", "volvox_lab.json"))
    ap.add_argument("--out", default=os.path.join(ROOT, "geo", "show.csv"))
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    lib = load_library()
    if a.cmd == "search":
        for e in search(lib, a.action, a.sequence, a.level, a.tag, a.min_s, a.max_s):
            print("%-36s %5.1f s  %-6s %-5s %s" % (e["id"], e["duration_s"], e["action"], e["level"], "-".join(e["sequence"])))
        return 0
    if a.cmd == "sequence":
        by_id = {e["id"]: e for e in lib}
        missing = [i for i in a.ids if i not in by_id]
        if missing:
            ap.error("no clip %s" % missing)
        show = sequence([M.load(by_id[i]["file"]) for i in a.ids], a.env if a.env else None)
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        M.to_csv(show, a.out)
        M.save(show, os.path.splitext(a.out)[0] + ".json")
        s = show["safety"]
        print("%s: %.1f s, %s; %d transitions; %s" % (a.out, show["meta"]["duration_s"],
                                                     "ok" if s["ok"] else "REJECTED " + "; ".join(s["reasons"]),
                                                     len(show["style"]["transitions"]), s.get("collision", "")))
        return 0 if s["ok"] else 1
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
