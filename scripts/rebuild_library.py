"""A show's clip library rebuilt in plain Python, beside the tracked one.

`show.py build` makes every motion of a show (choreo phrases, gestures and
paths, the moves, the scan) without Houdini, but writes the show's own
shows/<show>.compiled.json. This builds the same into geo/rebuild/<show>/
(not in git) -- the generators' full clips (style, labels, per-bar intent,
safety) as show.write_preview writes them, plus compiled.json -- and checks
it against the tracked compiled file: the same segments, the same samples,
the same labels. Then the review sheets (review_render.py) read the full clips.

    uv run scripts/rebuild_library.py shows/party.json          # ~3-10 min
    uv run scripts/rebuild_library.py shows/party.json --check-only
    uv run scripts/rebuild_library.py --self-test

Exit 1 when the rebuild differs from the tracked file (an input changed
since it was built, or a generator is not deterministic); warnings from
build checks added since are listed, not counted.
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

OUT = os.path.join(ROOT, "geo", "rebuild")


def compare(tracked, rebuilt, tol=1e-6):
    """[differences] between two compiled graphs (dicts): segment names and
    order, samples (t, q within tol), labels, hubs; and the warnings only in
    one of them, apart: (diffs, warnings_only_tracked, warnings_only_rebuilt)."""
    diffs = []
    na = [s["name"] for s in tracked["segments"]]
    nb = [s["name"] for s in rebuilt["segments"]]
    if na != nb:
        diffs.append("segments differ: only tracked %s, only rebuilt %s"
                     % (sorted(set(na) - set(nb)), sorted(set(nb) - set(na))))
    for sa, sb in zip(tracked["segments"], rebuilt["segments"]):
        if sa["name"] != sb["name"]:
            continue
        if len(sa["t"]) != len(sb["t"]):
            diffs.append("%s: %d samples, rebuilt %d" % (sa["name"], len(sa["t"]), len(sb["t"])))
            continue
        dq = max((abs(x - y) for qa, qb in zip(sa["q"], sb["q"]) for x, y in zip(qa, qb)), default=0.0)
        dt = max((abs(x - y) for x, y in zip(sa["t"], sb["t"])), default=0.0)
        if dq > tol or dt > tol:
            diffs.append("%s: joints differ by %.3g deg, times by %.3g s" % (sa["name"], dq, dt))
        if json.dumps(sa.get("labels"), sort_keys=True) != json.dumps(sb.get("labels"), sort_keys=True):
            diffs.append("%s: labels differ" % sa["name"])
    for k in set(tracked["hubs"]) | set(rebuilt["hubs"]):
        a, b = tracked["hubs"].get(k), rebuilt["hubs"].get(k)
        if a is None or b is None or max(abs(x - y) for x, y in zip(a, b)) > tol:
            diffs.append("hub %s: %s, rebuilt %s" % (k, a, b))
    wa = set((tracked.get("info") or {}).get("warnings") or [])
    wb = set((rebuilt.get("info") or {}).get("warnings") or [])
    return diffs, sorted(wa - wb), sorted(wb - wa)


def rebuild(cfg_path, out_root=OUT, log=print):
    import show
    name = os.path.splitext(os.path.basename(cfg_path))[0]
    out = os.path.join(out_root, name)
    t0 = time.time()
    g = show.build(os.path.abspath(cfg_path), log=log)
    show.write_preview(g, out)
    g.save(os.path.join(out, "compiled.json"))
    log("wrote %s: %d segments (%d idle) in %.0f s" % (os.path.relpath(out, ROOT), len(g.segments), len(g.idle()),
                                                        time.time() - t0))
    return out


def check(cfg_path, out_root=OUT, log=print):
    name = os.path.splitext(os.path.basename(cfg_path))[0]
    tracked = os.path.splitext(cfg_path)[0] + ".compiled.json"
    rebuilt = os.path.join(out_root, name, "compiled.json")
    diffs, only_t, only_r = compare(json.load(open(tracked)), json.load(open(rebuilt)))
    for d in diffs:
        log("DIFFERS " + d)
    for w in only_r:
        log("warning only in the rebuild (a check added since?): " + w)
    for w in only_t:
        log("warning only in the tracked file: " + w)
    log("%s: %s" % (os.path.relpath(rebuilt, ROOT), "the same motions as %s" % os.path.relpath(tracked, ROOT)
                    if not diffs else "%d differences from %s" % (len(diffs), os.path.relpath(tracked, ROOT))))
    return not diffs


def self_test():
    seg = lambda n, q, lab=None: {"name": n, "kind": "idle", "start": "a", "end": "a", "labels": lab or {},  # noqa
                                  "t": [0.0, 1.0], "q": [q, q]}
    a = {"hubs": {"a": [0.0] * 6}, "segments": [seg("x", [0.0] * 6), seg("y", [1.0] * 6, {"k": 1})],
         "info": {"warnings": ["w1"]}}
    b = json.loads(json.dumps(a))
    ok = compare(a, b) == ([], [], [])
    b["segments"][1]["q"][1][2] += 0.01
    b["info"]["warnings"].append("w2")
    d, ot, orb = compare(a, b)
    ok &= len(d) == 1 and "y" in d[0] and ot == [] and orb == ["w2"]
    b = json.loads(json.dumps(a))
    b["segments"][1]["labels"]["k"] = 2
    b["segments"].append(seg("z", [0.0] * 6))
    d, _, _ = compare(a, b)
    ok &= any("labels" in x for x in d) and any("only rebuilt ['z']" in x for x in d)
    print("self-test %s" % ("passed" if ok else "FAILED"))
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config")
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--check-only", action="store_true", help="compare an earlier rebuild, do not build")
    a = ap.parse_args(argv)
    if not a.check_only:
        rebuild(a.config, a.out)
    return 0 if check(a.config, a.out) else 1


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
