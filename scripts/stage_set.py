"""A quick-test set for the real FR20: the dance library turned to face the
work zone, every clip starting and ending at one stage home.

The dances are made facing the robot's own front (-X, J1 = 0) and start /
end at HOME. Turning a whole clip about the base (J1 + FACING_DEG) keeps the
motion exactly as designed -- timing, accelerations, self-clearance -- and
only changes where it happens in the room, so each turned clip is checked
against the room again (collision.py, the lab env, every role) and against
J1's limits. Every clip then starts and ends at STAGE HOME = HOME turned the
same way: going from one clip to the next needs no move at all.

    python scripts/stage_set.py            write tests/csv/stage/*.csv, geo/stage/ (clips + manifest)
    python scripts/stage_set.py --self-test

FACING_DEG -60 points the front at the centre of the work zone ("stage" in
envs/volvox_lab.json, 120 deg round from -X). First time: go to the start
of any stage clip (play_ui: Go to start); the player checks the move and
takes a detour if the straight one is not clear.
"""

import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import collision as C  # noqa: E402
import motion_clip as M  # noqa: E402
import robot_profile as RP  # noqa: E402

FACING_DEG = -60.0
SOURCE = os.path.join(ROOT, "geo", "dance")
OUT_CLIPS = os.path.join(ROOT, "geo", "stage")
OUT_CSV = os.path.join(ROOT, "tests", "csv", "stage")
ENV = os.path.join(ROOT, "envs", "volvox_lab.json")
LIMIT_PAD_DEG = 3.0
ZONE_INSET_M = 0.03                 # the TCP this far inside the controller's work area (as moves keep it)


def inset_zones(env, inset=ZONE_INSET_M):
    """The env with its work boxes shrunk by inset on every side."""
    objs = []
    for o in env["objects"]:
        if o["role"] == "work" and o["type"] == "box":
            o = dict(o, size=[max(0.0, x - 2 * inset) for x in o["size"]])
        objs.append(o)
    return dict(env, objects=objs)


def turned(clip, facing):
    """The clip turned about the base by facing degrees on J1 (TCP by FK)."""
    c = json.loads(json.dumps(clip))
    for p in c["points"]:
        p["q"][0] += facing
    c["tcp"] = M._tcp_path(c.get("robot", "fr20"), [p["q"] for p in c["points"]])
    c["id"] = "s_" + clip["id"]
    c.setdefault("meta", {})["stage"] = {"source": clip["id"], "facing_deg": facing}
    return c


def verdict(c, limits, model, env):
    """(ok, reasons, collision report) of a turned clip."""
    j1 = [p["q"][0] for p in c["points"]]
    lo, hi = limits[0]
    if min(j1) < lo + LIMIT_PAD_DEG or max(j1) > hi - LIMIT_PAD_DEG:
        return False, ["J1 %.0f..%.0f outside its limits %g..%g (less %g)" % (min(j1), max(j1), lo, hi, LIMIT_PAD_DEG)], None
    rep = C.check(model, env, [p["t"] for p in c["points"]], [p["q"] for p in c["points"]])
    return rep["ok"], ([] if rep["ok"] else ["cell: " + C.describe(rep)]), rep


def build(facing=FACING_DEG, source=SOURCE, out_clips=OUT_CLIPS, out_csv=OUT_CSV):
    prof = RP.load("fr20")
    limits = [tuple(x) for x in prof["robot"]["limits_deg"]]
    home = RP.home(prof)
    stage_home = [home[0] + facing] + list(home[1:])
    model, env = C.load_model("fr20"), inset_zones(C.load_env(ENV))
    os.makedirs(out_clips, exist_ok=True)
    os.makedirs(out_csv, exist_ok=True)
    for f in glob.glob(os.path.join(out_clips, "*.json")) + glob.glob(os.path.join(out_csv, "*.csv")):
        os.remove(f)
    kept, dropped = [], []
    for f in sorted(glob.glob(os.path.join(source, "d*.json"))):
        src = M.load(f)
        if not (src.get("safety") or {}).get("ok") or not src.get("points"):
            continue
        c = turned(src, facing)
        ok, reasons, rep = verdict(c, limits, model, env)
        s = c.setdefault("safety", {})
        s["ok"], s["reasons"] = ok, reasons
        if rep:
            # as the factories write them: the room, and the arm to itself
            s["collision"], s["min_clearance_m"] = C.describe(rep), rep["min_env_clearance_m"]
            s["min_self_clearance_m"] = rep["min_self_clearance_m"]
        M.save(c, os.path.join(out_clips, c["id"] + ".json"))
        if ok:
            M.to_csv(c, os.path.join(out_csv, c["id"] + ".csv"))
            kept.append(c["id"])
        else:
            dropped.append((c["id"], reasons[0]))
    M.write_manifest(out_clips)
    # a still clip at stage home: "Go to start" with it takes the arm there
    M.to_csv({"points": [{"t": 0.0, "q": stage_home}, {"t": 1.0, "q": stage_home}]},
             os.path.join(out_csv, "_stage_home.csv"))
    info = {"facing_deg": facing, "stage_home_deg": stage_home, "env": os.path.relpath(ENV, ROOT).replace("\\", "/"),
            "clips": kept, "dropped": [{"id": i, "why": w} for i, w in dropped]}
    with open(os.path.join(out_csv, "stage_set.json"), "w") as fh:
        json.dump(info, fh, indent=1)
    return info


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    src = M.load(os.path.join(ROOT, "tests", "clips", "d17_punch-float-punch.json"))
    c = turned(src, -60.0)
    check("J1 turned, every other joint and every time unchanged",
          all(abs(a["q"][0] - b["q"][0] + 60.0) < 1e-9 and a["q"][1:] == b["q"][1:] and a["t"] == b["t"]
              for a, b in zip(c["points"], src["points"])))
    r0 = sum(x * x for x in src["tcp"][0][:2]) ** 0.5
    r1 = sum(x * x for x in c["tcp"][0][:2]) ** 0.5
    check("the TCP turns about the base: same radius and height", abs(r0 - r1) < 1e-6 and abs(src["tcp"][0][2] - c["tcp"][0][2]) < 1e-6,
          "%.4f / %.4f" % (r0, r1))
    check("the source clip is not changed", src["points"][0]["q"][0] != c["points"][0]["q"][0])
    prof = RP.load("fr20")
    limits = [tuple(x) for x in prof["robot"]["limits_deg"]]
    far = turned(src, 170.0)
    ok, why, _ = verdict(far, limits, C.load_model("fr20"), C.load_env(ENV))
    check("a turn past J1's limit is dropped", not ok and "J1" in why[0], why)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--facing", type=float, default=FACING_DEG, help="J1 turn, deg (default %(default)s: the work zone)")
    a = ap.parse_args()
    info = build(a.facing)
    print("stage home %s" % info["stage_home_deg"])
    print("%d clips -> %s" % (len(info["clips"]), os.path.relpath(OUT_CSV, ROOT)))
    for d in info["dropped"]:
        print("  dropped %s: %s" % (d["id"], d["why"][:110]))
