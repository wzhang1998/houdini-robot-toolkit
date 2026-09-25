"""Menu front end for fairino_player.py, driven by playback.toml.

    python scripts/play.py            menu; the config is re-read before each step
    python scripts/play.py 5          run step 5 directly
    python scripts/play.py --self-test

Edit playback.toml (next to scripts/) instead of typing arguments. This only
turns the file into fairino_player.py's arguments -- all motion logic, and
its safety behaviour (--sim / --hardware, the hardware confirmation), stays
in the player.
"""

import os
import sys
import time
import tomllib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CONFIG = os.path.join(ROOT, "playback.toml")
sys.path.insert(0, HERE)

STEPS = [
    ("check", "read-only: controller, errors, pose, FK vs URDF"),
    ("dry-run", "no robot: how the clip is slowed, per-joint peaks"),
    ("goto-start", "MoveJ to the clip's first pose"),
    ("wiggle", "one joint out and back from the current pose"),
    ("play", "play the clip"),
]


def load(path=CONFIG):
    with open(path, "rb") as f:
        return tomllib.load(f)


def _stamped(csv_path, suffix, ext):
    stem = os.path.splitext(csv_path)[0]
    return "%s_%s_%s.%s" % (stem, suffix, time.strftime("%Y%m%d-%H%M%S"), ext)


def build_argv(cfg, step):
    """playback.toml + a step name -> fairino_player.py arguments."""
    r, c, m = cfg["robot"], cfg.get("clip", {}), cfg.get("motion", {})
    target = r.get("target", "sim")
    if target not in ("sim", "hardware"):
        raise ValueError('robot.target must be "sim" or "hardware", got %r' % target)
    base = ["--ip", str(r["ip"]), "--profile", r.get("profile", "fr20")]
    if step == "check":
        return base + ["--check"]

    argv = list(base)
    if step == "wiggle":
        w = cfg.get("wiggle", {})
        argv += ["--wiggle", str(w.get("joint", 6)), str(w.get("amp_deg", 5)),
                 str(w.get("period_s", 4)), str(w.get("cycles", 2))]
    else:
        csv = c.get("csv")
        if not csv:
            raise ValueError("clip.csv is empty in playback.toml")
        if not os.path.isabs(csv):
            csv = os.path.join(ROOT, csv)
        if not os.path.isfile(csv):
            raise ValueError("clip.csv does not exist: %s" % csv)
        argv = [csv] + argv

    argv += ["--speed", str(m.get("speed", 0.3)), "--rate", str(m.get("rate_hz", 125)),
             "--acc-limit", str(m.get("acc_limit", 150)), "--move-vel", str(m.get("move_vel", 20))]
    if step == "dry-run":
        return argv + ["--dry-run"]

    argv += ["--" + target]
    if target == "hardware" and not m.get("confirm", True):
        argv += ["--yes"]
    if step == "goto-start":
        return argv + ["--goto-start"]
    if step == "play":
        if c.get("record"):
            argv += ["--record", _stamped(csv, "actual_" + target, "csv")]
        if c.get("report"):
            argv += ["--report", _stamped(csv, target, "json")]
    return argv


def header(cfg):
    r, c, m = cfg["robot"], cfg.get("clip", {}), cfg.get("motion", {})
    tgt = r.get("target", "?").upper()
    warn = "   <<< REAL ARM" if tgt == "HARDWARE" else ""
    w = cfg.get("wiggle", {})
    return ("\n%s  %s  profile %s%s\n  clip   %s\n  speed  %g   rate %g Hz   acc %g   MoveJ %g %%\n"
            "  wiggle J%s %+g deg, %g s x %s"
            % (tgt, r.get("ip"), r.get("profile", "fr20"), warn,
               os.path.basename(c.get("csv", "") or "(none)"),
               m.get("speed", 0.3), m.get("rate_hz", 125), m.get("acc_limit", 150), m.get("move_vel", 20),
               w.get("joint", 6), w.get("amp_deg", 5), w.get("period_s", 4), w.get("cycles", 2)))


def run(step):
    import fairino_player
    cfg = load()
    argv = build_argv(cfg, step)
    print("> fairino_player.py " + " ".join(argv))
    try:
        fairino_player.main(argv)
    except SystemExit as e:
        if e.code not in (0, None):
            print("stopped: %s" % e.code)
    except Exception as e:
        print("ERROR: %s" % e)


def menu():
    while True:
        try:
            print(header(load()))
        except Exception as e:
            print("\nplayback.toml: %s" % e)
        for i, (name, desc) in enumerate(STEPS, start=1):
            print("  %d  %-11s %s" % (i, name, desc))
        choice = input("step (1-%d, e = open config, q = quit): " % len(STEPS)).strip().lower()
        if choice in ("q", "quit", ""):
            return
        if choice == "e":
            os.startfile(CONFIG) if hasattr(os, "startfile") else print(CONFIG)
            continue
        if choice.isdigit() and 1 <= int(choice) <= len(STEPS):
            run(STEPS[int(choice) - 1][0])
        else:
            print("?")


def self_test():
    cfg = {"robot": {"target": "hardware", "ip": "10.0.0.5", "profile": "fr20"},
           "clip": {"csv": __file__, "record": True, "report": False},
           "motion": {"speed": 0.3, "rate_hz": 125, "acc_limit": 300, "move_vel": 10, "confirm": True},
           "wiggle": {"joint": 6, "amp_deg": 5, "period_s": 4, "cycles": 2}}
    fails = []

    def check(label, ok, got):
        print("%s  %s  -- %s" % ("ok  " if ok else "FAIL", label, got))
        if not ok:
            fails.append(label)

    a = build_argv(cfg, "check")
    check("check is read-only (no --sim/--hardware, no clip)", "--check" in a and "--hardware" not in a, a)
    a = build_argv(cfg, "dry-run")
    check("dry-run never names a target", "--dry-run" in a and "--hardware" not in a, a)
    a = build_argv(cfg, "play")
    check("play on hardware: --hardware, speed 0.3, record, asks to confirm",
          "--hardware" in a and a[a.index("--speed") + 1] == "0.3" and "--record" in a and "--yes" not in a, a)
    a = build_argv(cfg, "wiggle")
    check("wiggle takes its joint/amp/period/cycles", a[a.index("--wiggle") + 1:a.index("--wiggle") + 5] == ["6", "5", "4", "2"], a)
    bad = dict(cfg, robot=dict(cfg["robot"], target="real"))
    try:
        build_argv(bad, "play")
        check("an unknown target is refused", False, "accepted")
    except ValueError as e:
        check("an unknown target is refused", True, e)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        n = int(sys.argv[1])
        if not 1 <= n <= len(STEPS):
            sys.exit("step must be 1-%d" % len(STEPS))
        run(STEPS[n - 1][0])
    else:
        menu()
