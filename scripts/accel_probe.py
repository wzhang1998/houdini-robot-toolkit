"""How hard can this arm accelerate? A stepped test on one joint.

The choreography, the Retime and Pre-Flight all plan with the profile's
acceleration limit -- 150 deg/s^2 on FR20, the manual's only figure, given
for a 20-25 kg extended load. With nothing on the flange the arm can very
likely do more, and at 150 a heavy arm cannot make a quick move bigger than
a few degrees (choreo.py). This measures instead of guessing:

    for each level (deg/s^2): the joint goes out and back by --amp degrees on
    a minimum-jerk profile whose peak acceleration is that level, streamed
    by ServoJ exactly as fairino_player.py plays clips; the actual joint is
    recorded, lag-aligned, compared with the command
    stop at the first controller error, or when tracking after lag passes
    --max-error degrees; report the last clean level

    python scripts/accel_probe.py --sim --joint 6
    python scripts/accel_probe.py --hardware --ip IP --joint 6 --amp 3
    python scripts/accel_probe.py --hardware --ip IP --joint 1 --amp 2 --levels 150 225 300 450

Start on J6 (the lightest), small amplitudes, then J4/J5, then J1-J3. It
moves the arm: --hardware asks before every level. A clear workspace and a
hand on the E-stop, as for any hardware run.

Before any motion it checks the test from the current pose: the joint stays
inside its limits, and the whole out-and-back clears the room (--env, the
Pre-Flight cell check); a failed check refuses to move. --check-only does
just that and exits. --direction -1 tests the other way. J1-J3 depend on
the pose (arm stretched: more inertia, more gravity) -- a result holds for
poses like the one it was measured in; the report keeps the pose.
scripts/accel_ui.py is the window for this. Then set the profile's
robot.max_acceleration_deg_s2 (per joint if they differ) at a margin below
the clean level, and re-run Retime / the factories.
"""

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import fairino_player as P  # noqa: E402

ROOT = os.path.dirname(HERE)
LEVELS = (150, 225, 300, 450, 600, 900)
ENV = os.path.join(ROOT, "envs", "volvox_lab.json")


def precheck(q0, joint, amp, limits, env_path=ENV, profile="fr20"):
    """Problems with testing `joint` by `amp` degrees (signed) from q0,
    before anything moves: [] when the joint stays in its limits and the
    out-and-back clears the room. Also returns {tcp_m, min_env_clearance_m}."""
    probs, info = [], {}
    lo, hi = limits[joint - 1]
    for label, q in (("start", q0[joint - 1]), ("end", q0[joint - 1] + amp)):
        if not lo <= q <= hi:
            probs.append("J%d %s %.2f deg outside its limits %g..%g" % (joint, label, q, lo, hi))
    if env_path:
        import collision as CL
        model = CL.load_model(profile)
        env = CL.load_env(env_path)
        samples, dt, _ = probe_clip(q0, joint, amp, 150.0)
        rep = CL.check(model, env, [i * dt for i in range(len(samples))], samples)
        info["tcp_m"] = [round(x, 4) for x in CL.capsules(model, q0)[1]]
        info["min_env_clearance_m"] = rep.get("min_env_clearance_m")
        if not rep["ok"]:
            probs.append("room: " + CL.describe(rep))
    return probs, info


def probe_clip(q0, joint, amp, acc, rate=125.0, rest=0.4, hold=0.3):
    """Equal-interval samples: rest, out by amp, hold, back, rest. The
    out / back moves are minimum-jerk with peak acceleration acc."""
    T = math.sqrt(5.7735 * abs(amp) / acc)
    dt = 1.0 / rate
    seg = [(rest, 0.0, 0.0), (T, 0.0, amp), (hold, amp, amp), (T, amp, 0.0), (rest, 0.0, 0.0)]
    out = []
    for dur, a, b in seg:
        n = max(1, int(round(dur / dt)))
        for k in range(n):
            u = k / float(n)
            s = u * u * u * (10 - 15 * u + 6 * u * u)
            q = list(q0)
            q[joint - 1] = q0[joint - 1] + a + (b - a) * s
            out.append(q)
    out.append(list(q0))
    return out, dt, {"move_s": round(T, 3), "peak_vel_deg_s": round(1.875 * abs(amp) / T, 1)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.116.128")
    tgt = ap.add_mutually_exclusive_group(required=True)
    tgt.add_argument("--sim", action="store_true")
    tgt.add_argument("--hardware", action="store_true")
    ap.add_argument("--joint", type=int, default=6)
    ap.add_argument("--amp", type=float, default=3.0, help="degrees out and back")
    ap.add_argument("--levels", type=float, nargs="+", default=list(LEVELS))
    ap.add_argument("--max-error", type=float, default=0.5, help="tracking after lag, degrees")
    ap.add_argument("--yes", action="store_true", help="hardware: do not ask before each level")
    ap.add_argument("--report", help="write the JSON report here")
    ap.add_argument("--direction", type=int, choices=(1, -1), default=1, help="+1 or -1: which way the joint turns")
    ap.add_argument("--env", default=ENV, help='the room to check the test against ("" = no check)')
    ap.add_argument("--profile", default="fr20")
    ap.add_argument("--check-only", action="store_true", help="read the pose, check the test, do not move")
    a = ap.parse_args(argv)
    if not 1 <= a.joint <= 6 or not 0 < a.amp <= 10:
        ap.error("--joint 1-6, --amp in (0, 10]")
    import robot_profile
    limits = [tuple(x) for x in robot_profile.load(a.profile, os.path.join(ROOT, "profiles"))["robot"]["limits_deg"]]
    amp = a.amp * a.direction
    ctrl = P.Controller(a.ip)
    q0 = ctrl.joints()
    probs, pinfo = precheck(q0, a.joint, amp, limits, a.env, a.profile)
    report = {"target": "hardware" if a.hardware else "sim", "controller": ctrl.model(), "joint": a.joint,
              "amp_deg": amp, "pose_deg": [round(x, 3) for x in q0], "levels": [], "clean_up_to": None,
              "precheck": probs or "ok", **pinfo}
    print("J%d from %.2f deg, %+.1f deg and back, levels %s deg/s^2; pose %s" % (
        a.joint, q0[a.joint - 1], amp, a.levels, report["pose_deg"]))
    print("pre-check: %s" % ("; ".join(probs) if probs else "ok -- joint in its limits, the test clears the room"))
    if probs or a.check_only:
        if probs:
            report["stopped"] = "pre-check: " + "; ".join(probs)
        print(json.dumps(report))
        return 2 if probs else 0
    for acc in a.levels:
        samples, dt, info = probe_clip(q0, a.joint, amp, acc)
        if a.hardware and not a.yes:
            if input("  %g deg/s^2 (%.2f s each way, %.0f deg/s peak): yes to move > "
                     % (acc, info["move_s"], info["peak_vel_deg_s"])).strip().lower() != "yes":
                report["stopped"] = "not confirmed at %g" % acc
                break
        try:
            pb, _, fb = P.play(ctrl, a.ip, samples, dt, move_vel_pct=10.0)
        except Exception as e:
            report["levels"].append(dict(info, acc=acc, error=str(e)[:200]))
            report["stopped"] = "error at %g: %s" % (acc, str(e)[:120])
            print("  %6g  ERROR %s" % (acc, e))
            break
        err = pb.get("tracking_after_lag_max_deg")
        errs = pb.get("controller_error_after")
        # what the joint actually did: J6 turns the bare flange about its own
        # axis, which is next to invisible -- the feedback says whether it moved
        vals = [fq[a.joint - 1] for _, fq in fb] if fb else []
        moved = round(max(vals) - min(vals), 3) if vals else None
        row = dict(info, acc=acc, tracking_after_lag_max_deg=err, lag_ms=pb.get("best_lag_ms"),
                   controller_error_after=errs, skipped=pb.get("skipped"), actual_travel_deg=moved)
        report["levels"].append(row)
        bad = (errs and any(errs)) or err is None or err > a.max_error
        print("  %6g deg/s^2  %.2f s  moved %s of %g deg  tracking %s deg  lag %s ms  errors %s  %s"
              % (acc, info["move_s"], moved, abs(amp), err, pb.get("best_lag_ms"), errs, "STOP" if bad else "ok"))
        if bad:
            report["stopped"] = "tracking %s deg / errors %s at %g" % (err, errs, acc)
            break
        report["clean_up_to"] = acc
        time.sleep(0.5)
    print("clean up to %s deg/s^2 on J%d" % (report["clean_up_to"], a.joint))
    if a.report:
        with open(a.report, "w") as f:
            json.dump(report, f, indent=1)
    print(json.dumps(report))
    return 0


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    import robot_profile
    limits = [tuple(x) for x in robot_profile.load("fr20", os.path.join(ROOT, "profiles"))["robot"]["limits_deg"]]
    home = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    s, _, _ = probe_clip(home, 6, -3.0, 300.0)
    check("a negative amplitude turns the joint the other way", min(q[5] for q in s) < -2.99 and max(q[5] for q in s) <= 0.0)
    p, info = precheck(home, 6, 3.0, limits)
    check("J6 at HOME passes (limits and room)", not p, p)
    check("... and reports where the tool is", info.get("tcp_m") and info["tcp_m"][2] > 0.8, info)
    near = [0.0, -90.0, 90.0, 84.0, -90.0, 0.0]
    p, _ = precheck(near, 4, 3.0, limits, env_path="")
    check("J4 at 84 deg + 3 is refused: its limit is 85", p and "limits" in p[0], p)
    p, _ = precheck(near, 4, -3.0, limits, env_path="")
    check("... the other way it is fine", not p, p)
    reach = [0.0, -2.7, 10.0, -181.5, -92.8, 0.8]      # a real probe pose: tool on the TV wall, arm low
    p, _ = precheck(reach, 1, 3.0, limits)
    check("a test from a pose touching the room is refused by the room check", p and p[0].startswith("room"), p)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
