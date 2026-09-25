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
hand on the E-stop, as for any hardware run. Then set the profile's
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

LEVELS = (150, 225, 300, 450, 600, 900)


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
    a = ap.parse_args(argv)
    if not 1 <= a.joint <= 6 or not 0 < a.amp <= 10:
        ap.error("--joint 1-6, --amp in (0, 10]")
    ctrl = P.Controller(a.ip)
    report = {"target": "hardware" if a.hardware else "sim", "controller": ctrl.model(), "joint": a.joint,
              "amp_deg": a.amp, "levels": [], "clean_up_to": None}
    q0 = ctrl.joints()
    print("J%d from %.2f deg, +%.1f deg and back, levels %s deg/s^2" % (a.joint, q0[a.joint - 1], a.amp, a.levels))
    for acc in a.levels:
        samples, dt, info = probe_clip(q0, a.joint, a.amp, acc)
        if a.hardware and not a.yes:
            if input("  %g deg/s^2 (%.2f s each way, %.0f deg/s peak): yes to move > "
                     % (acc, info["move_s"], info["peak_vel_deg_s"])).strip().lower() != "yes":
                report["stopped"] = "not confirmed at %g" % acc
                break
        try:
            pb, _, _ = P.play(ctrl, a.ip, samples, dt, move_vel_pct=10.0)
        except Exception as e:
            report["levels"].append(dict(info, acc=acc, error=str(e)[:200]))
            report["stopped"] = "error at %g: %s" % (acc, str(e)[:120])
            print("  %6g  ERROR %s" % (acc, e))
            break
        err = pb.get("tracking_after_lag_max_deg")
        errs = pb.get("controller_error_after")
        row = dict(info, acc=acc, tracking_after_lag_max_deg=err, lag_ms=pb.get("best_lag_ms"),
                   controller_error_after=errs, skipped=pb.get("skipped"))
        report["levels"].append(row)
        bad = (errs and any(errs)) or err is None or err > a.max_error
        print("  %6g deg/s^2  %.2f s  tracking %s deg  lag %s ms  errors %s  %s"
              % (acc, info["move_s"], err, pb.get("best_lag_ms"), errs, "STOP" if bad else "ok"))
        if bad:
            report["stopped"] = "tracking %s deg / errors %s at %g" % (err, errs, acc)
            break
        report["clean_up_to"] = acc
        time.sleep(0.5)
    print("clean up to %s deg/s^2 on J%d" % (report["clean_up_to"], a.joint))
    if a.report:
        with open(a.report, "w") as f:
            json.dump(report, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
