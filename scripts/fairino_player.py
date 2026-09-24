"""Play a joint-angle CSV on a Fairino arm by ServoJ streaming.

The CSV is the asset's export (frame, time_s, j1_deg..j6_deg[, speed_pct]),
robot frame, degrees. Checked against a Fairino controller in SimMachine:
URDF zero == controller zero and the signs agree, so the exported angles go
to the controller unchanged.

Playback follows the findings in td-robot-twin's PLAYBACK_FINDINGS.md, which
came from making the same thing smooth on UF850:

  1. condition the whole path before anything moves -- shape-preserving
     cubic interpolation per joint, at rest at both ends, resampled at equal
     intervals near the control rate, and slowed uniformly (never clipped
     per joint) until it fits the velocity / acceleration envelope;
  2. move to the first sample with a controller-planned MoveJ;
  3. stream ServoJ on an absolute clock -- deadline[n] = start + t[n] -- and
     when late, send the newest due sample and count the skipped ones rather
     than bursting a backlog;
  4. read feedback on its own connection so it never delays a send;
  5. report what actually happened: effective rate, skips, lateness,
     duration scale, tracking error.

Transport is the controller's XML-RPC on port 20003 -- what the official SDK
(FAIR-INNOVATION/fairino-python-sdk) calls underneath -- so this needs only
the standard library.

Usage:
    python scripts/fairino_player.py clip.csv --ip 192.168.116.128 --sim
    python scripts/fairino_player.py --self-test

--sim is required: this has only been run against SimMachine. A physical
arm needs a risk assessment, reduced dynamics and an operator at the E-stop
first, and will get its own flag when that has been done.
"""

import argparse
import bisect
import csv
import json
import math
import os
import sys
import threading
import time
import xmlrpc.client

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


# --------------------------------------------------------------------------
# trajectory conditioning -- pure, testable
# --------------------------------------------------------------------------

class TrajectoryError(ValueError):
    pass


def load_csv(path):
    """(times, joints) from an exported CSV; joints is a list of 6-lists."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 2:
        raise TrajectoryError("%s: need at least two rows" % path)
    names = ["j%d_deg" % i for i in range(1, 7)]
    missing = [n for n in ["time_s"] + names if n not in rows[0]]
    if missing:
        raise TrajectoryError("%s: missing columns %s" % (path, missing))
    t = [float(r["time_s"]) for r in rows]
    q = [[float(r[n]) for n in names] for r in rows]
    return t, q


def validate(t, q, limits):
    for i, (ti, qi) in enumerate(zip(t, q)):
        if not math.isfinite(ti) or not all(math.isfinite(x) for x in qi):
            raise TrajectoryError("row %d: non-finite value" % (i + 1))
        for j, (a, (lo, hi)) in enumerate(zip(qi, limits)):
            if not lo - 1e-6 <= a <= hi + 1e-6:
                raise TrajectoryError("row %d: J%d = %.3f outside [%g, %g]"
                                      % (i + 1, j + 1, a, lo, hi))
    for i in range(1, len(t)):
        if not t[i] > t[i - 1]:
            raise TrajectoryError("row %d: time_s not strictly increasing" % (i + 1))


def _pchip_slopes(t, y):
    """Fritsch-Carlson monotone slopes, zero at both ends (start/end at rest)
    and zero wherever the data changes direction -- so the curve never
    overshoots a sample."""
    n = len(t)
    h = [t[i + 1] - t[i] for i in range(n - 1)]
    d = [(y[i + 1] - y[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n
    for i in range(1, n - 1):
        if d[i - 1] * d[i] <= 0.0:
            m[i] = 0.0
        else:
            w1 = 2.0 * h[i] + h[i - 1]
            w2 = h[i] + 2.0 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])
    return m


class Path:
    """Per-joint shape-preserving cubic Hermite through the samples."""

    def __init__(self, t, q):
        self.t = [x - t[0] for x in t]
        self.q = q
        self.m = [_pchip_slopes(self.t, [row[j] for row in q]) for j in range(6)]
        self.duration = self.t[-1]

    def at(self, s):
        """Joint angles at time s (seconds from the start)."""
        t = self.t
        if s <= 0.0:
            return list(self.q[0])
        if s >= t[-1]:
            return list(self.q[-1])
        i = bisect.bisect_right(t, s) - 1
        h = t[i + 1] - t[i]
        u = (s - t[i]) / h
        h00 = 2 * u ** 3 - 3 * u ** 2 + 1
        h10 = u ** 3 - 2 * u ** 2 + u
        h01 = -2 * u ** 3 + 3 * u ** 2
        h11 = u ** 3 - u ** 2
        return [h00 * self.q[i][j] + h10 * h * self.m[j][i]
                + h01 * self.q[i + 1][j] + h11 * h * self.m[j][i + 1] for j in range(6)]


def _peaks(samples, dt):
    """Per-joint max |velocity| (list) and overall max |acceleration| over
    equal-interval samples, including from and back to rest at the ends."""
    padded = [samples[0]] + samples + [samples[-1]]
    vmax = [0.0] * 6
    amax = 0.0
    for k in range(1, len(padded)):
        for j in range(6):
            vmax[j] = max(vmax[j], abs(padded[k][j] - padded[k - 1][j]) / dt)
    for k in range(1, len(padded) - 1):
        for j in range(6):
            a = (padded[k + 1][j] - 2 * padded[k][j] + padded[k - 1][j]) / (dt * dt)
            amax = max(amax, abs(a))
    return vmax, amax


def condition(t, q, rate_hz, vel_limit, acc_limit, limits):
    """Validate, interpolate, resample at equal intervals near rate_hz, and
    slow down uniformly until the envelope holds. vel_limit is one number or
    one per joint (FR20: 120 on J1-J3, 180 on J4-J6). Returns (samples, dt,
    report); sample k is at k*dt seconds."""
    vel = list(vel_limit) if isinstance(vel_limit, (list, tuple)) else [float(vel_limit)] * 6
    validate(t, q, limits)
    path = Path(t, q)
    scale = 1.0
    for _ in range(6):
        duration = path.duration * scale
        n = max(1, int(math.ceil(duration * rate_hz)))
        dt = duration / n
        samples = [path.at(k * dt / scale) for k in range(n + 1)]
        vmax, amax = _peaks(samples, dt)
        need = max(max(v / lim for v, lim in zip(vmax, vel)), math.sqrt(amax / acc_limit))
        if need <= 1.0 + 1e-6:
            break
        scale *= need * 1.01
    else:
        raise TrajectoryError("could not fit the envelope after 6 rescales")
    for s in samples:                      # interpolation never leaves limits
        for j, (a, (lo, hi)) in enumerate(zip(s, limits)):
            if not lo - 1e-6 <= a <= hi + 1e-6:
                raise TrajectoryError("resampled J%d = %.3f outside limits" % (j + 1, a))
    report = {"source_rows": len(t), "source_duration_s": round(path.duration, 4),
              "time_scale": round(scale, 4), "played_duration_s": round(n * dt, 4),
              "samples": n + 1, "dt_s": round(dt, 6),
              "peak_vel_deg_s": [round(v, 3) for v in vmax], "peak_acc_deg_s2": round(amax, 3),
              "vel_limit": vel, "acc_limit": acc_limit}
    return samples, dt, report


# --------------------------------------------------------------------------
# controller -- XML-RPC, port 20003
# --------------------------------------------------------------------------

class Controller:
    def __init__(self, ip):
        self.ip = ip
        self.rpc = xmlrpc.client.ServerProxy("http://%s:20003" % ip)

    @staticmethod
    def _ok(ret, what):
        code = ret[0] if isinstance(ret, (list, tuple)) else ret
        if code != 0:
            raise RuntimeError("%s returned error %s" % (what, ret))
        return ret

    def model(self):
        return self._ok(self.rpc.GetSoftwareVersion(), "GetSoftwareVersion")[1]

    def error_code(self):
        return self.rpc.GetRobotErrorCode()

    def joints(self):
        return list(self._ok(self.rpc.GetActualJointPosDegree(1), "GetActualJointPosDegree")[1:7])

    def prepare(self):
        self._ok(self.rpc.Mode(0), "Mode(0) automatic")
        self._ok(self.rpc.RobotEnable(1), "RobotEnable(1)")

    def move_to(self, q, vel_pct):
        desc = self._ok(self.rpc.GetForwardKin([float(x) for x in q]), "GetForwardKin")[1:7]
        self._ok(self.rpc.MoveJ([float(x) for x in q], list(desc), 0, 0, float(vel_pct), 0.0, 100.0,
                                [0.0] * 4, -1.0, 0, [0.0] * 6), "MoveJ")

    def servo_start(self):
        self._ok(self.rpc.ServoMoveStart(), "ServoMoveStart")

    def servo_j(self, q, cmd_t, cmd_id):
        return self.rpc.ServoJ([float(x) for x in q], [0.0] * 4, 0.0, 0.0, float(cmd_t), 0.0, 0.0, int(cmd_id))

    def servo_end(self):
        return self.rpc.ServoMoveEnd()

    def stop(self):
        return self.rpc.StopMotion()


class Feedback(threading.Thread):
    """Actual joints on a separate connection, off the send path."""

    def __init__(self, ip, period_s=0.01):
        super().__init__(daemon=True)
        self.c = Controller(ip)
        self.period = period_s
        self.samples = []            # (perf_counter, [q])
        # not _stop: that name is Thread's own internal method
        self._halt = threading.Event()

    def run(self):
        while not self._halt.is_set():
            t0 = time.perf_counter()
            try:
                self.samples.append((t0, self.c.joints()))
            except Exception:
                pass
            left = self.period - (time.perf_counter() - t0)
            if left > 0:
                time.sleep(left)

    def stop(self):
        self._halt.set()
        self.join(timeout=2.0)


def _wait_until(deadline):
    while True:
        left = deadline - time.perf_counter()
        if left <= 0:
            return
        time.sleep(left - 0.0015 if left > 0.002 else 0)


def play(ctrl, feedback_ip, samples, dt, start_tol_deg=2.0, move_vel_pct=20.0):
    """Move to the first sample, stream the rest. Returns the timing report
    and the per-send log."""
    err = ctrl.error_code()
    if list(err)[:3] != [0, 0, 0]:
        raise RuntimeError("controller reports an error before playback: %s" % (err,))
    ctrl.prepare()
    ctrl.move_to(samples[0], move_vel_pct)
    t_end = time.time() + 30.0
    while True:
        cur = ctrl.joints()
        off = max(abs(a - b) for a, b in zip(cur, samples[0]))
        if off <= start_tol_deg:
            break
        if time.time() > t_end:
            raise RuntimeError("did not reach the first pose: %.2f deg off" % off)
        time.sleep(0.1)

    fb = Feedback(feedback_ip)
    fb.start()
    ctrl.servo_start()
    sends, skipped, late = [], 0, []
    start = time.perf_counter() + 0.05
    last = -1
    n = len(samples)
    try:
        while last < n - 1:
            nxt = last + 1
            _wait_until(start + nxt * dt)
            now = time.perf_counter()
            due = min(n - 1, int((now - start) / dt))      # newest due sample
            k = max(nxt, due)
            skipped += k - nxt
            late.append((now - (start + k * dt)) * 1000.0)
            t0 = time.perf_counter()
            ret = ctrl.servo_j(samples[k], dt, k)
            t1 = time.perf_counter()
            code = ret[0] if isinstance(ret, (list, tuple)) else ret
            if code != 0:
                raise RuntimeError("ServoJ sample %d returned %s" % (k, ret))
            sends.append((t0, k, (t1 - t0) * 1000.0))
            last = k
    except BaseException:
        ctrl.stop()
        raise
    finally:
        ctrl.servo_end()
        time.sleep(0.3)
        fb.stop()

    played = sends[-1][0] - sends[0][0] if len(sends) > 1 else 0.0
    st = sorted(s[2] for s in sends)
    report = {"sends": len(sends), "skipped": skipped,
              "effective_send_hz": round((len(sends) - 1) / played, 2) if played else None,
              "planned_hz": round(1.0 / dt, 2),
              "timeline_drift_s": round(played - (n - 1) * dt, 4),
              "late_over_2ms": sum(1 for x in late if x > 2.0),
              "max_late_ms": round(max(late), 2),
              "send_ms_p50": round(st[len(st) // 2], 2),
              "send_ms_p95": round(st[int(len(st) * 0.95)], 2),
              "send_ms_max": round(st[-1], 2),
              "feedback_samples": len(fb.samples)}
    report.update(tracking(start, samples, dt, fb.samples))
    return report


def tracking(start, samples, dt, feedback):
    """Actual vs commanded joints. The controller follows with some lag, so
    also report the error after the best constant lag (0-400 ms)."""
    if not feedback:
        return {"tracking": "no feedback"}

    def cmd_at(s):
        if s <= 0:
            return samples[0]
        k = s / dt
        i = int(k)
        if i >= len(samples) - 1:
            return samples[-1]
        f = k - i
        return [a + f * (b - a) for a, b in zip(samples[i], samples[i + 1])]

    fb = [(t - start, q) for t, q in feedback if 0 <= t - start <= (len(samples) - 1) * dt]
    if not fb:
        return {"tracking": "no feedback inside the playback window"}

    def err(lag):
        worst, sq = 0.0, 0.0
        for s, q in fb:
            c = cmd_at(s - lag)
            e = max(abs(a - b) for a, b in zip(q, c))
            worst = max(worst, e)
            sq += e * e
        return worst, math.sqrt(sq / len(fb))

    raw = err(0.0)
    best = min(((lag,) + err(lag) for lag in [i * 0.004 for i in range(101)]), key=lambda x: x[2])
    return {"tracking_raw_max_deg": round(raw[0], 4), "tracking_raw_rms_deg": round(raw[1], 4),
            "best_lag_ms": round(best[0] * 1000, 1),
            "tracking_after_lag_max_deg": round(best[1], 4),
            "tracking_after_lag_rms_deg": round(best[2], 4)}


# --------------------------------------------------------------------------
# self-test of the conditioning (no robot)
# --------------------------------------------------------------------------

def self_test():
    failures = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            failures.append(label)

    lim = [(-175, 175), (-265, 85), (-162, 162), (-265, 85), (-175, 175), (-175, 175)]
    t = [i / 24.0 for i in range(25)]
    q = [[30 * math.sin(2 * math.pi * s), -90 + 20 * s, 90, -90, -90 + 10 * math.cos(3 * s), 0] for s in t]
    p = Path(t, q)
    check("path passes through every sample",
          max(abs(a - b) for ti, qi in zip(t, q) for a, b in zip(p.at(ti), qi)) < 1e-9)
    # no overshoot: a step-like joint stays within its sample range
    qs = [[0, 0, 0, 0, 0, 10.0 if i >= 12 else 0.0] for i in range(25)]
    ps = Path(t, qs)
    vals = [ps.at(k / 1000.0)[5] for k in range(1001)]
    check("no overshoot on a step", min(vals) >= -1e-9 and max(vals) <= 10 + 1e-9,
          "range %.4f..%.4f" % (min(vals), max(vals)))
    # at rest = zero slope at both ends. (The first 8 ms step can still move
    # fast on a steep clip -- that is acceleration from rest, which _peaks
    # counts and the envelope scaling below absorbs.)
    eps = 1e-6
    v0 = max(abs(a - b) for a, b in zip(p.at(eps), p.at(0.0))) / eps
    v1 = max(abs(a - b) for a, b in zip(p.at(p.duration), p.at(p.duration - eps))) / eps
    check("starts and ends at rest", v0 < 1e-2 and v1 < 1e-2, "end slopes %.2e, %.2e deg/s" % (v0, v1))
    samples, dt, rep = condition(t, q, 125, 1e9, 1e9, lim)
    check("equal intervals near the rate", abs(1.0 / dt - 125) < 2, "%.2f Hz" % (1.0 / dt))
    samples, dt, rep = condition(t, q, 125, 60.0, 200.0, lim)
    vmax, amax = _peaks(samples, dt)
    check("envelope honoured after uniform scaling", max(vmax) <= 60.0 + 1e-6 and amax <= 200.0 + 1e-6,
          "scale %.3f, peaks %.2f deg/s, %.2f deg/s^2" % (rep["time_scale"], max(vmax), amax))
    # per joint: a J1 limit of 20 binds while the J5 limit of 1000 does not
    per = [20.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
    samples, dt, rep = condition(t, q, 125, per, 1e9, lim)
    vmax, _ = _peaks(samples, dt)
    check("per-joint limits: each joint held to its own",
          all(v <= l + 1e-6 for v, l in zip(vmax, per)) and vmax[0] > 0.95 * per[0],
          "J1 peak %.2f of 20, scale %.3f" % (vmax[0], rep["time_scale"]))
    for label, tt, qq in (("non-increasing time", [0, 0.1, 0.1], [[0] * 6] * 3),
                          ("out of limits", [0, 0.1], [[0] * 6, [0, 90, 0, 0, 0, 0]]),
                          ("non-finite", [0, 0.1], [[0] * 6, [float("nan")] + [0] * 5])):
        try:
            condition(tt, qq, 125, 120, 300, lim)
            check("rejects " + label, False)
        except TrajectoryError as e:
            check("rejects " + label, True, str(e))
    print()
    if failures:
        print("FAILED: %s" % "; ".join(failures))
        return 1
    print("OK: conditioning passes through samples, holds the envelope, rejects bad input")
    return 0


# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--ip", default="192.168.116.128")
    ap.add_argument("--profile", default="fr20")
    ap.add_argument("--rate", type=float, default=125.0, help="ServoJ rate, Hz (cmdT = 1/rate)")
    ap.add_argument("--vel-limit", type=float, default=None,
                    help="deg/s for every joint; default: the profile's per-joint max_velocity_deg_s")
    ap.add_argument("--acc-limit", type=float, default=300.0, help="deg/s^2")
    ap.add_argument("--move-vel", type=float, default=20.0, help="MoveJ speed %% to the first pose")
    ap.add_argument("--sim", action="store_true", help="required: the target is SimMachine")
    ap.add_argument("--dry-run", action="store_true", help="condition and report only; no robot I/O")
    ap.add_argument("--report", help="write the JSON report here")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)

    if a.self_test:
        return self_test()
    if not a.csv:
        ap.error("csv is required")

    import robot_profile
    prof = robot_profile.load(a.profile, os.path.join(os.path.dirname(HERE), "profiles"))
    limits = [tuple(x) for x in prof["robot"]["limits_deg"]]
    vel = [a.vel_limit] * 6 if a.vel_limit else robot_profile.velocity_limits(prof)

    t, q = load_csv(a.csv)
    samples, dt, report = condition(t, q, a.rate, vel, a.acc_limit, limits)
    report = {"csv": os.path.abspath(a.csv), "profile": a.profile, "conditioning": report}
    if not a.dry_run:
        if not a.sim:
            ap.error("refusing to stream without --sim: only SimMachine has been tested")
        ctrl = Controller(a.ip)
        model = ctrl.model()
        report["controller_model"] = model
        want = prof.get("id", a.profile).upper()
        if want not in model.upper():
            raise SystemExit("controller reports %r, profile is %r -- refusing" % (model, want))
        report["playback"] = play(ctrl, a.ip, samples, dt, move_vel_pct=a.move_vel)
    text = json.dumps(report, indent=1)
    print(text)
    if a.report:
        with open(a.report, "w") as f:
            f.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
