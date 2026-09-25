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
    python scripts/fairino_player.py --self-test
    python scripts/fairino_player.py clip.csv --dry-run
    python scripts/fairino_player.py --check --ip IP
    python scripts/fairino_player.py clip.csv --sim --record actual.csv
    python scripts/fairino_player.py clip.csv --hardware --ip IP --goto-start
    python scripts/fairino_player.py --hardware --ip IP --goto-home
    python scripts/fairino_player.py --hardware --ip IP --wiggle 6 5 4 2
    python scripts/fairino_player.py clip.csv --hardware --ip IP --speed 0.3 --record actual.csv

Every move names its target: --sim or --hardware. Speed defaults to 30 % of
the velocity / acceleration envelope (--speed 1.0 plays as designed).
--hardware also defaults to a 10 % MoveJ, prints what it is about to do and
waits for "yes" (skip with --yes). It is no substitute for a
risk assessment, a clear workspace and an operator at the E-stop.
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
    return _snap_uniform(t), q


def _snap_uniform(t):
    """Exact frame times when the CSV's are a fixed rate rounded to 4
    decimals (the asset writes %.4f: 1/24 s -> 0.0417). The rounding
    jitters every interval by up to 0.1 %, and acceleration from second
    differences amplifies that: a factory clip made to play at its own
    speed read x1.027 from its CSV. Uneven times are left alone."""
    n = len(t)
    if n < 3:
        return t
    # least-squares interval through t[0]: the end times are rounded too, so
    # (t[-1] - t[0]) / (n - 1) drifts past the tolerance on a long clip
    dt = sum(i * (t[i] - t[0]) for i in range(n)) / float(sum(i * i for i in range(n)))
    if dt > 0 and all(abs(t[i] - (t[0] + i * dt)) < 1e-4 for i in range(n)):
        fps = round(1.0 / dt)
        if fps and abs(1.0 / dt - fps) < 0.01:          # a whole frame rate: 24, 25, 30 ...
            dt = 1.0 / fps
        return [t[0] + i * dt for i in range(n)]
    return t


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
    """Slopes for the Hermite path, zero at both ends (start/end at rest).

    Each interior slope starts as the three-point (weighted central)
    estimate, which is accurate on smooth motion, then is limited:
      monotone data      clamped to 3x the smaller neighbouring secant
                         (Hyman's filter -- enough for the cubic to stay
                         monotone, so a step-like joint never overshoots)
      a turning point    clamped to the smaller neighbouring secant
      a flat neighbour   zero (a step's shoulder)
    The earlier Fritsch-Carlson slopes (harmonic mean, and zero at every
    turning point) under-read the slope next to a turn -- 24.4 vs a true 32.5
    deg/s on a 24 fps sine -- and the cubic then kinked at the samples: that
    sine read 1635 deg/s^2 against its true 790, and a retimed FR20 clip read
    J6 at 3.6x the acceleration limit at a turn made at 77 deg/s^2."""
    n = len(t)
    h = [t[i + 1] - t[i] for i in range(n - 1)]
    d = [(y[i + 1] - y[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n
    for i in range(1, n - 1):
        prod = d[i - 1] * d[i]
        if prod == 0.0:
            continue
        s_ = (d[i - 1] * h[i] + d[i] * h[i - 1]) / (h[i - 1] + h[i])
        small = min(abs(d[i - 1]), abs(d[i]))
        cap = 3.0 * small if prod > 0.0 else small
        m[i] = max(-cap, min(cap, s_))
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
    """Per-joint max |velocity| and max |acceleration| (lists) over
    equal-interval samples, including from and back to rest at the ends."""
    padded = [samples[0]] + samples + [samples[-1]]
    vmax = [0.0] * 6
    amax = [0.0] * 6
    for k in range(1, len(padded)):
        for j in range(6):
            vmax[j] = max(vmax[j], abs(padded[k][j] - padded[k - 1][j]) / dt)
    for k in range(1, len(padded) - 1):
        for j in range(6):
            a = (padded[k + 1][j] - 2 * padded[k][j] + padded[k - 1][j]) / (dt * dt)
            amax[j] = max(amax[j], abs(a))
    return vmax, amax


def _per_joint(x):
    return list(x) if isinstance(x, (list, tuple)) else [float(x)] * 6


def limiting(t, q, rate_hz, vel_limit, acc_limit, limits=None):
    """How much condition() will stretch this clip, and where it binds:
    {"scale_needed": time stretch (1.0 = plays at its own speed),
     "kind": "velocity"|"acceleration", "joint": 1-6,
     "time_s": source time of the worst sample, "ratio": fraction of the
     limit there at the clip's own speed}.
    It runs condition() itself, so Houdini's Pre-Flight and the player cannot
    disagree -- a one-pass estimate at the clip's own timing under-read the
    stretch (8.0 vs 9.5): resampled 5 points per 24 fps frame, the peaks of
    the interpolated curve do not show until the time is stretched."""
    vel, acc = _per_joint(vel_limit), _per_joint(acc_limit)
    limits = limits or [(-1e9, 1e9)] * 6
    samples, dt, rep = condition(t, q, rate_hz, vel, acc, limits)
    scale = rep["time_scale"]
    padded = [samples[0]] + samples + [samples[-1]]
    worst = {"kind": None, "joint": 0, "k": 0, "need": 0.0, "ratio": 0.0}
    for k in range(1, len(padded)):
        for j in range(6):
            r = abs(padded[k][j] - padded[k - 1][j]) / dt / vel[j] * scale      # at own speed
            if r > worst["need"]:
                worst.update(kind="velocity", joint=j + 1, k=k - 1, need=r, ratio=r)
            if k < len(padded) - 1:
                a = abs(padded[k + 1][j] - 2 * padded[k][j] + padded[k - 1][j]) / (dt * dt) / acc[j] * scale * scale
                if math.sqrt(a) > worst["need"]:
                    worst.update(kind="acceleration", joint=j + 1, k=k - 1, need=math.sqrt(a), ratio=a)
    return {"scale_needed": round(scale, 4), "kind": worst["kind"], "joint": worst["joint"],
            "time_s": round(worst["k"] * dt / scale, 4), "ratio": round(worst["ratio"], 4)}


def need_profile(t, q, rate_hz, vel_limit, acc_limit):
    """[(time_s, need)] along the clip at its own speed, one per resampled
    point: need = max over joints of v/vlim and sqrt(a/alim), so need > 1 is
    how much slower that moment has to be. Same interpolation and resampling
    as condition(), so Retime can slow exactly the moments the player would
    slow the whole clip for."""
    vel, acc = _per_joint(vel_limit), _per_joint(acc_limit)
    path = Path(t, q)
    n = max(1, int(math.ceil(path.duration * rate_hz)))
    dt = path.duration / n
    s = [path.at(k * dt) for k in range(n + 1)]
    pad = [s[0]] + s + [s[-1]]
    out = []
    for k in range(1, len(pad) - 1):
        need = 0.0
        for j in range(6):
            need = max(need, abs(pad[k + 1][j] - pad[k][j]) / dt / vel[j],
                       math.sqrt(abs(pad[k + 1][j] - 2 * pad[k][j] + pad[k - 1][j]) / (dt * dt) / acc[j]))
        out.append(((k - 1) * dt, need))
    return out


def condition(t, q, rate_hz, vel_limit, acc_limit, limits):
    """Validate, interpolate, resample at equal intervals near rate_hz, and
    slow down uniformly until the envelope holds. vel_limit is one number or
    one per joint (FR20: 120 on J1-J3, 180 on J4-J6). Returns (samples, dt,
    report); sample k is at k*dt seconds."""
    vel = _per_joint(vel_limit)
    acc = _per_joint(acc_limit)
    validate(t, q, limits)
    path = Path(t, q)
    scale = 1.0
    for _ in range(6):
        duration = path.duration * scale
        n = max(1, int(math.ceil(duration * rate_hz)))
        dt = duration / n
        samples = [path.at(k * dt / scale) for k in range(n + 1)]
        vmax, amax = _peaks(samples, dt)
        need = max(max(v / lim for v, lim in zip(vmax, vel)),
                   math.sqrt(max(a / lim for a, lim in zip(amax, acc))))
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
              "peak_vel_deg_s": [round(v, 3) for v in vmax],
              "peak_acc_deg_s2": [round(a, 3) for a in amax],
              "vel_limit": vel, "acc_limit": acc}
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


def _require_no_error(ctrl, when):
    err = ctrl.error_code()
    if list(err)[:3] != [0, 0, 0]:
        raise RuntimeError("controller reports an error %s: %s" % (when, err))


def goto(ctrl, q, move_vel_pct, tol_deg=2.0, timeout_s=60.0):
    """Controller-planned MoveJ to q, then wait until the arm is within
    tol_deg of it on every joint."""
    _require_no_error(ctrl, "before moving")
    ctrl.prepare()
    ctrl.move_to(q, move_vel_pct)
    t_end = time.time() + timeout_s
    while True:
        off = max(abs(a - b) for a, b in zip(ctrl.joints(), q))
        if off <= tol_deg:
            return off
        if time.time() > t_end:
            raise RuntimeError("did not reach the pose: %.2f deg off" % off)
        time.sleep(0.1)


def play(ctrl, feedback_ip, samples, dt, start_tol_deg=2.0, move_vel_pct=20.0):
    """Move to the first sample, stream the rest. Returns (report, start,
    feedback) -- start is the playback clock's zero, feedback the actual
    joints as (perf_counter, q) for record_aligned()."""
    goto(ctrl, samples[0], move_vel_pct, start_tol_deg)

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
    report["controller_error_after"] = list(ctrl.error_code())
    return report, start, fb.samples


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
# test clips, recording, read-only check
# --------------------------------------------------------------------------

def home_path_check(q_from, q_home, env_path, robot="fr20", steps=60):
    """The MoveJ to HOME, as the controller moves: every joint interpolated
    together (joint space), sampled and checked against the cell
    (collision.py). Returns (ok, one line). No env file: (True, "not checked")."""
    if not env_path or not os.path.exists(env_path):
        return True, "no cell file; path not checked"
    import collision
    model = collision.load_model(robot)
    env = collision.load_env(env_path)
    qs = [[a + (b - a) * k / float(steps) for a, b in zip(q_from, q_home)] for k in range(steps + 1)]
    ts = [k * 0.1 for k in range(steps + 1)]                  # timing does not matter here:
    zones = dict(env, objects=[o for o in env.get("objects", []) if o["role"] in ("obstacle", "keep_out")])
    rep = collision.check(model, zones, ts, qs)               # obstacles and keep-out only
    return rep["ok"], collision.describe(rep)


def wiggle_clip(q0, joint, amp_deg, period_s, cycles, limits, rate_hz=50.0):
    """One joint (1-based) goes q0 -> q0 + amp -> q0 each period, at rest at
    every return (raised cosine); every other joint holds q0. For a first
    ServoJ test on hardware: small, slow, one joint, starting where the arm
    already is."""
    n = int(round(period_s * cycles * rate_hz))
    t = [k / rate_hz for k in range(n + 1)]
    q = []
    for tk in t:
        row = list(q0)
        row[joint - 1] = q0[joint - 1] + amp_deg * (1.0 - math.cos(2.0 * math.pi * tk / period_s)) / 2.0
        q.append(row)
    validate(t, q, limits)
    return t, q


def record_aligned(src_t, time_scale, start, feedback):
    """Actual joints at each SOURCE row's time, in the export format, so the
    asset's Import CSV keys it frame-for-frame against the clip that was
    designed. Source time ts was commanded at playback time ts * time_scale;
    the actual joints there are interpolated from feedback. The controller's
    lag stays in -- that is what is being looked at."""
    fb = [(t - start, q) for t, q in feedback]
    if not fb:
        raise RuntimeError("no feedback to record")
    times = [x[0] for x in fb]
    rows = []
    t0 = src_t[0]
    for i, ts in enumerate(src_t):
        s_ = (ts - t0) * time_scale
        k = bisect.bisect_left(times, s_)
        if k <= 0:
            q = fb[0][1]
        elif k >= len(fb):
            q = fb[-1][1]
        else:
            (ta, qa), (tb, qb) = fb[k - 1], fb[k]
            f = (s_ - ta) / (tb - ta) if tb > ta else 0.0
            q = [a + f * (b - a) for a, b in zip(qa, qb)]
        rows.append([i + 1, ts - t0] + list(q))
    return rows


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "time_s"] + ["j%d_deg" % i for i in range(1, 7)])
        for r in rows:
            w.writerow([r[0], "%.4f" % r[1]] + ["%.6f" % x for x in r[2:]])


def _rot(axis, deg):
    c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {"x": ((1, 0, 0), (0, c, -s_), (0, s_, c)),
            "y": ((c, 0, s_), (0, 1, 0), (-s_, 0, c)),
            "z": ((c, -s_, 0), (s_, c, 0), (0, 0, 1))}[axis]


def check(ctrl, prof):
    """Read-only: which controller, is it in error, where is the arm, and does
    its forward kinematics agree with the profile's URDF. Nothing moves."""
    import urdf_rig as U
    root = os.path.dirname(HERE)
    rep = {"software_version": ctrl.rpc.GetSoftwareVersion(),
           "error_code": list(ctrl.error_code()),
           "current_joints_deg": [round(x, 3) for x in ctrl.joints()],
           "tcp_offset": list(ctrl.rpc.GetTCPOffset(0))}
    urdf = prof["rig"].get("urdf")
    if urdf:
        chain = U.parse_urdf(os.path.join(root, urdf))["chain"]
        flange = float(prof["rig"].get("flange_offset_m", 0.0))
        poses = [rep["current_joints_deg"], [0.0] * 6, [30, -90, 90, -90, -90, 0],
                 [-45, -60, 110, -30, 60, 45], [100, -120, 60, -150, 40, -80]]
        worst_p = worst_r = 0.0
        for q in poses:
            r = ctrl.rpc.GetForwardKin([float(x) for x in q])
            if r[0] != 0:
                continue
            x, y, z, rx, ry, rz = r[1:7]
            fk = U.forward_kinematics(chain, q)
            ours = U.flange_point(fk, flange)
            worst_p = max(worst_p, math.dist([v * 1000 for v in ours], [x, y, z]))
            R = U._mat_mul(_rot("z", rz), U._mat_mul(_rot("y", ry), _rot("x", rx)))
            Rq = fk[-1]["link_R"]
            M = [[sum(Rq[k][i] * R[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
            c = (M[0][0] + M[1][1] + M[2][2] - 1) / 2
            sn = 0.5 * math.sqrt((M[2][1] - M[1][2]) ** 2 + (M[0][2] - M[2][0]) ** 2
                                 + (M[1][0] - M[0][1]) ** 2)
            worst_r = max(worst_r, math.degrees(math.atan2(sn, c)))
        rep["fk_vs_urdf"] = {"poses": len(poses), "max_position_mm": round(worst_p, 4),
                             "max_orientation_deg": round(worst_r, 5),
                             "verdict": "match" if worst_p < 0.1 and worst_r < 0.01
                             else "MISMATCH -- do not play"}
    return rep


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
    t30 = [i / 24.0 for i in range(30)]
    rounded = [float("%.4f" % x) for x in t30]           # 1.2083 at the end: v40_line's case
    snapped = _snap_uniform(rounded)
    check("24 fps times written with 4 decimals are read back exact",
          max(abs(a - b) for a, b in zip(snapped, t30)) < 1e-12, "0.0417 -> %.6f" % snapped[1])
    envp = os.path.join(os.path.dirname(HERE), "envs", "volvox_lab.json")
    if os.path.exists(envp):
        ok_a, line_a = home_path_check([-10.6, -78.1, 97.3, -12.2, 83.3, -0.8], [0.0, -90.0, 90.0, -90.0, -90.0, 0.0], envp)
        ok_b, line_b = home_path_check([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, -90.0, 90.0, -90.0, -90.0, 0.0], envp)
        check("path to HOME: clear from the test clip's start, refused from all-zero (arm flat on the plate)",
              ok_a and not ok_b, "%s / %s" % (line_a, line_b))
    uneven = [0.0, 0.05, 0.08, 0.2]
    check("uneven times are left as they are", _snap_uniform(uneven) == uneven)
    # a smooth motion that turns around must not read as a jolt: 24 fps
    # samples of 20 sin(2 pi t) have a true peak acceleration of 20 (2 pi)^2
    tt = [i / 24.0 for i in range(49)]
    qq = [[20 * math.sin(2 * math.pi * x), 0, 0, 0, 0, 0] for x in tt]
    ps = Path(tt, qq)
    dtt = 1 / 500.0
    xs = [ps.at(k * dtt)[0] for k in range(int(2.0 / dtt) + 1)]
    inner = xs[int(0.3 / dtt):int(1.7 / dtt)]          # away from the rest-at-ends
    apk = max(abs(inner[k + 1] - 2 * inner[k] + inner[k - 1]) / dtt ** 2 for k in range(1, len(inner) - 1))
    true = 20 * (2 * math.pi) ** 2
    check("a smooth turn-around reads near its true acceleration (not a jolt)",
          apk < 1.3 * true, "peak %.0f vs true %.0f deg/s^2" % (apk, true))
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
    check("envelope honoured after uniform scaling", max(vmax) <= 60.0 + 1e-6 and max(amax) <= 200.0 + 1e-6,
          "scale %.3f, peaks %.2f deg/s, %.2f deg/s^2" % (rep["time_scale"], max(vmax), max(amax)))
    lim_rep = limiting(t, q, 125, 60.0, 200.0, lim)
    check("limiting() reports the stretch condition() applies, and where",
          lim_rep["scale_needed"] == round(rep["time_scale"], 4) and lim_rep["kind"] == "acceleration",
          "%s J%d at %.2f s, x%.3f (condition x%.3f)"
          % (lim_rep["kind"], lim_rep["joint"], lim_rep["time_s"], lim_rep["scale_needed"], rep["time_scale"]))
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
    q0 = [10, -80, 70, -80, -90, 5]
    tw, qw = wiggle_clip(q0, 6, 5.0, 4.0, 2, lim)
    others = max(abs(r[j] - q0[j]) for r in qw for j in range(5))
    check("wiggle: one joint, q0 -> q0+amp -> q0",
          qw[0] == q0 and max(abs(a - b) for a, b in zip(qw[-1], q0)) < 1e-9
          and abs(max(r[5] for r in qw) - (q0[5] + 5.0)) < 1e-6 and others == 0.0,
          "%d rows, %.1f s" % (len(qw), tw[-1]))
    src_t = [i / 24.0 for i in range(10)]
    scale, start = 2.0, 100.0
    fb = [(start + k * 0.01, [k * 0.01 * 3.0] * 6) for k in range(200)]
    rows = record_aligned(src_t, scale, start, fb)
    err = max(abs(r[2] - src_t[i] * scale * 3.0) for i, r in enumerate(rows))
    check("record_aligned: one row per source row, at source time",
          len(rows) == len(src_t) and [r[1] for r in rows] == src_t and err < 1e-9,
          "max error %.2e deg" % err)
    print()
    if failures:
        print("FAILED: %s" % "; ".join(failures))
        return 1
    print("OK: conditioning passes through samples, holds the envelope, rejects bad input")
    return 0


# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="joint CSV exported from the asset")
    ap.add_argument("--ip", default="192.168.116.128")
    ap.add_argument("--profile", default="fr20")
    tgt = ap.add_mutually_exclusive_group()
    tgt.add_argument("--sim", action="store_true", help="the target is SimMachine")
    tgt.add_argument("--hardware", action="store_true",
                     help="the target is a physical arm: reduced speed by default, asks to confirm")
    ap.add_argument("--speed", type=float, default=None,
                    help="fraction of the velocity / acceleration envelope; default 0.3 (1.0 = as designed in Houdini)")
    ap.add_argument("--rate", type=float, default=125.0, help="ServoJ rate, Hz (cmdT = 1/rate)")
    ap.add_argument("--vel-limit", type=float, default=None,
                    help="deg/s for every joint; default: the profile's per-joint max_velocity_deg_s")
    ap.add_argument("--acc-limit", type=float, default=None,
                    help="deg/s^2 for every joint, before --speed; default: the profile's max_acceleration_deg_s2")
    ap.add_argument("--move-vel", type=float, default=None,
                    help="MoveJ speed %% to the first pose; default 20 sim, 10 hardware")
    ap.add_argument("--check", action="store_true", help="read-only: identity, errors, pose, FK vs URDF")
    ap.add_argument("--goto-start", action="store_true", help="only MoveJ to the clip's first pose")
    ap.add_argument("--goto-home", action="store_true",
                    help="only MoveJ to the profile's HOME pose (robot.home_deg), path checked against --env first")
    ap.add_argument("--env", default=os.path.join(os.path.dirname(HERE), "envs", "volvox_lab.json"),
                    help="cell file for --goto-home's path check ('' to skip)")
    ap.add_argument("--wiggle", nargs=4, metavar=("JOINT", "AMP_DEG", "PERIOD_S", "CYCLES"),
                    help="play a generated one-joint swing from the current pose instead of a CSV")
    ap.add_argument("--record", help="write the actual joints, aligned to the clip's rows, as a CSV")
    ap.add_argument("--dry-run", action="store_true", help="condition and report only; no robot I/O")
    ap.add_argument("--yes", action="store_true", help="skip the hardware confirmation prompt")
    ap.add_argument("--report", help="write the JSON report here")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)

    if a.self_test:
        return self_test()

    import robot_profile
    prof = robot_profile.load(a.profile, os.path.join(os.path.dirname(HERE), "profiles"))
    limits = [tuple(x) for x in prof["robot"]["limits_deg"]]
    target = "hardware" if a.hardware else "sim" if a.sim else None
    report = {"profile": a.profile, "target": target}

    def done():
        text = json.dumps(report, indent=1)
        print(text)
        if a.report:
            with open(a.report, "w") as f:
                f.write(text)
        return 0

    ctrl = None
    if not a.dry_run:
        ctrl = Controller(a.ip)
        model = ctrl.model()
        report["controller_model"] = model
        if a.profile.upper() not in model.upper():
            raise SystemExit("controller reports %r, profile is %r -- refusing" % (model, a.profile))

    if a.check:
        report["check"] = check(ctrl, prof)
        return done()

    if a.goto_home:
        home = robot_profile.home(prof)
        if home is None:
            ap.error("profile %s has no robot.home_deg" % a.profile)
        if target is None or ctrl is None:
            ap.error("--goto-home moves the arm: say --sim or --hardware (not --dry-run)")
        cur = ctrl.joints()
        ok, line = home_path_check(cur, home, a.env, a.profile)
        report["home"] = home
        report["home_path"] = line
        if not ok:
            report["aborted"] = "path to HOME: " + line
            print("REFUSED: the MoveJ to HOME would " + line)
            return done()
        move_vel = a.move_vel if a.move_vel is not None else (10.0 if a.hardware else 20.0)
        if a.hardware and not a.yes:
            print("HARDWARE  %s at %s" % (report["controller_model"], a.ip))
            print("  now       %s" % [round(x, 1) for x in cur])
            print("  HOME      %s  (MoveJ at %g %%, largest joint move %.1f deg)"
                  % ([round(x, 1) for x in home], move_vel, max(abs(x - y) for x, y in zip(cur, home))))
            print("  path      %s" % line)
            if input("Clear workspace, hand on the E-stop. Type yes to move: ").strip().lower() != "yes":
                report["aborted"] = "not confirmed"
                return done()
        report["goto_home_off_deg"] = round(goto(ctrl, home, move_vel), 3)
        return done()

    if a.wiggle:
        if ctrl is None:
            ap.error("--wiggle needs the robot (it starts from the current pose)")
        j, amp, per, cyc = int(a.wiggle[0]), float(a.wiggle[1]), float(a.wiggle[2]), int(a.wiggle[3])
        t, q = wiggle_clip(ctrl.joints(), j, amp, per, cyc, limits)
        report["clip"] = "wiggle J%d %+g deg, %g s x %d" % (j, amp, per, cyc)
    elif a.csv:
        t, q = load_csv(a.csv)
        report["clip"] = os.path.abspath(a.csv)
    else:
        ap.error("give a CSV, --wiggle, or --check")

    speed = a.speed if a.speed is not None else 0.3
    if not 0.0 < speed <= 1.0:
        ap.error("--speed must be in (0, 1]")
    vel = [a.vel_limit] * 6 if a.vel_limit else robot_profile.velocity_limits(prof)
    vel = [v * speed for v in vel]
    acc = [a.acc_limit] * 6 if a.acc_limit else (robot_profile.acceleration_limits(prof) or [300.0] * 6)
    acc = [x * speed for x in acc]
    samples, dt, cond = condition(t, q, a.rate, vel, acc, limits)
    if cond["time_scale"] > 1.0:
        cond["limited_by"] = limiting(t, q, a.rate, vel, acc, limits)
    cond["speed"] = speed
    report["conditioning"] = cond
    if a.dry_run:
        return done()
    if target is None:
        ap.error("say where this goes: --sim or --hardware")

    move_vel = a.move_vel if a.move_vel is not None else (10.0 if a.hardware else 20.0)
    if a.hardware and not a.yes:
        cur = ctrl.joints()
        print("HARDWARE  %s at %s" % (report["controller_model"], a.ip))
        print("  clip      %s" % report["clip"])
        print("  speed     %.0f %% of the envelope -> plays %.1f s (scale %.2f)"
              % (speed * 100, cond["played_duration_s"], cond["time_scale"]))
        print("  now       %s" % [round(x, 1) for x in cur])
        print("  start     %s  (MoveJ at %g %%, largest joint move %.1f deg)"
              % ([round(x, 1) for x in samples[0]], move_vel,
                 max(abs(x - y) for x, y in zip(cur, samples[0]))))
        if input("Clear workspace, hand on the E-stop. Type yes to move: ").strip().lower() != "yes":
            report["aborted"] = "not confirmed"
            return done()

    if a.goto_start:
        report["goto_start_off_deg"] = round(goto(ctrl, samples[0], move_vel), 3)
        return done()

    pb, start_t, feedback = play(ctrl, a.ip, samples, dt, move_vel_pct=move_vel)
    report["playback"] = pb
    if a.wiggle and feedback:
        # A J6 wiggle turns the flange about its own axis: with no tool on it,
        # 5 deg is next to invisible. Say what the joint actually did.
        vals = [fq[j - 1] for _, fq in feedback]
        report["wiggle_actual"] = {"joint": j, "commanded_deg": amp,
                                   "actual_travel_deg": round(max(vals) - min(vals), 3)}
    if a.record:
        write_csv(a.record, record_aligned(t, cond["time_scale"], start_t, feedback))
        report["recorded"] = os.path.abspath(a.record)
    return done()


if __name__ == "__main__":
    sys.exit(main())
