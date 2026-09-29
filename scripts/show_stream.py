"""Stream the show to a Fairino arm: one continuous ServoJ stream driven by
the show's state machine (show.Runner), instead of one CSV per run.

    python scripts/show_stream.py --self-test
    python scripts/show_stream.py shows/party.json --sim --minutes 5
    python scripts/show_stream.py shows/party.json --sim --minutes 30 --osc      # TouchDesigner drives it
    python scripts/show_stream.py shows/party.json --hardware --ip IP --speed 0.3 --minutes 10

Every segment of the compiled show (shows/*.compiled.json, from `show.py
build`) was checked in the room at build time and meets the next at rest,
so the stream is the Runner's joints, tick by tick:

  1. start: MoveJ to the start hub through safe_move's checked route
     (fairino_player.move_checked; on hardware it asks first);
  2. stream ServoJ at 125 Hz on an absolute clock -- when late, the Runner
     is advanced by the ticks missed and only the newest pose is sent (the
     skip is counted), never a burst;
  3. every tick, before sending: the joint step is within the joints'
     velocity limits (times the speed scale) and the pose within the joint
     limits -- otherwise FAULT;
  4. a feedback thread on its own connection reads the actual joints and
     polls the controller's error code; an error is a FAULT;
  5. FAULT, Ctrl+C or OSC /robot/stop: StopMotion, ServoMoveEnd, report --
     a software stop, no substitute for the E-stop;
  6. the end (--minutes): the Runner pauses, the running clip finishes at a
     hub at rest, then the stream ends.

--speed scales the show's clock: speed s runs every segment s times as fast
(velocity x s, acceleration x s^2). Hardware defaults to 0.3.

The report (--log DIR): sends, skips, lateness, the largest step per tick,
tracking (actual vs commanded, after the best constant lag), the segments
played, the events; the commanded and actual joints as CSV.
"""

import argparse
import csv
import gc
import json
import math
import os
import queue
import sys
import threading
import time
from array import array

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import fairino_player as P  # noqa: E402

RATE_HZ = 125.0
STEP_MARGIN = 1.1                  # a tick may step 10 % past the velocity limit x speed (rounding, sampling)
ERROR_POLL_S = 0.1
SLOW_READ_S = 0.02                 # a joint read slower than this has no trustworthy time: left out of tracking
TRACKING_MAX_SAMPLES = 60000       # tracking of a long run is computed on evenly spaced feedback samples
MAX_PPM = 3000.0                   # a controller clock calibration further off than this is refused
MAX_BURST = 4                      # points sent back to back after a late tick; more are skipped
QUEUE_POLL_S = 1.0                 # the controller's motion queue, read once a second for the report (a diagnostic)
OPERATOR_STOPS = ("stop requested", "stopped by the operator (Ctrl+C)")    # a stop, not a fault
LAG_WINDOW_S = 180.0               # the lag is also measured per window: a growing lag is a clock drift
OSC_EVERY = 4                      # status out every 4 ticks (~31 Hz)
MIN_SPEED = 0.05                   # the speed is set before the stream starts; it is not changed while it runs
HARDWARE_SPEED = 0.3


class StreamFault(RuntimeError):
    pass


class Commands:
    """What OSC (another thread) asks of the Runner, applied by the stream
    loop between ticks -- the Runner is not thread-safe. Looks like a Runner
    to show.OscBridge."""

    def __init__(self, runner, speed=1.0):
        self.runner, self.q = runner, queue.Queue()
        self.sel = runner.sel                      # mood / energy: plain attribute sets
        self.stop_requested = threading.Event()
        self.speed_now = speed                     # what the stream plays at (fixed; reported over OSC)
        self.skipped = 0                           # ticks skipped so far (reported over OSC: a stall shows at once)

    def trigger(self, name="scan"):
        self.q.put(("trigger", name))

    def pause(self):
        self.q.put(("pause",))

    def resume(self):
        self.q.put(("resume",))

    def reset(self):
        self.q.put(("reset",))

    def stop(self):
        self.stop_requested.set()

    def status(self):
        """The Runner's status, time left in wall seconds (at this speed)."""
        s = self.runner.status()
        s["time_left"] = round(s["time_left"] / max(self.speed_now, 1e-6), 2)
        return s

    def apply(self):
        while True:
            try:
                cmd = self.q.get_nowait()
            except queue.Empty:
                return
            getattr(self.runner, cmd[0])(*cmd[1:])


class Rows:
    """(key, 6 joints) rows in a flat array('d'). A long show logs millions of
    rows: as Python lists they would be objects the garbage collector walks
    on every full collection, and its pauses grow with them (0.1-0.2 s after
    30 min, seen on SimMachine 2026-09-27). An array is one object."""

    def __init__(self):
        self.a = array("d")

    def add(self, key, q):
        self.a.append(key)
        self.a.extend(q)

    def __len__(self):
        return len(self.a) // 7

    def row(self, i):
        i = (i % len(self)) * 7
        return self.a[i], list(self.a[i + 1:i + 7])

    def __iter__(self):
        a = self.a
        for i in range(0, len(a), 7):
            yield a[i], list(a[i + 1:i + 7])


class Link(threading.Thread):
    """The actual joints, on their own connection, and every ERROR_POLL_S
    the controller's error code: the first non-zero one is kept in .error.
    A read is stamped at the middle of its round trip; one slower than
    SLOW_READ_S is not kept (when it was taken is not known well enough)."""

    def __init__(self, ip=None, period_s=0.01, ctrl=None):
        super().__init__(daemon=True)
        self.c = ctrl or P.Controller(ip)
        self.period = period_s
        self.rows = Rows()
        self.slow_reads = 0
        self.error = None
        self._last_poll = 0.0
        self._halt = threading.Event()
        self.queue_hist = array("i")                  # the controller's motion queue, every QUEUE_POLL_S
        self._last_queue = 0.0
        self._queue_ok = hasattr(self.c, "queue_length")

    @property
    def samples(self):
        return [(t, q) for t, q in self.rows]

    def stop(self):
        self._halt.set()
        self.join(timeout=2.0)

    def run(self):
        while not self._halt.is_set():
            t0 = time.perf_counter()
            try:
                q = self.c.joints()
                t1 = time.perf_counter()
                if t1 - t0 <= SLOW_READ_S:
                    self.rows.add((t0 + t1) / 2.0, q)
                else:
                    self.slow_reads += 1
                if self._queue_ok and t0 - self._last_queue >= QUEUE_POLL_S:
                    self._last_queue = t0
                    n = self.c.queue_length()
                    if n is None:
                        self._queue_ok = False            # this controller does not say
                    else:
                        self.queue_hist.append(n)
                if t0 - self._last_poll >= ERROR_POLL_S:
                    self._last_poll = t0
                    err = list(self.c.error_code())
                    if err[:3] != [0, 0, 0] and self.error is None:
                        self.error = err
            except Exception as e:                  # a lost link is a fault too
                if self.error is None and not self._halt.is_set():
                    self.error = ["feedback", str(e)[:120]]
            left = self.period - (time.perf_counter() - t0)
            if left > 0:
                time.sleep(left)


class Guard:
    """Per-tick checks before a pose is sent."""

    def __init__(self, vel_limits, limits_deg, dt, speed):
        self.vel, self.limits, self.dt, self.speed = vel_limits, limits_deg, dt, speed
        self.worst_ratio = 0.0

    def check(self, prev, q, ticks):
        for j, (a, b, v) in enumerate(zip(prev, q, self.vel)):
            allowed = v * self.dt * self.speed * ticks * STEP_MARGIN
            step = abs(b - a)
            self.worst_ratio = max(self.worst_ratio, step / (v * self.dt * self.speed * ticks))
            if step > allowed:
                raise StreamFault("J%d steps %.3f deg in %d tick(s), allowed %.3f" % (j + 1, step, ticks, allowed))
        for j, (x, (lo, hi)) in enumerate(zip(q, self.limits)):
            if not lo <= x <= hi:
                raise StreamFault("J%d at %.2f deg, outside its limits %g..%g" % (j + 1, x, lo, hi))


ASK = "[ask] "                       # a question to a window that started this process (show_ui)


def confirm(text, stdin=None, stdout=None):
    """Ask before moving the real arm. At a terminal: type yes. Started by a
    window (stdin a pipe): one line `[ask] <text as JSON>` out, one answer
    line in -- the window shows the text and says yes or no. No answer (end
    of input) is a no."""
    stdin, stdout = stdin or sys.stdin, stdout or sys.stdout
    if stdin.isatty():
        return input(text + "\nType yes to go on: ").strip().lower() == "yes"
    stdout.write(ASK + json.dumps(text) + "\n")
    stdout.flush()
    return stdin.readline().strip().lower() == "yes"


def realtime_priority():
    """Windows: this process at HIGH priority and the calling (stream) thread
    at TIME_CRITICAL, so other programs on the PC -- Houdini, TouchDesigner,
    a SimMachine VM -- delay a tick less often. Only this process changes.
    Returns what was set, or why not."""
    if os.name != "nt":
        return "not Windows: left as is"
    import ctypes
    from ctypes import wintypes
    k32 = ctypes.windll.kernel32
    # the pseudo-handles are -1 / -2: typed as HANDLE, or ctypes cuts them to 32 bits
    k32.GetCurrentProcess.restype = k32.GetCurrentThread.restype = wintypes.HANDLE
    k32.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    k32.SetThreadPriority.argtypes = [wintypes.HANDLE, ctypes.c_int]
    ok_p = k32.SetPriorityClass(k32.GetCurrentProcess(), 0x00000080)          # HIGH_PRIORITY_CLASS
    ok_t = k32.SetThreadPriority(k32.GetCurrentThread(), 15)                  # THREAD_PRIORITY_TIME_CRITICAL
    return "process high, stream thread time-critical" if ok_p and ok_t else "not set (%s, %s)" % (ok_p, ok_t)


def stream(ctrl, link, runner, commands, guard, dt, speed, minutes, osc=None, log=print, analyse=True,
           clock_ppm=None):
    """The stream loop. The arm must already be at the Runner's start pose.
    Returns (report, [(tick, q) sent], the clock's zero); a fault is in the
    report, not raised."""
    ticks_cmd = Rows()                              # (tick, q) sent
    sends_ms, late_ms, skipped = [], [], 0
    events = []
    runner.log = lambda text: events.append(text)
    prev = runner.step(0.0)

    end_at = minutes * 60.0
    ending, fault = False, None
    link.start()
    gc.collect()
    gc.freeze()                                     # what exists now (the graph, the show) is never walked again
    events.append("priority: " + realtime_priority())
    # Who paces the show: the controller's clock. ServoJ points are played
    # one per cmdT on the controller's own clock, and the real FR20's clock
    # runs ~950 ppm slower than this PC's (2026-09-28): points sent on the
    # PC's clock piled up in a buffer behind its motion queue (whose length,
    # GetMotionQueueLength, stayed put while the lag grew ~1 ms per s). So
    # with the controller's clock rate known (clock_ppm: measured by
    # controller_clock.py outside servo mode -- GetSystemClock takes ~300 ms
    # in it -- and kept per controller in playback.toml), each point's
    # deadline is the last one's plus dt at that rate, the way a disciplined
    # clock (NTP, PTP) is steered by its measured rate. A late tick sends the
    # points it owes back to back -- the controller buffers them (the FR20
    # took such pairs without error) -- up to MAX_BURST; more are skipped.
    # Without a calibration the PC's clock paces, a late tick is skipped.
    clocked = clock_ppm is not None
    if clocked and abs(clock_ppm) > MAX_PPM:
        raise ValueError("clock_ppm %.0f: beyond %.0f, not believed -- measure again" % (clock_ppm, MAX_PPM))
    events.append("pacing: " + ("the controller's clock, %+.0f ppm against this PC's (calibrated)" % clock_ppm
                                if clocked else "this PC's clock (no clock calibration for this controller)"))
    bursts, pid, tick = 0, 0, -1
    step = point_step(dt, clock_ppm)                # this PC's seconds per point
    ctrl.servo_start()
    start = time.perf_counter() + 0.05
    next_t = start
    try:
        while True:
            P._wait_until(next_t)
            now = time.perf_counter()
            owed = 1 + max(0, int((now - next_t) / step))    # points whose time has come
            late_ms.append((now - next_t) * 1000.0)
            if commands.stop_requested.is_set():
                fault = "stop requested"
                break
            if link.error is not None:
                raise StreamFault("controller error %s" % (link.error,))
            commands.apply()
            if not ending and now - start >= end_at:
                ending = True
                runner.pause()
                events.append("%.2f end of the run: finishing the clip at a hub" % runner.clock)
            n, adv = 1, 1
            if owed > 1:
                if clocked:                           # the controller buffers: send what is owed
                    n = min(owed, MAX_BURST)
                    adv = owed - n + 1                # beyond MAX_BURST the oldest are skipped
                    bursts += 1
                    if adv > 1:
                        skipped += adv - 1
                        events.append("%.2f skipped %d point(s): %d owed after a %.1f ms send"
                                      % (runner.clock, adv - 1, owed, sends_ms[-1] if len(sends_ms) else 0.0))
                else:                                 # the PC's clock: skip, never burst
                    adv = owed
                    skipped += owed - 1
                    events.append("%.2f skipped %d tick(s): woke %.1f ms after the tick was due; the send before "
                                  "took %.1f ms" % (runner.clock, owed - 1, (now - next_t) * 1000.0,
                                                    sends_ms[-1] if len(sends_ms) else 0.0))
            next_t += owed * step
            commands.skipped = skipped
            for i in range(n):
                a = adv if i == 0 else 1
                tick += a
                q = runner.step(dt * speed * a) if pid > 0 else prev
                guard.check(prev, q, a)
                t0 = time.perf_counter()
                ret = ctrl.servo_j(q, dt, pid)
                sends_ms.append((time.perf_counter() - t0) * 1000.0)
                code = ret[0] if isinstance(ret, (list, tuple)) else ret
                if code != 0:
                    raise StreamFault("ServoJ point %d returned %s" % (pid, ret))
                ticks_cmd.add(tick, q)                # the point's place in the play time
                prev, pid = q, pid + 1
            if osc is not None and pid % OSC_EVERY < n:
                try:
                    osc.send(prev)
                except Exception:
                    pass                            # status out is best effort; never stop the arm for it
            if ending and runner.state == "PAUSED":
                break
    except StreamFault as e:
        fault = str(e)
    except KeyboardInterrupt:
        fault = "stopped by the operator (Ctrl+C)"
    finally:
        if fault is not None:
            try:
                ctrl.stop()
            except Exception:
                pass
            runner.fault(fault)
        try:
            ctrl.servo_end()
        except Exception:
            pass
        time.sleep(0.3)
        link.stop()
        gc.unfreeze()
    if fault:
        log("STOPPED: %s" % fault)
    rep = report(start, dt, ticks_cmd, sends_ms, late_ms, skipped, link, runner, guard, fault, events, speed, analyse,
                 step=step)
    rep["pacing"] = "controller clock" if clocked else "pc clock"
    rep["clock_ppm"] = clock_ppm
    rep["bursts"] = bursts
    h = sorted(link.queue_hist)
    if h:                                           # the motion queue, for the record (it does not show the backlog)
        rep["queue"] = {"min": h[0], "median": h[len(h) // 2], "p99": h[int(len(h) * 0.99)], "max": h[-1]}
    return rep, ticks_cmd, start


def _dense(ticks_cmd):
    """Commanded pose per tick from the first to the last sent (a skipped
    tick holds the previous pose), for tracking()."""
    out = []
    k0, k1 = int(ticks_cmd.row(0)[0]), int(ticks_cmd.row(-1)[0])
    it = iter(ticks_cmd)
    cur = next(it)
    nxt = next(it, None)
    for k in range(k0, k1 + 1):
        while nxt is not None and nxt[0] <= k:
            cur, nxt = nxt, next(it, None)
        out.append(cur[1])
    return out


def lag_by_window(t0, dense, dt, feedback, window_s=LAG_WINDOW_S, lags_ms=range(0, 504, 4)):
    """The best constant lag (ms) of the arm behind the commands, per window
    of the run. It should stay put; if it grows, the controller consumes
    ServoJ slower than this PC's clock sends it and its queue is filling
    (seen on SimMachine 2026-09-27: 40 -> 68 ms over 30 min)."""
    def cmd_at(s):
        k = s / dt
        i = int(k)
        if i < 0:
            return dense[0]
        if i >= len(dense) - 1:
            return dense[-1]
        f = k - i
        return [a + f * (b - a) for a, b in zip(dense[i], dense[i + 1])]
    out = []
    pts = [(t - t0, q) for t, q in feedback if t - t0 > 0.3]
    w = 0.0
    end = (len(dense) - 1) * dt
    while w < end:
        win = [p for p in pts if w <= p[0] < w + window_s][::4]
        best = None
        for lag in lags_ms:
            sq = sum(sum((a - b) ** 2 for a, b in zip(q, cmd_at(s - lag / 1000.0))) for s, q in win)
            if win and (best is None or sq < best[1]):
                best = (lag, sq)
        if best:
            out.append(best[0])
        w += window_s
    return out


def report(start, dt, ticks_cmd, sends_ms, late_ms, skipped, link, runner, guard, fault, events, speed, analyse=True,
           step=None):
    ended = "at a hub" if not fault else ("stopped" if fault in OPERATOR_STOPS else "fault")
    rep = {"ended": ended, "fault": fault, "speed": speed, "scan_speed": getattr(runner, "scan_speed", 1.0),
           "sends": len(ticks_cmd), "skipped": skipped,
           "duration_s": round(ticks_cmd.row(-1)[0] * dt, 2) if len(ticks_cmd) else 0.0,
           "worst_step_of_limit": round(guard.worst_ratio, 3),
           "segments_played": [n for _, n in runner.history],
           "final_state": runner.state, "events": events[-200:]}
    if sends_ms:
        st = sorted(sends_ms)
        rep.update(send_ms_p50=round(st[len(st) // 2], 2), send_ms_p95=round(st[int(len(st) * 0.95)], 2),
                   send_ms_max=round(st[-1], 2), late_over_2ms=sum(1 for x in late_ms if x > 2.0),
                   max_late_ms=round(max(late_ms), 2))
    rep["feedback_samples"] = len(link.rows)
    rep["feedback_slow_reads_dropped"] = link.slow_reads
    rep["controller_error"] = link.error
    if analyse:
        add_tracking(rep, start, step or dt, ticks_cmd, link.samples)
    return rep


def point_step(dt, clock_ppm):
    """This PC's seconds per point: dt on the controller's clock (clock_ppm
    against this PC's; None: the PC's clock paces)."""
    return dt / (1.0 + (clock_ppm or 0.0) * 1e-6)


def add_tracking(rep, start, step, ticks_cmd, fb):
    """Actual vs commanded, after the best lag, and the lag per window, into
    rep. step: this PC's seconds per point (point_step) -- the feedback is
    stamped on this PC's clock, so point k is compared where it was sent,
    at k * step (on k * dt a calibrated run showed the calibration itself as
    a growing lag, 2026-09-28). Seconds of work on a long run (a few million
    interpolations), so main() writes the raw log first and adds this after."""
    if len(ticks_cmd) > 1:
        every = max(1, len(fb) // TRACKING_MAX_SAMPLES)
        t0, dense = start + ticks_cmd.row(0)[0] * step, _dense(ticks_cmd)
        rep.update(P.tracking(t0, dense, step, fb[::every]))
        rep["tracking_samples_used"] = len(fb[::every])
        rep["lag_ms_by_window"] = lag_by_window(t0, dense, step, fb, window_s=LAG_WINDOW_S)
        rep["lag_window_s"] = LAG_WINDOW_S
    return rep


def write_log(out_dir, stamp, rep, ticks_cmd, step, feedback, start):
    """The report and the joints: commanded at the PC time each point was
    sent for (k * step, point_step), actual as read -- one time base."""
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, "stream_%s" % stamp)
    with open(base + ".json", "w", newline="\n") as f:
        json.dump(rep, f, indent=1)
    with open(base + "_cmd.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s"] + ["j%d_deg" % i for i in range(1, 7)])
        for k, q in ticks_cmd:
            w.writerow(["%.4f" % (k * step)] + ["%.4f" % x for x in q])
    with open(base + "_actual.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s"] + ["j%d_deg" % i for i in range(1, 7)])
        for t, q in feedback:
            w.writerow(["%.4f" % (t - start)] + ["%.4f" % x for x in q])
    return base


# --------------------------------------------------------------------------

def self_test():
    """Against a fake controller, in real time (a few seconds)."""
    import show as S
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    A = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    B = [20.0] + A[1:]

    def seg(name, kind, q0, q1, start, end, dur):
        n = 20
        t = [dur * i / n for i in range(n + 1)]
        # smooth, at rest at both ends
        q = [[a + (b - a) * (3 * (i / n) ** 2 - 2 * (i / n) ** 3) for a, b in zip(q0, q1)] for i in range(n + 1)]
        return S.Segment(name, kind, t, q, start, end, {"action": "float", "effort": {"time": 0.0}})

    wave = [5.0] + A[1:]
    segs = [S.Segment("a_wave", "idle", [0.0, 0.5, 1.0], [A, wave, A], "a", "a", {"action": "float"}),
            seg("a_hold", "idle", A, A, "a", "a", 0.8), seg("b_hold", "idle", B, B, "b", "b", 0.8),
            seg("move_a_b", "move", A, B, "a", "b", 1.0), seg("move_b_a", "move", B, A, "b", "a", 1.0)]
    graph = S.Graph({"a": A, "b": B}, segs, {"start_hub": "a", "sequences": {"visit": {"hub": "b", "count": 1}}})
    vel = [120.0] * 6
    lim = [(-175, 175), (-265, 85), (-162, 162), (-265, 85), (-175, 175), (-175, 175)]
    dt = 1.0 / RATE_HZ

    class FakeCtrl:
        def __init__(self, error_after=None, slow_every=None):
            self.q, self.sent, self.calls = list(A), [], []
            self.error_after, self.slow_every = error_after, slow_every

        def servo_start(self):
            self.calls.append("start")

        def servo_j(self, q, cmd_t, cmd_id):
            if self.slow_every and len(self.sent) % self.slow_every == self.slow_every - 1:
                time.sleep(3 * dt)
            self.sent.append((cmd_id, list(q)))
            self.q = list(q)
            return 0

        def servo_end(self):
            self.calls.append("end")

        def stop(self):
            self.calls.append("stop")

        def joints(self):
            return list(self.q)

        def error_code(self):
            if self.error_after is not None and len(self.sent) > self.error_after:
                return [0, 5, 7]
            return [0, 0, 0]

    def FakeLink(ctrl):
        return Link(ctrl=ctrl)

    bad = [30.0] + A[1:]                             # a broken clip: 30 deg in 4 ms, mid-clip
    broken = S.Graph({"a": A}, [S.Segment("a_broken", "idle", [0.0, 0.3, 0.304, 1.0], [A, A, bad, A], "a", "a",
                                          {"action": "float"})], {"start_hub": "a"})

    def run(ctrl, minutes, speed=1.0, trigger=None, stop_at=None, g_=graph, clock_ppm=None):
        r = S.Runner(g_, S.Selector(g_.idle(), 1, seed=0), hub_stay=(2, 2), seed=0)
        cmds = Commands(r)
        if trigger:
            cmds.trigger(trigger)
        if stop_at:
            threading.Timer(stop_at, cmds.stop).start()
        g = Guard(vel, lim, dt, speed)
        rep, _, _ = stream(ctrl, FakeLink(ctrl), r, cmds, g, dt, speed, minutes, log=lambda *a: None,
                           clock_ppm=clock_ppm)
        return rep, r

    c = FakeCtrl()
    rep, r = run(c, 3.0 / 60)
    steps = [max(abs(x - y) for x, y in zip(a[1], b[1])) for a, b in zip(c.sent, c.sent[1:])]
    check("a run ends at a hub, at rest, without a fault", rep["ended"] == "at a hub" and r.state == "PAUSED"
          and c.sent[-1][1] in (A, B), (rep["ended"], r.state))
    check("one continuous stream: ServoMoveStart once, ServoMoveEnd once, no StopMotion",
          c.calls == ["start", "end"], c.calls)
    check("no jump between segments (largest step within the limit)", rep["worst_step_of_limit"] <= 1.0,
          rep["worst_step_of_limit"])
    check("tick ids increase", all(b[0] > a[0] for a, b in zip(c.sent, c.sent[1:])))
    check("the run lasted about the asked time (plus the clip that finishes)", 3.0 <= rep["duration_s"] <= 3.0 + 1.2,
          rep["duration_s"])
    check("segments were played through the graph", len(rep["segments_played"]) >= 3, rep["segments_played"])
    check("tracking is reported", "tracking_after_lag_max_deg" in rep, list(rep)[-6:])

    c = FakeCtrl()
    rep, r = run(c, 3.0 / 60, trigger="visit")
    check("a trigger from another thread is applied between ticks: the sequence plays",
          "move_a_b" in rep["segments_played"] and "b_hold" in rep["segments_played"], rep["segments_played"])

    c = FakeCtrl(error_after=60)
    rep, r = run(c, 3.0 / 60)
    check("a controller error stops the stream: FAULT, StopMotion, ServoMoveEnd",
          rep["ended"] == "fault" and "controller error" in rep["fault"] and c.calls[-2:] == ["stop", "end"]
          and r.state == "FAULT", (rep["fault"], c.calls))

    c = FakeCtrl()
    rep, r = run(c, 3.0 / 60, g_=broken)
    check("a joint jump is refused before it is sent", rep["ended"] == "fault" and "steps" in rep["fault"]
          and max(abs(q[0] - A[0]) for _, q in c.sent) < 30.0, rep["fault"])

    c = FakeCtrl()
    rep, r = run(c, 1.0, stop_at=0.6)
    check("/robot/stop (or Ctrl+C) stops at once with StopMotion", rep["fault"] == "stop requested"
          and "stop" in c.calls and rep["duration_s"] < 1.5, (rep["fault"], rep["duration_s"]))
    check("... and the run reads as stopped, not as a fault", rep["ended"] == "stopped", rep["ended"])

    c = FakeCtrl(slow_every=40)
    rep, r = run(c, 2.0 / 60)
    check("a slow send is caught up by skipping ticks, not bursting (and still no jump)",
          rep["skipped"] > 0 and rep["worst_step_of_limit"] <= 1.0 and rep["ended"] == "at a hub",
          (rep["skipped"], rep["worst_step_of_limit"]))
    why = [e for e in rep["events"] if "skipped" in e]
    check("... and each skip says why (how late the wake-up, how long the send before)",
          why and all("send before took" in e for e in why), why[:2])
    check("the stream runs at a high priority on Windows",
          any(e.startswith("priority: " + ("process high" if os.name == "nt" else "")) for e in rep["events"]),
          [e for e in rep["events"] if e.startswith("priority")])

    import collections

    class QueueCtrl(FakeCtrl):
        """A controller with a motion queue, playing one point per period on
        its own clock -- slow_ctrl slower than this PC's (the real FR20:
        ~0.04 %; here exaggerated)."""

        def __init__(self, slow_ctrl=0.03, slow_every=None):
            super().__init__(slow_every=slow_every)
            self.pending, self.t_last, self.lock = collections.deque(), None, threading.Lock()
            self.peak = 0
            self.period = dt * (1.0 + slow_ctrl)

        def _consume(self):
            now = time.perf_counter()
            if self.t_last is None or not self.pending:
                self.t_last = now
            while self.pending and now - self.t_last >= self.period:
                self.q = self.pending.popleft()
                self.t_last += self.period

        def servo_j(self, q, cmd_t, cmd_id):
            if self.slow_every and len(self.sent) % self.slow_every == self.slow_every - 1:
                time.sleep(3 * dt)
            with self.lock:
                self._consume()
                self.pending.append(list(q))
                self.peak = max(self.peak, len(self.pending))
                self.sent.append((cmd_id, list(q)))
            return 0

        def joints(self):
            with self.lock:
                self._consume()
                return list(self.q)

    global MAX_PPM
    max_ppm, MAX_PPM = MAX_PPM, 50000.0               # an exaggerated 3 % slow controller clock, believed here
    try:
        blind, told = QueueCtrl(), QueueCtrl()
        rep_b, _ = run(blind, 10.0 / 60)
        rep_t, _ = run(told, 10.0 / 60, clock_ppm=(1.0 / 1.03 - 1.0) * 1e6)
        slow = QueueCtrl(slow_ctrl=0.0, slow_every=40)
        rep_s, _ = run(slow, 2.0 / 60, clock_ppm=0.0)
    finally:
        MAX_PPM = max_ppm
    check("without a clock calibration the PC's clock paces, and a slow controller's buffer piles up (the FR20's lag)",
          rep_b["pacing"] == "pc clock" and blind.peak > 25, blind.peak)
    check("... with its clock rate: points paced by the controller's clock, its buffer stays small",
          rep_t["pacing"] == "controller clock" and told.peak <= 12
          and rep_t["ended"] == "at a hub", (rep_t["clock_ppm"], told.peak))
    check("... and a slow send is sent late, back to back: nothing skipped, no jump",
          rep_s["skipped"] == 0 and rep_s["bursts"] > 0 and rep_s["worst_step_of_limit"] <= 1.0,
          (rep_s["skipped"], rep_s["bursts"], rep_s["worst_step_of_limit"]))

    sug = suggest_clock_ppm({"lag_ms_by_window": [104, 280, 460], "lag_window_s": 180.0, "clock_ppm": None})
    check("a lag growing ~1 ms per s asks for a clock calibration of ~ -990 ppm", abs(sug + 988.9) < 1.0, sug)

    import io

    class Pipe(io.StringIO):
        def isatty(self):
            return False
    out = io.StringIO()
    said = confirm("MoveJ to rest\nat 10 %", Pipe("yes\n"), out)
    check("started by a window, a question goes out as one [ask] line and 'yes' comes back",
          said and out.getvalue() == ASK + json.dumps("MoveJ to rest\nat 10 %") + "\n", out.getvalue())
    check("... anything but yes, or no answer at all, is a no",
          not confirm("go?", Pipe("no\n"), io.StringIO()) and not confirm("go?", Pipe(""), io.StringIO()))

    r = S.Runner(graph, S.Selector(graph.idle(), 1, seed=0), hub_stay=(2, 2), seed=0)
    check("the speed cannot be changed while streaming (no live speed command)",
          not hasattr(Commands(r), "set_speed"))

    c = FakeCtrl()
    rep, r = run(c, 2.0 / 60, speed=0.5)
    check("speed 0.5 plays the same clips half as fast", rep["worst_step_of_limit"] <= 1.0
          and rep["ended"] == "at a hub", rep["worst_step_of_limit"])
    dense = [[20.0 * math.sin(0.8 * k * dt)] * 6 for k in range(int(40 / dt))]
    fb = [(k * dt + (0.04 if k * dt < 20 else 0.064), q) for k, q in enumerate(dense)][::2]
    lags = lag_by_window(0.0, dense, dt, fb, window_s=10.0)
    check("the lag is measured per window (a growing one shows)", lags == [40, 40, 64, 64], lags)
    # paced by a (exaggerated) slow controller clock: point k leaves this PC at
    # k * step, not k * dt; an arm 40 ms behind every point has a steady lag
    step = point_step(dt, -3000.0)
    ticks = Rows()
    for k, q in enumerate(dense):
        ticks.add(k, q)
    fb = [(k * step + 0.04, q) for k, q in enumerate(dense)][::2]
    global LAG_WINDOW_S
    keep, LAG_WINDOW_S = LAG_WINDOW_S, 10.0
    try:
        paced = add_tracking({}, 0.0, step, ticks, fb)["lag_ms_by_window"]
    finally:
        LAG_WINDOW_S = keep
    check("a clock-paced run's lag is measured on the PC times its points were sent at (steady, not the "
          "calibration's own drift)", len(paced) >= 4 and set(paced) == {40}, paced)
    check("... which is dt at the controller's rate", abs(step - dt / (1.0 - 3000e-6)) < 1e-15
          and point_step(dt, None) == dt, step)

    rows = Rows()
    rows.add(3, [1, 2, 3, 4, 5, 6])
    rows.add(4, [7, 8, 9, 10, 11, 12])
    check("log rows round-trip through the flat array", list(rows) == [(3.0, [1, 2, 3, 4, 5, 6]),
                                                                    (4.0, [7, 8, 9, 10, 11, 12])]
          and rows.row(-1)[0] == 4.0 and len(rows) == 2)

    class SlowCtrl(FakeCtrl):
        def joints(self):
            time.sleep(0.05)
            return list(self.q)
    ln = Link(ctrl=SlowCtrl())
    ln.start()
    time.sleep(0.3)
    ln.stop()
    check("a slow joint read is not kept (its time is not known)", len(ln.rows) == 0 and ln.slow_reads > 0,
          (len(ln.rows), ln.slow_reads))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    import robot_profile as RP
    import show as S
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", help="the show config (its compiled graph is read: run show.py build first)")
    tgt = ap.add_mutually_exclusive_group(required=True)
    tgt.add_argument("--sim", action="store_true", help="the target is SimMachine")
    tgt.add_argument("--hardware", action="store_true", help="the target is the physical arm: asks first, speed 0.3")
    ap.add_argument("--ip", default=None, help="controller IP (default: playback.toml's)")
    ap.add_argument("--speed", type=float, default=None, help="show clock scale (default 1.0 sim, 0.3 hardware)")
    ap.add_argument("--scan-speed", type=float, default=1.0,
                    help="the scan alone at this fraction of its built speed (0 < f <= 1; tuning an exposure)")
    ap.add_argument("--osc-out", action="append", default=[], metavar="HOST:PORT",
                    help="also send the status here (a control window and TouchDesigner both listening); repeatable")
    ap.add_argument("--minutes", type=float, default=5.0)
    ap.add_argument("--osc", action="store_true", help="TouchDesigner in and out (the config's osc ports)")
    ap.add_argument("--move-vel", type=float, default=None, help="MoveJ %% to the start hub (default 20 sim, 10 hardware)")
    ap.add_argument("--env", default=os.path.join(ROOT, "envs", "volvox_lab.usda"), help="the room, for the start move")
    ap.add_argument("--log", default=os.path.join(ROOT, "logs", "stream"), help="where the report and joints go")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--yes", action="store_true", help="skip the hardware confirmation")
    ap.add_argument("--clock-ppm", type=float, default=None,
                    help="the controller's clock against this PC's (controller_clock.py measures it; default: "
                         "playback.toml's [controller_clock_ppm] for this IP; none: the PC's clock paces)")
    ap.add_argument("--goto-start", action="store_true",
                    help="only move to the start hub (checked against the room, --move-vel %%), then end")
    a = ap.parse_args(argv)
    speed = a.speed if a.speed is not None else (HARDWARE_SPEED if a.hardware else 1.0)
    if not MIN_SPEED <= speed <= 1.0:
        ap.error("--speed must be within %g..1 (the clips are made at the show's range speed)" % MIN_SPEED)
    also = []
    for t in a.osc_out:
        host, _, port = t.rpartition(":")
        if not host or not port.isdigit():
            ap.error("--osc-out wants HOST:PORT, got %r" % t)
        also.append((host, int(port)))
    ip = a.ip or _toml_ip()
    cfg_path = os.path.abspath(a.config)
    cfg = json.load(open(cfg_path))
    graph = S.Graph.load(S.compiled_path(cfg_path))
    prof = RP.load("fr20")
    dt = 1.0 / RATE_HZ
    if not 0.0 < a.scan_speed <= 1.0:
        ap.error("--scan-speed must be within 0..1 (the scan plays at most as fast as built)")
    runner = S.runner_for(graph, seed=a.seed, scan_speed=a.scan_speed)
    start_q = runner.step(0.0)

    def ask(text):
        if not a.hardware or a.yes:
            return True
        return confirm(text)

    ctrl = P.Controller(ip)
    print("%s at %s: %s" % ("HARDWARE" if a.hardware else "SimMachine", ip, ctrl.model()))
    P._require_no_error(ctrl, "before starting")
    rep = {"target": "hardware" if a.hardware else "sim", "config": os.path.relpath(cfg_path, ROOT)}
    move_vel = a.move_vel if a.move_vel is not None else (10.0 if a.hardware else 20.0)
    off = P.move_checked(ctrl, start_q, a.env, "fr20", move_vel, rep, "start", ask, "the start hub (%s)" % runner.hub)
    if off is None:
        print(json.dumps(rep, indent=1))
        return 1
    if a.goto_start:
        print("at the start hub (%s), %.2f deg off. Not streaming (--goto-start)." % (runner.hub, off))
        return 0
    plan = ("Stream %s: %.1f min at speed %.2f (fixed for the run) from hub %s, ServoJ %g Hz%s."
            "\nStop: Ctrl+C or OSC /robot/stop "
            "(software stop). Keep a hand on the E-stop." % (cfg.get("name", "show"), a.minutes, speed, runner.hub,
                                                              RATE_HZ, ", TouchDesigner on OSC" if a.osc else ""))
    print(plan)
    if not ask(plan):
        return 1
    clock_ppm = a.clock_ppm if a.clock_ppm is not None else toml_clock_ppm(ip)
    if clock_ppm is None:
        print("no playback-rate calibration for %s: the PC's clock paces (the controller's lag may grow). After this "
              "run, put the report's clock_ppm_suggested in playback.toml [controller_clock_ppm] \"%s\"" % (ip, ip))
    cmds = Commands(runner, speed)
    osc = None
    if a.osc:
        o = cfg["osc"]
        osc = S.OscBridge(cmds, o["listen_port"], o["send_host"], o["send_port"], also=also)
        print("OSC in :%d, out %s:%d" % (o["listen_port"], o["send_host"], o["send_port"]))
    guard = Guard(RP.velocity_limits(prof), RP.motion_limits(prof), dt, speed)     # J6: the tool cable's range
    link = Link(ip)
    try:
        out, ticks_cmd, start = stream(ctrl, link, runner, cmds, guard, dt, speed, a.minutes, osc=osc, analyse=False,
                                       clock_ppm=clock_ppm)
    finally:
        if osc is not None:
            osc.close()
    out.update(rep)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    # the data first: a report that is interrupted while it analyses still leaves the run behind
    step = point_step(dt, clock_ppm)
    base = write_log(a.log, stamp, out, ticks_cmd, step, link.samples, start)
    print("log written: %s (the joints; tracking follows)" % (base + ".json"))
    sys.stdout.flush()
    add_tracking(out, start, step, ticks_cmd, link.samples)
    out["clock_ppm_suggested"] = suggest_clock_ppm(out)
    with open(base + ".json", "w", newline="\n") as f:
        json.dump(out, f, indent=1)
    summary = {k: out.get(k) for k in ("ended", "fault", "duration_s", "sends", "skipped", "worst_step_of_limit",
                                       "speed",
                                       "tracking_after_lag_max_deg", "tracking_after_lag_rms_deg", "best_lag_ms",
                                       "send_ms_p95", "max_late_ms", "controller_error", "pacing", "clock_ppm",
                                       "lag_ms_by_window", "clock_ppm_suggested")}
    print(json.dumps(summary, indent=1))
    if out.get("clock_ppm_suggested") is not None and abs(out["clock_ppm_suggested"] - (clock_ppm or 0.0)) > 50:
        print('the lag drifted: set playback.toml [controller_clock_ppm] "%s" = %.1f (was %s)'
              % (ip, out["clock_ppm_suggested"], clock_ppm))
    print("log:", base + ".json")
    return 0 if out["ended"] in ("at a hub", "stopped") else 1       # a stop by the operator is not an error


def toml_clock_ppm(ip, path=None):
    """playback.toml's [controller_clock_ppm] value for this controller's IP, or None."""
    import tomllib
    path = path or os.path.join(ROOT, "playback.toml")
    if not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        v = tomllib.load(f).get("controller_clock_ppm", {}).get(ip)
    return float(v) if v is not None else None


def suggest_clock_ppm(rep):
    """The clock calibration the run's own lag asks for: the lag per window
    should stay put; a lag growing by g ms per s means the controller plays
    g * 1000 ppm slower than the points were paced. None with < 2 windows."""
    w = rep.get("lag_ms_by_window") or []
    if len(w) < 2:
        return None
    n = len(w)
    mx = (n - 1) / 2.0
    slope = sum((i - mx) * (x - sum(w) / n) for i, x in enumerate(w)) / sum((i - mx) ** 2 for i in range(n))
    grow = slope / rep.get("lag_window_s", LAG_WINDOW_S)     # ms of lag per s
    return round((rep.get("clock_ppm") or 0.0) - grow * 1000.0, 1)


def _toml_ip():
    path = os.path.join(ROOT, "playback.toml")
    for line in open(path):
        s = line.split("#")[0].strip()
        if s.startswith("ip") and "=" in s:
            return s.split("=", 1)[1].strip().strip('"')
    raise SystemExit("no ip in playback.toml: pass --ip")


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
