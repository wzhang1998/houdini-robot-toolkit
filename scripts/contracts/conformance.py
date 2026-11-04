"""Conformance: the tests a new arm backend, camera source or Director
passes before it goes near the show. The same suites run against the
reference implementations here, so they are known to pass on something.

    python scripts/contracts/conformance.py                 everything, against SimBackend, RecordedPerception,
                                                            BaselineDirector (+ show_stream's loop on SimBackend)
    python scripts/contracts/conformance.py --quick         without the 3 s stream run
    python scripts/contracts/conformance.py --robot module:factory_fn [--profile uf850]
    python scripts/contracts/conformance.py --perception module:source_fn
    python scripts/contracts/conformance.py --director module:director_fn

A new one is accepted by handing its constructor to the same suite:

    robot_suite(lambda: MyFactory(...), "uf850", report)        a BackendFactory: command() / feedback()
    perception_suite(lambda: MySource(...), report)
    director_suite(lambda graph, seed: MyDirector(...), graph, report)

THE ROBOT SUITE MOVES THE ARM IT IS GIVEN: run it against the vendor's
simulator (SimMachine, UFactory Studio's simulation), never a physical
arm. The fault tests (a jump, a target past a limit) run only on a backend
whose BackendInfo says simulated=True, unless --allow-fault-tests.
"""

import argparse
import importlib
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from contracts import director as D  # noqa: E402
from contracts import perception as PC  # noqa: E402
from contracts import robot as R  # noqa: E402

RATE_HZ = 125.0
ARRIVE_TOL_DEG = 0.5
POLL_BUDGET_MS = 5.0


class Report:
    def __init__(self, verbose=True):
        self.fails, self.n, self.verbose, self.section_name = [], 0, verbose, ""

    def section(self, name):
        self.section_name = name
        if self.verbose:
            print("\n== %s" % name)

    def check(self, label, ok, detail=""):
        self.n += 1
        if self.verbose:
            print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)[:300]) if detail != "" else ""))
        if not ok:
            self.fails.append("%s: %s" % (self.section_name, label))
        return ok

    def done(self):
        print("\n%d checks, %d failed%s" % (self.n, len(self.fails), "" if not self.fails else ":\n  " +
                                             "\n  ".join(self.fails)))
        return 1 if self.fails else 0


# --------------------------------------------------------------------------
# Robot
# --------------------------------------------------------------------------

def _wait_joints(conn, target, tol, timeout_s=10.0):
    t_end = time.time() + timeout_s
    while True:
        off = max(abs(a - b) for a, b in zip(conn.joints(), target))
        if off <= tol or time.time() > t_end:
            return off
        time.sleep(0.02)


def _refused(conn, fb, code, start, target):
    """A jump / out-of-limit target refused: a non-zero return or a fault within 0.5 s, and the arm
    never near the target."""
    t_end = time.time() + 0.5
    faulted = not R.is_ok(code)
    while not faulted and time.time() < t_end:
        faulted = list(fb.error_code())[:3] != R.NO_ERROR
        time.sleep(0.01)
    away = max(abs(a - b) for a, b in zip(fb.joints(), target))
    return faulted, away


def robot_suite(make_factory, profile_id, report, integration=True, allow_fault_tests=False):
    import robot_profile as RP
    prof = RP.load(profile_id)
    n = prof["robot"]["num_joints"]
    limits = [tuple(x) for x in prof["robot"]["limits_deg"]]
    vel = RP.velocity_limits(prof)
    report.section("robot: %s" % profile_id)
    f = make_factory()
    c, fb = f.command(), f.feedback()
    report.check("both connections are RobotBackends", isinstance(c, R.RobotBackend) and isinstance(fb, R.RobotBackend)
                 and not R.missing_methods(c), R.missing_methods(c))
    report.check("they are two connections, not one object", c is not fb)
    info = getattr(c, "info", None)
    report.check("BackendInfo says what it is (kind, profile, simulated)", isinstance(info, R.BackendInfo)
                 and info.profile_id == profile_id, info)
    simulated = bool(info and info.simulated)
    report.check("model() names the profile (fairino_player.main refuses otherwise)",
                 profile_id.upper() in str(c.model()).upper(), c.model())
    c.reset_errors()
    e = list(c.error_code())
    report.check("error_code(): [ret, main, sub] ints, [0, 0, 0] after reset_errors()",
                 len(e) >= 3 and all(isinstance(x, int) for x in e[:3]) and e[:3] == R.NO_ERROR, e)
    q0 = list(c.joints())
    report.check("joints(): %d finite angles inside the limits" % n,
                 len(q0) == n and R.finite(q0) and R.within_limits(q0, limits), q0)
    report.check("the feedback connection reads the same pose", max(abs(a - b) for a, b in zip(q0, fb.joints())) < 0.05)
    for name in R.OPTIONAL_METHODS:
        if hasattr(c, name) and name == "queue_length":
            ql = c.queue_length()
            report.check("queue_length(): an int or None", ql is None or isinstance(ql, int), ql)

    # a planned move, small and slow
    target = list(q0)
    j = 0
    lo, hi = limits[j]
    target[j] = q0[j] + (5.0 if q0[j] + 5.0 < hi - 1.0 else -5.0)
    c.prepare()
    c.move_to(target, 10.0)
    off = _wait_joints(fb, target, ARRIVE_TOL_DEG)
    report.check("move_to(): there within %.1f deg (waiting on joints(), as goto does)" % ARRIVE_TOL_DEG,
                 off <= ARRIVE_TOL_DEG, round(off, 3))
    raised = False
    try:
        bad = list(target)
        bad[0] = limits[0][1] + 10.0
        c.move_to(bad, 10.0)
    except RuntimeError:
        raised = True
    report.check("move_to() past a limit raises (RuntimeError) and nothing moves",
                 raised and max(abs(a - b) for a, b in zip(fb.joints(), target)) < ARRIVE_TOL_DEG)
    c.reset_errors()

    # a stream: one joint, a 2 deg raised cosine over 2 s at the servo rate, on an absolute clock
    dt = 1.0 / RATE_HZ
    base = list(fb.joints())
    jj = n - 1
    amp = 2.0 if base[jj] + 2.0 < limits[jj][1] else -2.0
    steps = int(2.0 * RATE_HZ)
    c.servo_start()
    ok_all, t0 = True, time.perf_counter()
    last = base
    for k in range(1, steps + 1):
        q = list(base)
        q[jj] = base[jj] + amp * (1.0 - math.cos(2.0 * math.pi * k / steps)) / 2.0
        while time.perf_counter() < t0 + k * dt:
            time.sleep(0.0005)
        ok_all = ok_all and R.is_ok(c.servo_j(q, dt, k))
        last = q
    c.servo_end()
    off = _wait_joints(fb, last, ARRIVE_TOL_DEG, 2.0)
    report.check("servo_j at %g Hz: every point taken, the arm where the last one said" % RATE_HZ,
                 ok_all and off <= ARRIVE_TOL_DEG and list(fb.error_code())[:3] == R.NO_ERROR,
                 (ok_all, round(off, 3), fb.error_code()))

    # stop mid-stream: no further motion
    c.servo_start()
    here = list(fb.joints())
    for k in range(1, 20):
        q = list(here)
        q[jj] += 0.05 * k * (1 if amp > 0 else -1)
        c.servo_j(q, dt, 1000 + k)
        time.sleep(dt)
    fb.stop()                                           # the watchdog's way: from the feedback connection
    a = list(fb.joints())
    time.sleep(0.3)
    b = list(fb.joints())
    c.servo_end()
    report.check("stop() from the feedback connection: still 0.3 s later", max(abs(x - y) for x, y in zip(a, b)) < 0.2,
                 max(abs(x - y) for x, y in zip(a, b)))
    c.reset_errors()

    if simulated or allow_fault_tests:
        c.servo_start()
        here = list(fb.joints())
        jump = list(here)
        jump[0] += 30.0 if here[0] + 30.0 < limits[0][1] else -30.0
        faulted, away = _refused(c, fb, c.servo_j(jump, dt, 5000), here, jump)
        report.check("a 30 deg jump in one tick is refused (a fault, the arm stays)", faulted and away > 25.0,
                     (faulted, round(away, 2), fb.error_code()))
        c.servo_end()
        c.reset_errors()
        report.check("reset_errors() clears it", list(c.error_code())[:3] == R.NO_ERROR, c.error_code())
        c.servo_start()
        here = list(fb.joints())
        past = list(here)
        past[1] = limits[1][1] + 5.0
        faulted, away = _refused(c, fb, c.servo_j(past, 10.0, 6000), here, past)
        report.check("a target past a joint limit is refused", faulted and R.within_limits(fb.joints(), limits),
                     (faulted, fb.joints()))
        c.servo_end()
        c.reset_errors()
    else:
        report.check("fault tests skipped: not a simulated backend (--allow-fault-tests on a vendor simulator)", True)

    if integration:
        _stream_integration(make_factory, profile_id, prof, report)


def _stream_integration(make_factory, profile_id, prof, report):
    """show_stream.stream itself -- the show's 125 Hz loop, its Guard, its feedback Link -- on this
    backend: a two-hub graph for 3 s, ending at a hub at rest."""
    import robot_profile as RP
    import show as S
    import show_stream as SS
    f = make_factory()
    c, fb = f.command(), f.feedback()
    lim = [tuple(x) for x in prof["robot"]["limits_deg"]]
    A = list(c.joints())
    B = list(A)
    B[0] += 10.0 if A[0] + 10.0 < lim[0][1] else -10.0

    def seg(name, kind, q0, q1, start, end, dur):
        k = 40
        t = [dur * i / k for i in range(k + 1)]
        q = [[a + (b - a) * (3 * (i / k) ** 2 - 2 * (i / k) ** 3) for a, b in zip(q0, q1)] for i in range(k + 1)]
        return S.Segment(name, kind, t, q, start, end, {"action": "float"})
    segs = [seg("a_hold", "idle", A, A, "a", "a", 0.8), seg("b_hold", "idle", B, B, "b", "b", 0.8),
            seg("move_a_b", "move", A, B, "a", "b", 1.0), seg("move_b_a", "move", B, A, "b", "a", 1.0)]
    g = S.Graph({"a": A, "b": B}, segs, {"start_hub": "a"})
    r = S.Runner(g, S.Selector(g.idle(), 1, seed=0), hub_stay=(1, 1), seed=0)
    dt = 1.0 / SS.RATE_HZ
    guard = SS.Guard(RP.velocity_limits(prof), lim, dt, 1.0)
    rep, ticks, _ = SS.stream(c, SS.Link(ctrl=fb), r, SS.Commands(r), guard, dt, 1.0, 3.0 / 60, log=lambda *a: None)
    report.check("show_stream.stream on this backend: 3 s, both hubs, ends at a hub, no fault",
                 rep["ended"] == "at a hub" and rep.get("fault") is None and "move_a_b" in rep["segments_played"],
                 (rep["ended"], rep.get("fault"), rep["segments_played"]))


# --------------------------------------------------------------------------
# Perception
# --------------------------------------------------------------------------

def perception_suite(make_source, report, horizon_s=8.0, step_s=1.0 / 30.0):
    report.section("perception")
    src = make_source()
    report.check("a PerceptionSource", isinstance(src, PC.PerceptionSource))
    info = src.info()
    report.check("info(): the robot base frame, a kind, a source id", info.frame == PC.FRAME and info.kind
                 and info.source_id, info)
    scenes, slow, errs, now = [], [], [], 0.0
    last_t, back_in_time, future = None, 0, 0
    while now <= horizon_s:
        t0 = time.perf_counter()
        s = src.poll(now)
        slow.append((time.perf_counter() - t0) * 1000.0)
        if s is not None:
            e = PC.validate(s.to_dict())
            if e:
                errs.append((round(now, 3), e))
            if last_t is not None and s.t < last_t:
                back_in_time += 1
            if s.received is not None and s.received > now + 1e-9:
                future += 1
            last_t = s.t
            scenes.append(s)
        now += step_s
    slow.sort()
    report.check("some frames over %.0f s" % horizon_s, len(scenes) > 0, len(scenes))
    report.check("every frame keeps the SceneState contract", not errs, errs[:2])
    report.check("poll() never blocks (p99 under %.0f ms)" % POLL_BUDGET_MS,
                 slow[int(0.99 * (len(slow) - 1))] < POLL_BUDGET_MS, round(slow[-1], 3))
    report.check("time never runs back", back_in_time == 0, back_in_time)
    report.check("nothing arrives before it is polled (received <= now)", future == 0, future)
    if scenes:
        k = horizon_s / 2.0
        report.check("polling the same time twice gives the same frame", src.poll(k) == src.poll(k))
        sm = PC.summarize(scenes[-1], now)
        report.check("summarize() takes every frame", isinstance(sm, PC.SceneSummary), sm)
    src.close()


# --------------------------------------------------------------------------
# Director
# --------------------------------------------------------------------------

def _inputs(graph, n=120, seed=7):
    """Questions a Director may be asked: every idle hub, empty / near / waving / stale rooms, recent
    picks, the energy wanted, a sequence's mood."""
    import random
    rng = random.Random(seed)
    lib = D.LibraryIndex.from_graph(graph)
    scenes = [PC.SceneSummary(),
              PC.SceneSummary(1, 0, {"id": 3, "dist_m": 1.8, "bearing_deg": -90.0, "height_m": 1.6}, 0, [3], 0.05, False),
              PC.SceneSummary(2, 1, {"id": 5, "dist_m": 2.6, "bearing_deg": -80.0, "height_m": 1.7}, 1, [5, 8], 0.1,
                              False),
              PC.SceneSummary(1, 0, {"id": 9, "dist_m": 3.5, "bearing_deg": -100.0, "height_m": 1.5}, 0, [9], 4.0, True)]
    out = []
    for k in range(n):
        hub = rng.choice(lib.hubs)
        here = [e.id for e in lib.at(hub)]
        recent = rng.sample(here, min(len(here), rng.randint(0, 6)))
        show = D.ShowState(round(rng.uniform(0, 3600), 2), hub, rng.choice(["IDLE", "IDLE", "MOVE"]),
                           rng.choice([None, None, "greet", "calm"]), rng.choice([None, "float", "punch"]),
                           None, rng.choice([None, -1.0, 0.2, 0.9]), round(rng.random(), 3), recent, recent[-6:], [])
        out.append(D.DirectorInput(rng.choice(scenes), lib, show, here, k + 1))
    return out


def director_suite(make_director, graph, report, budget_s=D.DEFAULT_BUDGET_S, minutes=5.0):
    report.section("director")
    d = make_director(graph, 1)
    report.check("a Director: a name, a version, decide()", isinstance(d, D.Director)
                 and isinstance(d.name, str) and isinstance(str(d.version), str))
    lat, invalid, mutated, not_json = [], [], 0, 0
    for inp in _inputs(graph):
        before = json.dumps(inp.to_dict(), sort_keys=True)
        t0 = time.perf_counter()
        ans = d.decide(inp)
        lat.append((time.perf_counter() - t0) * 1000.0)
        if isinstance(ans, dict):
            try:
                ans = D.Intent.from_dict(ans)
            except ValueError as e:
                invalid.append(str(e))
                continue
        e = D.validate_intent(ans, inp)
        if e:
            invalid.append(e)
        if json.dumps(inp.to_dict(), sort_keys=True) != before:
            mutated += 1
        try:
            json.dumps(ans.to_dict())
        except Exception:
            not_json += 1
    lat.sort()
    report.check("every answer is a valid Intent for its question (enum, fields, a clip at the hub)", not invalid,
                 invalid[:2])
    report.check("the input is never changed", mutated == 0, mutated)
    report.check("every answer serialises (the decision log)", not_json == 0, not_json)
    p95 = lat[int(0.95 * (len(lat) - 1))]
    report.check("p95 latency within the budget (%.0f ms)" % (budget_s * 1000), p95 <= budget_s * 1000.0,
                 "p95 %.3f ms" % p95)
    if getattr(d, "deterministic", True):
        a, b = make_director(graph, 5), make_director(graph, 5)
        qs = _inputs(graph, 40, seed=11)
        same = [a.decide(x) for x in qs] == [b.decide(x) for x in qs]
        report.check("same seed, same questions: the same answers (or set deterministic = False)", same)
    from contracts import director_replay as DR
    log, r, sel = DR.closed_loop(graph, make_director(graph, 1), None, minutes=minutes, seed=1, trigger_every=60,
                                 budget_s=max(budget_s, 0.5))
    report.check("%g min of the show on a dry clock: it plays, no error or invalid answer" % minutes,
                 len(r.history) > 5 and sel.counts["error"] == 0 and sel.counts["invalid"] == 0, sel.counts)


def baseline_equivalence(graph, report, minutes=15.0, seeds=(1, 2, 3)):
    """BaselineDirector through DirectorSelector plays exactly what show.Selector plays: the same
    clips, routes and triggers, tick for tick, with the operator's mood and energy changing."""
    import show as S
    from contracts.baseline_director import BaselineDirector
    report.section("baseline equivalence")
    for seed in seeds:
        r1 = S.runner_for(graph, seed=seed)
        r2, sel = D.runner_with(graph, BaselineDirector.from_graph(graph, seed), seed=seed, budget_s=5.0)
        events = {120.0: ("trigger", "scan"), 200.0: ("mood", "float"), 260.0: ("trigger", "greet"),
                  300.0: ("energy", 0.9), 420.0: ("energy", -1.0), 450.0: ("mood", None), 500.0: ("trigger", "calm"),
                  640.0: ("trigger", "scan")}
        t, dt = 0.0, 0.05
        pending = sorted(events)
        while t < minutes * 60.0:
            t += dt
            while pending and t >= pending[0]:
                kind, v = events[pending.pop(0)]
                for r in (r1, r2):
                    if kind == "trigger":
                        r.trigger(v)
                    elif kind == "mood":
                        r.sel.mood = v
                    else:
                        r.sel.energy = v
            r1.step(dt)
            r2.step(dt)
        report.check("seed %d: the same %d segments in the same order, every pick the Director's" % (seed,
                     len(r1.history)), r1.history == r2.history and sel.counts["director"] == sel.seq,
                     (len(r1.history), len(r2.history), sel.counts))
        sel.close()


def controller_structural(report):
    """fairino_player.Controller satisfies RobotBackend as it is (no connection is made)."""
    import fairino_player as P
    report.section("fairino_player.Controller")
    c = P.Controller("127.0.0.1")
    report.check("Controller is a RobotBackend (structurally; nothing connected)", isinstance(c, R.RobotBackend)
                 and not R.missing_methods(c), R.missing_methods(c))
    report.check("... with every optional method too", all(callable(getattr(c, m, None)) for m in R.OPTIONAL_METHODS))


def profiles_report(report):
    import robot_profile as RP
    report.section("profiles (readiness; informational)")
    for pid in RP.list_profiles():
        rows = R.profile_readiness(pid)
        missing = [a for a, s, _ in rows if s == "missing"]
        report.check("%s loads; missing for the full show: %s" % (pid, ", ".join(missing) or "nothing"), True)


def _load(spec):
    mod, _, fn = spec.partition(":")
    return getattr(importlib.import_module(mod), fn)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="skip the 3 s show_stream run")
    ap.add_argument("--robot", help="module:fn -> a BackendFactory (run against a simulator)")
    ap.add_argument("--profile", default="fr20")
    ap.add_argument("--allow-fault-tests", action="store_true")
    ap.add_argument("--perception", help="module:fn -> a PerceptionSource")
    ap.add_argument("--director", help="module:fn(graph, seed) -> a Director")
    ap.add_argument("--show", default=os.path.join(ROOT, "shows", "party.compiled.json"))
    a = ap.parse_args(argv)
    import show as S
    from contracts.baseline_director import BaselineDirector
    from contracts.recorded_perception import RecordedPerception, write_synthetic
    from contracts.sim_backend import SimFactory
    rep = Report()
    graph = S.Graph.load(a.show)
    if a.robot or a.perception or a.director:
        if a.robot:
            robot_suite(_load(a.robot), a.profile, rep, not a.quick, a.allow_fault_tests)
        if a.perception:
            perception_suite(_load(a.perception), rep)
        if a.director:
            director_suite(_load(a.director), graph, rep)
        return rep.done()
    import tempfile
    rec, _ = write_synthetic(tempfile.mkdtemp(), seconds=10.0)
    controller_structural(rep)
    robot_suite(lambda: SimFactory("fr20"), "fr20", rep, integration=not a.quick)
    robot_suite(lambda: SimFactory("uf850", q0=[0.0, -30.0, -60.0, 0.0, 60.0, 0.0]), "uf850", rep, integration=False)
    perception_suite(lambda: RecordedPerception(rec, latency_s=0.09), rep)
    perception_suite(lambda: RecordedPerception(rec, loop=True), rep, horizon_s=25.0)
    director_suite(lambda g, seed: BaselineDirector.from_graph(g, seed), graph, rep)
    baseline_equivalence(graph, rep)
    profiles_report(rep)
    return rep.done()


if __name__ == "__main__":
    sys.exit(main())
