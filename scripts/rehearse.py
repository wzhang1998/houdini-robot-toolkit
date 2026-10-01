"""The rehearsal: every motion the show can play, once each, in one run -- for
walking the real arm through all of it slowly before the show plays on its
own (the room, the cables, the paper's stand: what the build's checks do not
know), with how far along it is and how long is left.

A runner for show_stream.py (--rehearse) in place of the show's state
machine, as scan_test.py is: it plays only the build's own, checked
segments, in a fixed order, then ends at the start hub:

    at the start hub    the scan (to_scan, scan, from_scan), its idle clips (calm first)
    every move          each hub-to-hub move once (a circuit through them all,
                        back to the start hub); at each hub, the first time
                        there, its idle clips (calm first)

Sequences (greet, calm) and showpieces are idle clips at their hubs: all
covered. The interactive mode (track_mode.py) makes its moves live: not here.

    python scripts/rehearse.py shows/party_bigwipe.json --speed 0.3     the plan, its length (nothing moves)
    python scripts/show_stream.py shows/party_bigwipe.json --hardware --speed 0.3 --osc --rehearse
    python scripts/show_stream.py ... --rehearse --rehearse-from 41     on from step 41 (after a stop)
    python scripts/rehearse.py --self-test

Pause (/robot/pause, the window's) holds at the end of the running segment
(always at rest at a hub); Resume goes on. Triggers are ignored. The status
is the show's (show.OscBridge), so TouchDesigner's LEDs and ceiling follow
as in the show; its sequence reads "rehearsal 23/87", and plan_left /
plan_total (segment seconds) give show_stream the time left.
"""

import argparse
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import show as S  # noqa: E402


def move_circuit(graph, start):
    """Every hub-to-hub move once, from start back to start: a circuit through
    them all when each hub has as many moves out as in (Hierholzer's); else
    greedy -- an unused move from here, or the route to the nearest hub that
    has one (its moves played again) -- then the route home."""
    moves = [s for s in graph.segments if s.kind == "move"]
    out_n, in_n = {}, {}
    for m in moves:
        out_n[m.start] = out_n.get(m.start, 0) + 1
        in_n[m.end] = in_n.get(m.end, 0) + 1
    if all(out_n.get(h, 0) == in_n.get(h, 0) for h in set(out_n) | set(in_n)):
        left = {h: [m for m in moves if m.start == h] for h in out_n}
        stack, path = [(start, None)], []
        while stack:
            h, via = stack[-1]
            if left.get(h):
                m = left[h].pop(0)
                stack.append((m.end, m))
            else:
                stack.pop()
                if via is not None:
                    path.append(via)
        path.reverse()
        if len(path) == len(moves):
            return path
    unused, path, here = list(moves), [], start
    while unused:
        m = next((x for x in unused if x.start == here), None)
        if m is None:
            routes = [graph.route(here, x.start) for x in unused]
            routes = [r for r in routes if r is not None]
            if not routes:
                break                                    # the rest cannot be reached from here
            way = min(routes, key=lambda r: sum(x.duration for x in r))
            path += way
            here = way[-1].end
            continue
        unused.remove(m)
        path.append(m)
        here = m.end
    return path + (graph.route(here, start) or [])


def plan(graph, start=None):
    """The rehearsal's segments in order (see the module's doc). Raises
    ValueError when an idle clip's hub or the scan cannot be reached."""
    start = start or graph.info.get("start_hub") or graph.idle_hubs()[0]
    segs = {s.kind: s for s in graph.segments if s.kind in ("to_scan", "scan", "from_scan")}
    scan = [segs[k] for k in ("to_scan", "scan", "from_scan")] if len(segs) == 3 else []
    out, seen = [], set()

    def arrive(hub):
        if scan and hub == scan[0].start and "scan" not in seen:
            seen.add("scan")
            out.extend(scan)
            out.extend(graph.route(scan[-1].end, hub) or [])
        if hub not in seen:
            seen.add(hub)
            out.extend(sorted(graph.idle(hub), key=lambda c: float(c.labels.get("energy") or 0.0)))

    arrive(start)
    for m in move_circuit(graph, start):
        out.append(m)
        arrive(m.end)
    missed = [h for h in graph.idle_hubs() if h not in seen] + (["the scan"] if scan and "scan" not in seen else [])
    if missed:
        raise ValueError("the rehearsal cannot reach %s from %s" % (", ".join(missed), start))
    return out


def wall_s(seg, speed, scan_speed=1.0):
    """A segment's wall seconds at the show speed (the scan also at its own)."""
    return seg.duration / speed / (scan_speed if seg.kind == "scan" else 1.0)


class Rehearsal:
    def __init__(self, graph, start_step=1, scan_speed=1.0, log=print):
        self.g, self.log, self.scan_speed = graph, log, scan_speed
        self.items = plan(graph)
        if not 1 <= start_step <= len(self.items):
            raise ValueError("--rehearse-from %d: the rehearsal has steps 1..%d" % (start_step, len(self.items)))
        self.k = start_step - 1
        self.seg, self.seg_t, self.clock = self.items[self.k], 0.0, 0.0
        self.hub = self.seg.start
        self.state, self.fault_reason = S.STATE_OF[self.seg.kind], None
        self.paused, self.ended, self.finished = False, False, False
        self.sel = types.SimpleNamespace(mood=None, energy=None)     # what show_stream's Commands sets
        self.history = []

    @property
    def ends_when_paused(self):
        return self.finished                                        # show_stream ends the stream then

    @property
    def queue(self):
        return self.items[self.k + 1:self.k + 7]                    # show.Runner.cues reads the next one

    # --- commands (show.OscBridge / show_stream.Commands) ---------------------
    def trigger(self, name="scan"):
        self.log("%.2f trigger %s ignored: the rehearsal plays its own list" % (self.clock, name))

    def pause(self):
        self.paused = True

    def resume(self):
        if not self.ended:
            self.paused = False

    def end(self):
        """The run's end (show_stream's --minutes): finish the running segment, then stop there."""
        self.ended = self.paused = True

    def fault(self, reason):
        self.state, self.fault_reason = "FAULT", reason
        self.log("%.2f FAULT %s" % (self.clock, reason))

    def reset(self):
        if self.state == "FAULT":
            self.state, self.fault_reason = "HOLD", None

    # --- the clock (show_stream's stream loop) -----------------------------------
    def step(self, dt):
        self.clock += dt
        if self.state in ("FAULT", "HOLD"):
            return self.seg.at(self.seg_t)
        if self.state == "PAUSED":
            if self.paused or self.finished:
                return self.seg.at(self.seg_t)
            self._next()
            return self.seg.at(self.seg_t)
        self.seg_t += dt * (self.scan_speed if self.seg.kind == "scan" else 1.0)
        while self.seg_t >= self.seg.duration and self.state != "PAUSED":
            left = self.seg_t - self.seg.duration
            self.history.append((round(self.clock, 3), self.seg.name))
            self.hub = self.seg.end
            if self.k + 1 >= len(self.items) or self.paused:
                self.finished = self.k + 1 >= len(self.items)
                self.state, self.seg_t = "PAUSED", self.seg.duration
                self.log("%.2f %s at %s after step %d/%d" % (self.clock, "done" if self.finished else "paused",
                                                              self.hub, self.k + 1, len(self.items)))
                break
            self._next()
            self.seg_t = left
        return self.seg.at(self.seg_t)

    def _next(self):
        self.k += 1
        self.seg, self.seg_t = self.items[self.k], 0.0
        self.state = S.STATE_OF[self.seg.kind]
        self.log("%.2f step %d/%d: %s" % (self.clock, self.k + 1, len(self.items), self.seg.name))

    # --- the status (show.OscBridge) -------------------------------------------------
    def plan_left(self):
        """Segment seconds left in the whole rehearsal (the scan at its speed)."""
        if self.finished:
            return 0.0
        now = (self.seg.duration - self.seg_t) / (self.scan_speed if self.seg.kind == "scan" else 1.0)
        return now + sum(wall_s(s, 1.0, self.scan_speed) for s in self.items[self.k + 1:])

    def plan_total(self):
        return sum(wall_s(s, 1.0, self.scan_speed) for s in self.items)

    def status(self, lag_s=0.0, rate=1.0):
        u, led, mps = S.Runner.scan_state(self, lag_s, rate)
        lab = self.seg.labels or {}
        beat, bpm = S.clip_beat(lab, self.seg_t)
        d = self.seg.duration
        nxt = self.items[self.k + 1].name if self.k + 1 < len(self.items) else "nothing: the rehearsal ends here"
        return {"state": self.state, "clip": self.seg.name, "hub": self.hub,
                "sequence": "rehearsal %d/%d" % (self.k + 1, len(self.items)),
                "family": lab.get("family") or "", "action": lab.get("action") or "",
                "clip_t": round(self.seg_t / rate, 3), "clip_len": round(d / rate, 3),
                "beat": round(beat, 4), "bpm_now": round(bpm * rate, 2),
                "progress": round(min(1.0, self.seg_t / d), 4) if d > 0 else 1.0,
                "scan": round(min(1.0, self.seg_t / d), 4) if self.seg.kind == "scan" else -1.0,
                "scan_u": u, "scan_led": led, "scan_mps": mps,
                "time_left": round(max(0.0, d - self.seg_t), 2),
                "next": ("paused at %s (resume to go on)" % self.hub) if self.state == "PAUSED" and not self.finished
                else nxt,
                "queue": [x.name for x in self.queue], "pending": [], "fault": self.fault_reason,
                "energy": 0.0, "clip_energy": round(float(lab.get("energy") or 0.0), 3),
                "plan_left": round(self.plan_left(), 2), "plan_total": round(self.plan_total(), 2),
                "step": self.k + 1, "steps": len(self.items),
                **S.Runner.cues(self, lag_s, rate)}


def mmss(s):
    s = int(round(s))
    return "%d:%02d:%02d" % (s // 3600, s // 60 % 60, s % 60) if s >= 3600 else "%d:%02d" % (s // 60, s % 60)


def table(items, speed, scan_speed=1.0):
    """The plan as lines: step, kind, name, hub(s), its wall time, when it ends."""
    lines, t = [], 0.0
    for i, s in enumerate(items, 1):
        w = wall_s(s, speed, scan_speed)
        t += w
        where = s.start if s.start == s.end else "%s -> %s" % (s.start, s.end)
        lines.append("%3d  %-9s %-28s %-24s %6.1f s  ends %s" % (i, s.kind, s.name, where, w, mmss(t)))
    return lines, t


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    g = S.Graph.load(S.compiled_path(os.path.join(ROOT, "shows", "party.json")))
    items = plan(g)
    names = [s.name for s in items]
    check("every idle clip once", sorted(n for n in names if S.graph_kind(g, n) == "idle")
          == sorted(s.name for s in g.idle()))
    moves = [s.name for s in g.segments if s.kind == "move"]
    check("every move once (the moves between the 4 hubs are a circuit)",
          sorted(n for n in names if n in moves) == sorted(moves), len([n for n in names if n in moves]))
    check("the scan once, in its order", [n for n in names if "scan" in S.graph_kind(g, n)]
          == ["to_scan", "scan", "from_scan"])
    start = g.info.get("start_hub")
    check("it starts and ends at the start hub", items[0].start == start and items[-1].end == start,
          (items[0].start, items[-1].end))
    check("each segment starts where the one before ended (no jump)",
          all(a.end == b.start for a, b in zip(items, items[1:])))
    hub_clips = [s for s in items if s.kind == "idle" and s.start == start]
    energies = [float(s.labels.get("energy") or 0.0) for s in hub_clips]
    check("a hub's clips calm first", energies == sorted(energies))
    _, total = table(items, 0.3)
    check("its length at 0.3: every segment's time / 0.3", abs(total - sum(s.duration for s in items) / 0.3) < 1e-6,
          mmss(total))

    # unbalanced moves (a -> b -> c -> a, b -> a): greedy, still all of them, back home
    A, B, C = [0.0] * 6, [10.0] + [0.0] * 5, [20.0] + [0.0] * 5

    def seg(name, kind, q0, q1, h0, h1, dur=1.0):
        return S.Segment(name, kind, [0.0, dur], [q0, q1], h0, h1, {})
    tri = S.Graph({"a": A, "b": B, "c": C},
                  [seg("ia", "idle", A, A, "a", "a"), seg("ib", "idle", B, B, "b", "b"), seg("ic", "idle", C, C, "c", "c"),
                   seg("ab", "move", A, B, "a", "b"), seg("bc", "move", B, C, "b", "c"), seg("ca", "move", C, A, "c", "a"),
                   seg("ba", "move", B, A, "b", "a")], {"start_hub": "a"})
    ms = [s.name for s in move_circuit(tri, "a")]
    check("unbalanced moves: each at least once, joined, home at the end",
          set(ms) == {"ab", "bc", "ca", "ba"} and all(x[1] == y[0] for x, y in zip(ms, ms[1:]))
          and ms[0][0] == "a" and ms[-1][1] == "a", ms)
    cut = S.Graph({"a": A, "b": B}, [seg("ia", "idle", A, A, "a", "a"), seg("ib", "idle", B, B, "b", "b")],
                  {"start_hub": "a"})
    try:
        plan(cut)
        check("a hub it cannot reach is refused", False)
    except ValueError as e:
        check("a hub it cannot reach is refused", "b" in str(e), e)

    # the runner: plays the list, pause holds at the segment's end, the end PAUSED and finished
    notes = []
    r = Rehearsal(g, log=notes.append)
    dt, prev, worst = 0.008, r.step(0.0), 0.0
    check("it starts at step 1, the first segment's first pose", r.k == 0 and prev == items[0].q[0])
    t_all = r.plan_total()
    check("plan_left at the start is the whole plan", abs(r.status()["plan_left"] - t_all) < 0.05,
          (r.status()["plan_left"], t_all))
    for _ in range(int(20.0 / dt)):
        q = r.step(dt)
        worst = max(worst, max(abs(a - b) for a, b in zip(q, prev)))
        prev = q
    r.pause()
    k = r.k
    for _ in range(int((r.seg.duration + 1.0) / dt)):
        prev = r.step(dt)
    check("pause: held at the running segment's end, PAUSED, not finished", r.state == "PAUSED" and r.k == k
          and not r.finished and prev == r.seg.q[-1], (r.state, r.k, k))
    r.trigger("scan")
    check("a trigger is ignored and said", "ignored" in notes[-1], notes[-1])
    r.resume()
    r.step(dt)
    check("resume: the next step", r.k == k + 1 and r.state != "PAUSED", (r.k, r.state))
    left_before = r.plan_left()
    while not r.finished:
        q = r.step(dt)
        worst = max(worst, max(abs(a - b) for a, b in zip(q, prev)))
        prev = q
    check("it ends PAUSED at the start hub after the last step, ends_when_paused", r.state == "PAUSED"
          and r.ends_when_paused and r.hub == start and prev == items[-1].q[-1] and r.k == len(items) - 1)
    check("every segment played once, in order", [n for _, n in r.history] == names, len(r.history))
    check("no joint jumps more than a tick at the limits", worst < S.max_step(dt), (round(worst, 4),
                                                                                  round(S.max_step(dt), 4)))
    check("plan_left counted down to 0", left_before > 0 and r.plan_left() == 0.0)

    r = Rehearsal(g, start_step=5, log=notes.append)
    check("from step 5: its first pose, status 5/N", r.step(0.0) == items[4].q[0]
          and r.status()["sequence"] == "rehearsal 5/%d" % len(items))
    try:
        Rehearsal(g, start_step=len(items) + 1, log=notes.append)
        check("a step past the end is refused", False)
    except ValueError:
        check("a step past the end is refused", True)
    r = Rehearsal(g, log=notes.append)
    for _ in range(int(3.0 / dt)):
        r.step(dt)
    r.end()
    r.resume()
    while r.state != "PAUSED":
        r.step(dt)
    check("the run's end (--minutes) stops after the running segment, resume does not undo it",
          r.state == "PAUSED" and r.k == 0 and r.paused)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config")
    ap.add_argument("--speed", type=float, default=0.3, help="the show speed it will run at (default 0.3)")
    ap.add_argument("--scan-speed", type=float, default=1.0)
    a = ap.parse_args(argv)
    cfg_path = os.path.abspath(a.config)
    g = S.Graph.load(S.compiled_path(cfg_path))
    S.require_fresh(g, cfg_path)
    items = plan(g)
    lines, total = table(items, a.speed, a.scan_speed)
    print("\n".join(lines))
    kinds = {}
    for s in items:
        kinds[s.kind] = kinds.get(s.kind, 0) + 1
    print("\n%d steps (%s) at speed %g: %s, plus the move to the start hub"
          % (len(items), ", ".join("%d %s" % (n, k) for k, n in sorted(kinds.items())), a.speed, mmss(total)))
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
