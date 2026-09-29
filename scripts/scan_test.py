"""The scan, step by step: for lining the strip up with a real frame and
trying exposures on the paper, before the show plays it on its own.

A runner for show_stream.py (--scan-test) in place of the show's state
machine: it plays only the build's own, checked segments -- to_scan, scan,
from_scan -- one at a time, on a command, and holds still between them:

    at the start pos (the scan's home hub)   to_scan  -> at the scan's start
    at the scan's start (its first frame)    scan [f] -> at the scan's end
                                             back     -> at the start pos (to_scan in reverse)
    at the scan's end                        return   -> at the start pos (from_scan)
    finish (pause, or the run's end)         the way back from wherever it is, then PAUSED

"scan 0.1" plays the scan at 0.1 of its built speed (the exposure); a
command that is not allowed where the arm is, or while it moves, is ignored
and logged. The status is the show's (show.OscBridge): state TO_SCAN while
going to or holding at the scan's start, SCAN while scanning or holding at
its end, FROM_SCAN on the way back, IDLE at the start pos -- so
TouchDesigner's pixel scan follows it as in the show -- and `next` lists
the commands allowed now (a window enables just those).

    uv run scripts/scan_test.py        self-test
    uv run scripts/scan_test_ui.py     the window
"""

import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import show as S  # noqa: E402

HOME, START, END = "home", "scan_start", "scan_end"
STEPS = {HOME: ["to_scan"], START: ["scan", "back"], END: ["return"]}


class ScanTest:
    ends_when_paused = True             # Finish ends the stream (the show's pause only holds at a hub)

    def __init__(self, graph, scan_speed=1.0, log=print):
        segs = {s.name: s for s in graph.segments}
        missing = [n for n in ("to_scan", "scan", "from_scan") if n not in segs]
        if missing:
            raise ValueError("the show has no %s: build it with a scan" % ", ".join(missing))
        self.to_scan, self.scan, self.from_scan = segs["to_scan"], segs["scan"], segs["from_scan"]
        self.back = S.Segment("back_from_scan_start", "move", self.to_scan.t,
                              [list(q) for q in reversed(self.to_scan.q)], START, self.to_scan.start)
        self.home_hub = self.to_scan.start
        self.scan_speed = scan_speed
        self.sel = types.SimpleNamespace(mood=None, energy=None)     # what show_stream's Commands sets
        self.log = log
        self.clock = 0.0
        self.at, self.moving, self.finishing = HOME, False, False
        self.state, self.fault_reason = "IDLE", None
        self.seg, self.seg_t = self.to_scan, 0.0                       # held at its first pose: the start pos
        self.done = []                                                 # the steps played
        self.history = []                                              # (clock, segment) as show.Runner's

    @property
    def hub(self):
        return {HOME: self.home_hub, START: START, END: END}[self.at]

    # --- commands (show.OscBridge / show_stream.Commands) ---------------------
    def allowed(self):
        if self.moving or self.state in ("FAULT", "HOLD", "PAUSED") or self.finishing:
            return []
        return list(STEPS[self.at])

    def trigger(self, name="to_scan"):
        word, _, arg = str(name).partition(" ")
        if word not in self.allowed():
            self.log("%.2f %r ignored: %s" % (self.clock, name, "moving" if self.moving else
                                               "allowed now: %s" % (", ".join(self.allowed()) or "nothing")))
            return
        if word == "scan" and arg:
            f = float(arg)
            if not 0.0 < f <= 1.0:
                self.log("%.2f scan speed %g ignored: 0..1 of the built scan's" % (self.clock, f))
                return
            self.scan_speed = f
        seg = {"to_scan": self.to_scan, "scan": self.scan, "back": self.back, "return": self.from_scan}[word]
        self._play(seg)

    def pause(self):
        """Finish: back to the start pos the way it came, then PAUSED (the
        stream ends there)."""
        self.finishing = True
        if not self.moving:
            self._go_home()

    def resume(self):
        self.finishing = False
        if self.state == "PAUSED":
            self.state = "IDLE"

    def fault(self, reason):
        self.state, self.fault_reason = "FAULT", reason
        self.log("%.2f FAULT %s" % (self.clock, reason))

    def reset(self):
        if self.state == "FAULT":
            self.state, self.fault_reason = "HOLD", None

    # --- the clock (show_stream's stream loop) -----------------------------------
    def step(self, dt):
        self.clock += dt
        if self.moving and self.state not in ("FAULT", "HOLD"):
            self.seg_t += dt * (self.scan_speed if self.seg is self.scan else 1.0)
            if self.seg_t >= self.seg.duration:
                self.seg_t = self.seg.duration
                self.moving = False
                self.at = {self.to_scan: START, self.scan: END}.get(self.seg, HOME)
                self.state = {START: "TO_SCAN", END: "SCAN", HOME: "IDLE"}[self.at]
                self.log("%.2f at the %s" % (self.clock, {HOME: "start pos", START: "scan's start",
                                                         END: "scan's end"}[self.at]))
                if self.finishing:
                    self._go_home()
        return self.seg.at(self.seg_t)

    def _play(self, seg):
        self.seg, self.seg_t, self.moving = seg, 0.0, True
        self.history.append((round(self.clock, 3), seg.name))
        self.state = {"to_scan": "TO_SCAN", "scan": "SCAN"}.get(seg.kind, "FROM_SCAN")
        self.done.append(seg.name)
        self.log("%.2f %s%s" % (self.clock, seg.name, (" at %.2f of its speed" % self.scan_speed)
                                if seg is self.scan else ""))

    def _go_home(self):
        if self.at == START:
            self._play(self.back)
        elif self.at == END:
            self._play(self.from_scan)
        else:
            self.state = "PAUSED"

    # --- the status (show.OscBridge) -------------------------------------------------
    def status(self, lag_s=0.0, rate=1.0):
        u, led, mps = S.Runner.scan_state(self, lag_s, rate)
        d = self.seg.duration
        where = self.hub
        return {"state": self.state, "clip": self.seg.name if self.moving else "holding at %s" % where,
                "hub": where, "sequence": "scan test",
                "progress": round(min(1.0, self.seg_t / d), 4) if d > 0 else 1.0,
                "scan": round(min(1.0, self.seg_t / d), 4) if self.seg is self.scan else -1.0,
                "scan_u": u, "scan_led": led, "scan_mps": mps,
                "time_left": round(max(0.0, d - self.seg_t) / (self.scan_speed if self.seg is self.scan else 1.0), 2)
                if self.moving else 0.0,
                "next": ",".join(self.allowed()), "queue": [], "pending": [],
                "fault": self.fault_reason, "energy": 0.0, "clip_energy": 0.0}


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    g = S.Graph.load(S.compiled_path(os.path.join(ROOT, "shows", "party.json")))
    notes = []
    r = ScanTest(g, log=notes.append)
    dt = 0.008

    def run(seconds):
        qs = [r.step(dt) for _ in range(int(seconds / dt))]
        return qs

    def jump(qs, prev):
        return max(max(abs(a - b) for a, b in zip(x, y)) for x, y in zip([prev] + qs[:-1], qs))

    home = r.step(0.0)
    check("it starts holding at the start pos (to_scan's first pose), idle, to_scan allowed",
          home == r.to_scan.q[0] and r.state == "IDLE" and r.status()["next"] == "to_scan", r.status()["next"])
    run(1.0)
    check("it holds still until told", r.step(dt) == home)
    r.trigger("scan")
    check("a step not allowed where it is is ignored and said", not r.moving and "ignored" in notes[-1], notes[-1])
    r.trigger("to_scan")
    check("to_scan: TO_SCAN, moving, nothing allowed meanwhile", r.moving and r.state == "TO_SCAN"
          and r.status()["next"] == "", r.status())
    qs = run(r.to_scan.duration + 0.5)
    check("... then held at the scan's first frame, scan and back allowed, still TO_SCAN (TD keeps the LEDs "
          "ready, off: u -1)", qs[-1] == r.scan.q[0] and r.at == START and r.status()["next"] == "scan,back"
          and r.state == "TO_SCAN" and r.status()["scan_u"] == -1.0, (r.at, r.status()["next"]))
    r.trigger("scan 0.5")
    check("scan at half its speed: SCAN, the LEDs' u moving across", r.moving and r.scan_speed == 0.5
          and r.state == "SCAN")
    t_scan = r.scan.duration / 0.5
    qs = run(t_scan * 0.5)
    mid = r.status()
    check("... halfway through at half speed: u near 0.5, the LEDs on, the strip at half the built m/s, the time "
          "left at that speed", 0.35 < mid["scan_u"] < 0.65 and mid["scan_led"] == 1 and abs(mid["scan_mps"] - 0.1) < 0.01
          and abs(mid["time_left"] - t_scan / 2) < 0.05, mid)
    qs += run(t_scan * 0.5 + 0.5)
    check("... then held at its end, return allowed", qs[-1] == r.scan.q[-1] and r.at == END
          and r.status()["next"] == "return", r.status()["next"])
    check("no joint jumps more than the scan's own steps", jump(qs, r.scan.q[0]) < 0.5, jump(qs, r.scan.q[0]))
    r.trigger("return")
    qs = run(r.from_scan.duration + 0.5)
    check("return: FROM_SCAN, then IDLE at the start pos", qs[-1] == r.from_scan.q[-1] and r.at == HOME
          and r.state == "IDLE", r.state)
    r.trigger("to_scan")
    run(r.to_scan.duration + 0.5)
    r.trigger("back")
    check("back from the scan's start: to_scan in reverse", r.state == "FROM_SCAN" and r.seg is r.back)
    qs = run(r.to_scan.duration + 0.5)
    check("... to the start pos", qs[-1] == home and r.at == HOME)
    r.trigger("to_scan")
    run(1.0)
    r.pause()
    check("finish while moving: nothing new until at the scan's start", r.seg is r.to_scan and r.moving)
    qs = run(2 * r.to_scan.duration + 1.0)
    check("... then the way back and PAUSED at the start pos (the stream ends)", qs[-1] == home
          and r.state == "PAUSED" and r.status()["next"] == "", (r.state, r.done[-3:]))
    check("every step joined at rest: first and last steps of each segment tiny",
          all(max(abs(a - b) for a, b in zip(s.q[0], s.q[1])) < 0.01 for s in (r.to_scan, r.scan, r.from_scan, r.back)))
    r = ScanTest(g, log=notes.append)
    r.trigger("to_scan")
    run(r.to_scan.duration + 0.5)
    r.trigger("scan 2")
    check("a scan speed over 1 is refused, the arm stays at the scan's start", not r.moving and r.at == START
          and r.scan_speed == 1.0 and "refused" not in notes[-1] and "ignored" in notes[-1], notes[-1])
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
