"""A show: the arm plays through a clip library, switching on outside events
(TouchDesigner over OSC), always between validated motions.

    python scripts/show.py build shows/party.json          compile: every motion made and checked
    python scripts/show.py dry-run shows/party.json [--minutes 10] [--trigger-every 45]
    python scripts/show.py osc shows/party.json            dry-run clock, driven by OSC (TouchDesigner)
    python scripts/show.py --self-test

The motion graph (the usual motion-graph / hub idea: every motion starts and
ends at rest at a hub, so any two join without a jump):

    home --idle clip--> home          the generated library (tests/csv/stage), turned to face the stage
    home --to_scan----> scan_start    a checked MoveJ-like move (safe_move.route), timed by retime_topp
    scan_start --scan-> scan_end      the fixed sweep across the paper (a PLACEHOLDER line for now)
    scan_end --from_scan--> home

Everything is made and checked at BUILD time, against the room (collision.py,
the lab env, the controller's work area) plus the canvas: idle clips keep
idle_canvas_m from the paper, the scan and its moves scan_canvas_m. At show
time nothing is planned, only chosen: the runner walks the graph.

Runner states: IDLE -> (trigger) -> TO_SCAN -> SCAN -> FROM_SCAN -> IDLE;
PAUSED (holds at home); FAULT (holds, needs reset). A trigger is taken at the
end of the running clip (the worst wait is the longest idle clip,
select.max_idle_s). Selection: weighted random, no repeat within the last
select.no_repeat, weighted towards a mood (a Laban action) and an energy
(0 calm .. 1 lively) that TouchDesigner may send.

OSC in:  /robot/trigger [name]   /robot/pause   /robot/resume   /robot/reset
         /robot/mood <action>    /robot/energy <0..1>
OSC out: /robot/state s  /robot/clip s  /robot/progress f  /robot/scan f (0..1 while
         scanning: the LED strip's column clock)  /robot/joints f*6

Backends (who moves): the dry-run clock here; Isaac Sim
(scripts/isaac/run_show.py); SimMachine / the real FR20 next (a ServoJ
stream over the same step()).
"""

import argparse
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

SCHEMA = "motionlab.show.compiled/1"
STEP_DEG = 1.0                      # joint-space sampling of a planned move


# --------------------------------------------------------------------------
# segments and the graph (pure data once compiled)
# --------------------------------------------------------------------------

class Segment:
    def __init__(self, name, kind, t, q, start, end, labels=None):
        self.name, self.kind, self.t, self.q = name, kind, list(t), [list(x) for x in q]
        self.start, self.end, self.labels = start, end, labels or {}

    @property
    def duration(self):
        return self.t[-1]

    def at(self, s):
        """Joints at time s (linear between samples, held at the ends)."""
        t, q = self.t, self.q
        if s <= t[0]:
            return list(q[0])
        if s >= t[-1]:
            return list(q[-1])
        lo, hi = 0, len(t) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if t[mid] <= s:
                lo = mid
            else:
                hi = mid
        f = (s - t[lo]) / (t[hi] - t[lo])
        return [a + f * (b - a) for a, b in zip(q[lo], q[hi])]

    def to_dict(self):
        return {"name": self.name, "kind": self.kind, "start": self.start, "end": self.end,
                "labels": self.labels, "t": [round(x, 5) for x in self.t],
                "q": [[round(v, 5) for v in x] for x in self.q]}

    @staticmethod
    def from_dict(d):
        return Segment(d["name"], d["kind"], d["t"], d["q"], d["start"], d["end"], d.get("labels"))


class Graph:
    def __init__(self, hubs, segments, info=None):
        self.hubs, self.segments, self.info = hubs, segments, info or {}

    def idle(self):
        return [s for s in self.segments if s.kind == "idle"]

    def one(self, kind):
        return [s for s in self.segments if s.kind == kind][0]

    def save(self, path):
        with open(path, "w") as f:
            json.dump({"schema": SCHEMA, "info": self.info, "hubs": self.hubs,
                       "segments": [s.to_dict() for s in self.segments]}, f)

    @staticmethod
    def load(path):
        d = json.load(open(path))
        return Graph(d["hubs"], [Segment.from_dict(s) for s in d["segments"]], d.get("info"))

    def check_joins(self, tol_deg=0.05):
        """[(segment, 'start'|'end', hub, off deg)] where a segment does not
        start / end exactly at its hub."""
        bad = []
        for s in self.segments:
            for which, q, hub in (("start", s.q[0], s.start), ("end", s.q[-1], s.end)):
                off = max(abs(a - b) for a, b in zip(q, self.hubs[hub]))
                if off > tol_deg:
                    bad.append((s.name, which, hub, off))
        return bad


# --------------------------------------------------------------------------
# build: make and check every motion (heavy imports only here)
# --------------------------------------------------------------------------

def canvas_box(c):
    """The paper as a thin box for collision.py (yaw from its normal)."""
    n = c["normal"]
    yaw = math.degrees(math.atan2(n[1], n[0]))
    return {"name": "canvas", "type": "box", "center": list(c["center"]), "role": "obstacle",
            "size": [c.get("thickness", 0.02), c["size"][0], c["size"][1]], "yaw_deg": yaw}


def _env_with_canvas(env, cfg, margin):
    box = dict(canvas_box(cfg["canvas"]), margin_m=margin)
    return dict(env, objects=list(env["objects"]) + [box])


def timed_move(waypoints, safety, vel, acc):
    """A joint-space path through waypoints (straight in joint space per leg,
    as the controller's MoveJ), at rest at both ends, timed by retime_topp."""
    import retime_topp as T
    q = [list(waypoints[0])]
    for a, b in zip(waypoints, waypoints[1:]):
        n = max(2, int(math.ceil(max(abs(x - y) for x, y in zip(a, b)) / STEP_DEG)))
        q += [[x + (y - x) * k / float(n) for x, y in zip(a, b)] for k in range(1, n + 1)]
    t = T.plan(q, [v * safety for v in vel], [a_ * safety for a_ in acc])
    return t, q


def build(cfg_path, log=print):
    import collision as C
    import robot_profile as RP
    import safe_move
    cfg = json.load(open(cfg_path))
    prof = RP.load("fr20")
    vel, acc = RP.velocity_limits(prof), RP.acceleration_limits(prof)
    model = C.load_model("fr20")
    env = C.load_env(os.path.join(ROOT, cfg["env"]))
    hubs = {k: list(v) for k, v in cfg["hubs"].items()}
    home = hubs["home"]
    segs, dropped = [], []

    # idle: the stage library (every clip home -> home), kept off the paper
    idle_env = _env_with_canvas(env, cfg, cfg["margins"]["idle_canvas_m"])
    src = os.path.join(ROOT, cfg["idle_manifest"])
    max_s = cfg["select"].get("max_idle_s", 1e9)
    for f in sorted(os.listdir(src)):
        if not f.endswith(".json") or f == "manifest.json":
            continue
        c = json.load(open(os.path.join(src, f)))
        if not (c.get("safety") or {}).get("ok") or not c.get("points"):
            continue
        t, q = [p["t"] for p in c["points"]], [p["q"] for p in c["points"]]
        if t[-1] > max_s:
            dropped.append((c["id"], "longer than %.0f s (trigger latency)" % max_s))
            continue
        rep = C.check(model, idle_env, t, q)
        if not rep["ok"]:
            dropped.append((c["id"], C.describe(rep)))
            continue
        m = (c.get("labels") or {}).get("measured") or {}
        labels = {"action": m.get("action"), "effort": {k: m.get(k) for k in ("weight", "time", "space", "flow")},
                  "intent": (c.get("style") or {}).get("bars") and [b.get("action") for b in c["style"]["bars"]]}
        segs.append(Segment(c["id"], "idle", t, q, "home", "home", labels))
    log("idle: %d clips (%d dropped)" % (len(segs), len(dropped)))

    # scan: a PLACEHOLDER straight sweep in front of the canvas, tool at the paper
    scan_env = _env_with_canvas(env, cfg, cfg["margins"]["scan_canvas_m"])
    scan = placeholder_scan(cfg, scan_env)
    if not scan["safety"]["ok"]:
        raise SystemExit("scan: %s" % scan["safety"]["reasons"])
    st, sq = [p["t"] for p in scan["points"]], [p["q"] for p in scan["points"]]
    hubs["scan_start"], hubs["scan_end"] = list(sq[0]), list(sq[-1])
    segs.append(Segment("scan", "scan", st, sq, "scan_start", "scan_end"))

    # the moves joining them: planned once, checked, timed
    for name, kind, a, b in (("to_scan", "to_scan", "home", "scan_start"),
                             ("from_scan", "from_scan", "scan_end", "home")):
        path, why = safe_move.route(hubs[a], hubs[b], scan_env, model)
        if path is None:
            raise SystemExit("%s: %s" % (name, why))
        t, q = timed_move([hubs[a]] + path, cfg["transition_safety"], vel, acc)
        rep = C.check(model, scan_env, t, q)
        if not rep["ok"]:
            raise SystemExit("%s: %s" % (name, C.describe(rep)))
        segs.append(Segment(name, kind, t, q, a, b))
        log("%s: %.1f s, %s" % (name, t[-1], why))
    g = Graph(hubs, segs, {"config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"), "built": time.strftime("%Y-%m-%d %H:%M"),
                           "dropped": [{"id": i, "why": w} for i, w in dropped], "canvas": cfg["canvas"]})
    bad = g.check_joins()
    if bad:
        raise SystemExit("segments do not meet at their hubs: %s" % bad)
    return g


def placeholder_scan(cfg, env):
    """A straight line along the canvas at the standoff, the tool pointing at
    the paper (clip_factory's pipeline: reach, IK, TOPP, room check)."""
    import clip_factory
    c, s = cfg["canvas"], cfg["scan"]
    n = c["normal"]
    centre = [c["center"][i] - n[i] * s["standoff_m"] for i in range(3)]
    v = {"id": "scan", "primitive": "line", "center": centre, "size": s["length_m"] / 2.0,
         "plane": "xy", "tool": list(n), "safety": s["safety"], "tags": ["scan", "placeholder"]}
    return clip_factory.make(v, env=env)


def compiled_path(cfg_path):
    return os.path.splitext(cfg_path)[0] + ".compiled.json"


# --------------------------------------------------------------------------
# choosing and running
# --------------------------------------------------------------------------

class Selector:
    """Next idle clip: weighted random, no repeat within the last
    no_repeat, weighted towards a mood (Laban action) and an energy."""

    def __init__(self, clips, no_repeat=8, seed=None):
        self.clips, self.no_repeat = clips, no_repeat
        self.recent, self.mood, self.energy = [], None, None
        self.rng = random.Random(seed)

    def weight(self, c):
        w = 1.0
        if self.mood and c.labels.get("action") == self.mood:
            w *= 4.0
        e = (c.labels.get("effort") or {})
        if self.energy is not None and e.get("time") is not None:
            lively = (e["time"] + 1.0) / 2.0                  # sudden = lively
            w *= 0.25 + 1.5 * (1.0 - abs(lively - self.energy))
        return w

    def pick(self):
        pool = [c for c in self.clips if c.name not in self.recent[-self.no_repeat:]] or self.clips
        ws = [self.weight(c) for c in pool]
        c = self.rng.choices(pool, weights=ws)[0]
        self.recent.append(c.name)
        return c


class Runner:
    """Walks the graph one step at a time: step(dt) -> joints (degrees)."""

    def __init__(self, graph, selector, log=None):
        self.g, self.sel, self.log = graph, selector, log or (lambda *a: None)
        self.state, self.pending, self.paused, self.fault_reason = "IDLE", [], False, None
        self.clock, self.seg_t = 0.0, 0.0
        self.seg = self._next_idle()
        self.history = []

    # events (from OSC, a keyboard, a test script)
    def trigger(self, name="scan"):
        if name not in self.pending:
            self.pending.append(name)
            self.log("%.2f trigger %s (in %s, %.1f s left)" % (self.clock, name, self.seg.name, self.seg.duration - self.seg_t))

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def fault(self, reason):
        self.state, self.fault_reason = "FAULT", reason
        self.log("%.2f FAULT %s" % (self.clock, reason))

    def reset(self):
        if self.state == "FAULT":
            self.state, self.fault_reason = "HOLD", None

    def _next_idle(self):
        s = self.sel.pick()
        self.state = "IDLE"
        return s

    def _advance(self):
        k = self.seg.kind
        self.history.append((round(self.clock, 3), self.seg.name))
        if k == "idle" and "scan" in self.pending:
            self.pending.remove("scan")
            nxt, self.state = self.g.one("to_scan"), "TO_SCAN"
        elif k == "to_scan":
            nxt, self.state = self.g.one("scan"), "SCAN"
        elif k == "scan":
            nxt, self.state = self.g.one("from_scan"), "FROM_SCAN"
        elif self.paused:
            self.state = "PAUSED"
            return
        else:
            nxt = self._next_idle()
        self.log("%.2f %s -> %s (%s)" % (self.clock, self.seg.name, nxt.name, self.state))
        self.seg, self.seg_t = nxt, 0.0

    def step(self, dt):
        self.clock += dt
        if self.state in ("FAULT", "HOLD"):
            return self.seg.at(self.seg_t)
        if self.state == "PAUSED":
            if self.paused and "scan" not in self.pending:
                return self.seg.at(self.seg_t)
            self.state = "IDLE"
            self.seg_t = self.seg.duration
            self._advance()
            return self.seg.at(self.seg_t)
        self.seg_t += dt
        while self.seg_t >= self.seg.duration and self.state not in ("PAUSED",):
            left = self.seg_t - self.seg.duration
            self._advance()
            if self.state == "PAUSED":
                self.seg_t = self.seg.duration
                break
            self.seg_t = left
        return self.seg.at(self.seg_t)

    def status(self):
        prog = self.seg_t / self.seg.duration if self.seg.duration > 0 else 1.0
        return {"state": self.state, "clip": self.seg.name, "progress": round(min(1.0, prog), 4),
                "scan": round(min(1.0, prog), 4) if self.seg.kind == "scan" else -1.0,
                "pending": list(self.pending), "fault": self.fault_reason}


# --------------------------------------------------------------------------
# OSC (TouchDesigner)
# --------------------------------------------------------------------------

class OscBridge:
    """/robot/* in and out; python-osc (the usual TouchDesigner link)."""

    def __init__(self, runner, listen_port, send_host, send_port):
        from pythonosc import dispatcher, osc_server, udp_client
        import threading
        self.runner = runner
        d = dispatcher.Dispatcher()
        d.map("/robot/trigger", lambda a, *v: runner.trigger(str(v[0]) if v else "scan"))
        d.map("/robot/pause", lambda a, *v: runner.pause())
        d.map("/robot/resume", lambda a, *v: runner.resume())
        d.map("/robot/reset", lambda a, *v: runner.reset())
        d.map("/robot/mood", lambda a, *v: setattr(runner.sel, "mood", str(v[0]) if v and v[0] else None))
        d.map("/robot/energy", lambda a, *v: setattr(runner.sel, "energy", float(v[0]) if v else None))
        self.server = osc_server.ThreadingOSCUDPServer(("0.0.0.0", listen_port), d)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = udp_client.SimpleUDPClient(send_host, send_port)

    def send(self, q):
        s = self.runner.status()
        self.client.send_message("/robot/state", s["state"])
        self.client.send_message("/robot/clip", s["clip"])
        self.client.send_message("/robot/progress", float(s["progress"]))
        self.client.send_message("/robot/scan", float(s["scan"]))
        self.client.send_message("/robot/joints", [float(x) for x in q])

    def close(self):
        self.server.shutdown()


# --------------------------------------------------------------------------
# dry run
# --------------------------------------------------------------------------

def dry_run(graph, minutes=10.0, trigger_every=45.0, dt=0.008, seed=1, log=print):
    """The show on a clock, no robot: triggers every trigger_every s (with
    jitter). Returns a report: continuity (largest joint step per tick),
    trigger latencies, clips played."""
    sel = Selector(graph.idle(), no_repeat=8, seed=seed)
    r = Runner(graph, sel, log=log)
    rng = random.Random(seed)
    next_trig = trigger_every * (0.5 + rng.random())
    prev, worst, latencies, t_trig = r.step(0.0), 0.0, [], None
    t, end = 0.0, minutes * 60.0
    while t < end:
        t += dt
        if t >= next_trig:
            r.trigger("scan")
            t_trig = t
            next_trig = t + trigger_every * (0.5 + rng.random())
        q = r.step(dt)
        worst = max(worst, max(abs(a - b) for a, b in zip(q, prev)))
        prev = q
        if t_trig is not None and r.state == "SCAN":
            latencies.append(t - t_trig)
            t_trig = None
    played = [n for _, n in r.history]
    return {"minutes": minutes, "clips_played": len(played), "scans": played.count("scan"),
            "distinct_idle": len(set(n for n in played if n not in ("scan", "to_scan", "from_scan"))),
            "worst_step_deg_per_tick": round(worst, 4), "max_step_allowed": round(max_step(dt), 4),
            "trigger_to_scan_s": {"max": round(max(latencies), 2) if latencies else None,
                                  "mean": round(sum(latencies) / len(latencies), 2) if latencies else None}}


def max_step(dt):
    """The largest joint step one tick may take at the profile's velocity limits."""
    import robot_profile as RP
    return max(RP.velocity_limits(RP.load("fr20"))) * dt


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    home = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    a = [10.0] + home[1:]
    b = [20.0] + home[1:]

    def seg(name, kind, q0, q1, start, end, dur=2.0):
        return Segment(name, kind, [0.0, dur / 2, dur], [q0, [(x + y) / 2 for x, y in zip(q0, q1)], q1], start, end,
                       {"action": "punch" if name == "i1" else "float", "effort": {"time": 1.0 if name == "i1" else -1.0}})

    g = Graph({"home": home, "scan_start": a, "scan_end": b},
              [seg("i1", "idle", home, home, "home", "home", 3.0), seg("i2", "idle", home, home, "home", "home", 3.0),
               seg("to_scan", "to_scan", home, a, "home", "scan_start"), seg("scan", "scan", a, b, "scan_start", "scan_end"),
               seg("from_scan", "from_scan", b, home, "scan_end", "home")])
    check("segments meet at their hubs", not g.check_joins())
    r = Runner(g, Selector(g.idle(), no_repeat=1, seed=0))
    r.step(1.0)
    r.trigger("scan")
    states = []
    for _ in range(200):
        r.step(0.05)
        states.append(r.state)
    order = [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]]
    check("a trigger waits for the clip, then to_scan -> scan -> from_scan -> idle",
          order[:4] == ["IDLE", "TO_SCAN", "SCAN", "FROM_SCAN"] and "IDLE" in order[4:], order)
    r.pause()
    for _ in range(200):
        r.step(0.05)
    check("pause holds at home after the running clip", r.state == "PAUSED" and r.step(0.05) == home, r.state)
    r.resume()
    r.step(0.05)
    check("resume goes on idling", r.state == "IDLE", r.state)
    r.fault("test")
    q = r.step(0.5)
    check("a fault holds the pose", r.state == "FAULT" and q == r.step(0.5))
    sel = Selector(g.idle(), no_repeat=1, seed=3)
    picks = [sel.pick().name for _ in range(50)]
    check("no repeat within the window", all(x != y for x, y in zip(picks, picks[1:])), picks[:6])
    sel = Selector(g.idle(), no_repeat=0, seed=3)
    sel.mood = "punch"
    picks = [sel.pick().name for _ in range(400)]
    check("a mood makes its clips likelier", picks.count("i1") > 2 * picks.count("i2"), (picks.count("i1"), picks.count("i2")))
    sel = Selector(g.idle(), no_repeat=0, seed=3)
    sel.energy = 1.0
    picks = [sel.pick().name for _ in range(400)]
    check("energy 1 prefers sudden clips", picks.count("i1") > 2 * picks.count("i2"), (picks.count("i1"), picks.count("i2")))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=("build", "dry-run", "osc"))
    ap.add_argument("config")
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--trigger-every", type=float, default=45.0)
    a = ap.parse_args(argv)
    cfg_path = os.path.abspath(a.config)
    if a.command == "build":
        g = build(cfg_path)
        g.save(compiled_path(cfg_path))
        print("wrote %s: %d segments, hubs %s" % (os.path.relpath(compiled_path(cfg_path), ROOT), len(g.segments), list(g.hubs)))
        return 0
    g = Graph.load(compiled_path(cfg_path))
    if a.command == "dry-run":
        rep = dry_run(g, a.minutes, a.trigger_every, log=lambda *x: None)
        print(json.dumps(rep, indent=1))
        return 0 if rep["worst_step_deg_per_tick"] <= rep["max_step_allowed"] else 1
    cfg = json.load(open(cfg_path))
    r = Runner(g, Selector(g.idle(), cfg["select"]["no_repeat"]), log=print)
    o = cfg["osc"]
    bridge = OscBridge(r, o["listen_port"], o["send_host"], o["send_port"])
    print("OSC in :%d, out %s:%d -- Ctrl+C to stop" % (o["listen_port"], o["send_host"], o["send_port"]))
    try:
        while True:
            q = r.step(1.0 / 30)
            bridge.send(q)
            time.sleep(1.0 / 30)
    except KeyboardInterrupt:
        bridge.close()
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
