"""The interactive mode on its own -- not the party show: the arm plays the
v9 library's idle clips at random, facing the guests less often than the
show does, and when somebody stands on the spot in front of greet (or waves
within 1.2 m of the glass) it comes to greet and turns to them (engage.py:
perk up, follow, nod goodbye), then plays on. End: the people are let go,
the clip playing finishes at a hub, the stream ends and the arm goes back
to the start pose.

    python scripts/track_mode.py --self-test
    python scripts/track_mode.py --sim --ip 192.168.116.128 --engage-share 0.35
    python scripts/track_mode.py --hardware --ip IP --speed 0.3            # asks before every move
    uv run scripts/track_ui.py                                             # the window for it

The people come from TouchDesigner's people_track (OSC, port 9011: the
Femto, or a recording). The stream is show_stream's (ServoJ 125 Hz, its
Guard every tick, the feedback thread, the fault path); the start and the
return are its checked MoveJ (fairino_player.move_checked). Commands on
stdin, one a line (track_ui sends them): `end` (finish and go back to the
start pose), `stop` (a software stop now). Status on stdout: `[status] {json}`
twice a second.

What plays:
- IDLE: show.Runner over the library's idle clips (no scan, no big wipes,
  nothing triggered), with AwaySelector: a clip's weight falls with how
  squarely its LEDs face the guests (show.facing, its mean), and a hub's
  with its clips' (so rest, facing the paper, gets more of the time);
- CALL: somebody is a candidate (Engage.candidate): the queue is dropped,
  the clip playing finishes at its hub and the Runner goes to greet and
  waits there (a checked route; clips end at their hub at rest);
- WAIT: at greet, at rest, Engage steps from the hub pose; nobody after
  WAIT_S: back to IDLE;
- ENGAGE: Engage's own states (its plans checked before each move); at its
  RETURN it is back at the hub pose and the Runner goes on from there.
Clips play at --speed (time scaled); the interactive mode at its own share
of the joints' limits (--engage-share). A late tick is never a jump: the
show clock slips a tick instead of skipping ahead (see the safety audit).
"""

import argparse
import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import show as S  # noqa: E402

GREET = "greet"
WAIT_S = 4.0                  # at greet with nobody to engage this long: play on
FACING_FLOOR = 0.25           # a clip's weight x (FLOOR + (1 - FLOOR) (1 - facing)): squarely facing -> x 0.25
HUB_FLOOR = 0.3               # a hub's pick weight: FLOOR + (1 - its clips' mean facing) x 1.7
STATUS_S = 0.5
PORT = 9011


def clip_facing(seg, eyes, every=6):
    """How squarely a clip's LEDs face the guests, its mean over the clip (show.facing: 0 away .. 1 at them)."""
    qs = seg.q[::every] or seg.q
    return sum(S.facing(q, eyes) for q in qs) / len(qs)


class AwaySelector(S.Selector):
    """show.Selector with a clip's weight lowered the more it faces the guests: the arm turning to someone
    is then the more striking."""

    def __init__(self, clips, facing, no_repeat=6, seed=None, arc=None):
        super().__init__(clips, no_repeat, seed, arc)
        self.facing = facing

    def weight(self, c, mood=None, clock=0.0):
        f = self.facing.get(c.name, 0.0)
        return super().weight(c, mood, clock) * (FACING_FLOOR + (1.0 - FACING_FLOOR) * (1.0 - f))


class AwayRunner(S.Runner):
    """show.Runner whose moves between hubs favour the hubs whose clips face the guests least. Nothing
    else changed: the same checked clips and routes."""

    def __init__(self, graph, selector, hub_weight, **kw):
        super().__init__(graph, selector, **kw)
        self.hub_weight = hub_weight

    def _plan_next(self):
        if self.pending:
            return super()._plan_next()
        self.sequence = None
        hubs = self.g.idle_hubs()
        self.stay -= 1
        if self.stay <= 0 and len(hubs) > 1:
            others = [h for h in hubs if h != self.hub and self.g.route(self.hub, h)]
            if others:
                other = self.rng.choices(others, weights=[self.hub_weight.get(h, 1.0) for h in others])[0]
                self.queue += self.g.route(self.hub, other)
                lo, hi = self.hub_stay
                self.stay = self.rng.randint(lo, hi)
                return
        self.queue.append(self.sel.pick(self.hub, clock=self.clock))


def away_runner(graph, cfg, seed=None):
    """The mode's Runner: the library's idle clips, no showpieces, AwaySelector and the hubs weighted."""
    eyes = S.audience_eyes(cfg)
    clips = [c for c in graph.idle() if "showpiece" not in (c.labels.get("intent") or [])]
    facing = {c.name: clip_facing(c, eyes) for c in clips}
    hub_weight = {}
    for h in graph.idle_hubs():
        fs = [facing[c.name] for c in clips if c.start == h]
        hub_weight[h] = HUB_FLOOR + 1.7 * (1.0 - (sum(fs) / len(fs) if fs else 1.0))
    sel = graph.info.get("select", {})
    r = AwayRunner(graph, AwaySelector(clips, facing, sel.get("no_repeat", 6), seed=seed, arc=sel.get("arc")),
                   hub_weight, hub_stay=tuple(sel.get("hub_stay", (2, 4))), seed=seed, sequences={})
    r.sequences = {"__to_greet": {"hub": GREET, "count": 0}}       # CALL's route, never an OSC trigger here
    return r, facing, hub_weight


class TrackMode:
    """show_stream's runner (show.Runner's face: step, pause, fault, clock, state, history, sel) for the
    interactive mode: IDLE / CALL / WAIT / ENGAGE over an AwayRunner and an Engage, the people from rx
    (poll() -> (people, hands) | None, robot frame)."""

    ends_when_paused = False
    scan_speed = 1.0

    def __init__(self, runner, engage, rx, dt, clip_speed=1.0, out=print):
        self.r, self.en, self.rx, self.dt, self.clip_speed = runner, engage, rx, dt, clip_speed
        self.sel = runner.sel
        self.hub = runner.hub
        self.clock, self.state, self.mode = 0.0, "PLAYING", "IDLE"
        self.log, self.out = out, out
        self.ending, self.t_wait, self.frames, self.calls = False, 0.0, 0, 0
        self.q = list(runner.step(0.0))
        self.slips = 0
        self._last_status = -1.0

    @property
    def history(self):
        return self.r.history

    # the stream's commands (Commands.apply, between ticks)
    def pause(self):
        """The end: nobody fed from now on (Engage says goodbye), the Runner finishes at a hub."""
        if not self.ending:
            self.ending = True
            self.ends_when_paused = True
            self.r.pause()
            self.log("%.2f end asked (%s)" % (self.clock, self.mode))

    def resume(self):
        pass                                          # the end is latched: nothing resumes it

    def trigger(self, name="scan"):
        pass                                          # nothing is triggered in this mode

    def reset(self):
        pass

    def fault(self, why):
        self.state = "FAULT"
        self.log("%.2f FAULT: %s" % (self.clock, why))

    # ------------------------------------------------------------------
    def step(self, adv):
        """The pose for the next point. A late stream asks for several ticks at once; the mode moves one
        (the clock slips, never a jump)."""
        if adv > 0.0:
            if adv > 1.5 * self.dt:
                self.slips += int(round(adv / self.dt)) - 1
            self._tick()
        return list(self.q)

    def _tick(self):
        self.clock += self.dt
        now = self.clock
        got = self.rx.poll() if self.rx is not None else None
        if got is not None and not self.ending:
            people, hands = got
            self.en.update([tuple(p[:5]) + (now,) for p in people], now, [tuple(h[:5]) + (now,) for h in hands])
            self.frames += 1
        elif self.ending and self.en.state != "OFF":
            self.en.update([], now, [])
        cdt = self.dt * self.clip_speed
        if self.state in ("FAULT", "PAUSED"):
            return
        if self.mode == "IDLE":
            self.q = self.r.step(cdt)
            if not self.ending and self.en.candidate(now) is not None:
                self._call()
        elif self.mode == "CALL":
            self.q = self.r.step(cdt)
            if self.r.state == "PAUSED":
                if self.r.hub == GREET and not self.ending:
                    self.mode, self.t_wait = "WAIT", now
                    self.log("%.2f at greet, waiting" % now)
                elif not self.ending:
                    self.r.resume()
                    self.mode = "IDLE"
        elif self.mode == "WAIT":
            q_hub = self.r.step(cdt)                      # paused at greet: the hub pose, still
            if self.ending:
                self.q = q_hub
            else:
                self.q = self.en.step(q_hub, now)
                if self.en.state != "OFF":
                    self.mode = "ENGAGE"
                    self.log("%.2f engaged person %s" % (now, self.en.who))
                elif now - self.t_wait > WAIT_S:
                    self.r.resume()
                    self.mode = "IDLE"
                    self.log("%.2f nobody at greet: playing on" % now)
        elif self.mode == "ENGAGE":
            self.q = self.en.step(self.q, now)
            if self.en.resume:
                self.log("%.2f back at greet (%d engaged so far)" % (now, self.en.engagements))
                if not self.ending:
                    self.r.resume()
                self.mode = "IDLE"
        if self.ending and self.r.state == "PAUSED" and self.en.state == "OFF" and self.mode in ("IDLE", "CALL",
                                                                                              "WAIT"):
            self.state = "PAUSED"
            self.log("%.2f ended at %s" % (now, self.r.hub))
        if now - self._last_status >= STATUS_S:
            self._last_status = now
            self.out("[status] " + json.dumps(self.status()))

    def _call(self):
        """Someone to engage while the arm is elsewhere: to greet once the clip playing ends."""
        self.calls += 1
        self.r.queue.clear()
        if self.r.seg.end == GREET:
            self.r.pause()
        else:
            self.r.trigger("__to_greet")
            self.r.pause()
        self.mode = "CALL"
        self.log("%.2f somebody to engage: to greet after %s" % (self.clock, self.r.seg.name))

    def status(self):
        en = self.en
        return {"t": round(self.clock, 1), "mode": self.mode, "state": self.state, "clip": self.r.seg.name,
                "hub": self.r.hub, "engage": en.state, "who": en.who, "engaged": en.engagements,
                "refused": en.refused, "unsafe": en.unsafe, "people": len([p for p in en.people.values()
                                                                          if self.clock - p["last"] < 0.5]),
                "frames": self.frames, "calls": self.calls, "slips": self.slips, "ending": self.ending}


# ---------------------------------------------------------------------- the run
def build(cfg_path, dt, clip_speed=1.0, engage_share=None, rx=None, seed=None, out=print):
    """(TrackMode, graph, env): the Runner at the start hub, an Engage at greet."""
    import collision as C
    import engage as EN
    cfg = json.load(open(cfg_path))
    graph = S.Graph.load(S.compiled_path(os.path.abspath(cfg_path)))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    if engage_share is not None:
        EN.SHARE = engage_share
    runner, _, _ = away_runner(graph, cfg, seed)
    en = EN.Engage(cfg, graph.hubs[GREET], env, C.load_model("fr20"), dt=dt)
    return TrackMode(runner, en, rx, dt, clip_speed, out), graph, env


def main(argv=None):
    import fairino_player as P
    import robot_profile as RP
    import show_stream as SS
    import track_osc as TO
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    tgt = ap.add_mutually_exclusive_group(required=True)
    tgt.add_argument("--sim", action="store_true", help="the target is SimMachine")
    tgt.add_argument("--hardware", action="store_true", help="the real arm: asks before every move")
    ap.add_argument("--ip", required=True, help="the controller's IP (no default: say which arm)")
    ap.add_argument("--config", default=os.path.join(ROOT, "shows", "party.json"))
    ap.add_argument("--speed", type=float, default=None, help="the clips' speed (default 1.0 sim, 0.3 hardware)")
    ap.add_argument("--engage-share", type=float, default=None,
                    help="the interactive mode's share of the joints' limits (default engage.SHARE 0.35; "
                         "hardware first tests: 0.2)")
    ap.add_argument("--minutes", type=float, default=240.0, help="an end by itself after this long")
    ap.add_argument("--osc-in", type=int, default=PORT)
    ap.add_argument("--move-vel", type=float, default=None, help="MoveJ %% to the start pose (20 sim, 10 hardware)")
    ap.add_argument("--env", default=os.path.join(ROOT, "envs", "volvox_lab.usda"))
    ap.add_argument("--log", default=os.path.join(ROOT, "logs", "track_mode"))
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--stdin-control", action="store_true",
                    help="started by track_ui: stdin carries end / stop / the answers; its end stops")
    a = ap.parse_args(argv)
    speed = a.speed if a.speed is not None else (0.3 if a.hardware else 1.0)
    if not 0.05 <= speed <= 1.0:
        ap.error("--speed within 0.05..1")
    share = a.engage_share if a.engage_share is not None else (0.2 if a.hardware else 0.35)
    if not 0.05 <= share <= 0.35:
        ap.error("--engage-share within 0.05..0.35 (as tested)")
    dt = 1.0 / SS.RATE_HZ
    mode, graph, env = build(a.config, dt, speed, share, TO.TrackIn(a.osc_in), a.seed,
                             out=lambda s: print(s, flush=True))
    cmds = SS.Commands(mode)
    answers = None
    if a.stdin_control:                                  # started by track_ui: one reader for all of stdin
        answers = SS.Answers()
        threading.Thread(target=SS.read_stdin, args=(sys.stdin, cmds, answers, a.ip, {"end": cmds.pause}),
                         daemon=True).start()
    ask = (lambda text: SS.confirm(text, stdin=answers)) if a.hardware else (lambda text: True)
    import collision as C
    cfg = json.load(open(a.config))
    move_env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["scan_canvas_m"])     # the room + the paper
    S.require_fresh(graph, os.path.abspath(a.config))
    ctrl = P.Controller(a.ip)
    model = ctrl.model()
    print("%s at %s: %s" % ("HARDWARE" if a.hardware else "SimMachine", a.ip, model), flush=True)
    if a.sim:
        SS.check_sim_ip(a.ip)
    P._require_no_error(ctrl, "before starting")
    rep = {"target": "hardware" if a.hardware else "sim", "mode": "track_mode", "speed": speed,
           "engage_share": share}
    move_vel = a.move_vel if a.move_vel is not None else (10.0 if a.hardware else 20.0)
    start_q = list(mode.q)
    if P.move_checked(ctrl, start_q, a.env, "fr20", move_vel, rep, "start", ask,
                      "the start pose (%s)" % graph.info.get("start_hub"), env=move_env,
                      stop=cmds.stop_requested) is None:
        print(json.dumps(rep, indent=1))
        return 1
    plan = ("Interactive mode: clips at speed %.2f, the interactive mode at %.2f of the limits, the people from "
            "OSC :%d. End: back to the start pose. Stop: STOP / Ctrl+C (software). Keep a hand on the E-stop."
            % (speed, share, a.osc_in))
    print(plan, flush=True)
    if not ask(plan):
        return 1
    prof = RP.load("fr20")
    guard = SS.Guard(RP.velocity_limits(prof), RP.motion_limits(prof), dt, min(1.0, max(speed, share) * 1.1))
    link = SS.Link(a.ip)
    out, ticks_cmd, start = SS.stream(ctrl, link, mode, cmds, guard, dt, 1.0, a.minutes, analyse=False)
    out.update(rep)
    out["track_mode"] = dict(mode.status(), stats=mode.en.engagements)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    base = SS.write_log(a.log, stamp, out, ticks_cmd, SS.point_step(dt, None), link.samples, start)
    ended = out["ended"]
    print(json.dumps({k: out.get(k) for k in ("ended", "fault", "duration_s", "sends", "skipped",
                                              "worst_step_of_limit", "track_mode")}, indent=1), flush=True)
    if ended == "at a hub":                               # back to the start pose, checked, as the start
        back = {}
        P.move_checked(ctrl, start_q, a.env, "fr20", move_vel, back, "return", ask,
                       "back to the start pose (%s)" % graph.info.get("start_hub"), env=move_env,
                       stop=cmds.stop_requested)
        out["return"] = back
    with open(base + ".json", "w", newline="\n") as f:
        json.dump(out, f, indent=1)
    print("log:", base + ".json", flush=True)
    return 0 if ended in ("at a hub", "stopped") else 1


# ---------------------------------------------------------------------- self-test
class ScriptRx:
    """Scripted people: frames(t) -> (people, hands) at 30 Hz, t the mode's clock."""

    def __init__(self, mode_clock, frames):
        self.clock, self.frames, self.k = mode_clock, frames, -1

    def poll(self):
        k = int(self.clock() * 30.0)
        if k == self.k:
            return None
        self.k = k
        return self.frames(self.clock())


def self_test():
    import collision as C
    import engage as EN
    import robot_profile as RP
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    cfg_path = os.path.join(ROOT, "shows", "party.json")
    cfg = json.load(open(cfg_path))
    dt = 1.0 / 125.0
    graph = S.Graph.load(S.compiled_path(cfg_path))
    eyes = S.audience_eyes(cfg)

    # the selector: less time facing the guests than the show's own
    def facing_share(r, seconds):
        n, hits = 0, 0
        for i in range(int(seconds / 0.1)):
            q = r.step(0.1)
            if i % 5 == 0:
                n += 1
                hits += S.facing(q, eyes) > 0.5
        return hits / n
    show_r = S.runner_for(graph, seed=3)
    away_r, facing, hub_w = away_runner(graph, cfg, seed=3)
    a, b = facing_share(show_r, 1800.0), facing_share(away_r, 1800.0)
    check("idle: facing the guests less of the time than the show (30 min each: %.0f %% -> %.0f %%)"
          % (100 * a, 100 * b), b < a - 0.15, hub_w)
    check("... only the library's idle clips, and every move a built route (no showpieces, no scan)",
          all(n in {c.name for c in graph.idle()} or n.startswith("move_") for _, n in away_r.history),
          sorted({n for _, n in away_r.history if n not in {c.name for c in graph.idle()}})[:6])

    # the mode, stepped as the stream steps it, with scripted people
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    model = C.load_model("fr20")
    spot, _ = EN.track_spot(cfg, graph.hubs[GREET])

    def run(frames, seconds, end_at=None, a_every=None, end_if=None):
        holder = {}
        rx = ScriptRx(lambda: holder["m"].clock, frames)
        m, _, _ = build(cfg_path, dt, 1.0, None, rx, seed=5, out=lambda s: None)
        logs = []
        m.log = logs.append
        holder["m"] = m
        vlim = RP.velocity_limits(RP.load("fr20"))
        prev, worst, bad, qs = list(m.q), 0.0, 0, []
        for i in range(int(seconds / dt)):
            if (end_at is not None and abs(i * dt - end_at) < dt / 2) or (end_if is not None and end_if(m)):
                m.pause()
            ticks = 3 if a_every and i % a_every == 0 else 1
            q = m.step(dt * ticks)
            worst = max(worst, max(abs(x - y) / dt / v for x, y, v in zip(q, prev, vlim)))
            if i % 25 == 0 and not C.check(model, env, [0.0], [q])["ok"]:
                bad += 1
            prev = q
            qs.append(q)
            if m.state == "PAUSED":
                break
        return m, logs, worst, bad, qs

    nobody = lambda t: ([], [])                                   # noqa: E731

    def visitor(t):                                              # steps onto the spot at 20 s, leaves at 60 s
        if 20.0 <= t < 60.0:
            return [(1, spot[0], spot[1], 1.62, 0.9)], []
        return [], []
    m, logs, worst, bad, qs = run(visitor, 110.0)
    st = m.status()
    t_eng = next((float(x.split()[0]) for x in logs if "engaged person" in x), None)
    check("somebody on the spot while the arm is elsewhere: it comes to greet and engages them (%s s)" % t_eng,
          m.calls >= 1 and m.en.engagements == 1 and t_eng is not None and 21.0 < t_eng < 60.0, st)
    check("... and plays on after the goodbye", st["mode"] == "IDLE" and any("back at greet" in x for x in logs),
          [x for x in logs if "back" in x])
    check("... every pose clear (sampled), no step over the limits (%.2f of them)" % worst,
          bad == 0 and worst <= 1.0 and m.en.unsafe == 0 and m.en.refused == 0, (bad, round(worst, 3), m.en.unsafe))
    m, logs, worst, bad, qs = run(nobody, 200.0, end_at=30.0)
    check("End: the clip finishes at a hub, still, then the mode is PAUSED (the stream ends)",
          m.state == "PAUSED" and m.r.state == "PAUSED" and max(abs(x - y) for x, y in zip(qs[-1], qs[-2])) < 1e-9,
          (m.state, m.r.hub, round(m.clock, 1)))
    m, logs, worst, bad, qs = run(visitor, 120.0, end_if=lambda m: m.en.state == "TRACK")
    check("End while engaged: goodbye, back at greet, then PAUSED there", m.state == "PAUSED" and m.r.hub == GREET
          and m.en.state == "OFF", (m.state, m.r.hub, m.en.state))
    m, logs, worst, bad, qs = run(visitor, 60.0, a_every=40)
    check("a late stream (3 ticks asked every 40th point): one tick moved, the clock slips -- no jump "
          "(%.2f of the limits)" % worst, worst <= 1.0 and m.slips > 0, m.slips)

    def waver_gone(t):                                           # on the spot 20-24 s only: gone when it arrives
        return ([(1, spot[0], spot[1], 1.62, 0.9)], []) if 20.0 <= t < 24.0 else ([], [])
    m, logs, worst, bad, qs = run(waver_gone, 70.0)
    check("called, but gone by the time the arm is at greet: it waits %.0f s, then plays on" % WAIT_S,
          m.calls >= 1 and m.en.engagements == 0 and any("nobody at greet" in x for x in logs),
          [x for x in logs if "greet" in x][:3])
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
