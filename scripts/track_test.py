"""The tracking test on the SimMachine, apart from the party show: the greet
clips with the gaze and (--engage) the interactive mode -- track_runner's
Runner, the loop Isaac's run_tracking --live runs -- the people from
TouchDesigner's people_track over OSC (/track/people, /track/hands;
TD-ROBOT-UVSCAN), streamed as show_stream streams the show: ServoJ at 125
Hz on an absolute clock, the same guard every tick (joint steps within the
velocity limits, poses within the joint limits: else FAULT), the feedback
thread, the report.

    python scripts/track_test.py --self-test
    python scripts/track_test.py --sim --minutes 2 --engage          # TD's people_track sending to 9011

SimMachine only: a real-arm test of the interactive mode is the user's call
(IMPLEMENTATION_PLAN Stage 11). The end (--minutes): the people are let go
(the arm says goodbye, the gaze goes back), the clip playing finishes at
the hub, then the stream ends. Stop: Ctrl+C (software stop).
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import show_stream as SS  # noqa: E402

PORT = 9011                     # people_track sends to Isaac on 9010 and here on 9011


class TrackShow:
    """show_stream's runner (show.Runner's face: step, pause, fault, clock,
    state, history) over a track_runner.Runner fed by OSC (track_osc.TrackIn,
    or anything with poll() -> (people, hands) | None)."""

    hub = "greet"
    sel = None
    scan_speed = 1.0
    ends_when_paused = False

    def __init__(self, rn, rx, dt):
        self.rn, self.rx, self.dt = rn, rx, dt
        self.clock, self.state = 0.0, "PLAYING"
        self.history, self.log = [], print
        self.frames = self.engaged_log = 0
        self._clip, self._q = None, list(rn.q)

    def step(self, adv):
        """The pose adv (s) on: the runner stepped once a dt (ticks missed are stepped through)."""
        n = max(1, int(round(adv / self.dt)))
        got = self.rx.poll() if self.rx is not None else None
        for i in range(n):
            if i > 0 or self.clock > 0.0 or adv > 0.0:
                self.clock += self.dt
            now = self.clock
            if i == 0 and got is not None and self.state == "PLAYING":
                people, hands = got
                self.rn.feed([p[:5] + (now,) for p in people], [h[:5] + (now,) for h in hands], now)
                self.frames += 1
            if self.state == "PAUSED":
                return list(self._q)
            was = self.rn.en.state if self.rn.en is not None else None
            clip0 = (self.rn.player.clip, self.rn.player.t0)
            q = self.rn.step(now)
            if self.rn.en is not None and self.rn.en.state != was:
                self.log("%.2f interactive: %s -> %s (person %s)" % (now, was, self.rn.en.state, self.rn.en.who))
            clip1 = (self.rn.player.clip, self.rn.player.t0)
            if clip1 != self._clip:
                self.history.append((now, clip1[0].name))
                self._clip = clip1
            if self.state == "PAUSING" and clip1 != clip0 and self._settled():
                self.state = "PAUSED"              # at the hub: a clip ended (clips end at the hub, at rest)
                q = list(self.rn.player.at(self.rn.player.t0))
            self._q = q
        return list(self._q)

    def _settled(self):
        en = self.rn.en
        return (en is None or en.state == "OFF") and max(abs(x) for x in self.rn.gz.offsets) < 1e-3

    def pause(self):
        """The end: nobody fed any more (goodbye, the gaze back), the clip finishes at the hub."""
        if self.state == "PLAYING":
            self.state = "PAUSING"
            self.rn.feed([], [], self.clock)

    def fault(self, why):
        self.state = "FAULT"
        self.log("%.2f FAULT: %s" % (self.clock, why))


def self_test():
    """Against a fake controller in real time, the people from a femto_sim scene over OSC (localhost 9018)."""
    import threading
    import collision as C
    import show as S
    import track_osc as TO
    import track_runner as TRN
    import robot_profile as RP
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    csv_path = os.path.join(ROOT, "geo", "tracking", "femto_sim_mc_wave_call.csv")
    if not os.path.exists(csv_path):
        print("skip: %s not there (mocap_scenes.py --write-all geo/tracking)" % csv_path)
        return 0
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    graph = S.Graph.load(S.compiled_path(os.path.join(ROOT, "shows", "party.json")))
    model = C.load_model("fr20")
    dt = 1.0 / SS.RATE_HZ
    prof = RP.load("fr20")

    class FakeCtrl:
        def __init__(self, q):
            self.q, self.sent, self.calls = list(q), [], []

        def servo_start(self):
            self.calls.append("start")

        def servo_j(self, q, cmd_t, cmd_id):
            self.sent.append(list(q))
            self.q = list(q)
            return 0

        def servo_end(self):
            self.calls.append("end")

        def stop(self):
            self.calls.append("stop")

        def joints(self):
            return list(self.q)

        def error_code(self):
            return [0, 0, 0]

    stop = threading.Event()

    def sender():                                          # people_track's messages, from its CSV, at 30 fps
        sys.path.insert(0, os.path.join(os.path.expanduser("~"), "Documents", "GitHub", "TD-ROBOT-UVSCAN",
                                        "td-modules", "people_track"))
        import socket
        import people_track as PT
        R, p, _ = PT.load_extrinsic(os.path.join(ROOT, "geo", "tracking", "femto_sim_extrinsic.json"))
        frames, n = PT.load_frames(csv_path)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        t0, k = time.monotonic(), -1
        while not stop.is_set():
            kk = PT.frame_at(time.monotonic() - t0, n)
            if kk != k:
                k = kk
                ppl, hands = PT.messages(frames.get(k, []), R, p)
                sock.sendto(PT.osc_message("/track/hands", PT.osc_args(k / 30.0, hands)), ("127.0.0.1", 9018))
                sock.sendto(PT.osc_message("/track/people", PT.osc_args(k / 30.0, ppl)), ("127.0.0.1", 9018))
            time.sleep(0.005)

    rn = TRN.Runner(cfg, graph, env, model, dt, engage=True)
    show = TrackShow(rn, TO.TrackIn(9018), dt)
    th = threading.Thread(target=sender, daemon=True)
    th.start()
    ctrl = FakeCtrl(rn.q)
    guard = SS.Guard(RP.velocity_limits(prof), RP.motion_limits(prof), dt, 1.0)
    events = []
    rep, ticks, _ = SS.stream(ctrl, SS.Link(ctrl=ctrl), show, SS.Commands(show), guard, dt, 1.0, 20.0 / 60,
                              log=events.append, analyse=False)
    stop.set()
    st = rn.stats()
    check("20 s of TD's mc_wave_call over OSC: the stream ends at the hub, no fault",
          rep["ended"] == "at a hub" and show.state == "PAUSED", (rep["ended"], rep["fault"], show.state))
    check("... the people arrived (~30 frames a second)", show.frames > 20 * 25, show.frames)
    check("... the waver called the arm (an engagement), none refused",
          st["engage"]["engagements"] >= 1 and st["engage"]["refused"] == 0, st["engage"])
    check("... every pose clear (the gaze's and the interactive mode's checks) and every step within the limits",
          st["unsafe"] == 0 and st["engage"]["unsafe"] == 0 and rep["worst_step_of_limit"] <= 1.0,
          (st["unsafe"], st["engage"]["unsafe"], rep["worst_step_of_limit"]))
    check("... ends where it started: the hub", max(abs(a - b) for a, b in zip(ctrl.sent[-1], graph.hubs["greet"]))
          < 0.05, [round(x, 2) for x in ctrl.sent[-1]])
    check("one stream: ServoMoveStart once, ServoMoveEnd once", ctrl.calls == ["start", "end"], ctrl.calls)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    import collision as C
    import fairino_player as P
    import robot_profile as RP
    import show as S
    import track_osc as TO
    import track_runner as TRN
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", required=True, help="the target is SimMachine (the only one)")
    ap.add_argument("--ip", default=None, help="SimMachine's IP (default: playback.toml's)")
    ap.add_argument("--config", default=os.path.join(ROOT, "shows", "party.json"))
    ap.add_argument("--minutes", type=float, default=2.0)
    ap.add_argument("--engage", action="store_true", help="the interactive mode on top of the gaze")
    ap.add_argument("--osc-in", type=int, default=PORT, help="the people from here (people_track: 9011)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--move-vel", type=float, default=20.0, help="MoveJ %% to the greet hub")
    ap.add_argument("--env", default=os.path.join(ROOT, "envs", "volvox_lab.usda"), help="the room, for the start move")
    ap.add_argument("--log", default=os.path.join(ROOT, "logs", "track_test"))
    a = ap.parse_args(argv)
    ip = a.ip or SS._toml_ip()
    cfg = json.load(open(a.config))
    graph = S.Graph.load(S.compiled_path(os.path.abspath(a.config)))
    S.require_fresh(graph, os.path.abspath(a.config))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    dt = 1.0 / SS.RATE_HZ
    rn = TRN.Runner(cfg, graph, env, C.load_model("fr20"), dt, engage=a.engage, seed=a.seed)
    show = TrackShow(rn, TO.TrackIn(a.osc_in), dt)
    ctrl = P.Controller(ip)
    model = ctrl.model()
    if "sim" not in str(model).lower() and not str(ip).startswith("192.168.116."):
        raise SystemExit("%s at %s does not look like SimMachine: track_test is for SimMachine only" % (model, ip))
    print("SimMachine at %s: %s" % (ip, model))
    P._require_no_error(ctrl, "before starting")
    rep = {"target": "sim", "test": "tracking", "engage": a.engage, "osc_in": a.osc_in}
    off = P.move_checked(ctrl, rn.q, a.env, "fr20", a.move_vel, rep, "start", lambda text: True, "the greet hub")
    if off is None:
        print(json.dumps(rep, indent=1))
        return 1
    print("Streaming the tracking test: %.1f min, the people from OSC :%d%s. Stop: Ctrl+C."
          % (a.minutes, a.osc_in, ", the interactive mode on" if a.engage else ""))
    prof = RP.load("fr20")
    guard = SS.Guard(RP.velocity_limits(prof), RP.motion_limits(prof), dt, 1.0)
    link = SS.Link(ip)
    out, ticks_cmd, start = SS.stream(ctrl, link, show, SS.Commands(show), guard, dt, 1.0, a.minutes, analyse=False)
    out.update(rep)
    out["tracking"] = dict(rn.stats(), osc_frames=show.frames, looked_last=rn.looked)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    step = SS.point_step(dt, None)
    base = SS.write_log(a.log, stamp, out, ticks_cmd, step, link.samples, start)
    SS.add_tracking(out, start, step, ticks_cmd, link.samples)
    with open(base + ".json", "w", newline="\n") as f:
        json.dump(out, f, indent=1)
    print(json.dumps({k: out.get(k) for k in ("ended", "fault", "duration_s", "sends", "skipped", "worst_step_of_limit",
                                              "tracking_after_lag_max_deg", "best_lag_ms", "controller_error",
                                              "tracking")}, indent=1))
    print("log:", base + ".json")
    return 0 if out["ended"] in ("at a hub", "stopped") else 1


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
