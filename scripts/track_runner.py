"""The tracking layer as a live runner would run it, without a robot: the
greet clips back to back (engage.ClipPlayer) with the gaze on top
(tracking: Attention -> TargetInput -> Gaze) and, with engage, the
interactive mode (engage.Engage) -- fed a frame of people at a time,
stepped every dt. One loop for every live test: Isaac's run_tracking
--live, the SimMachine's track_test.py, and offline here.

    rn = Runner(cfg, graph, env, model, dt=1/120, engage=True)
    rn.feed(people, hands, now)          # a frame: [(pid, x, y, z, conf, t)], stamped on arrival
    q = rn.step(now)                     # the pose to send (deg), once a dt
    rn.stats()                           # counters over the whole run (the gaze is renewed after each RETURN)

    python scripts/track_runner.py                 self-test: the mocap scenes through it, with and without engage
    python scripts/track_runner.py mc_wave_call --engage    one, with a report
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)


class Runner:
    def __init__(self, cfg, graph, env, model, dt, engage=False, seed=1, hub="greet"):
        import engage as EN
        import tracking as TR
        self.TR, self.dt = TR, dt
        self.player = EN.ClipPlayer(graph, hub, seed)
        g = next((o for o in env["objects"] if o["name"] == "partition_left"), None)       # the glass
        self.ti, self.att = TR.TargetInput(), TR.Attention(glass=(g["normal"], g["offset"]) if g else None)
        self._new_gaze = lambda: TR.Gaze(anchor=graph.hubs[hub], dt=dt, env=env, model=model)  # noqa: E731
        self.gz = self._new_gaze()
        self.en = EN.Engage(cfg, graph.hubs[hub], env, model, dt=dt) if engage else None
        self.looked = self.tgt = None
        self.q = list(graph.hubs[hub])
        self.done = {"unsafe": 0, "shrunk": 0, "held_slow": 0, "retreats": 0}       # of the gazes renewed
        self.gazes, self.states, self.ticks = 1, {}, 0

    def feed(self, people, hands, now, direct=False):
        """A frame of people (and their hands): whom to look at (Attention), and the interactive mode's view.
        direct: one person straight to TargetInput, no Attention (Isaac's dragged head)."""
        if self.en is not None:
            self.en.update(people, now, hands)
        if direct:
            for pid, x, y, z, conf, t in people[:1]:
                self.ti.target(x, y, z, 1.0, now, 0, now=now)
            return
        self.att.update(people, now, hands)
        ch = self.att.choose(now)
        self.looked = ch[1] if ch else None
        if ch is not None:
            self.ti.target(ch[0][0], ch[0][1], ch[0][2], 0.9, now, ch[1], now=now)
        elif self.ti.seen is not None:
            self.ti.lost()

    def step(self, now):
        """The pose to send now (deg)."""
        self.player.advance(now)
        base = self.player.at
        self.tgt = self.ti.now(now)
        q = self.q
        if self.en is None or self.en.state == "OFF":
            q = self.gz.step(base(now), self.tgt, now, [base(now + d) for d in self.TR.AHEAD_S], clip=base)
        if self.en is not None:
            q = self.en.step(q, now)
            self.states[self.en.state] = self.states.get(self.en.state, 0) + 1
            if self.en.resume:                   # back at the hub: the clips from the start, a fresh gaze
                self.player.restart(now)
                for k in self.done:
                    self.done[k] += getattr(self.gz, k, 0)
                self.gz = self._new_gaze()
                self.gazes += 1
        self.q = list(q)
        self.ticks += 1
        return self.q

    def robot_status(self, now):
        """What TouchDesigner hears (show.osc_messages: its LEDs play), as track_mode's robot_status: the greet
        clip playing, IDLE, with its cues; engaged, the look family, no clip cues, and the look on the strip."""
        import engage as EN
        import show as S
        pl = self.player
        if not getattr(self, "_warm", False):
            for c in pl.clips:                                    # the cues of every clip now, none while playing
                S.motion_cues(c)
            self._warm = True
        seg, t = pl.clip, max(0.0, min(now - pl.t0, pl.clip.duration))
        lab = seg.labels or {}
        beat, bpm = S.clip_beat(lab, t)
        s = {"state": "IDLE", "clip": seg.name, "hub": "greet", "sequence": None, "family": lab.get("family") or "",
             "action": lab.get("action") or "", "clip_t": round(t, 3), "clip_len": round(seg.duration, 3),
             "beat": round(beat, 4), "bpm_now": round(bpm, 2),
             "progress": round(t / seg.duration, 4) if seg.duration > 0 else 1.0, "scan": -1.0, "scan_u": -1.0,
             "scan_led": 0, "scan_mps": 0.0, "time_left": round(seg.duration - t, 2), "next": pl.next.name,
             "queue": [], "pending": [], "fault": None, "energy": 0.0,
             "clip_energy": round(float(lab.get("energy") or 0.0), 3), **S.cue_values(seg, t, pl.next)}
        if self.engaged:
            te = round(now - getattr(self.en, "t_engaged", now), 3)
            s.update(clip="interactive", family="look", action=self.en.state.lower(), clip_t=te, clip_len=te + 10.0,
                     progress=0.0, beat=0.0, anticipate=0.0, release=0.0, accent_in=-1.0, led_speed_a=0.0,
                     led_speed_b=0.0)
        s.update(EN.look_status(self.en, self.q) if self.en is not None else {"look_u": -1.0, "look_w": 0.0})
        return s

    @property
    def engaged(self):
        return self.en is not None and self.en.state != "OFF"

    @property
    def shown(self):
        """Whom the arm is on: the engaged one, else the gaze's."""
        return self.en.who if self.engaged else self.looked

    def stats(self):
        out = {k: v + getattr(self.gz, k, 0) for k, v in self.done.items()}
        out.update(turns=self.att.turns, seconds=round(self.ticks * self.dt, 1))
        if self.en is not None:
            out["engage"] = {"engagements": self.en.engagements, "refused": self.en.refused,
                             "unsafe": self.en.unsafe,
                             "seconds_in": {k: round(v * self.dt, 1) for k, v in self.states.items()}}
        return out


def run_scene(name, engage, seed=1, dt=1.0 / 120.0, loops=1, trace=None, told=None):
    """A mocap scene's /track/ events (mocap_scenes: the Femto's noise, latency, drops) through a Runner,
    `loops` times back to back; every pose checked against the room. (runner, contacts, joint speed share)."""
    import collision as C
    import robot_profile as RP
    import show as S
    import track_sim as TS
    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    graph = S.Graph.load(S.compiled_path(os.path.join(ROOT, "shows", "party.json")))
    model = C.load_model("fr20")
    rn = Runner(cfg, graph, env, model, dt, engage=engage, seed=seed)
    events, _, dur = TS.scenario(name, seed, cfg)            # mc_ scenes, rec:<a recording> ...
    vlim = RP.velocity_limits(RP.load("fr20"))
    evs = [dict(e, t=e["t"] + k * dur) for k in range(loops) for e in events]
    i, hands, bad, vmax, prev = 0, [], 0, 0.0, None
    for tick in range(int(loops * dur / dt)):
        now = tick * dt
        while i < len(evs) and evs[i]["t"] <= now:
            e = evs[i]
            i += 1
            got = [p + (now,) for p in TS.people_of(e["args"])]
            if e["addr"] == "/track/hands":
                hands = got
            else:
                rn.feed(got, hands, now)
        q = rn.step(now)
        if told is not None and tick % 60 == 0:           # what TouchDesigner would hear, twice a second
            told.append((rn.en.state if rn.en else "OFF", rn.robot_status(now), list(q)))
        if tick % 12 == 0 and not C.check(model, env, [0.0], [q])["ok"]:
            bad += 1
            if trace is not None:
                trace.append((round(now, 2), rn.en.state if rn.en else "gaze", [round(x, 1) for x in q]))
        if prev is not None:
            vmax = max(vmax, max(abs(a - b) / dt / v for a, b, v in zip(q, prev, vlim)))
        prev = q
    return rn, bad, vmax


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    import mocap_cmu as MC
    if not os.path.exists(os.path.join(MC.CMU_DIR, "07_01.amc")):
        print("skip: the CMU clips are not in %s" % MC.CMU_DIR)
        return 0
    for name in ("mc_wave_call", "mc_crowd", "mc_chat"):
        for engage in (False, True):
            rn, bad, vmax = run_scene(name, engage, loops=3)
            st = rn.stats()
            e = st.get("engage", {})
            check("%s%s, 3 times over: every pose clear (the gaze's %d and the room check), within the speed "
                  "limits%s" % (name, " engage" if engage else "", rn.gazes,
                                ", %d engaged, 0 unsafe" % e.get("engagements", 0) if engage else ""),
                  st["unsafe"] == 0 and bad == 0 and vmax <= 1.0 and e.get("unsafe", 0) == 0,
                  (st["unsafe"], bad, round(vmax, 3), e.get("engagements"), e.get("refused")))
    told = []
    rn, _, _ = run_scene("mc_wave_call", True, loops=2, told=told)
    check("mc_wave_call engage: the waver called it each time round", rn.en.engagements >= 2, rn.en.engagements)
    import show as S
    idle = [s for st, s, q in told if st == "OFF"]
    looking = [s for st, s, q in told if st in ("PERK", "TRACK")]
    check("robot_status (TouchDesigner's LEDs, as the show's): the greet clips as they play -- IDLE, the clip, "
          "its cues; engaged: the look family and the look on the strip",
          idle and all(s["state"] == "IDLE" and s["clip"].startswith("greet") and "anticipate" in s for s in idle)
          and looking and all(s["family"] == "look" and 0.0 <= s["look_u"] <= 1.0 and s["look_w"] > 0.0
                              for s in looking)
          and all(len(S.osc_messages(s, q)) == len(S.osc_messages(idle[0], q)) for _, s, q in told),
          (idle[:1], looking[:1]))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        tr = []
        rn, bad, vmax = run_scene(sys.argv[1], "--engage" in sys.argv, loops=3, trace=tr)
        print(json.dumps(dict(rn.stats(), room_bad=bad, speed_share=round(vmax, 3))))
        for x in tr[:20]:
            print(x)
        sys.exit(0)
    sys.exit(self_test())
