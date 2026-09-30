"""Simulated tracking data: people in front of the wall as the OAK-D process
will report them, for testing the tracking layer before a camera.

The messages are the agreed format (docs/next_steps_2026-09-28.md):

    /track/target  x y z conf t id    robot-base metres, ~30 Hz; t: when measured
    /track/lost                       nobody tracked any more
    /track/people  t n  id x y z conf  (n times)     everybody in view, one frame (the crowd scenarios)

Each scenario has a ground truth -- where the person's head (or hand) really
is, or None when nobody is there -- and errors like the OAK-D's: a frame
every 1/30 s with jitter, 60-110 ms from measurement to arrival (camera,
detector, network), depth noise, dropped frames, some low-confidence ones.
Some add what makes a naive tracker jerk: the detector jumping to another
person, a one-frame depth outlier, the person lost for a while or for good.

The people stand in the show's audience zone (shows/party.json zones:
outside the wall, out of reach): u along it (-1 .. 1 m, left to right as
the robot sees them), v across it (towards the wall -), heads ~1.6 m high.

    uv run scripts/track_sim.py                       self-test
    uv run scripts/track_sim.py --list
    uv run scripts/track_sim.py --write walk_across out.jsonl
    uv run scripts/track_sim.py --osc 127.0.0.1:9000 walk_across    as live OSC, in real time
"""

import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

FPS = 30.0
FRAME_JITTER_S = 0.004
LATENCY_S = (0.06, 0.11)             # measurement to arrival
NOISE_M = (0.015, 0.015, 0.01)       # along the zone, across it (depth: more below), up
DEPTH_NOISE_M = 0.03
DROP_P = 0.15
LOW_CONF_P = 0.05
LOST_AFTER_S = 0.3                   # the tracker process says /track/lost this long after the last detection
HEAD_Z = 1.6


def zone_frame(cfg=None):
    """(centre, along, across) of the audience zone, robot base frame."""
    cfg = cfg or json.load(open(os.path.join(ROOT, "shows", "party.json")))
    a = cfg["zones"]["audience"]
    yaw = math.radians(a.get("yaw_deg", 0.0))
    return a["center"], (math.cos(yaw), math.sin(yaw), 0.0), (-math.sin(yaw), math.cos(yaw), 0.0)


def _world(frame, u, v, z):
    c, al, ac = frame
    return (c[0] + u * al[0] + v * ac[0], c[1] + u * al[1] + v * ac[1], z)


def _smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def _walk(t, t0, u0, u1, speed):
    """u at t walking from u0 at t0 to u1 at `speed` m/s (eased at the ends), and the end time."""
    dur = abs(u1 - u0) / speed
    return u0 + (u1 - u0) * _smooth((t - t0) / dur), t0 + dur


# ground truths: t -> (u, v, z, person id) or None -------------------------------

def _walk_across(t):
    if t < 2.0:
        return None
    u, t1 = _walk(t, 2.0, -1.0, 1.0, 1.2)
    if t < t1 + 3.0:
        return u, 0.0, HEAD_Z + 0.02 * math.sin(3 * t), 1
    u, t2 = _walk(t, t1 + 3.0, 1.0, -0.3, 1.0)
    return u, 0.1, HEAD_Z, 1


def _stand_still(t):
    return 0.2, 0.0, HEAD_Z + 0.01 * math.sin(1.3 * t), 1


def _hand_wave(t):                     # the hand, not the head: up and down in front of the body
    return 0.1, -0.15, 1.35 + 0.25 * math.sin(2 * math.pi * 0.6 * t), 1


def _jump(t):                          # the detector switches to another person 1.3 m away
    return (-0.6, 0.0, HEAD_Z, 1) if t < 8.0 else (0.7, 0.2, 1.72, 2)


def _outlier(t):
    return 0.0, 0.0, HEAD_Z, 1


def _lost_2s(t):
    if 6.0 <= t < 8.0:
        return None
    return 0.3 * math.sin(0.4 * t), 0.0, HEAD_Z, 1


def _lost_for_good(t):
    return None if t >= 6.0 else (-0.2 + 0.05 * t, 0.0, HEAD_Z, 1)


def _in_and_out(t):
    if 2.0 <= t < 7.0 or t >= 12.0:
        return (0.4 if t < 7.0 else -0.4), 0.0, HEAD_Z, 1 if t < 7.0 else 3
    return None


def _out_of_reach(t):                  # walks on past the zone's end, and ducks low
    u, _ = _walk(t, 1.0, 0.0, 2.6, 1.0)
    return u, 0.0, HEAD_Z - (0.6 if t > 8.0 else 0.0), 1


def _run(t):                           # runs across at 2.5 m/s
    if t < 2.0:
        return None
    u, t1 = _walk(t, 2.0, -1.0, 1.0, 2.5)
    return u, 0.0, HEAD_Z, 1


# several people: t -> [(u, v, z, person id), ...] ------------------------------

def _two_standing(t):
    return [] if t < 1.0 else [(-0.5, 0.0, HEAD_Z, 1), (0.5, 0.0, 1.66, 2)]


def _crowd(t):                         # five, staggered: each more than GROUP_M from the next
    return [(-1.0 + 0.5 * k, -0.4 * (k % 2), HEAD_Z + 0.03 * k, k + 1) for k in range(5) if t >= 0.5 + 0.7 * k]


def _passer_by(t):
    out = [(0.2, 0.0, HEAD_Z, 1)]
    if 4.0 <= t:
        u, t1 = _walk(t, 4.0, -1.2, 1.4, 1.2)
        if t < t1:
            out.append((u, -0.3, 1.7, 2))
    return out


def _group(t):                         # three within 0.3 m of each other: one group
    return [(0.0, 0.0, HEAD_Z, 1), (0.3, 0.0, 1.62, 2), (0.15, -0.1, 1.55, 3)]


def _handover(t):                      # A there; B comes at 4 s; A leaves at 8 s
    out = [(-0.5, 0.0, HEAD_Z, 1)] if t < 8.0 else []
    if t >= 4.0:
        out.append((0.6, 0.1, 1.65, 2))
    return out


PEOPLE = {"two_standing", "crowd", "passer_by", "group", "handover"}

SCENARIOS = {
    "walk_across": (_walk_across, 16.0, {}, "enters at the left, walks across at 1.2 m/s, stops, walks back"),
    "stand_still": (_stand_still, 12.0, {}, "stands still: only the noise moves"),
    "hand_wave": (_hand_wave, 12.0, {}, "a hand up and down 0.5 m at 0.6 Hz (modes B / C)"),
    "jump": (_jump, 16.0, {}, "the detector jumps to another person 1.3 m away at 8 s"),
    "outlier": (_outlier, 12.0, {"outliers": [(6.0, (0.0, 3.0, -0.4)), (9.0, (1.5, 0.0, 0.0)), (9.033, (1.5, 0.0, 0.0))]},
                "one depth outlier 3 m off at 6 s, two frames 1.5 m off at 9 s"),
    "lost_2s": (_lost_2s, 14.0, {}, "lost from 6 to 8 s, back where it was heading"),
    "lost_for_good": (_lost_for_good, 14.0, {}, "tracked, then lost at 6 s and never back"),
    "in_and_out": (_in_and_out, 18.0, {}, "comes in at 2 s, leaves at 7 s, another comes in at 12 s"),
    "out_of_reach": (_out_of_reach, 14.0, {}, "walks past the zone's end, then ducks"),
    "run": (_run, 8.0, {}, "runs across at 2.5 m/s"),
    "noisy": (_stand_still, 12.0, {"noise": 3.0, "drop": 0.4}, "stands still, three times the noise, 40 % dropped"),
    "two_standing": (_two_standing, 40.0, {}, "two people standing 1 m apart: looked at in turn"),
    "crowd": (_crowd, 60.0, {}, "five people arriving one by one, staggered: everyone looked at in turn"),
    "passer_by": (_passer_by, 14.0, {}, "one stands; another walks by at 1.2 m/s: never looked at"),
    "group": (_group, 20.0, {}, "three people within 0.3 m: looked at as one, their middle"),
    "handover": (_handover, 16.0, {}, "A there, B comes at 4 s, A leaves at 8 s: B next, at once"),
}


def _mocap():
    """mocap_scenes (real people, CMU): their scenes join SCENARIOS when the clips are there."""
    try:
        import mocap_scenes as MS
        import mocap_cmu as MC
    except ImportError:
        return None
    if not os.path.exists(os.path.join(MC.CMU_DIR, "07_01.amc")):
        return None
    for k, (dur, specs) in MS.SCENES.items():
        SCENARIOS.setdefault(k, (None, dur, {}, "real people (CMU mocap): " + ", ".join(sp[1] for sp in specs)))
        PEOPLE.add(k)
    return MS


def recording(path, seed=1):
    """A recording of the Femto (TD's people_track Record, or a simulation: femto_format's CSV) as a crowd
    scenario ("rec:<path>"): its frames through femto_format, as TD sends them, 70-110 ms late; the
    extrinsic: femto_extrinsic.json beside it (people_track's Calibrate), else the simulation's -- a simulation
    (femto_sim_*) always its own; truth(t):
    the recording's own heads (the camera is all there is)."""
    import femto_format as FF
    folder = os.path.dirname(os.path.abspath(path))
    names = ("femto_sim_extrinsic.json",) if os.path.basename(path).startswith("femto_sim_") else \
        ("femto_extrinsic.json", "femto_sim_extrinsic.json")          # a simulation: its own camera, always
    ext = next(p for p in (os.path.join(folder, n) for n in names) if os.path.exists(p))
    e = json.load(open(ext))
    frames = FF.read(path)
    t0 = frames[0][1] if frames else 0.0
    heads = [({bid: FF.to_robot(e["R"], e["p"], j["head"][:3]) for bid, j in b if "head" in j}) for _, _, b in frames]

    def truth(t):
        k = int(round((t) * FPS))
        return heads[k] if 0 <= k < len(heads) else {}
    PEOPLE.add("rec:" + path)
    events = FF.events(frames, e["R"], e["p"], seed)
    events.sort(key=lambda x: x["t"])
    return events, truth, (frames[-1][1] - t0 + 1.0 / FPS) if frames else 0.0


def scenario(name, seed=1, cfg=None):
    """(events, truth, duration): events sorted by arrival, each
    {"t": arrival s, "addr": "/track/target" | "/track/lost", "args": [...]};
    truth(t) -> (x, y, z) in the robot base frame, or None. A crowd scenario
    (PEOPLE): /track/people events and truth(t) -> {person id: (x, y, z)}."""
    if name.startswith("mc_"):
        return _mocap().scenario(name, seed, cfg)
    if name.startswith("rec:"):
        return recording(name[4:], seed)
    if name in PEOPLE:
        return _people_scenario(name, seed, cfg)
    fn, dur, opt, _ = SCENARIOS[name]
    rng = random.Random(seed)
    frame = zone_frame(cfg)
    k = opt.get("noise", 1.0)
    outliers = {round(t0 * FPS): off for t0, off in opt.get("outliers", [])}

    def truth(t):
        p = fn(t)
        return None if p is None else _world(frame, p[0], p[1], p[2])

    events, last_seen, lost_sent = [], None, True
    n = int(dur * FPS)
    for i in range(n):
        tm = i / FPS + rng.uniform(-FRAME_JITTER_S, FRAME_JITTER_S)
        p = fn(max(0.0, tm))
        arrive = tm + rng.uniform(*LATENCY_S)
        if p is None:
            if not lost_sent and last_seen is not None and tm - last_seen >= LOST_AFTER_S:
                events.append({"t": arrive, "addr": "/track/lost", "args": []})
                lost_sent = True
            continue
        if rng.random() < opt.get("drop", DROP_P):
            continue
        u, v, z, pid = p
        u += rng.gauss(0, NOISE_M[0] * k)
        v += rng.gauss(0, NOISE_M[1] * k) + rng.gauss(0, DEPTH_NOISE_M * k)
        z += rng.gauss(0, NOISE_M[2] * k)
        if i in outliers:
            du, dv, dz = outliers[i]
            u, v, z = u + du, v + dv, z + dz
        conf = rng.uniform(0.25, 0.45) if rng.random() < LOW_CONF_P else rng.uniform(0.7, 0.95)
        x, y, zz = _world(frame, u, v, z)
        events.append({"t": arrive, "addr": "/track/target",
                       "args": [round(x, 4), round(y, 4), round(zz, 4), round(conf, 3), round(tm, 4), pid]})
        last_seen, lost_sent = tm, False
    events.sort(key=lambda e: e["t"])
    return events, truth, dur


def _people_scenario(name, seed=1, cfg=None):
    """Everybody in view each frame, as the OAK-D process would send them:
    one /track/people message a frame (after the latency), each person with
    their own noise, some dropped from a frame, some low-confidence."""
    fn, dur, opt, _ = SCENARIOS[name]
    rng = random.Random(seed)
    frame = zone_frame(cfg)

    def truth(t):
        return {pid: _world(frame, u, v, z) for u, v, z, pid in fn(t)}

    events = []
    for i in range(int(dur * FPS)):
        tm = i / FPS + rng.uniform(-FRAME_JITTER_S, FRAME_JITTER_S)
        seen = []
        for u, v, z, pid in fn(max(0.0, tm)):
            if rng.random() < opt.get("drop", DROP_P):
                continue
            u += rng.gauss(0, NOISE_M[0])
            v += rng.gauss(0, NOISE_M[1]) + rng.gauss(0, DEPTH_NOISE_M)
            z += rng.gauss(0, NOISE_M[2])
            conf = rng.uniform(0.25, 0.45) if rng.random() < LOW_CONF_P else rng.uniform(0.7, 0.95)
            x, y, zz = _world(frame, u, v, z)
            seen += [pid, round(x, 4), round(y, 4), round(zz, 4), round(conf, 3)]
        events.append({"t": tm + rng.uniform(*LATENCY_S), "addr": "/track/people",
                       "args": [round(tm, 4), len(seen) // 5] + seen})
    events.sort(key=lambda e: e["t"])
    return events, truth, dur


def people_of(args):
    """[(pid, x, y, z, conf)] of a /track/people message's arguments (t, n, then n people)."""
    n = int(args[1])
    return [tuple(args[2 + 5 * k:7 + 5 * k]) for k in range(n)]


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    ev, truth, dur = scenario("walk_across")
    tg = [e for e in ev if e["addr"] == "/track/target"]
    rate = len(tg) / (dur - 2.0)
    check("~30 Hz less the dropped frames while somebody is there", 22 < rate < 28, "%.1f Hz" % rate)
    lat = [e["t"] - e["args"][4] for e in tg]
    check("60-110 ms from measurement to arrival", min(lat) >= 0.059 and max(lat) <= 0.111, (min(lat), max(lat)))
    err = max(math.dist(e["args"][:3], truth(e["args"][4])) for e in tg if abs(e["args"][4] - 8.0) > 0.5)
    check("each measurement near the truth (noise, no outliers here)", err < 0.2, "%.3f m" % err)
    c, al, _ = zone_frame()
    first = tg[0]["args"]
    check("people stand in the audience zone, in the robot's frame, heads ~1.6 m high",
          abs(first[2] - HEAD_Z) < 0.1 and math.dist(first[:2], c[:2]) < 1.5, first)
    check("the same seed, the same data", scenario("jump", seed=4)[0] == scenario("jump", seed=4)[0])
    rec = os.path.join(ROOT, "geo", "tracking", "femto_sim_mc_wave_call.csv")
    if _MS is not None and os.path.exists(rec):
        a_ = scenario("mc_wave_call")[0]
        b_ = scenario("rec:" + rec)[0]
        close = len(a_) == len(b_) and all(x["addr"] == y["addr"] and x["args"][1:2] == y["args"][1:2] and all(
            abs(u - v) < 1e-3 for u, v in zip(x["args"][2:], y["args"][2:])) for x, y in zip(a_, b_))
        check("a recording (rec:, the Femto's format) plays as the scene it was written from: one door",
              close, (len(a_), len(b_)))
    ev, truth, _ = scenario("jump")
    ids = sorted({e["args"][5] for e in ev if e["addr"] == "/track/target"})
    before = [e["args"] for e in ev if e["addr"] == "/track/target" and 7.5 < e["args"][4] < 8.0][-1]
    after = [e["args"] for e in ev if e["addr"] == "/track/target" and 8.0 <= e["args"][4] < 8.5][0]
    check("jump: another person 1.3 m away from one frame to the next", ids == [1, 2]
          and math.dist(before[:3], after[:3]) > 1.1, math.dist(before[:3], after[:3]))
    ev, truth, _ = scenario("outlier")
    far = [e for e in ev if e["addr"] == "/track/target" and math.dist(e["args"][:3], truth(e["args"][4])) > 1.0]
    check("outlier: one frame 3 m off, two 1.5 m off (unless dropped)", 1 <= len(far) <= 3, len(far))
    ev, _, _ = scenario("lost_2s")
    lost = [e["t"] for e in ev if e["addr"] == "/track/lost"]
    gap = [e for e in ev if e["addr"] == "/track/target" and 6.0 <= e["args"][4] < 8.0]
    check("lost_2s: no targets from 6 to 8 s, one /track/lost ~0.3 s in", not gap and len(lost) == 1
          and 6.3 < lost[0] < 6.5, lost)
    ev, _, _ = scenario("lost_for_good")
    check("lost_for_good: nothing after the /track/lost", ev[-1]["addr"] == "/track/lost")
    ev, truth, _ = scenario("in_and_out")
    check("in_and_out: two people, a /track/lost when the first leaves",
          sorted({e["args"][5] for e in ev if e["addr"] == "/track/target"}) == [1, 3]
          and sum(e["addr"] == "/track/lost" for e in ev) == 1 and truth(9.0) is None)
    ev, truth, dur = scenario("crowd")
    counts = [e["args"][1] for e in ev]
    check("crowd: one /track/people a frame, up to five in it, arriving one by one",
          all(e["addr"] == "/track/people" for e in ev) and max(counts) == 5 and counts[0] <= 1
          and len(truth(30.0)) == 5, (min(counts), max(counts)))
    first = people_of(ev[-1]["args"])
    check("... each person's id and head in the robot frame",
          first and all(len(p) == 5 and 1 <= p[0] <= 5 and abs(p[3] - HEAD_Z) < 0.3 for p in first), first[:1])
    ev, truth, _ = scenario("handover")
    check("handover: A gone after 8 s, B from 4 s", sorted(truth(6.0)) == [1, 2] and sorted(truth(9.0)) == [2])
    ev, _, _ = scenario("noisy")
    tg = [e for e in ev if e["addr"] == "/track/target"]
    check("noisy: 40 % dropped", 15 < len(tg) / 12.0 < 21, "%.1f Hz" % (len(tg) / 12.0))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


_MS = _mocap()


def main(argv):
    if "--list" in argv:
        for k, (_, dur, _, what) in SCENARIOS.items():
            print("%-14s %4.0f s  %s" % (k, dur, what))
        return 0
    if "--write" in argv:
        i = argv.index("--write")
        ev, _, _ = scenario(argv[i + 1])
        with open(argv[i + 2], "w") as f:
            for e in ev:
                f.write(json.dumps(e) + "\n")
        print("wrote %d messages to %s" % (len(ev), argv[i + 2]))
        return 0
    if "--osc" in argv:
        import time
        from pythonosc import udp_client
        i = argv.index("--osc")
        host, _, port = argv[i + 1].rpartition(":")
        ev, _, dur = scenario(argv[i + 2])
        cl = udp_client.SimpleUDPClient(host, int(port))
        t0 = time.perf_counter()
        for e in ev:
            time.sleep(max(0.0, e["t"] - (time.perf_counter() - t0)))
            cl.send_message(e["addr"], e["args"])
        print("sent %d messages over %.0f s" % (len(ev), dur))
        return 0
    return self_test()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
