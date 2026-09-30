"""Tracking scenes from real people (CMU motion capture, mocap_cmu.py) placed
in the audience zone, seen as the Femto would see them: the tracking data
for the tracking layer (track_sim's format: /track/people and /track/hands,
robot frame) and a camera-frame table for TouchDesigner's people_track
module to play back in place of the Femto.

    ev, truth, dur = scenario("mc_wave_call")          # track_sim's scenario() for mocap scenes
    write_camera_csv("mc_wave_call", "geo/tracking/femto_sim_mc_wave_call.csv")

    python scripts/mocap_scenes.py --self-test
    python scripts/mocap_scenes.py --write-all geo/tracking          every scene's camera table for TD

A person: a clip, where (u along the zone, v across it: + towards the glass),
facing the arm or walking along; a standing clip plays back and forth to the
scene's length, a walking one once (they come and go). The Femto: FEMTO_POSE
(on the guests' side of the glass, above it, looking down at them); its body
tracking's error -- 1 cm at the head, 2 cm at the wrists, 70-110 ms late,
30 frames a second, a frame or a hand missed now and then.
"""

import csv
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import mocap_cmu as MC  # noqa: E402

FPS = 30.0
LATENCY_S = (0.07, 0.11)
NOISE_M = {"head": 0.01, "wrist": 0.02}
DROP_P, HAND_DROP_P = 0.03, 0.08
FEMTO_HEIGHT_M, FEMTO_PITCH_DEG = 2.3, 30.0      # above the glass, looking down at the guests

# name: (duration s, [(pid, clip, u, v, pose, start s)]); pose: "face" (facing the arm), "+u" / "-u" (walking),
# ("pair", k): the k-th of a pair captured together, placed as captured
SCENES = {
    "mc_walk_by": (8.0, [(1, "07_01", -1.4, 0.1, "+u", 1.0)]),
    "mc_wave_call": (16.0, [(1, "141_20", -0.45, 0.0, "face", 0.0), (2, "141_16", 0.55, 0.05, "face", 3.0)]),
    "mc_stretch": (12.0, [(1, "141_13", 0.5, 0.0, "face", 0.0)]),
    "mc_chat": (20.0, [(1, "18_08", -0.3, 0.1, ("pair", 0), 0.0), (2, "19_08", -0.3, 0.1, ("pair", 1), 0.0)]),
    "mc_crowd": (24.0, [(1, "141_20", 0.0, 0.0, "face", 0.0), (2, "143_25", 0.6, 0.1, "face", 4.0),
                        (3, "07_01", -1.5, 0.35, "+u", 2.0), (4, "35_01", 1.5, -0.2, "-u", 6.0),
                        (5, "13_26", 0.2, -0.75, "face", 0.0)]),
    "mc_approach": (14.0, [(1, "16_33", -1.6, 0.0, "+u", 1.0)]),
}
HEAD_Z = {1: 1.62, 2: 1.70, 3: 1.75, 4: 1.66, 5: 1.80}   # each person's head height (the clips' own vary)


def zone(cfg=None):
    import track_sim as TS
    return TS.zone_frame(cfg)


def _heading(tr, pose):
    """The clip's own direction: walking -- where it goes; else -- where it faces (the shoulders)."""
    if pose in ("+u", "-u"):
        a, b = tr[0]["root"], tr[-1]["root"]
        return math.atan2(b[1] - a[1], b[0] - a[0])
    l, r = tr[0]["lclavicle"], tr[0]["rclavicle"]
    s = (r[0] - l[0], r[1] - l[1])
    return math.atan2(s[0], -s[1])                  # forward = up x (right - left)


class Person:
    """A clip placed in the room: at(t) -> {"head", "lwrist", "rwrist"} (robot frame) or None (not there).
    pair: the first frames of both clips of a pair captured together ([first, second]): both are placed
    with the one transform -- their middle at (u, v), the line between them along the zone."""

    def __init__(self, pid, name, u, v, pose, start, dur, frame, pair=None):
        self.pid, self.start, self.dur = pid, start, dur
        self.t, self.tr = MC.clip(name, FPS)
        c, along, across = frame
        if pair is not None:
            a, b = pair[0]["root"], pair[1]["root"]
            self.origin = [(x + y) / 2 for x, y in zip(a, b)]
            self.rot = math.atan2(along[1], along[0]) - math.atan2(b[1] - a[1], b[0] - a[0])
        else:
            want = {"+u": math.atan2(along[1], along[0]),
                    "-u": math.atan2(-along[1], -along[0])}.get(pose, math.atan2(across[1], across[0]))  # face the arm
            self.rot = want - _heading(self.tr, pose)
            self.origin = self.tr[0]["root"]
        self.at_xy = (c[0] + u * along[0] + v * across[0], c[1] + u * along[1] + v * across[1])
        heads = [p["head"][2] for p in self.tr]
        self.dz = HEAD_Z.get(pid, 1.65) - sum(heads) / len(heads)
        self.walk = pose in ("+u", "-u")

    def _place(self, p):
        x, y = p[0] - self.origin[0], p[1] - self.origin[1]
        c, s = math.cos(self.rot), math.sin(self.rot)
        return (self.at_xy[0] + c * x - s * y, self.at_xy[1] + s * x + c * y, p[2] + self.dz)

    def at(self, t):
        k = t - self.start
        if k < 0:
            return None
        n = len(self.tr)
        i = int(round(k * FPS))
        if self.walk:
            if i >= n:
                return None
        else:
            period = 2 * (n - 1)
            i = i % period
            i = i if i < n else period - i                  # back and forth
        p = self.tr[i]
        return {k2: self._place(p[k2]) for k2 in ("head", "lwrist", "rwrist")}


def people(name, cfg=None):
    dur, specs = SCENES[name]
    frame = zone(cfg)
    firsts = {}                                            # a pair's first frames, in the pair's order
    for pid, clip_name, u, v, pose, start in specs:
        if isinstance(pose, tuple):
            firsts.setdefault((u, v, start), {})[pose[1]] = MC.clip(clip_name, FPS)[1][0]
    out = []
    for pid, clip_name, u, v, pose, start in specs:
        pair = None
        if isinstance(pose, tuple):
            f = firsts[(u, v, start)]
            pair = [f[0], f[1]]
        out.append(Person(pid, clip_name, u, v, pose if pair is None else "face", start, dur, frame, pair))
    return dur, out


def _hand(pts):
    return pts["rwrist"] if pts["rwrist"][2] >= pts["lwrist"][2] else pts["lwrist"]


def scenario(name, seed=1, cfg=None):
    """(events, truth, duration) as track_sim.scenario: /track/people and /track/hands, one each a frame
    after the latency; truth(t) -> {pid: head}."""
    dur, ps = people(name, cfg)
    rng = random.Random(seed)

    def truth(t):
        out = {}
        for p in ps:
            a = p.at(t)
            if a is not None:
                out[p.pid] = a["head"]
        return out

    def noisy(q, s):
        return tuple(round(x + rng.gauss(0, s), 4) for x in q)

    events = []
    for i in range(int(dur * FPS)):
        tm = i / FPS
        arrive = tm + rng.uniform(*LATENCY_S)
        seen, hands = [], []
        for p in ps:
            a = p.at(tm)
            if a is None or rng.random() < DROP_P:
                continue
            conf = round(rng.uniform(0.75, 0.95), 3)
            seen += [p.pid] + list(noisy(a["head"], NOISE_M["head"])) + [conf]
            if rng.random() >= HAND_DROP_P:
                hands += [p.pid] + list(noisy(_hand(a), NOISE_M["wrist"])) + [conf]
        events.append({"t": arrive, "addr": "/track/people", "args": [round(tm, 4), len(seen) // 5] + seen})
        events.append({"t": arrive + 1e-4, "addr": "/track/hands", "args": [round(tm, 4), len(hands) // 5] + hands})
    events.sort(key=lambda e: e["t"])
    return events, truth, dur


# ------------------------------------------------------------------ the Femto's view (for TouchDesigner)
def femto_pose(cfg=None):
    """(R, p) camera to robot: the Femto above the glass in front of the zone's middle, looking down at the
    guests at FEMTO_PITCH_DEG; camera axes as the Kinect's: x right, y down, z forward."""
    import collision as C
    cfg = cfg or json.load(open(os.path.join(ROOT, "shows", "party.json")))
    c, along, across = zone(cfg)
    g = next(o for o in C.load_env(cfg["env"])["objects"] if o["name"] == "partition_left")
    n, off = g["normal"], g["offset"]
    k = (off - (n[0] * c[0] + n[1] * c[1])) / (n[0] * across[0] + n[1] * across[1])   # the glass along -across
    p = (c[0] + (k - 0.05) * across[0], c[1] + (k - 0.05) * across[1], FEMTO_HEIGHT_M)
    fwd0 = (-across[0], -across[1], 0.0)                                               # towards the guests
    a = math.radians(FEMTO_PITCH_DEG)
    z = (fwd0[0] * math.cos(a), fwd0[1] * math.cos(a), -math.sin(a))                   # forward, down a
    x = (z[1] * 1.0 - 0.0, 0.0 - z[0] * 1.0, 0.0)                                       # right = forward x up
    nx = math.sqrt(x[0] ** 2 + x[1] ** 2) or 1.0
    x = (x[0] / nx, x[1] / nx, 0.0)
    y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0])   # down = forward x right
    R = [[x[i], y[i], z[i]] for i in range(3)]                                         # columns: the camera's axes
    return R, p


def to_camera(R, p, q):
    d = [q[i] - p[i] for i in range(3)]
    return tuple(sum(R[i][j] * d[i] for i in range(3)) for j in range(3))


def layout(cfg=None):
    """What a top-down view of the tracking needs (robot base frame, m; for TouchDesigner's people_track):
    the glass (normal, offset: the guests on its far side), the band BAND_M in front of it where waving
    counts, the audience zone's corners, the interactive mode's spot (centre, radius) and follow zone's
    corners (at greet), the Femto's position."""
    import collision as C
    import engage as E
    import show as S
    import tracking as TR
    cfg = cfg or json.load(open(os.path.join(ROOT, "shows", "party.json")))
    c, along, across = zone(cfg)
    g = next(o for o in C.load_env(cfg["env"])["objects"] if o["name"] == "partition_left")
    hub = S.Graph.load(S.compiled_path(os.path.join(ROOT, "shows", "party.json"))).hubs["greet"]
    spot, r = E.track_spot(cfg, hub)
    size = cfg["zones"]["audience"]["size"]

    def rect(o, hu, hv):
        return [[round(o[0] + su * hu * along[0] + sv * hv * across[0], 4),
                 round(o[1] + su * hu * along[1] + sv * hv * across[1], 4)]
                for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return {"glass": {"normal": list(g["normal"][:2]), "offset": g["offset"]}, "band_m": TR.BAND_M,
            "zone": rect(c, size[0] / 2, size[1] / 2), "spot": [round(spot[0], 4), round(spot[1], 4)], "spot_r": r,
            "follow": rect(spot, E.FOLLOW_M[0], E.FOLLOW_M[1]), "femto": [round(x, 4) for x in femto_pose(cfg)[1]]}


def write_camera_csv(name, path, seed=1, cfg=None):
    """The scene as the Femto's body tracking would give it, camera frame (m): a row a body a frame --
    frame, t, body, conf, head xyz, wrist_left xyz, wrist_right xyz (empty when the hand was missed)."""
    dur, ps = people(name, cfg)
    R, p = femto_pose(cfg)
    rng = random.Random(seed)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t", "body", "conf", "head_x", "head_y", "head_z", "wristl_x", "wristl_y", "wristl_z",
                    "wristr_x", "wristr_y", "wristr_z"])
        for i in range(int(dur * FPS)):
            t = i / FPS
            for per in ps:
                a = per.at(t)
                if a is None or rng.random() < DROP_P:
                    continue
                row = [i, round(t, 4), per.pid, round(rng.uniform(0.75, 0.95), 3)]
                row += [round(x + rng.gauss(0, NOISE_M["head"]), 4) for x in to_camera(R, p, a["head"])]
                for k in ("lwrist", "rwrist"):
                    row += ([round(x + rng.gauss(0, NOISE_M["wrist"]), 4) for x in to_camera(R, p, a[k])]
                            if rng.random() >= HAND_DROP_P else ["", "", ""])
                w.writerow(row)
    return path


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    if not os.path.exists(os.path.join(MC.CMU_DIR, "07_01.amc")):
        print("skip: the CMU clips are not in %s" % MC.CMU_DIR)
        return 0
    c, along, across = zone()
    dur, ps = people("mc_walk_by")
    a, b = ps[0].at(1.0), ps[0].at(3.0)
    moved = (b["head"][0] - a["head"][0]) * along[0] + (b["head"][1] - a["head"][1]) * along[1]
    check("a walker goes along the zone (+u), then is gone when the clip ends", moved > 1.5 and ps[0].at(7.9) is None,
          (round(moved, 2), ps[0].at(7.9)))
    dur, ps = people("mc_wave_call")
    w = ps[1].at(5.0)
    check("the waver: head %.2f m up; not there before 3 s" % HEAD_Z[2],
          abs(w["head"][2] - HEAD_Z[2]) < 0.2 and ps[1].at(2.0) is None, round(w["head"][2], 2))
    ev, truth, dur = scenario("mc_crowd")
    kinds = {e["addr"] for e in ev}
    n = max(e["args"][1] for e in ev if e["addr"] == "/track/people")
    ids = {e["args"][2 + 5 * k] for e in ev if e["addr"] == "/track/people" for k in range(e["args"][1])}
    check("the crowd: /track/people and /track/hands a frame; five people come and go (four at once at most)",
          kinds == {"/track/people", "/track/hands"} and n == 4 and ids == {1, 2, 3, 4, 5}, (kinds, n, sorted(ids)))
    R, p = femto_pose()
    q = (c[0], c[1], 1.6)
    cam = to_camera(R, p, q)
    back = tuple(p[i] + sum(R[i][j] * cam[j] for j in range(3)) for i in range(3))
    check("the Femto: above the glass looking down at the zone (the zone's middle ahead of it, below its axis)",
          cam[2] > 0.5 and cam[1] > 0.0 and math.dist(back, q) < 1e-9 and p[2] == FEMTO_HEIGHT_M,
          [round(x, 2) for x in cam])
    lay = layout()
    n, off = lay["glass"]["normal"], lay["glass"]["offset"]
    gd = lambda q: off - n[0] * q[0] - n[1] * q[1]
    check("the layout: the spot and the zone on the guests' side of the glass, the spot within the band, "
          "the Femto at the glass", gd(lay["spot"]) > 0 and all(gd(q) > 0 for q in lay["zone"])
          and gd(lay["spot"]) < lay["band_m"] and abs(gd(lay["femto"])) < 0.2,
          (round(gd(lay["spot"]), 2), round(gd(lay["femto"]), 2)))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--write-all" in sys.argv:
        out = sys.argv[sys.argv.index("--write-all") + 1]
        for k in SCENES:
            print(write_camera_csv(k, os.path.join(out, "femto_sim_%s.csv" % k)))
        R, p = femto_pose()
        with open(os.path.join(out, "femto_sim_extrinsic.json"), "w") as f:
            json.dump({"R": R, "p": p, "note": "camera (x right, y down, z forward) to robot base: q = R c + p",
                       "layout": layout()}, f, indent=1)
        sys.exit(0)
    sys.exit(self_test())
