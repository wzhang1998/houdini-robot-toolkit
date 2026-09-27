"""A show: the arm plays through a clip library, switching on outside events
(TouchDesigner over OSC), always between validated motions.

    python scripts/show.py build shows/party.json          make and check every motion
    python scripts/show.py dry-run shows/party.json [--minutes 10] [--trigger-every 45]
    python scripts/show.py osc shows/party.json            dry-run clock, driven by OSC (TouchDesigner)
    python scripts/show.py --self-test

A motion graph with hubs (as in game motion graphs and Boston Dynamics'
Choreographer): every motion starts and ends at rest at a hub, so any two
join without a jump.

    hub --idle clip--> same hub       generated for that hub (choreo.py phrases that start and
                                      end at the hub pose, facing the hub's own J1)
    hub --move--> other hub           a checked MoveJ-like move (safe_move.route, retime_topp)
    scan hub --to_scan--> scan_start --scan--> scan_end --from_scan--> scan hub
                                      the fixed sweep across the paper (a PLACEHOLDER line for now)

The show config (shows/*.json) says where the hubs are, how many clips
each gets, how long they may be, the stage (work zone) and the paper. The
Houdini show scene edits the same file. Everything is made and checked at
BUILD time against the room (collision.py, the lab env, the controller's
work area, the stage) plus the paper: idle clips and moves keep
margins.idle_canvas_m from it, the scan and its moves scan_canvas_m. At
show time nothing is planned, only chosen.

Runner: plays idle clips at a hub, moves to another hub every few clips
(select.hub_stay), and takes requests at the end of the running clip:
    scan            route to the scan hub, to_scan, scan, from_scan
    <sequence>      route to its hub, play its clips (count, mood), back to idling
States: IDLE, MOVE, TO_SCAN, SCAN, FROM_SCAN, PAUSED (holds at a hub), FAULT
(holds, needs reset). Selection: weighted random, no repeat within the last
select.no_repeat at a hub, weighted towards a mood (a Laban action) and an
energy (0 calm .. 1 lively) that TouchDesigner may send.

OSC in:  /robot/trigger [name] (default scan)   /robot/pause   /robot/resume   /robot/reset
         /robot/mood <action>   /robot/energy <0..1>
OSC out: /robot/state s  /robot/clip s  /robot/hub s  /robot/progress f
         /robot/scan f (0..1 while scanning: the LED strip's column clock)  /robot/joints f*6

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

SCHEMA = "motionlab.show.compiled/2"
STEP_DEG = 1.0                      # joint-space sampling of a planned move
STATE_OF = {"idle": "IDLE", "move": "MOVE", "to_scan": "TO_SCAN", "scan": "SCAN", "from_scan": "FROM_SCAN"}


# --------------------------------------------------------------------------
# segments and the graph (pure data once compiled)
# --------------------------------------------------------------------------

class Segment:
    def __init__(self, name, kind, t, q, start, end, labels=None):
        self.name, self.kind, self.t, self.q = name, kind, list(t), [list(x) for x in q]
        self.start, self.end, self.labels = start, end, labels or {}
        self._path = None                              # built on first use by at()

    @property
    def duration(self):
        return self.t[-1]

    def at(self, s):
        """Joints at time s, held at the ends. Between samples a
        shape-preserving cubic (the player's PCHIP), so a 24 fps clip
        streamed at 125 Hz has no velocity step at every sample."""
        t, q = self.t, self.q
        if s <= t[0]:
            return list(q[0])
        if s >= t[-1]:
            return list(q[-1])
        if self._path is None:
            import fairino_player
            self._path = fairino_player.Path(t, q)
        return self._path.at(s - t[0])

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

    def idle(self, hub=None):
        return [s for s in self.segments if s.kind == "idle" and (hub is None or s.start == hub)]

    def one(self, kind):
        return [s for s in self.segments if s.kind == kind][0]

    def move(self, a, b):
        m = [s for s in self.segments if s.kind == "move" and s.start == a and s.end == b]
        return m[0] if m else None

    def idle_hubs(self):
        return sorted(set(s.start for s in self.idle()))

    def route(self, a, b):
        """Hub-to-hub moves from a to b (breadth first), [] when a == b, None if none."""
        if a == b:
            return []
        seen, frontier = {a: []}, [a]
        while frontier:
            nxt = []
            for h in frontier:
                for s in self.segments:
                    if s.kind == "move" and s.start == h and s.end not in seen:
                        seen[s.end] = seen[h] + [s]
                        if s.end == b:
                            return seen[b]
                        nxt.append(s.end)
            frontier = nxt
        return None

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


def show_env(env, cfg, canvas_margin):
    """The room for this show: the config's stage replaces the env's work zone
    of that name; the paper is an obstacle with canvas_margin."""
    objs = []
    stage = cfg.get("stage")
    for o in env["objects"]:
        if stage and o["name"] == stage.get("name", "stage"):
            o = dict(o, center=list(stage["center"]), size=list(stage["size"]), yaw_deg=stage.get("yaw_deg", 0.0))
        objs.append(o)
    if cfg.get("canvas"):
        objs.append(dict(canvas_box(cfg["canvas"]), margin_m=canvas_margin))
    return dict(env, objects=objs)


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


def out_of_range(cfg, qs, tcps):
    """Why a motion leaves the show's operating range (cfg["range"]), or
    None: J1 within j1_deg (the directions the arm may face), the TCP's
    height within tcp_z."""
    r = cfg.get("range") or {}
    if r.get("j1_deg"):
        lo, hi = r["j1_deg"]
        j1 = [q[0] for q in qs]
        if min(j1) < lo or max(j1) > hi:
            return "J1 %.0f..%.0f outside the range %g..%g" % (min(j1), max(j1), lo, hi)
    if r.get("tcp_z") and tcps:
        lo, hi = r["tcp_z"]
        z = [p[2] for p in tcps]
        if min(z) < lo or max(z) > hi:
            return "TCP height %.2f..%.2f outside %g..%g m" % (min(z), max(z), lo, hi)
    return None


def hub_clips(name, hub, n, lib, env, model, kin, rng, log=print, cfg=None):
    """n idle clips for one hub: choreo phrases made facing the robot's front
    from the hub's pose with J1 = 0, then turned by the hub's J1 (as
    stage_set.py turns clips), then checked in the room. [Segment], [why dropped]."""
    import choreo
    import collision as C
    import stage_set
    local = [0.0] + list(hub[1:])
    lo, hi = lib["duration_s"]
    got, dropped, tries = [], [], 0
    while len(got) < n and tries < n * lib.get("tries", 6):
        tries += 1
        spec = choreo.random_spec(rng, bars=rng.choice(lib["bars"]),
                                  actions=None if not lib.get("actions") else
                                  [rng.choice(lib["actions"]) for _ in range(rng.choice(lib["bars"]))])
        spec["bpm"] = int(min(max(spec["bpm"], lib["bpm"][0]), lib["bpm"][1]))
        spec["start"] = local
        clip = choreo.make_clip(spec, seed=rng.randrange(10 ** 9), env=None, kin=kin)
        if not clip["safety"].get("ok"):
            dropped.append(clip["safety"]["reasons"][0][:80])
            continue
        d = clip["points"][-1]["t"]
        if not lo <= d <= hi:
            dropped.append("%.1f s, outside %g-%g s" % (d, lo, hi))
            continue
        c = stage_set.turned(clip, hub[0])
        t, q = [p["t"] for p in c["points"]], [p["q"] for p in c["points"]]
        if not all(-172.0 < x[0] < 172.0 for x in q):
            dropped.append("J1 past its limit")
            continue
        why = out_of_range(cfg or {}, q, c.get("tcp"))
        if why:
            dropped.append(why)
            continue
        rep = C.check(model, env, t, q)
        if not rep["ok"]:
            dropped.append(C.describe(rep)[:80])
            continue
        m = (clip.get("labels") or {}).get("measured") or {}
        labels = {"action": m.get("action"), "effort": {k: m.get(k) for k in ("weight", "time", "space", "flow")},
                  "intent": [b["action"] for b in spec["bars"]], "bpm": spec["bpm"],
                  "clearance_m": rep["min_env_clearance_m"]}
        seg_name = "%s_%02d_%s" % (name, len(got), "-".join(labels["intent"]))
        seg = Segment(seg_name, "idle", t, q, name, name, labels)
        seg.clip = dict(c, id=seg_name)
        got.append(seg)
    log("  %s: %d clips from %d tries" % (name, len(got), tries))
    return got, dropped


def gesture_clips(name, hub, h, cfg, env, rig, rng, log=print):
    """Gesture clips for a hub (gestures.py): the families in turn, towards
    the show's zones; kept when they fit the length and clear the room."""
    import gestures as G
    lib = cfg["library"]
    lo, hi = lib["duration_s"]
    fams = h.get("families") or list(G.FAMILIES)
    n = h.get("clips", 0)
    got, dropped, tries = [], [], 0
    per = lib.get("tries", 6)
    for slot in range(n * 2):                                  # the families in turn, a slot each
        if len(got) >= n:
            break
        fam = fams[slot % len(fams)]
        for _ in range(per):
            tries += 1
            seg = _one_gesture(name, fam, hub, cfg, env, rig, rng, lib, lo, hi, len(got), dropped)
            if seg:
                got.append(seg)
                break
    log("  %s: %d gestures from %d tries" % (name, len(got), tries))
    return got, dropped


def _one_gesture(name, fam, hub, cfg, env, rig, rng, lib, lo, hi, index, dropped):
    import gestures as G
    if True:
        bpm = rng.randint(*lib["bpm"])
        k = rng.uniform(*lib.get("intensity", (0.4, 0.9)))
        clip = G.make(rig, hub, fam, cfg["zones"], rng, bpm=bpm, intensity=k, env=env,
                      safety=(cfg.get("range") or {}).get("speed", G.PLAN_SAFETY))
        if clip is None:
            dropped.append("%s: no clean draw" % fam)
            return None
        d = clip["points"][-1]["t"]
        if not lo <= d <= hi:
            dropped.append("%s %.1f s, outside %g-%g s" % (fam, d, lo, hi))
            return None
        why = out_of_range(cfg, [p["q"] for p in clip["points"]], clip.get("tcp"))
        if why:
            dropped.append("%s: %s" % (fam, why))
            return None
        m = (clip.get("labels") or {}).get("measured") or {}
        t, q = [p["t"] for p in clip["points"]], [p["q"] for p in clip["points"]]
        labels = {"action": m.get("action"), "effort": {x: m.get(x) for x in ("weight", "time", "space", "flow")},
                  "intent": [fam], "family": fam, "bpm": bpm, "intensity": round(k, 2),
                  "clearance_m": clip["safety"].get("min_clearance_m"), "wrist_share": round(G.wrist_share(q), 2)}
        seg_name = "%s_%02d_%s" % (name, index, fam)
        seg = Segment(seg_name, "idle", t, q, name, name, labels)
        seg.clip = dict(clip, id=seg_name)
        return seg


def authored_clip(a, hubs, env, model, vel, acc, cfg, log=print):
    """An authored clip (a["csv"]: the asset's joint export) played from hub
    a["hub"] and back: move in (safe_move.route, timed), the clip, move out --
    one idle segment from the hub to the hub, checked in the room."""
    import collision as C
    import fairino_player as P
    import motion_labels
    import motion_clip as M
    import safe_move
    hub = hubs[a["hub"]]
    t, q = P.load_csv(os.path.join(ROOT, a["csv"]))
    path_in, why_in = safe_move.route(hub, q[0], env, model)
    path_out, why_out = safe_move.route(q[-1], hub, env, model)
    if path_in is None or path_out is None:
        log("  authored %s: no clear move %s" % (a.get("id", a["csv"]), why_in if path_in is None else why_out))
        return None
    ti, qi = timed_move([hub] + path_in, cfg["transition_safety"], vel, acc)
    to, qo = timed_move([q[-1]] + path_out, cfg["transition_safety"], vel, acc)
    hold = 0.3                                                # a beat of rest between the parts
    t_all = list(ti) + [ti[-1] + hold + x for x in t]
    t_all += [t_all[-1] + hold + x for x in to]
    q_all = list(qi) + [list(x) for x in q] + list(qo)
    rep = C.check(model, env, t_all, q_all)
    if not rep["ok"]:
        log("  authored %s refused: %s" % (a.get("id", a["csv"]), C.describe(rep)))
        return None
    name = "%s_%s" % (a["hub"], a.get("id") or os.path.splitext(os.path.basename(a["csv"]))[0])
    clip = {"points": [{"t": x, "q": y} for x, y in zip(t_all, q_all)], "tcp": M._tcp_path("fr20", q_all)}
    m = (motion_labels.label(clip).get("measured") or {})
    labels = {"action": m.get("action"), "effort": {x: m.get(x) for x in ("weight", "time", "space", "flow")},
              "intent": ["authored"], "source": a["csv"], "clearance_m": rep["min_env_clearance_m"]}
    log("  authored %s: %.1f s with its moves" % (name, t_all[-1]))
    return Segment(name, "idle", t_all, q_all, a["hub"], a["hub"], labels)


def resolve_hubs(cfg, rig):
    """{name: joints}: a hub's q as given, or solved from where the tool tip
    is ("tcp") and the point it looks at ("look"), nearest "near" (or q)."""
    import gestures as G
    out = {}
    for k, h in cfg["hubs"].items():
        if h.get("tcp") and h.get("look"):
            near = h.get("near") or h.get("q") or [-60.0, -90.0, 90.0, -90.0, -90.0, 0.0]
            q = G.hub_pose(rig, h["tcp"], h["look"], near)
            if q is None:
                raise SystemExit("hub %s: no pose puts the tool at %s looking at %s" % (k, h["tcp"], h["look"]))
            out[k] = [round(x, 4) for x in q]
        else:
            out[k] = list(h["q"])
    return out


def build(cfg_path, log=print):
    import choreo
    import collision as C
    import robot_profile as RP
    import safe_move
    cfg = json.load(open(cfg_path))
    prof = RP.load("fr20")
    vel, acc = RP.velocity_limits(prof), RP.acceleration_limits(prof)
    model = C.load_model("fr20")
    env = C.load_env(os.path.join(ROOT, cfg["env"]))
    idle_env = show_env(env, cfg, cfg["margins"]["idle_canvas_m"])
    scan_env = show_env(env, cfg, cfg["margins"]["scan_canvas_m"])
    import gestures as G
    rig = G.Rig()
    hubs = resolve_hubs(cfg, rig)
    menv = safe_move.move_env(idle_env)
    for k, q in hubs.items():
        hit = safe_move.blocked(model, menv, q)
        if hit:
            raise SystemExit("hub %s is not clear: %s near %s (%.3f m)" % (k, hit[0], hit[1], hit[2]))
    lib = cfg["library"]
    rng = random.Random(lib.get("seed", 1))
    kin = choreo.Kin()
    segs, dropped = [], {}
    log("idle clips (%g-%g s, bars %s):" % (lib["duration_s"][0], lib["duration_s"][1], lib["bars"]))
    for k, h in cfg["hubs"].items():
        if h.get("generator", "choreo") == "gestures":
            got, why = gesture_clips(k, hubs[k], h, cfg, idle_env, rig, rng, log)
        else:
            got, why = hub_clips(k, hubs[k], h.get("clips", 0), lib, idle_env, model, kin, rng, log, cfg)
        segs += got
        dropped[k] = why
        if h.get("clips", 0) and not got:
            raise SystemExit("hub %s: no clip could be made (%s)" % (k, "; ".join(why[:3])))

    # authored clips (a curve drawn in Houdini, exported as a joint CSV):
    # a checked move from the hub to its start, the clip, and back
    for a in cfg.get("authored", []):
        seg = authored_clip(a, hubs, idle_env, model, vel, acc, cfg, log)
        if seg:
            segs.append(seg)

    # moves between the hubs, both ways
    for a in hubs:
        for b in hubs:
            if a == b:
                continue
            path, why = safe_move.route(hubs[a], hubs[b], idle_env, model)
            if path is None:
                log("  no move %s -> %s: %s" % (a, b, why))
                continue
            t, q = timed_move([hubs[a]] + path, cfg["transition_safety"], vel, acc)
            rep = C.check(model, idle_env, t, q)
            if not rep["ok"]:
                log("  move %s -> %s refused: %s" % (a, b, C.describe(rep)))
                continue
            segs.append(Segment("move_%s_%s" % (a, b), "move", t, q, a, b))

    # the scan (PLACEHOLDER line) and the moves to and from it
    if cfg.get("scan"):
        scan = placeholder_scan(cfg, scan_env)
        if not scan["safety"]["ok"]:
            raise SystemExit("scan: %s" % scan["safety"]["reasons"])
        st, sq = [p["t"] for p in scan["points"]], [p["q"] for p in scan["points"]]
        hubs["scan_start"], hubs["scan_end"] = list(sq[0]), list(sq[-1])
        segs.append(Segment("scan", "scan", st, sq, "scan_start", "scan_end"))
        home = cfg["scan"]["from_hub"]
        for name, kind, a, b in (("to_scan", "to_scan", home, "scan_start"), ("from_scan", "from_scan", "scan_end", home)):
            path, why = safe_move.route(hubs[a], hubs[b], scan_env, model)
            if path is None:
                raise SystemExit("%s: %s" % (name, why))
            t, q = timed_move([hubs[a]] + path, cfg["transition_safety"], vel, acc)
            rep = C.check(model, scan_env, t, q)
            if not rep["ok"]:
                raise SystemExit("%s: %s" % (name, C.describe(rep)))
            segs.append(Segment(name, kind, t, q, a, b))
            log("%s: %.1f s, %s" % (name, t[-1], why))
    g = Graph(hubs, segs, {"config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"),
                           "built": time.strftime("%Y-%m-%d %H:%M"), "start_hub": cfg["start_hub"],
                           "dropped": dropped, "canvas": cfg.get("canvas"), "stage": cfg.get("stage"),
                           "sequences": cfg.get("sequences", {}), "select": cfg.get("select", {})})
    bad = g.check_joins()
    if bad:
        raise SystemExit("segments do not meet at their hubs: %s" % bad)
    unreachable = [h for h in g.idle_hubs() if g.route(cfg["start_hub"], h) is None]
    if unreachable:
        raise SystemExit("hubs not reachable from %s: %s" % (cfg["start_hub"], unreachable))
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


def write_preview(graph, out_dir):
    """Every segment as a motion clip JSON (t, q, TCP by FK) and a manifest,
    for Houdini (the show scene previews any of them on the arm)."""
    import motion_clip as M
    os.makedirs(out_dir, exist_ok=True)
    for f in os.listdir(out_dir):
        if f.endswith(".json"):
            os.remove(os.path.join(out_dir, f))
    rows = []
    for s in graph.segments:
        if getattr(s, "clip", None):                          # the generator's own clip: style, labels, safety
            M.save(s.clip, os.path.join(out_dir, s.name + ".json"))
            rows.append({"name": s.name, "kind": s.kind, "start": s.start, "end": s.end,
                         "duration_s": round(s.duration, 2), "labels": s.labels})
            continue
        clip = {"schema": M.SCHEMA, "id": s.name, "robot": "fr20", "joint_names": ["j%d" % i for i in range(1, 7)],
                "units": {"angle": "deg", "time": "s", "length": "m"},
                "points": [{"t": t, "q": q} for t, q in zip(s.t, s.q)], "tcp": M._tcp_path("fr20", s.q),
                "meta": {"duration_s": s.duration, "show": {"kind": s.kind, "start": s.start, "end": s.end}},
                "labels": s.labels, "safety": {"ok": True}}
        M.save(clip, os.path.join(out_dir, s.name + ".json"))
        rows.append({"name": s.name, "kind": s.kind, "start": s.start, "end": s.end,
                     "duration_s": round(s.duration, 2), "labels": s.labels})
    with open(os.path.join(out_dir, "manifest.json"), "w") as f:
        json.dump({"hubs": graph.hubs, "segments": rows,
                   "clips": [{"file": r["name"] + ".json", "id": r["name"], "ok": True} for r in rows]}, f, indent=1)


# --------------------------------------------------------------------------
# choosing and running
# --------------------------------------------------------------------------

class Selector:
    """Next idle clip at a hub: weighted random, no repeat within the last
    no_repeat, weighted towards a mood (Laban action) and an energy."""

    def __init__(self, clips, no_repeat=6, seed=None):
        self.clips, self.no_repeat = clips, no_repeat
        self.recent, self.mood, self.energy = [], None, None
        self.rng = random.Random(seed)

    def weight(self, c, mood=None):
        w = 1.0
        mood = mood or self.mood
        if mood and c.labels.get("action") == mood:
            w *= 4.0
        e = (c.labels.get("effort") or {})
        if self.energy is not None and e.get("time") is not None:
            lively = (e["time"] + 1.0) / 2.0                  # sudden = lively
            w *= 0.25 + 1.5 * (1.0 - abs(lively - self.energy))
        return w

    def pick(self, hub=None, mood=None):
        at = [c for c in self.clips if hub is None or c.start == hub]
        pool = [c for c in at if c.name not in self.recent[-self.no_repeat:]] or at
        c = self.rng.choices(pool, weights=[self.weight(x, mood) for x in pool])[0]
        self.recent.append(c.name)
        return c


class Runner:
    """Walks the graph one step at a time: step(dt) -> joints (degrees)."""

    def __init__(self, graph, selector, start_hub=None, hub_stay=(2, 4), sequences=None, seed=None, log=None):
        self.g, self.sel, self.log = graph, selector, log or (lambda *a: None)
        self.hub = start_hub or graph.info.get("start_hub") or graph.idle_hubs()[0]
        self.hub_stay, self.sequences = hub_stay, sequences if sequences is not None else graph.info.get("sequences", {})
        self.rng = random.Random(seed)
        self.pending, self.queue, self.paused, self.fault_reason = [], [], False, None
        self.clock, self.seg_t, self.sequence = 0.0, 0.0, None
        self.stay = self.rng.randint(*self.hub_stay)
        self.history = []
        self.seg = self.sel.pick(self.hub)
        self.state = "IDLE"

    # events (from OSC, a keyboard, a test script)
    def trigger(self, name="scan"):
        if name != "scan" and name not in self.sequences:
            self.log("%.2f unknown trigger %r" % (self.clock, name))
            return
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

    def _plan_next(self):
        """Fill the queue with what comes next, from self.hub."""
        if self.pending:
            name = self.pending.pop(0)
            if name == "scan":
                via = self.g.route(self.hub, self.g.one("to_scan").start)
                self.queue += via + [self.g.one("to_scan"), self.g.one("scan"), self.g.one("from_scan")]
                self.sequence = "scan"
                return
            seq = self.sequences[name]
            via = self.g.route(self.hub, seq.get("hub", self.hub)) or []
            self.queue += via
            self.queue += [("seq", name, seq.get("hub", self.hub), seq.get("mood"))] * int(seq.get("count", 2))
            self.sequence = name
            return
        self.sequence = None
        hubs = self.g.idle_hubs()
        self.stay -= 1
        if self.stay <= 0 and len(hubs) > 1:
            other = self.rng.choice([h for h in hubs if h != self.hub])
            route = self.g.route(self.hub, other)
            if route:
                self.queue += route
                self.stay = self.rng.randint(*self.hub_stay)
                return
        self.queue.append(self.sel.pick(self.hub))

    def _advance(self):
        self.history.append((round(self.clock, 3), self.seg.name))
        self.hub = self.seg.end if self.seg.end in self.g.hubs else self.hub
        if self.paused and not self.queue and not self.pending and self.seg.end in self.g.idle_hubs():
            self.state = "PAUSED"
            return
        if not self.queue:
            self._plan_next()
        nxt = self.queue.pop(0)
        if isinstance(nxt, tuple):                              # a sequence's clip, chosen now
            _, name, hub, mood = nxt
            nxt = self.sel.pick(hub, mood)
        self.log("%.2f %s -> %s" % (self.clock, self.seg.name, nxt.name))
        self.seg, self.seg_t = nxt, 0.0
        self.state = STATE_OF[nxt.kind]

    def step(self, dt):
        self.clock += dt
        if self.state in ("FAULT", "HOLD"):
            return self.seg.at(self.seg_t)
        if self.state == "PAUSED":
            if self.paused and not self.pending:
                return self.seg.at(self.seg_t)
            self.state = "IDLE"
            self._advance()
            return self.seg.at(self.seg_t)
        self.seg_t += dt
        while self.seg_t >= self.seg.duration:
            left = self.seg_t - self.seg.duration
            self._advance()
            if self.state == "PAUSED":
                self.seg_t = self.seg.duration
                break
            self.seg_t = left
        return self.seg.at(self.seg_t)

    def status(self):
        prog = self.seg_t / self.seg.duration if self.seg.duration > 0 else 1.0
        return {"state": self.state, "clip": self.seg.name, "hub": self.hub, "sequence": self.sequence,
                "progress": round(min(1.0, prog), 4),
                "scan": round(min(1.0, prog), 4) if self.seg.kind == "scan" else -1.0,
                "pending": list(self.pending), "fault": self.fault_reason}


def runner_for(graph, seed=None, log=None):
    sel = graph.info.get("select", {})
    return Runner(graph, Selector(graph.idle(), sel.get("no_repeat", 6), seed=seed),
                  hub_stay=tuple(sel.get("hub_stay", (2, 4))), seed=seed, log=log)


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
        if hasattr(runner, "stop"):                    # the streaming backend: a software stop
            d.map("/robot/stop", lambda a, *v: runner.stop())
        self.server = osc_server.ThreadingOSCUDPServer(("0.0.0.0", listen_port), d)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = udp_client.SimpleUDPClient(send_host, send_port)

    def send(self, q):
        s = self.runner.status()
        self.client.send_message("/robot/state", s["state"])
        self.client.send_message("/robot/clip", s["clip"])
        self.client.send_message("/robot/hub", s["hub"])
        self.client.send_message("/robot/progress", float(s["progress"]))
        self.client.send_message("/robot/scan", float(s["scan"]))
        self.client.send_message("/robot/joints", [float(x) for x in q])

    def close(self):
        self.server.shutdown()


# --------------------------------------------------------------------------
# dry run
# --------------------------------------------------------------------------

def dry_run(graph, minutes=10.0, trigger_every=45.0, dt=0.008, seed=1, log=print, triggers=("scan",)):
    """The show on a clock, no robot: a trigger (from triggers, in turn)
    every ~trigger_every s. Returns a report: continuity (largest joint step
    per tick), trigger latencies, clips played, hubs visited."""
    r = runner_for(graph, seed=seed, log=log)
    rng = random.Random(seed)
    next_trig = trigger_every * (0.5 + rng.random()) if trigger_every else float("inf")
    prev, worst, latencies, t_trig, k = r.step(0.0), 0.0, [], None, 0
    t, end = 0.0, minutes * 60.0
    while t < end:
        t += dt
        if t >= next_trig:
            r.trigger(triggers[k % len(triggers)])
            k += 1
            t_trig = t
            next_trig = t + trigger_every * (0.5 + rng.random())
        q = r.step(dt)
        worst = max(worst, max(abs(a - b) for a, b in zip(q, prev)))
        prev = q
        if t_trig is not None and r.sequence is not None and r.state != "IDLE" or (t_trig is not None and r.state == "SCAN"):
            latencies.append(t - t_trig)
            t_trig = None
    played = [n for _, n in r.history]
    idle = [n for n in played if graph_kind(graph, n) == "idle"]
    return {"minutes": minutes, "segments_played": len(played), "scans": played.count("scan"),
            "idle_clips_played": len(idle), "distinct_idle": len(set(idle)), "idle_available": len(graph.idle()),
            "hubs_visited": sorted(set(graph_start(graph, n) for n in idle)),
            "moves": sum(1 for n in played if graph_kind(graph, n) == "move"),
            "worst_step_deg_per_tick": round(worst, 4), "max_step_allowed": round(max_step(dt), 4),
            "trigger_to_start_s": {"max": round(max(latencies), 2) if latencies else None,
                                   "mean": round(sum(latencies) / len(latencies), 2) if latencies else None}}


def graph_kind(graph, name):
    return next((s.kind for s in graph.segments if s.name == name), None)


def graph_start(graph, name):
    return next((s.start for s in graph.segments if s.name == name), None)


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

    A = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    B = [30.0] + A[1:]
    S0, S1 = [10.0] + A[1:], [20.0] + A[1:]

    def seg(name, kind, q0, q1, start, end, dur=2.0, action="float", time_=-1.0):
        return Segment(name, kind, [0.0, dur / 2, dur], [q0, [(x + y) / 2 for x, y in zip(q0, q1)], q1], start, end,
                       {"action": action, "effort": {"time": time_}})

    segs = [seg("a1", "idle", A, A, "a", "a", 3.0, "punch", 1.0), seg("a2", "idle", A, A, "a", "a", 3.0),
            seg("b1", "idle", B, B, "b", "b", 3.0), seg("b2", "idle", B, B, "b", "b", 3.0, "punch", 1.0),
            seg("move_a_b", "move", A, B, "a", "b"), seg("move_b_a", "move", B, A, "b", "a"),
            seg("to_scan", "to_scan", A, S0, "a", "scan_start"), seg("scan", "scan", S0, S1, "scan_start", "scan_end"),
            seg("from_scan", "from_scan", S1, A, "scan_end", "a")]
    g = Graph({"a": A, "b": B, "scan_start": S0, "scan_end": S1}, segs,
              {"start_hub": "a", "sequences": {"greet": {"hub": "b", "count": 2, "mood": "punch"}}})
    check("segments meet at their hubs", not g.check_joins())
    check("routes between hubs", [s.name for s in g.route("b", "a")] == ["move_b_a"] and g.route("a", "a") == [])
    r = Runner(g, Selector(g.idle(), 1, seed=0), hub_stay=(2, 2), seed=0)
    hubs = set()
    prev = r.step(0.0)
    jump = 0.0
    for _ in range(2000):
        q = r.step(0.05)
        jump = max(jump, max(abs(x - y) for x, y in zip(q, prev)))
        prev = q
        if r.seg.kind == "idle":
            hubs.add(r.seg.start)
    check("idling visits every hub, moving between them", hubs == {"a", "b"}, hubs)
    check("no jump between segments", jump < 20.0 * 0.05 * 2, round(jump, 3))
    # a scan from hub b: route to a, then to_scan, scan, from_scan
    r = Runner(g, Selector(g.idle(), 1, seed=0), hub_stay=(99, 99), seed=0, start_hub="b")
    r.seg = g.idle("b")[0]
    r.hub = "b"
    r.step(1.0)
    r.trigger("scan")
    seen = []
    for _ in range(400):
        r.step(0.05)
        if not seen or seen[-1] != r.seg.name:
            seen.append(r.seg.name)
    check("scan from another hub: move there first, then to_scan, scan, from_scan",
          seen[1:5] == ["move_b_a", "to_scan", "scan", "from_scan"], seen[:6])
    # a sequence: to its hub, its clips with its mood
    r = Runner(g, Selector(g.idle(), 0, seed=4), hub_stay=(99, 99), seed=0)
    r.step(0.5)
    r.trigger("greet")
    seen = []
    for _ in range(300):
        r.step(0.05)
        if not seen or seen[-1] != r.seg.name:
            seen.append(r.seg.name)
    check("a sequence goes to its hub and plays its clips there", seen[1] == "move_a_b" and
          all(x.startswith("b") for x in seen[2:4]), seen[:5])
    r.trigger("nonsense")
    check("an unknown trigger is ignored", "nonsense" not in r.pending)
    r.pause()
    for _ in range(400):
        r.step(0.05)
    check("pause holds at a hub after the running clip", r.state == "PAUSED" and r.seg.end in ("a", "b"), r.state)
    r.resume()
    r.step(0.05)
    check("resume goes on", r.state != "PAUSED", r.state)
    r.fault("test")
    q = r.step(0.5)
    check("a fault holds the pose", r.state == "FAULT" and q == r.step(0.5))
    sel = Selector(g.idle(), no_repeat=1, seed=3)
    picks = [sel.pick("a").name for _ in range(50)]
    check("no repeat within the window", all(x != y for x, y in zip(picks, picks[1:])), picks[:6])
    sel = Selector(g.idle(), no_repeat=0, seed=3)
    sel.mood = "punch"
    picks = [sel.pick("a").name for _ in range(400)]
    check("a mood makes its clips likelier", picks.count("a1") > 2 * picks.count("a2"), (picks.count("a1"), picks.count("a2")))
    sel = Selector(g.idle(), no_repeat=0, seed=3)
    sel.energy = 1.0
    picks = [sel.pick("a").name for _ in range(400)]
    check("energy 1 prefers sudden clips", picks.count("a1") > 2 * picks.count("a2"), (picks.count("a1"), picks.count("a2")))
    rng_cfg = {"range": {"j1_deg": [-30, 30], "tcp_z": [0.5, 1.5]}}
    check("the operating range refuses J1 past its sector",
          out_of_range(rng_cfg, [[0, 0, 0, 0, 0, 0], [40, 0, 0, 0, 0, 0]], None) is not None)
    check("... and a TCP too high", out_of_range(rng_cfg, [[0] * 6], [(0, 0, 1.7)]) is not None)
    check("... and lets a motion inside it through", out_of_range(rng_cfg, [[10] * 6], [(0, 0, 1.0)]) is None)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=("build", "dry-run", "osc"))
    ap.add_argument("config")
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--trigger-every", type=float, default=45.0)
    ap.add_argument("--triggers", default="scan", help="comma separated, used in turn")
    a = ap.parse_args(argv)
    cfg_path = os.path.abspath(a.config)
    if a.command == "build":
        t0 = time.time()
        g = build(cfg_path)
        g.save(compiled_path(cfg_path))
        name = os.path.splitext(os.path.basename(cfg_path))[0]
        write_preview(g, os.path.join(ROOT, "geo", "show", name))
        print("wrote %s: %d segments (%d idle at %s), hubs %s, %.0f s" % (
            os.path.relpath(compiled_path(cfg_path), ROOT), len(g.segments), len(g.idle()), g.idle_hubs(),
            list(g.hubs), time.time() - t0))
        return 0
    g = Graph.load(compiled_path(cfg_path))
    if a.command == "dry-run":
        rep = dry_run(g, a.minutes, a.trigger_every, log=lambda *x: None, triggers=a.triggers.split(","))
        print(json.dumps(rep, indent=1))
        return 0 if rep["worst_step_deg_per_tick"] <= rep["max_step_allowed"] else 1
    cfg = json.load(open(cfg_path))
    r = runner_for(g, log=print)
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
