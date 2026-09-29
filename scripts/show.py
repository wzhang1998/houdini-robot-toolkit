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
    hub --move--> other hub           a checked move (safe_move.route, timed by Ruckig: transitions.py)
    scan hub --to_scan--> scan_start --scan--> scan_end --from_scan--> scan hub
                                      one pass across the paper at an even speed (scan_line),
                                      in and out at each end's own approach (scan_way)

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
energy (0 calm .. 1 lively). Every idle clip has a measured energy (how fast
and how big it moves, ranked within the library). Unless TouchDesigner sets
one, the energy follows the show's arc (select.arc): it builds from calm to a
peak over period_s, bursts, and drops back -- so the show breathes instead of
playing clips of one kind. The family just played is avoided, and at high
energy the arm changes hub (level) more often.

OSC in:  /robot/trigger [name] (default scan)   /robot/pause   /robot/resume   /robot/reset
         /robot/mood <action>   /robot/energy <0..1> (below 0: back to the arc)
OSC out: /robot/state s  /robot/clip s  /robot/hub s  /robot/progress f
         /robot/scan f (0..1 of the scan's time)  /robot/joints f*6
         /robot/scan/u f (across the opening, 0..1: the LEDs' column)  /robot/scan/led i  /robot/scan/speed f

Backends (who moves): the dry-run clock here; Isaac Sim
(scripts/isaac/run_show.py); SimMachine / the real FR20 next (a ServoJ
stream over the same step()).
"""

import argparse
import bisect
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
        segs = [Segment.from_dict(s) for s in d["segments"]]
        if any(s.kind == "idle" and "energy" not in (s.labels or {}) for s in segs):
            add_energy(segs)                           # compiled before clips carried their energy
        return Graph(d["hubs"], segs, d.get("info"))

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

def canvas_parts(c):
    """What stands at the paper, as boxes: [{name, kind ("canvas" / "wood"),
    center, size (depth along the normal, width, height), yaw_deg}]. The
    canvas c["size"] x c["thickness"] at c["center"]; its frame c["frame"]
    {outer [w, h], face_width, depth}: four rails centred on the canvas, in
    front of it (towards the robot) and over its edges -- the user's frame,
    2026-09-28: 67 x 46 in, 5.25 in rails, the canvas behind them. Left and
    right as seen from the robot's side, facing the paper."""
    n = c["normal"]
    yaw = math.degrees(math.atan2(n[1], n[0]))
    u = (-n[1], n[0], 0.0)                                    # along the width, to the left
    t = c.get("thickness", 0.02)

    def box(name, kind, dx, dy, dz, size):
        at = [c["center"][i] + n[i] * dx + u[i] * dy for i in range(3)]
        at[2] += dz
        return {"name": name, "kind": kind, "center": at, "size": list(size), "yaw_deg": yaw}
    out = [box("canvas", "canvas", 0.0, 0.0, 0.0, (t, c["size"][0], c["size"][1]))]
    f = c.get("frame")
    if f:
        (w, h), fw, d = f["outer"], f["face_width"], f["depth"]
        dx = -t / 2.0 - d / 2.0                               # the rails' backs on the canvas's face
        out += [box("frame_top", "wood", dx, 0.0, h / 2.0 - fw / 2.0, (d, w, fw)),
                box("frame_bottom", "wood", dx, 0.0, -(h / 2.0 - fw / 2.0), (d, w, fw)),
                box("frame_left", "wood", dx, w / 2.0 - fw / 2.0, 0.0, (d, fw, h - 2 * fw)),
                box("frame_right", "wood", dx, -(w / 2.0 - fw / 2.0), 0.0, (d, fw, h - 2 * fw))]
    return out


def canvas_boxes(c):
    """The paper in its frame as box obstacles for collision.py."""
    return [{"name": p["name"], "type": "box", "center": p["center"], "role": "obstacle", "size": p["size"],
             "yaw_deg": p["yaw_deg"]} for p in canvas_parts(c)]


def show_env(env, cfg, canvas_margin):
    """The room for this show: the config's stage replaces the env's work zone
    of that name; the paper is an obstacle with canvas_margin; the ceiling
    keeps margins.ceiling_m from every motion (a sprinkler hangs from its
    centre, 2026-09-28)."""
    objs = []
    stage = cfg.get("stage")
    ceiling_m = (cfg.get("margins") or {}).get("ceiling_m")
    for o in env["objects"]:
        if stage and o["name"] == stage.get("name", "stage"):
            o = dict(o, center=list(stage["center"]), size=list(stage["size"]), yaw_deg=stage.get("yaw_deg", 0.0))
        if ceiling_m and o["type"] == "halfspace" and o["normal"][2] < -0.9:
            o = dict(o, margin_m=max(ceiling_m, o.get("margin_m") or 0.0))
        objs.append(o)
    if cfg.get("canvas"):
        objs += [dict(o, margin_m=canvas_margin) for o in canvas_boxes(cfg["canvas"])]
    return dict(env, objects=objs)


def timed_move(waypoints, safety, vel, acc, jerk=None, max_dev_deg=None):
    """A joint move through waypoints, at rest at both ends: straight legs
    (as the controller's MoveJ), blended past the corners, jerk-limited by
    Ruckig (transitions.move; jerk None = acc / 0.2 s). TOPP was used until
    2026-09-27: time-optimal but not jerk-limited, its 125 Hz stream went
    over the acceleration limit (170 vs 150 deg/s^2 on the hub moves)."""
    import transitions
    if max_dev_deg is None:
        return transitions.move(waypoints, vel, acc, jerk, safety)
    return transitions.move(waypoints, vel, acc, jerk, safety, max_dev_deg=max_dev_deg)


BLENDS_DEG = (2.0, 0.5, 0.0)       # how far a move may cut its corners, tried in turn until it checks clear


def checked_move(waypoints, safety, vel, acc, model, env):
    """timed_move through the route's waypoints, checked in env: the route
    (safe_move) is clear along its straight legs, but a blended corner
    leaves them by up to max_dev_deg -- so a move that fails its check is
    timed again with tighter corners, down to stopping at each (0: the legs
    exactly). (t, q, report)."""
    import collision as C
    for dev in BLENDS_DEG:
        t, q = timed_move(waypoints, safety, vel, acc, max_dev_deg=dev)
        rep = C.check(model, env, t, q)
        if rep["ok"]:
            break
    return t, q, rep


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


def family_maker(fam):
    """The generator of a family: a spatial path (paths.py) or a gesture
    (gestures.py); both make(rig, hub_q, family, zones, rng, bpm, intensity,
    env=, safety=) -> clip or None."""
    import gestures as G
    import paths as PA
    if fam in PA.FAMILIES:
        return PA.make
    if fam in G.FAMILIES:
        return G.make
    raise SystemExit("family %r: not a gesture %s or a path %s" % (fam, G.FAMILIES, PA.FAMILIES))


def gesture_clips(name, hub, h, cfg, env, rig, rng, log=print):
    """Clips from a hub's families (gestures.py, paths.py), in turn, towards
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
        clip = family_maker(fam)(rig, hub, fam, cfg["zones"], rng, bpm=bpm, intensity=k, env=env,
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
        if (clip.get("labels") or {}).get("params"):
            labels["params"] = clip["labels"]["params"]
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

    # showpieces (showpiece.py): big wipes along a wall, from a hub and back
    if cfg.get("showpieces"):
        import showpiece
        for sp in cfg["showpieces"]:
            seg = showpiece.make(sp, hubs, idle_env, cfg, log)
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
            t, q, rep = checked_move([hubs[a]] + path, cfg["transition_safety"], vel, acc, model, idle_env)
            if not rep["ok"]:
                log("  move %s -> %s refused: %s" % (a, b, C.describe(rep)))
                continue
            segs.append(Segment("move_%s_%s" % (a, b), "move", t, q, a, b))

    # the scan: one pass, left to right, and the moves to and from it. In
    # and out as a painting cell does: the checked route ends at a pose
    # approach_m back from the scan's start (the same tool attitude, clear
    # by the moves' margins), then straight in; at the end straight back out
    # to the end's own approach, then a checked route home -- never back over
    # the paper, which a second pass would expose again (the user, 2026-09-28)
    if cfg.get("scan"):
        st, sq, slab = scan_line(cfg, scan_env, prof)
        hubs["scan_start"], hubs["scan_end"] = list(sq[0]), list(sq[-1])
        segs.append(Segment("scan", "scan", st, sq, "scan_start", "scan_end", slab))
        log("scan: %.2f m/s over the %.2f m opening, LEDs on %.2f-%.2f s of %.2f s, %.3f m clear"
            % (slab["speed_mps"], slab["exposed_m"], slab["led_on_s"][0], slab["led_on_s"][1], st[-1],
               slab["clearance_m"]))
        home = cfg["scan"]["from_hub"]
        back = cfg["scan"].get("approach_m", 0.2)
        backs = [back + 0.1 * k for k in range(4)]
        way = slab["travel"]
        ends = {}
        for end, inward, out in (("scan_start", way, False), ("scan_end", [-x for x in way], True)):
            got = scan_way(rig, hubs[end], cfg["canvas"]["normal"], backs, inward, model, scan_env, hubs[home], out)
            if got is None:
                raise SystemExit("scan: no approach at its %s with a clear route %s %s"
                                 % (end.replace("scan_", ""), "to" if out else "from", home))
            ends[end] = got
            log("approach at the scan's %s: %.2f m back, %.2f m down, %.2f m inward; %s"
                % (end.replace("scan_", ""), got[1][0], got[1][1], got[1][2], got[3]))
        _, _, path, _ = ends["scan_start"]
        t, q, rep = checked_move([hubs[home]] + path + [hubs["scan_start"]], cfg["transition_safety"], vel, acc,
                                 model, scan_env)
        if not rep["ok"]:
            raise SystemExit("to_scan: %s" % C.describe(rep))
        segs.append(Segment("to_scan", "to_scan", t, q, home, "scan_start"))
        log("to_scan: %.1f s" % t[-1])
        pe, _, path, _ = ends["scan_end"]
        t, q, rep = checked_move([hubs["scan_end"], pe] + path, cfg["transition_safety"], vel, acc, model, scan_env)
        if not rep["ok"]:
            raise SystemExit("from_scan: %s" % C.describe(rep))
        log("from_scan: %.1f s" % t[-1])
        segs.append(Segment("from_scan", "from_scan", t, q, "scan_end", home))
    add_energy(segs)
    g = Graph(hubs, segs, {"config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"),
                           "built": time.strftime("%Y-%m-%d %H:%M"), "start_hub": cfg["start_hub"],
                           "inputs": input_digests(cfg_path),
                           "dropped": dropped, "canvas": cfg.get("canvas"), "stage": cfg.get("stage"),
                           "sequences": cfg.get("sequences", {}), "select": cfg.get("select", {})})
    bad = g.check_joins()
    if bad:
        raise SystemExit("segments do not meet at their hubs: %s" % bad)
    out = limit_breaches(segs, RP.motion_limits(prof))
    if out:
        raise SystemExit("segments outside the motion limits (J6: the tool cable's range): %s" % out)
    unreachable = [h for h in g.idle_hubs() if g.route(cfg["start_hub"], h) is None]
    if unreachable:
        raise SystemExit("hubs not reachable from %s: %s" % (cfg["start_hub"], unreachable))
    return g


def limit_breaches(segments, limits):
    """[(segment, joint 1-6, the furthest value outside)] of every segment
    that leaves limits -- robot_profile.motion_limits: J6 inside the tool
    cable's range."""
    out = []
    for s in segments:
        for j, (lo, hi) in enumerate(limits):
            vals = [q[j] for q in s.q]
            worst = max(vals, key=lambda v: max(lo - v, v - hi))
            if worst < lo - 1e-6 or worst > hi + 1e-6:
                out.append((s.name, j + 1, round(worst, 1)))
    return out


def approach_pose(rig, q, normal, back, down=0.0, side=(0.0, 0.0, 0.0)):
    """Joints with the tool back m further from the paper along its normal
    (-normal), down m lower and moved by side, the tool's attitude kept,
    nearest q; None if not reachable."""
    R, tcp, _ = rig.tool(q)
    p = [tcp[i] - normal[i] * back + side[i] for i in range(3)]
    p[2] -= down
    return rig.solve(p, (R[0][2], R[1][2], R[2][2]), 0.0, q, R=R)


def scan_way(rig, q, normal, backs, inward, model, env, home, out, grid=True):
    """The approach at a scan end and the checked route between it and home
    (out: from the approach to home, else home to it). Candidates back
    (backs, m) from the paper, lower (0.1 m steps) and slid inward (0.1 m
    steps towards the scan's middle; the LEDs are off there), clear by the
    moves' margins, the gentlest first. Three rounds over them, each only
    when the ones before found nothing, each taking its first:
      1. a straight MoveJ, or one after a key pose where J6 alone turns the
         strip level (it turns parallel to the paper, 0.2 m and more from
         it; the upright strip then swings no rail over);
      2. cuRobo's detour, when its service runs (the shortest, not folded);
      3. grid=True: the grid's detour -- folded postures, the arm laid
         towards the floor (2026-09-28), so last.
    (The build spent most of its 33 min running the grid on every
    candidate while looking for a straight one, 2026-09-29.)
    (approach, (back, down, inward), path, why) or None."""
    import curobo_bridge
    import safe_move
    menv = safe_move.move_env(env)

    def route(a, **kw):
        return safe_move.route(a, home, env, model, **kw) if out else safe_move.route(home, a, env, model, **kw)

    cands = []
    for b, d, s in sorted(((b, d, s) for b in backs for d in (0.0, 0.1, 0.2, 0.3) for s in (0.0, 0.1, 0.2, 0.3)),
                          key=lambda c: (c[0] + c[1] + c[2], c)):
        a = approach_pose(rig, q, normal, b, d, [x * s for x in inward])
        if a is not None and not safe_move.blocked(model, menv, a):
            cands.append((a, (b, d, s)))
    for a, key in cands:                                   # 1: straight, or straight after J6
        path, why = route(a, grid=False, curobo=False)
        if path is not None:
            return a, key, path, why
        for j6 in (-90.0, 90.0, home[5]):
            v = list(a[:5]) + [j6]
            if safe_move.blocked(model, menv, v) or safe_move.segment_clear(model, menv, a, v) is not None:
                continue
            p2, why2 = route(v, grid=False, curobo=False)
            if p2 is not None:
                return (a, key, [v] + p2 if out else p2 + [a],
                        "the strip turned level at J6 %.0f there, then %s" % (j6, why2))
    if curobo_bridge.available():                          # 2: cuRobo
        for a, key in cands:
            path, why = route(a, grid=False)
            if path is not None:
                return a, key, path, why
    if grid:                                               # 3: the grid
        for a, key in cands:
            path, why = route(a, curobo=False)
            if path is not None:
                return a, key, path, why
    return None


def scan_line(cfg, env, prof, dt=0.016):
    """The scan (scan_ends, scan_profile): IK along the line with the tool
    level and pointing at the paper (roll 0 keeps the LED strip, along the
    flange's y, upright), inside the joint limits at scan.safety of them and
    clear of the room. (times, joints, labels); labels["led_on_s"]: when the
    strip is over the opening -- light it only then. SystemExit when it
    cannot be done, saying why."""
    import capability as CAP
    import clip_factory as CF
    import collision as C
    import fairino_player as P
    tool = C.tool_def(prof)
    tool_z = (tool or {}).get("tcp", {}).get("xyz", [0.0, 0.0, 0.0])[2] if tool and tool.get("tcp") else 0.0
    strip = C.strip_box(tool)
    strip_w = strip["size"][0] if strip else 0.0254                  # across the scan
    start, end, info = scan_ends(cfg, tool_z, strip_w)
    s = cfg["scan"]
    v, lead = s["speed_mps"], s.get("lead_m", 0.05)
    ts, ss, (t_on, t_off) = scan_profile(info["cruise_m"], v, s.get("accel_mps2", 0.5), dt)
    d = info["direction"]
    pts = [tuple(start[i] + d[i] * x for i in range(3)) for x in ss]
    model, chain, fo, vel, acc = CF._model()
    R = CAP.tool_frame(cfg["canvas"]["normal"], 0.0)
    try:
        qs = CF._solve_along(model, R, pts, fo, list(CF.REFERENCE), 20.0)
    except CF.Rejected as e:
        raise SystemExit("scan: %s (from %s to %s)" % (e, [round(x, 3) for x in start], [round(x, 3) for x in end]))
    for k in range(1, len(qs)):
        qs[k] = [b - 360.0 * round((b - a) / 360.0) for a, b in zip(qs[k - 1], qs[k])]
    safety = s.get("safety", 0.5)
    lim = P.limiting(ts, qs, 125.0, [x * safety for x in vel], [x * safety for x in acc])
    if lim["scale_needed"] > 1.0 + 1e-3:
        raise SystemExit("scan: %.2f m/s asks J%d for %.2fx its %s at safety %.2f -- slower (scan.speed_mps) or a "
                         "gentler ramp (scan.accel_mps2)" % (v, lim["joint"], lim["scale_needed"], lim["kind"], safety))
    rep = C.check(C.load_model("fr20"), env, ts, qs)
    if not rep["ok"]:
        raise SystemExit("scan: %s" % C.describe(rep))
    labels = {"speed_mps": v, "exposed_m": round(info["exposed_m"], 4), "direction": s.get("direction", "left_to_right"),
              "led_gap_m": s["led_gap_m"], "led_on_s": [round(t_on + lead / v, 4), round(t_off - lead / v, 4)],
              "u": [round(u, 5) for u in scan_positions(ss, info, lead, strip_w)],
              "travel": [round(x, 6) for x in d],
              "clearance_m": rep["min_env_clearance_m"]}
    return ts, [list(q) for q in qs], labels


def scan_profile(cruise_m, v, a, dt):
    """The scan's time law: from rest up to v, cruise_m at exactly v, back to
    rest -- the ramps with a sine-shaped acceleration (no jerk step), their
    peak a. An exposure wants an even speed wherever the light falls, so the
    ramps are run outside it. (times, distance along, (cruise start, end))."""
    ta = math.pi * v / (2.0 * a)                          # peak of v*pi/(2 ta) = a
    ramp = v * ta / 2.0
    tc = cruise_m / v
    total = 2 * ta + tc
    n = max(2, int(math.ceil(total / dt)))
    ts, ss = [], []
    for k in range(n + 1):
        t = total * k / n
        if t < ta:
            s = v * (t / 2.0 - ta / (2 * math.pi) * math.sin(math.pi * t / ta))
        elif t <= ta + tc:
            s = ramp + v * (t - ta)
        else:
            r = total - t
            s = 2 * ramp + cruise_m - v * (r / 2.0 - ta / (2 * math.pi) * math.sin(math.pi * r / ta))
        ts.append(t)
        ss.append(s)
    return ts, ss, (ta, ta + tc)


def scan_positions(ss, info, lead, strip_w):
    """Each scan sample's place across the frame's opening (u): 0 with the
    strip's centre on its left edge, 1 on its right, below 0 / above 1 on the
    ramps -- the LEDs light while 0 <= u <= 1, the image's column is u."""
    start = info["ramp_m"] + lead + strip_w / 2.0
    return [(s - start) / info["exposed_m"] for s in ss]


def canvas_opening(c):
    """The paper's width that shows between the frame's rails (m)."""
    f = c.get("frame")
    return min(c["size"][0], f["outer"][0] - 2 * f["face_width"]) if f else c["size"][0]


def scan_ends(cfg, tool_z, strip_w=0.0254):
    """(start, end, info) of the scan's TCP line (the flange's working point;
    the LED face tool_z beyond it): level, along the paper's width, the LED
    face scan.led_gap_m from the paper's face, left to right as seen from the
    robot's side facing it (scan.direction) -- one pass, so the paper is
    exposed once. The line covers the opening, the strip's width and
    scan.lead_m each side at the cruise speed, then the ramps (scan_profile)."""
    c, s = cfg["canvas"], cfg["scan"]
    n = c["normal"]
    right = (n[1], -n[0], 0.0)
    way = 1.0 if s.get("direction", "left_to_right") == "left_to_right" else -1.0
    standoff = s["led_gap_m"] + c.get("thickness", 0.02) / 2.0 + tool_z
    centre = [c["center"][i] - n[i] * standoff for i in range(3)]
    opening = canvas_opening(c)
    cruise = opening + strip_w + 2 * s.get("lead_m", 0.05)
    v, a = s["speed_mps"], s.get("accel_mps2", 0.5)
    ramp = v * (math.pi * v / (2.0 * a)) / 2.0
    half = cruise / 2.0 + ramp
    start = [centre[i] - way * right[i] * half for i in range(3)]
    end = [centre[i] + way * right[i] * half for i in range(3)]
    return start, end, {"exposed_m": opening, "cruise_m": cruise, "ramp_m": ramp, "speed_mps": v,
                        "direction": [way * x for x in right]}


def _canonical(x):
    """A JSON value without its notes (keys starting with _), keys sorted:
    what a build depends on, not how the file is laid out or annotated."""
    if isinstance(x, dict):
        return {k: _canonical(v) for k, v in sorted(x.items()) if not str(k).startswith("_")}
    if isinstance(x, list):
        return [_canonical(v) for v in x]
    return x


def input_digests(cfg_path, root=ROOT):
    """{input: sha256} of what a build reads: the show config and the robot
    profile (as JSON, notes left out), the profile's tool URDF and the room
    (bytes). Kept in the compiled show's info["inputs"]."""
    import hashlib

    def h(data):
        return hashlib.sha256(data).hexdigest()[:16]

    def of_json(path):
        return h(json.dumps(_canonical(json.load(open(path, encoding="utf8"))), sort_keys=True).encode())

    def of_file(path):
        return h(open(path, "rb").read()) if path and os.path.exists(path) else None
    cfg = json.load(open(cfg_path, encoding="utf8"))
    prof_path = os.path.join(root, "profiles", "fr20.json")
    prof = json.load(open(prof_path, encoding="utf8"))
    tool = (prof.get("tool") or {}).get("urdf")
    return {"config": of_json(cfg_path), "profile": of_json(prof_path),
            "tool": of_file(os.path.join(root, tool) if tool else None),
            "env": of_file(os.path.join(root, cfg["env"]))}


def stale_inputs(info, cfg_path, root=ROOT):
    """The inputs that changed since the compiled show (its info) was built,
    by name; ["inputs not recorded"] for a show built before they were."""
    was = (info or {}).get("inputs")
    if not was:
        return ["inputs not recorded"]
    now = input_digests(cfg_path, root)
    return [k for k in sorted(now) if now[k] != was.get(k)]


def require_fresh(graph, cfg_path):
    """SystemExit, saying what to do, when the compiled show is out of date
    against its inputs -- a clip past the cable's J6 range streamed from a
    stale file faults mid-show (the review, 2026-09-28)."""
    changed = stale_inputs(graph.info, cfg_path)
    if changed:
        why = ("built before its inputs were recorded" if changed == ["inputs not recorded"]
               else "%s changed since it was built" % ", ".join(changed))
        raise SystemExit("%s is out of date (%s): uv run scripts/show.py build %s"
                         % (compiled_path(cfg_path), why, os.path.relpath(cfg_path, ROOT)))


def compiled_path(cfg_path):
    return os.path.splitext(cfg_path)[0] + ".compiled.json"


_WATCH = {}


def watched_points(robot="fr20"):
    """(chain, points in the last link's frame) a guest's eye follows: the
    TCP, and the ends of the mounted tool's longest part (the LED strip's
    tips, collision.tool_capsules) -- a strip spun by J6 alone is lively
    though the TCP stands still."""
    if robot not in _WATCH:
        import collision as C
        import robot_profile as RP
        import urdf_rig as U
        prof = RP.load(robot)
        fo = float(prof["rig"].get("flange_offset_m", 0.0))
        pts = [(0.0, 0.0, fo)]
        strip = C.strip_capsule(C.tool_capsules(C.tool_def(prof), fo))
        if strip:
            pts += [strip["a"], strip["b"]]
        _WATCH[robot] = (U.parse_urdf(os.path.join(ROOT, prof["rig"]["urdf"]))["chain"], pts)
    return _WATCH[robot][1]


def motion_stats(t, q, watch=None, robot="fr20"):
    """How a clip looks from outside: the TCP's height span and range; the
    largest extent and the mean and peak speed (m/s) of the watched points
    (watched_points: the TCP and the tool's tips), the liveliest of them;
    the duration (s)."""
    import ur_ik
    import urdf_rig as U
    watch = watch or watched_points(robot)
    chain = _WATCH[robot][0]
    paths = [[] for _ in watch]
    for qk in q:
        R, p6 = ur_ik.pose_of(chain, qk)
        for path, w in zip(paths, watch):
            path.append(U._add(p6, U._mat_vec(R, w)))
    z = [p[2] for p in paths[0]]
    ext, vm, vp = 0.0, 0.0, 0.0
    for path in paths:
        ext = max(ext, max(max(p[i] for p in path) - min(p[i] for p in path) for i in range(3)))
        sp = [math.dist(a, b) / (t1 - t0) for a, b, t0, t1 in zip(path, path[1:], t, t[1:]) if t1 > t0]
        if sp:
            vm, vp = max(vm, sum(sp) / len(sp)), max(vp, max(sp))
    return {"z_min": round(min(z), 3), "z_max": round(max(z), 3), "z_span": round(max(z) - min(z), 3),
            "extent": round(ext, 3), "v_mean": round(vm, 3), "v_peak": round(vp, 3), "duration": round(t[-1], 2)}


def add_energy(segments):
    """labels["stats"] and labels["energy"] (0 calm .. 1 lively) on every
    idle segment: speed and size, ranked within the library, so the energy
    arc always has clips at both ends."""
    idle = [s for s in segments if s.kind == "idle"]
    for s in idle:
        s.labels = dict(s.labels or {})
        s.labels["stats"] = motion_stats(s.t, s.q)
    if not idle:
        return
    def rank(key):
        vals = sorted(s.labels["stats"][key] for s in idle)
        return {id(s): (vals.index(s.labels["stats"][key]) / max(1, len(vals) - 1)) for s in idle}
    vm, vp, ex = rank("v_mean"), rank("v_peak"), rank("extent")
    raw = {id(s): 0.45 * vm[id(s)] + 0.35 * vp[id(s)] + 0.2 * ex[id(s)] for s in idle}
    order = sorted(raw.values())
    for s in idle:
        s.labels["energy"] = round(order.index(raw[id(s)]) / max(1, len(order) - 1), 3)


def family_of(seg):
    """What kind of clip it is, for contrast: a gesture's or a path's family,
    a dance's measured action."""
    lab = seg.labels or {}
    return lab.get("family") or lab.get("action") or seg.name


def report(graph, cfg=None):
    """The library as numbers (motion_stats per idle clip) and whether it
    meets the variety the show asks for (cfg["library"]["variety"], or the
    defaults): (rows, summary, [targets missed])."""
    idle = graph.idle()
    if any("stats" not in (s.labels or {}) for s in idle):
        add_energy(graph.segments)
    rows = [dict(name=s.name, hub=s.start, family=family_of(s), energy=s.labels.get("energy"), **s.labels["stats"])
            for s in idle]
    med = lambda xs: sorted(xs)[len(xs) // 2] if xs else 0.0
    zs = [r["z_min"] for r in rows] + [r["z_max"] for r in rows]
    summary = {"clips": len(rows), "families": len({r["family"] for r in rows}),
               "z_range": [min(zs), max(zs)] if zs else None,
               "z_span_median": med([r["z_span"] for r in rows]), "extent_median": med([r["extent"] for r in rows]),
               "v_peak_max": max((r["v_peak"] for r in rows), default=0.0),
               "v_peak_min": min((r["v_peak"] for r in rows), default=0.0),
               "short_share": round(sum(r["duration"] <= 5.0 for r in rows) / max(1, len(rows)), 2),
               "duration_range": [min(r["duration"] for r in rows), max(r["duration"] for r in rows)] if rows else None}
    want = dict(VARIETY, **((cfg or {}).get("library", {}).get("variety") or {}))
    missed = []
    if summary["z_span_median"] < want["z_span_median"]:
        missed.append("median height change %.2f m < %.2f" % (summary["z_span_median"], want["z_span_median"]))
    if summary["extent_median"] < want["extent_median"]:
        missed.append("median extent %.2f m < %.2f" % (summary["extent_median"], want["extent_median"]))
    if zs and summary["z_range"][1] - summary["z_range"][0] < want["z_range_m"]:
        missed.append("heights used %.2f-%.2f m, less than %.2f m apart" % (summary["z_range"][0], summary["z_range"][1],
                                                                           want["z_range_m"]))
    if summary["short_share"] < want["short_share"]:
        missed.append("%.0f%% of clips <= 5 s < %.0f%%" % (100 * summary["short_share"], 100 * want["short_share"]))
    if summary["v_peak_max"] < want["v_peak_fast"]:
        missed.append("fastest clip %.2f m/s < %.2f" % (summary["v_peak_max"], want["v_peak_fast"]))
    return rows, summary, missed


VARIETY = {"z_span_median": 0.15, "extent_median": 0.3, "z_range_m": 0.8, "short_share": 0.2, "v_peak_fast": 1.0}


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

ARC = {"period_s": 180.0, "calm": 0.1, "peak": 1.0, "burst_s": 20.0}


class Selector:
    """Next idle clip at a hub: weighted random, no repeat within the last
    no_repeat, towards a mood (Laban action) and an energy -- the operator's
    (TouchDesigner) or else the arc's -- and away from the family just played."""

    def __init__(self, clips, no_repeat=6, seed=None, arc=None):
        self.clips, self.no_repeat = clips, no_repeat
        self.recent, self.mood, self.energy = [], None, None
        self.arc = dict(ARC, **arc) if isinstance(arc, dict) else (ARC if arc is None else None)
        self.rng = random.Random(seed)

    def target_energy(self, clock=0.0):
        """The energy wanted now: the operator's, or the arc's -- a build from
        calm to the peak over period_s, a burst at the peak for burst_s, then
        calm again. None: no preference."""
        if self.energy is not None and self.energy >= 0.0:
            return self.energy
        a = self.arc
        if not a:
            return None
        u = (clock % a["period_s"]) / a["period_s"]
        burst = a["burst_s"] / a["period_s"]
        if u >= 1.0 - burst:
            return a["peak"]
        return a["calm"] + (a["peak"] - a["calm"]) * (u / (1.0 - burst)) ** 1.5 * 0.85

    def weight(self, c, mood=None, clock=0.0):
        w = 1.0
        mood = mood or self.mood
        if mood and c.labels.get("action") == mood:
            w *= 4.0
        want = self.target_energy(clock)
        e = c.labels.get("energy")
        if e is None and (c.labels.get("effort") or {}).get("time") is not None:
            e = (c.labels["effort"]["time"] + 1.0) / 2.0          # no measured energy: sudden = lively
        if want is not None and e is not None:
            w *= 0.1 + 2.0 * math.exp(-((e - want) / 0.22) ** 2)
        fams = [family_of(x) for x in self.recent_segs[-3:]]
        if fams and family_of(c) == fams[-1]:
            w *= 0.15                                             # not the same kind twice in a row
        elif family_of(c) in fams:
            w *= 0.5
        return w

    recent_segs = ()

    def pick(self, hub=None, mood=None, clock=0.0):
        at = [c for c in self.clips if hub is None or c.start == hub]
        pool = [c for c in at if c.name not in self.recent[-self.no_repeat:]] or at
        c = self.rng.choices(pool, weights=[self.weight(x, mood, clock) for x in pool])[0]
        self.recent.append(c.name)
        self.recent_segs = list(self.recent_segs)[-5:] + [c]
        return c


class Runner:
    """Walks the graph one step at a time: step(dt) -> joints (degrees)."""

    def __init__(self, graph, selector, start_hub=None, hub_stay=(2, 4), sequences=None, seed=None, log=None,
                 scan_speed=1.0):
        if not 0.0 < scan_speed <= 1.0:
            raise ValueError("scan_speed %r: the scan plays at most as fast as built (0 < f <= 1)" % scan_speed)
        self.scan_speed = scan_speed                  # the scan only, for tuning an exposure; it starts and ends still
        self.g, self.sel, self.log = graph, selector, log or (lambda *a: None)
        self.hub = start_hub or graph.info.get("start_hub") or graph.idle_hubs()[0]
        self.hub_stay, self.sequences = hub_stay, sequences if sequences is not None else graph.info.get("sequences", {})
        self.rng = random.Random(seed)
        self.pending, self.queue, self.paused, self.fault_reason = [], [], False, None
        self.clock, self.seg_t, self.sequence = 0.0, 0.0, None
        self.stay = self.rng.randint(*self.hub_stay)
        self.history = []
        self.seg = self.sel.pick(self.hub, clock=0.0)
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
                want = self.sel.target_energy(self.clock)
                lo, hi = self.hub_stay
                # high energy: change level sooner (a move between hubs is a big motion); calm: stay
                self.stay = lo if want is not None and want > 0.7 else self.rng.randint(lo, hi)
                return
        self.queue.append(self.sel.pick(self.hub, clock=self.clock))

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
            nxt = self.sel.pick(hub, mood, clock=self.clock)
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
        self.seg_t += dt * (self.scan_speed if self.seg.kind == "scan" else 1.0)
        while self.seg_t >= self.seg.duration:
            left = self.seg_t - self.seg.duration
            self._advance()
            if self.state == "PAUSED":
                self.seg_t = self.seg.duration
                break
            self.seg_t = left
        return self.seg.at(self.seg_t)

    def _label(self, item):
        """A queue item as shown: a segment's name, or a sequence clip still
        to be picked ("greet: a clip at greet")."""
        if isinstance(item, tuple):
            _, name, hub, mood = item
            return "%s: a %sclip at %s" % (name, (mood + " ") if mood else "", hub)
        return item.name

    def next_up(self):
        """What plays after the running clip, in words."""
        if self.state in ("FAULT", "HOLD"):
            return "nothing: held (reset to go on)"
        if self.state == "PAUSED":
            return "nothing: paused at %s (resume to go on)" % self.hub
        if self.queue:
            return self._label(self.queue[0])
        if self.pending:
            return "the route to %s, when this clip ends" % self.pending[0]
        if self.paused:
            return "pause at %s, when this clip ends" % self.seg.end
        return "an idle clip at %s, picked when this clip ends" % self.seg.end

    def scan_state(self, lag_s=0.0, rate=1.0):
        """(u, led, m/s) of the strip across the scan's opening (its labels'
        u, scan_positions), where the arm is lag_s of wall time behind the
        commands; rate: segment seconds a wall second (the stream's speed).
        (-1, 0, 0) outside the scan."""
        us = (self.seg.labels or {}).get("u") if self.seg.kind == "scan" else None
        if not us:
            return -1.0, 0, 0.0
        seg_rate = rate * self.scan_speed
        t = max(0.0, self.seg_t - lag_s * seg_rate)
        ts = self.seg.t

        def u_at(x):
            x = min(max(x, ts[0]), ts[-1])
            i = min(max(bisect.bisect_right(ts, x) - 1, 0), len(ts) - 2)
            f = (x - ts[i]) / (ts[i + 1] - ts[i]) if ts[i + 1] > ts[i] else 0.0
            return us[i] + f * (us[i + 1] - us[i])
        u = u_at(t)
        h = 0.02
        dudt = (u_at(t + h) - u_at(t - h)) / (min(t + h, ts[-1]) - max(t - h, ts[0]) or h)
        mps = dudt * float((self.seg.labels or {}).get("exposed_m", 0.0)) * seg_rate
        return round(u, 4), 1 if 0.0 <= u <= 1.0 else 0, round(abs(mps), 4)

    def status(self, lag_s=0.0, rate=1.0):
        prog = self.seg_t / self.seg.duration if self.seg.duration > 0 else 1.0
        u, led, mps = self.scan_state(lag_s, rate)
        return {"state": self.state, "clip": self.seg.name, "hub": self.hub, "sequence": self.sequence,
                "progress": round(min(1.0, prog), 4),
                "scan": round(min(1.0, prog), 4) if self.seg.kind == "scan" else -1.0,
                "scan_u": u, "scan_led": led, "scan_mps": mps,
                "time_left": round(max(0.0, self.seg.duration - self.seg_t), 2),
                "next": self.next_up(), "queue": [self._label(x) for x in self.queue[:6]],
                "pending": list(self.pending), "fault": self.fault_reason,
                "energy": round(self.sel.target_energy(self.clock) or 0.0, 3),
                "clip_energy": round(float((self.seg.labels or {}).get("energy") or 0.0), 3)}


def runner_for(graph, seed=None, log=None, scan_speed=1.0):
    sel = graph.info.get("select", {})
    return Runner(graph, Selector(graph.idle(), sel.get("no_repeat", 6), seed=seed, arc=sel.get("arc")),
                  hub_stay=tuple(sel.get("hub_stay", (2, 4))), seed=seed, log=log, scan_speed=scan_speed)


# --------------------------------------------------------------------------
# OSC (TouchDesigner)
# --------------------------------------------------------------------------

class OscBridge:
    """/robot/* in and out; python-osc (the usual TouchDesigner link)."""

    def __init__(self, runner, listen_port, send_host, send_port, also=(), lag_s=0.0):
        """Status goes to send_host:send_port and to each (host, port) in
        `also` -- a control window and TouchDesigner can both listen. lag_s:
        how far the arm is behind the commands (the scan's position is sent
        where the arm is, for the LEDs)."""
        self.lag_s = lag_s
        from pythonosc import dispatcher, osc_server, udp_client
        import threading
        self.runner = runner
        d = dispatcher.Dispatcher()
        d.map("/robot/trigger", lambda a, *v: runner.trigger(str(v[0]) if v else "scan"))
        d.map("/robot/pause", lambda a, *v: runner.pause())
        d.map("/robot/resume", lambda a, *v: runner.resume())
        d.map("/robot/reset", lambda a, *v: runner.reset())
        d.map("/robot/mood", lambda a, *v: setattr(runner.sel, "mood", str(v[0]) if v and v[0] else None))
        d.map("/robot/energy", lambda a, *v: setattr(runner.sel, "energy",
                                                     float(v[0]) if v and float(v[0]) >= 0.0 else None))
        if hasattr(runner, "stop"):                    # the streaming backend: a software stop
            d.map("/robot/stop", lambda a, *v: runner.stop())
        self.server = osc_server.ThreadingOSCUDPServer(("0.0.0.0", listen_port), d)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.clients = [udp_client.SimpleUDPClient(h, p) for h, p in [(send_host, send_port)] + list(also)]

    def send(self, q):
        s = self.runner.status(lag_s=self.lag_s)
        msgs = [("/robot/state", s["state"]), ("/robot/clip", s["clip"]), ("/robot/hub", s["hub"]),
                ("/robot/progress", float(s["progress"])), ("/robot/scan", float(s["scan"])),
                ("/robot/scan/u", float(s["scan_u"])), ("/robot/scan/led", int(s["scan_led"])),
                ("/robot/scan/speed", float(s["scan_mps"])),
                ("/robot/joints", [float(x) for x in q]),
                ("/robot/sequence", s.get("sequence") or ""), ("/robot/next", s.get("next", "")),
                ("/robot/queue", " | ".join(s.get("queue", []))), ("/robot/pending", ",".join(s.get("pending", []))),
                ("/robot/time_left", float(s.get("time_left", 0.0))), ("/robot/fault", s.get("fault") or ""),
                ("/robot/energy_now", float(s.get("energy", 0.0))), ("/robot/clip_energy", float(s.get("clip_energy", 0.0)))]
        if hasattr(self.runner, "speed_now"):                  # the streaming backend
            msgs.append(("/robot/speed_now", float(self.runner.speed_now)))
            msgs.append(("/robot/skipped", int(self.runner.skipped)))
        for c in self.clients:
            for addr, value in msgs:
                c.send_message(addr, value)

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
    by_name = {x.name: x for x in graph.idle()}
    fams = [family_of(by_name[n]) for n in idle]
    energies = [float(by_name[n].labels.get("energy") or 0.0) for n in idle]
    return {"minutes": minutes, "segments_played": len(played), "scans": played.count("scan"),
            "idle_clips_played": len(idle), "distinct_idle": len(set(idle)), "idle_available": len(graph.idle()),
            "hubs_visited": sorted(set(graph_start(graph, n) for n in idle)),
            "moves": sum(1 for n in played if graph_kind(graph, n) == "move"),
            "same_family_twice": sum(x == y for x, y in zip(fams, fams[1:])),
            "energy_played": {"min": round(min(energies), 2), "max": round(max(energies), 2)} if energies else None,
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
    # where the strip is over the paper, for the LEDs (TouchDesigner)
    us = scan_positions([0.0, 0.1, 0.5, 1.5, 1.6], {"exposed_m": 1.0, "ramp_m": 0.08}, 0.02, 0.0)
    check("the scan's position across the opening: 0 at its left edge, 1 at its right, beyond on the ramps",
          [round(u, 3) for u in us] == [-0.1, 0.0, 0.4, 1.4, 1.5], us)
    sseg = Segment("scan", "scan", [0.0, 2.0], [S0, S1], "scan_start", "scan_end",
                   {"u": [-0.5, 1.5], "exposed_m": 1.0})
    r = Runner(g, Selector(g.idle(), 1, seed=0), hub_stay=(99, 99), seed=0, scan_speed=0.5)
    r.seg, r.seg_t = sseg, 1.0
    now, late = r.status(), r.status(lag_s=1.0, rate=1.0)
    check("status gives the scan's position, LEDs on over the opening, its speed now (at the run's scan speed), "
          "and where the arm is a lag behind the commands",
          now["scan_u"] == 0.5 and now["scan_led"] == 1 and abs(now["scan_mps"] - 0.5) < 1e-6
          and late["scan_u"] == 0.0 and r.status(lag_s=5.0)["scan_led"] == 0,
          (now["scan_u"], now["scan_led"], now["scan_mps"], late["scan_u"]))
    # the scan's speed at run time (tuning an exposure): only the scan slows
    r = Runner(g, Selector(g.idle(), 1, seed=0), hub_stay=(99, 99), seed=0, scan_speed=0.5)
    r.step(0.5)
    r.trigger("scan")
    ticks = {}
    for _ in range(600):
        r.step(0.05)
        ticks[r.seg.name] = ticks.get(r.seg.name, 0) + 1
    check("scan_speed 0.5 plays the scan (2 s) in 4 s, the moves to and from it as built",
          abs(ticks.get("scan", 0) - 80) <= 1 and abs(ticks.get("to_scan", 0) - 40) <= 1, ticks)
    bad = []
    for f in (0.0, 1.5, -1.0):
        try:
            Runner(g, Selector(g.idle(), 1, seed=0), scan_speed=f)
            bad.append(f)
        except ValueError:
            pass
    check("a scan speed is slower than built, never faster (0 < f <= 1)", not bad, bad)
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
    r2 = Runner(g, Selector(g.idle(), 0, seed=4), hub_stay=(99, 99), seed=0)
    r2.step(0.5)
    before = r2.status()["next"]
    r2.trigger("greet")
    after = r2.status()
    check("status says what is next: a pick at the hub, then the route to a triggered sequence",
          "picked when this clip ends" in before and "route to greet" in after["next"]
          and after["pending"] == ["greet"] and after["time_left"] > 0, (before, after["next"]))
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
    sel = Selector(g.idle(), no_repeat=0, seed=3, arc=False)          # the mood alone, no arc
    base = [sel.pick("a").name for _ in range(400)].count("a1")
    sel.mood = "punch"
    picks = [sel.pick("a").name for _ in range(400)]
    check("a mood makes its clips likelier (than without it)", picks.count("a1") > base + 60, (base, picks.count("a1")))
    sel = Selector(g.idle(), no_repeat=0, seed=3)
    sel.energy = 1.0
    picks = [sel.pick("a").name for _ in range(400)]
    check("energy 1 prefers sudden clips", picks.count("a1") > 2 * picks.count("a2"), (picks.count("a1"), picks.count("a2")))
    sel = Selector(g.idle(), seed=3)
    wants = [sel.target_energy(t) for t in (0.0, 60.0, 120.0, 170.0, 185.0)]
    check("the arc builds from calm, bursts at its peak, and starts again",
          wants[0] < wants[1] < wants[2] < wants[3] and wants[3] == 1.0 and wants[4] < 0.3, wants)
    sel.energy = 0.4
    check("an energy from the operator overrides the arc ...", sel.target_energy(170.0) == 0.4)
    sel.energy = None
    check("... until it is cleared", sel.target_energy(170.0) == 1.0)
    moving = [Segment("e%d" % k, "idle", [0.0, 1.0, 2.0], [A, [A[0] + 5.0 * k] + list(A[1:]), A], "a", "a", {})
              for k in range(1, 5)]
    cv = {"center": [0.0, 1.0, 1.3], "normal": [0.0, 1.0, 0.0], "size": [1.5, 1.0], "thickness": 0.02,
          "frame": {"outer": [1.702, 1.168], "face_width": 0.1334, "depth": 0.02}}
    parts = {p["name"]: p for p in canvas_parts(cv)}
    rail = parts.get("frame_left", {})
    check("the frame is four wooden rails over the canvas's edges (opening 1.435 m < the 1.5 m canvas), in front "
          "of it; left as seen from the robot's side",
          sorted(parts) == ["canvas", "frame_bottom", "frame_left", "frame_right", "frame_top"]
          and abs(parts["frame_top"]["size"][1] - 1.702) < 1e-9 and abs(rail["size"][1] - 0.1334) < 1e-9
          and abs(rail["center"][0] + (0.851 - 0.0667)) < 1e-9 and abs(rail["center"][1] - 0.98) < 1e-9
          and all(p["kind"] == "wood" for n, p in parts.items() if n != "canvas") and parts["canvas"]["kind"] == "canvas",
          {n: (p["center"], p["size"]) for n, p in parts.items()})
    room = {"objects": [{"name": "ceiling", "type": "halfspace", "normal": [0.0, 0.0, -1.0], "offset": -2.155,
                         "role": "obstacle"},
                        {"name": "floor", "type": "halfspace", "normal": [0.0, 0.0, 1.0], "offset": 0.0, "role": "obstacle"}]}
    se = {o["name"]: o for o in show_env(room, {"margins": {"ceiling_m": 0.3}}, 0.15)["objects"]}
    check("the ceiling keeps margins.ceiling_m from every motion (the sprinkler), the floor its own",
          se["ceiling"].get("margin_m") == 0.3 and "margin_m" not in se["floor"], se)
    ts, ss, (on, off) = scan_profile(1.0, 0.3, 0.5, 0.01)
    vs = [(b - a) / (t1 - t0) for a, b, t0, t1 in zip(ss, ss[1:], ts, ts[1:])]
    cruise = [v for v, t in zip(vs, ts) if on <= t <= off - 0.01]
    check("the scan's time law: still at both ends, the cruise (1.0 m) at exactly its speed",
          ss[0] == 0.0 and vs[0] < 0.01 and vs[-1] < 0.01 and max(abs(v - 0.3) for v in cruise) < 1e-6
          and abs(off - on - 1.0 / 0.3) < 1e-9, (vs[0], vs[-1], min(cruise), max(cruise), on, off))
    sc = {"canvas": cv, "scan": {"direction": "left_to_right", "speed_mps": 0.3, "accel_mps2": 0.5, "led_gap_m": 0.06,
                                 "lead_m": 0.05}}
    a, b, info = scan_ends(sc, tool_z=0.07)
    check("left to right as seen from the robot's side: the scan starts at the left end (-x here), the LED face "
          "led_gap_m from the paper",
          a[0] < -0.8 and b[0] > 0.8 and abs(a[1] - (0.99 - 0.06 - 0.07)) < 1e-9
          and abs(info["exposed_m"] - 1.435) < 1e-3, (a, b, info))
    import shutil
    import tempfile
    root = tempfile.mkdtemp()
    for d in ("profiles", "envs", "shows", "assets"):
        os.makedirs(os.path.join(root, d))
    open(os.path.join(root, "profiles", "fr20.json"), "w").write(json.dumps({"robot": {"n": 6}, "tool": None}))
    open(os.path.join(root, "envs", "room.usda"), "w").write("#usda 1.0\n")
    cp = os.path.join(root, "shows", "s.json")
    json.dump({"env": "envs/room.usda", "hubs": {"a": {"q": [0] * 6}}, "_note": "x"}, open(cp, "w"))
    built = {"inputs": input_digests(cp, root)}
    fresh = stale_inputs(built, cp, root)
    json.dump({"env": "envs/room.usda", "hubs": {"a": {"q": [0] * 6}}, "_note": "a new note"}, open(cp, "w"), indent=4)
    noted = stale_inputs(built, cp, root)
    open(os.path.join(root, "envs", "room.usda"), "a").write("# moved a wall\n")
    moved = stale_inputs(built, cp, root)
    shutil.rmtree(root, ignore_errors=True)
    check("a compiled show knows its inputs: none changed, a note or the layout changed (not stale), the room "
          "changed (stale), built before they were recorded (stale)",
          fresh == [] and noted == [] and moved == ["env"] and stale_inputs({}, cp, root), (fresh, noted, moved))
    check("no frame: the canvas alone", [p["name"] for p in canvas_parts(dict(cv, frame=None))] == ["canvas"])
    spin = Segment("spin", "idle", [0.0, 1.0], [A, list(A[:5]) + [170.0]], "a", "a", {})
    lim6 = [(-175.0, 175.0)] * 5 + [(-150.0, 150.0)]
    check("a clip that turns J6 past the tool cable's range is named, with how far",
          limit_breaches([spin], lim6) == [("spin", 6, 170.0)] and not limit_breaches([spin], [(-175.0, 175.0)] * 6),
          limit_breaches([spin], lim6))
    turn = motion_stats([0.0, 1.0], [A, list(A[:5]) + [A[5] + 90.0]])
    bare = motion_stats([0.0, 1.0], [A, list(A[:5]) + [A[5] + 90.0]], watch=watched_points("fr20")[:1])
    check("a turn of J6 alone moves the tool's tips (a guest sees the strip spin), not the TCP",
          turn["v_peak"] > 0.5 and bare["v_peak"] < 0.01, (turn["v_peak"], bare["v_peak"]))
    add_energy(moving)
    e = {x.name: x.labels["energy"] for x in moving}
    check("every idle clip gets a measured energy, ranked 0..1", min(e.values()) == 0.0 and max(e.values()) == 1.0, e)
    fam = Graph(g.hubs, [seg("f%d" % i, "idle", A, A, "a", "a", action=("punch" if i < 3 else "float"))
                         for i in range(6)])
    sel = Selector(fam.idle(), no_repeat=0, seed=5, arc=False)
    picks = [family_of(sel.pick("a")) for _ in range(300)]
    same = sum(x == y for x, y in zip(picks, picks[1:])) / float(len(picks) - 1)
    check("the family just played is rarely played again at once", same < 0.3, round(same, 2))
    rows, summ, missed = report(Graph(g.hubs, moving))
    check("the report measures every idle clip and names the targets it misses",
          len(rows) == 4 and summ["extent_median"] > 0 and any("median height" in m for m in missed), (summ, missed))
    rng_cfg = {"range": {"j1_deg": [-30, 30], "tcp_z": [0.5, 1.5]}}
    check("the operating range refuses J1 past its sector",
          out_of_range(rng_cfg, [[0, 0, 0, 0, 0, 0], [40, 0, 0, 0, 0, 0]], None) is not None)
    check("... and a TCP too high", out_of_range(rng_cfg, [[0] * 6], [(0, 0, 1.7)]) is not None)
    check("... and lets a motion inside it through", out_of_range(rng_cfg, [[10] * 6], [(0, 0, 1.0)]) is None)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=("build", "dry-run", "report", "osc"))
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
    if a.command == "report":
        rows, summ, missed = report(g, json.load(open(cfg_path)))
        print("%-26s %-6s %-10s %5s %11s %6s %6s %6s %6s %5s" % ("clip", "hub", "family", "energy", "height m",
                                                               "span", "extent", "v mean", "v peak", "s"))
        for r in sorted(rows, key=lambda r: r["energy"] or 0.0):
            print("%-26s %-6s %-10s %5.2f %5.2f-%5.2f %6.2f %6.2f %6.2f %6.2f %5.1f" % (
                r["name"], r["hub"], r["family"][:10], r["energy"] or 0.0, r["z_min"], r["z_max"], r["z_span"],
                r["extent"], r["v_mean"], r["v_peak"], r["duration"]))
        print(json.dumps(summ))
        print("variety: " + ("meets the targets" if not missed else "MISSES -- " + "; ".join(missed)))
        return 0 if not missed else 1
    if a.command == "dry-run":
        require_fresh(g, cfg_path)
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
