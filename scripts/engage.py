"""The interactive mode (Stage 11, modes B / C): somebody steps onto a spot on
the floor straight in front of the greet hub and the arm stops its clip,
turns to them -- visibly: it perks up, then settles facing them -- and
follows them: left and right along the wall (B), a raised hand up and down
(C), in a small box around the greet hub's tool point; they step off, are
lost, or 30 s pass: a nod goodbye, back to the hub, the clips go on. Once
engaged they may move within FOLLOW_M of the spot (1.6 m along the wall,
1 m across); a hand from about the chest up is followed.

    spot         SPOT_R_M around the point of the audience zone's middle line
                 straight across from the greet hub's tool point (track_spot)
    the box      BOX_M (along the wall, up, towards the person) around the hub's
                 tool point, 25 x 15 x 6 cm: what the room leaves the upright
                 1 m strip there; the tool faces the head (B) or the raised
                 hand (C), the strip upright as at the hub (face_pose)

    uv run scripts/engage.py        self-test (the spot, the box: every pose reachable and clear)
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

SPOT_R_M = 0.3                     # the spot: 0.6 m across (the user, 2026-09-29)
BOX_M = ((-0.15, 0.10), (-0.12, 0.03), (0.0, 0.06))   # along the wall (left -), up, towards the person: what
                                   # the room leaves -- right of +0.1 the strip meets the wall, above +0.03 (the tool tilted down at a child) the
                                   # ceiling's margin (the 1 m strip stands upright), left of -0.15 the partition
AIM_YAW_DEG = 30.0                 # the tool turns at most this far left / right of straight at the spot ...
AIM_PITCH_DEG = (-20.0, 25.0)      # ... and tilts down / up at most this far: a child, a hand up at the side
BOX_ROOM_M = 0.02                  # every pose of the box keeps this much more than the room's margins


def _unit(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return tuple(x / n for x in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def track_spot(cfg, hub_q):
    """(centre (x, y), radius): the point of the audience zone's middle line
    straight across from the hub's tool point."""
    import show as S
    import track_sim as TS
    c, along, _ = TS.zone_frame(cfg)
    tcp = S.tool_pose(hub_q)[1]
    u = (tcp[0] - c[0]) * along[0] + (tcp[1] - c[1]) * along[1]
    return (c[0] + u * along[0], c[1] + u * along[1]), SPOT_R_M


def box_axes(cfg, hub_q, spot):
    """(along, up, towards) unit axes of the box: along the audience zone,
    world up, and level towards the spot from the hub's tool point."""
    import show as S
    import track_sim as TS
    _, along, _ = TS.zone_frame(cfg)
    tcp = S.tool_pose(hub_q)[1]
    towards = _unit((spot[0] - tcp[0], spot[1] - tcp[1], 0.0))
    return tuple(along), (0.0, 0.0, 1.0), towards


def _clamp_aim(d, towards):
    """d within AIM_YAW_DEG of `towards` (level) and AIM_PITCH_DEG of level."""
    yaw0 = math.atan2(towards[1], towards[0])
    yaw = math.atan2(d[1], d[0])
    dy = (yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi
    dy = max(-math.radians(AIM_YAW_DEG), min(math.radians(AIM_YAW_DEG), dy))
    pitch = math.asin(max(-1.0, min(1.0, d[2])))
    pitch = max(math.radians(AIM_PITCH_DEG[0]), min(math.radians(AIM_PITCH_DEG[1]), pitch))
    return (math.cos(pitch) * math.cos(yaw0 + dy), math.cos(pitch) * math.sin(yaw0 + dy), math.sin(pitch))


def face_pose(rig, hub_q, axes, offset, aim, near=None):
    """The joints with the tool point at the hub's plus `offset` (along, up,
    towards; clamped to the box) and the tool facing `aim` (turned and tilted
    no further than AIM_YAW_DEG / AIM_PITCH_DEG), the strip kept upright as
    at the hub; None when out of reach."""
    import show as S
    R0, tcp0, _ = S.tool_pose(hub_q)
    o = [max(lo, min(hi, x)) for x, (lo, hi) in zip(offset, BOX_M)]
    p = tuple(tcp0[i] + sum(o[k] * axes[k][i] for k in range(3)) for i in range(3))
    z = _clamp_aim(_unit(tuple(a - b for a, b in zip(aim, p))), axes[2])
    strip0 = (R0[0][1], R0[1][1], R0[2][1])
    y = _unit(tuple(s - _dot(strip0, z) * zz for s, zz in zip(strip0, z)))
    x = _cross(y, z)
    R = tuple((x[i], y[i], z[i]) for i in range(3))
    return rig.solve(p, z, 0.0, near or hub_q, R=R)


GLASS = "partition_left"           # the room's halfspace between the guests and the arm
WAVE_S = 1.0                       # waving this long (tracking.is_waving), within the follow zone and BAND_M of
                                   # the glass: as good as stepping onto the spot (the user)
DWELL_S = 1.0                      # on the spot this long (not walking) before the arm turns to them
FOLLOW_M = (0.8, 0.5)              # once engaged, followed while within this of the spot (along, across): the
                                   # spot says "me"; then they may move -- and the edge does not flap
LEAVE_S = 0.7                      # off the spot or unseen this long: goodbye
LOST_CROWD_S = 1.5                 # ... unseen this long when others are near them (hidden behind someone)
CROWD_NEAR_M = 1.0
REID_M, REID_S = 0.3, 0.5          # the tracker gives the one followed a new id: a new id this near where they were,
                                   # this soon after, is them
JUMP_MPS = 3.0                     # the one followed "moving" faster than this: the tracker swapped two ids
MAX_S = 30.0                       # at most this long for one person (the user)
REST_S = 3.0                       # after a goodbye, nobody new for this long
PERK_S, BYE_S = 0.6, 0.5           # the perk-up and the nod, held
PERK_UP_M = 0.35                   # perking up: looks this much above the head, the tool at the box's top, leaning in
NOD_DOWN_M = 0.6                   # the nod: looks this much below the head
HAND_UP_M = 0.45                   # a hand above (head - this, about the chest) is up: mode C follows it
C_RANGE_M = (1.0, 2.0)             # a hand's height mapped onto the box's height
B_GAIN = 0.18                      # the tool moves along the wall this much of the person's own move (the box is small;
                                   # the aim turns with them the whole way)
SHARE = 0.35                       # of the joints' velocity, acceleration and jerk limits
SLOW_SHARE = 0.08                  # ... near a slow zone
SLOW_NEAR_M = 0.15
BRANCH_DEG = 90.0                  # a target a joint this far from the pose sent is refused (another IK branch)
PLAN_STEP_S = 0.05                 # a planned move is checked this often along it
STATES = ("OFF", "ENTER", "PERK", "TRACK", "BYE", "RETURN")


class Engage:
    """The interactive mode, tick by tick: step(q_clip, now) -> the pose to
    send. OFF: the clip's pose as it is (the caller's, gaze included). Someone
    on the spot DWELL_S, not walking: ENTER -- a planned move from the clip's
    pose and speed (every PLAN_STEP_S of it checked first; not clear: no
    engagement) to the perk-up pose -- PERK (held) -- TRACK (B: the tool
    along the wall with them; C: up with a raised hand, facing it) -- off the
    spot or unseen LEAVE_S, or MAX_S: BYE (a nod) -- RETURN (planned, checked)
    to the hub -- OFF with `resume` set once: the caller starts the clips
    again from the hub. Every pose sent is checked; not clear: the arm stops
    where it is. Near a slow zone the speed limit is SLOW_SHARE."""

    def __init__(self, cfg, hub_q, env, model, dt=0.008, rig=None):
        import collision as C
        import gestures as G
        import robot_profile as RP
        import transitions as T
        from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory
        self.C, self.Result, self.Trajectory, self.RuckigCls = C, Result, Trajectory, Ruckig
        self.rig = rig or G.Rig()
        self.hub, self.env, self.model, self.dt = list(hub_q), env, model, dt
        self.spot, self.r = track_spot(cfg, hub_q)
        self.axes = box_axes(cfg, hub_q, self.spot)
        self.slow = [o for o in (env or {}).get("objects", []) if o["role"] == "slow"]
        g = next((o for o in (env or {}).get("objects", []) if o["name"] == GLASS), None)
        self.glass = (g["normal"], g["offset"]) if g else None
        prof = RP.load("fr20")
        vel, acc = RP.velocity_limits(prof), RP.acceleration_limits(prof)
        self.vmax = [v * SHARE for v in vel]
        self.vslow = [v * SLOW_SHARE for v in vel]
        self.otg, self.inp, self.out = Ruckig(6, dt), InputParameter(6), OutputParameter(6)
        self.inp.max_velocity = list(self.vmax)
        self.inp.max_acceleration = [a * SHARE for a in acc]
        self.inp.max_jerk = [j * SHARE for j in T.default_jerk(acc)]
        self.state, self.t_state, self.who, self.t_engaged = "OFF", 0.0, None, 0.0
        self.people = {}                 # pid -> {"pos", "hist", "in_since", "last", "speed", "done"}
        self.hands = {}                  # pid -> (x, y, z, t)
        self.q_prev = []                 # the last poses sent (for the speed at ENTER)
        self.resume = False
        self.rest_until = -1.0
        self.unsafe = self.refused = self.engagements = 0
        self.q = list(hub_q)
        self.fresh = False

    # --- the people -------------------------------------------------------------
    def _on_spot(self, p, r):
        return math.dist(p[:2], self.spot) <= r

    def _in_follow(self, p):
        """Within FOLLOW_M of the spot, along the wall and across it."""
        d = (p[0] - self.spot[0], p[1] - self.spot[1])
        along = d[0] * self.axes[0][0] + d[1] * self.axes[0][1]
        across = d[0] * self.axes[2][0] + d[1] * self.axes[2][1]
        return abs(along) <= FOLLOW_M[0] and abs(across) <= FOLLOW_M[1]

    def _engaged(self):
        return self.who if self.state in ("ENTER", "PERK", "TRACK") else None

    def _hand_over(self, old, new):
        """The one followed is `new` now (the tracker's ids changed): their state goes with them."""
        a, b = self.people[old], self.people[new]
        for k in ("in_since", "done"):
            a[k], b[k] = b[k], a[k]
        self.who = new
        self.reids = getattr(self, "reids", 0) + 1

    def update(self, people, now, hands=()):
        """people: [(pid, x, y, z, conf, t)]; hands: [(pid, x, y, z, conf, t)] (a hand each, raised or not).
        The tracker's ids are not trusted for the one followed: a jump faster than JUMP_MPS to where
        another id is means the ids swapped; a new id near where they vanished is them."""
        import tracking as TR
        frame = {pid: (x, y, z) for pid, x, y, z, conf, t in people if conf >= TR.MIN_CONF}
        w = self._engaged()
        was = self.people.get(w) if w is not None else None
        if was is not None and w in frame:
            dt = max(now - was["last"], 1.0 / 30.0)
            if math.dist(frame[w][:2], was["pos"][:2]) / dt > JUMP_MPS:
                other = [pid for pid, p in frame.items() if pid != w and math.dist(p[:2], was["pos"][:2]) <= REID_M]
                if other and other[0] in self.people:
                    self._hand_over(w, other[0])               # the ids swapped: follow the person, not the number
                else:
                    was["jumps"] = was.get("jumps", 0) + 1
                    if was["jumps"] < TR.SWITCH_FRAMES:
                        del frame[w]                           # a glitch: not taken, unless the next frames agree
                    else:
                        was["jumps"] = 0
            else:
                was["jumps"] = 0
        for pid, x, y, z, conf, t in people:
            if pid not in frame:
                continue
            pr = self.people.get(pid)
            if pr is None or (now - pr["last"] > LEAVE_S and pid != self._engaged()):
                pr = self.people[pid] = {"hist": [], "in_since": None, "done": False, "first": now}
            pr.update(pos=(x, y, z), last=now)
            pr["hist"].append((t, (x, y, z)))
            while pr["hist"] and pr["hist"][0][0] < t - 2.0 * TR.SPEED_WINDOW_S:
                del pr["hist"][0]
            pr["speed"] = TR._walking_speed(pr["hist"], t)
            inside = self._on_spot((x, y), self.r) if pr["in_since"] is None else self._in_follow((x, y))
            if inside and pr["in_since"] is None:
                pr["in_since"] = now
            elif not inside:
                pr["in_since"], pr["done"] = None, False         # stepped off: may come again
        w = self._engaged()
        was = self.people.get(w) if w is not None else None
        if was is not None and w not in frame and now - was["last"] <= REID_S:
            new = [pid for pid, p in frame.items() if pid != w and now - self.people[pid].get("first", -1e9) <= REID_S
                   and math.dist(p[:2], was["pos"][:2]) <= REID_M]
            if new:
                self._hand_over(w, new[0])                     # vanished, a new id where they were: them
        for pid, x, y, z, conf, t in hands:
            if conf >= TR.MIN_CONF:
                self.hands[pid] = (x, y, z, now)
                pr = self.people.get(pid)
                if pr is not None:                             # the hand about the head: waving?
                    hs = pr.setdefault("hands", [])
                    hs.append((t, (x - pr["pos"][0], y - pr["pos"][1], z - pr["pos"][2])))
                    while hs and hs[0][0] < t - TR.ACT_WINDOW_S:
                        del hs[0]
        keep = self._engaged()
        for pid in [k for k, pr in self.people.items()
                    if now - pr["last"] > (LOST_CROWD_S if k == keep else LEAVE_S)]:
            del self.people[pid]
        self.fresh = True                                  # TRACK aims again (not every tick: the data's rate)

    def _waving(self, pr, now):
        """Waving at the arm: the hand swinging above the head (tracking.is_waving: over WAVE_S), not
        walking, in the follow zone (the arm can face them) and within BAND_M of the glass."""
        import tracking as TR
        return (pr.get("speed") is not None and pr["speed"] <= TR.PASSER_MPS and self._in_follow(pr["pos"])
                and (self.glass is None or TR.glass_distance(self.glass, pr["pos"]) <= TR.BAND_M)
                and TR.is_waving(pr.get("hands", []), now))

    def candidate(self, now):
        """Whom the mode would engage now (on the spot DWELL_S, or waving), without starting: None when
        nobody, or while resting after a goodbye. For a player whose arm is elsewhere (track_mode: it
        comes to the hub first)."""
        return self._candidate(now) if now >= self.rest_until else None

    def _candidate(self, now):
        import tracking as TR
        ok = [(pr["in_since"] if pr["in_since"] is not None else now, pid) for pid, pr in self.people.items()
              if not pr["done"] and pr.get("speed") is not None and pr["speed"] <= TR.PASSER_MPS
              and ((pr["in_since"] is not None and now - pr["in_since"] >= DWELL_S) or self._waving(pr, now))]
        near = lambda pid: math.dist(self.people[pid]["pos"][:2], (0.0, 0.0))                  # noqa: E731
        return min(ok, key=lambda x: (near(x[1]), x[0]))[1] if ok else None             # the nearest the arm

    # --- the motion -------------------------------------------------------------
    def _clear(self, q):
        rep = self.C.check(self.model, self.env, [0.0], [q])
        return rep["ok"]

    def _plan_ok(self, target):
        """The move from where the arm is now to target, checked every PLAN_STEP_S."""
        from ruckig import InputParameter
        inp = InputParameter(6)
        for k in ("current_position", "current_velocity", "current_acceleration", "max_velocity",
                  "max_acceleration", "max_jerk"):
            setattr(inp, k, list(getattr(self.inp, k)))
        inp.target_position, inp.target_velocity = list(target), [0.0] * 6
        traj = self.Trajectory(6)
        if self.RuckigCls(6).calculate(inp, traj) not in (self.Result.Working, self.Result.Finished):
            return False
        n = max(1, int(traj.duration / PLAN_STEP_S))
        return all(self._clear(list(traj.at_time(traj.duration * i / n)[0])) for i in range(n + 1))

    def _pose(self, offset, aim):
        """face_pose on the branch of the pose last sent (self.q); None when out of reach or on
        another branch (a joint BRANCH_DEG or more away: that way lies a flip, not a turn)."""
        q = face_pose(self.rig, self.hub, self.axes, offset, aim, near=self.q)
        if q is None or max(abs(a - b) for a, b in zip(q, self.q)) >= BRANCH_DEG:
            return None
        return q

    def _head(self):
        pr = self.people.get(self.who)
        return pr["pos"] if pr else None

    def _set(self, state, now):
        self.state, self.t_state = state, now

    def _track_target(self, now):
        """TRACK's pose: B along with the person, C up with a raised hand."""
        head = self._head()
        if head is None:
            return None, "B"
        du = (head[0] - self.spot[0]) * self.axes[0][0] + (head[1] - self.spot[1]) * self.axes[0][1]
        hand = self.hands.get(self.who)
        if hand is not None and now - hand[3] <= LEAVE_S and hand[2] > head[2] - HAND_UP_M:
            f = (hand[2] - C_RANGE_M[0]) / (C_RANGE_M[1] - C_RANGE_M[0])
            lo, hi = BOX_M[1]
            up = lo + (hi - lo) * max(0.0, min(1.0, f))
            return self._pose((du * B_GAIN, up, 0.03), hand[:3]), "C"
        return self._pose((du * B_GAIN, 0.0, 0.03), head), "B"

    def step(self, q_clip, now):
        """The pose to send now; self.state says which mode."""
        self.resume = False
        if self.state == "OFF":
            self.q_prev = (self.q_prev + [list(q_clip)])[-3:]
            who = self._candidate(now) if now >= self.rest_until else None
            if who is not None:
                self._start(who, q_clip, now)
            if self.state == "OFF":
                self.q = list(q_clip)
                return self.q
        pr = self.people.get(self.who)
        off = pr is not None and pr["in_since"] is None          # out of the follow zone
        unseen = now - pr["last"] if pr is not None else float("inf")
        crowd = pr is not None and any(o is not pr and now - o["last"] <= 0.3
                                       and math.dist(o["pos"][:2], pr["pos"][:2]) <= CROWD_NEAR_M
                                       for o in self.people.values())
        if self.state in ("PERK", "TRACK"):
            if not off:
                self.t_gone = now
            if (off and now - self.t_gone >= LEAVE_S) or unseen >= (LOST_CROWD_S if crowd else LEAVE_S):
                self._goodbye(now)
        if self.state == "TRACK" and now - self.t_engaged >= MAX_S:
            self._goodbye(now)
        target = None
        if self.state == "ENTER":
            if self.reached:
                self._set("PERK", now)
        elif self.state == "PERK":
            if now - self.t_state >= PERK_S:
                self._set("TRACK", now)
        if self.state == "TRACK" and (self.fresh or self.inp.target_position is None):
            target, self.mode = self._track_target(now)
            self.fresh = False
        elif self.state == "BYE":
            if self.reached and now - self.t_state >= BYE_S:
                if self._plan_ok(self.hub):
                    self._set("RETURN", now)
                    target = self.hub
                else:
                    self.refused += 1                      # wait where it is, try again
        elif self.state == "RETURN" and self.reached:
            self._set("OFF", now)
            self.resume, self.q_prev = True, []
            self.rest_until = now + REST_S
            self.who = None
            self.q = list(self.hub)
            return self.q
        if target is not None:
            self.inp.target_position, self.inp.target_velocity = list(target), [0.0] * 6
        near = self.slow and any(self.C.sdf(o, self.rig.tool(self.q)[1]) < SLOW_NEAR_M for o in self.slow)
        self.inp.max_velocity = list(self.vslow if near else self.vmax)
        res = self.otg.update(self.inp, self.out)
        q = list(self.out.new_position)
        if not self._clear(q):
            self.unsafe += 1                               # stop where it is (the last pose sent was clear)
            self.inp.target_position = list(self.inp.current_position)
            self.inp.current_velocity = [0.0] * 6
            self.inp.current_acceleration = [0.0] * 6
            return self.q
        self.out.pass_to_input(self.inp)
        self.reached = res == self.Result.Finished
        self.q = q
        return q

    def _start(self, who, q_clip, now):
        self.q = list(q_clip)
        head = self.people[who]["pos"]
        perk = self._pose((0.0, BOX_M[1][1], BOX_M[2][1]), (head[0], head[1], head[2] + PERK_UP_M))
        dt = self.dt
        qs = self.q_prev
        v = [(b - a) / dt for a, b in zip(qs[-2], qs[-1])] if len(qs) >= 2 else [0.0] * 6
        a = ([(c - 2 * b + x) / dt ** 2 for x, b, c in zip(qs[-3], qs[-2], qs[-1])] if len(qs) >= 3 else [0.0] * 6)
        self.inp.current_position = list(q_clip)
        self.inp.current_velocity = [max(-m, min(m, x)) for x, m in zip(v, self.vmax)]
        self.inp.current_acceleration = [max(-m, min(m, x)) for x, m in zip(a, self.inp.max_acceleration)]
        if perk is None or not self._plan_ok(perk):
            self.refused += 1
            self.people[who]["done"] = True                # not now: they may step off and on again
            return
        self.who, self.t_engaged, self.t_gone, self.reached, self.mode = who, now, now, False, "B"
        self.people[who]["done"] = True                    # once per stay on the spot
        if self.people[who]["in_since"] is None:           # called by waving: followed in the zone from now
            self.people[who]["in_since"] = now
        self.inp.target_position, self.inp.target_velocity = list(perk), [0.0] * 6
        self.engagements += 1
        self._set("ENTER", now)

    def _goodbye(self, now):
        head = self._head() or (self.spot[0], self.spot[1], 1.6)
        cur = self.inp.current_position
        nod = self._pose((0.0, 0.0, 0.0), (head[0], head[1], head[2] - NOD_DOWN_M))
        self.inp.target_position = list(nod if nod is not None else cur)
        self.inp.target_velocity = [0.0] * 6
        self.reached = False
        self._set("BYE", now)


class ClipPlayer:
    """A hub's idle clips back to back in a seeded order, as the show plays
    them, that the interactive mode can cut short: restart(now) begins a clip
    from the hub at now (where RETURN leaves the arm). advance(now) once a
    tick; at(t) reads without moving on (the look-ahead reaches the next clip)."""

    def __init__(self, graph, hub="greet", seed=1):
        import random
        self.clips = sorted(graph.idle(hub), key=lambda s: s.name)
        self.rng = random.Random(seed)
        self.restart(0.0)

    def restart(self, now):
        self.t0, self.clip, self.next = now, self.rng.choice(self.clips), self.rng.choice(self.clips)

    def advance(self, now):
        while now - self.t0 >= self.clip.duration:
            self.t0 += self.clip.duration
            self.clip, self.next = self.next, self.rng.choice(self.clips)

    def at(self, t):
        s = t - self.t0
        if s < self.clip.duration:
            return self.clip.at(max(0.0, s))
        return self.next.at(min(s - self.clip.duration, self.next.duration))


def self_test():
    import json
    from tracking import BAND_M, glass_distance
    import collision as C
    import gestures as G
    import show as S
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    cfg = json.load(open(os.path.join(ROOT, "shows", "party.json")))
    g = S.Graph.load(S.compiled_path(os.path.join(ROOT, "shows", "party.json")))
    hub = g.hubs["greet"]
    spot, r = track_spot(cfg, hub)
    tcp = S.tool_pose(hub)[1]
    a = cfg["zones"]["audience"]
    check("the spot: 0.6 m across, straight in front of the greet hub, on the audience zone's middle line",
          r == 0.3 and math.dist(spot, a["center"][:2]) < a["size"][0] / 2 and 1.0 < math.dist(spot, tcp[:2]) < 1.6,
          ([round(x, 3) for x in spot], round(math.dist(spot, tcp[:2]), 2)))
    rig = G.Rig()
    axes = box_axes(cfg, hub, spot)
    head = (spot[0], spot[1], 1.6)
    q = face_pose(rig, hub, axes, (0.0, 0.0, 0.0), head)
    R, t, d = S.tool_pose(q)
    to = _unit(tuple(h - x for h, x in zip(head, t)))
    check("the face pose: the tool at the hub's point, facing the head on the spot, the strip upright",
          q is not None and math.dist(t, tcp) < 1e-3 and _dot(d, to) > 0.999 and abs(R[2][1]) > 0.9,
          (round(_dot(d, to), 4), round(R[2][1], 3)))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    env = dict(env, objects=[dict(o, margin_m=o.get("margin_m", env.get("margin_m", 0.05)) + BOX_ROOM_M)
                             if o["role"] == "obstacle" else o for o in env["objects"]])
    model = C.load_model("fr20")
    grid = lambda lo, hi, k: [lo + (hi - lo) * i / (k - 1) for i in range(k)]      # noqa: E731
    bad, jump, n = [], 0.0, 0
    for du in (-SPOT_R_M - 0.2, 0.0, SPOT_R_M + 0.2):                  # the person anywhere they are followed
        for hz in (1.2, 1.6, 2.1):                                     # a head, or a hand up
            aim = (spot[0] + du * axes[0][0], spot[1] + du * axes[0][1], hz)
            prev_row = None
            for oa in grid(*BOX_M[0], 6):
                row = []
                for ou in grid(*BOX_M[1], 4):
                    for od in grid(*BOX_M[2], 2):
                        qq = face_pose(rig, hub, axes, (oa, ou, od), aim)
                        n += 1
                        row.append(qq)
                        if qq is None:
                            bad.append(("no IK", oa, ou, od, du, hz))
                            continue
                        rep = C.check(model, env, [0.0], [qq])
                        if not rep["ok"]:
                            bad.append((C.describe(rep)[:60], round(oa, 2), round(ou, 2), od, du, hz))
                if prev_row:
                    jump = max([jump] + [max(abs(x - y) for x, y in zip(a_, b_))
                                         for a_, b_ in zip(prev_row, row) if a_ and b_])
                prev_row = row
    check("the box: %d poses over it, facing anyone they follow, reachable and clear by %.0f mm more than the "
          "room's margins" % (n, 1000 * BOX_ROOM_M), not bad, bad[:3])
    check("... one branch: no joint turns more than 25 deg between neighbouring poses (5 cm apart)", jump < 25.0,
          round(jump, 1))

    # --- the state machine, the clip at rest at the hub, people at 30 Hz -------------------
    env0 = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    import robot_profile as RP
    vel = RP.velocity_limits(RP.load("fr20"))

    def run(person, dur, hand=None):
        """person(t) -> (du along the spot, dv across, z) or None; hand(t) -> z or None. The run's log."""
        en = Engage(cfg, hub, env0, model)
        log, dt = [], en.dt
        for i in range(int(dur / dt)):
            now = i * dt
            if i % 4 == 0:                                   # ~31 Hz, as the data
                p = person(now)
                ppl, hands = [], []
                if p is not None:
                    du, dv, z = p
                    x = spot[0] + du * axes[0][0] + dv * axes[2][0]
                    y = spot[1] + du * axes[0][1] + dv * axes[2][1]
                    ppl.append((1, x, y, z, 0.9, now))
                    hz = hand(now) if hand else None
                    if hz is not None:
                        hands.append((1, x + 0.2 * axes[0][0], y + 0.2 * axes[0][1], hz, 0.9, now))
                en.update(ppl, now, hands)
            q = en.step(hub, now)
            log.append((now, en.state, list(q), en.resume, getattr(en, "mode", None)))
        return en, log

    def first(log, st):
        return next((t for t, s, *_ in log if s == st), None)

    def run_many(people, dur):
        """people(t) -> [(pid, du, dv, z)] (the tracker's ids). The run's log: (t, state, q, who)."""
        en = Engage(cfg, hub, env0, model)
        log = []
        for i in range(int(dur / en.dt)):
            now = i * en.dt
            if i % 4 == 0:
                en.update([(pid, spot[0] + du * axes[0][0] + dv * axes[2][0],
                            spot[1] + du * axes[0][1] + dv * axes[2][1], z, 0.9, now)
                           for pid, du, dv, z in people(now)], now)
            q = en.step(hub, now)
            log.append((now, en.state, list(q), en.who))
        return en, log

    def aim_along(q):
        """Where along the spot's line the tool looks (m, at the spot's distance)."""
        _, tp, d = S.tool_pose(q)
        k = _dot((spot[0] - tp[0], spot[1] - tp[1], 0.0), axes[2]) / max(1e-6, _dot(d, axes[2]))
        return _dot(tuple(tp[i] + k * d[i] - (spot[0], spot[1], 0.0)[i] for i in range(3)), axes[0])

    stay = lambda t: (0.0, 0.0, 1.62) if t >= 1.0 else (0.0, 2.0, 1.62)     # steps onto the spot at 1 s
    en, log = run(lambda t: stay(t) if t < 12.0 else (0.0, 2.0, 1.62), 20.0)
    t_enter, t_track = first(log, "ENTER"), first(log, "TRACK")
    check("steps onto the spot: the arm turns to them after %.0f s on it (ENTER), perks up, then follows (TRACK)"
          % DWELL_S, t_enter is not None and DWELL_S + 1.0 <= t_enter < DWELL_S + 1.5 and t_track is not None
          and t_track > t_enter, (t_enter, t_track))
    q_tr = [q for t, s, q, *_ in log if s == "TRACK" and t <= 10.0][-1]          # still on the spot
    R_, tp, d_ = S.tool_pose(q_tr)
    to = _unit((spot[0] - tp[0], spot[1] - tp[1], 1.62 - tp[2]))
    check("... the tool faces them", math.degrees(math.acos(min(1.0, _dot(d_, to)))) < 5.0,
          round(math.degrees(math.acos(min(1.0, _dot(d_, to)))), 1))
    t_bye, t_off = first(log, "BYE"), next((t for t, s, q, r, m in log if r), None)
    q_end = log[-1][2]
    check("steps off at 12 s: a nod %.1f s later, back to the hub, the clips told to go on (once)" % LEAVE_S,
          t_bye is not None and 12.0 + LEAVE_S - 0.1 <= t_bye < 12.0 + LEAVE_S + 0.3 and t_off is not None
          and sum(r for *_, r, m in log) == 1 and max(abs(a - b) for a, b in zip(q_end, hub)) < 1e-6
          and log[-1][1] == "OFF", (t_bye, t_off))
    peak = max(max(abs(b - a) / en.dt / v for a, b, v in zip(x[2], y[2], vel)) for x, y in zip(log, log[1:]))
    check("... every joint within %.0f %% of its speed limit, every pose clear" % (100 * SHARE),
          peak <= SHARE * 1.01 and en.unsafe == 0, (round(peak, 3), en.unsafe))

    sway = lambda t: (0.0 if t < 5.0 else 0.7 * math.sin(2 * math.pi * (t - 5.0) / 8.0), 0.0, 1.62)
    en, log = run(sway, 14.0)
    tr = [(t, q) for t, s, q, *_ in log if s == "TRACK" and t > 6.0]
    along = [_dot(tuple(a - b for a, b in zip(S.tool_pose(q)[1], tcp)), axes[0]) for t, q in tr]
    yaw = [math.degrees(math.atan2(_dot(S.tool_pose(q)[2], axes[0]), _dot(S.tool_pose(q)[2], axes[2]))) for t, q in tr]
    check("B: they walk 0.7 m left and right of the spot (inside the follow zone, %.1f x %.1f m): still engaged; "
          "the tool moves along the wall and turns with them" % (2 * FOLLOW_M[0], 2 * FOLLOW_M[1]),
          en.engagements == 1 and first(log, "BYE") is None and max(along) > 0.08 and min(along) < -0.1
          and max(yaw) - min(yaw) > 30.0,
          (round(min(along), 3), round(max(along), 3), round(min(yaw), 1), round(max(yaw), 1)))
    far = lambda t: (0.0 if t < 5.0 else min(1.2, 0.3 * (t - 5.0)), 0.0, 1.62)
    en, log = run(far, 14.0)
    t_bye = first(log, "BYE")
    check("... walking on past %.1f m along: goodbye" % FOLLOW_M[0],
          t_bye is not None and 5.0 + FOLLOW_M[0] / 0.3 < t_bye < 5.0 + FOLLOW_M[0] / 0.3 + LEAVE_S + 0.5, t_bye)
    en, log = run(stay, 12.0, hand=lambda t: 1.3 if t >= 6.0 else 0.9)
    modes = {m for t, s, q, r, m in log if s == "TRACK" and t > 8.0}
    check("C: a hand at chest height (1.3 m) is up already", modes == {"C"}, modes)
    en, log = run(stay, 12.0, hand=lambda t: 2.0 if t >= 6.0 else 0.9)
    modes = {m for t, s, q, r, m in log if s == "TRACK" and t > 8.0}
    up_b = [S.tool_pose(q)[2][2] for t, s, q, *_ in log if s == "TRACK" and 5.0 < t < 6.0]     # B settled
    up_c = [S.tool_pose(q)[2][2] for t, s, q, *_ in log if s == "TRACK" and t > 9.0]
    check("C: a hand raised to 2 m: the tool rises and tilts up to it", modes == {"C"} and up_c and up_b
          and min(up_c) > max(up_b) + 0.1, (modes, round(max(up_b), 2) if up_b else None,
                                            round(min(up_c), 2) if up_c else None))
    en, log = run(lambda t: (0.0, 0.0, 1.62), 45.0)
    t_in, t_bye = first(log, "ENTER"), first(log, "BYE")
    check("stays on: goodbye %.0f s after the arm turned to them, and not again while they stay" % MAX_S,
          t_bye is not None and abs(t_bye - t_in - MAX_S) < 0.1 and en.engagements == 1, (t_in, t_bye, en.engagements))
    walk = lambda t: (-1.2 + 1.2 * t, 0.0, 1.65) if t < 2.0 else None
    en, log = run(walk, 6.0)
    check("someone walking through the spot at 1.2 m/s: not engaged", en.engagements == 0)
    pl = ClipPlayer(g)
    qa, qb_ = pl.at(0.0), pl.at(pl.clip.duration - 1e-6)
    ahead = pl.at(pl.clip.duration + 0.5)                   # the look-ahead reads the next clip, moves nothing
    first_clip = pl.clip
    pl.advance(pl.clip.duration + 0.5)
    moved = pl.clip is not first_clip and max(abs(a - b) for a, b in zip(pl.at(pl.t0 + 0.5), ahead)) < 1e-9
    pl.restart(7.3)
    check("the clip player: clips from the hub, back to back; cut short, the next starts at the hub",
          max(abs(a - b) for a, b in zip(qa, hub)) < 0.05 and max(abs(a - b) for a, b in zip(qb_, hub)) < 0.05
          and max(abs(a - b) for a, b in zip(pl.at(7.3), hub)) < 0.05 and moved)
    # several people: the tracker's ids are not the people
    renumber = lambda t: [(1 if t < 8.0 else 7, 0.0, 0.0, 1.62)]
    en, log = run_many(renumber, 14.0)
    check("the one followed gets a new id from the tracker (1 -> 7, same place): still followed, no goodbye",
          first(log, "BYE") is None and log[-1][3] == 7 and log[-1][1] == "TRACK", (first(log, "BYE"), log[-1][3]))
    swapped = lambda t: ([(1, 0.0, 0.0, 1.62), (2, 0.4, 0.1, 1.7)] if t < 8.0
                         else [(2, 0.0, 0.0, 1.62), (1, 0.4, 0.1, 1.7)])
    en, log = run_many(swapped, 14.0)
    looks = [aim_along(q) for t, st, q, w in log if st == "TRACK" and t > 10.0]
    check("two people's ids swapped at 8 s: the arm keeps looking at the same person (on the spot), not the other",
          looks and max(abs(x) for x in looks) < 0.12 and log[-1][3] == 2, (round(max(abs(x) for x in looks), 3)
                                                                           if looks else None, log[-1][3]))
    both = lambda t: [(1, 0.0, 0.2, 1.62), (2, 0.1, -0.2, 1.7)]
    en, log = run_many(both, 4.0)
    check("two step onto the spot together: the one nearer the arm is followed",
          first(log, "ENTER") is not None and log[-1][3] == 2, log[-1][3])
    def run_wave(person, waving_from, dur):
        """person(t) -> (du, dv, z); the hand swings above the head from waving_from, else hangs down."""
        en = Engage(cfg, hub, env0, model)
        log = []
        for i in range(int(dur / en.dt)):
            now = i * en.dt
            if i % 4 == 0:
                du, dv, z = person(now)
                x = spot[0] + du * axes[0][0] + dv * axes[2][0]
                y = spot[1] + du * axes[0][1] + dv * axes[2][1]
                if now >= waving_from:
                    sw = 0.15 * math.sin(2 * math.pi * 1.2 * now)
                    hand = (1, x + sw * axes[0][0], y + sw * axes[0][1], z + 0.15, 0.9, now)
                else:
                    hand = (1, x + 0.25 * axes[0][0], y + 0.25 * axes[0][1], z - 0.7, 0.9, now)
                en.update([(1, x, y, z, 0.9, now)], now, [hand])
            q = en.step(hub, now)
            log.append((now, en.state, list(q), en.who))
        return en, log
    en, log = run_wave(lambda t: (0.6, 0.0, 1.62), 2.0, 8.0)
    t_in = first(log, "ENTER")
    d_glass = glass_distance(en.glass, (spot[0] + 0.6 * axes[0][0], spot[1] + 0.6 * axes[0][1], 0.0))
    check("waving off the spot (0.6 m along it, %.2f m from the glass): the arm turns to them after %.0f s of it, "
          "and follows" % (d_glass, WAVE_S), t_in is not None and 2.0 + WAVE_S <= t_in < 2.0 + WAVE_S + 0.6
          and log[-1][1] == "TRACK" and d_glass <= BAND_M, (t_in, log[-1][1]))
    en, log = run_wave(lambda t: (1.2, 0.0, 1.62), 2.0, 8.0)
    check("waving beyond the follow zone (1.2 m along): not engaged (the arm cannot face them there)",
          en.engagements == 0)
    en, log = run_wave(lambda t: (-1.2 + 0.9 * t, 0.0, 1.62), 0.5, 3.0)
    check("waving while walking past (0.9 m/s, through the zone): not engaged", en.engagements == 0)
    en = Engage(cfg, hub, env0, model)                       # candidate(): asked without stepping (the arm elsewhere)
    for i in range(int(3.0 / en.dt)):
        now = i * en.dt
        if i % 4 == 0:
            en.update([(1, spot[0], spot[1], 1.62, 0.9, now)], now)
        if i == int(0.5 / en.dt):
            early = en.candidate(now)
    check("candidate(): nobody before the dwell, the one on the spot after it -- and nothing moves",
          early is None and en.candidate(now) == 1 and en.state == "OFF", (early, en.candidate(now), en.state))
    en.rest_until = now + 1.0
    check("... nobody while resting after a goodbye", en.candidate(now) is None)
    hidden = lambda t: [p for p in [(1, 0.0, 0.0, 1.62), (2, 0.5, 0.3, 1.7)] if not (p[0] == 1 and 8.0 <= t < 9.2)]
    en, log = run_many(hidden, 14.0)
    check("hidden behind someone for 1.2 s: still followed (%.1f s allowed with people near)" % LOST_CROWD_S,
          first(log, "BYE") is None and log[-1][3] == 1, first(log, "BYE"))
    alone = lambda t: [] if 8.0 <= t < 9.2 else [(1, 0.0, 0.0, 1.62)]
    en, log = run_many(alone, 14.0)
    check("... unseen 1.2 s with nobody near: goodbye (%.1f s)" % LEAVE_S, first(log, "BYE") is not None)
    hover = lambda t: (0.28 + 0.12 * (0.5 + 0.5 * math.sin(2 * math.pi * t / 1.5)), 0.0, 1.62)
    en, log = run(hover, 15.0)
    check("hovering at the spot's edge (0.28-0.40 m out): engaged once, no goodbye from the edge",
          en.engagements == 1 and first(log, "BYE") is None, (en.engagements, first(log, "BYE")))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
