"""The interactive mode (Stage 11, modes B / C): somebody steps onto a spot on
the floor straight in front of the greet hub and the arm stops its clip,
turns to them -- visibly: it perks up, then settles facing them -- and
follows them: left and right along the wall (B), a raised hand up and down
(C), in a small box around the greet hub's tool point; they step off, are
lost, or 30 s pass: a nod goodbye, back to the hub, the clips go on.

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


def self_test():
    import json
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
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
