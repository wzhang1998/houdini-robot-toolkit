"""A tracking scenario in Isaac Sim: the motion track_eval.py made (the greet
clips with the gaze offsets, from simulated tracking data) played on the
FR20 with physics, the person and the tracked target shown, contacts with
the room reported, and a video from the audience's side.

    uv run scripts/track_eval.py --export geo/tracking                  first: the motions
    C:/isaacsim6/python.bat scripts/isaac/run_tracking.py jump walk_across --headless --video
    C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --all --headless
    C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live
        live: the greet clips play on; drag the person's head (select it, W, drag; the body follows) and
        the tracking layer (tracking.py, as show_stream would run it) turns the arm to it in real time;
        "Person there" off: the person lost. --headless --minutes 2: the heads move by themselves (a check).
    C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --people 3
        three people: tracking.Attention picks whom to look at (they take turns; a passer-by is not looked at);
        the one looked at turns green
    C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --engage
        the interactive mode: drag the head onto the ring in front of greet -- the arm stops, perks up, turns to
        you and follows you (B: walk up to 0.8 m either way); drag the pink hand up to the chest or higher (C);
        walk away: a nod, back

Per scenario: geo/tracking/<name>.json in, geo/tracking/<name>_isaac.json out
(tracking error of the simulated arm against the commands, contacts between
the arm and the room) and, with --video, geo/tracking/<name>.mp4. The
person: a body and a head at the real position; the tracked target: a green
ring around the head, hidden while nobody is tracked; the gaze: a green
line from the tool point along the tool axis (where the strip faces).
"""

import argparse
import glob
import json
import math
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)

ap = argparse.ArgumentParser()
ap.add_argument("scenarios", nargs="*")
ap.add_argument("--live", action="store_true", help="drag the target in a window; the tracking layer runs live")
ap.add_argument("--minutes", type=float, default=0.0, help="--live: stop after this long (0: until closed)")
ap.add_argument("--seed", type=int, default=1, help="--live: the greet clips' order")
ap.add_argument("--engage", action="store_true",
                help="--live: the interactive mode (engage.py): stand on the spot in front of greet (the ring on "
                     "the floor), the arm stops its clip and follows you (B); drag the pink hand up (C)")
ap.add_argument("--people", type=int, default=1, help="--live: this many heads to drag (several: Attention picks one)")
ap.add_argument("--all", action="store_true")
ap.add_argument("--config", default=os.path.join(ROOT, "shows", "party.json"))
ap.add_argument("--dir", default=os.path.join(ROOT, "geo", "tracking"))
ap.add_argument("--headless", action="store_true")
ap.add_argument("--video", action="store_true", help="an mp4 per scenario from the audience's side")
ap.add_argument("--camera", default="", help="room, audience, side, or 'ex ey ez tx ty tz [focal]' (robot frame); "
                                          "default: behind the audience zone, above the person")
ap.add_argument("--look", default="room", choices=("room", "plain"), help="room: lit as the lab (room_look.py)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720, "renderer": "RaytracedLighting"})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402

from isaac_stage import (PHYSICS_DT, attach_tool, camera_spec, contact_paths, import_robot, load_room,  # noqa: E402
                         render_settings, use_camera)

FPS_VIDEO = 30


def audience_camera(cfg):
    """Behind the audience zone's middle, eye height, towards the greet hub."""
    a = cfg["zones"]["audience"]
    yaw = math.radians(a.get("yaw_deg", 0.0))
    across = (-math.sin(yaw), math.cos(yaw))
    c = a["center"]
    eye = [c[0] - across[0] * 2.2, c[1] - across[1] * 2.2, 2.3]
    g = cfg["hubs"]["greet"]["tcp"]
    return eye, [g[0], g[1], g[2] - 0.2]


def marker(stage, path, radius, rgb):
    s = UsdGeom.Sphere.Define(stage, Sdf.Path(path))
    s.CreateRadiusAttr(radius)
    s.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    op = s.AddTranslateOp()
    return s, op


def body(stage, path="/World/Person"):
    """A person: a column for the body, a sphere for the head (moved together)."""
    root = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    op = root.AddTranslateOp()
    col = UsdGeom.Cylinder.Define(stage, Sdf.Path(path + "/Body"))
    col.CreateRadiusAttr(0.17)
    col.CreateHeightAttr(1.4)
    col.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.3, 0.45)])
    col.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.9))     # below the head
    head = UsdGeom.Sphere.Define(stage, Sdf.Path(path + "/Head"))
    head.CreateRadiusAttr(0.11)
    head.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.7, 0.55)])
    return root, op


def ring(stage, path, radius, rgb):
    """A torus-like ring (a closed curve) facing the camera's way, moved as one."""
    root = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    op = root.AddTranslateOp()
    c = UsdGeom.BasisCurves.Define(stage, Sdf.Path(path + "/Curve"))
    pts = [Gf.Vec3f(radius * math.cos(a), radius * math.sin(a), 0.0) for a in [k * math.pi / 16 for k in range(33)]]
    c.CreatePointsAttr(pts)
    c.CreateCurveVertexCountsAttr([len(pts)])
    c.CreateTypeAttr("linear")
    c.CreateWidthsAttr([0.02] * len(pts))
    c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    return root, op


def line(stage, path, rgb):
    c = UsdGeom.BasisCurves.Define(stage, Sdf.Path(path))
    c.CreateTypeAttr("linear")
    c.CreateCurveVertexCountsAttr([2])
    c.CreateWidthsAttr([0.012, 0.012])
    c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    c.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(0, 0, 0.01)])
    return c


def show_or_hide(prim, on):
    img = UsdGeom.Imageable(prim)
    (img.MakeVisible if on else img.MakeInvisible)()


SKIN, LOOKED = (0.85, 0.7, 0.55), (0.35, 0.9, 0.45)


def walker(k, n, home, along, now):
    """--live --headless: person k of n at `now` -- (where the head is, there?).
    Alone: walking to and fro across the audience, gone 30-33 s. In a crowd:
    person 0 stands, person 1 stands and leaves 40-45 s, person 2 walks
    quickly across and back (a passer-by), the rest stand."""
    if n == 1:
        x = home[0] + 0.8 * math.sin(2.0 * math.pi * now / 12.0)
        z = home[2] + 0.08 * math.sin(2.0 * math.pi * now / 3.0)
        return (x, home[1], z), not 30.0 <= now % 60.0 < 33.0
    if k == 2:                                                   # a passer-by: 1.2 m/s across, every 15 s
        ph = now % 15.0
        u = -1.4 + 1.2 * ph if ph < 2.4 else None
        if u is None:
            return home, False
        return (home[0] + along[0] * u, home[1] + along[1] * u, home[2]), True
    z = home[2] + 0.02 * math.sin(2.0 * math.pi * now / (3.0 + k))
    return (home[0], home[1], z), not (k == 1 and 40.0 <= now % 60.0 < 45.0)


def engage_walker(en, now):
    """--live --engage --headless: person 1 -- off the spot, onto it at 3 s,
    walking 0.6 m left and right of it from 8 s (inside the follow zone), (a
    hand up 16-21 s), off at 24 s, back on at 32 s, staying past the 30 s cap."""
    on = 3.0 <= now < 24.0 or now >= 32.0
    du = 0.6 * math.sin(2.0 * math.pi * (now - 8.0) / 8.0) if 8.0 <= now < 24.0 else 0.0
    dv = 0.0 if on else 1.2
    ax, tw = en.axes[0], en.axes[2]
    return (en.spot[0] + du * ax[0] - dv * tw[0], en.spot[1] + du * ax[1] - dv * tw[1], 1.62)


def live():
    """The tracking layer live: the greet clips back to back (track_eval's
    base motion), --people heads to drag (each its own prim; a body follows
    each), fed at ~30 Hz -- one person straight to TargetInput, several
    through Attention (whom to look at; the one looked at turns green) --
    TargetInput + Gaze each physics tick, in real time."""
    import time
    import collision as C
    import show as S
    import track_eval as TE
    import track_sim as TS
    import tracking as TR
    cfg = json.load(open(args.config))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    graph = S.Graph.load(S.compiled_path(os.path.abspath(args.config)))
    model = C.load_model("fr20")
    base, clips = TE.base_motion(graph, "greet", 4 * 3600.0, seed=args.seed)
    ti, att = TR.TargetInput(), TR.Attention()
    new_gaze = lambda: TR.Gaze(anchor=graph.hubs["greet"], dt=PHYSICS_DT, env=env, model=model)  # noqa: E731
    gz = new_gaze()
    en = player = None
    if args.engage:
        import engage as EN
        player = EN.ClipPlayer(graph, "greet", args.seed)
        base = player.at
        en = EN.Engage(cfg, graph.hubs["greet"], env, model, dt=PHYSICS_DT)
    eyes = S.audience_eyes(cfg)
    _, along, _ = TS.zone_frame(cfg)
    n = max(1, args.people)
    homes = [tuple(eyes[i] + along[i] * (k - (n - 1) / 2.0) * 0.7 for i in range(2)) + (eyes[2] + 0.03 * k,)
             for k in range(n)]

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    room = load_room(stage, os.path.abspath(args.config), args.look)
    robot = world.scene.add(SingleArticulation(import_robot(), name="fr20"))
    attach_tool(stage)
    heads, bodies = [], []
    for k in range(n):                     # a head to drag (its own prim, so the gizmo moves it), a body under it
        h, h_op = marker(stage, "/World/LiveHead%d" % (k + 1), 0.11, SKIN)
        h_op.Set(Gf.Vec3d(*homes[k]))
        b = UsdGeom.Cylinder.Define(stage, Sdf.Path("/World/LiveBody%d" % (k + 1)))
        b.CreateRadiusAttr(0.17)
        b.CreateHeightAttr(1.4)
        b.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.3, 0.45)])
        heads.append((h, h_op))
        bodies.append((b, b.AddTranslateOp()))
    target, target_op = ring(stage, "/World/Target", 0.16, (0.2, 0.95, 0.3))
    gaze = line(stage, "/World/Gaze", (0.2, 0.95, 0.3))
    hand = spot_curve = None
    if en is not None:                                   # the spot on the floor, a hand for person 1 (mode C)
        spot_root, spot_op = ring(stage, "/World/Spot", en.r, (0.9, 0.85, 0.3))
        spot_op.Set(Gf.Vec3d(en.spot[0], en.spot[1], 0.012))
        spot_curve = UsdGeom.BasisCurves(stage.GetPrimAtPath("/World/Spot/Curve"))
        hand, hand_op = marker(stage, "/World/LiveHand1", 0.06, (1.0, 0.45, 0.7))
        hand_home = tuple(homes[0][i] + along[i] * 0.25 for i in range(2)) + (homes[0][2] - 0.5,)
        hand_op.Set(Gf.Vec3d(*hand_home))
    world.reset()
    dof = list(robot.dof_names)
    idx = np.array([dof.index("j%d" % i) for i in range(1, 7)])
    robot.set_joint_positions(np.radians(base(0.0)), joint_indices=idx)

    contacts = []
    try:
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType

        def on_contact(headers, data):
            for hd in headers:
                if hd.type == ContactEventType.CONTACT_FOUND:
                    a, b = contact_paths(hd)
                    if ("/Room" in a) != ("/Room" in b) and "/Live" not in a + b:
                        contacts.append((a, b))
        sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)  # noqa: F841
    except Exception as e:
        print("[track] contact reports unavailable: %s" % e)
    render_settings()
    eye, look = audience_camera(cfg)
    use_camera(stage, "/World/TrackCam", *(camera_spec(args.camera, cfg, room) if args.camera else (eye, look, 16.0)))

    there, label = [True] * n, None
    if not args.headless:
        import omni.ui as ui
        win = ui.Window("Tracking", width=400, height=180 + 28 * n)
        with win.frame:
            with ui.VStack(spacing=6):
                label = ui.Label("", height=110, word_wrap=True)
                for k in range(n):
                    with ui.HStack(height=24):
                        box = ui.CheckBox(width=24)
                        box.model.set_value(True)
                        box.model.add_value_changed_fn(lambda m, k=k: there.__setitem__(k, m.get_value_as_bool()))
                        ui.Label("Person %d there (off: gone)" % (k + 1))

                def home_all():
                    for k in range(n):
                        heads[k][1].Set(Gf.Vec3d(*homes[k]))
                ui.Button("Heads back to the audience", clicked_fn=home_all)
        print("[track] live: select a head, W, drag it; the 'Tracking' panel says who is there%s"
              % ("; the one looked at turns green" if n > 1 else ""))

    wall0, end = time.monotonic(), (args.minutes * 60.0 if args.minutes else None)
    worst, ticks, feed = 0.0, 0, int(round(1.0 / (30.0 * PHYSICS_DT)))
    tracked_ticks, looked, green, looked_at = 0, None, None, set()
    states, lit, vel_ratio, prev_q = {}, None, 0.0, None
    import robot_profile as RP
    vlim = RP.velocity_limits(RP.load("fr20"))
    while app.is_running():
        now = ticks * PHYSICS_DT
        if player is not None:
            player.advance(now)
        pos = []
        for k, (h, h_op) in enumerate(heads):
            if args.headless and en is not None and k == 0:
                p, there[k] = engage_walker(en, now), True
                h_op.Set(Gf.Vec3d(*p))
            elif args.headless:
                p, there[k] = walker(k, n, homes[k], along, now)
                h_op.Set(Gf.Vec3d(*p))
            else:
                p = tuple(UsdGeom.Xformable(h.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                          .ExtractTranslation())
            pos.append(p)
        hand_p = None
        if hand is not None:
            if args.headless:
                hand_p = (pos[0][0] + 0.2 * along[0], pos[0][1] + 0.2 * along[1],
                          2.0 if 16.0 <= now < 21.0 else pos[0][2] - 0.5)          # raised 16-21 s
                hand_op.Set(Gf.Vec3d(*hand_p))
            else:
                hand_p = tuple(UsdGeom.Xformable(hand.GetPrim()).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()).ExtractTranslation())
        if en is not None and ticks % feed == 0:
            en.update([(k + 1,) + pos[k] + (1.0, now) for k in range(n) if there[k]], now,
                      [(1,) + hand_p + (1.0, now)] if hand_p and there[0] else [])
        if ticks % feed == 0:
            if n == 1:
                if there[0]:
                    ti.target(pos[0][0], pos[0][1], pos[0][2], 1.0, now, 0, now=now)
            else:
                att.update([(k + 1,) + pos[k] + (1.0, now) for k in range(n) if there[k]], now)
                ch = att.choose(now)
                looked = ch[1] if ch else None
                if ch is not None:
                    ti.target(ch[0][0], ch[0][1], ch[0][2], 0.9, now, ch[1], now=now)
                elif ti.seen is not None:
                    ti.lost()
        tgt = ti.now(now)
        if en is None or en.state == "OFF":
            q = gz.step(base(now), tgt, now, [base(now + d) for d in TR.AHEAD_S])
        if en is not None:
            q = en.step(q, now)
            states[en.state] = states.get(en.state, 0) + 1
            if en.resume:                                    # back at the hub: the clips go on, no gaze left over
                player.restart(now)
                gz = new_gaze()
            if en.state != lit:                              # the spot lights up while it is on
                spot_curve.GetDisplayColorAttr().Set([Gf.Vec3f(*((0.25, 1.0, 0.4) if en.state != "OFF"
                                                                 else (0.9, 0.85, 0.3)))])
                lit = en.state
        if prev_q is not None:
            vel_ratio = max(vel_ratio, max(abs(a - b) / PHYSICS_DT / v for a, b, v in zip(q, prev_q, vlim)))
        prev_q = list(q)
        robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=idx))
        for k in range(n):
            bodies[k][1].Set(Gf.Vec3d(pos[k][0], pos[k][1], pos[k][2] - 0.9))    # the body under the head
            show_or_hide(bodies[k][0].GetPrim(), there[k])
            show_or_hide(heads[k][0].GetPrim(), there[k] or not args.headless)  # a head stays draggable
        if n > 1 and looked != green:
            for k in range(n):
                heads[k][0].GetDisplayColorAttr().Set([Gf.Vec3f(*(LOOKED if k + 1 == looked else SKIN))])
            green = looked
        show_or_hide(target.GetPrim(), tgt is not None)
        if tgt is not None:
            target_op.Set(Gf.Vec3d(*tgt))
            tracked_ticks += 1
            if looked is not None:
                looked_at.add(looked)
        R, tcp, _ = S.tool_pose(q)
        ax = (R[0][2], R[1][2], R[2][2])
        gaze.GetPointsAttr().Set([Gf.Vec3f(*tcp), Gf.Vec3f(*[tcp[j] + 1.5 * ax[j] for j in range(3)])])
        world.step(render=not args.headless and ticks % 2 == 0)
        if now > 0.5:
            sim = np.degrees(robot.get_joint_positions(joint_indices=idx))
            worst = max(worst, max(abs(float(sim[j]) - q[j]) for j in range(6)))
        if label is not None and ticks % 15 == 0:
            off = gz.offsets
            who = ("person %d" % looked if looked else "no one") if n > 1 else ("TRACKED" if tgt is not None else "no one")
            label.text = ("%s%s   offsets J1 %+.1f  J5 %+.1f deg\nunsafe %d   shrunk %d   held (slow zone) %d   "
                          "contacts %d\ntracking error %.2f deg   %.0f s" % (
                              "looking at " if n > 1 else "", who, off[0], off[1], gz.unsafe, gz.shrunk,
                              getattr(gz, "held_slow", 0), len(contacts), worst, now))
            if en is not None:
                label.text = ("INTERACTIVE: %s%s   (%d so far)\n" % (
                    en.state, (" " + en.mode) if en.state == "TRACK" else "", en.engagements)
                    if en.state != "OFF" else "clips (stand on the ring to start)\n") + label.text
        ticks += 1
        if not args.headless:
            ahead = now - (time.monotonic() - wall0)
            if ahead > 0.0:
                time.sleep(ahead)
        if end is not None and now >= end:
            break
    out = {"people": n, "live_seconds": round(ticks * PHYSICS_DT, 1),
           "tracked_share": round(tracked_ticks / max(1, ticks), 2), "looked_at": sorted(looked_at),
           "turns": att.turns, "unsafe_ticks": gz.unsafe, "shrunk_ticks": gz.shrunk,
           "held_slow_ticks": getattr(gz, "held_slow", 0), "arm_room_contacts": len(contacts),
           "tracking_max_deg": round(worst, 3), "wall_s": round(time.monotonic() - wall0, 1),
           "joint_speed_of_limit": round(vel_ratio, 3)}
    if en is not None:
        out["engage"] = {"engagements": en.engagements, "refused": en.refused, "unsafe_ticks": en.unsafe,
                         "seconds_in": {k: round(v * PHYSICS_DT, 1) for k, v in states.items()}}
    print("[track] live %s" % json.dumps(out), flush=True)


def main():
    cfg = json.load(open(args.config))
    names = sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(args.dir, "*.json"))
                   if not p.endswith("_isaac.json")) if args.all else args.scenarios
    if not names:
        raise SystemExit("which scenarios? (--all, or names; first: uv run scripts/track_eval.py --export %s)" % args.dir)
    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    env = load_room(stage, os.path.abspath(args.config), args.look)      # the show's room, lit as the lab
    prim_path = import_robot()
    attach_tool(stage)
    robot = world.scene.add(SingleArticulation(prim_path, name="fr20"))
    person, person_op = body(stage)
    crowd = [body(stage, "/World/Crowd%d" % k) for k in range(6)]          # a crowd scenario's people
    target, target_op = ring(stage, "/World/Target", 0.16, (0.2, 0.95, 0.3))
    gaze = line(stage, "/World/Gaze", (0.2, 0.95, 0.3))
    import gestures as G
    rig = G.Rig()
    world.reset()
    dof = list(robot.dof_names)
    idx = np.array([dof.index("j%d" % i) for i in range(1, 7)])

    contacts = []
    clock = [0.0]
    try:
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType

        def on_contact(headers, data):
            for h in headers:
                if h.type == ContactEventType.CONTACT_FOUND:
                    a, b = contact_paths(h)
                    if ("/Room" in a) != ("/Room" in b) and "/Person" not in a + b and "/Target" not in a + b:
                        contacts.append((round(clock[0], 3), a, b))
        sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)  # noqa: F841
    except Exception as e:
        print("[track] contact reports unavailable: %s" % e)

    from omni.kit.viewport.utility import capture_viewport_to_file
    render_settings()
    eye, look = audience_camera(cfg)
    view = camera_spec(args.camera, cfg, env) if args.camera else (eye, look, 16.0)
    vp = use_camera(stage, "/World/TrackCam", *view)

    results = {}
    for name in names:
        d = json.load(open(os.path.join(args.dir, name + ".json")))
        ts = [k * d["dt"] for k in range(len(d["q"]))]
        q0 = d["q"][0]
        robot.set_joint_positions(np.radians(q0), joint_indices=idx)
        robot.set_joint_velocities(np.zeros(6), joint_indices=idx)
        for _ in range(30):                                     # settle at the first pose
            robot.apply_action(ArticulationAction(joint_positions=np.radians(q0), joint_indices=idx))
            world.step(render=False)
        del contacts[:]
        frames = os.path.join(args.dir, "_frames_" + name)
        if args.video:
            shutil.rmtree(frames, ignore_errors=True)
            os.makedirs(frames)
        worst, sq, n, f = [0.0] * 6, [0.0] * 6, 0, 0
        steps = int(ts[-1] / PHYSICS_DT)
        for i in range(steps):
            t = i * PHYSICS_DT
            clock[0] = t
            k = min(int(t / d["dt"]), len(ts) - 2)
            a = (t - ts[k]) / d["dt"]
            q = [x + a * (y - x) for x, y in zip(d["q"][k], d["q"][k + 1])]
            robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=idx))
            p, g = d["truth"][k], d["target"][k]
            if d.get("people"):                                  # everybody there; the ring marks the one looked at
                here = d["people"][k]
                for c, (root, op) in enumerate(crowd):
                    show_or_hide(root.GetPrim(), c < len(here))
                    if c < len(here):
                        op.Set(Gf.Vec3d(*here[c][1:]))
                p = None
            else:
                for root, _ in crowd:
                    show_or_hide(root.GetPrim(), False)
            show_or_hide(person.GetPrim(), p is not None)
            if p is not None:
                person_op.Set(Gf.Vec3d(*p))
            show_or_hide(target.GetPrim(), g is not None)
            if g is not None:
                target_op.Set(Gf.Vec3d(*g))
            R, tcp, _ = rig.tool(q)
            ax = (R[0][2], R[1][2], R[2][2])
            gaze.GetPointsAttr().Set([Gf.Vec3f(*tcp), Gf.Vec3f(*[tcp[j] + 1.5 * ax[j] for j in range(3)])])
            shoot = args.video and i % int(round(1.0 / (FPS_VIDEO * PHYSICS_DT))) == 0
            world.step(render=(not args.headless) or shoot)
            if shoot:
                capture_viewport_to_file(vp, os.path.join(frames, "f_%05d.png" % f))
                f += 1
            sim = np.degrees(robot.get_joint_positions(joint_indices=idx))
            if t > 0.5:
                for j in range(6):
                    e = abs(float(sim[j]) - q[j])
                    worst[j] = max(worst[j], e)
                    sq[j] += e * e
                n += 1
        out = {"scenario": name, "offline": d["result"], "sim_seconds": round(ts[-1], 2),
               "tracking_max_deg": [round(x, 3) for x in worst],
               "tracking_rms_deg": [round(math.sqrt(s / n), 4) for s in sq],
               "arm_room_contacts": len(contacts), "first_contacts": contacts[:5]}
        if args.video:
            for _ in range(60):
                app.update()                                   # the last captures written
            mp4 = os.path.join(args.dir, name + ".mp4")
            r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS_VIDEO), "-i",
                                os.path.join(frames, "f_%05d.png"), "-vf", "scale=1280:-2", "-pix_fmt", "yuv420p",
                                "-c:v", "libx264", "-crf", "22", mp4], capture_output=True, text=True)
            out["video"] = os.path.relpath(mp4, ROOT).replace("\\", "/") if r.returncode == 0 else r.stderr[-300:]
            shutil.rmtree(frames, ignore_errors=True)
        json.dump(out, open(os.path.join(args.dir, name + "_isaac.json"), "w"), indent=1)
        results[name] = out
        print("[track] %-14s contacts %d, tracking max %.3f deg%s" % (
            name, len(contacts), max(worst), ("  " + out["video"]) if args.video else ""), flush=True)
    bad = [k for k, r in results.items() if r["arm_room_contacts"]]
    print("[track] %s" % ("contacts in: " + ", ".join(bad) if bad else "no contact in %d scenarios" % len(results)))
    if not args.headless:
        while app.is_running():
            world.step(render=True)


if args.live:
    live()
else:
    main()
app.close()
