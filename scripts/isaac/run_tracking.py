"""A tracking scenario in Isaac Sim: the motion track_eval.py made (the greet
clips with the gaze offsets, from simulated tracking data) played on the
FR20 with physics, the person and the tracked target shown, contacts with
the room reported, and a video from the audience's side.

    uv run scripts/track_eval.py --export geo/tracking                  first: the motions
    C:/isaacsim6/python.bat scripts/isaac/run_tracking.py jump walk_across --headless --video
    C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --all --headless
    C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live
        live: the greet clips play on; drag the orange ball (the person's head: select it, W, drag) and
        the tracking layer (tracking.py, as show_stream would run it) turns the arm to it in real time;
        "Person there" off: the person lost. --headless --minutes 2: the ball walks by itself (a check).

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


def body(stage):
    """A person: a column for the body, a sphere for the head (moved together)."""
    root = UsdGeom.Xform.Define(stage, Sdf.Path("/World/Person"))
    op = root.AddTranslateOp()
    col = UsdGeom.Cylinder.Define(stage, Sdf.Path("/World/Person/Body"))
    col.CreateRadiusAttr(0.17)
    col.CreateHeightAttr(1.4)
    col.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.3, 0.45)])
    col.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.9))     # below the head
    head = UsdGeom.Sphere.Define(stage, Sdf.Path("/World/Person/Head"))
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


def walker(eyes, now):
    """--live --headless: where the ball is at `now` and whether anybody is
    there -- walking to and fro across the audience, nodding, gone 30-33 s."""
    x = eyes[0] + 0.8 * math.sin(2.0 * math.pi * now / 12.0)
    z = eyes[2] + 0.08 * math.sin(2.0 * math.pi * now / 3.0)
    return (x, eyes[1], z), not 30.0 <= now % 60.0 < 33.0


def live():
    """The tracking layer live: the greet clips back to back (track_eval's
    base motion), a ball to drag as the person's head, the target fed at
    ~30 Hz, TargetInput + Gaze each physics tick, in real time."""
    import time
    import collision as C
    import show as S
    import tracking as TR
    import track_eval as TE
    cfg = json.load(open(args.config))
    env = S.show_env(C.load_env(cfg["env"]), cfg, cfg["margins"]["idle_canvas_m"])
    graph = S.Graph.load(S.compiled_path(os.path.abspath(args.config)))
    base, clips = TE.base_motion(graph, "greet", 4 * 3600.0, seed=args.seed)
    ti = TR.TargetInput()
    gz = TR.Gaze(anchor=graph.hubs["greet"], dt=PHYSICS_DT, env=env, model=C.load_model("fr20"))
    eyes = S.audience_eyes(cfg)

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    room = load_room(stage, os.path.abspath(args.config), args.look)
    robot = world.scene.add(SingleArticulation(import_robot(), name="fr20"))
    attach_tool(stage)
    person, person_op = body(stage)
    ball, ball_op = marker(stage, "/World/LiveTarget", 0.09, (1.0, 0.55, 0.1))
    ball_op.Set(Gf.Vec3d(*eyes))
    target, target_op = ring(stage, "/World/Target", 0.16, (0.2, 0.95, 0.3))
    gaze = line(stage, "/World/Gaze", (0.2, 0.95, 0.3))
    world.reset()
    dof = list(robot.dof_names)
    idx = np.array([dof.index("j%d" % i) for i in range(1, 7)])
    q0 = base(0.0)
    robot.set_joint_positions(np.radians(q0), joint_indices=idx)

    contacts = []
    try:
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType

        def on_contact(headers, data):
            for h in headers:
                if h.type == ContactEventType.CONTACT_FOUND:
                    a, b = contact_paths(h)
                    if ("/Room" in a) != ("/Room" in b) and "/Person" not in a + b and "/Live" not in a + b:
                        contacts.append((a, b))
        sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)  # noqa: F841
    except Exception as e:
        print("[track] contact reports unavailable: %s" % e)
    render_settings()
    eye, look = audience_camera(cfg)
    use_camera(stage, "/World/TrackCam", *(camera_spec(args.camera, cfg, room) if args.camera else (eye, look, 16.0)))

    there, label = [True], None
    if not args.headless:
        import omni.ui as ui
        win = ui.Window("Tracking", width=380, height=210)
        with win.frame:
            with ui.VStack(spacing=6):
                label = ui.Label("", height=110, word_wrap=True)
                with ui.HStack(height=24):
                    box = ui.CheckBox(width=24)
                    box.model.set_value(True)
                    box.model.add_value_changed_fn(lambda m: there.__setitem__(0, m.get_value_as_bool()))
                    ui.Label("Person there (off: lost)")
                ui.Button("Ball back to the audience", clicked_fn=lambda: ball_op.Set(Gf.Vec3d(*eyes)))
        print("[track] live: select the orange ball, W, drag it; the 'Tracking' panel toggles the person")

    wall0, end = time.monotonic(), (args.minutes * 60.0 if args.minutes else None)
    worst, ticks, feed = 0.0, 0, int(round(1.0 / (30.0 * PHYSICS_DT)))
    tracked_ticks = 0
    while app.is_running():
        now = ticks * PHYSICS_DT
        if args.headless:
            p, there[0] = walker(eyes, now)
            ball_op.Set(Gf.Vec3d(*p))
        else:
            m = UsdGeom.Xformable(ball.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            p = tuple(m.ExtractTranslation())
        if ticks % feed == 0 and there[0]:
            ti.target(p[0], p[1], p[2], 1.0, now, 0, now=now)
        tgt = ti.now(now)
        qb = base(now)
        q = gz.step(qb, tgt, now, [base(now + d) for d in TR.AHEAD_S])
        robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=idx))
        person_op.Set(Gf.Vec3d(*p))
        show_or_hide(person.GetPrim(), there[0])
        show_or_hide(target.GetPrim(), tgt is not None)
        if tgt is not None:
            target_op.Set(Gf.Vec3d(*tgt))
            tracked_ticks += 1
        R, tcp, _ = S.tool_pose(q)
        ax = (R[0][2], R[1][2], R[2][2])
        gaze.GetPointsAttr().Set([Gf.Vec3f(*tcp), Gf.Vec3f(*[tcp[j] + 1.5 * ax[j] for j in range(3)])])
        world.step(render=not args.headless and ticks % 2 == 0)
        if now > 0.5:
            sim = np.degrees(robot.get_joint_positions(joint_indices=idx))
            worst = max(worst, max(abs(float(sim[j]) - q[j]) for j in range(6)))
        if label is not None and ticks % 15 == 0:
            off = gz.offsets
            label.text = ("%s   offsets J1 %+.1f  J5 %+.1f deg\nunsafe %d   shrunk %d   governed %d   contacts %d\n"
                          "tracking error %.2f deg   %s" % ("TRACKED" if tgt is not None else "no one (the clip alone)",
                                                           off[0], off[1], gz.unsafe, gz.shrunk, gz.governed,
                                                           len(contacts), worst,
                                                           "%.0f s" % now))
        ticks += 1
        if not args.headless:
            ahead = now - (time.monotonic() - wall0)
            if ahead > 0.0:
                time.sleep(ahead)
        if end is not None and now >= end:
            break
    out = {"live_seconds": round(ticks * PHYSICS_DT, 1), "tracked_share": round(tracked_ticks / max(1, ticks), 2),
           "unsafe_ticks": gz.unsafe, "shrunk_ticks": gz.shrunk, "governed_ticks": gz.governed,
           "arm_room_contacts": len(contacts), "tracking_max_deg": round(worst, 3),
           "wall_s": round(time.monotonic() - wall0, 1)}
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
