"""The show (scripts/show.py's state machine and clip graph) in Isaac Sim:
the FR20 from its URDF, the room from envs/volvox_lab.usda, physics on.

    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json                 window, panel, keys, OSC
    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --headless --minutes 3 --auto-trigger 30

The same Runner as the dry run and (next) the real arm: each physics step
asks it for joints and sends them to the arm's position drives. Triggers:
    the "Show" panel's buttons; keys T (scan) P (pause) R (resume) X (reset);
    OSC from TouchDesigner (/robot/trigger ... on :9000, status back on :9001);
    --auto-trigger S: a trigger every ~S s (a test without anybody there).
Writes <out>/isaac_show_<stamp>.csv (time, state, clip, commanded and
simulated joints) and a JSON summary: tracking error (commanded vs the
simulated arm under gravity and its drives), states and clips played,
trigger-to-scan waits, and contacts between the arm and the room (PhysX).
"""

import argparse
import json
import math
import os

import sys

import time

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)

ap = argparse.ArgumentParser()
ap.add_argument("config")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--minutes", type=float, default=0.0, help="stop after this long (0: run until the window closes)")
ap.add_argument("--auto-trigger", type=float, default=0.0, help="a scan trigger every ~S s")
ap.add_argument("--no-osc", action="store_true")
ap.add_argument("--seed", type=int, default=None)
ap.add_argument("--out", default=os.path.join(ROOT, "geo", "isaac"))
ap.add_argument("--snapshot", default="", help="render ~3 s, save the viewport to this PNG, stop")
ap.add_argument("--camera", default="", help="'ex ey ez tx ty tz': the view (robot frame); default: inside the room")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900, "renderer": "RaytracedLighting"})

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import show  # noqa: E402

PHYSICS_DT = 1.0 / 120.0


def show_camera(env, cfg):
    """(eye, target): the room's corner farthest from the operator (keep-out
    zones), 0.35 m in from the walls and just under the ceiling, looking at
    the stage's centre -- a view of the whole working area from inside."""
    import room_geom as RG
    fp = RG.footprint(env)
    z0, z1 = RG.heights(env)
    cx, cy = sum(p[0] for p in fp) / len(fp), sum(p[1] for p in fp) / len(fp)
    people = [o["center"] for o in env["objects"] if o["role"] == "keep_out"] or [(cx, cy)]
    corner = max(fp, key=lambda p: min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in people))
    d = math.hypot(cx - corner[0], cy - corner[1])
    k = min(1.0, 0.9 / d)                                     # ~0.6 m off both walls (clear of a plant in the corner)
    eye = [corner[0] + (cx - corner[0]) * k, corner[1] + (cy - corner[1]) * k, z1 - 0.2]
    st = cfg.get("stage") or next((o for o in env["objects"] if o["name"] == "stage"), None)
    target = [st["center"][0] / 2, st["center"][1] / 2, 0.7] if st else [cx, cy, 0.8]   # between the base and the stage
    return eye, target
URDF = os.path.join(ROOT, "assets", "fairino_description", "urdf", "fairino20_v6.urdf")
PKG = os.path.join(ROOT, "assets", "fairino_description")
STIFFNESS = 1.0e5      # Nm/rad: stiff position drives, so tracking shows the dynamics, not a soft spring
DAMPING = 1.0e3        # Nm*s/rad
ROBOT_USD_DIR = os.path.join(ROOT, "geo", "isaac", "fr20_usd")


def robot_usd():
    """The FR20 as USD, converted from the vendor URDF by Isaac Sim 6's
    importer (URDF stays the source); converted again when the URDF is newer."""
    import omni.kit.app
    em = omni.kit.app.get_app().get_extension_manager()
    for ext in ("omni.scene.optimizer.core", "isaacsim.robot.schema"):
        em.set_extension_enabled_immediate(ext, True)
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    done = os.path.join(ROBOT_USD_DIR, "converted.txt")
    if os.path.exists(done) and os.path.getmtime(done) > os.path.getmtime(URDF):
        return open(done).read().strip()
    os.makedirs(ROBOT_USD_DIR, exist_ok=True)
    cfg = URDFImporterConfig()
    cfg.urdf_path = URDF
    cfg.usd_path = ROBOT_USD_DIR
    cfg.ros_package_paths = [{"name": "fairino_description", "path": PKG.replace("\\", "/")}]
    cfg.fix_base = True
    cfg.merge_fixed_joints = False
    cfg.joint_target_type = "position"
    cfg.override_joint_stiffness = STIFFNESS
    cfg.override_joint_damping = DAMPING
    usd = URDFImporter(cfg).import_urdf()
    open(done, "w").write(usd)
    print("[show] converted the URDF to %s" % usd)
    return usd


def import_robot():
    """Reference the robot USD into the stage; the articulation root's path."""
    usd = robot_usd()
    add_reference_to_stage(usd, "/World/fr20")
    stage = omni.usd.get_context().get_stage()
    root = None
    for prim in stage.Traverse():
        if not prim.GetPath().pathString.startswith("/World/fr20"):
            continue
        if root is None and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            root = prim.GetPath().pathString
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
    if root is None:
        raise SystemExit("no articulation root under /World/fr20 in %s" % usd)
    return root


def main():
    cfg_path = os.path.abspath(args.config)
    cfg = json.load(open(cfg_path))
    graph = show.Graph.load(show.compiled_path(cfg_path))
    runner = show.runner_for(graph, seed=args.seed, log=lambda *a: print("[show]", *a))

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    room_usd = os.path.join(ROOT, os.path.splitext(cfg["env"])[0] + ".usda")
    if not os.path.exists(room_usd):
        raise SystemExit("no %s: export it first (hython scripts/env_to_usd.py %s %s --show %s)"
                         % (room_usd, cfg["env"], room_usd, args.config))
    add_reference_to_stage(room_usd, "/World/Room")
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(600)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Key"))       # a soft key from above
    key.CreateIntensityAttr(2500)
    key.CreateAngleAttr(8.0)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 30.0))
    prim_path = import_robot()
    robot = world.scene.add(SingleArticulation(prim_path, name="fr20"))
    world.reset()
    names = list(robot.dof_names)
    idx = [names.index("j%d" % i) for i in range(1, 7)]
    robot.set_joint_positions(np.radians(runner.seg.q[0]), joint_indices=np.array(idx))
    print("[show] robot %s, dofs %s" % (prim_path, names))

    # contacts between the arm and the room
    contacts = []
    try:
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType

        def on_contact(headers, data):
            for h in headers:
                if h.type == ContactEventType.CONTACT_FOUND:
                    a, b = str(h.actor0), str(h.actor1)
                    if ("/Room" in a) != ("/Room" in b):
                        contacts.append((round(float(runner.clock), 3), a, b))
        sub_contacts = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)  # noqa: F841
    except Exception as e:                                          # the report still has tracking and states
        print("[show] contact reports unavailable: %s" % e)

    bridge = None
    if not args.no_osc:
        o = cfg["osc"]
        try:
            bridge = show.OscBridge(runner, o["listen_port"], o["send_host"], o["send_port"])
            print("[show] OSC in :%d, out %s:%d" % (o["listen_port"], o["send_host"], o["send_port"]))
        except Exception as e:
            print("[show] OSC off: %s" % e)

    # our own camera, inside the room, made the viewport's active one
    import carb.settings
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport
    st = carb.settings.get_settings()
    st.set("/rtx/rendermode", "RaytracedLighting")             # real time, no path-traced grain
    st.set("/rtx/hydra/faceCulling/enabled", True)             # honour the walls' "singleSided": culled from outside
    cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/ShowCam"))
    cam.CreateFocalLengthAttr(13.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    import collision as CL
    eye, target = show_camera(CL.load_env(os.path.join(ROOT, cfg["env"])), cfg)
    if args.camera:
        v = [float(x) for x in args.camera.split()]
        eye, target = v[:3], v[3:6]
    set_camera_view(eye=eye, target=target, camera_prim_path="/World/ShowCam")
    vp = get_active_viewport()
    if vp is not None:
        vp.camera_path = "/World/ShowCam"
    label = None
    if not args.headless:
        import carb.input
        import omni.appwindow as appwindow
        import omni.ui as ui
        pass
        keys = {"T": lambda: runner.trigger("scan"), "P": runner.pause, "R": runner.resume, "X": runner.reset}

        def on_key(e, *a):
            if e.type == carb.input.KeyboardEventType.KEY_PRESS and e.input.name in keys:
                keys[e.input.name]()
            return True
        inp = carb.input.acquire_input_interface()
        inp.subscribe_to_keyboard_events(appwindow.get_default_app_window().get_keyboard(), on_key)
        win = ui.Window("Show", width=360, height=300)
        with win.frame:
            with ui.VStack(spacing=6):
                label = ui.Label("", height=90, word_wrap=True)
                ui.Button("Trigger scan  (T)", clicked_fn=lambda: runner.trigger("scan"))
                with ui.HStack(spacing=6):
                    for name in runner.sequences:
                        ui.Button(name, clicked_fn=lambda n=name: runner.trigger(n))
                with ui.HStack(spacing=6):
                    ui.Button("Pause (P)", clicked_fn=runner.pause)
                    ui.Button("Resume (R)", clicked_fn=runner.resume)
                    ui.Button("Reset (X)", clicked_fn=runner.reset)

    os.makedirs(args.out, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    log_path = os.path.join(args.out, "isaac_show_%s.csv" % stamp)
    log = open(log_path, "w")
    log.write("t,state,clip," + ",".join("cmd_j%d" % i for i in range(1, 7)) + "," +
              ",".join("sim_j%d" % i for i in range(1, 7)) + "\n")
    import random
    rng = random.Random(args.seed)
    next_trig = args.auto_trigger * (0.5 + rng.random()) if args.auto_trigger else None
    t_trig, waits, worst, sq, n = None, [], [0.0] * 6, [0.0] * 6, 0
    end = args.minutes * 60.0 if args.minutes else None
    while app.is_running():
        q = runner.step(PHYSICS_DT)
        robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=np.array(idx)))
        world.step(render=(not args.headless) or bool(args.snapshot))
        sim = np.degrees(robot.get_joint_positions(joint_indices=np.array(idx)))
        if runner.clock > 1.0:                                     # settle first
            for j in range(6):
                e = abs(float(sim[j]) - q[j])
                worst[j] = max(worst[j], e)
                sq[j] += e * e
            n += 1
        st = runner.status()
        log.write("%.4f,%s,%s,%s,%s\n" % (runner.clock, st["state"], st["clip"],
                                          ",".join("%.4f" % x for x in q), ",".join("%.4f" % x for x in sim)))
        if next_trig is not None and runner.clock >= next_trig:
            runner.trigger("scan")
            t_trig = runner.clock
            next_trig = runner.clock + args.auto_trigger * (0.5 + rng.random())
        if t_trig is not None and st["state"] == "SCAN":
            waits.append(runner.clock - t_trig)
            t_trig = None
        if bridge is not None and int(runner.clock / PHYSICS_DT) % 4 == 0:
            bridge.send(q)
        if label is not None:
            label.text = "%s   %s   hub %s   %s\nprogress %.0f%%   scan %s\npending %s%s" % (
                st["state"], st["clip"], st["hub"], st["sequence"] or "", 100 * st["progress"],
                ("%.0f%%" % (100 * st["scan"])) if st["scan"] >= 0 else "-", st["pending"] or "-",
                ("\nFAULT " + st["fault"]) if st["fault"] else "")
        if args.snapshot and runner.clock >= 3.0:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            capture_viewport_to_file(get_active_viewport(), os.path.abspath(args.snapshot))
            for _ in range(30):                                 # let the capture finish
                app.update()
            print("[show] snapshot %s" % args.snapshot)
            break
        if end is not None and runner.clock >= end:
            break
    log.close()
    played = [c for _, c in runner.history]
    summary = {"config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"), "log": os.path.relpath(log_path, ROOT).replace("\\", "/"),
               "sim_seconds": round(runner.clock, 1), "clips_played": len(played), "scans": played.count("scan"),
               "distinct_idle": len(set(c for c in played if show.graph_kind(graph, c) == "idle")),
               "hubs_visited": sorted(set(show.graph_start(graph, c) for c in played if show.graph_kind(graph, c) == "idle")),
               "trigger_to_scan_s": {"max": round(max(waits), 2) if waits else None,
                                     "mean": round(sum(waits) / len(waits), 2) if waits else None},
               "tracking_max_deg": [round(x, 3) for x in worst],
               "tracking_rms_deg": [round(math.sqrt(s / n), 4) if n else None for s in sq],
               "arm_room_contacts": len(contacts), "first_contacts": contacts[:5]}
    with open(os.path.join(args.out, "isaac_show_%s.json" % stamp), "w") as f:
        json.dump(summary, f, indent=1)
    print("[show] summary " + json.dumps(summary))
    if bridge is not None:
        bridge.close()


main()
app.close()
