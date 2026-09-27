"""The show (scripts/show.py's state machine and clip graph) in Isaac Sim:
the FR20 from its URDF, the room from envs/volvox_lab.usda, physics on.

    C:/isaacsim/python.bat scripts/isaac/run_show.py shows/party.json                 window, panel, keys, OSC
    C:/isaacsim/python.bat scripts/isaac/run_show.py shows/party.json --headless --minutes 3 --auto-trigger 30

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
import re
import sys
import tempfile
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
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900})

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import PhysxSchema, UsdLux, UsdPhysics, Sdf  # noqa: E402

import show  # noqa: E402

PHYSICS_DT = 1.0 / 120.0
URDF = os.path.join(ROOT, "assets", "fairino_description", "urdf", "fairino20_v6.urdf")
PKG = os.path.join(ROOT, "assets", "fairino_description")
STIFFNESS = 1.0e5      # angular drive, per degree (PhysX units): stiff, so tracking shows the dynamics, not a soft spring
DAMPING = 1.0e3


def urdf_with_absolute_meshes():
    """The vendor URDF with package://fairino_description/ resolved (the
    importer does not know the ROS package)."""
    text = open(URDF).read()
    text = re.sub(r"package://fairino_description/", PKG.replace("\\", "/") + "/", text)
    path = os.path.join(tempfile.gettempdir(), "fr20_isaac.urdf")
    open(path, "w").write(text)
    return path


def import_robot():
    from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
    _, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = False
    cfg.fix_base = True
    cfg.import_inertia_tensor = True
    cfg.distance_scale = 1.0
    cfg.make_default_prim = False
    cfg.create_physics_scene = False
    cfg.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
    _, prim_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_with_absolute_meshes(),
                                             import_config=cfg, get_articulation_root=True)
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() in ("j1", "j2", "j3", "j4", "j5", "j6"):
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr().Set(STIFFNESS)
            drive.CreateDampingAttr().Set(DAMPING)
            drive.CreateMaxForceAttr().Set(1.0e6)
        if prim.GetPath().pathString.startswith("/fairino") or "link" in prim.GetName():
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
    return prim_path


def main():
    cfg_path = os.path.abspath(args.config)
    cfg = json.load(open(cfg_path))
    graph = show.Graph.load(show.compiled_path(cfg_path))
    runner = show.Runner(graph, show.Selector(graph.idle(), cfg["select"]["no_repeat"], seed=args.seed),
                         log=lambda *a: print("[show]", *a))

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    room_usd = os.path.join(ROOT, os.path.splitext(cfg["env"])[0] + ".usda")
    if not os.path.exists(room_usd):
        raise SystemExit("no %s: export it first (hython scripts/env_to_usd.py %s %s --show %s)"
                         % (room_usd, cfg["env"], room_usd, args.config))
    add_reference_to_stage(room_usd, "/World/Room")
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(800)
    prim_path = import_robot()
    robot = world.scene.add(SingleArticulation(prim_path, name="fr20"))
    world.reset()
    names = list(robot.dof_names)
    idx = [names.index("j%d" % i) for i in range(1, 7)]
    home = graph.hubs["home"]
    robot.set_joint_positions(np.radians(home), joint_indices=np.array(idx))
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
                        contacts.append((round(runner.clock, 3), a, b))
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

    label = None
    if not args.headless:
        import carb.input
        import omni.appwindow
        import omni.ui as ui
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[2.2, -1.6, 2.3], target=[-0.3, 0.6, 0.8])
        keys = {"T": lambda: runner.trigger("scan"), "P": runner.pause, "R": runner.resume, "X": runner.reset}

        def on_key(e, *a):
            if e.type == carb.input.KeyboardEventType.KEY_PRESS and e.input.name in keys:
                keys[e.input.name]()
            return True
        inp = carb.input.acquire_input_interface()
        inp.subscribe_to_keyboard_events(omni.appwindow.get_default_app_window().get_keyboard(), on_key)
        win = ui.Window("Show", width=340, height=230)
        with win.frame:
            with ui.VStack(spacing=6):
                label = ui.Label("", height=90, word_wrap=True)
                ui.Button("Trigger scan  (T)", clicked_fn=lambda: runner.trigger("scan"))
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
        world.step(render=not args.headless)
        sim = np.degrees(robot.get_joint_positions(joint_indices=np.array(idx)))
        if runner.clock > 1.0:                                     # settle first
            for j in range(6):
                e = abs(sim[j] - q[j])
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
            label.text = "%s   %s\nprogress %.0f%%   scan %s\npending %s%s" % (
                st["state"], st["clip"], 100 * st["progress"],
                ("%.0f%%" % (100 * st["scan"])) if st["scan"] >= 0 else "-", st["pending"] or "-",
                ("\nFAULT " + st["fault"]) if st["fault"] else "")
        if end is not None and runner.clock >= end:
            break
    log.close()
    played = [c for _, c in runner.history]
    summary = {"config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"), "log": os.path.relpath(log_path, ROOT).replace("\\", "/"),
               "sim_seconds": round(runner.clock, 1), "clips_played": len(played), "scans": played.count("scan"),
               "distinct_idle": len(set(c for c in played if c not in ("scan", "to_scan", "from_scan"))),
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
