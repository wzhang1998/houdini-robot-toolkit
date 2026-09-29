"""The show (scripts/show.py's state machine and clip graph) in Isaac Sim:
the FR20 from its URDF, the room from the show's layer (shows/<show>.usda: the
room, envs/<room>.usda, with the show's paper and stage over it), lit and
coloured as the lab (isaac_stage.load_room), physics on; the camera on the
whole room (--camera audience / side / 'ex ey ez tx ty tz').

    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json                 window, panel, keys, OSC
    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --headless --minutes 3 --auto-trigger 30
    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --headless --no-osc --minutes 5 \
        --auto-trigger 60 --seed 1 --camera audience --no-guides --video       the 5 min demo, recorded

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
ap.add_argument("--no-tool", action="store_true", help="the bare arm (the profile's tool left off)")
ap.add_argument("--seed", type=int, default=None)
ap.add_argument("--out", default=os.path.join(ROOT, "geo", "isaac"))
ap.add_argument("--snapshot", default="", help="render ~3 s, save the viewport to this PNG, stop")
ap.add_argument("--video", action="store_true",
                help="record <out>/isaac_show_<stamp>.mp4 (30 fps), the state, clip and scan in the corner "
                     "(overlay.py); with --headless --minutes 5 --auto-trigger 60 --no-osc: the demo")
ap.add_argument("--camera", default="room",
                help="room (the whole room), audience, side, or 'ex ey ez tx ty tz [focal]' (robot frame)")
ap.add_argument("--look", default="room", choices=("room", "plain"), help="room: lit as the lab (room_look.py)")
ap.add_argument("--no-guides", action="store_true", help="hide the safety guides (zones' outlines): the demo's look")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900, "renderer": "RaytracedLighting"})

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

import overlay  # noqa: E402
import show  # noqa: E402

from isaac_stage import (PHYSICS_DT, attach_tool, camera_spec, contact_paths, import_robot, load_room,  # noqa: E402
                         render_settings, use_camera)

FPS_VIDEO = 30


def encode(frames, corner, shots, stamp):
    """The frames as an mp4 with the corner's text burnt in (overlay.py);
    its path, relative to the repo. The frames go."""
    import shutil
    import subprocess
    for _ in range(60):
        app.update()                                        # the last captures written
    with open(os.path.join(frames, "overlay.ass"), "w", encoding="utf-8") as f:
        f.write(overlay.ass(overlay.compress(corner, 0.5), 1600, 900, shots / float(FPS_VIDEO)))
    mp4 = os.path.join(args.out, "isaac_show_%s.mp4" % stamp)
    r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS_VIDEO), "-i", "f_%05d.png",
                        "-vf", "ass=overlay.ass", "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "22",
                        os.path.abspath(mp4)], cwd=frames, capture_output=True, text=True)
    if r.returncode:
        print("[show] ffmpeg: %s (frames kept in %s)" % (r.stderr[-400:], frames))
        return None
    shutil.rmtree(frames, ignore_errors=True)
    print("[show] video %s" % mp4)
    return os.path.relpath(mp4, ROOT).replace("\\", "/")


def main():
    cfg_path = os.path.abspath(args.config)
    cfg = json.load(open(cfg_path))
    graph = show.Graph.load(show.compiled_path(cfg_path))
    show.require_fresh(graph, cfg_path)
    runner = show.runner_for(graph, seed=args.seed, log=lambda *a: print("[show]", *a))

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    env = load_room(stage, cfg_path, args.look, guides=not args.no_guides)                           # the show's room, lit as the lab
    prim_path = import_robot()
    if not args.no_tool:
        attach_tool(stage)
    robot =world.scene.add(SingleArticulation(prim_path, name="fr20"))
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
                    a, b = contact_paths(h)
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

    # our own camera (default: the whole room), made the viewport's active one
    render_settings()
    use_camera(stage, "/World/ShowCam", *camera_spec(args.camera, cfg, env))
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
    frames, shots, corner = None, 0, []
    viz, viz_u = None, -1.0
    if cfg.get("scan"):                                        # the scan's area, the lit strip, the paper exposed
        from scan_viz import ScanViz
        viz = ScanViz(stage, cfg)
    if args.video:
        import shutil
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        frames = os.path.join(args.out, "_frames_show_%s" % stamp)
        shutil.rmtree(frames, ignore_errors=True)
        os.makedirs(frames)
    while app.is_running():
        q = runner.step(PHYSICS_DT)
        robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=np.array(idx)))
        shoot = frames is not None and runner.clock * FPS_VIDEO >= shots
        if viz is not None and (shoot or not args.headless):
            s0 = runner.status()
            if s0["state"] == "TO_SCAN":
                viz_u = -1.0                                   # a new pass: the paper fresh (it fades between)
            elif s0["state"] == "SCAN":
                viz_u = max(viz_u, s0["scan_u"])
            elif viz_u >= 0.0:
                viz_u = 2.0                                    # after the pass: all of it, until the next
            viz.update(viz_u, bool(s0["scan_led"]), q)
        world.step(render=(not args.headless) or bool(args.snapshot) or shoot)
        if shoot:
            capture_viewport_to_file(get_active_viewport(), os.path.join(frames, "f_%05d.png" % shots))
            st = runner.status()
            corner.append((shots / float(FPS_VIDEO), overlay.label(st["state"], st["clip"], st["scan"])))
            shots += 1
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
    video = encode(frames, corner, shots, stamp) if frames is not None else None
    played = [c for _, c in runner.history]
    summary = {"config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"), "log": os.path.relpath(log_path, ROOT).replace("\\", "/"),
               "sim_seconds": round(runner.clock, 1), "clips_played": len(played), "scans": played.count("scan"),
               "distinct_idle": len(set(c for c in played if show.graph_kind(graph, c) == "idle")),
               "hubs_visited": sorted(set(show.graph_start(graph, c) for c in played if show.graph_kind(graph, c) == "idle")),
               "trigger_to_scan_s": {"max": round(max(waits), 2) if waits else None,
                                     "mean": round(sum(waits) / len(waits), 2) if waits else None},
               "tracking_max_deg": [round(x, 3) for x in worst],
               "tracking_rms_deg": [round(math.sqrt(s / n), 4) if n else None for s in sq],
               "arm_room_contacts": len(contacts), "first_contacts": contacts[:5],
               "video": video}
    with open(os.path.join(args.out, "isaac_show_%s.json" % stamp), "w") as f:
        json.dump(summary, f, indent=1)
    print("[show] summary " + json.dumps(summary))
    if bridge is not None:
        bridge.close()


main()
app.close()
