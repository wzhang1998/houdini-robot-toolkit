"""The show (scripts/show.py's state machine and clip graph) in Isaac Sim:
the FR20 from its URDF, the room from the show's layer (shows/<show>.usda: the
room, envs/<room>.usda, with the show's paper and stage over it), lit and
coloured as the lab (isaac_stage.load_room), physics on; the camera on the
whole room (--camera audience / side / 'ex ey ez tx ty tz').

    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json                 window, panel, keys, OSC
    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --headless --minutes 3 --auto-trigger 30
    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --headless --no-osc --minutes 5 \
        --auto-trigger 60 --seed 1 --camera audience --video       the 5 min demo, recorded
    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --osc-out 127.0.0.1:9002 --artnet 6455 \
        --canvas 6457
    C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --headless --no-osc --minutes 5 \
        --auto-trigger 60 --seed 1 --camera audience --canvas-sim --video
        the demo with the paper as TD would show it (canvas_model.py: pixel_scan and canvas_sim in numpy;
        --canvas-image, default TD's banana), no TD needed
        TouchDesigner live: TD hears the show (as from show_stream) and its LEDs come back over Art-Net, drawn
        as the strip's 60 LEDs, and its canvas preview on the paper (--canvas), in real time. Only with show_stream, scan_test and show_ui closed (the one
        OSC port, 9000; TD's STOP here holds the arm in Isaac) and TD's Controller IP cleared.

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
ap.add_argument("--auto-triggers", default="scan",
                help="what --auto-trigger fires, in turn, comma separated (e.g. scan,low_wipe_rows,scan,greet_wipe_cols)")
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
ap.add_argument("--guides", action="store_true", help="draw the safety guides (zones' outlines); hidden by default")
ap.add_argument("--no-guides", action="store_true", help=argparse.SUPPRESS)          # the default now
ap.add_argument("--osc-out", action="append", default=[], metavar="HOST:PORT",
                help="the status to this target too (TouchDesigner: 127.0.0.1:9002), as show_stream --osc-out")
ap.add_argument("--artnet", type=int, default=0, metavar="PORT",
                help="draw the strip's 60 LEDs as TouchDesigner sends them (Art-Net on 127.0.0.1:PORT, e.g. 6455: "
                     "a second DMX Out CHOP aimed here); implies --realtime")
ap.add_argument("--led0", choices=("minus", "plus"), default="minus", help="the strip's end LED 0 is at (flange y)")
ap.add_argument("--led-gain", type=float, default=1.0, help="brighten the drawn LEDs (a dim, capped pattern)")
ap.add_argument("--realtime", action="store_true", help="the show's clock on the wall clock (TD live)")
ap.add_argument("--canvas", type=int, default=0, metavar="PORT",
                help="TD's canvas preview on the paper (canvas_link on 127.0.0.1:PORT, e.g. 6457), in place of the "
                     "flat exposed area; implies --realtime")
ap.add_argument("--canvas-sim", action="store_true",
                help="the paper as TD's canvas preview would show it, without TD (canvas_model.py): for videos")
ap.add_argument("--canvas-image", default="", help="the image the --canvas-sim scan writes (default: TD's banana)")
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

from isaac_stage import (PHYSICS_DT, attach_tool, camera_spec, contact_paths, encode_video, import_robot,  # noqa: E402
                         load_room, render_settings, use_camera)

FPS_VIDEO = 30


def main():
    cfg_path = os.path.abspath(args.config)
    cfg = json.load(open(cfg_path))
    graph = show.Graph.load(show.compiled_path(cfg_path))
    show.require_fresh(graph, cfg_path)
    runner = show.runner_for(graph, seed=args.seed, log=lambda *a: print("[show]", *a))

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    env = load_room(stage, cfg_path, args.look, guides=args.guides and not args.no_guides)                           # the show's room, lit as the lab
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
            also = [(h, int(pt)) for h, pt in (x.rsplit(":", 1) for x in args.osc_out)]
            runner.stop = lambda: runner.fault("stop (TouchDesigner)")  # /robot/stop: held, as a stopped stream
            bridge = show.OscBridge(runner, o["listen_port"], o["send_host"], o["send_port"], also=also, cfg=cfg)
            print("[show] OSC in :%d, out %s:%d%s" % (o["listen_port"], o["send_host"], o["send_port"],
                                                    "".join(", %s:%d" % x for x in also)))
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
    auto_names, n_trig = [x.strip() for x in args.auto_triggers.split(",") if x.strip()] or ["scan"], 0
    t_trig, waits, worst, sq, n = None, [], [0.0] * 6, [0.0] * 6, 0
    end = args.minutes * 60.0 if args.minutes else None
    frames, shots, corner = None, 0, []
    viz, viz_u = None, -1.0
    if cfg.get("scan"):                                        # the scan's area, the lit strip, the paper exposed
        from scan_viz import ScanViz
        viz = ScanViz(stage, cfg)
    leds, rx = None, None
    if args.artnet:
        import artnet
        from led_viz import LedViz
        rx = artnet.Receiver(args.artnet)
        leds = LedViz(stage, led0=args.led0, gain=args.led_gain)
        print("[show] LEDs from Art-Net on 127.0.0.1:%d" % args.artnet)
    canvas_rx, canvas = None, None
    if args.canvas and cfg.get("scan"):
        import canvas_link
        from canvas_viz import CanvasViz
        canvas_rx = canvas_link.Receiver(args.canvas)
        canvas = CanvasViz(stage, cfg)
        print("[show] canvas preview from TD on 127.0.0.1:%d" % args.canvas)
    model, img_levels, t_model, canvas_seen = None, None, 0.0, -1
    if args.canvas_sim and cfg.get("scan") and canvas is None:
        import canvas_model as CM
        from canvas_viz import CanvasViz
        area_w, area_h = show.scan_area(cfg)[1:]
        model = CM.CanvasModel(max(1, int(round(90 * area_w / area_h))), 90)
        img_levels = CM.prepare_image(args.canvas_image or CM.BANANA)
        canvas = CanvasViz(stage, cfg)
        canvas.update(*model.view())
        print("[show] canvas simulated (canvas_model.py): %s" % (args.canvas_image or CM.BANANA))
    realtime = args.realtime or bool(args.artnet) or bool(args.canvas)
    wall0, steps = time.monotonic(), 0
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
        steps += 1
        draw = ((not args.headless) and (not realtime or steps % 2 == 0)       # real time: drawn at 60 Hz
                or bool(args.snapshot) or shoot)
        if leds is not None and draw:
            leds.update(rx.poll(), q)
        if viz is not None and draw:
            s0 = runner.status()
            if s0["state"] == "TO_SCAN":
                viz_u = -1.0                                   # a new pass: the paper fresh (it fades between)
            elif s0["state"] == "SCAN":
                viz_u = max(viz_u, s0["scan_u"])
            elif viz_u >= 0.0:
                viz_u = 2.0                                    # after the pass: all of it, until the next
            img = None
            if canvas_rx is not None:                                # a new image only (TD sends 15 a second)
                got = canvas_rx.poll()
                if got is not None and canvas_rx.packets != canvas_seen:
                    img, canvas_seen = got, canvas_rx.packets
            on = s0["state"] == "SCAN" and bool(s0["scan_led"])
            if model is not None and (runner.clock - t_model >= 1.0 / 15 or shoot):    # TD's model without TD
                u = s0["scan_u"] if on else -1.0
                model.step(runner.clock - t_model, u, CM.leds_at(img_levels, u) * CM.PIXEL_SCAN["master"])
                t_model = runner.clock
                img = model.view()
            if img is not None:
                canvas.update(*img)                                  # TD's paper instead of the flat area
            painted = canvas is not None and canvas.size is not None
            viz.update(-1.0 if painted else viz_u, bool(s0["scan_led"]) and leds is None, q)
        world.step(render=draw)
        if realtime:
            ahead = runner.clock - (time.monotonic() - wall0)
            if ahead > 0.0:
                time.sleep(ahead)
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
            name = auto_names[n_trig % len(auto_names)]
            n_trig += 1
            runner.trigger(name)
            if name == "scan":
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
            if rx is not None:
                age = rx.age()
                label.text += "\nLEDs: " + ("waiting for TD (Art-Net :%d)" % args.artnet if age is None else
                                            "%d packets, last %.1f s ago" % (rx.packets, age))
            if canvas_rx is not None:
                age = canvas_rx.age()
                label.text += "\nCanvas: " + ("waiting for TD (:%d)" % args.canvas if age is None else
                                              "%d frames, last %.1f s ago" % (canvas_rx.packets, age))
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
    video = (encode_video(app, frames, corner, shots, os.path.join(args.out, "isaac_show_%s.mp4" % stamp), FPS_VIDEO)
             if frames is not None else None)
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
    if rx is not None:
        rx.close()
    if canvas_rx is not None:
        canvas_rx.close()


main()
app.close()
