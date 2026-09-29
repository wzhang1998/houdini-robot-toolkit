"""A recorded run rendered at leisure: the arm, TouchDesigner's LEDs and its
canvas preview as they played live (run_show --record, td_capture.py), 30
frames a second from any camera, the state and clip in the corner -- TD ran
in real time as in the show; the render takes as long as it takes.

    C:/isaacsim6/python.bat scripts/isaac/replay_render.py geo/isaac/td_capture_<stamp>
    C:/isaacsim6/python.bat scripts/isaac/replay_render.py geo/isaac/td_capture_<stamp> --camera room --start 40 --end 70

Writes <capture>/replay_<camera>.mp4. The room as every Isaac view (isaac_stage.load_room), guides hidden.
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, HERE)

ap = argparse.ArgumentParser()
ap.add_argument("capture", help="a run_show --record directory (td_capture.py)")
ap.add_argument("--camera", default="audience",
                help="room, audience (default), side, or 'ex ey ez tx ty tz [focal]' (robot frame)")
ap.add_argument("--start", type=float, default=0.0, help="from this time of the run (s)")
ap.add_argument("--end", type=float, default=0.0, help="to this time (s; 0: the end)")
ap.add_argument("--look", default="room", choices=("room", "plain"))
ap.add_argument("--guides", action="store_true", help="draw the safety guides")
ap.add_argument("--led0", choices=("minus", "plus"), default="minus", help="the strip's end LED 0 is at (flange y)")
ap.add_argument("--led-gain", type=float, default=1.0, help="brighten the drawn LEDs")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 1600, "height": 900, "renderer": "RaytracedLighting"})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402

import overlay  # noqa: E402
import td_capture  # noqa: E402
from canvas_viz import CanvasViz  # noqa: E402
from isaac_stage import (PHYSICS_DT, attach_tool, camera_spec, encode_video, import_robot, load_room,  # noqa: E402
                         render_settings, use_camera)
from led_viz import LedViz  # noqa: E402
from scan_viz import ScanViz  # noqa: E402

FPS = 30


def main():
    cap = td_capture.Reader(os.path.abspath(args.capture))
    if not cap.frames:
        raise SystemExit("%s: no frames recorded" % args.capture)
    cfg_path = os.path.join(ROOT, cap.meta["config"])
    cfg = json.load(open(cfg_path))
    end = min(args.end or cap.duration(), cap.duration())
    print("[replay] %s: %.1f s recorded, %d canvas images; rendering %.1f-%.1f s from %s"
          % (args.capture, cap.duration(), len(cap.canvas_t), args.start, end, args.camera))

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / FPS)
    stage = omni.usd.get_context().get_stage()
    env = load_room(stage, cfg_path, args.look, guides=args.guides)
    robot = world.scene.add(SingleArticulation(import_robot(), name="fr20"))
    attach_tool(stage)
    world.reset()
    names = list(robot.dof_names)
    idx = np.array([names.index("j%d" % i) for i in range(1, 7)])
    first = cap.frame_at(args.start)
    robot.set_joint_positions(np.radians(first["q"]), joint_indices=idx)
    scan = ScanViz(stage, cfg) if cfg.get("scan") else None       # the area's outline; TD draws the rest
    leds = LedViz(stage, led0=args.led0, gain=args.led_gain)
    canvas = CanvasViz(stage, cfg) if cfg.get("scan") and cap.canvas_t else None
    render_settings()
    use_camera(stage, "/World/ReplayCam", *camera_spec(args.camera, cfg, env))

    tag = args.camera if " " not in args.camera.strip() else "custom"
    frames = os.path.join(os.path.abspath(args.capture), "_frames_%s" % tag)
    os.makedirs(frames, exist_ok=True)
    for f in os.listdir(frames):
        os.remove(os.path.join(frames, f))
    steps = max(1, int(round(1.0 / (FPS * PHYSICS_DT))))
    for _ in range(int(1.0 / PHYSICS_DT)):                              # settle at the first pose
        robot.apply_action(ArticulationAction(joint_positions=np.radians(first["q"]), joint_indices=idx))
        world.step(render=False)
    corner, shown, shots = [], None, 0
    n = int((end - args.start) * FPS)
    for k in range(n):
        t = args.start + k / float(FPS)
        fr = cap.frame_at(t)
        for s in range(steps):                                            # the arm driven as it was, 120 Hz
            robot.apply_action(ArticulationAction(joint_positions=np.radians(fr["q"]), joint_indices=idx))
            world.step(render=s == steps - 1)
        leds.update(fr["leds"], fr["q"])
        if scan is not None:
            scan.update(-1.0, False, fr["q"])
        if canvas is not None:
            i = cap.canvas_index(t)
            if i is not None and i != shown:
                canvas.update(*cap.canvas(i))
                shown = i
        world.render()
        capture_viewport_to_file(get_active_viewport(), os.path.join(frames, "f_%05d.png" % shots))
        corner.append((shots / float(FPS), overlay.label(fr["state"], fr["clip"], fr["scan"])))
        shots += 1
        if k % (FPS * 10) == 0:
            print("[replay] %.0f / %.0f s" % (t, end), flush=True)
    mp4 = os.path.join(os.path.abspath(args.capture), "replay_%s.mp4" % tag)
    encode_video(app, frames, corner, shots, mp4, FPS, tag="replay")


main()
app.close()
