"""Built segments played back to back in Isaac Sim, physics on, recorded:
to look at a new scan (show.py build --scan-only) before the library is
made again.

    uv run scripts/show.py build shows/party_vscan.json --scan-only
    C:/isaacsim6/python.bat scripts/isaac/record_segments.py shows/party_vscan.json --headless
    C:/isaacsim6/python.bat scripts/isaac/record_segments.py shows/party.json --graph shows/party.compiled.json

The room is the show's layer (shows/<show>.usda: write it with room_usd.py
--show), lit and coloured after the lab's photo (room_look.py; --look plain
for the flat one). Drawn over it: the scan's area outlined on the paper, the strip
violet while its LEDs are on (the scan's labels.led_on_s) and the paper it
has exposed so far. One pass per camera (--cameras: room, the whole
room from the corner behind the robot, as the photo; audience, the
guests' side, from well behind the audience zone through the glass; side,
along the paper), then the passes side by side in <out>/<show>_segments.mp4, and a
JSON summary: tracking (commanded vs simulated), contacts with the room.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)

ap = argparse.ArgumentParser()
ap.add_argument("config")
ap.add_argument("--graph", default="", help="a compiled show (default: geo/show/<show>_scan/compiled.json)")
ap.add_argument("--segments", default="to_scan,scan,from_scan", help="comma separated, played in this order")
ap.add_argument("--cameras", default="room,audience", help="room, audience, side")
ap.add_argument("--look", default="room", choices=("room", "plain"),
                help="room: lights and surfaces after the lab's photo (room_look.py); plain: the flat grey room")
ap.add_argument("--still", type=float, default=-1.0, help="only a PNG per camera at this time (s), to tune the look")
ap.add_argument("--out", default=os.path.join(ROOT, "geo", "isaac"))
ap.add_argument("--headless", action="store_true")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720, "renderer": "RaytracedLighting"})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Gf, Sdf, UsdGeom  # noqa: E402

import show  # noqa: E402
from isaac_stage import (PHYSICS_DT, attach_tool, cameras, contact_paths, import_robot, load_room,  # noqa: E402
                         render_settings, use_camera)

FPS_VIDEO = 30
HOLD_S = 0.5                     # still between the segments, as the show's hubs


def main():
    cfg_path = os.path.abspath(args.config)
    cfg = json.load(open(cfg_path))
    name = os.path.splitext(os.path.basename(cfg_path))[0]
    gpath = args.graph or os.path.join(ROOT, "geo", "show", name + "_scan", "compiled.json")
    graph = show.Graph.load(gpath)
    by_name = {s.name: s for s in graph.segments}
    segs = [by_name[s] for s in args.segments.split(",") if s in by_name]
    missing = [s for s in args.segments.split(",") if s not in by_name]
    if missing:
        raise SystemExit("not in %s: %s (there: %s)" % (gpath, ", ".join(missing), ", ".join(by_name)))
    for a, b in zip(segs, segs[1:]):
        jump = max(abs(x - y) for x, y in zip(a.q[-1], b.q[0]))
        if jump > 0.01:
            raise SystemExit("%s ends %.2f deg from where %s starts" % (a.name, jump, b.name))

    # one timeline: (t0, segment) with a hold after each
    plan, t = [], 0.0
    for s in segs:
        plan.append((t, s))
        t += s.duration + HOLD_S
    total = t

    def at(t):
        for t0, s in reversed(plan):
            if t >= t0:
                return s, t - t0
        return plan[0][1], 0.0

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    env = load_room(stage, cfg_path, args.look)
    prim_path = import_robot()
    attach_tool(stage)
    robot = world.scene.add(SingleArticulation(prim_path, name="fr20"))
    scanning = cfg.get("scan") and "scan" in by_name
    if scanning:
        from scan_viz import ScanViz
        viz = ScanViz(stage, cfg)
        scan_seg = by_name["scan"]
        on0, on1 = scan_seg.labels["led_on_s"]
        u_of = scan_seg.labels["u"]
    world.reset()
    dof = list(robot.dof_names)
    idx = np.array([dof.index("j%d" % i) for i in range(1, 7)])

    contacts, clock = [], [0.0]
    try:
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType

        def on_contact(headers, data):
            for h in headers:
                if h.type == ContactEventType.CONTACT_FOUND:
                    a, b = contact_paths(h)
                    if ("/Room" in a) != ("/Room" in b):
                        contacts.append((round(clock[0], 3), a, b))
        sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)  # noqa: F841
    except Exception as e:
        print("[record] contact reports unavailable: %s" % e)

    from omni.kit.viewport.utility import capture_viewport_to_file
    render_settings()
    cams = cameras(cfg, env)
    os.makedirs(args.out, exist_ok=True)
    summary = {"config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"),
               "graph": os.path.relpath(gpath, ROOT).replace("\\", "/"),
               "segments": [[s.name, round(s.duration, 2)] for s in segs], "passes": {}}
    videos = []
    for cam_name in args.cameras.split(","):
        eye, look, focal = cams[cam_name]
        vp = use_camera(stage, "/World/Cam_" + cam_name, eye, look, focal)
        q0 = segs[0].q[0]
        robot.set_joint_positions(np.radians(q0), joint_indices=idx)
        robot.set_joint_velocities(np.zeros(6), joint_indices=idx)
        for _ in range(60):                                     # settle, and the camera's first frames drawn
            robot.apply_action(ArticulationAction(joint_positions=np.radians(q0), joint_indices=idx))
            world.step(render=True)
        del contacts[:]
        frames = os.path.join(args.out, "_frames_" + cam_name)
        shutil.rmtree(frames, ignore_errors=True)
        os.makedirs(frames)
        worst, f, lit = [0.0] * 6, 0, 0
        every = int(round(1.0 / (FPS_VIDEO * PHYSICS_DT)))
        still_i = int(args.still / PHYSICS_DT) if args.still >= 0 else -1
        for i in range(int(total / PHYSICS_DT)):
            t = i * PHYSICS_DT
            clock[0] = t
            seg, s = at(t)
            q = seg.at(s)
            robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=idx))
            shoot = i == still_i if still_i >= 0 else i % every == 0
            if scanning and shoot:
                done = [p[0] for p in plan if p[1].name == "scan"][0]
                if t < done:
                    u = -1.0
                elif seg.name == "scan":
                    u = u_of[min(len(u_of) - 1, int(s / scan_seg.duration * (len(u_of) - 1)))]
                else:
                    u = 2.0
                viz.update(u, seg.name == "scan" and on0 <= s <= on1, q)
                lit += seg.name == "scan" and on0 <= s <= on1
            world.step(render=shoot or not args.headless)
            if shoot and still_i >= 0:                          # one converged picture, then the next camera
                for _ in range(40):
                    world.step(render=True)
                png = os.path.join(args.out, "%s_%s_still.png" % (name, cam_name))
                capture_viewport_to_file(vp, png)
                for _ in range(20):
                    app.update()
                print("[record] still %s" % png, flush=True)
                break
            if shoot:
                capture_viewport_to_file(vp, os.path.join(frames, "f_%05d.png" % f))
                f += 1
            sim = np.degrees(robot.get_joint_positions(joint_indices=idx))
            if t > 0.5:
                for j in range(6):
                    worst[j] = max(worst[j], abs(float(sim[j]) - q[j]))
        if still_i >= 0:
            shutil.rmtree(frames, ignore_errors=True)
            continue
        for _ in range(60):
            app.update()                                        # the last captures written
        mp4 = os.path.join(args.out, "%s_%s.mp4" % (name, cam_name))
        r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS_VIDEO), "-i",
                            os.path.join(frames, "f_%05d.png"), "-vf", "scale=1280:-2", "-pix_fmt", "yuv420p",
                            "-c:v", "libx264", "-crf", "22", mp4], capture_output=True, text=True)
        if r.returncode:
            raise SystemExit("ffmpeg: %s" % r.stderr[-400:])
        shutil.rmtree(frames, ignore_errors=True)
        videos.append(mp4)
        summary["passes"][cam_name] = {"video": os.path.relpath(mp4, ROOT).replace("\\", "/"),
                                       "tracking_max_deg": [round(x, 3) for x in worst],
                                       "arm_room_contacts": len(contacts), "first_contacts": contacts[:5],
                                       "frames_leds_on": lit}
        print("[record] %s: %s, contacts %d, tracking max %.3f deg" % (cam_name, mp4, len(contacts), max(worst)),
              flush=True)
    if len(videos) > 1:
        both = os.path.join(args.out, "%s_segments.mp4" % name)
        inputs = sum((["-i", v] for v in videos), [])
        r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + inputs +
                           ["-filter_complex", "hstack=inputs=%d" % len(videos), "-pix_fmt", "yuv420p",
                            "-c:v", "libx264", "-crf", "22", both], capture_output=True, text=True)
        summary["video"] = os.path.relpath(both, ROOT).replace("\\", "/") if r.returncode == 0 else r.stderr[-300:]
    json.dump(summary, open(os.path.join(args.out, "%s_segments.json" % name), "w"), indent=1)
    print("[record] %s" % json.dumps(summary))


main()
app.close()
