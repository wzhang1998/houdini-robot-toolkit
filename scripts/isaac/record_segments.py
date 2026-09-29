"""Built segments played back to back in Isaac Sim, physics on, recorded:
to look at a new scan (show.py build --scan-only) before the library is
made again.

    uv run scripts/show.py build shows/party_vscan.json --scan-only
    C:/isaacsim6/python.bat scripts/isaac/record_segments.py shows/party_vscan.json --headless
    C:/isaacsim6/python.bat scripts/isaac/record_segments.py shows/party.json --graph shows/party.compiled.json

The room is the show's layer (shows/<show>.usda: write it with room_usd.py
--show). Drawn over it: the scan's area outlined on the paper, the strip
violet while its LEDs are on (the scan's labels.led_on_s) and the paper it
has exposed so far. One pass per camera (--cameras: audience, from behind
the audience zone at eye height; side, from inside the room along the
paper), then the passes side by side in <out>/<show>_segments.mp4, and a
JSON summary: tracking (commanded vs simulated), contacts with the room.
"""

import argparse
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
ap.add_argument("config")
ap.add_argument("--graph", default="", help="a compiled show (default: geo/show/<show>_scan/compiled.json)")
ap.add_argument("--segments", default="to_scan,scan,from_scan", help="comma separated, played in this order")
ap.add_argument("--cameras", default="audience,side")
ap.add_argument("--out", default=os.path.join(ROOT, "geo", "isaac"))
ap.add_argument("--headless", action="store_true")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720, "renderer": "RaytracedLighting"})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux  # noqa: E402

import show  # noqa: E402
from isaac_stage import PHYSICS_DT, attach_tool, import_robot  # noqa: E402

FPS_VIDEO = 30
HOLD_S = 0.5                     # still between the segments, as the show's hubs
VIOLET = (0.55, 0.2, 1.0)


def cameras(cfg):
    """{name: (eye, target, focal mm)}: what a guest sees, and the gap from the side."""
    a = cfg["zones"]["audience"]
    middle, w, h = show.scan_area(cfg) if cfg.get("scan") else (cfg["canvas"]["center"], 1.0, 1.0)
    n = cfg["canvas"]["normal"]
    right = (n[1], -n[0], 0.0)
    eye_side = [middle[0] + right[0] * 1.5 - n[0] * 1.4, middle[1] + right[1] * 1.5 - n[1] * 1.4, 1.8]
    return {"audience": ([a["center"][0], a["center"][1], 1.6], middle, 18.0),
            "side": (eye_side, [middle[0] - n[0] * 0.4, middle[1] - n[1] * 0.4, middle[2]], 14.0)}


def curve(stage, path, rgb, width, closed=False):
    c = UsdGeom.BasisCurves.Define(stage, Sdf.Path(path))
    c.CreateTypeAttr("linear")
    c.CreateWrapAttr("periodic" if closed else "nonperiodic")
    c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    c.CreateWidthsAttr([width])
    c.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    return c


def set_points(c, pts):
    c.GetPointsAttr().Set([Gf.Vec3f(*p) for p in pts])
    c.GetCurveVertexCountsAttr().Set([len(pts)])


def area_corners(cfg, lift):
    """The scan area's corners on the paper's face, lift m in front of it."""
    middle, w, h = show.scan_area(cfg)
    n = cfg["canvas"]["normal"]
    left = (-n[1], n[0], 0.0)
    off = cfg["canvas"].get("thickness", 0.02) / 2.0 + lift
    base = [middle[i] - n[i] * off for i in range(3)]
    return [[base[0] + left[0] * sx * w / 2, base[1] + left[1] * sx * w / 2, base[2] + sz * h / 2]
            for sx, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1))]


def exposed_quad(cfg, u):
    """The paper exposed at u (0..1 of the scan): the area's part the strip has passed."""
    tl, tr, br, bl = area_corners(cfg, 0.004)
    u = min(max(u, 0.0), 1.0)
    d = cfg["scan"].get("direction", "left_to_right")
    lerp = lambda a, b: [a[i] + (b[i] - a[i]) * u for i in range(3)]  # noqa: E731
    if d == "top_to_bottom":
        return [tl, tr, lerp(tr, br), lerp(tl, bl)]
    if d == "bottom_to_top":
        return [lerp(bl, tl), lerp(br, tr), br, bl]
    if d == "left_to_right":               # left as seen from the robot's side: tl is on its left
        return [tl, lerp(tl, tr), lerp(bl, br), bl]
    return [lerp(tr, tl), tr, br, lerp(br, bl)]


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
    room_usd = os.path.splitext(cfg_path)[0] + ".usda"
    if not os.path.exists(room_usd):
        raise SystemExit("no %s: python scripts/room_usd.py --show %s" % (room_usd, args.config))

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
    add_reference_to_stage(room_usd, "/World/Room")
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(600)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Key"))
    key.CreateIntensityAttr(2500)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 30.0))
    prim_path = import_robot()
    attach_tool(stage)
    robot = world.scene.add(SingleArticulation(prim_path, name="fr20"))
    scanning = cfg.get("scan") and "scan" in by_name
    if scanning:
        outline = curve(stage, "/World/ScanArea", (0.35, 0.35, 0.4), 0.006, closed=True)
        set_points(outline, area_corners(cfg, 0.003))
        exposed = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/Exposed"))
        exposed.CreateFaceVertexCountsAttr([4])
        exposed.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        exposed.CreateDoubleSidedAttr(True)
        exposed.CreateDisplayColorAttr([Gf.Vec3f(*VIOLET)])
        exposed.CreatePointsAttr([Gf.Vec3f(0, 0, -5)] * 4)          # out of sight until the scan starts
        leds = curve(stage, "/World/LEDs", VIOLET, 0.03)
        scan_seg = by_name["scan"]
        on0, on1 = scan_seg.labels["led_on_s"]
        u_of = scan_seg.labels["u"]
    import gestures as G
    rig = G.Rig()
    import collision as C
    import robot_profile as RP
    strip = C.strip_box(C.tool_def(RP.load("fr20")))
    half = (strip["size"][1] if strip else 1.0) / 2.0
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
                    a, b = str(h.actor0), str(h.actor1)
                    if ("/Room" in a) != ("/Room" in b):
                        contacts.append((round(clock[0], 3), a, b))
        sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)  # noqa: F841
    except Exception as e:
        print("[record] contact reports unavailable: %s" % e)

    import carb.settings
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    st = carb.settings.get_settings()
    st.set("/rtx/rendermode", "RaytracedLighting")
    st.set("/rtx/hydra/faceCulling/enabled", True)             # the walls culled from outside: seen through
    cams = cameras(cfg)
    os.makedirs(args.out, exist_ok=True)
    summary = {"config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"),
               "graph": os.path.relpath(gpath, ROOT).replace("\\", "/"),
               "segments": [[s.name, round(s.duration, 2)] for s in segs], "passes": {}}
    videos = []
    for cam_name in args.cameras.split(","):
        eye, look, focal = cams[cam_name]
        path = "/World/Cam_" + cam_name
        cam = UsdGeom.Camera.Define(stage, Sdf.Path(path))
        cam.CreateFocalLengthAttr(focal)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
        set_camera_view(eye=eye, target=look, camera_prim_path=path)
        vp = get_active_viewport()
        vp.camera_path = path
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
        for i in range(int(total / PHYSICS_DT)):
            t = i * PHYSICS_DT
            clock[0] = t
            seg, s = at(t)
            q = seg.at(s)
            robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=idx))
            shoot = i % every == 0
            if scanning and shoot:
                done = [p[0] for p in plan if p[1].name == "scan"][0]
                if t < done:
                    u = -1.0
                elif seg.name == "scan":
                    u = u_of[min(len(u_of) - 1, int(s / scan_seg.duration * (len(u_of) - 1)))]
                else:
                    u = 2.0
                exposed.GetPointsAttr().Set([Gf.Vec3f(*p) for p in exposed_quad(cfg, u)] if u > 0 else
                                            [Gf.Vec3f(0, 0, -5)] * 4)
                on = seg.name == "scan" and on0 <= s <= on1
                lit += on
                R, tcp, _ = rig.tool(q)
                ax = (R[0][1], R[1][1], R[2][1])                  # the strip along the flange's y
                set_points(leds, [[tcp[j] + ax[j] * k * half for j in range(3)] for k in (-1, 1)] if on else
                           [[0, 0, -5], [0, 0, -5.01]])
            world.step(render=shoot or not args.headless)
            if shoot:
                capture_viewport_to_file(vp, os.path.join(frames, "f_%05d.png" % f))
                f += 1
            sim = np.degrees(robot.get_joint_positions(joint_indices=idx))
            if t > 0.5:
                for j in range(6):
                    worst[j] = max(worst[j], abs(float(sim[j]) - q[j]))
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
