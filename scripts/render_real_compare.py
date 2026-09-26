"""A Houdini render of a joint CSV from roughly where a phone filmed the real
FR20, to put the two side by side.

    hython scripts/render_real_compare.py CLIP.csv OUT.mp4 [--frame N] [--res 1280 720]
    hython scripts/render_real_compare.py CLIP.csv OUT.mp4 --view curve [--frame N | --range A B] [--res 1600 900]

scenes/FR20_rig.hiplc as saved (robot_arm with its room, zones and goal
curve), Pose Source switched to Imported CSV with the clip -- so the arm
plays the file, not the scene's own solve. Not saved. OpenGL, the
viewport's look, no HUD. Frames at the scene's 24 fps, timed by the CSV's
time_s; ffmpeg makes 30 fps, holding the first and last frame 1 s each.
--frame N: one still of CSV frame N (to line the camera up).

CAM_POS / CAM_AIM (robot base frame, m, Z up): the phone's view of
IMG_0349.MOV (2026-09-25) -- behind the base, high, looking towards the TV
wall with the shelves on the right.

--view curve: square on to the plane the goal curve lies in (a fit), from
the robot's side, so the drawing reads; the curve drawn as a thick cyan
tube, the room as floor and solid furniture only (no lines over the curve).
--range A B: CSV frames A..B, no holds (a short loop).
"""

import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE).replace("\\", "/")
sys.path.insert(0, HERE)

CAM_POS = (1.7, -0.5, 2.2)
CAM_AIM = (-1.0, 0.95, 0.25)
CAM_FOCAL = 24.0                      # lined up by eye against IMG_0349.MOV, 2026-09-25
HOLD_S = 1.0
CURVE_RGB = (0.15, 0.85, 1.0)           # the viewport's cyan
CURVE_RADIUS_M = 0.008
CURVE_CAM = (3.2, 0.7, 0.35, 36.0, 0.9) # distance from the plane, lift, aim below the curve's centre, sideways (m), focal (mm)
OUT_FPS = 30


def _h(p):
    return (p[0], p[2], -p[1])        # robot frame (Z up) -> Houdini (Y up)


def look_at(pos, aim):
    """Houdini camera world transform (looks down -Z) at pos towards aim."""
    import hou
    p, a = hou.Vector3(*_h(pos)), hou.Vector3(*_h(aim))
    z = (p - a).normalized()
    x = hou.Vector3(0, 1, 0).cross(z).normalized()
    y = z.cross(x)
    return hou.Matrix4(((x[0], x[1], x[2], 0), (y[0], y[1], y[2], 0), (z[0], z[1], z[2], 0), (p[0], p[1], p[2], 1)))


def curve_camera(points):
    """(pos, aim, focal) square on to the plane of the goal curve (robot
    frame points), on the robot's side of it."""
    import numpy as np
    c = np.array(points)
    m = c.mean(0)
    n = np.linalg.svd(c - m)[2][2]
    if np.dot(n, -m) < 0:                   # towards the base
        n = -n
    side_axis = np.cross((0.0, 0.0, 1.0), n)
    side_axis /= np.linalg.norm(side_axis)
    if np.dot(side_axis, -m) > 0:           # sideways away from the base, so the arm stands to one side
        side_axis = -side_axis
    dist, lift, drop, focal, side = CURVE_CAM
    if os.environ.get("CURVE_CAM"):
        dist, lift, drop, focal, side = [float(x) for x in os.environ["CURVE_CAM"].split()]
    pos = m + n * dist + np.array((0.0, 0.0, lift)) + side_axis * side
    aim = m - np.array((0.0, 0.0, drop))
    return tuple(float(x) for x in pos), tuple(float(x) for x in aim), focal


def _curve_tube(hou):
    """The goal curve as a thick, unlit cyan tube (a line is a hair at 1600 px)."""
    geo = hou.node("/obj").createNode("geo", "compare_curve", run_init_scripts=False)
    om = geo.createNode("object_merge", "curve")
    om.parm("objpath1").set("/obj/fr20/CURVE_IN")
    tube = geo.createNode("polywire", "tube")
    tube.setInput(0, om)
    tube.parm("radius").set(CURVE_RADIUS_M)
    tube.parm("div").set(8)
    col = geo.createNode("attribwrangle", "cyan")
    col.setInput(0, tube)
    col.parm("class").set(2)                              # points
    col.parm("snippet").set("v@Cd = set(%g, %g, %g);\nsetdetailattrib(0, 'gl_lit', 0);" % CURVE_RGB)
    col.setDisplayFlag(True)
    col.setRenderFlag(True)


def render(csv, frames_dir, frame=None, res=(1280, 720), view="phone", span=None):
    import hou
    import fairino_player as P
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    hou.hipFile.load(ROOT + "/scenes/FR20_rig.hiplc", suppress_save_prompt=True, ignore_load_warnings=True)
    arm = hou.node("/obj/fr20/robot_arm")
    arm.parm("pose_source").set("2")                        # Imported CSV
    arm.parm("import_csv").set(csv.replace("\\", "/"))
    arm.parm("import_start").set(1)
    # the arm and the goal curve (the room is drawn below); targets and
    # analysis off. The curve is the scene's own: check it is this clip's
    # (curve_gap_mm) before trusting the picture
    for p in arm.parms():
        if p.name().startswith("show_") and p.name() not in ("show_robot", "show_cell", "show_curve"):
            p.set(0)
    # the room as robot_arm draws it, but see-through faces left out: the
    # phone stood in the operator's zone, and from inside it the zone's
    # faces tint everything. Outlines and solid objects stay (the curve
    # view: solid objects only, nothing drawn over the curve)
    if arm.parm("show_cell") is not None and arm.evalParm("show_cell"):
        arm.parm("show_cell").set(0)
        env = hou.node("/obj").createNode("geo", "compare_room", run_init_scripts=False)
        sop = env.createNode("python", "room")
        sop.parm("python").set("import sys\nsys.path.insert(0, %r)\nimport cell_sop, collision\n"
                               "cell_sop.env_geometry(hou.pwd().geometry(), collision.load_env(%r))\n"
                               % (ROOT + "/scripts", ROOT + "/envs/volvox_lab.json"))
        cut = env.createNode("attribwrangle", "outlines")
        cut.setInput(0, sop)
        cut.parm("class").set(1)
        keep = "f@Alpha >= 0.99 && closed" if view == "curve" else "f@Alpha >= 0.99"
        cut.parm("snippet").set('int closed = primintrinsic(0, "closed", @primnum);\n'
                                'if (!(%s)) removeprim(0, @primnum, 1);' % keep)
        cut.setDisplayFlag(True)
        cut.setRenderFlag(True)
    # out_viz may merge the scene's own input curves: show the asset alone
    out_viz = hou.node("/obj/fr20/out_viz")
    if out_viz is not None and out_viz.isDisplayFlagSet():
        arm.setDisplayFlag(True)
        arm.setRenderFlag(True)
    ci = hou.node("/obj/fr20/CURVE_IN")
    if ci is not None and arm.evalParm("show_curve"):
        gap = curve_gap_mm([tuple(p.position()) for p in ci.geometry().points()], csv)
        print("goal curve vs the clip's TCP path: %.1f mm at most" % gap)
        if gap > 20.0:
            raise SystemExit("the scene's goal curve is not this clip's (%.0f mm off): save the scene that made it" % gap)
    t, _ = P.load_csv(csv)
    last = 1 + int(round(t[-1] * hou.fps()))
    cam = hou.node("/obj").createNode("cam", "compare_cam")
    pos, aim, focal = CAM_POS, CAM_AIM, CAM_FOCAL
    if view == "curve":
        arm.parm("show_curve").set(0)
        _curve_tube(hou)
        pos, aim, focal = curve_camera([(x, -z, y) for x, y, z in (tuple(p.position()) for p in ci.geometry().points())])
    if os.environ.get("COMPARE_CAM"):                       # "x y z ax ay az focal", to line it up
        v = [float(x) for x in os.environ["COMPARE_CAM"].split()]
        pos, aim, focal = v[0:3], v[3:6], v[6]
    cam.setWorldTransform(look_at(pos, aim))
    cam.parm("focal").set(focal)
    cam.parm("resx").set(res[0])
    cam.parm("resy").set(res[1])
    import render_clip_review
    render_clip_review.viewport_look(cam)
    rop = hou.node("/out").createNode("opengl")
    rop.parm("camera").set(cam.path())
    rop.parm("tres").set(1)
    rop.parm("res1").set(res[0])
    rop.parm("res2").set(res[1])
    rop.parm("picture").set(frames_dir.replace("\\", "/") + "/f_$F4.png")
    rop.parm("trange").set(1)
    rop.parmTuple("f").deleteAllKeyframes()
    rop.parmTuple("f").set((frame, frame, 1) if frame else (span[0], span[1], 1) if span else (1, last, 1))
    for n, v in (("aamode", 3), ("hqlighting", 1), ("shadows", 0), ("usehdr", 1)):
        if rop.parm(n) is not None:
            rop.parm(n).set(v)
    rop.render()
    return last


def curve_gap_mm(curve_houdini, csv):
    """Largest distance (mm) from the clip's TCP path (URDF FK) to the goal
    curve (Houdini points): a few mm when the scene holds this clip's curve."""
    import math
    import fairino_player as P
    import motion_clip as M
    curve = [(x, -z, y) for x, y, z in curve_houdini]
    _, q = P.load_csv(csv)

    def seg(p, a, b):
        ab = [b[i] - a[i] for i in range(3)]
        L = sum(x * x for x in ab)
        u = max(0.0, min(1.0, sum(x * (p[i] - a[i]) for i, x in enumerate(ab)) / L)) if L > 0 else 0.0
        return math.dist(p, [a[i] + u * ab[i] for i in range(3)])
    return 1000.0 * max(min(seg(p, a, b) for a, b in zip(curve, curve[1:])) for p in M._tcp_path("fr20", q))


def encode(frames_dir, out, fps_in=24.0, hold=HOLD_S):
    frames = sorted(glob.glob(frames_dir + "/f_*.png"))
    start = int(os.path.basename(frames[0])[2:6])
    vf = "fps=%d" % OUT_FPS
    if hold:
        vf += ",tpad=start_mode=clone:start_duration=%g:stop_mode=clone:stop_duration=%g" % (hold, hold)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "%g" % fps_in, "-start_number", str(start),
                    "-i", frames_dir + "/f_%04d.png", "-vf", vf, "-an", "-pix_fmt", "yuv420p", "-c:v", "libx264",
                    "-profile:v", "high", "-level", "4.0", "-crf", "18", out], check=True)
    return out


if __name__ == "__main__":
    args = sys.argv[1:]
    csv, out = os.path.abspath(args[0]), os.path.abspath(args[1])
    frame = int(args[args.index("--frame") + 1]) if "--frame" in args else None
    res = tuple(int(x) for x in args[args.index("--res") + 1:args.index("--res") + 3]) if "--res" in args else (1280, 720)
    view = args[args.index("--view") + 1] if "--view" in args else "phone"
    span = tuple(int(x) for x in args[args.index("--range") + 1:args.index("--range") + 3]) if "--range" in args else None
    frames_dir = os.path.splitext(out)[0] + "_frames"
    os.makedirs(frames_dir, exist_ok=True)
    if not frame:
        for f in glob.glob(frames_dir + "/f_*.png"):
            os.remove(f)
    n = render(csv, frames_dir, frame, res, view, span)
    if frame:
        print("still", frames_dir + "/f_%04d.png" % frame)
    else:
        print("wrote", encode(frames_dir, out, hold=0 if span else HOLD_S), "frames %d-%d" % (span or (1, n)))
