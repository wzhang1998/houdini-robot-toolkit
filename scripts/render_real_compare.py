"""A Houdini render of a joint CSV from roughly where a phone filmed the real
FR20, to put the two side by side.

    hython scripts/render_real_compare.py CLIP.csv OUT.mp4 [--frame N] [--res 1280 720]

scenes/FR20_rig.hiplc as saved (robot_arm with its room, zones and goal
curve), Pose Source switched to Imported CSV with the clip -- so the arm
plays the file, not the scene's own solve. Not saved. OpenGL, the
viewport's look, no HUD. Frames at the scene's 24 fps, timed by the CSV's
time_s; ffmpeg makes 30 fps, holding the first and last frame 1 s each.
--frame N: one still of CSV frame N (to line the camera up).

CAM_POS / CAM_AIM (robot base frame, m, Z up): the phone's view of
IMG_0349.MOV (2026-09-25) -- behind the base, high, looking towards the TV
wall with the shelves on the right.
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


def render(csv, frames_dir, frame=None, res=(1280, 720)):
    import hou
    import fairino_player as P
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    hou.hipFile.load(ROOT + "/scenes/FR20_rig.hiplc", suppress_save_prompt=True, ignore_load_warnings=True)
    arm = hou.node("/obj/fr20/robot_arm")
    arm.parm("pose_source").set("2")                        # Imported CSV
    arm.parm("import_csv").set(csv.replace("\\", "/"))
    arm.parm("import_start").set(1)
    # the saved scene's goal curve, targets and analysis belong to its own
    # solve, not to the imported clip: next to the real run they would
    # mislead. The arm only (the room is drawn below).
    for p in arm.parms():
        if p.name().startswith("show_") and p.name() not in ("show_robot", "show_cell"):
            p.set(0)
    # the room as robot_arm draws it, but see-through faces left out: the
    # phone stood in the operator's zone, and from inside it the zone's
    # faces tint everything. Outlines and solid objects stay.
    if arm.parm("show_cell") is not None and arm.evalParm("show_cell"):
        arm.parm("show_cell").set(0)
        env = hou.node("/obj").createNode("geo", "compare_room", run_init_scripts=False)
        sop = env.createNode("python", "room")
        sop.parm("python").set("import sys\nsys.path.insert(0, %r)\nimport cell_sop, collision\n"
                               "cell_sop.env_geometry(hou.pwd().geometry(), collision.load_env(%r))\n"
                               % (ROOT + "/scripts", ROOT + "/envs/volvox_lab.json"))
        cut = env.createNode("blast", "outlines")
        cut.setInput(0, sop)
        cut.parm("group").set("@Alpha<0.99")
        cut.parm("grouptype").set(4)
        cut.setDisplayFlag(True)
        cut.setRenderFlag(True)
    # out_viz may merge the scene's own input curves: show the asset alone
    out_viz = hou.node("/obj/fr20/out_viz")
    if out_viz is not None and out_viz.isDisplayFlagSet():
        arm.setDisplayFlag(True)
        arm.setRenderFlag(True)
    t, _ = P.load_csv(csv)
    last = 1 + int(round(t[-1] * hou.fps()))
    cam = hou.node("/obj").createNode("cam", "compare_cam")
    pos, aim, focal = CAM_POS, CAM_AIM, CAM_FOCAL
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
    rop.parmTuple("f").set((frame, frame, 1) if frame else (1, last, 1))
    for n, v in (("aamode", 3), ("hqlighting", 1), ("shadows", 0), ("usehdr", 1)):
        if rop.parm(n) is not None:
            rop.parm(n).set(v)
    rop.render()
    return last


def encode(frames_dir, out, fps_in=24.0):
    frames = sorted(glob.glob(frames_dir + "/f_*.png"))
    start = int(os.path.basename(frames[0])[2:6])
    vf = "fps=%d,tpad=start_mode=clone:start_duration=%g:stop_mode=clone:stop_duration=%g" % (OUT_FPS, HOLD_S, HOLD_S)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "%g" % fps_in, "-start_number", str(start),
                    "-i", frames_dir + "/f_%04d.png", "-vf", vf, "-an", "-pix_fmt", "yuv420p", "-c:v", "libx264",
                    "-crf", "18", out], check=True)
    return out


if __name__ == "__main__":
    args = sys.argv[1:]
    csv, out = os.path.abspath(args[0]), os.path.abspath(args[1])
    frame = int(args[args.index("--frame") + 1]) if "--frame" in args else None
    res = tuple(int(x) for x in args[args.index("--res") + 1:args.index("--res") + 3]) if "--res" in args else (1280, 720)
    frames_dir = os.path.splitext(out)[0] + "_frames"
    os.makedirs(frames_dir, exist_ok=True)
    if not frame:
        for f in glob.glob(frames_dir + "/f_*.png"):
            os.remove(f)
    n = render(csv, frames_dir, frame, res)
    if frame:
        print("still", frames_dir + "/f_%04d.png" % frame)
    else:
        print("wrote", encode(frames_dir, out), "frames 1-%d" % n)
