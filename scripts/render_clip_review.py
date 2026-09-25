"""Review videos of the clip library: every clip rendered in the room, its
metadata burnt in, tiled into grid videos, plus a contact sheet.

The PDG network (the usual way: work items per clip -> ROP OpenGL -> ffmpeg
/ ImageMagick) is scenes/FR20_review.hiplc, built by
scripts/build_review_scene.py; this module holds what its steps run.

    hython scripts/build_review_scene.py [--cook] [--ids ...]    # the PDG route
    python scripts/render_clip_review.py [--ids ...]              # the same without PDG, one clip at a time
    python scripts/render_clip_review.py --self-test

Per clip:

    frames   OpenGL (the viewport's renderer) in scenes/FR20_review.hiplc:
             the measured room (see-through objects by their outlines), the
             FR20 in white playing the clip, its TCP path as a tube coloured
             bar by bar by the INTENDED Laban action; the user's viewport
             camera, its headlight, its grey background
    tile     ffmpeg: the frames + text -- id, duration, verdict; intended vs
             measured action per bar; peak acceleration as % of each joint's
             limit, peak velocity %, room clearance, the player's time scale;
             while it plays, the current bar (intended -> measured). A
             rejected clip is framed red; one rejected before any motion
             (unreachable) is a still: the arm at HOME, the path it was
             asked for, red where the arm cannot hold the tool direction
    poster   one labelled frame of the tile (40 % in), for the contact sheet

All clips: pages (ffmpeg xstack, --grid per page), overview pages (every clip, 5 x 5
per video) and a contact sheet (ImageMagick montage of the posters).
Output under geo/review/ (not in git).
"""

import argparse
import glob
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE).replace("\\", "/")
OUT = ROOT + "/geo/review"
HYTHON = "C:/Program Files/Side Effects Software/Houdini 22.0.368/bin/hython.exe"
FFMPEG = "ffmpeg"
FONT = "C\\:/Windows/Fonts/consola.ttf"
SETS = {"dance": ROOT + "/geo/dance", "clips": ROOT + "/geo/clips", "stage": ROOT + "/geo/stage"}
FPS = 24.0
TILE = (480, 480)
# the user's viewport in FR20_rig, 2026-09-25 (the robot's right-front, above, a
# 50 mm lens): the stage in front, the cart and operator to the left, the TV
# wall to the right. World transform rows (Houdini: translate in the last row)
CAM_XFORM = ((-0.757216, -0.000002, -0.653166, 0.0), (-0.124517, 0.981661, 0.144349, 0.0),
             (0.641188, 0.190633, -0.743329, 0.0), (4.841888, 2.383749, -6.454002, 1.0))
CAM_FOCAL, CAM_APERTURE = 50.0, 41.4214
FRAME_ON = (-0.35, 1.05, -0.45)          # Houdini frame: robot (-0.35, 0.45, 1.05), where the arm works
WINDOW = 0.62                            # the fraction of the viewport's width a square tile shows
ROBOT_RGB = (0.93, 0.93, 0.92)
# the viewport's lighting: no scene lights, High Quality, the headlight at 0.8
# with specular, direction HEADLIGHT_DIR in camera space; a lit OpenGL render
# reads darker than the viewport's headlight at the same number (HEAD_GAIN)
HEADLIGHT, HEADLIGHT_DIR, HEAD_GAIN, AMBIENT = 0.8, (-30.0, -30.0, -100.0), 2.2, 0.3
BG_BOTTOM, BG_TOP = (0.20, 0.20, 0.21), (0.30, 0.30, 0.31)     # the DarkGrey colour scheme
ACTION_RGB = {"punch": (0.95, 0.25, 0.2), "slash": (1.0, 0.6, 0.15), "press": (0.65, 0.2, 0.15),
              "wring": (0.7, 0.35, 0.9), "dab": (1.0, 0.9, 0.2), "flick": (0.25, 0.9, 0.95),
              "glide": (0.3, 0.85, 0.35), "float": (0.55, 0.7, 1.0)}
# how much of the room a review shows: which cell_sop prims are kept (VEX,
# per primitive; `closed` is 0 for an outline)
ROOMS = {
    "full": 'f@Alpha >= 0.99',                                           # every outline, solid furniture, floor
    "floor": 's@name == "floor" || s@name == "base_plate"',             # the floor and the base plate only
    "solid": 's@role == "obstacle" && f@Alpha >= 0.99 && closed',       # + solid furniture, no lines
    "solid_work": '(s@role == "obstacle" && f@Alpha >= 0.99 && closed) || (s@name == "stage" && !closed)',
}
ROOM = "solid_work"                                         # chosen by the user, 2026-09-25


# --------------------------------------------------------------------------
# clips and their text (pure)
# --------------------------------------------------------------------------

def nframes(clip):
    """Frames at 24 fps: frame f is t = (f - 1) / 24 (cell_sop.q_at)."""
    pts = clip.get("points") or []
    return 1 + int(math.ceil(pts[-1]["t"] * FPS)) if pts else 0


def clip_items(sets=("dance", "clips"), ids=None, ok_only=False):
    """One entry per clip, in manifest order: what a work item carries."""
    out = []
    for s in sets:
        man = json.load(open(SETS[s] + "/manifest.json"))
        for e in man["clips"]:
            if (ids and e["id"] not in ids) or (ok_only and not e.get("ok")):
                continue
            path = (SETS[s] + "/" + e["file"]).replace("\\", "/")
            c = json.load(open(path))
            out.append({"id": e["id"], "set": s, "clip": path, "ok": int(bool(e.get("ok"))),
                        "nframes": nframes(c), "dir": "%s/%s/%s" % (OUT, s, e["id"])})
    return out


def _hex(rgb):
    return "0x%02x%02x%02x" % tuple(int(255 * max(0.0, min(1.0, c))) for c in rgb)


def bars(clip):
    """[(t0, t1, intended, measured)] -- dance clips; [] for primitives."""
    return [(b["t0"], b["t1"], b.get("intent"), b.get("action")) for b in (clip.get("labels") or {}).get("bars") or []]


def header_lines(clip):
    """The static text of a tile: [(text, colour)], each fitting 480 px."""
    s, lab = clip.get("safety", {}), clip.get("labels") or {}
    ok = s.get("ok")
    dur = clip.get("meta", {}).get("duration_s", 0.0)
    lines = [("%s   %.1f s   %s" % (clip["id"], dur, "OK" if ok else "REJECTED"), "white" if ok else "0xff5050")]
    b = bars(clip)
    if b:
        agree = sum(i == m for _, _, i, m in b)
        lines.append(("intent " + "-".join(i or "?" for _, _, i, _ in b), "white"))
        lines.append(("meas.  " + "-".join(m or "?" for _, _, _, m in b) + "  %d/%d" % (agree, len(b)),
                      "0x80ff80" if agree == len(b) else "0xffd060"))
    else:
        prim = (clip.get("style") or {}).get("primitive") or (clip.get("style") or {}).get("generator", "")
        lines.append(("%s   measured %s" % (prim, (lab.get("measured") or {}).get("action", "?")), "white"))
    pa, la, pv, lv = s.get("peak_acc"), s.get("acc_limit"), s.get("peak_vel"), s.get("vel_limit")
    if pa and la:
        lines.append(("acc%J1-6 " + " ".join("%d" % round(100 * a / l) for a, l in zip(pa, la)), "white"))
    extra = []
    if pv and lv:
        extra.append("vel %d%%" % round(100 * max(v / l for v, l in zip(pv, lv))))
    if s.get("min_clearance_m") is not None:
        extra.append("room %dmm" % round(1000 * s["min_clearance_m"]))
    if s.get("playback_scale") is not None:
        extra.append("x%.2f" % s["playback_scale"])
    if extra:
        lines.append(("   ".join(extra), "white"))
    if not ok and s.get("reasons"):
        lines.append((("; ".join(s["reasons"]))[:52], "0xff5050"))
    return lines


def _esc(path):
    return path.replace("\\", "/").replace(":", "\\:")


def tile_filter(clip, textdir, w, h):
    """ffmpeg -vf for one tile: the header box, the current bar, a red frame
    when rejected. Text goes through files (no escaping, no % expansion)."""
    os.makedirs(textdir, exist_ok=True)
    fs = max(10, int(w / 36))                  # ~50 characters across a tile
    ok = (clip.get("safety") or {}).get("ok")
    m = 0 if ok else 8                         # inside the red frame of a rejected clip
    f = ["scale=%d:%d" % (w, h)]
    lines = header_lines(clip)
    f.append("drawbox=x=0:y=0:w=iw:h=%d:color=black@0.55:t=fill" % (int(fs * 1.35) * len(lines) + 8 + m))
    for i, (text, col) in enumerate(lines):
        p = os.path.join(textdir, "h%d.txt" % i)
        with open(p, "w", encoding="utf-8") as fh:
            fh.write(text)
        f.append("drawtext=fontfile='%s':textfile='%s':expansion=none:x=%d:y=%d:fontsize=%d:fontcolor=%s"
                 % (FONT, _esc(p), 6 + m, 4 + m + i * int(fs * 1.35), fs, col))
    for k, (t0, t1, want, got) in enumerate(bars(clip)):
        p = os.path.join(textdir, "b%d.txt" % k)
        with open(p, "w", encoding="utf-8") as fh:
            fh.write("bar %d/%d  %s -> %s" % (k + 1, len(bars(clip)), want, got))
        f.append("drawtext=fontfile='%s':textfile='%s':expansion=none:x=6:y=h-%d:fontsize=%d:fontcolor=%s"
                 ":box=1:boxcolor=black@0.55:enable='between(t,%.3f,%.3f)'"
                 % (FONT, _esc(p), int(fs * 1.6), int(fs * 1.15), _hex(ACTION_RGB.get(want, (1, 1, 1))), t0, t1))
    f.append("drawtext=fontfile='%s':text='%%{pts\\:hms}':x=w-tw-6:y=h-%d:fontsize=%d:fontcolor=white"
             % (FONT, int(fs * 1.5), fs))
    if not ok:
        f.append("drawbox=x=0:y=0:w=iw:h=ih:color=0xff3030:t=6")          # rejected: a red frame
    return ",".join(f)


def pages(items, per_page):
    return [items[i:i + per_page] for i in range(0, len(items), per_page)]


def grid_for(n):
    """Columns x rows holding n tiles, as square as possible."""
    cols = int(math.ceil(math.sqrt(n)))
    return cols, int(math.ceil(n / float(cols)))


# --------------------------------------------------------------------------
# the scene (hython)
# --------------------------------------------------------------------------

def review_path(node):
    """Python SOP: the TCP path of the clip named on CELL_CTRL, coloured by
    the intended action of its bar, unlit."""
    import hou
    geo = node.geometry()
    geo.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Global, "gl_lit", 0)          # unlit: the action colours at full strength
    path = hou.node("/obj/CELL_CTRL").evalParm("clip")
    if not path.lower().endswith(".json") or not os.path.exists(path):
        return
    clip = json.load(open(path))
    if not clip.get("points") and (clip.get("style") or {}).get("primitive"):
        # rejected before any motion: the path it was asked for, red where the
        # arm cannot hold the tool direction (the factory's own check)
        import clip_factory
        poly = geo.createPolygon(is_closed=False)
        for xyz, ok in clip_factory.reach_along(clip["style"], 120):
            p = geo.createPoint()
            p.setPosition((xyz[0], xyz[2], -xyz[1]))
            p.setAttribValue("Cd", (0.85, 0.85, 0.85) if ok else (1.0, 0.2, 0.15))
            poly.addVertex(p)
        return
    b = bars(clip)
    poly = geo.createPolygon(is_closed=False)
    for pt, xyz in zip(clip["points"], clip.get("tcp") or []):
        c = next((ACTION_RGB.get(want, (0.85, 0.85, 0.85)) for t0, t1, want, _ in b if t0 <= pt["t"] <= t1),
                 (0.85, 0.85, 0.85))
        p = geo.createPoint()
        p.setPosition((xyz[0], xyz[2], -xyz[1]))
        p.setAttribValue("Cd", c)
        poly.addVertex(p)


def setup_scene(w=TILE[0], h=TILE[1], room=ROOM):
    """Turn the loaded scenes/FR20_cell.hiplc into the review scene; returns
    the camera. The clip comes from CELL_CTRL's Clip, as always. room: a
    ROOMS key -- how much of the room is drawn."""
    import hou
    obj = hou.node("/obj")
    # the TCP path as a tube (a line is a hair at 480 px)
    po = obj.createNode("geo", "review_path", run_init_scripts=False)
    sop = po.createNode("python", "path")
    sop.parm("python").set("import sys, hou\nsys.path.insert(0, %r)\nimport render_clip_review as R\n"
                           "R.review_path(hou.pwd())\n" % (ROOT + "/scripts"))
    tube = po.createNode("polywire", "tube")
    tube.setInput(0, sop)
    tube.parm("radius").set(0.01)
    tube.parm("div").set(6)
    tube.setDisplayFlag(True)
    tube.setRenderFlag(True)
    # see-through faces (walls, zones, the stage) stacked in front of the
    # camera wash the picture out: never drawn; the rest per ROOMS
    env_obj = hou.node("/obj/cell_env")
    shown = [c for c in env_obj.children() if c.isDisplayFlagSet()][0]
    cut = env_obj.createNode("attribwrangle", "review_room")
    cut.setInput(0, shown)
    cut.parm("class").set(1)                              # primitives
    cut.parm("snippet").set('int closed = primintrinsic(0, "closed", @primnum);\n'
                            'if (!(%s)) removeprim(0, @primnum, 1);' % ROOMS[room])
    cut.setDisplayFlag(True)
    cut.setRenderFlag(True)
    # the arm, the subject: drawn once (not the asset's own room), near-white
    arm = hou.node("/obj/fr20/robot_arm")
    for n in ("show_cell", "show_curve_check"):
        if arm.parm(n) is not None:
            arm.parm(n).set(0)
    strip = arm.parent().createNode("attribdelete", "review_nocd")
    strip.setInput(0, arm)
    for n in ("ptdel", "vtxdel", "primdel"):
        strip.parm(n).set("Cd")
    white = arm.parent().createNode("color", "review_white")
    white.setInput(0, strip)
    white.parm("class").set(1)
    white.parmTuple("color").set(ROBOT_RGB)
    white.setDisplayFlag(True)
    white.setRenderFlag(True)
    for n in ("capsules", "ghosts"):
        if hou.node("/obj/" + n) is not None:
            hou.node("/obj/" + n).setDisplayFlag(False)
    # the user's viewport camera, cropped square onto the stage
    cam = obj.createNode("cam", "review_cam")
    cam.setWorldTransform(hou.Matrix4(CAM_XFORM))
    cam.parm("focal").set(CAM_FOCAL)
    cam.parm("aperture").set(CAM_APERTURE)
    p = hou.Vector3(*FRAME_ON) * cam.worldTransform().inverted()
    k = CAM_FOCAL / CAM_APERTURE
    cam.parm("winx").set(k * p[0] / -p[2])
    cam.parm("winy").set(k * p[1] / -p[2] * (float(w) / h))
    cam.parm("winsizex").set(WINDOW)
    cam.parm("winsizey").set(WINDOW)
    cam.parm("resx").set(w)
    cam.parm("resy").set(h)
    viewport_look(cam)
    return cam


def viewport_look(cam):
    """The viewport's lighting and background for an OpenGL render through
    cam: its headlight (a distant light riding on the camera), a soft
    ambient, the grey gradient behind everything."""
    import hou
    obj = hou.node("/obj")
    d = hou.Vector3(*HEADLIGHT_DIR).normalized()
    head = obj.createNode("hlight::2.0", "headlight")
    head.setFirstInput(cam)
    head.parm("light_type").set("distant")
    head.parmTuple("r").set((math.degrees(math.asin(d[1])), math.degrees(math.atan2(-d[0], -d[2])), 0.0))
    head.parm("light_intensity").set(HEADLIGHT * HEAD_GAIN)
    head.parm("shadow_type").set("off")
    amb = obj.createNode("envlight", "ambient")
    amb.parm("light_intensity").set(AMBIENT)
    amb.parm("shadow_type").set("off")
    # its grey gradient: an unlit panel far behind the scene, facing the camera
    # (a sphere round the scene would stand between it and the lights)
    sky = obj.createNode("geo", "review_backdrop", run_init_scripts=False)
    sky.setFirstInput(cam)
    sky.parmTuple("t").set((0.0, 0.0, -30.0))
    grid = sky.createNode("grid", "panel")
    grid.parm("orient").set(0)
    grid.parmTuple("size").set((80, 80))
    grad = sky.createNode("attribwrangle", "gradient")
    grad.setInput(0, grid)
    grad.parm("class").set(2)
    grad.parm("snippet").set("float t = clamp(fit(@P.y, -14, 14, 0, 1), 0, 1);\n"
                             "v@Cd = lerp(set(%g, %g, %g), set(%g, %g, %g), t);\n"
                             "setdetailattrib(0, 'gl_lit', 0);" % (BG_BOTTOM + BG_TOP))
    grad.setDisplayFlag(True)
    grad.setRenderFlag(True)


def opengl_settings(rop):
    """The viewport's look on an OpenGL ROP (or a ROP OpenGL TOP)."""
    for n, v in (("aamode", 3), ("hqlighting", 1), ("shadows", 0), ("ambocclusion", 0), ("usehdr", 1)):
        if rop.parm(n) is not None:
            rop.parm(n).set(v)


def render_frames(clip_path, frames_dir, w=TILE[0], h=TILE[1], room=ROOM):
    """Without PDG: one clip's frames, in this hython process."""
    import hou
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    hou.hipFile.load(ROOT + "/scenes/FR20_cell.hiplc", suppress_save_prompt=True, ignore_load_warnings=True)
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    cam = setup_scene(w, h, room)
    hou.node("/obj/CELL_CTRL").parm("clip").set(clip_path)
    n = nframes(json.load(open(clip_path)))
    rop = hou.node("/out").createNode("opengl")
    rop.parm("camera").set(cam.path())
    rop.parm("picture").set(frames_dir + "/f_$F4.png")
    rop.parm("trange").set(1)
    rop.parmTuple("f").deleteAllKeyframes()
    one = os.environ.get("REVIEW_ONE_FRAME")               # a single frame, to look at the picture
    rop.parmTuple("f").set((int(one), int(one), 1) if one else (1, n, 1))
    opengl_settings(rop)
    rop.render()


# --------------------------------------------------------------------------
# ffmpeg / ImageMagick steps
# --------------------------------------------------------------------------

def _run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("%s failed:\n%s" % (os.path.basename(cmd[0]), (r.stderr or r.stdout)[-1500:]))
    return r


def encode_tile(item, w=TILE[0], h=TILE[1]):
    """frames/f_####.png (or nothing: a card) -> tile.mp4 + poster.png."""
    clip = json.load(open(item["clip"]))
    d = item["dir"]
    os.makedirs(d, exist_ok=True)
    tile, poster = d + "/tile.mp4", d + "/poster.png"
    vf = tile_filter(clip, d + "/text", w, h)
    frames = sorted(glob.glob(d + "/frames/f_*.png"))
    if frames and clip.get("points"):
        start = int(os.path.basename(frames[0])[2:6])
        src = ["-framerate", "%g" % FPS, "-start_number", str(start), "-i", d + "/frames/f_%04d.png"]
    elif frames:                                           # rejected before any motion: its one still, 3 s
        src = ["-loop", "1", "-framerate", "%g" % FPS, "-t", "3", "-i", frames[0]]
    else:
        src = ["-f", "lavfi", "-i", "color=c=0x201010:s=%dx%d:d=3:r=%g" % (w, h, FPS)]
    _run([FFMPEG, "-y", "-loglevel", "error"] + src + ["-vf", vf, "-pix_fmt", "yuv420p", "-c:v", "libx264",
                                                      "-crf", "20", tile])
    at = 0.4 * (len(frames) / FPS if frames else 3.0)
    _run([FFMPEG, "-y", "-loglevel", "error", "-ss", "%.3f" % at, "-i", tile, "-frames:v", "1", poster])
    return tile, poster


def _duration(path):
    return float(_run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", path]).stdout)


def stack(tiles, cols, rows, w, h, path, crf=22):
    """Tiles into one video, cols x rows; shorter ones hold their last frame."""
    durs = [_duration(t) for t in tiles]
    longest = max(durs)
    args = [FFMPEG, "-y", "-loglevel", "error"]
    for t in tiles:
        args += ["-i", t]
    parts, labels = [], []
    for i, dur in enumerate(durs):
        parts.append("[%d:v]scale=%d:%d,tpad=stop_mode=clone:stop_duration=%.3f[v%d]" % (i, w, h, longest - dur + 0.05, i))
        labels.append("[v%d]" % i)
    for i in range(len(tiles), cols * rows):               # empty cells: black
        parts.append("color=c=black:s=%dx%d:d=%.3f[v%d]" % (w, h, longest, i))
        labels.append("[v%d]" % i)
    layout = "|".join("%d_%d" % ((i % cols) * w, (i // cols) * h) for i in range(cols * rows))
    parts.append("%sxstack=inputs=%d:layout=%s[out]" % ("".join(labels), cols * rows, layout))
    _run(args + ["-filter_complex", ";".join(parts), "-map", "[out]", "-t", "%.3f" % longest,
                 "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", str(crf), path])
    print("wrote", path, "%.1f s, %d tiles" % (longest, len(tiles)))
    return path


def build_videos(items, grid=(4, 2), overview=(5, 5), overview_tile=432):
    """Pages per set, and every clip in overview pages of overview[0] x [1]."""
    out = []
    for s in sorted(set(i["set"] for i in items), key=list(SETS).index):
        tiles = [i["dir"] + "/tile.mp4" for i in items if i["set"] == s and os.path.exists(i["dir"] + "/tile.mp4")]
        for k, grp in enumerate(pages(tiles, grid[0] * grid[1])):
            out.append(stack(grp, grid[0], grid[1], TILE[0], TILE[1], "%s/page_%s_%d.mp4" % (OUT, s, k + 1)))
    every = [i["dir"] + "/tile.mp4" for i in items if os.path.exists(i["dir"] + "/tile.mp4")]
    sets = sorted(set(i["set"] for i in items))
    name = "overview_" + sets[0] if len(sets) == 1 else "overview"          # one set: its own overview pages
    for k, grp in enumerate(pages(every, overview[0] * overview[1])):
        out.append(stack(grp, overview[0], overview[1], overview_tile, overview_tile,
                         "%s/%s_%d.mp4" % (OUT, name, k + 1), crf=24))
    return out


def contact_sheet(posters, path, magick="magick"):
    """ImageMagick montage of the posters (the PDG network uses its ImageMagick TOP)."""
    c, _ = grid_for(len(posters))
    _run([magick, "montage"] + posters + ["-tile", "%dx" % c, "-geometry", "+4+4", "-background", "#202224", path])
    return path


# --------------------------------------------------------------------------
# without PDG
# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", choices=("dance", "clips", "all"), default="all")
    ap.add_argument("--ids", nargs="*")
    ap.add_argument("--ok-only", action="store_true", help="leave out the rejected clips (else framed red)")
    ap.add_argument("--force", action="store_true", help="render again even where frames exist")
    ap.add_argument("--hython", default=HYTHON)
    ap.add_argument("--magick", default="magick")
    a = ap.parse_args(argv)
    items = clip_items(["dance", "clips"] if a.set == "all" else [a.set], a.ids, a.ok_only)
    for it in items:
        fr = it["dir"] + "/frames"
        if it["nframes"] and (a.force or not glob.glob(fr + "/f_*.png")):
            os.makedirs(fr, exist_ok=True)
            for f in glob.glob(fr + "/*.png"):
                os.remove(f)
            # one process per clip: a second OpenGL render in one hython crashes 22.0.368
            _run([a.hython, os.path.abspath(__file__), "--frames", it["clip"], fr])
        encode_tile(it)
        print("tile", it["id"])
    build_videos(items)
    contact_sheet([i["dir"] + "/poster.png" for i in items], OUT + "/contact_sheet.png", a.magick)
    return 0


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    c = json.load(open(ROOT + "/tests/clips/d17_punch-float-punch.json"))
    lines = header_lines(c)
    check("a dance tile names the clip, intent and measured per bar", lines[0][0].startswith(c["id"])
          and lines[1][0].startswith("intent") and lines[2][0].startswith("meas.") and "/" in lines[2][0], lines)
    check("... and acceleration as % of each joint's limit",
          any(t.startswith("acc%J1-6") and len(t.split()) == 7 for t, _ in lines), lines)
    check("every line fits a 480 px tile (~52 characters)", all(len(t) <= 52 for t, _ in lines), [len(t) for t, _ in lines])
    b = bars(c)
    check("bars carry their times and both actions", b and all(len(x) == 4 and x[1] > x[0] for x in b), b)
    import tempfile
    f = tile_filter(c, tempfile.mkdtemp(), 480, 480)
    check("the bar text shows only while its bar plays", f.count("enable='between(t,") == len(b), f[:120])
    check("an OK clip has no red frame", "0xff3030" not in f)
    bad = dict(c, safety=dict(c["safety"], ok=False, reasons=["cell: obstacle"]))
    check("a rejected clip is framed red, its reason in red", "0xff3030" in tile_filter(bad, tempfile.mkdtemp(), 480, 480)
          and any(t.startswith("cell") for t, _ in header_lines(bad)))
    check("frames at 24 fps from the clip's last time", nframes(c) == 1 + int(math.ceil(c["points"][-1]["t"] * 24)))
    check("pages of 8", [len(p) for p in pages(list(range(19)), 8)] == [8, 8, 3])
    check("the review's room style is one of ROOMS, 'full' among them", ROOM in ROOMS and "full" in ROOMS)
    check("98 clips in one square-ish grid", grid_for(98) == (10, 10) and grid_for(3) == (2, 2))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    if "--frames" in sys.argv:
        i = sys.argv.index("--frames")
        room = sys.argv[sys.argv.index("--room") + 1] if "--room" in sys.argv else ROOM
        render_frames(sys.argv[i + 1], sys.argv[i + 2], room=room)
        sys.exit(0)
    sys.exit(main())
