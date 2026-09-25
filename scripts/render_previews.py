"""Preview pictures of the atlas and the clip library, rendered headless.

    hython scripts/render_previews.py [out_dir]     (default docs/images/previews)

Each picture renders in its own hython process: a second OpenGL render in
one process crashed hython 22.0.368 (segmentation fault).

Needs the atlas baked (build_atlas_scene.py --bake) and the factory run
(build_factory_scene.py --cook). Writes, with the OpenGL ROP:

    atlas_overview.png   half shell + vertical headroom slice, FR20 for scale
    atlas_top.png        horizontal slice at 0.6 m, headroom, from above
    atlas_wrist.png      vertical slice coloured by |sin q5| (red = wrist
                         singularity, where J4/J6 have to spin)
    clips_workspace.png  every factory variant's path around the robot: ok
                         clips coloured by TCP speed, rejected ones red
    clips_sheet.png      the ok clips one per cell, drawn in their own plane,
                         coloured by TCP speed, with id / duration / peak speed
    cell_overview.png    the cell (envs/volvox_lab.json), the robot mid-phrase
                         and its capsules coloured by clearance
    cell_ghosts.png      onion skin of a sample dance phrase in the cell
    dance_sheet.png      every dance phrase's TCP path seen from the front,
                         coloured bar by bar by its intended Laban action,
                         with the measured action sequence

Nothing is saved to the scenes.
"""

import glob
import json
import math
import os
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
sys.path.insert(0, ROOT + "/scripts")
ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
OUT = (ARGS[0] if ARGS else ROOT + "/docs/images/previews").replace("\\", "/")
ONLY = next((a[len("--only="):] for a in sys.argv[1:] if a.startswith("--only=")), None)
CLIPS = ROOT + "/geo/clips"


def _install():
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)


def _camera(name, eye, target, res=(1080, 1350), ortho_width=None):
    cam = hou.node("/obj").createNode("cam", name)
    eye, target = hou.Vector3(eye), hou.Vector3(target)
    d = (target - eye).normalized()
    cam.parmTuple("t").set(tuple(eye))
    cam.parmTuple("r").set((math.degrees(math.asin(max(-1.0, min(1.0, d[1])))),
                            math.degrees(math.atan2(-d[0], -d[2])), 0.0))
    cam.parm("resx").set(res[0])
    cam.parm("resy").set(res[1])
    if ortho_width:
        cam.parm("projection").set("ortho")
        cam.parm("orthowidth").set(ortho_width)
    return cam


def _lights():
    if hou.node("/obj/key") is None:
        k = hou.node("/obj").createNode("hlight::2.0", "key")
        k.parmTuple("t").set((4, 6, 6))
        k.parmTuple("r").set((-40, 30, 0))
        hou.node("/obj").createNode("envlight", "fill")


def _render(cam, path, objects):
    rop = hou.node("/out").createNode("opengl")
    rop.parm("camera").set(cam.path())
    rop.parm("picture").set(path)
    rop.parm("trange").set(0)
    rop.parm("aamode").set(3)
    if rop.parm("vobject"):
        rop.parm("vobject").set(" ".join(objects))
    for o in hou.node("/obj").children():
        if o.type().name() == "geo" or o.type().name() == "subnet":
            o.setDisplayFlag(o.path() in objects or o.name() in objects)
    rop.render()
    print("wrote", path)


# --------------------------------------------------------------------------
# atlas
# --------------------------------------------------------------------------

def atlas_pictures():
    hou.hipFile.load(ROOT + "/scenes/FR20_atlas.hiplc", suppress_save_prompt=True, ignore_load_warnings=True)
    _install()
    _lights()
    objs = ["/obj/fr20_atlas", "/obj/fr20_robot"]
    geo = hou.node("/obj/fr20_atlas")
    look = geo.node("slice_look")
    grid = geo.node("slice")

    if ONLY == "atlas_overview":
        cam = _camera("c_over", (6.2, 2.8, 4.2), (0, 0.55, 0))
        _render(cam, OUT + "/atlas_overview.png", objs)
        return

    # horizontal slice from above, shell hidden
    grid.parm("orient").set(2)               # ZX
    grid.parmTuple("size").set((4.4, 4.4))
    grid.parmTuple("t").set((0.0, 0.6, 0.0))
    geo.node("OUT").setInput(0, None)
    if ONLY == "atlas_top":
        cam = _camera("c_top", (0.0, 7.0, 0.001), (0, 0.6, 0), res=(1080, 1080), ortho_width=4.6)
        _render(cam, OUT + "/atlas_top.png", objs)
        return

    # vertical slice by wrist distance
    # removing a Merge input shifts the others down: set both again
    out = geo.node("OUT")
    out.setInput(0, geo.node("shell_look"))
    out.setInput(1, look)
    grid.parm("orient").set(0)
    grid.parmTuple("size").set((4.4, 3.4))
    grid.parmTuple("t").set((0.0, 0.65, 0.0))
    look.parm("snippet").set(look.parm("snippet").eval()
                             .replace('float t = clamp(h / chf("full_speed"), 0, 1);',
                                      'float t = clamp(f@wrist / 0.5, 0, 1);'))
    cam = _camera("c_wrist", (0.0, 0.65, 6.5), (0, 0.65, 0), res=(1080, 1000), ortho_width=4.6)
    _render(cam, OUT + "/atlas_wrist.png", objs)


# --------------------------------------------------------------------------
# clips
# --------------------------------------------------------------------------

def _speed_rgb(v, vmax):
    t = max(0.0, min(1.0, v / vmax))
    return hou.Color.fromHSV(240.0 * (1.0 - t), 0.85, 1.0).rgb() if hasattr(hou.Color, "fromHSV") else (t, 0.3, 1.0 - t)


def _hsv(h, s, v):
    c = hou.Color()
    c.setHSV((h, s, v))
    return c.rgb()


def _clips():
    man = json.load(open(CLIPS + "/manifest.json"))
    out = []
    for e in man["clips"]:
        c = json.load(open(os.path.join(CLIPS, e["file"])))
        out.append((e, c))
    return out


def _urdf_to_h(p):
    return (p[0], p[2], -p[1])


def _polyline(geo, pts, colours, width_attr=None):
    poly = geo.createPolygon(is_closed=False)
    cd = geo.findPointAttrib("Cd")
    for p, c in zip(pts, colours):
        pt = geo.createPoint()
        pt.setPosition(p)
        pt.setAttribValue(cd, c)
        poly.addVertex(pt)


def _tcp_speeds(c):
    ts = [p["t"] for p in c["points"]]
    tcp = c["tcp"]
    v = [0.0]
    for i in range(1, len(tcp)):
        dt = ts[i] - ts[i - 1]
        v.append(math.dist(tcp[i], tcp[i - 1]) / dt if dt > 0 else 0.0)
    return v


def clip_pictures():
    import clip_factory as F
    hou.hipFile.clear(suppress_save_prompt=True)
    # $HIP must be scenes/: the asset finds profiles/ at $HIP/.. (not saved)
    hou.hipFile.setName(ROOT + "/scenes/_previews.hiplc")
    _install()
    _lights()
    clips = _clips()
    vmax = max(max(_tcp_speeds(c)) for e, c in clips if e["ok"])

    # 1. every variant around the robot
    obj = hou.node("/obj")
    rob = obj.createNode("geo", "robot", run_init_scripts=False)
    arm = rob.createNode("wenyi::robot_arm::1.0", "robot_arm")
    arm.parm("robot_profile").set("fr20")
    arm.hdaModule().on_profile_changed(arm)   # a scripted set() does not run the callback
    arm.setDisplayFlag(True)
    paths = obj.createNode("geo", "paths", run_init_scripts=False)
    sop = paths.createNode("python", "draw")
    sop.parm("python").set("")
    geo = hou.Geometry()
    geo.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    for e, c in clips:
        if e["ok"]:
            v = _tcp_speeds(c)
            _polyline(geo, [_urdf_to_h(p) for p in c["tcp"]], [_hsv(240.0 * (1 - min(1, x / vmax)), 0.9, 1.0) for x in v])
        else:
            st = dict(c["style"], id=c["id"])
            pts = F.path_points(st, 120)
            _polyline(geo, [_urdf_to_h(p) for p in pts], [(0.95, 0.1, 0.1)] * len(pts))
    # line width through a Polywire so the paths read on a phone
    stash = paths.createNode("stash", "src")
    stash.parm("stash").set(geo)
    wire = paths.createNode("polywire", "wire")
    wire.setInput(0, stash)
    wire.parm("radius").set(0.014)
    wire.setDisplayFlag(True)
    sop.destroy()
    if ONLY == "clips_workspace":
        cam = _camera("c_ws", (3.3, 3.6, 3.9), (0.0, 0.35, -0.1), res=(1080, 1080))
        _render(cam, OUT + "/clips_workspace.png", ["/obj/robot", "/obj/paths"])
        return

    # 2. contact sheet of the ok clips
    rob.setDisplayFlag(False)
    paths.setDisplayFlag(False)
    sheet = obj.createNode("geo", "sheet", run_init_scripts=False)
    oks = [(e, c) for e, c in clips if e["ok"]]
    cols = 5
    rows = int(math.ceil(len(oks) / float(cols)))
    cell = 1.0
    g2 = hou.Geometry()
    g2.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    merge = sheet.createNode("merge", "all")
    for k, (e, c) in enumerate(oks):
        r, col = divmod(k, cols)
        cx, cy = col * cell * 1.1, -r * cell * 1.3
        a, b = F._axes(c["style"].get("plane", "xy"))
        uv = [(sum(p[i] * a[i] for i in range(3)), sum(p[i] * b[i] for i in range(3))) for p in c["tcp"]]
        us, vs = [x for x, _ in uv], [y for _, y in uv]
        mu, mv = (min(us) + max(us)) / 2, (min(vs) + max(vs)) / 2
        span = max(max(us) - min(us), max(vs) - min(vs), 1e-6)
        s = 0.8 * cell / span
        v = _tcp_speeds(c)
        _polyline(g2, [(cx + (x - mu) * s, cy + (y - mv) * s, 0.0) for x, y in uv],
                  [_hsv(240.0 * (1 - min(1, x / vmax)), 0.9, 1.0) for x in v])
        txt = sheet.createNode("font", "t%d" % k)
        txt.parm("text").set("%s  %.1f s  %.2f m/s" % (c["id"], c["meta"]["duration_s"], max(v)))
        txt.parm("fontsize").set(0.075)
        txt.parmTuple("t").set((cx, cy - 0.5 * cell, 0.0))
        merge.setNextInput(txt)
    stash = sheet.createNode("stash", "paths")
    stash.parm("stash").set(g2)
    wire = sheet.createNode("polywire", "wire")
    wire.setInput(0, stash)
    wire.parm("radius").set(0.008)
    merge.setNextInput(wire)
    merge.setDisplayFlag(True)
    w = cols * cell * 1.1
    h = rows * cell * 1.3
    cx, cy = (cols - 1) * cell * 1.1 / 2, -(rows - 1) * cell * 1.3 / 2 - 0.1
    cam = _camera("c_sheet", (cx, cy, 10.0), (cx, cy, 0.0), res=(1080, int(1080 * h / w)), ortho_width=w)
    _render(cam, OUT + "/clips_sheet.png", ["/obj/sheet"])
    print("TCP speed colour: blue 0 -> red %.2f m/s" % vmax)


ACTION_HUE = {"punch": 0, "slash": 28, "press": 50, "wring": 290, "dab": 185, "flick": 60, "glide": 120, "float": 215}
SAMPLE = ROOT + "/tests/clips"


def _sample_clip():
    files = sorted(glob.glob(SAMPLE + "/*.json"), key=os.path.getsize)
    return files[-1] if files else ROOT + "/tests/csv/fr20_test.csv"


def cell_pictures():
    hou.hipFile.load(ROOT + "/scenes/FR20_cell.hiplc", suppress_save_prompt=True, ignore_load_warnings=True)
    _install()
    _lights()
    ctrl = hou.node("/obj/CELL_CTRL")
    ctrl.parm("clip").set(_sample_clip())
    hou.session.cell_sop.fit_range()
    end = hou.playbar.frameRange()[1]
    if ONLY == "cell_overview":
        hou.setFrame(int(end * 0.45))
        cam = _camera("c_cell", (2.2, 3.4, -2.4), (-0.7, 0.75, 0.1), res=(1080, 1080))
        _render(cam, OUT + "/cell_overview.png", ["/obj/cell_env", "/obj/fr20", "/obj/capsules"])
    else:
        hou.setFrame(1)
        cam = _camera("c_ghost", (1.4, 2.6, -2.6), (-0.8, 0.9, 0.0), res=(1080, 1080))
        _render(cam, OUT + "/cell_ghosts.png", ["/obj/cell_env", "/obj/ghosts"])


def dance_sheet():
    hou.hipFile.clear(suppress_save_prompt=True)
    hou.hipFile.setName(ROOT + "/scenes/_previews.hiplc")
    _install()
    _lights()
    d = ROOT + "/geo/dance"
    man = json.load(open(d + "/manifest.json"))
    clips = [json.load(open(os.path.join(d, e["file"]))) for e in man["clips"] if e["ok"]]
    cols = 6
    rows = int(math.ceil(len(clips) / float(cols)))
    g = hou.Geometry()
    g.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    obj = hou.node("/obj")
    sheet = obj.createNode("geo", "dance", run_init_scripts=False)
    merge = sheet.createNode("merge", "all")
    for k, c in enumerate(clips):
        r, col = divmod(k, cols)
        cx, cy = col * 1.25, -r * 1.45
        # seen from the front: the robot's left on the viewer's right
        uv = [(p[1], p[2]) for p in c["tcp"]]
        us, vs = [a for a, _ in uv], [b for _, b in uv]
        mu, mv = (min(us) + max(us)) / 2, (min(vs) + max(vs)) / 2
        sc = 0.9 / max(max(us) - min(us), max(vs) - min(vs), 0.05)
        ts = [p["t"] for p in c["points"]]
        bars = c["labels"].get("bars", [])

        def hue(t):
            for b in bars:
                if b["t0"] <= t <= b["t1"]:
                    return ACTION_HUE.get(b["intent"], 0)
            return None
        run = []
        for (u, v), t in zip(uv, ts):
            h = hue(t)
            col_ = _hsv(h, 0.9, 1.0) if h is not None else (0.75, 0.75, 0.75)
            run.append(((cx + (u - mu) * sc, cy + (v - mv) * sc, 0.0), col_))
        _polyline(g, [p for p, _ in run], [c_ for _, c_ in run])
        txt = sheet.createNode("font", "t%d" % k)
        seq = c["labels"].get("sequence", [])
        txt.parm("text").set("%s\n%.0f s  %s" % (c["id"][:3] + " " + "-".join(b["intent"] for b in bars),
                                                 c["meta"]["duration_s"], "-".join(seq)))
        txt.parm("fontsize").set(0.07)
        txt.parmTuple("t").set((cx, cy - 0.58, 0.0))
        merge.setNextInput(txt)
    # legend
    for i, (a, h) in enumerate(sorted(ACTION_HUE.items(), key=lambda x: x[0])):
        _polyline(g, [(i * 0.9, 1.0, 0.0), (i * 0.9 + 0.25, 1.0, 0.0)], [_hsv(h, 0.9, 1.0)] * 2)
        t = sheet.createNode("font", "leg%d" % i)
        t.parm("text").set(a)
        t.parm("fontsize").set(0.09)
        t.parmTuple("t").set((i * 0.9 + 0.45, 0.97, 0.0))
        merge.setNextInput(t)
    st = sheet.createNode("stash", "paths")
    st.parm("stash").set(g)
    w = sheet.createNode("polywire", "wire")
    w.setInput(0, st)
    w.parm("radius").set(0.009)
    merge.setNextInput(w)
    merge.setDisplayFlag(True)
    wd, ht = cols * 1.25, rows * 1.45 + 0.8
    cx, cy = (cols - 1) * 1.25 / 2, -(rows - 1) * 1.45 / 2 + 0.3
    cam = _camera("c_dance", (cx, cy, 12.0), (cx, cy, 0.0), res=(1080, int(1080 * ht / wd)), ortho_width=wd)
    _render(cam, OUT + "/dance_sheet.png", ["/obj/dance"])


# atlas_wrist (--only=atlas_wrist) is left out of the default set: with the
# tool down the best branch keeps |sin q5| >= 0.5 everywhere, so it is one
# flat colour -- worth rendering for tilted tool directions
PICTURES = ("atlas_overview", "atlas_top", "clips_workspace", "clips_sheet", "cell_overview", "cell_ghosts",
            "dance_sheet")

if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    if ONLY is None:
        import subprocess
        for name in PICTURES:
            subprocess.run([sys.executable, os.path.abspath(__file__), OUT, "--only=" + name], check=False)
    else:
        _install()
        if ONLY.startswith("atlas"):
            atlas_pictures()
        elif ONLY.startswith("cell"):
            cell_pictures()
        elif ONLY == "dance_sheet":
            dance_sheet()
        else:
            clip_pictures()
