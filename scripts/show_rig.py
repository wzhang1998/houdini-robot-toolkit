"""The show, edited in Houdini: zones, hubs, the operating range and the
library as objects and parameters next to the arm, written back to the
show config (shows/*.json). It lives in the show scene
(scenes/FR20_show.hiplc, built by scripts/build_show_scene.py); the rig
scene (FR20_rig.hiplc) stays the hand-authoring tool.

Install it into an open scene (the Python Shell, or the Houdini Agent
bridge); it adds objects and leaves the arm alone:

    import show_rig; show_rig.install()                  # shows/party.json
    import show_rig; show_rig.install("D:/.../shows/other.json")

In the show scene it also plays a built segment, or holds a hub's pose, on
the arm (CELL_CTRL's clip), draws the room once (cell_env; the robot_arm's
own Show Cell off), hides the collision capsules, and turns the viewport's
Remove Backfaces on (the room's walls face in: a cutaway, as in Isaac).

What it adds (/obj, in a network box "SHOW"):

    SHOW          the controls: config file; Load / Write; operating range
                  (J1 sector, TCP height band, speed); library (clip length,
                  bars, BPM, intensity, seed, clips per hub visit); Add Hub,
                  Add Zone; Add the robot_arm's exported clip to the library;
                  Build Show, Dry Run, Check; the report
    zone_<name>   a box: move, rotate (Y only) and scale it in the viewport.
                  Translate = centre, Rotate Y = yaw, Scale = size.
                  zone_stage is the work zone the clips stay in (used when
                  SHOW's "Override the Env's Stage" is on)
    hub_<name>    a hub. Tool + Look mode: the null is where the tool tip
                  is, look_<name> is what the tool looks at, and the arm pose
                  is solved live (nearest the hub's Seed Pose). Joints mode:
                  the pose is the hub's joint angles, the null sits at its
                  tool tip (locked)
    look_<name>   the look target of hub_<name>
    show_viz      drawn live: each hub's pose as a ghost arm (its colour; red
                  when it cannot be reached, is not clear of the room, or is
                  outside the operating range), the look rays, the operating
                  range (J1 sector and TCP height band), the built clips'
                  tool paths (idle in the hub's colour, moves white)

The config stays the one source: Load reads it into the objects, Write
writes them back (keys the scene does not show -- OSC, sequences, canvas,
scan, margins -- are kept), Build runs `python scripts/show.py build` on it.

Frames: the config and collision.py are in the robot base frame (URDF, Z
up); Houdini is Y up: (x, y, z) -> (x, z, -y). A yaw about the robot's Z is
the same angle about Houdini's Y.

    python scripts/show_rig.py --self-test               # the parts without hou
"""

import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE).replace("\\", "/")
if HERE not in sys.path:
    sys.path.insert(0, HERE)

DEFAULT_CONFIG = ROOT + "/shows/party.json"
PYTHON = os.environ.get("SHOW_PYTHON", "python")          # a Python with the toolkit's needs
J1_FACING_DEG = 180.0              # J1 = 0 reaches along -X (UR convention): facing = J1 + 180
HUB_RGB = [(0.95, 0.45, 0.2), (0.3, 0.7, 1.0), (0.55, 0.9, 0.35), (0.85, 0.4, 0.95), (1.0, 0.85, 0.25)]
ZONE_RGB = {"audience": (0.35, 0.6, 1.0), "greet": (1.0, 0.55, 0.2), "idle": (0.6, 0.85, 0.4),
            "stage": (0.2, 0.9, 0.35)}
BAD_RGB = (1.0, 0.1, 0.1)
GHOST_SCALE = 0.55                 # ghost arms thinner than the collision capsules: they read as a pose
REST_Q = [-60.0, -90.0, 90.0, -90.0, -90.0, 0.0]
FAMILIES = "look wave nod reach tilt trace"


# --------------------------------------------------------------------------
# frames and conversions (no hou)
# --------------------------------------------------------------------------

def to_h(p):
    """Robot frame (Z up) -> Houdini (Y up)."""
    return (p[0], p[2], -p[1])


def to_r(p):
    """Houdini (Y up) -> robot frame (Z up)."""
    return (p[0], -p[2], p[1])


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _r(v, n=4):
    return [round(float(x), n) for x in v]


def zone_to_xform(z):
    """A zone {center, size, yaw_deg} -> the object's (t, r, s) in Houdini."""
    s = z["size"]
    return to_h(z["center"]), (0.0, z.get("yaw_deg", 0.0), 0.0), (s[0], s[2], s[1])


def xform_to_zone(t, r, s, scale=1.0):
    """An object's (t, r, s, uniform scale) -> a zone {center, size, yaw_deg}.
    Only the rotation about Y is kept: zones are upright boxes."""
    return {"center": _r(to_r(t)), "size": _r((s[0] * scale, s[2] * scale, s[1] * scale)),
            "yaw_deg": round(_wrap(r[1]), 3)}


def range_guide(j1_lo, j1_hi, z_lo, z_hi, radius, step=5.0):
    """The operating range as polylines in the robot frame: the J1 sector on
    the floor (arc and its two edges), the TCP height band (arcs at z_lo and
    z_hi) and the sector's edges between them. Facing = J1 + J1_FACING_DEG."""
    n = max(2, int(math.ceil((j1_hi - j1_lo) / step)) + 1)
    angs = [math.radians(j1_lo + (j1_hi - j1_lo) * k / (n - 1) + J1_FACING_DEG) for k in range(n)]

    def arc(z):
        return [(radius * math.cos(a), radius * math.sin(a), z) for a in angs]

    floor = [(0.0, 0.0, 0.01)] + arc(0.01) + [(0.0, 0.0, 0.01)]
    edges = [[(radius * math.cos(a), radius * math.sin(a), z_lo), (radius * math.cos(a), radius * math.sin(a), z_hi)]
             for a in (angs[0], angs[-1])]
    return [floor, arc(z_lo), arc(z_hi)] + edges


def merge_config(cfg, scene):
    """The config with what the scene shows put over it. `scene` holds
    zones {name: zone}, stage (a zone or None: the env's), hubs {name: hub},
    start_hub, range, library, select, and authored (list, optional). Keys
    the scene does not show are kept, and so are a hub's other keys (note)."""
    out = json.loads(json.dumps(cfg))
    zones = {}
    for name, z in scene["zones"].items():
        zones[name] = dict(cfg.get("zones", {}).get(name, {}), **z)
    out["zones"] = zones
    if scene.get("stage"):
        out["stage"] = dict(cfg.get("stage") or {}, name="stage", **scene["stage"])
    else:
        out.pop("stage", None)
    hubs = {}
    for name, h in scene["hubs"].items():
        old = dict(cfg.get("hubs", {}).get(name, {}))
        if h.get("tcp") is not None:
            for k in ("q",):
                old.pop(k, None)
        else:
            for k in ("tcp", "look", "near"):
                old.pop(k, None)
        hubs[name] = dict(old, **h)
    out["hubs"] = hubs
    out["start_hub"] = scene["start_hub"] if scene["start_hub"] in hubs else next(iter(hubs), None)
    out["range"] = dict(cfg.get("range", {}), **scene["range"])
    out["library"] = dict(cfg.get("library", {}), **scene["library"])
    out["select"] = dict(cfg.get("select", {}), **scene["select"])
    if scene.get("authored") is not None:
        out["authored"] = scene["authored"]
    return out


def check_config(cfg):
    """What in the config names something that is not there, or cannot be."""
    bad = []
    hubs, zones = cfg.get("hubs", {}), cfg.get("zones", {})
    if not hubs:
        bad.append("no hubs")
    if cfg.get("start_hub") not in hubs:
        bad.append("start hub %r is not a hub" % cfg.get("start_hub"))
    for name, s in cfg.get("sequences", {}).items():
        if s.get("hub") not in hubs:
            bad.append("sequence %s plays at hub %r, which is not there" % (name, s.get("hub")))
    if cfg.get("scan") and cfg["scan"].get("from_hub") not in hubs:
        bad.append("the scan starts from hub %r, which is not there" % cfg["scan"].get("from_hub"))
    for a in cfg.get("authored", []):
        if a.get("hub") not in hubs:
            bad.append("authored clip %s plays at hub %r, which is not there" % (a.get("id"), a.get("hub")))
    for name, h in hubs.items():
        if h.get("zone") and h["zone"] not in zones:
            bad.append("hub %s uses zone %r, which is not there" % (name, h["zone"]))
        if h.get("generator") == "gestures" and not (h.get("tcp") and h.get("look")):
            bad.append("hub %s makes gestures but has no tool + look" % name)
    r = cfg.get("range", {})
    for k in ("j1_deg", "tcp_z"):
        if r.get(k) and r[k][0] >= r[k][1]:
            bad.append("range %s: %g is not below %g" % (k, r[k][0], r[k][1]))
    if r.get("speed") is not None and not 0.05 <= r["speed"] <= 1.0:
        bad.append("range speed %g is not within 0.05..1" % r["speed"])
    return bad


def add_authored(authored, csv, hub, clip_id):
    """The authored list with (csv, hub, id) in it: an entry with the same id
    is replaced. csv is kept relative to the repo when it is inside it."""
    p = csv.replace("\\", "/")
    if p.lower().startswith(ROOT.lower() + "/"):
        p = p[len(ROOT) + 1:]
    out = [a for a in authored if a.get("id") != clip_id]
    out.append({"id": clip_id, "hub": hub, "csv": p})
    return out


# --------------------------------------------------------------------------
# the scene (hou)
# --------------------------------------------------------------------------

def _hou():
    import hou
    return hou


def _show():
    return _hou().node("/obj/SHOW")


def cfg_path(node=None):
    return (node or _show()).evalParm("config").replace("\\", "/")


def show_name(node=None):
    return os.path.splitext(os.path.basename(cfg_path(node)))[0]


def preview_dir(node=None):
    return ROOT + "/geo/show/" + show_name(node)


def _cb(code):
    hou = _hou()
    return {"script_callback": "import show_rig; show_rig." + code,
            "script_callback_language": hou.scriptLanguage.Python}


def _objs(kind):
    """{name: node} of the show's objects of a kind ("zone" / "hub" / "look")."""
    out = {}
    for n in _hou().node("/obj").children():
        p = n.parm("show_role")
        if p and p.evalAsString() == kind:
            out[n.name()[len(kind) + 1:]] = n
    return out


def _role_parm(node, kind):
    hou = _hou()
    g = node.parmTemplateGroup()
    if g.find("show_role") is None:
        g.append(hou.StringParmTemplate("show_role", "Show Role", 1, default_value=(kind,), is_hidden=True))
        node.setParmTemplateGroup(g)
    node.parm("show_role").set(kind)


def _xform(node):
    ev = lambda n: node.parmTuple(n).eval()
    return ev("t"), ev("r"), ev("s"), node.evalParm("scale")


def _set_xform(node, t, r, s):
    node.parmTuple("t").set(t)
    node.parm("ry").set(r[1])                        # rx / rz are locked: zones turn about Y only
    node.parmTuple("s").set(s)
    node.parm("scale").set(1.0)


# --- building the controls ---------------------------------------------------

def _show_parms(node, config):
    hou = _hou()
    T = hou
    g = node.parmTemplateGroup()
    if g.find("config") is not None:
        return
    g.append(T.StringParmTemplate("config", "Show Config", 1, string_type=T.stringParmType.FileReference,
                                  default_value=(config,)))
    g.append(T.ButtonParmTemplate("load_b", "Load Config", help="Config -> the zone and hub objects",
                                  **_cb("load_config()")))
    g.append(T.ButtonParmTemplate("write_b", "Write Config", help="The scene -> the config file (other keys kept)",
                                  **_cb("write_config()")))

    hubs = T.FolderParmTemplate("hubs_f", "Hubs and Zones", folder_type=T.folderType.Simple)
    hubs.addParmTemplate(T.StringParmTemplate(
        "start_hub", "Start Hub", 1, menu_type=T.menuType.StringReplace,
        item_generator_script="import show_rig; return show_rig.hub_menu()",
        item_generator_script_language=T.scriptLanguage.Python))
    hubs.addParmTemplate(T.StringParmTemplate("new_name", "New Name", 1, default_value=("hub2",)))
    hubs.addParmTemplate(T.ButtonParmTemplate("add_hub_b", "Add Hub (Tool + Look)", **_cb("add_hub()")))
    hubs.addParmTemplate(T.ButtonParmTemplate("add_zone_b", "Add Zone", **_cb("add_zone()")))
    hubs.addParmTemplate(T.ToggleParmTemplate(
        "stage_override", "Override the Env's Stage", default_value=False,
        help="Off: the stage (work zone) of the env file. On: zone_stage"))
    g.append(hubs)

    rng = T.FolderParmTemplate("range_f", "Operating Range", folder_type=T.folderType.Simple)
    rng.addParmTemplate(T.FloatParmTemplate(
        "j1_range", "J1 Range (deg)", 2, default_value=(-150.0, 150.0), min=-175.0, max=175.0,
        help="The directions the arm may face. A clip whose J1 leaves it is dropped"))
    rng.addParmTemplate(T.FloatParmTemplate(
        "tcp_z", "Tool Tip Height (m)", 2, default_value=(0.35, 1.9), min=0.0, max=2.5,
        help="The band the tool tip stays in, above the robot's base"))
    rng.addParmTemplate(T.FloatParmTemplate(
        "speed", "Speed (of the limits)", 1, default_value=(0.85,), min=0.05, max=1.0,
        help="Clips are made at this fraction of the robot's measured velocity / acceleration limits"))
    rng.addParmTemplate(T.FloatParmTemplate("guide_radius", "Guide Radius (m)", 1, default_value=(1.2,),
                                            min=0.3, max=2.0, help="Only how the range is drawn"))
    g.append(rng)

    lib = T.FolderParmTemplate("lib_f", "Library", folder_type=T.folderType.Simple)
    lib.addParmTemplate(T.FloatParmTemplate("duration", "Clip Length (s)", 2, default_value=(4.0, 12.0),
                                            min=1.0, max=30.0))
    lib.addParmTemplate(T.IntParmTemplate("bars", "Bars", 2, default_value=(1, 2), min=1, max=4))
    lib.addParmTemplate(T.IntParmTemplate("bpm", "BPM", 2, default_value=(80, 125), min=40, max=180))
    lib.addParmTemplate(T.FloatParmTemplate(
        "intensity", "Intensity", 2, default_value=(0.4, 0.9), min=0.0, max=1.0,
        help="How big and quick the gestures are (a range: each clip draws one)"))
    lib.addParmTemplate(T.IntParmTemplate("seed", "Seed", 1, default_value=(11,)))
    lib.addParmTemplate(T.IntParmTemplate("hub_stay", "Clips per Hub Visit", 2, default_value=(2, 4),
                                          min=1, max=20))
    lib.addParmTemplate(T.IntParmTemplate("no_repeat", "No Repeat Within", 1, default_value=(6,), min=0, max=40))
    g.append(lib)

    auth = T.FolderParmTemplate("auth_f", "Authored Clip", folder_type=T.folderType.Simple)
    auth.addParmTemplate(T.LabelParmTemplate(
        "auth_note", "", column_labels=("A curve drawn on the robot_arm, exported as a joint CSV, "
                                        "played from a hub and back (checked moves in and out).",)))
    auth.addParmTemplate(T.StringParmTemplate(
        "auth_csv", "Joint CSV", 1, string_type=T.stringParmType.FileReference,
        default_value=('`chs("/obj/fr20/robot_arm/export_csv")`',)))
    auth.addParmTemplate(T.StringParmTemplate(
        "auth_hub", "Play at Hub", 1, menu_type=T.menuType.StringReplace,
        item_generator_script="import show_rig; return show_rig.hub_menu()",
        item_generator_script_language=T.scriptLanguage.Python))
    auth.addParmTemplate(T.StringParmTemplate("auth_id", "Clip Id", 1, default_value=("cat",)))
    auth.addParmTemplate(T.ButtonParmTemplate("auth_b", "Add Clip to the Library", **_cb("add_clip()")))
    g.append(auth)

    run = T.FolderParmTemplate("run_f", "Build and Check", folder_type=T.folderType.Simple)
    run.addParmTemplate(T.ButtonParmTemplate("check_b", "Check", help="Hubs reachable and clear, names consistent",
                                             **_cb("check()")))
    run.addParmTemplate(T.ButtonParmTemplate("build_b", "Build Show", help="Write, make and check every clip "
                                             "(a minute or more)", **_cb("build_show()")))
    run.addParmTemplate(T.FloatParmTemplate("dry_minutes", "Dry Run Minutes", 1, default_value=(10.0,),
                                            min=1.0, max=120.0))
    run.addParmTemplate(T.ButtonParmTemplate("dry_b", "Dry Run", **_cb("dry_run()")))
    run.addParmTemplate(T.ToggleParmTemplate("show_paths", "Show Built Paths", default_value=True))
    run.addParmTemplate(T.StringParmTemplate(
        "segment", "Play Segment on the Arm", 1, menu_type=T.menuType.Normal,
        item_generator_script="import show_rig; return show_rig.segment_menu()",
        item_generator_script_language=T.scriptLanguage.Python,
        help="A built segment played on the arm (the show scene: CELL_CTRL's clip)",
        **_cb("preview_segment()")))
    run.addParmTemplate(T.StringParmTemplate(
        "pose_hub", "Pose the Arm at Hub", 1, menu_type=T.menuType.Normal,
        item_generator_script="import show_rig; return show_rig.hub_menu()",
        item_generator_script_language=T.scriptLanguage.Python, **_cb("preview_hub()")))
    run.addParmTemplate(T.StringParmTemplate("report", "Report", 1, tags={"editor": "1", "editorlines": "10-30"}))
    g.append(run)
    node.setParmTemplateGroup(g)


def _hub_parms(node):
    hou = _hou()
    T = hou
    g = node.parmTemplateGroup()
    if g.find("hub_mode") is not None:
        return
    f = T.FolderParmTemplate("hub_f", "Hub", folder_type=T.folderType.Simple)
    f.addParmTemplate(T.MenuParmTemplate(
        "hub_mode", "Mode", ("tool", "joints"), ("Tool + Look", "Joints"),
        help="Tool + Look: this null is the tool tip, look_<name> what it aims at; the pose is solved. "
             "Joints: the pose is Seed Pose as given", **_cb("hub_mode_changed(kwargs['node'])")))
    f.addParmTemplate(T.FloatParmTemplate(
        "hub_q", "Seed Pose (deg)", 6, default_value=tuple(REST_Q), min=-270.0, max=270.0,
        help="Joints mode: the hub pose. Tool + Look: the solve picks the pose nearest this "
             "(keeps the elbow / wrist configuration)"))
    f.addParmTemplate(T.ButtonParmTemplate("keep_b", "Keep the Solved Pose as Seed",
                                           **_cb("keep_solve(kwargs['node'])")))
    f.addParmTemplate(T.IntParmTemplate("hub_clips", "Clips", 1, default_value=(8,), min=0, max=40))
    f.addParmTemplate(T.MenuParmTemplate("hub_gen", "Generator", ("choreo", "gestures"),
                                         ("Dance phrases (choreo)", "Gestures (tool + look)")))
    f.addParmTemplate(T.StringParmTemplate(
        "hub_zone", "Zone", 1, menu_type=T.menuType.StringReplace,
        item_generator_script="import show_rig; return show_rig.zone_menu()",
        item_generator_script_language=T.scriptLanguage.Python))
    f.addParmTemplate(T.StringParmTemplate("hub_families", "Gesture Families", 1, default_value=(FAMILIES,),
                                           help="Space separated: " + FAMILIES))
    g.append(f)
    node.setParmTemplateGroup(g)


def _menu(names):
    out = []
    for n in names:
        out += [n, n]
    return out


def hub_menu():
    return _menu(sorted(_objs("hub")))


def zone_menu():
    return _menu([""] + sorted(_objs("zone")))


def _box(node, rgb):
    """A unit box drawn as its bottom and top rings (the uprights crowd the
    view, as in the Isaac scene), in the object's colour."""
    if node.node("rings"):
        return
    for c in node.children():
        c.destroy()
    sop = node.createNode("python", "rings")
    sop.parm("python").set("import show_rig\nshow_rig.unit_rings(hou.pwd())\n")
    col = node.createNode("color", "colour")
    col.setInput(0, sop)
    col.parmTuple("color").set(rgb)
    col.setDisplayFlag(True)
    col.setRenderFlag(True)
    node.layoutChildren()


def unit_rings(node):
    """Python SOP of a zone object: the unit box's bottom and top rings."""
    geo = node.geometry()
    for y in (-0.5, 0.5):
        poly = geo.createPolygon(is_closed=False)
        for x, z in ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5)):
            pt = geo.createPoint()
            pt.setPosition((x, y, z))
            poly.addVertex(pt)


def _zone_obj(name):
    hou = _hou()
    node = hou.node("/obj/zone_" + name) or hou.node("/obj").createNode("geo", "zone_" + name,
                                                                        run_init_scripts=False)
    _role_parm(node, "zone")
    rgb = ZONE_RGB.get(name, (0.8, 0.8, 0.8))
    _box(node, rgb)
    node.setColor(hou.Color(rgb))
    for p in ("rx", "rz"):
        node.parm(p).lock(True)                  # zones are upright boxes: yaw only
    return node


def _null(name, kind, rgb, size):
    hou = _hou()
    node = hou.node("/obj/" + name) or hou.node("/obj").createNode("null", name)
    _role_parm(node, kind)
    node.parm("controltype").set(4 if kind == "hub" else 1)     # Null and Circles / Circles
    node.parm("geoscale").set(size)
    node.setColor(hou.Color(rgb))
    return node


def _hub_objs(name, colour):
    hub = _null("hub_" + name, "hub", colour, 0.06)
    _hub_parms(hub)
    return hub


def _look_obj(name, colour):
    return _null("look_" + name, "look", colour, 0.12)


def _viz_obj():
    hou = _hou()
    viz = hou.node("/obj/show_viz")
    if viz and viz.node("tubes"):
        return viz
    if viz:
        viz.destroy()                                # the older show panel's drawing
    viz = hou.node("/obj").createNode("geo", "show_viz", run_init_scripts=False)
    sop = viz.createNode("python", "show")
    sop.parm("python").set("import show_rig\nshow_rig.viz(hou.pwd())\n")
    wire = viz.createNode("polywire", "tubes")
    wire.setInput(0, sop)
    wire.parm("radius").set(1.0)
    wire.parm("usescaleattrib").set(1)
    wire.parm("scaleattrib").set("pscale")
    wire.parm("div").set(8)
    wire.setDisplayFlag(True)
    wire.setRenderFlag(True)
    viz.layoutChildren()
    return viz


def _layout():
    hou = _hou()
    obj = hou.node("/obj")
    nodes = [n for n in obj.children() if n.parm("show_role") or n.name() in ("SHOW", "show_viz")]
    x0 = max([n.position()[0] for n in obj.children() if n not in nodes] + [0.0]) + 4.0
    order = sorted(nodes, key=lambda n: (n.name() not in ("SHOW", "show_viz"), n.name()))
    for k, n in enumerate(order):
        n.setPosition((x0 + 3.0 * (k % 3), -1.2 * (k // 3)))
    box = obj.findNetworkBox("SHOW") or obj.createNetworkBox("SHOW")
    box.setComment("Show: zones, hubs, range (scripts/show_rig.py)")
    for n in nodes:
        box.addNode(n)
    box.fitAroundContents()


def install(config=DEFAULT_CONFIG):
    """Add the show's objects to the open scene and load the config. Run it
    again to refresh (the objects are reused); /obj/fr20 is not touched."""
    hou = _hou()
    with hou.undos.group("Install the show objects"):
        show = hou.node("/obj/SHOW") or hou.node("/obj").createNode("null", "SHOW")
        show.parm("controltype").set(0)
        show.parm("geoscale").set(0.001)
        _show_parms(show, config)
        show.parm("config").set(config)
        _viz_obj()
        load_config(show)
        _tidy_show_scene()
        _layout()
    clean_view()
    return show


def _tidy_show_scene():
    """In the show scene (it has CELL_CTRL): the room is drawn once, by
    cell_env -- the robot_arm's own Show Cell is turned off -- and the
    collision capsules are hidden (they cover the arm)."""
    hou = _hou()
    if hou.node("/obj/CELL_CTRL") is None:
        return
    arm = hou.node("/obj/fr20/robot_arm")
    if arm is not None and arm.parm("show_cell") is not None:
        arm.parm("show_cell").set(0)
    for name in ("capsules", "ghosts", "CELL_CTRL"):           # CELL_CTRL: its null's axes sit on the base
        n = hou.node("/obj/" + name)
        if n is not None:
            n.setDisplayFlag(False)
    old = hou.node("/obj/SHOW_CTRL")                # the older show panel (joint-angle hubs only)
    if old is not None:
        old.destroy()


def clean_view(viewers=None):
    """Remove Backfaces on in the scene viewers: the room's walls face in,
    so the walls near the camera vanish (the cutaway the Isaac scene has)."""
    hou = _hou()
    if not hou.isUIAvailable():
        return
    for pane in viewers or hou.ui.paneTabs():
        if pane.type() == hou.paneTabType.SceneViewer:
            for vp in pane.viewports():
                vp.settings().setRemoveBackfaces(True)


def clean_view_on_load():
    """For a scene's hou.session: clean_view once the UI is up."""
    hou = _hou()
    if not hou.isUIAvailable():
        return

    def once():
        hou.ui.removeEventLoopCallback(once)
        clean_view()
    hou.ui.addEventLoopCallback(once)


# --- config <-> scene ---------------------------------------------------------

def load_config(node=None):
    """The config file -> the zone and hub objects and SHOW's parameters.
    Show objects the config no longer names are removed."""
    hou = _hou()
    node = node or _show()
    cfg = json.load(open(cfg_path(node)))
    env = json.load(open(os.path.join(ROOT, cfg["env"])))
    zones = dict(cfg.get("zones", {}))
    stage = cfg.get("stage") or next((o for o in env["objects"] if o["name"] == "stage"), None)
    if stage:
        zones["stage"] = stage
    node.parm("stage_override").set(1 if cfg.get("stage") else 0)
    for name, z in zones.items():
        _set_xform(_zone_obj(name), *zone_to_xform(z))
    for name, n in _objs("zone").items():
        if name not in zones:
            n.destroy()

    rig = _rig()
    for k, (name, h) in enumerate(cfg["hubs"].items()):
        colour = HUB_RGB[k % len(HUB_RGB)]
        hub = _hub_objs(name, colour)
        tool = bool(h.get("tcp") and h.get("look"))
        hub.parm("hub_mode").set("tool" if tool else "joints")
        hub.parmTuple("hub_q").set(h.get("near") or h.get("q") or REST_Q)
        hub.parm("hub_clips").set(h.get("clips", 8))
        hub.parm("hub_gen").set(h.get("generator", "gestures" if tool else "choreo"))
        hub.parm("hub_zone").set(h.get("zone", ""))
        hub.parm("hub_families").set(" ".join(h.get("families", FAMILIES.split())))
        if tool:
            _unlock(hub)
            hub.parmTuple("t").set(to_h(h["tcp"]))
            _look_obj(name, colour).parmTuple("t").set(to_h(h["look"]))
        else:
            _pin_joints_hub(hub, rig)
            if hou.node("/obj/look_" + name):
                hou.node("/obj/look_" + name).destroy()
    for kind in ("hub", "look"):
        for name, n in _objs(kind).items():
            if name not in cfg["hubs"]:
                n.destroy()

    node.parm("start_hub").set(cfg.get("start_hub", next(iter(cfg["hubs"]))))
    r = cfg.get("range", {})
    node.parmTuple("j1_range").set(r.get("j1_deg", (-150.0, 150.0)))
    node.parmTuple("tcp_z").set(r.get("tcp_z", (0.35, 1.9)))
    node.parm("speed").set(r.get("speed", 0.85))
    lib = cfg["library"]
    node.parmTuple("duration").set(lib["duration_s"])
    node.parmTuple("bars").set((min(lib["bars"]), max(lib["bars"])))
    node.parmTuple("bpm").set(lib["bpm"])
    node.parmTuple("intensity").set(lib.get("intensity", (0.4, 0.9)))
    node.parm("seed").set(lib.get("seed", 1))
    sel = cfg.get("select", {})
    node.parmTuple("hub_stay").set(sel.get("hub_stay", (2, 4)))
    node.parm("no_repeat").set(sel.get("no_repeat", 6))
    if cfg["hubs"]:
        node.parm("auth_hub").set(node.evalParm("auth_hub") or next(iter(cfg["hubs"])))
    _cook_viz()


def _unlock(hub):
    for p in ("tx", "ty", "tz"):
        hub.parm(p).lock(False)


def _pin_joints_hub(hub, rig):
    """A joints hub's null sits at its pose's tool tip, locked."""
    import collision as C
    _unlock(hub)
    _, tcp = C.capsules(_cmodel(), list(hub.parmTuple("hub_q").eval()))
    hub.parmTuple("t").set(to_h(tcp))
    for p in ("tx", "ty", "tz"):
        hub.parm(p).lock(True)


def scene_parts(node=None):
    """What the scene shows, as merge_config's `scene`."""
    node = node or _show()
    zones = {}
    stage = None
    for name, n in _objs("zone").items():
        z = xform_to_zone(*_xform(n))
        if name == "stage":
            stage = z if node.evalParm("stage_override") else None
        else:
            zones[name] = z
    hubs = {}
    looks = _objs("look")
    for name, n in _objs("hub").items():
        h = {"clips": n.evalParm("hub_clips"), "generator": n.parm("hub_gen").evalAsString()}
        if n.evalParm("hub_zone"):
            h["zone"] = n.evalParm("hub_zone")
        q = _r(n.parmTuple("hub_q").eval(), 3)
        if n.parm("hub_mode").evalAsString() == "tool" and name in looks:
            h.update(tcp=_r(to_r(n.parmTuple("t").eval())), look=_r(to_r(looks[name].parmTuple("t").eval())),
                     near=q)
        else:
            h["q"] = q
        if h["generator"] == "gestures":
            h["families"] = n.evalParm("hub_families").split()
        hubs[name] = h
    b0, b1 = node.parmTuple("bars").eval()
    return {
        "zones": zones, "stage": stage, "hubs": hubs, "start_hub": node.evalParm("start_hub"),
        "range": {"j1_deg": _r(node.parmTuple("j1_range").eval(), 2), "tcp_z": _r(node.parmTuple("tcp_z").eval(), 3),
                  "speed": round(node.evalParm("speed"), 3)},
        "library": {"duration_s": _r(node.parmTuple("duration").eval(), 2), "bars": list(range(b0, b1 + 1)),
                    "bpm": list(node.parmTuple("bpm").eval()), "intensity": _r(node.parmTuple("intensity").eval(), 2),
                    "seed": node.evalParm("seed")},
        "select": {"hub_stay": list(node.parmTuple("hub_stay").eval()), "no_repeat": node.evalParm("no_repeat")},
    }


def scene_config(node=None):
    node = node or _show()
    return merge_config(json.load(open(cfg_path(node))), scene_parts(node))


def write_config(node=None):
    """The scene -> the config file. Refused (with the reasons) when the
    config would name something that is not there."""
    hou = _hou()
    node = node or _show()
    cfg = scene_config(node)
    bad = check_config(cfg)
    if bad:
        _report(node, "Not written:\n  " + "\n  ".join(bad), error=True)
        return None
    with open(cfg_path(node), "w", newline="\n") as f:
        json.dump(cfg, f, indent=1)
    _report(node, "Wrote %s" % os.path.relpath(cfg_path(node), ROOT))
    hou.node("/obj/show_viz/show").cook(force=True)
    return cfg


def _report(node, text, error=False):
    hou = _hou()
    node.parm("report").set(text[-6000:])
    if error and hou.isUIAvailable():
        hou.ui.displayMessage(text.splitlines()[0], details=text, severity=hou.severityType.Error)


# --- buttons ------------------------------------------------------------------

def add_hub(node=None):
    """A new Tool + Look hub, in front of the greet zone (or the robot)."""
    hou = _hou()
    node = node or _show()
    name = node.evalParm("new_name").strip()
    if not name or hou.node("/obj/hub_" + name):
        _report(node, "Pick a new, unused name for the hub (New Name)", error=True)
        return
    k = len(_objs("hub"))
    colour = HUB_RGB[k % len(HUB_RGB)]
    with hou.undos.group("Add hub " + name):
        hub = _hub_objs(name, colour)
        hub.parm("hub_mode").set("tool")
        hub.parm("hub_gen").set("gestures")
        hub.parmTuple("hub_q").set((64.0, -100.0, 105.0, -193.0, -88.0, 4.0))
        hub.parm("hub_zone").set("greet" if "greet" in _objs("zone") else "")
        hub.parmTuple("t").set(to_h((0.0, -0.6, 1.2)))
        _look_obj(name, colour).parmTuple("t").set(to_h((-0.3, -1.8, 1.5)))
        _layout()
    hub.setSelected(True, clear_all_selected=True)
    _cook_viz()


def add_zone(node=None):
    hou = _hou()
    node = node or _show()
    name = node.evalParm("new_name").strip()
    if not name or hou.node("/obj/zone_" + name):
        _report(node, "Pick a new, unused name for the zone (New Name)", error=True)
        return
    with hou.undos.group("Add zone " + name):
        z = _zone_obj(name)
        _set_xform(z, *zone_to_xform({"center": (-0.3, 0.0, 1.0), "size": (0.6, 0.6, 0.6), "yaw_deg": -11.28}))
        _layout()
    z.setSelected(True, clear_all_selected=True)


def hub_mode_changed(hub):
    """Switching a hub to Tool + Look puts the tool tip and a look target
    where its joints pose has them; to Joints pins it at its seed pose."""
    hou = _hou()
    name = hub.name()[4:]
    rig = _rig()
    if hub.parm("hub_mode").evalAsString() == "tool":
        import gestures as G
        tcp, look, _ = G.home_of(rig, list(hub.parmTuple("hub_q").eval()))
        _unlock(hub)
        hub.parmTuple("t").set(to_h(tcp))
        _look_obj(name, hub.color().rgb()).parmTuple("t").set(to_h(look))
    else:
        _pin_joints_hub(hub, rig)
        if hou.node("/obj/look_" + name):
            hou.node("/obj/look_" + name).destroy()
    _cook_viz()


def keep_solve(hub):
    """The solved pose becomes the seed: later drags stay near it."""
    q = _solve_hub(hub)
    if q is None:
        _report(_show(), "hub %s: no pose to keep (not reachable)" % hub.name()[4:], error=True)
        return
    hub.parmTuple("hub_q").set(_r(q, 3))


def add_clip(node=None):
    """The robot_arm's exported joint CSV -> the config's authored clips."""
    node = node or _show()
    csv = node.evalParm("auth_csv")
    if not os.path.exists(csv):
        _report(node, "No joint CSV at %s -- export the clip from the robot_arm first" % csv, error=True)
        return
    cfg = json.load(open(cfg_path(node)))
    clip_id = node.evalParm("auth_id").strip() or os.path.splitext(os.path.basename(csv))[0]
    cfg["authored"] = add_authored(cfg.get("authored", []), csv, node.evalParm("auth_hub"), clip_id)
    with open(cfg_path(node), "w", newline="\n") as f:
        json.dump(cfg, f, indent=1)
    _report(node, "Added %s (at hub %s) to %s -- Build Show makes its moves in and out and checks it"
            % (clip_id, node.evalParm("auth_hub"), os.path.basename(cfg_path(node))))


def segment_menu(node=None):
    """(token, label) of the built segments, from the preview manifest."""
    node = node or _show()
    man = preview_dir(node) + "/manifest.json"
    items = []
    if os.path.exists(man):
        for s in json.load(open(man))["segments"]:
            items += [s["name"], "%s  (%s, %.1f s)" % (s["name"], s["kind"], s["duration_s"])]
    return items or ["", "(build the show first)"]


def _play_on_arm(path):
    hou = _hou()
    ctrl = hou.node("/obj/CELL_CTRL")
    if ctrl is None:
        _report(_show(), "Playing on the arm needs the show scene (CELL_CTRL)", error=True)
        return
    import cell_sop
    ctrl.parm("clip").set(path)
    cell_sop.fit_range()


def preview_segment(node=None):
    node = node or _show()
    name = node.evalParm("segment")
    if name:
        _play_on_arm(preview_dir(node) + "/" + name + ".json")


def preview_hub(node=None):
    """The arm held at a hub's pose (a two-point still clip)."""
    node = node or _show()
    name = node.evalParm("pose_hub")
    h = scene_parts(node)["hubs"].get(name)
    q = h and _hub_q(h)
    if q is None:
        _report(node, "hub %s: no pose to show" % name, error=True)
        return
    os.makedirs(preview_dir(node), exist_ok=True)
    path = preview_dir(node) + "/_hub_%s.json" % name
    with open(path, "w") as f:
        json.dump({"id": "hub", "points": [{"t": 0.0, "q": list(q)}, {"t": 1.0, "q": list(q)}]}, f)
    _play_on_arm(path)


def _run(args, title):
    hou = _hou()
    with hou.InterruptableOperation(title, open_interrupt_dialog=True):
        r = subprocess.run([PYTHON, ROOT + "/scripts/show.py"] + args, capture_output=True, text=True, cwd=ROOT)
    return r.returncode, (r.stdout + r.stderr).strip()


def build_show(node=None):
    """Write the config, build the show (every clip made and checked), redraw."""
    node = node or _show()
    if write_config(node) is None:
        return 1, ""
    code, out = _run(["build", cfg_path(node)], "Building the show (clips made and checked)")
    _report(node, out, error=bool(code))
    _cook_viz()
    return code, out


def dry_run(node=None):
    node = node or _show()
    seqs = list(json.load(open(cfg_path(node))).get("sequences", {}))
    code, out = _run(["dry-run", cfg_path(node), "--minutes", str(node.evalParm("dry_minutes")),
                      "--trigger-every", "40", "--triggers", ",".join(["scan"] + seqs)], "Dry run of the show")
    _report(node, out, error=bool(code))
    return code, out


def check(node=None):
    """The hubs solved and checked, and the config's names: into the report."""
    node = node or _show()
    cfg = scene_config(node)
    lines = ["config: " + ("; ".join(check_config(cfg)) or "names consistent")]
    for name, h in cfg["hubs"].items():
        q, why = hub_status(cfg, h)
        lines.append("hub %-8s %s" % (name, why or "clear, reachable, in range: " + " ".join("%.1f" % x for x in q)))
    _report(node, "\n".join(lines))


# --- solving and drawing ------------------------------------------------------

_CACHE = {}


def _rig():
    if "rig" not in _CACHE:
        import gestures as G
        _CACHE["rig"] = G.Rig()
    return _CACHE["rig"]


def _cmodel():
    """The collision model (capsules), not the IK one Rig carries."""
    if "cmodel" not in _CACHE:
        import collision as C
        _CACHE["cmodel"] = C.load_model("fr20")
    return _CACHE["cmodel"]


def _hub_q(h):
    """A hub's pose: its joints, or solved from tool + look (cached)."""
    if not (h.get("tcp") and h.get("look")):
        return list(h["q"])
    import gestures as G
    key = ("pose", tuple(h["tcp"]), tuple(h["look"]), tuple(h["near"]))
    if key not in _CACHE:
        _CACHE[key] = G.hub_pose(_rig(), h["tcp"], h["look"], h["near"])
    return _CACHE[key]


def _solve_hub(hub):
    h = scene_parts()["hubs"][hub.name()[4:]]
    return _hub_q(h)


def _move_env(cfg):
    import collision as C
    import safe_move
    import show as S
    key = ("env", json.dumps([cfg["env"], cfg.get("stage"), cfg.get("canvas"), cfg["margins"]["idle_canvas_m"]]))
    if key not in _CACHE:
        env = C.load_env(os.path.join(ROOT, cfg["env"]))
        _CACHE[key] = safe_move.move_env(S.show_env(env, cfg, cfg["margins"]["idle_canvas_m"]))
    return _CACHE[key]


def hub_status(cfg, h):
    """(q, why not usable or None) of a hub."""
    import collision as C
    import safe_move
    import show as S
    q = _hub_q(h)
    if q is None:
        return None, "no pose puts the tool there looking at the target (out of reach, or only near the " \
                     "wrist singularity)"
    blocked = safe_move.blocked(_cmodel(), _move_env(cfg), q)
    if blocked:
        return q, "not clear: %s near %s" % blocked[:2]
    _, tcp = C.capsules(_cmodel(), q)
    out = S.out_of_range(cfg, [q], [tcp])
    return q, out and "out of range: " + out


def viz(node):
    """The Python SOP of show_viz: hubs as ghost arms (opaque: the viewport
    does not draw a point Alpha here), look rays, the
    operating range, the built tool paths. Warns for a hub that cannot be used."""
    hou = _hou()
    import collision as C
    geo = node.geometry()
    geo.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Point, "pscale", 0.004)
    geo.addAttrib(hou.attribType.Prim, "name", "")
    show = _show()
    if show is None:
        return
    cfg = scene_config(show)

    def line(pts, cd, name, width=0.004):
        poly = geo.createPolygon(is_closed=False)
        for p in pts:
            pt = geo.createPoint()
            pt.setPosition(to_h(p))
            pt.setAttribValue("Cd", cd)
            pt.setAttribValue("pscale", width)
            poly.addVertex(pt)
        poly.setAttribValue("name", name)

    r = cfg["range"]
    for pl in range_guide(r["j1_deg"][0], r["j1_deg"][1], r["tcp_z"][0], r["tcp_z"][1],
                          show.evalParm("guide_radius")):
        line(pl, (0.75, 0.75, 0.8), "range", 0.003)

    colour, bad = {}, []
    for k, (name, h) in enumerate(cfg["hubs"].items()):
        n = hou.node("/obj/hub_" + name)
        colour[name] = n.color().rgb() if n else HUB_RGB[k % len(HUB_RGB)]
        q, why = hub_status(cfg, h)
        cd = BAD_RGB if why else colour[name]
        if why:
            bad.append("hub %s: %s" % (name, why))
        if q is not None:
            caps, tcp = C.capsules(_cmodel(), q)
            for link, a, b, rad in caps:
                line([a, b], cd, "hub_%s/%s" % (name, link), rad * GHOST_SCALE)
            if h.get("look"):
                line([tcp, h["look"]], cd, "look_" + name, 0.004)
        elif h.get("look"):
            line([h["tcp"], h["look"]], BAD_RGB, "look_" + name, 0.004)

    man = preview_dir(show) + "/manifest.json"
    if show.evalParm("show_paths") and os.path.exists(man):
        for s in json.load(open(man))["segments"]:
            f = preview_dir(show) + "/" + s["name"] + ".json"
            if not os.path.exists(f):
                continue
            tcp = json.load(open(f)).get("tcp") or []
            cd = colour.get(s["start"], (0.8, 0.8, 0.8)) if s["kind"] == "idle" else \
                (0.9, 0.9, 0.9) if s["kind"] == "move" else (1.0, 0.85, 0.2)
            if len(tcp) > 1:
                line(tcp[::2] + tcp[-1:], cd, s["name"], 0.003)
    if bad:
        raise hou.NodeWarning("\n".join(bad))


def _cook_viz():
    hou = _hou()
    sop = hou.node("/obj/show_viz/show")
    if sop:
        try:
            sop.cook(force=True)
        except hou.OperationFailed:
            pass                                   # its warning / error shows on the node


# --------------------------------------------------------------------------

def self_test():
    fails = []

    def check_(label, ok, detail=""):
        print("%s  %s %s" % ("ok  " if ok else "FAIL", label, detail))
        if not ok:
            fails.append(label)

    p = (0.3, -1.2, 0.9)
    check_("to_h / to_r invert", all(abs(a - b) < 1e-12 for a, b in zip(to_r(to_h(p)), p)))
    check_("robot Z up is Houdini Y up", to_h((0, 0, 1)) == (0, 1, 0))
    z = {"center": [-0.2606, -0.5395, 1.2], "size": [1.3, 0.5, 0.7], "yaw_deg": -11.28}
    back = xform_to_zone(*zone_to_xform(z))
    check_("zone -> object -> zone", back == {"center": z["center"], "size": z["size"], "yaw_deg": -11.28}, back)
    t, r, s = zone_to_xform(z)
    check_("uniform scale folds into the size", xform_to_zone(t, r, s, 2.0)["size"] == [2.6, 1.0, 1.4])
    check_("yaw wraps to -180..180", xform_to_zone(t, (0, 350.0, 0), s)["yaw_deg"] == -10.0)
    # a yaw about robot Z equals the same angle about Houdini Y
    th = math.radians(30.0)
    v = (1.0, 0.0, 0.0)
    rob = (math.cos(th) * v[0] - math.sin(th) * v[1], math.sin(th) * v[0] + math.cos(th) * v[1], 0.0)
    H = to_h(v)
    hou_rot = (H[0] * math.cos(th) + H[2] * math.sin(th), H[1], -H[0] * math.sin(th) + H[2] * math.cos(th))
    check_("yaw is the same angle in both frames", all(abs(a - b) < 1e-12 for a, b in zip(to_h(rob), hou_rot)))

    g = range_guide(-90.0, 90.0, 0.4, 1.8, 1.0)
    a0 = math.degrees(math.atan2(g[1][0][1], g[1][0][0]))
    a1 = math.degrees(math.atan2(g[1][-1][1], g[1][-1][0]))
    check_("range guide spans J1 + 180", abs(_wrap(a0 - 90.0)) < 1e-9 and abs(_wrap(a1 + 90.0)) < 1e-9, (a0, a1))
    check_("range guide band heights", g[1][0][2] == 0.4 and g[2][0][2] == 1.8)

    cfg = {"env": "e.json", "osc": {"listen_port": 9000}, "sequences": {"greet": {"hub": "greet"}},
           "hubs": {"rest": {"q": REST_Q, "note": "home"},
                    "greet": {"tcp": [0, -0.6, 1.2], "look": [0, -2, 1.5], "near": REST_Q, "clips": 10}},
           "zones": {"greet": {"center": [0, 0, 0], "size": [1, 1, 1], "note": "for them"}},
           "range": {"j1_deg": [-150, 150], "_note": "n"}, "library": {"seed": 1, "tries": 6}, "start_hub": "rest"}
    scene = {"zones": {"greet": {"center": [1, 0, 0], "size": [1, 1, 1], "yaw_deg": 0.0}},
             "stage": None, "start_hub": "rest",
             "hubs": {"rest": {"q": [0.0] * 6, "clips": 4},
                      "greet": {"q": REST_Q, "clips": 2}},
             "range": {"j1_deg": [-90, 90], "tcp_z": [0.4, 1.8], "speed": 0.5},
             "library": {"seed": 3}, "select": {"hub_stay": [1, 2]}}
    m = merge_config(cfg, scene)
    check_("merge keeps keys the scene does not show", m["osc"] == cfg["osc"] and m["sequences"] == cfg["sequences"])
    check_("merge keeps a hub's note", m["hubs"]["rest"]["note"] == "home")
    check_("merge keeps a zone's note", m["zones"]["greet"]["note"] == "for them")
    check_("a hub switched to joints loses tcp / look / near",
           not any(k in m["hubs"]["greet"] for k in ("tcp", "look", "near")))
    check_("merge keeps other library / range keys", m["library"]["tries"] == 6 and m["range"]["_note"] == "n")
    check_("merge does not change its input", cfg["hubs"]["rest"]["q"] == REST_Q)

    check_("a consistent config passes", check_config(m) == [], check_config(m))
    m2 = json.loads(json.dumps(m))
    del m2["hubs"]["greet"]
    check_("a sequence at a removed hub is caught", any("sequence greet" in x for x in check_config(m2)))
    m3 = json.loads(json.dumps(m))
    m3["range"]["tcp_z"] = [1.0, 0.5]
    check_("an upside-down range is caught", any("tcp_z" in x for x in check_config(m3)))
    m4 = json.loads(json.dumps(m))
    m4["hubs"]["rest"]["generator"] = "gestures"
    check_("gestures without tool + look are caught", any("gestures" in x for x in check_config(m4)))

    a = add_authored([{"id": "cat", "hub": "rest", "csv": "x.csv"}], ROOT + "/tests/csv/cat.csv", "greet", "cat")
    check_("authored: same id replaced, path made relative",
           a == [{"id": "cat", "hub": "greet", "csv": "tests/csv/cat.csv"}], a)
    print("%d failed" % len(fails) if fails else "all passed")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    print(__doc__)
