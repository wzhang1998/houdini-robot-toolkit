"""The show as one Houdini tool: /obj/robot_show, a Geometry whose parameter
page holds everything -- robot profile, environment, show config, zones,
hubs, operating range, library, authored clip, build / dry run / preview,
display -- written back to the show config (shows/*.json).

    import show_rig; show_rig.install()                    # shows/party.json
    hython scripts/build_show_scene.py                     # scenes/FR20_show.hiplc

Inside the node (Python gives data only; the shapes are Houdini's nodes):

    robot_arm      the wenyi::robot_arm asset, FK from the preview clip
                   (Arm Plays), drawing the room from Environment
    zones          a Box drawn by Convert Line, copied onto one data point
                   per zone (centre, yaw, size)
    hub ghosts     the arm's own link meshes posed per hub (For-Each,
                   Transform Pieces), in the hub's colour; red when the hub
                   cannot be reached, is not clear, or is out of range
    rays, paths    look rays and the built clips' tool paths, by PolyWire
    range          the J1 sector and TCP height band, Circle SOPs on the
                   node's parameters
    each branch has a Display toggle; OUT merges them packed

Zones and hubs are multiparms, in the robot base frame (Z up, metres: the
numbers of the WebUI and the config). Edit in Viewport puts handles on the
chosen zone (move, turn, scale) or hub (tool tip, look target); they write
the parameters. Check / Build / Dry Run run show.py on the written config.

Frames: Houdini is Y up: (x, y, z) -> (x, z, -y); a yaw about the robot's Z
is the same angle about Houdini's Y.

    python scripts/show_rig.py --self-test                 # the parts without hou
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
GHOST_ALPHA = 0.3                  # a hub's ghost: the robot's own meshes, see-through, in the hub's colour
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


def zone_instance(z):
    """A zone {center, size, yaw_deg} -> (P, orient, scale) of the point a
    unit Box is copied onto, in Houdini: orient turns by the yaw about +Y
    (the same angle as about the robot's Z), scale is the size (Y up)."""
    s, half = z["size"], math.radians(z.get("yaw_deg", 0.0)) / 2.0
    return tuple(to_h(z["center"])), (0.0, math.sin(half), 0.0, math.cos(half)), (s[0], s[2], s[1])


def zx_arc_angle(j1_deg):
    """The Circle SOP angle (orientation ZX) that points where the arm faces
    at J1 = j1_deg. In ZX an angle a points along Houdini (cos a, 0, sin a);
    a robot-frame direction phi is Houdini (cos phi, 0, -sin phi); the arm
    faces J1 + J1_FACING_DEG. show_viz's range arcs use this in expressions."""
    return -(j1_deg + J1_FACING_DEG)


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
# the tool (hou)
# --------------------------------------------------------------------------

TOOL = "robot_show"


def _hou():
    import hou
    return hou


def tool(node=None):
    """The show tool: the node given, or its parent (a SOP inside it), or
    /obj/robot_show."""
    hou = _hou()
    for n in (node, node and node.parent()):
        if n is not None and n.parm("config") is not None and n.parm("zones") is not None:
            return n
    return hou.node("/obj/" + TOOL)


def cfg_path(node=None):
    return tool(node).evalParm("config").replace("\\", "/")


def env_path(node=None):
    return tool(node).evalParm("env_file").replace("\\", "/")


def show_name(node=None):
    return os.path.splitext(os.path.basename(cfg_path(node)))[0]


def preview_dir(node=None):
    return ROOT + "/geo/show/" + show_name(node)


def _rel(path):
    p = path.replace("\\", "/")
    return p[len(ROOT) + 1:] if p.lower().startswith(ROOT.lower() + "/") else p


def _cb(code):
    hou = _hou()
    return {"script_callback": "import show_rig; show_rig." + code,
            "script_callback_language": hou.scriptLanguage.Python}


def _menu_script(fn):
    """A menu's item generator. Houdini runs a one-line menu script as an
    expression (no statements, no return): it must be more than one line."""
    return "import show_rig\nreturn show_rig.%s(kwargs['node'])" % fn


def _menu(names):
    out = []
    for n in names:
        out += [n, n]
    return out


def hub_menu(node=None):
    return _menu(hub_names(node))


def zone_menu(node=None):
    return ["", "(none)"] + _menu(sorted(zone_names(node)))   # an empty label drops the whole menu


def profile_menu(node=None):
    names = sorted(os.path.splitext(f)[0] for f in os.listdir(ROOT + "/profiles") if f.endswith(".json"))
    return _menu(names)


def segment_menu(node=None):
    """(token, label) of the built segments, from the preview manifest."""
    man = preview_dir(node) + "/manifest.json"
    items = []
    if os.path.exists(man):
        for s in json.load(open(man))["segments"]:
            items += [s["name"], "%s  (%s, %.1f s)" % (s["name"], s["kind"], s["duration_s"])]
    return items or ["", "(build the show first)"]


def edit_menu(node=None):
    """What the viewport handles edit: a zone or a hub."""
    items = []
    for n in zone_names(node):
        items += ["zone:" + n, "Zone  " + n]
    for n in hub_names(node):
        items += ["hub:" + n, "Hub  " + n]
    return items or ["", "(nothing to edit)"]


def zone_names(node=None):
    t = tool(node)
    return [t.evalParm("zone_name%d" % i) for i in range(1, t.evalParm("zones") + 1)]


def hub_names(node=None):
    t = tool(node)
    return [t.evalParm("hub_name%d" % i) for i in range(1, t.evalParm("hubs") + 1)]


# --- the parameters -----------------------------------------------------------

def _parms(node, config, env):
    """Every control of the tool, in tabs, on the one node."""
    hou = _hou()
    T = hou
    g = node.parmTemplateGroup()
    if g.find("config") is not None:
        return

    setup = T.FolderParmTemplate("setup_f", "Setup", folder_type=T.folderType.Tabs)
    setup.addParmTemplate(T.StringParmTemplate(
        "robot_profile", "Robot Profile", 1, default_value=("fr20",), menu_type=T.menuType.Normal,
        item_generator_script=_menu_script("profile_menu"),
        item_generator_script_language=T.scriptLanguage.Python,
        help="profiles/<name>.json: the arm drawn and checked. The show generator is made for the FR20",
        **_cb("profile_changed(kwargs['node'])")))
    setup.addParmTemplate(T.StringParmTemplate(
        "env_file", "Environment", 1, default_value=(env,), string_type=T.stringParmType.FileReference,
        file_type=T.fileType.Any, tags={"filechooser_pattern": "*.json"},
        help="envs/<room>.json: the measured room the clips are checked against"))
    setup.addParmTemplate(T.StringParmTemplate(
        "config", "Show Config", 1, default_value=(config,), string_type=T.stringParmType.FileReference,
        file_type=T.fileType.Any, tags={"filechooser_pattern": "*.json"},
        help="shows/<name>.json: the show this node edits"))
    setup.addParmTemplate(T.ButtonParmTemplate("load_b", "Load Config", join_with_next=True,
                                               help="The config file -> these parameters", **_cb("load_config(kwargs['node'])")))
    setup.addParmTemplate(T.ButtonParmTemplate("write_b", "Write Config", join_with_next=True,
                                               help="These parameters -> the config file (other keys kept)",
                                               **_cb("write_config(kwargs['node'])")))
    setup.addParmTemplate(T.ButtonParmTemplate("check_b", "Check", help="Hubs reachable, clear, in range; names consistent",
                                               **_cb("check(kwargs['node'])")))
    setup.addParmTemplate(T.SeparatorParmTemplate("setup_sep"))
    setup.addParmTemplate(T.StringParmTemplate(
        "edit_item", "Edit in Viewport", 1, menu_type=T.menuType.Normal,
        item_generator_script=_menu_script("edit_menu"), item_generator_script_language=T.scriptLanguage.Python,
        help="The zone or hub the viewport handles move", join_with_next=True))
    setup.addParmTemplate(T.ButtonParmTemplate("edit_b", "Handles On", help="Viewport handles for the item above "
                                               "(Esc or another tool leaves them)", **_cb("edit_in_viewport(kwargs['node'])")))
    g.append(setup)

    zones = T.FolderParmTemplate("zones_f", "Zones", folder_type=T.folderType.Tabs)
    zones.addParmTemplate(T.ToggleParmTemplate(
        "stage_override", "Use Zone 'stage' as the Stage", default_value=False,
        help="Off: the stage (work zone) of the environment file. On: the zone named 'stage' here"))
    zl = T.FolderParmTemplate("zones", "Zones", folder_type=T.folderType.MultiparmBlock)
    zl.addParmTemplate(T.StringParmTemplate("zone_name#", "Name", 1))
    zl.addParmTemplate(T.FloatParmTemplate("zone_center#", "Centre (robot frame, m)", 3,
                                           help="Robot base frame, Z up, metres: the numbers of the WebUI / config"))
    zl.addParmTemplate(T.FloatParmTemplate("zone_size#", "Size (m)", 3, default_value=(0.6, 0.6, 0.6), min=0.01, max=5.0))
    zl.addParmTemplate(T.FloatParmTemplate("zone_yaw#", "Yaw (deg)", 1, min=-180.0, max=180.0))
    zones.addParmTemplate(zl)
    g.append(zones)

    hubs = T.FolderParmTemplate("hubs_f", "Hubs", folder_type=T.folderType.Tabs)
    hubs.addParmTemplate(T.StringParmTemplate(
        "start_hub", "Start Hub", 1, menu_type=T.menuType.Normal,
        item_generator_script=_menu_script("hub_menu"), item_generator_script_language=T.scriptLanguage.Python))
    hl = T.FolderParmTemplate("hubs", "Hubs", folder_type=T.folderType.MultiparmBlock)
    hl.addParmTemplate(T.StringParmTemplate("hub_name#", "Name", 1))
    hl.addParmTemplate(T.MenuParmTemplate(
        "hub_mode#", "Mode", ("tool", "joints"), ("Tool Tip + Look At", "Joint Angles"),
        help="Tool Tip + Look At: the pose is solved nearest the Seed Pose. Joint Angles: the Seed Pose is the pose",
        **_cb("hub_mode_changed(kwargs)")))
    hl.addParmTemplate(T.FloatParmTemplate("hub_tcp#", "Tool Tip (robot frame, m)", 3,
                                           disable_when="{ hub_mode# == joints }"))
    hl.addParmTemplate(T.FloatParmTemplate("hub_look#", "Look At (robot frame, m)", 3,
                                           disable_when="{ hub_mode# == joints }"))
    hl.addParmTemplate(T.FloatParmTemplate(
        "hub_q#", "Seed Pose (deg)", 6, default_value=tuple(REST_Q), min=-270.0, max=270.0,
        help="Joint Angles: the hub pose. Tool Tip + Look At: the solve picks the pose nearest this "
             "(keeps the elbow / wrist configuration)"))
    hl.addParmTemplate(T.ButtonParmTemplate("hub_keep#", "Keep the Solved Pose as Seed",
                                            disable_when="{ hub_mode# == joints }", **_cb("keep_solve(kwargs)")))
    hl.addParmTemplate(T.IntParmTemplate("hub_clips#", "Clips", 1, default_value=(8,), min=0, max=40))
    hl.addParmTemplate(T.MenuParmTemplate("hub_gen#", "Generator", ("choreo", "gestures"),
                                          ("Dance Phrases", "Gestures (needs Tool Tip + Look At)")))
    hl.addParmTemplate(T.StringParmTemplate(
        "hub_zone#", "Zone", 1, menu_type=T.menuType.Normal,
        item_generator_script=_menu_script("zone_menu"), item_generator_script_language=T.scriptLanguage.Python))
    hl.addParmTemplate(T.StringParmTemplate("hub_families#", "Gesture Families", 1, default_value=(FAMILIES,),
                                            disable_when="{ hub_gen# == choreo }", help="Space separated: " + FAMILIES))
    hubs.addParmTemplate(hl)
    g.append(hubs)

    rng = T.FolderParmTemplate("range_f", "Operating Range", folder_type=T.folderType.Tabs)
    rng.addParmTemplate(T.FloatParmTemplate(
        "j1_range", "J1 Range (deg)", 2, default_value=(-150.0, 150.0), min=-175.0, max=175.0,
        help="The directions the arm may face. A clip whose J1 leaves it is dropped"))
    rng.addParmTemplate(T.FloatParmTemplate("tcp_z", "Tool Tip Height (m)", 2, default_value=(0.35, 1.9),
                                            min=0.0, max=2.5, help="The band the tool tip stays in, above the base"))
    rng.addParmTemplate(T.FloatParmTemplate(
        "speed", "Speed (of the limits)", 1, default_value=(0.85,), min=0.05, max=1.0,
        help="Clips are made at this fraction of the robot's measured velocity / acceleration limits"))
    rng.addParmTemplate(T.FloatParmTemplate("guide_radius", "Guide Radius (m)", 1, default_value=(1.2,),
                                            min=0.3, max=2.0, help="Only how the range is drawn"))
    g.append(rng)

    lib = T.FolderParmTemplate("lib_f", "Library", folder_type=T.folderType.Tabs)
    lib.addParmTemplate(T.FloatParmTemplate("duration", "Clip Length (s)", 2, default_value=(4.0, 12.0),
                                            min=1.0, max=30.0))
    lib.addParmTemplate(T.IntParmTemplate("bars", "Bars", 2, default_value=(1, 2), min=1, max=4))
    lib.addParmTemplate(T.IntParmTemplate("bpm", "BPM", 2, default_value=(80, 125), min=40, max=180))
    lib.addParmTemplate(T.FloatParmTemplate("intensity", "Intensity", 2, default_value=(0.4, 0.9), min=0.0, max=1.0,
                                            help="How big and quick the gestures are (each clip draws one)"))
    lib.addParmTemplate(T.IntParmTemplate("seed", "Seed", 1, default_value=(11,)))
    lib.addParmTemplate(T.IntParmTemplate("hub_stay", "Clips per Hub Visit", 2, default_value=(2, 4), min=1, max=20))
    lib.addParmTemplate(T.IntParmTemplate("no_repeat", "No Repeat Within", 1, default_value=(6,), min=0, max=40))
    lib.addParmTemplate(T.SeparatorParmTemplate("lib_sep"))
    lib.addParmTemplate(T.LabelParmTemplate(
        "auth_note", "Authored Clip", column_labels=("A curve drawn on a robot_arm (the rig scene), exported as a "
                                                     "joint CSV, played from a hub and back.",)))
    lib.addParmTemplate(T.StringParmTemplate(
        "auth_csv", "Joint CSV", 1, string_type=T.stringParmType.FileReference, file_type=T.fileType.Any,
        tags={"filechooser_pattern": "*.csv"}, help="The robot_arm's Export CSV (Output tab)"))
    lib.addParmTemplate(T.StringParmTemplate(
        "auth_hub", "Play at Hub", 1, menu_type=T.menuType.Normal,
        item_generator_script=_menu_script("hub_menu"), item_generator_script_language=T.scriptLanguage.Python))
    lib.addParmTemplate(T.StringParmTemplate("auth_id", "Clip Id", 1, default_value=("cat",)))
    lib.addParmTemplate(T.ButtonParmTemplate("auth_b", "Add Clip to the Library", **_cb("add_clip(kwargs['node'])")))
    g.append(lib)

    run = T.FolderParmTemplate("run_f", "Build and Preview", folder_type=T.folderType.Tabs)
    run.addParmTemplate(T.ButtonParmTemplate("build_b", "Build Show", join_with_next=True,
                                             help="Write, then make and check every clip (a minute or more)",
                                             **_cb("build_show(kwargs['node'])")))
    run.addParmTemplate(T.ButtonParmTemplate("dry_b", "Dry Run", join_with_next=True, **_cb("dry_run(kwargs['node'])")))
    run.addParmTemplate(T.FloatParmTemplate("dry_minutes", "Minutes", 1, default_value=(10.0,), min=1.0, max=120.0))
    run.addParmTemplate(T.StringParmTemplate(
        "segment", "Play Segment on the Arm", 1, menu_type=T.menuType.Normal,
        item_generator_script=_menu_script("segment_menu"), item_generator_script_language=T.scriptLanguage.Python,
        **_cb("preview_segment(kwargs['node'])")))
    run.addParmTemplate(T.StringParmTemplate(
        "pose_hub", "Hold the Arm at Hub", 1, menu_type=T.menuType.Normal,
        item_generator_script=_menu_script("hub_menu"), item_generator_script_language=T.scriptLanguage.Python,
        **_cb("preview_hub(kwargs['node'])")))
    run.addParmTemplate(T.StringParmTemplate(
        "preview_clip", "Arm Plays", 1, string_type=T.stringParmType.FileReference, file_type=T.fileType.Any,
        help="The clip on the arm (a segment, a hub, or any joint CSV / clip JSON)"))
    run.addParmTemplate(T.ButtonParmTemplate("fit_b", "Fit the Timeline to It", **_cb("fit_range(kwargs['node'])")))
    run.addParmTemplate(T.StringParmTemplate("report", "Report", 1, tags={"editor": "1", "editorlines": "10-30"}))
    g.append(run)

    disp = T.FolderParmTemplate("display_f", "Display", folder_type=T.folderType.Tabs)
    for name, label, on in (("show_robot", "Robot", True), ("show_room", "Room", True), ("show_zones", "Zones", True),
                            ("show_ghosts", "Hub Ghosts", True), ("show_rays", "Look Rays", True),
                            ("show_paths", "Built Tool Paths", True), ("show_range", "Operating Range", True)):
        disp.addParmTemplate(T.ToggleParmTemplate(name, label, default_value=on))
    g.append(disp)
    node.setParmTemplateGroup(g)


# --- the network inside -------------------------------------------------------

def _py(parent, name, fn):
    sop = parent.createNode("python", name)
    sop.parm("python").set("import show_rig\nshow_rig.%s(hou.pwd())\n" % fn)
    return sop


def _switch(parent, name, source, toggle):
    """A branch shown when the tool's toggle is on, packed so the final merge
    sees no attribute mismatch between branches."""
    pack = parent.createNode("pack", name + "_packed")
    pack.setInput(0, source)
    sw = parent.createNode("switch", name)
    sw.setInput(0, parent.createNode("null", name + "_off"))
    sw.setInput(1, pack)
    sw.parm("input").setExpression('ch("../%s")' % toggle)
    return sw


def _tube(parent, name, source):
    wire = parent.createNode("polywire", name)
    wire.setInput(0, source)
    wire.parm("radius").set(1.0)
    wire.parm("usescaleattrib").set(1)
    wire.parm("scaleattrib").set("pscale")
    wire.parm("div").set(8)
    return wire


def _network(node):
    """Everything inside the tool. Python SOPs give data only (points and
    lines with attributes); the shapes are Houdini's nodes."""
    hou = _hou()
    for c in node.children():
        c.destroy()

    # the robot (and, through it, the room), playing the preview clip
    arm = node.createNode("wenyi::robot_arm::1.0", "robot_arm")
    arm.parm("robot_profile").set(node.evalParm("robot_profile"))
    arm.hdaModule().on_profile_changed(arm)
    arm.parm("pose_source").set(0)                               # FK
    for j in range(1, 7):
        arm.parm("fk_j%d" % j).setExpression("__import__('show_rig').joint(hou.pwd(), %d)" % j,
                                             hou.exprLanguage.Python)
    arm.parm("env_file").set('`chs("../env_file")`')
    arm.parm("show_cell").setExpression('ch("../show_room")')
    arm.parm("show_robot").setExpression('ch("../show_robot")')     # the robot and the room: the arm's own toggles

    # zones: a Box, drawn as its edges, copied onto one data point per zone
    zpts = _py(node, "zone_points", "zone_points")
    box = node.createNode("box", "unit_box")
    edges = node.createNode("convertline", "box_edges")
    edges.setInput(0, box)
    edges.parm("computelength").set(0)
    zones = node.createNode("copytopoints::2.0", "zones")
    zones.setInput(0, edges)
    zones.setInput(1, zpts)
    zones.parm("targetattribs").set(1)
    zones.parm("applyto1").set(0)
    zones.parm("applymethod1").set(0)
    zones.parm("applyattribs1").set("Cd")

    # hub ghosts: the arm's own link meshes, posed per hub by Transform Pieces
    poses = _py(node, "hub_poses", "hub_poses")
    links = node.createNode("object_merge", "robot_links")
    links.parm("objpath1").set("../robot_arm/urdf_normals")
    moving = node.createNode("blast", "moving_links")            # the base does not move: the arm's own base shows
    moving.setInput(0, links)
    moving.parm("group").set("@name=base")
    moving.parm("grouptype").set("prims")
    rest = node.createNode("object_merge", "rest_skeleton")
    rest.parm("objpath1").set("../robot_arm/urdf_skeleton")
    begin = node.createNode("block_begin", "each_hub")
    begin.setInput(0, poses)
    begin.parm("method").set("piece")
    begin.parm("blockpath").set("../each_hub_end")
    pose = node.createNode("xformpieces", "pose_links")
    pose.setInput(0, moving)
    pose.setInput(1, begin)
    pose.setInput(2, rest)
    colour = node.createNode("color", "hub_colour")
    colour.setInput(0, pose)
    for i, c in enumerate("rgb"):
        colour.parm("color" + c).setExpression('point("../each_hub", 0, "Cd", %d)' % i)
    end = node.createNode("block_end", "each_hub_end")
    end.setInput(0, colour)
    end.parm("itermethod").set("pieces")
    end.parm("method").set("merge")
    end.parm("class").set("point")
    end.parm("useattrib").set(1)
    end.parm("attrib").set("hub")
    end.parm("blockpath").set("../each_hub")
    end.parm("templatepath").set("../each_hub")
    ghosts = node.createNode("material", "ghost_material")
    ghosts.setInput(0, end)
    ghosts.parm("shop_materialpath1").set(_ghost_material().path())

    # look rays and hub markers, built paths: data lines, tubes by PolyWire
    rays = _tube(node, "ray_tubes", _py(node, "rays", "rays"))
    paths = _tube(node, "path_tubes", _py(node, "paths", "paths"))

    # the operating range: Circle SOPs on the tool's parameters.
    lo, hi = 'ch("../j1_rangex")', 'ch("../j1_rangey")'
    arcs = []
    for name, kind, height in (("range_sector", "slicedarc", "0.01"), ("range_low", "openarc", 'ch("../tcp_zx")'),
                               ("range_high", "openarc", 'ch("../tcp_zy")')):
        c = node.createNode("circle", name)
        c.parm("type").set("poly")
        c.parm("orient").set("zx")
        c.parm("arc").set(kind)
        c.parm("divs").set(72)
        c.parmTuple("rad")[0].setExpression('ch("../guide_radius")')
        c.parmTuple("rad")[1].setExpression('ch("../guide_radius")')
        c.parm("ty").setExpression(height)
        c.parmTuple("angle")[0].setExpression("-(%s + %g)" % (hi, J1_FACING_DEG))     # zx_arc_angle(hi)
        c.parmTuple("angle")[1].setExpression("-(%s + %g)" % (lo, J1_FACING_DEG))     # zx_arc_angle(lo)
        arcs.append(c)
    whole = node.createNode("divide", "sector_whole")            # the sliced arc is a fan: one outline only
    whole.setInput(0, arcs[0])
    whole.parm("convex").set(0)
    whole.parm("removesh").set(1)
    sector = node.createNode("convertline", "sector_outline")
    sector.setInput(0, whole)
    sector.parm("computelength").set(0)
    rng = node.createNode("merge", "range_arcs")
    for i, n in enumerate((sector, arcs[1], arcs[2])):
        rng.setInput(i, n)
    look = node.createNode("attribcreate::2.0", "range_look")
    look.setInput(0, rng)
    look.parm("numattr").set(2)
    look.parm("name1").set("Cd")
    look.parm("class1").set("point")
    look.parm("size1").set(3)
    for i, v in enumerate((0.75, 0.75, 0.8)):
        look.parmTuple("value1v")[i].set(v)
    look.parm("name2").set("pscale")
    look.parm("class2").set("point")
    look.parm("value2v1").set(0.003)
    range_tubes = _tube(node, "range_tubes", look)

    arm_packed = node.createNode("pack", "robot_packed")
    arm_packed.setInput(0, arm)
    out = node.createNode("merge", "all")
    shown = [arm_packed,
             _switch(node, "zones_shown", zones, "show_zones"),
             _switch(node, "ghosts_shown", ghosts, "show_ghosts"),
             _switch(node, "rays_shown", rays, "show_rays"),
             _switch(node, "paths_shown", paths, "show_paths"),
             _switch(node, "range_shown", range_tubes, "show_range")]
    for i, s in enumerate(shown):
        out.setInput(i, s)
    final = node.createNode("output", "OUT")
    final.setInput(0, out)
    final.setDisplayFlag(True)
    final.setRenderFlag(True)
    node.layoutChildren()
    return node


def _ghost_material():
    """/mat/hub_ghost: a Principled Shader in each point's colour, opacity
    GHOST_ALPHA with blending. (A point or detail Alpha drew nothing in this
    viewport, 2026-09-27; the material at least draws in the hub's colour.)"""
    hou = _hou()
    m = hou.node("/mat/hub_ghost") or hou.node("/mat").createNode("principledshader::2.0", "hub_ghost")
    m.parm("basecolor_usePointColor").set(1)
    m.parm("opac").set(GHOST_ALPHA)
    m.parm("alphablendmode").set("blend")
    return m


def install(config=DEFAULT_CONFIG, env=None, name=TOOL):
    """Create (or rebuild) the show tool /obj/<name> and load the config.
    Its parameters are kept when it exists already."""
    hou = _hou()
    with hou.undos.group("Install the show tool"):
        node = hou.node("/obj/" + name)
        fresh = node is None
        if fresh:
            node = hou.node("/obj").createNode("geo", name, run_init_scripts=False)
        cfg = json.load(open(config))
        _parms(node, config, env or ROOT + "/" + cfg["env"])
        _network(node)
        node.setColor(hou.Color((0.3, 0.6, 0.9)))
        node.setComment("The show: robot, room, zones, hubs, range. All controls on this node's parameters.")
        node.setGenericFlag(hou.nodeFlag.DisplayComment, True)
        if fresh:
            load_config(node)
    clean_view()
    return node


def remove_old_objects():
    """The scattered objects of the earlier show scene, replaced by the tool."""
    hou = _hou()
    obj = hou.node("/obj")
    gone = []
    for n in list(obj.children()):
        old = n.parm("show_role") is not None or n.name() in (
            "SHOW", "SHOW_CTRL", "show_viz", "hub_ghosts", "CELL_CTRL", "cell_env", "fr20", "capsules", "ghosts")
        if old and n.name() != TOOL:
            gone.append(n.name())
            n.destroy()
    for box in obj.networkBoxes():
        if box.name() == "SHOW":
            obj.deleteItems([box])
    return gone


def clean_view(viewers=None):
    """Remove Backfaces on in the scene viewers: the room's walls face in,
    so the walls near the camera vanish (the cutaway the Isaac scene has).
    And a low transparency cutoff, for the see-through hub ghosts."""
    hou = _hou()
    if not hou.isUIAvailable():
        return
    for pane in viewers or hou.ui.paneTabs():
        if pane.type() == hou.paneTabType.SceneViewer:
            for vp in pane.viewports():
                vp.settings().setRemoveBackfaces(True)
                vp.settings().setTransparencyCutoff(0.05)


def on_load():
    """For a scene's hou.session: the viewport look and the handle state,
    once the UI is up."""
    hou = _hou()
    if not hou.isUIAvailable():
        return

    def once():
        hou.ui.removeEventLoopCallback(once)
        clean_view()
        register_state()
    hou.ui.addEventLoopCallback(once)


# --- config <-> parameters -----------------------------------------------------

def load_config(node=None):
    """The config file -> the tool's parameters."""
    node = tool(node)
    cfg = json.load(open(cfg_path(node)))
    env = json.load(open(env_path(node)))
    zones = dict(cfg.get("zones", {}))
    stage = cfg.get("stage") or next((o for o in env["objects"] if o["name"] == "stage"), None)
    if stage:
        zones["stage"] = stage
    node.parm("stage_override").set(1 if cfg.get("stage") else 0)
    node.parm("zones").set(len(zones))
    for i, (name, z) in enumerate(zones.items(), start=1):
        node.parm("zone_name%d" % i).set(name)
        node.parmTuple("zone_center%d" % i).set(z["center"])
        node.parmTuple("zone_size%d" % i).set(z["size"])
        node.parm("zone_yaw%d" % i).set(z.get("yaw_deg", 0.0))
    node.parm("hubs").set(len(cfg["hubs"]))
    rig = None
    for i, (name, h) in enumerate(cfg["hubs"].items(), start=1):
        tool_mode = bool(h.get("tcp") and h.get("look"))
        node.parm("hub_name%d" % i).set(name)
        node.parm("hub_mode%d" % i).set("tool" if tool_mode else "joints")
        q = h.get("near") or h.get("q") or REST_Q
        node.parmTuple("hub_q%d" % i).set(q)
        if tool_mode:
            node.parmTuple("hub_tcp%d" % i).set(h["tcp"])
            node.parmTuple("hub_look%d" % i).set(h["look"])
        else:                                                    # where its pose has them, for switching mode
            import gestures as G
            rig = rig or _rig()
            tcp, look, _ = G.home_of(rig, list(q))
            node.parmTuple("hub_tcp%d" % i).set(_r(tcp))
            node.parmTuple("hub_look%d" % i).set(_r(look))
        node.parm("hub_clips%d" % i).set(h.get("clips", 8))
        node.parm("hub_gen%d" % i).set(h.get("generator", "gestures" if tool_mode else "choreo"))
        node.parm("hub_zone%d" % i).set(h.get("zone", ""))
        node.parm("hub_families%d" % i).set(" ".join(h.get("families", FAMILIES.split())))
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
    node.parm("auth_hub").set(node.evalParm("auth_hub") or cfg.get("start_hub", ""))
    if not node.evalParm("edit_item") and zones:
        node.parm("edit_item").set("zone:" + next(iter(zones)))
    _report(node, "Loaded %s: %d zones, %d hubs" % (_rel(cfg_path(node)), len(zones), len(cfg["hubs"])))


def scene_parts(node=None):
    """What the tool's parameters say, as merge_config's `scene`."""
    node = tool(node)
    zones, stage = {}, None
    for i in range(1, node.evalParm("zones") + 1):
        name = node.evalParm("zone_name%d" % i).strip() or "zone%d" % i
        z = {"center": _r(node.parmTuple("zone_center%d" % i).eval()),
             "size": _r(node.parmTuple("zone_size%d" % i).eval()),
             "yaw_deg": round(_wrap(node.evalParm("zone_yaw%d" % i)), 3)}
        if name == "stage":
            stage = z if node.evalParm("stage_override") else None
        else:
            zones[name] = z
    hubs = {}
    for i in range(1, node.evalParm("hubs") + 1):
        name = node.evalParm("hub_name%d" % i).strip() or "hub%d" % i
        h = {"clips": node.evalParm("hub_clips%d" % i), "generator": node.parm("hub_gen%d" % i).evalAsString()}
        if node.evalParm("hub_zone%d" % i):
            h["zone"] = node.evalParm("hub_zone%d" % i)
        q = _r(node.parmTuple("hub_q%d" % i).eval(), 3)
        if node.parm("hub_mode%d" % i).evalAsString() == "tool":
            h.update(tcp=_r(node.parmTuple("hub_tcp%d" % i).eval()),
                     look=_r(node.parmTuple("hub_look%d" % i).eval()), near=q)
        else:
            h["q"] = q
        if h["generator"] == "gestures":
            h["families"] = node.evalParm("hub_families%d" % i).split()
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
    node = tool(node)
    cfg = merge_config(json.load(open(cfg_path(node))), scene_parts(node))
    cfg["env"] = _rel(env_path(node))
    return cfg


def write_config(node=None):
    """The parameters -> the config file. Refused (with the reasons) when the
    config would name something that is not there."""
    node = tool(node)
    cfg = scene_config(node)
    bad = check_config(cfg)
    if bad:
        _report(node, "Not written:\n  " + "\n  ".join(bad), error=True)
        return None
    with open(cfg_path(node), "w", newline="\n") as f:
        json.dump(cfg, f, indent=1)
    _report(node, "Wrote %s" % _rel(cfg_path(node)))
    _cook(node)
    return cfg


def _report(node, text, error=False):
    hou = _hou()
    tool(node).parm("report").set(text[-6000:])
    if error and hou.isUIAvailable():
        hou.ui.displayMessage(text.splitlines()[0], details=text, severity=hou.severityType.Error)


# --- buttons ------------------------------------------------------------------

def profile_changed(node):
    arm = node.node("robot_arm")
    if arm is not None:
        arm.parm("robot_profile").set(node.evalParm("robot_profile"))
        arm.hdaModule().on_profile_changed(arm)
    _CACHE.clear()
    _cook(node)


def hub_mode_changed(kwargs):
    """Tool Tip + Look At <-> Joint Angles keeps the pose: to joints, the
    solved pose becomes the Seed Pose; to tool, the tool tip and look target
    are put where the Seed Pose has them."""
    node, i = kwargs["node"], kwargs["script_multiparm_index"]
    name = node.evalParm("hub_name%s" % i)
    if node.parm("hub_mode%s" % i).evalAsString() == "joints":
        h = dict(scene_parts(node)["hubs"][name])
        h.update(tcp=_r(node.parmTuple("hub_tcp%s" % i).eval()), look=_r(node.parmTuple("hub_look%s" % i).eval()),
                 near=_r(node.parmTuple("hub_q%s" % i).eval(), 3))
        q = _hub_q(h)
        if q is not None:
            node.parmTuple("hub_q%s" % i).set(_r(q, 3))
    else:
        import gestures as G
        tcp, look, _ = G.home_of(_rig(), list(node.parmTuple("hub_q%s" % i).eval()))
        node.parmTuple("hub_tcp%s" % i).set(_r(tcp))
        node.parmTuple("hub_look%s" % i).set(_r(look))
    _cook(node)


def keep_solve(kwargs):
    """The solved pose becomes the seed: later drags stay near it."""
    node, i = kwargs["node"], kwargs["script_multiparm_index"]
    h = scene_parts(node)["hubs"][node.evalParm("hub_name%s" % i)]
    q = _hub_q(h)
    if q is None:
        _report(node, "hub %s: no pose to keep (not reachable)" % node.evalParm("hub_name%s" % i), error=True)
        return
    node.parmTuple("hub_q%s" % i).set(_r(q, 3))


def add_clip(node=None):
    """The chosen joint CSV -> the config's authored clips."""
    node = tool(node)
    csv = node.evalParm("auth_csv")
    if not csv or not os.path.exists(csv):
        _report(node, "No joint CSV at %r -- pick the robot_arm's export (Output > Export CSV)" % csv, error=True)
        return
    cfg = json.load(open(cfg_path(node)))
    clip_id = node.evalParm("auth_id").strip() or os.path.splitext(os.path.basename(csv))[0]
    cfg["authored"] = add_authored(cfg.get("authored", []), csv, node.evalParm("auth_hub"), clip_id)
    with open(cfg_path(node), "w", newline="\n") as f:
        json.dump(cfg, f, indent=1)
    _report(node, "Added %s (at hub %s) to %s -- Build Show makes its moves in and out and checks it"
            % (clip_id, node.evalParm("auth_hub"), os.path.basename(cfg_path(node))))


def preview_segment(node=None):
    node = tool(node)
    name = node.evalParm("segment")
    if name:
        node.parm("preview_clip").set(preview_dir(node) + "/" + name + ".json")
        fit_range(node)


def preview_hub(node=None):
    """The arm held at a hub's pose (a two-point still clip)."""
    node = tool(node)
    name = node.evalParm("pose_hub")
    h = scene_parts(node)["hubs"].get(name)
    q = h and _hub_q(h)
    if q is None:
        _report(node, "hub %s: no pose to show" % name, error=True)
        return
    os.makedirs(preview_dir(node), exist_ok=True)
    path = preview_dir(node) + "/_hub_%s.json" % name
    with open(path, "w", newline="\n") as f:
        json.dump({"id": "hub", "points": [{"t": 0.0, "q": list(q)}, {"t": 1.0, "q": list(q)}]}, f)
    node.parm("preview_clip").set(path)
    fit_range(node)


def _clip(path):
    """(times, joints) of a clip JSON or joint CSV, cached by file time."""
    key = ("clip", path, os.path.getmtime(path) if os.path.exists(path) else 0)
    if key not in _CACHE:
        if not os.path.exists(path):
            _CACHE[key] = ([], [])
        elif path.lower().endswith(".json"):
            c = json.load(open(path))
            _CACHE[key] = ([p["t"] for p in c["points"]], [p["q"] for p in c["points"]])
        else:
            import fairino_player
            _CACHE[key] = fairino_player.load_csv(path)
    return _CACHE[key]


def joint(arm, j):
    """The arm's FK expression: joint j (1-6, degrees) of the preview clip at
    the current frame (24 fps from frame 1); the start hub's pose without one."""
    hou = _hou()
    node = tool(arm.parent())
    t, q = _clip(node.evalParm("preview_clip"))
    if not t:
        h = scene_parts(node)["hubs"].get(node.evalParm("start_hub"))
        pose = (h and _hub_q(h)) or REST_Q
        return pose[j - 1]
    s = (hou.frame() - 1.0) / 24.0
    if s <= t[0]:
        return q[0][j - 1]
    if s >= t[-1]:
        return q[-1][j - 1]
    import bisect
    i = bisect.bisect_right(t, s) - 1
    f = (s - t[i]) / (t[i + 1] - t[i])
    return q[i][j - 1] + f * (q[i + 1][j - 1] - q[i][j - 1])


def fit_range(node=None):
    hou = _hou()
    t, _ = _clip(tool(node).evalParm("preview_clip"))
    if t:
        end = 1 + int(math.ceil(t[-1] * 24.0))
        hou.playbar.setFrameRange(1, end)
        hou.playbar.setPlaybackRange(1, end)
        hou.setFrame(1)


def _run(args, title):
    hou = _hou()
    with hou.InterruptableOperation(title, open_interrupt_dialog=True):
        r = subprocess.run([PYTHON, ROOT + "/scripts/show.py"] + args, capture_output=True, text=True, cwd=ROOT)
    return r.returncode, (r.stdout + r.stderr).strip()


def build_show(node=None):
    """Write the config, build the show (every clip made and checked), redraw."""
    node = tool(node)
    if write_config(node) is None:
        return 1, ""
    code, out = _run(["build", cfg_path(node)], "Building the show (clips made and checked)")
    _report(node, out, error=bool(code))
    _cook(node)
    return code, out


def dry_run(node=None):
    node = tool(node)
    seqs = list(json.load(open(cfg_path(node))).get("sequences", {}))
    code, out = _run(["dry-run", cfg_path(node), "--minutes", str(node.evalParm("dry_minutes")),
                      "--trigger-every", "40", "--triggers", ",".join(["scan"] + seqs)], "Dry run of the show")
    _report(node, out, error=bool(code))
    return code, out


def check(node=None):
    """The hubs solved and checked, and the config's names: into the report."""
    node = tool(node)
    cfg = scene_config(node)
    lines = ["config: " + ("; ".join(check_config(cfg)) or "names consistent")]
    if node.evalParm("robot_profile") != "fr20":
        lines.append("note: the show generator and its checks are made for the FR20")
    for name, h in cfg["hubs"].items():
        q, why = hub_status(cfg, h)
        lines.append("hub %-8s %s" % (name, why or "clear, reachable, in range: " + " ".join("%.1f" % x for x in q)))
    _report(node, "\n".join(lines))


# --- solving and drawing (the Python SOPs: data only) ---------------------------

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


def _move_env(cfg):
    import collision as C
    import safe_move
    import show as S
    path = os.path.join(ROOT, cfg["env"])
    key = ("env", json.dumps([cfg["env"], os.path.getmtime(path), cfg.get("stage"), cfg.get("canvas"),
                              cfg["margins"]["idle_canvas_m"]]))
    if key not in _CACHE:
        env = C.load_env(path)
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


def _hub_colour(k):
    return HUB_RGB[k % len(HUB_RGB)]


def zone_points(sop):
    """Python SOP: one point per zone -- P (centre), orient, scale (size), Cd,
    name. The zones are a Box copied onto them."""
    hou = _hou()
    geo = sop.geometry()
    for name, size, default in (("orient", 4, (0.0, 0.0, 0.0, 1.0)), ("scale", 3, (1.0, 1.0, 1.0)),
                                ("Cd", 3, (1.0, 1.0, 1.0))):
        geo.addAttrib(hou.attribType.Point, name, default)
    geo.addAttrib(hou.attribType.Point, "name", "")
    node = tool(sop)
    for i in range(1, node.evalParm("zones") + 1):
        name = node.evalParm("zone_name%d" % i)
        z = {"center": node.parmTuple("zone_center%d" % i).eval(), "size": node.parmTuple("zone_size%d" % i).eval(),
             "yaw_deg": node.evalParm("zone_yaw%d" % i)}
        P, orient, scale = zone_instance(z)
        pt = geo.createPoint()
        pt.setPosition(P)
        pt.setAttribValue("orient", orient)
        pt.setAttribValue("scale", scale)
        pt.setAttribValue("Cd", ZONE_RGB.get(name, (0.8, 0.8, 0.8)))
        pt.setAttribValue("name", name)


def hub_poses(sop):
    """Python SOP: each hub's posed skeleton (urdf_rig.posed_skeleton: points
    name, P, transform, as the robot_arm asset poses its meshes), tagged with
    the hub's name and colour (red when it cannot be used). Warns for such a
    hub."""
    hou = _hou()
    import urdf_rig as UR
    geo = sop.geometry()
    geo.addAttrib(hou.attribType.Point, "name", "")
    geo.addAttrib(hou.attribType.Point, "transform", (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
    geo.addAttrib(hou.attribType.Point, "hub", "")
    geo.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    cfg = scene_config(sop)
    m = _cmodel()
    bad = []
    for k, (name, h) in enumerate(cfg["hubs"].items()):
        q, why = hub_status(cfg, h)
        if why:
            bad.append("hub %s: %s" % (name, why))
        if q is None:
            continue
        cd = BAD_RGB if why else _hub_colour(k)
        for j in UR.posed_skeleton(m["chain"], m["flange_offset"], q):
            pt = geo.createPoint()
            pt.setPosition(j["P"])
            pt.setAttribValue("name", j["name"])
            pt.setAttribValue("transform", [c for row in j["transform"] for c in row])
            pt.setAttribValue("hub", name)
            pt.setAttribValue("Cd", cd)
    if bad:
        raise hou.NodeWarning("\n".join(bad))


def _lines_geo(sop):
    hou = _hou()
    geo = sop.geometry()
    geo.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Point, "pscale", 0.004)

    def line(pts, cd, width):
        poly = geo.createPolygon(is_closed=False)
        for p in pts:
            pt = geo.createPoint()
            pt.setPosition(to_h(p))
            pt.setAttribValue("Cd", cd)
            pt.setAttribValue("pscale", width)
            poly.addVertex(pt)
    return line


def rays(sop):
    """Python SOP: each Tool Tip + Look At hub's look ray, tool tip to look
    target (red when the hub cannot be used)."""
    line = _lines_geo(sop)
    cfg = scene_config(sop)
    for k, (name, h) in enumerate(cfg["hubs"].items()):
        if h.get("look"):
            q, why = hub_status(cfg, h)
            line([h["tcp"], h["look"]], BAD_RGB if why else _hub_colour(k), 0.004)


def paths(sop):
    """Python SOP: the built clips' tool paths (idle in their hub's colour,
    moves white, the scan yellow)."""
    line = _lines_geo(sop)
    node = tool(sop)
    names = hub_names(node)
    man = preview_dir(node) + "/manifest.json"
    if not os.path.exists(man):
        return
    for s in json.load(open(man))["segments"]:
        f = preview_dir(node) + "/" + s["name"] + ".json"
        if not os.path.exists(f):
            continue
        tcp = json.load(open(f)).get("tcp") or []
        if s["kind"] == "idle":
            cd = _hub_colour(names.index(s["start"])) if s["start"] in names else (0.8, 0.8, 0.8)
        else:
            cd = (0.9, 0.9, 0.9) if s["kind"] == "move" else (1.0, 0.85, 0.2)
        if len(tcp) > 1:
            line(tcp[::2] + tcp[-1:], cd, 0.003)


def _cook(node):
    hou = _hou()
    node = tool(node)
    for name in ("zone_points", "hub_poses", "rays", "paths"):
        sop = node.node(name)
        if sop:
            try:
                sop.cook(force=True)
            except hou.OperationFailed:
                pass                                   # its warning / error shows on the node


# --- viewport handles (a Python viewer state) -----------------------------------

STATE = "robot_show_edit"


def edit_in_viewport(node=None):
    """Handles in the viewport for the tool's Edit in Viewport item."""
    hou = _hou()
    node = tool(node)
    register_state()
    sv = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
    if sv is None:
        return
    sv.setPwd(node.parent())
    node.setCurrent(True, clear_all_selected=True)
    sv.setCurrentState(STATE)


def _item(node):
    kind, _, name = node.evalParm("edit_item").partition(":")
    names = zone_names(node) if kind == "zone" else hub_names(node) if kind == "hub" else []
    return (kind, names.index(name) + 1) if name in names else (None, None)


class EditState(object):
    """Handles for one zone (an xform: move, turn about the vertical, scale)
    or one hub (translate: the tool tip; a second one: the look target). The
    tool's parameters are in the robot frame; handles work in Houdini's."""

    def __init__(self, state_name, scene_viewer):
        hou = _hou()
        self.sv = scene_viewer
        self.zone = hou.Handle(scene_viewer, "zone")
        self.tip = hou.Handle(scene_viewer, "tip")
        self.look = hou.Handle(scene_viewer, "look")

    def _node(self):
        return tool()

    def _refresh(self):
        node = self._node()
        kind, i = _item(node) if node else (None, None)
        tool_hub = kind == "hub" and node.parm("hub_mode%d" % i).evalAsString() == "tool"
        self.zone.show(kind == "zone")
        self.tip.show(tool_hub)
        self.look.show(tool_hub)
        for h in (self.zone, self.tip, self.look):
            h.update()

    def onEnter(self, kwargs):
        self._refresh()

    def onResume(self, kwargs):
        self._refresh()

    def onParmChangeEvent(self, kwargs):
        if kwargs.get("parm_name") == "edit_item":
            self._refresh()

    def onStateToHandle(self, kwargs):
        node = self._node()
        kind, i = _item(node)
        p = kwargs["parms"]
        name = kwargs["handle"]
        if kind == "zone" and name == "zone":
            t, r, s = zone_to_xform({"center": node.parmTuple("zone_center%d" % i).eval(),
                                     "size": node.parmTuple("zone_size%d" % i).eval(),
                                     "yaw_deg": node.evalParm("zone_yaw%d" % i)})
            p["tx"], p["ty"], p["tz"] = t
            p["rx"], p["ry"], p["rz"] = r
            p["sx"], p["sy"], p["sz"] = s
        elif kind == "hub" and name in ("tip", "look"):
            v = to_h(node.parmTuple(("hub_tcp%d" if name == "tip" else "hub_look%d") % i).eval())
            p["tx"], p["ty"], p["tz"] = v

    def onHandleToState(self, kwargs):
        hou = _hou()
        node = self._node()
        kind, i = _item(node)
        p = kwargs["parms"]
        name = kwargs["handle"]
        with hou.undos.group("Move %s" % node.evalParm("edit_item")):
            if kind == "zone" and name == "zone":
                z = xform_to_zone((p["tx"], p["ty"], p["tz"]), (p["rx"], p["ry"], p["rz"]), (p["sx"], p["sy"], p["sz"]))
                node.parmTuple("zone_center%d" % i).set(z["center"])
                node.parmTuple("zone_size%d" % i).set(z["size"])
                node.parm("zone_yaw%d" % i).set(z["yaw_deg"])
            elif kind == "hub" and name in ("tip", "look"):
                v = _r(to_r((p["tx"], p["ty"], p["tz"])))
                node.parmTuple(("hub_tcp%d" if name == "tip" else "hub_look%d") % i).set(v)


def register_state():
    """The viewer state behind Edit in Viewport (registered once per session)."""
    hou = _hou()
    if not hou.isUIAvailable():
        return
    try:
        if hou.ui.isRegisteredViewerState(STATE):
            hou.ui.unregisterViewerState(STATE)
    except hou.OperationFailed:
        pass
    t = hou.ViewerStateTemplate(STATE, "Robot Show: Edit", hou.objNodeTypeCategory())
    t.bindFactory(EditState)
    t.bindHandle("xform", "zone", settings="translate(1) rotate(1) scale(1)")
    t.bindHandle("translate", "tip")
    t.bindHandle("translate", "look")
    hou.ui.registerViewerState(t)


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

    P, o, sc = zone_instance(z)
    # rotate the unit X axis by the quaternion o and compare with the yaw in Houdini
    qx, qy, qz, qw = o
    v = (1.0, 0.0, 0.0)
    rot_x = (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qw * qz), 2 * (qx * qz - qw * qy))
    th = math.radians(z["yaw_deg"])
    check_("a zone copies a Box turned by its yaw, sized Y up",
           all(abs(a - b) < 1e-9 for a, b in zip(rot_x, (math.cos(th), 0.0, -math.sin(th))))
           and sc == (1.3, 0.7, 0.5) and P == to_h(z["center"]), (rot_x, sc))

    for j1 in (-150.0, -60.0, 0.0, 90.0):
        a = math.radians(zx_arc_angle(j1))
        phi = math.radians(j1 + J1_FACING_DEG)
        circle = (math.cos(a), 0.0, math.sin(a))
        check_("the range arc at J1 %g points where the arm faces" % j1,
               all(abs(x - y) < 1e-9 for x, y in zip(circle, to_h((math.cos(phi), math.sin(phi), 0.0)))))

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
