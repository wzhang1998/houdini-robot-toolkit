"""Houdini side of the show (scenes/FR20_show.hiplc, built by
scripts/build_show_scene.py): SHOW_CTRL's parameters <-> the show config
(shows/*.json), the Build / Dry Run buttons, previews on the arm, and the
show drawn in the viewport.

The config file stays the one source: Load reads it into the parameters,
Write writes the parameters back (keys the panel does not show -- OSC,
sequences, margins -- are kept), Build runs `python scripts/show.py build`
on it (the same code the dry run, Isaac and the robot use) and the viewport
redraws from what it wrote.

Drawn (show_geo): the stage (the controller-facing work zone the clips must
stay in), the paper, each hub's TCP (red when the hub pose is not clear of
the room), and every built segment's TCP path -- idle clips in their hub's
colour, moves white, the scan and its moves yellow.
"""

import json
import os
import subprocess
import sys

import hou

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE).replace("\\", "/")
sys.path.insert(0, HERE)

PYTHON = os.environ.get("SHOW_PYTHON", "python")          # a Python with the toolkit's needs (python-osc for osc)
HUB_RGB = [(0.95, 0.45, 0.2), (0.3, 0.7, 1.0), (0.55, 0.9, 0.35), (0.85, 0.4, 0.95), (1.0, 0.85, 0.25)]


def _ctrl():
    return hou.node("/obj/SHOW_CTRL")


def cfg_path(node=None):
    return (node or _ctrl()).evalParm("config").replace("\\", "/")


def show_name(node=None):
    return os.path.splitext(os.path.basename(cfg_path(node)))[0]


def preview_dir(node=None):
    return ROOT + "/geo/show/" + show_name(node)


# --------------------------------------------------------------------------
# parameters <-> config
# --------------------------------------------------------------------------

def load_config(node=None):
    """shows/*.json -> SHOW_CTRL's parameters."""
    node = node or _ctrl()
    cfg = json.load(open(cfg_path(node)))
    hubs = list(cfg["hubs"].items())
    node.parm("hubs").set(len(hubs))
    for i, (name, h) in enumerate(hubs, start=1):
        node.parm("hub_name%d" % i).set(name)
        node.parmTuple("hub_q%d" % i).set(h["q"])
        node.parm("hub_clips%d" % i).set(h.get("clips", 0))
    node.parm("start_hub").set(cfg.get("start_hub", hubs[0][0]))
    lib = cfg["library"]
    node.parmTuple("duration").set(lib["duration_s"])
    node.parmTuple("bars").set((min(lib["bars"]), max(lib["bars"])))
    node.parmTuple("bpm").set(lib["bpm"])
    node.parm("seed").set(lib.get("seed", 1))
    st = cfg.get("stage")
    node.parm("stage_override").set(1 if st else 0)
    if st:
        node.parmTuple("stage_center").set(st["center"])
        node.parmTuple("stage_size").set(st["size"])
        node.parm("stage_yaw").set(st.get("yaw_deg", 0.0))
    else:
        o = _env_stage(cfg)
        if o:
            node.parmTuple("stage_center").set(o["center"])
            node.parmTuple("stage_size").set(o["size"])
            node.parm("stage_yaw").set(o.get("yaw_deg", 0.0))
    c = cfg.get("canvas")
    if c:
        node.parmTuple("canvas_center").set(c["center"])
        node.parmTuple("canvas_normal").set(c["normal"])
        node.parmTuple("canvas_size").set(c["size"])
    node.parm("idle_canvas_m").set(cfg["margins"]["idle_canvas_m"])
    node.parmTuple("hub_stay").set(cfg.get("select", {}).get("hub_stay", (2, 4)))


def _env_stage(cfg):
    env = json.load(open(os.path.join(ROOT, cfg["env"])))
    return next((o for o in env["objects"] if o["name"] == "stage"), None)


def parms_to_config(node=None):
    """SHOW_CTRL's parameters over the config file's contents (other keys kept)."""
    node = node or _ctrl()
    cfg = json.load(open(cfg_path(node)))
    hubs = {}
    for i in range(1, node.evalParm("hubs") + 1):
        name = node.evalParm("hub_name%d" % i).strip() or "hub%d" % i
        old = cfg["hubs"].get(name, {})
        hubs[name] = dict(old, q=[round(x, 3) for x in node.parmTuple("hub_q%d" % i).eval()],
                          clips=node.evalParm("hub_clips%d" % i))
    cfg["hubs"] = hubs
    start = node.evalParm("start_hub")
    cfg["start_hub"] = start if start in hubs else next(iter(hubs))
    b0, b1 = node.parmTuple("bars").eval()
    cfg["library"].update(duration_s=list(node.parmTuple("duration").eval()), bars=list(range(b0, b1 + 1)),
                          bpm=list(node.parmTuple("bpm").eval()), seed=node.evalParm("seed"))
    if node.evalParm("stage_override"):
        cfg["stage"] = {"name": "stage", "center": [round(x, 4) for x in node.parmTuple("stage_center").eval()],
                        "size": [round(x, 4) for x in node.parmTuple("stage_size").eval()],
                        "yaw_deg": node.evalParm("stage_yaw")}
    else:
        cfg.pop("stage", None)
    if cfg.get("canvas"):
        cfg["canvas"].update(center=[round(x, 4) for x in node.parmTuple("canvas_center").eval()],
                             normal=[round(x, 4) for x in node.parmTuple("canvas_normal").eval()],
                             size=list(node.parmTuple("canvas_size").eval()))
    if cfg.get("scan") and cfg["scan"].get("from_hub") not in hubs:
        cfg["scan"]["from_hub"] = cfg["start_hub"]
    cfg["margins"]["idle_canvas_m"] = node.evalParm("idle_canvas_m")
    cfg.setdefault("select", {})["hub_stay"] = list(node.parmTuple("hub_stay").eval())
    return cfg


def write_config(node=None):
    node = node or _ctrl()
    cfg = parms_to_config(node)
    with open(cfg_path(node), "w") as f:
        json.dump(cfg, f, indent=1)
    return cfg


# --------------------------------------------------------------------------
# buttons
# --------------------------------------------------------------------------

def _run(args, title):
    with hou.InterruptableOperation(title, open_interrupt_dialog=True):
        r = subprocess.run([PYTHON, ROOT + "/scripts/show.py"] + args, capture_output=True, text=True, cwd=ROOT)
    return r.returncode, (r.stdout + r.stderr).strip()


def build_show(node=None):
    """Write the config, build the show (every clip made and checked), redraw."""
    node = node or _ctrl()
    write_config(node)
    code, out = _run(["build", cfg_path(node)], "Building the show (clips made and checked)")
    node.parm("report").set(out[-4000:])
    hou.node("/obj/show_viz/show").cook(force=True)
    if code and hou.isUIAvailable():
        hou.ui.displayMessage("Build failed", details=out[-4000:], severity=hou.severityType.Error)
    return code, out


def dry_run(node=None):
    node = node or _ctrl()
    code, out = _run(["dry-run", cfg_path(node), "--minutes", str(node.evalParm("dry_minutes")),
                      "--trigger-every", "40", "--triggers", "scan," + ",".join(_sequences(node))],
                     "Dry run of the show")
    node.parm("report").set(out[-4000:])
    return code, out


def _sequences(node):
    return list(json.load(open(cfg_path(node))).get("sequences", {}))


def segment_menu(node=None):
    """Menu items (token, label) of the built segments, from the preview manifest."""
    node = node or _ctrl()
    man = preview_dir(node) + "/manifest.json"
    items = []
    if os.path.exists(man):
        for s in json.load(open(man))["segments"]:
            items += [s["name"], "%s  (%s, %.1f s)" % (s["name"], s["kind"], s["duration_s"])]
    return items or ["", "(build the show first)"]


def preview_segment(node=None):
    """Play the chosen segment on the arm (CELL_CTRL's clip), range fitted."""
    node = node or _ctrl()
    name = node.evalParm("segment")
    if not name:
        return
    hou.node("/obj/CELL_CTRL").parm("clip").set(preview_dir(node) + "/" + name + ".json")
    hou.session.cell_sop.fit_range()


def preview_hub(node=None):
    """Put the arm at the chosen hub (a two-point still clip)."""
    node = node or _ctrl()
    i = node.evalParm("hub_pick")
    if not 1 <= i <= node.evalParm("hubs"):
        return
    q = list(node.parmTuple("hub_q%d" % i).eval())
    os.makedirs(preview_dir(node), exist_ok=True)
    path = preview_dir(node) + "/_hub_%s.json" % node.evalParm("hub_name%d" % i)
    json.dump({"id": "hub", "points": [{"t": 0.0, "q": q}, {"t": 1.0, "q": q}]}, open(path, "w"))
    hou.node("/obj/CELL_CTRL").parm("clip").set(path)
    hou.session.cell_sop.fit_range()


# --------------------------------------------------------------------------
# drawing
# --------------------------------------------------------------------------

def _h(p):
    return (p[0], p[2], -p[1])          # robot frame (Z up) -> Houdini (Y up)


def show_geo(node):
    """The show in the viewport: stage, paper, hubs, segment paths."""
    import math
    import collision as C
    import safe_move
    import show as S
    geo = node.geometry()
    geo.addAttrib(hou.attribType.Prim, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Prim, "name", "")
    geo.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Point, "name", "")
    geo.addAttrib(hou.attribType.Point, "pscale", 0.03)
    ctrl = _ctrl()
    cfg = parms_to_config(ctrl)

    def line(pts, cd, name, closed=False):
        poly = geo.createPolygon(is_closed=closed)
        for p in pts:
            pt = geo.createPoint()
            pt.setPosition(_h(p))
            poly.addVertex(pt)
        poly.setAttribValue("Cd", cd)
        poly.setAttribValue("name", name)

    def box(center, size, yaw, cd, name):
        c, s = center, size
        cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        corner = lambda i, j, k: (c[0] + cy * i * s[0] / 2 - sy * j * s[1] / 2,
                                  c[1] + sy * i * s[0] / 2 + cy * j * s[1] / 2, c[2] + k * s[2] / 2)
        for k in (-1, 1):
            line([corner(-1, -1, k), corner(1, -1, k), corner(1, 1, k), corner(-1, 1, k), corner(-1, -1, k)], cd, name)
        for i, j in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            line([corner(i, j, -1), corner(i, j, 1)], cd, name)

    st = cfg.get("stage") or _env_stage(cfg)
    if st:
        box(st["center"], st["size"], st.get("yaw_deg", 0.0), (0.2, 0.9, 0.35), "stage")
    if cfg.get("canvas"):
        cb = S.canvas_box(cfg["canvas"])
        box(cb["center"], cb["size"], cb["yaw_deg"], (1.0, 0.85, 0.2), "canvas")
    # hubs: the TCP at each hub pose, red when the pose is not clear
    model = C.load_model("fr20")
    env = C.load_env(os.path.join(ROOT, cfg["env"]))
    menv = safe_move.move_env(S.show_env(env, cfg, cfg["margins"]["idle_canvas_m"]))
    colour = {}
    for k, (name, h) in enumerate(cfg["hubs"].items()):
        colour[name] = HUB_RGB[k % len(HUB_RGB)]
        caps, tcp = C.capsules(model, h["q"])
        blocked = safe_move.blocked(model, menv, h["q"])
        pt = geo.createPoint()
        pt.setPosition(_h(tcp))
        pt.setAttribValue("Cd", (1.0, 0.1, 0.1) if blocked else colour[name])
        pt.setAttribValue("name", name + (" (NOT CLEAR: %s near %s)" % blocked[:2] if blocked else ""))
        pt.setAttribValue("pscale", 0.06)
        # the arm's centre line at the hub, so a hub reads as a pose
        line([a for _, a, b, r in caps] + [caps[-1][2]], colour[name], "hub_" + name)
    # built segments
    pd = preview_dir(ctrl)
    man = pd + "/manifest.json"
    if os.path.exists(man):
        for s in json.load(open(man))["segments"]:
            f = pd + "/" + s["name"] + ".json"
            if not os.path.exists(f):
                continue
            tcp = json.load(open(f)).get("tcp") or []
            if s["kind"] == "idle":
                cd = colour.get(s["start"], (0.8, 0.8, 0.8))
            elif s["kind"] == "move":
                cd = (0.9, 0.9, 0.9)
            else:
                cd = (1.0, 0.85, 0.2)
            line(tcp[::2] + tcp[-1:], cd, s["name"])
