"""PythonModule of wenyi::dance_phrase -- a Laban phrase, made and previewed in Houdini.

Readable copy (scripts/ vs otls/: keep in sync). Built into
otls/sop_wenyi.dance_phrase.1.0.hdalc by scripts/build_dance_hda.py.

The node wraps scripts/choreo.py: Bars (one Laban action each), Tempo,
Flow, Seed, an optional planning acceleration and the cell file. Generate
makes the phrase -- a few seconds, so it runs on the button, not on every
parameter change -- and keeps the clip on the node (user data "clip"), so
the scene reopens with it and frames never recompute it.

Output: the TCP path (Houdini frame, Y up), coloured by each bar's action,
point attributes t / speed / bar / action; a point "now" at the current
frame; detail attributes q (6 floats, robot-frame degrees at the current
frame), ok, status, duration, sequence (measured actions per bar).

Drive robot_arm points a wenyi::robot_arm's FK joints at q (Pose Source FK),
so the real FR20 mesh dances in the viewport. Export writes the player's CSV
and the clip JSON; Load Clip reads a clip JSON (the factories' geo/dance,
tests/clips) back into the node, bars and all.
"""

import json
import math
import os
import sys

import hou

FPS = 24.0
ACTIONS = ["punch", "slash", "press", "wring", "dab", "flick", "glide", "float"]
HUE = {"punch": 0, "slash": 28, "press": 50, "wring": 290, "dab": 185, "flick": 60, "glide": 120, "float": 215}


def _scripts():
    s = hou.text.expandString("$HIP/../scripts")
    if s not in sys.path:
        sys.path.insert(0, s)
    return s


def _clip(node):
    raw = node.userData("clip")
    if not raw:
        return None
    cache = getattr(hou.session, "_dance_cache", None) if hasattr(hou, "session") else None
    key = (node.path(), len(raw), hash(raw))
    if cache is not None and cache.get("key") == key:
        return cache["clip"]
    c = json.loads(raw)
    try:
        hou.session._dance_cache = {"key": key, "clip": c}
    except Exception:
        pass
    return c


def spec(node):
    n = node.evalParm("bars")
    bars = [{"action": ACTIONS[node.evalParm("action%d" % (i + 1))]} for i in range(n)]
    return {"bars": bars, "bpm": int(node.evalParm("bpm")), "flow": round(node.evalParm("flow"), 3)}


def generate(node):
    _scripts()
    import importlib
    import choreo
    import collision
    import motion_labels
    importlib.reload(motion_labels)
    importlib.reload(choreo)
    sp = spec(node)
    if not sp["bars"]:
        raise hou.NodeError("add at least one bar")
    acc = node.evalParm("acc")
    kin = choreo.Kin(acc=acc if acc > 0 else None)
    envp = node.evalParm("env_file").strip()
    env = collision.load_env(envp) if envp and os.path.exists(envp) else None
    with hou.InterruptableOperation("Generating phrase", open_interrupt_dialog=True):
        c = choreo.make_clip(sp, seed=int(node.evalParm("seed")), env=env, kin=kin,
                             clip_id=node.name(), tags=("houdini",))
    node.setUserData("clip", json.dumps(c))
    node.parm("status").set(status_line(c))
    node.cook(force=True)
    if c["points"]:
        fit_range(node)


def status_line(c):
    s = c.get("safety", {})
    if not s.get("ok"):
        return "REJECTED: " + "; ".join(s.get("reasons", [])[:2])
    lab = c.get("labels", {})
    return "ok  %.1f s  plays at its own speed  |  measured %s  |  %s  |  bpm %s%s" % (
        c["meta"]["duration_s"], "-".join(lab.get("sequence", [])), s.get("collision", "no cell"),
        c["style"].get("bpm_played"), "" if c["style"].get("tempo_scale", 1.0) <= 1.0 else
        " (x%.2f slower to fit)" % c["style"]["tempo_scale"])


def fit_range(node):
    c = _clip(node)
    if not c or not c["points"]:
        return
    end = 1 + int(math.ceil(c["points"][-1]["t"] * FPS))
    hou.playbar.setFrameRange(1, end)
    hou.playbar.setPlaybackRange(1, end)


def q_at(node, frame=None):
    c = _clip(node)
    if not c or not c["points"]:
        return [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    pts = c["points"]
    s = ((frame if frame is not None else hou.frame()) - 1.0) / FPS
    if s <= pts[0]["t"]:
        return list(pts[0]["q"])
    if s >= pts[-1]["t"]:
        return list(pts[-1]["q"])
    i = min(len(pts) - 2, int(s * FPS))
    while i > 0 and pts[i]["t"] > s:
        i -= 1
    while i < len(pts) - 2 and pts[i + 1]["t"] <= s:
        i += 1
    a, b = pts[i], pts[i + 1]
    f = (s - a["t"]) / (b["t"] - a["t"])
    return [x + f * (y - x) for x, y in zip(a["q"], b["q"])]


def joint(path, j):
    """For robot_arm's FK expressions: hou.node(path).hdaModule().joint(path, j)."""
    return q_at(hou.node(path))[j - 1]


def _h(p):
    return (p[0], p[2], -p[1])


def cook(sop):
    node = sop.parent()
    geo = sop.geometry()
    geo.clear()
    c = _clip(node)
    for name, default in (("ok", 0), ("duration", 0.0), ("status", ""), ("sequence", "")):
        geo.addAttrib(hou.attribType.Global, name, default)
    geo.addAttrib(hou.attribType.Global, "q", (0.0,) * 6)
    geo.setGlobalAttribValue("status", node.evalParm("status"))
    geo.setGlobalAttribValue("q", tuple(q_at(node)))
    if not c or not c["points"] or not c.get("tcp"):
        return
    geo.setGlobalAttribValue("ok", int(bool(c["safety"].get("ok"))))
    geo.setGlobalAttribValue("duration", float(c["meta"]["duration_s"]))
    geo.setGlobalAttribValue("sequence", "-".join(c.get("labels", {}).get("sequence", [])))
    for name, default in (("Cd", (1.0, 1.0, 1.0)), ("t", 0.0), ("speed", 0.0), ("bar", -1), ("action", "")):
        geo.addAttrib(hou.attribType.Point, name, default)
    bars = c.get("labels", {}).get("bars", [])
    ts = [p["t"] for p in c["points"]]
    poly = geo.createPolygon(is_closed=False)
    for i, (t, p) in enumerate(zip(ts, c["tcp"])):
        pt = geo.createPoint()
        pt.setPosition(_h(p))
        v = math.dist(c["tcp"][i], c["tcp"][i - 1]) / (t - ts[i - 1]) if i else 0.0
        bi, act = -1, ""
        for k, b in enumerate(bars):
            if b["t0"] <= t <= b["t1"]:
                bi, act = k, b.get("intent", "")
        col = hou.Color()
        col.setHSV((HUE.get(act, 0), 0.9 if act else 0.0, 1.0 if act else 0.7))
        pt.setAttribValue("Cd", col.rgb())
        pt.setAttribValue("t", t)
        pt.setAttribValue("speed", v)
        pt.setAttribValue("bar", bi)
        pt.setAttribValue("action", act)
        poly.addVertex(pt)
    # where the tool is now
    _scripts()
    import collision
    _, tcp = collision.capsules(collision.load_model("fr20"), q_at(node))
    now = geo.createPoint()
    now.setPosition(_h(tcp))
    now.setAttribValue("Cd", (1.0, 1.0, 1.0))
    grp = geo.createPointGroup("now")
    grp.add(now)


def drive_robot(node):
    """Point robot_arm's FK joints at this phrase (Pose Source FK)."""
    path = node.evalParm("robot").strip()
    arm = hou.node(path) if path else None
    if arm is None or not arm.type().name().startswith("wenyi::robot_arm"):
        raise hou.NodeError("Robot: the path of a wenyi::robot_arm node")
    arm.parm("pose_source").set(0)
    for j in range(1, 7):
        arm.parm("fk_j%d" % j).setExpression(
            'hou.node("%s").hdaModule().joint("%s", %d)' % (node.path(), node.path(), j), hou.exprLanguage.Python)
    node.parm("status").set(node.evalParm("status").split("  ||  ")[0] + "  ||  driving " + arm.path())


def export(node):
    _scripts()
    import motion_clip
    c = _clip(node)
    if not c or not c["points"]:
        raise hou.NodeError("Generate first")
    path = node.evalParm("export_csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    motion_clip.to_csv(c, path)
    motion_clip.save(c, os.path.splitext(path)[0] + ".json")
    node.parm("status").set(status_line(c) + "  ||  exported " + os.path.basename(path))


def load_clip(node):
    path = node.evalParm("clip_file").strip()
    if not path or not os.path.exists(path):
        raise hou.NodeError("Clip: a clip JSON (geo/dance, tests/clips)")
    with open(path) as f:
        c = json.load(f)
    sp = c.get("style", {}).get("spec")
    if sp:
        node.parm("bars").set(len(sp["bars"]))
        for i, b in enumerate(sp["bars"]):
            node.parm("action%d" % (i + 1)).set(ACTIONS.index(b["action"]))
        node.parm("bpm").set(sp.get("bpm", 90))
        node.parm("flow").set(sp.get("flow", 0.0))
        node.parm("seed").set(c["style"].get("seed", 0))
    node.setUserData("clip", json.dumps(c))
    node.parm("status").set(status_line(c) + "  ||  loaded " + os.path.basename(path))
    node.cook(force=True)
    fit_range(node)
