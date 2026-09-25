"""PythonModule of wenyi::robot_arm -- the joint setup follows the profile.

Readable copy. The asset runs the text stored in its PythonModule section;
keep the two in sync (README: scripts/ vs otls/).

Until this module existed, three things inside the asset were written for
UF850 and nothing else, although limits and presets already came from the
profile:

    configurejoints1   the axis each joint turns about (J5 pinned to z) and
                       literal limits on J2/J4/J6 (+-132, +-360)
    rigpose_fk         the FK axis per joint, and J3's sign as a literal -1
    fk_j1..fk_j6       UF850's limits baked into the slider templates as
                       strict ranges, so any other robot was clamped to them

Here they are EXPRESSIONS on the internal nodes, evaluated against the
current Robot Profile, rather than values a callback writes. That is not a
style choice: a locked instance forbids writes to its internal parameters
(see ctrl_recache_btn.py), so a callback that rewrote them would only ever
work on an unlocked copy.

What a callback still does -- on_profile_changed() -- touches only promoted
parameters, which locked instances allow.

Expressions call in through the parent, because an internal node is not an
asset and has no hdaModule of its own:

    hou.pwd().parent().hdaModule().weight(hou.pwd(), 5, "y")
"""
import math
import os
import sys

import hou

ASSET_TYPE = "wenyi::robot_arm::1.0"
_CACHE = {}


def _root():
    return hou.expandString("$HIP/..")


def _robot_profile():
    """scripts/robot_profile.py, reloaded when the file changes. Python keeps
    the first import for the whole session: after acceleration_limits() was
    added, a session that had imported the module earlier did not have it,
    and Max Joint Acceleration's default evaluated to an error (0)."""
    scripts = _root() + "/scripts"
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    import importlib
    import robot_profile
    mtime = os.path.getmtime(robot_profile.__file__)
    if getattr(robot_profile, "_loaded_mtime", None) != mtime:
        robot_profile = importlib.reload(robot_profile)
        robot_profile._loaded_mtime = mtime
    return robot_profile


def asset_of(node):
    """The robot_arm instance that owns node (node itself if it is one)."""
    cur = node
    while cur is not None and cur.type().name() != ASSET_TYPE:
        cur = cur.parent()
    if cur is None:
        raise hou.NodeError("%s is not inside a %s" % (node.path(), ASSET_TYPE))
    return cur


def profile(node):
    """The asset's current profile. Cached per file and modification time, so
    editing the JSON takes effect on the next cook without reloading."""
    asset = asset_of(node)
    pid = asset.parm("robot_profile").eval() or "uf850"
    directory = os.path.join(_root(), "profiles")
    path = os.path.join(directory, pid + ".json")
    if not os.path.isfile(path):
        raise hou.NodeError("Robot Profile '%s' has no %s" % (pid, path))
    key = (path, os.path.getmtime(path))
    prof = _CACHE.get(key)
    if prof is None:
        prof = _robot_profile().load(pid, directory)
        _CACHE[key] = prof
    return prof


def _axis(prof, joint):
    return prof["rig"]["rotation_axis"][joint - 1]


def _preset_joints(prof):
    return {g["joint"] for g in prof["presets"].values()
            if isinstance(g, dict) and "joint" in g}


# --------------------------------------------------------------------------
# configurejoints1
# --------------------------------------------------------------------------

def weight(node, joint, axis):
    """Rotation weight of joint (1-based) about axis. Only the profile's axis
    moves. J4 keeps its Roll Freedom control: FBIK ignores J4's limits on
    both robots measured so far, so the weight is the only thing that binds."""
    asset = asset_of(node)
    prof = profile(asset)
    if _axis(prof, joint) != axis:
        return 0.0
    if joint == 4:
        return asset.node("TCP_PATH_CTRL").evalParm("cfg_j4_weight")
    return 1.0


def range_limit(node, joint, axis, which):
    """Configure Joints range for joint about axis; which is 'min' or 'max'.

    Off-axis ranges are pinned at zero. On the axis, a joint the profile's
    presets gate (shoulder/elbow/wrist) takes the preset range the
    Configuration menus wrote to cfg_jN_min/max; every other joint takes its
    full profile limit, converted to the configurejoints frame.
    """
    asset = asset_of(node)
    prof = profile(asset)
    if _axis(prof, joint) != axis:
        return 0.0
    lo = hi = None
    if joint in _preset_joints(prof):
        tcp = asset.node("TCP_PATH_CTRL")
        p_lo, p_hi = tcp.parm("cfg_j%d_min" % joint), tcp.parm("cfg_j%d_max" % joint)
        if p_lo is not None and p_hi is not None:
            lo, hi = p_lo.eval(), p_hi.eval()
    if lo is None:
        lo, hi = _robot_profile().joint_limits_configurejoints(prof, joint)
    lo, hi = fbik_range(lo, hi)
    return lo if which == "min" else hi


FBIK_EDGE = 179.0


def fbik_range(lo, hi):
    """A joint range as Full Body IK can use it.

    FBIK reads a range as Euler degrees within +-180. One that runs past that
    edge without being a full turn -- FR20's J2 and J4, [-265, 85] -- jams the
    solver: J2 sat at 5 deg on every frame of a curve and the tip missed by
    200-800 mm, while the same solve with J2 from -179 tracked every sampled
    frame exactly. So such a range keeps only its part inside the edge.
    -180 itself still failed one frame in nine, hence the 1 deg margin.

    A range of a full turn or more (UF850's +-360) means unlimited and passes
    through unchanged, as does any range already inside the edge."""
    if hi - lo >= 360.0 or (lo >= -FBIK_EDGE and hi <= FBIK_EDGE):
        return lo, hi
    return max(lo, -FBIK_EDGE), min(hi, FBIK_EDGE)


# --------------------------------------------------------------------------
# FK
# --------------------------------------------------------------------------

def fk_value(node, joint):
    """Joint_controller jN: the typed FK angle, robot frame, clamped to the
    profile's limits. The slider templates no longer carry one robot's
    limits, so the clamp lives here.

    With Pose Source on Imported CSV it is the imported file's angle at this
    frame instead (import_value) -- read live, not keyed onto a node, so it
    works on a locked instance -- and not clamped: Import checks the file's
    limits, and a clamp would export something other than the file."""
    asset = asset_of(node)
    if int(asset.evalParm("pose_source")) == 2:
        v = import_value(asset, joint)
        if v is not None:
            return v
    lo, hi = profile(asset)["robot"]["limits_deg"][joint - 1]
    return min(max(asset.evalParm("fk_j%d" % joint), lo), hi)


# --------------------------------------------------------------------------
# Imported CSV: the file, read live
# --------------------------------------------------------------------------

_IMPORT = {}


def read_clip(path):
    """(times, joints) of a joint CSV (frame, time_s, j1_deg..) or a clip
    JSON (motionlab.clip/1); times from the first row, degrees, robot frame."""
    import csv as _csv
    import json as _json
    if path.lower().endswith(".json"):
        with open(path) as f:
            c = _json.load(f)
        t = [float(p["t"]) for p in c["points"]]
        q = [[float(x) for x in p["q"]] for p in c["points"]]
    else:
        with open(path, newline="") as f:
            rows = list(_csv.DictReader(f))
        if not rows:
            raise ValueError("%s has no rows" % path)
        cols = sorted((c for c in rows[0] if c and c.startswith("j") and c.endswith("_deg")),
                      key=lambda c: int(c[1:-4]))
        if not cols:
            raise ValueError("%s has no j1_deg .. columns" % path)
        if "time_s" in rows[0]:
            t = [float(r["time_s"]) for r in rows]
        else:
            t = [i / hou.fps() for i in range(len(rows))]
        q = [[float(r[c]) for c in cols] for r in rows]
    if len(t) < 1:
        raise ValueError("%s is empty" % path)
    t0 = t[0]
    t = [x - t0 for x in t]
    # a fixed frame rate written with 4 decimals (1/24 -> 0.0417): exact
    # frame times, as fairino_player reads them
    n = len(t)
    if n > 2:
        dt = sum(i * t[i] for i in range(n)) / float(sum(i * i for i in range(n)))
        if dt > 0 and all(abs(t[i] - i * dt) < 1e-4 for i in range(n)):
            fps = round(1.0 / dt)
            if fps and abs(1.0 / dt - fps) < 0.01:
                dt = 1.0 / fps
            t = [i * dt for i in range(n)]
    return t, q


def import_clip(node):
    """The file named in Import CSV, cached by path and modification time --
    replace the file and the next cook reads the new one. None when unset."""
    asset = asset_of(node)
    p = asset.parm("import_csv")
    path = p.eval().strip() if p is not None else ""
    if not path:
        return None
    if not os.path.isfile(path):
        raise hou.NodeError("Import CSV: no file at %s" % path)
    key = (path, os.path.getmtime(path))
    hit = _IMPORT.get(asset.path())
    if hit is None or hit[0] != key:
        hit = (key, read_clip(path))
        _IMPORT[asset.path()] = hit
    return hit[1]


def import_value(node, joint, frame=None):
    """The imported file's angle of joint (1-based) at frame: frame Start
    Frame is the file's first row, then by its time_s, linear between rows,
    held at both ends."""
    clip = import_clip(node)
    if clip is None:
        return None
    t, q = clip
    asset = asset_of(node)
    sp = asset.parm("import_start")
    start = sp.eval() if sp is not None else 1
    s = ((frame if frame is not None else hou.frame()) - start) / hou.fps()
    if s <= t[0] or len(t) == 1:
        return q[0][joint - 1]
    if s >= t[-1]:
        return q[-1][joint - 1]
    import bisect
    i = bisect.bisect_right(t, s) - 1
    f = (s - t[i]) / (t[i + 1] - t[i])
    return q[i][joint - 1] + f * (q[i + 1][joint - 1] - q[i][joint - 1])


def fk_rotation(node, joint, axis):
    """rigpose_fk rotation of joint about axis: robot angle * profile sign on
    the profile's axis, zero on the others."""
    asset = asset_of(node)
    prof = profile(asset)
    if _axis(prof, joint) != axis:
        return 0.0
    angle = asset.node("Joint_controller").evalParm("j%d" % joint)
    return angle * float(prof["rig"]["sign"][joint - 1])


# --------------------------------------------------------------------------
# tool frame
# --------------------------------------------------------------------------

def flange_offset(node):
    """TCP_PATH_CTRL flange_offset: metres along joint_6's +Y from the joint
    to the tool mounting face. The tool-frame VEX (tool_goal_offset,
    tool_tip, tool_mount) and joint_angles add it before any tool, so a
    robot whose last joint sits behind its face (FR20, 0.12 m) reaches with
    the face. Profiles without the field mean zero -- joint_6 is the face,
    which is what the asset assumed before this value existed."""
    return float(profile(node)["rig"].get("flange_offset_m", 0.0))


# --------------------------------------------------------------------------
# where the rig and the mesh come from
# --------------------------------------------------------------------------
#
# A profile names one source for the robot's body:
#   rig.fbx   skinned FBX (UF850): fbxcharacterimport1 gives the rest
#             skeleton and bonedeform2 the mesh
#   rig.urdf  vendor URDF (FR20): urdf_skeleton builds the rest skeleton and
#             urdf_links places one rigid STL per link; urdf_drive moves each
#             link with its joint
# The FBX wins when a profile has both, since its skin was made for it. A
# skeleton wired into input 0 overrides either for the rig, not the mesh.

def _urdf_rig():
    _robot_profile()  # puts <toolkit>/scripts on sys.path
    import urdf_rig
    return urdf_rig


def _uses_urdf(prof):
    return bool(prof["rig"].get("urdf")) and prof["rig"].get("fbx") is None


def urdf_model(node):
    """The current profile's parsed URDF and its package root, or None when
    the profile has no URDF. Cached per file and modification time.

    Mesh paths are package:// URIs, resolved the ROS way: the URDF sits at
    <package_root>/<package>/urdf/<file>.urdf."""
    rel = profile(node)["rig"].get("urdf")
    if not rel:
        return None
    path = os.path.normpath(os.path.join(_root(), rel))
    if not os.path.isfile(path):
        raise hou.NodeError("Robot Profile's rig.urdf does not exist: %s" % path)
    key = ("urdf", path, os.path.getmtime(path))
    model = _CACHE.get(key)
    if model is None:
        model = {"parsed": _urdf_rig().parse_urdf(path),
                 "package_root": os.path.dirname(os.path.dirname(os.path.dirname(path)))}
        _CACHE[key] = model
    return model


def skel_source(node):
    """SKEL_SOURCE input: 0 FBX, 1 the skeleton wired into input 0, 2 URDF."""
    asset = asset_of(node)
    if asset.evalParm("in_skel_wired"):
        return 1
    return 2 if _uses_urdf(profile(asset)) else 0


def mesh_source(node):
    """ROBOT_MESH_SOURCE input: 0 the FBX skin, 1 the URDF links."""
    return 1 if _uses_urdf(profile(node)) else 0


def cook_urdf_skeleton(node):
    """urdf_skeleton (Python SOP): the rest skeleton built from the URDF at
    its zero pose. Empty when the profile has no URDF."""
    model = urdf_model(node)
    if model is None:
        return
    rig = profile(node)["rig"]
    ur = _urdf_rig()
    ur.write_skeleton(node.geometry(), ur.build_rest_skeleton(
        model["parsed"]["chain"], flange_offset(node), rig["joint_name_pattern"]))


def cook_urdf_links(node):
    """urdf_links (Python SOP): every link's STL at the URDF zero pose, with a
    primitive 'name' of the joint that moves it ('base' for the static link).
    Empty when the profile has no URDF."""
    model = urdf_model(node)
    if model is None:
        return
    parsed = model["parsed"]
    ur = _urdf_rig()
    placements = ur.link_placements(parsed["chain"], parsed["links"],
                                    model["package_root"], parsed["root_link"])
    missing = [p[1] for p in placements if not os.path.isfile(p[1])]
    if missing:
        raise hou.NodeError("URDF link meshes not found: %s" % ", ".join(missing))
    ur.write_link_meshes(node.geometry(), placements,
                         profile(node)["rig"]["joint_name_pattern"])


# --------------------------------------------------------------------------
# IK solver: FBIK or closed form
# --------------------------------------------------------------------------
#
# rig.ik_solver in the profile picks the solver:
#   "fbik"            KineFX Full Body IK (default; UF850)
#   "ur_closed_form"  scripts/ur_ik.py, for UR-type arms (FR20)
#
# The closed form runs in analytic_ik (Python SOP), which only computes
# joint angles and writes them as detail attributes. ik_rigpose -- a copy of
# rigpose_fk -- then poses the skeleton from them, so the IK result goes
# through the same KineFX machinery as FK and everything downstream
# (measure_residual, joint_angles, the cache, export) is unchanged.
#
# Branch choice, per frame: the Solve tab's shoulder / elbow / wrist presets
# filter the up to 8 branches (the usual industrial configuration flags);
# of those left, the one nearest the previous frame's solution wins, else the
# one nearest the profile's rig.ik_reference_deg. The previous-frame memory
# is per asset; Recache clears it, and a cache is written in frame order, so
# a cached solve is continuous and reproducible.

_IK_MEMO = {}
DEFAULT_IK_REFERENCE = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]


def ik_solver_index(node):
    """IK_SOLVER input: 0 FBIK, 1 closed form."""
    return 1 if profile(node)["rig"].get("ik_solver") == "ur_closed_form" else 0


def _ur_ik():
    _robot_profile()
    import ur_ik
    return ur_ik


def _ur_model(node):
    """ur_ik.analyse() of the profile's URDF, cached like urdf_model()."""
    model = urdf_model(node)
    if model is None:
        raise hou.NodeError("rig.ik_solver is ur_closed_form but the profile has no rig.urdf")
    path = os.path.normpath(os.path.join(_root(), profile(node)["rig"]["urdf"]))
    key = ("ur_ik", path, os.path.getmtime(path))
    m = _CACHE.get(key)
    if m is None:
        m = _ur_ik().analyse(model["parsed"]["chain"])
        _CACHE[key] = m
    return m


def cook_cell_env(node):
    """cell_env (Python SOP): the room from Setup > Cell Environment, drawn by
    scripts/cell_sop.py -- obstacles grey, keep-out red, slow orange, the
    stage green, tall walls see-through. Empty when no file is set."""
    geo = node.geometry()
    geo.clear()
    p = asset_of(node).parm("env_file")
    path = p.eval().strip() if p is not None else ""
    if not path or not os.path.exists(path):
        return
    scripts = _root() + "/scripts"
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    import cell_sop
    import collision
    cell_sop.env_geometry(geo, collision.load_env(path))


def clear_ik_memo(asset):
    """Forget previous-frame solutions for this asset (Recache calls it)."""
    for k in [k for k in _IK_MEMO if k[0] == asset.path()]:
        del _IK_MEMO[k]


def _preset_ranges(asset, prof):
    """{joint: (lo, hi)} from the Solve tab presets, robot frame."""
    ctrl = asset.node("TCP_PATH_CTRL")
    out = {}
    for j in _preset_joints(prof):
        lo, hi = ctrl.parm("cfg_j%d_min" % j), ctrl.parm("cfg_j%d_max" % j)
        if lo is not None and hi is not None:
            out[j] = (lo.eval(), hi.eval())
    return out


def cook_analytic_ik(node):
    """analytic_ik (Python SOP). Input 0: the rest skeleton; input 1: the
    joint_6 goal from tool_goal_offset (P, 3@transform). Passes the skeleton
    through and writes detail attributes:

        ik_q         6 floats, robot frame, degrees
        ik_ok        1 when a solution within limits and presets exists
        ik_singular  1 at the wrist singularity (q5 ~ 0)
        ik_nsol      branches that reproduced the goal, before filtering
        ik_branch    (shoulder, wrist, elbow) as +1/-1
    """
    geo = node.geometry()
    asset = asset_of(node)
    prof = profile(asset)
    for name, default in (("ik_q", (0.0,) * 6), ("ik_branch", (0, 0, 0))):
        if geo.findGlobalAttrib(name) is None:
            geo.addAttrib(hou.attribType.Global, name, default)
    for name in ("ik_ok", "ik_singular", "ik_nsol"):
        if geo.findGlobalAttrib(name) is None:
            geo.addAttrib(hou.attribType.Global, name, 0)
    if ik_solver_index(asset) != 1:
        return

    K = _ur_ik()
    model = _ur_model(asset)
    chain = model["chain"]
    rest6 = [p for p in geo.points() if p.attribValue("name") == "joint_6"]
    goal_geo = node.inputs()[1].geometry() if len(node.inputs()) > 1 and node.inputs()[1] else None
    if not rest6 or goal_geo is None or not goal_geo.points():
        raise hou.NodeError("analytic_ik needs joint_6 on input 0 and a goal point on input 1")
    t = rest6[0].attribValue("transform")
    rest_rows = (tuple(t[0:3]), tuple(t[3:6]), tuple(t[6:9]))
    gp = goal_geo.points()[0]
    g = gp.attribValue("transform")
    goal_rows = (tuple(g[0:3]), tuple(g[3:6]), tuple(g[6:9]))
    R6, p6 = K.urdf_pose_from_kinefx_full(chain, goal_rows, tuple(gp.position()), rest_rows)

    frame = round(hou.frame(), 4)
    reference = _IK_MEMO.get((asset.path(), round(frame - 1, 4)))
    if reference is None:
        # floats: JSON gives ints, and ik_q is a float attribute
        reference = [float(x) for x in prof["rig"].get("ik_reference_deg", DEFAULT_IK_REFERENCE)]

    raw = K.solve(model, R6, p6, q6_when_singular=reference[5])
    sols = K.within_limits(model, raw)
    for j, (lo, hi) in _preset_ranges(asset, prof).items():
        sols = [s for s in sols if lo - 1e-9 <= s["q"][j - 1] <= hi + 1e-9]
    best = K.nearest(sols, reference)

    geo.setGlobalAttribValue("ik_nsol", len(raw))
    if best is None:
        # No admissible branch -- out of reach, or every branch outside the
        # limits / presets. Get as near as the arm can from the previous
        # frame's pose (continuous), and say so: ik_ok 0, and
        # measure_residual shows the miss, as it does for FBIK.
        q, _, _ = K.closest(model, R6, p6, reference)
        geo.setGlobalAttribValue("ik_q", tuple(float(x) for x in q))
        geo.setGlobalAttribValue("ik_ok", 0)
        _IK_MEMO[(asset.path(), frame)] = list(q)
        return
    geo.setGlobalAttribValue("ik_q", tuple(best["q"]))
    geo.setGlobalAttribValue("ik_ok", 1)
    geo.setGlobalAttribValue("ik_singular", 1 if best["singular"] else 0)
    geo.setGlobalAttribValue("ik_branch", tuple(best["branch"]))
    _IK_MEMO[(asset.path(), frame)] = list(best["q"])


def ik_rotation(node, joint, axis):
    """ik_rigpose rotation of joint about axis: analytic_ik's angle * profile
    sign on the profile's axis, zero on the others -- fk_rotation's rule.

    Read from ik_rigpose's own input (analytic_ik is wired into it), so the
    dependency is a wire Houdini tracks, not a side read."""
    asset = asset_of(node)
    prof = profile(asset)
    if _axis(prof, joint) != axis:
        return 0.0
    src = node.inputs()[0] if node.inputs() else None
    if src is None or src.geometry().findGlobalAttrib("ik_q") is None:
        return 0.0
    q = src.geometry().attribValue("ik_q")
    return q[joint - 1] * float(prof["rig"]["sign"][joint - 1])


# --------------------------------------------------------------------------
# joint velocity limits
# --------------------------------------------------------------------------

def velocity_limits(node):
    """Effective per-joint velocity limits, deg/s: the profile's (one per
    joint -- FR20's base three are 120, its wrist 180) capped by the asset's
    Max Joint Velocity, which on_profile_changed() sets to the profile's
    highest so it binds only when lowered by hand."""
    asset = asset_of(node)
    per_joint = _robot_profile().velocity_limits(profile(asset))
    cap = asset.parm("max_velocity")
    cap = float(cap.eval()) if cap is not None else max(per_joint)
    return [min(v, cap) for v in per_joint]


def acceleration_limits(node):
    """Per-joint acceleration limits, deg/s^2: the profile's
    (robot.max_acceleration_deg_s2, one per joint), capped by the asset's
    Max Joint Acceleration -- like velocity. Its default is the profile's
    highest, so it does not bind unless lowered. None without a profile
    figure. Retime plans with these, Pre-Flight checks them, and
    fairino_player.py plays with the profile's -- the same numbers."""
    asset = asset_of(node)
    acc = _robot_profile().acceleration_limits(profile(asset))
    p = asset.parm("max_acceleration")
    cap = float(p.eval()) if p is not None else 0.0
    if acc is None:
        return [cap] * profile(asset)["robot"]["num_joints"] if cap > 0 else None
    return [min(a, cap) for a in acc] if cap > 0 else acc


def acc_limit(node, joint):
    """Joint `joint`'s (1-based) acceleration limit, for the Retime folder's
    read-only per-joint display; 0 without one."""
    acc = acceleration_limits(node)
    return acc[joint - 1] if acc else 0.0


def profile_max_velocity(node):
    """Max Joint Velocity's default: the profile's highest per-joint limit,
    so the cap does not bind unless someone lowers it."""
    return max(_robot_profile().velocity_limits(profile(node)))


def profile_max_acceleration(node):
    """Max Joint Acceleration's default: the profile's limit. A literal
    default (150, FR20's) left UF850 instances at FR20's number, since only
    a profile CHANGE used to write it."""
    acc = _robot_profile().acceleration_limits(profile(node))
    return max(acc) if acc else 0.0


def vel_limit(node, joint):
    """path_metrics vel_limitN: joint's effective limit (1-based)."""
    return velocity_limits(node)[joint - 1]


def vel_limit_expr(joint):
    return CALL + "vel_limit(hou.pwd(), %d)" % joint


# --------------------------------------------------------------------------
# UI
# --------------------------------------------------------------------------

def profile_menu():
    """Robot Profile menu: every profiles/*.json next to the scene."""
    directory = os.path.join(_root(), "profiles")
    items = []
    if os.path.isdir(directory):
        for fn in sorted(os.listdir(directory)):
            if fn.endswith(".json"):
                pid = fn[:-5]
                items += [pid, pid]
    return items


def on_profile_changed(asset):
    """Runs when Robot Profile changes and when an instance is created.

    Writes only promoted parameters -- allowed on a locked instance:
      * invert_jN from the profile's sign, which joint_angles.py checks;
      * Robot Mesh on when the profile has a body to show (FBX skin or URDF
        links), off when it has neither;
      * Max Joint Velocity / Acceleration back to their defaults, which
        read the profile;
      * the Configuration presets reset through cfg_reset, which reloads the
        preset ranges from the new profile.
    """
    prof = profile(asset)
    for k, s in enumerate(prof["rig"]["sign"], start=1):
        p = asset.parm("invert_j%d" % k)
        if p is not None:
            p.set(1 if int(s) == -1 else 0)
    if asset.parm("show_robot") is not None:
        has_body = prof["rig"].get("fbx") is not None or bool(prof["rig"].get("urdf"))
        asset.parm("show_robot").set(1 if has_body else 0)
    # Max Joint Velocity / Acceleration default to expressions that read the
    # profile (profile_max_velocity / profile_max_acceleration); reverting
    # puts the expression back where a typed value had replaced it
    for name in ("max_velocity", "max_acceleration"):
        p = asset.parm(name)
        if p is not None:
            p.revertToDefaults()
    reset = asset.parm("cfg_reset")
    if reset is not None:
        reset.pressButton()


# --------------------------------------------------------------------------
# expression text, used when installing the module into the definition
# --------------------------------------------------------------------------

CALL = "hou.pwd().parent().hdaModule()."


def weight_expr(joint, axis):
    return CALL + 'weight(hou.pwd(), %d, "%s")' % (joint, axis)


def range_expr(joint, axis, which):
    return CALL + 'range_limit(hou.pwd(), %d, "%s", "%s")' % (joint, axis, which)


def fk_value_expr(joint):
    return CALL + "fk_value(hou.pwd(), %d)" % joint


def fk_rotation_expr(joint, axis):
    return CALL + 'fk_rotation(hou.pwd(), %d, "%s")' % (joint, axis)


def flange_offset_expr():
    return CALL + "flange_offset(hou.pwd())"


def skel_source_expr():
    return CALL + "skel_source(hou.pwd())"


def mesh_source_expr():
    return CALL + "mesh_source(hou.pwd())"


def ik_solver_expr():
    return CALL + "ik_solver_index(hou.pwd())"


def ik_rotation_expr(joint, axis):
    return CALL + 'ik_rotation(hou.pwd(), %d, "%s")' % (joint, axis)


ANALYTIC_IK_CODE = "hou.pwd().parent().hdaModule().cook_analytic_ik(hou.pwd())\n"


# Python SOP code for urdf_skeleton / urdf_links
URDF_SKELETON_CODE = "hou.pwd().parent().hdaModule().cook_urdf_skeleton(hou.pwd())\n"
URDF_LINKS_CODE = "hou.pwd().parent().hdaModule().cook_urdf_links(hou.pwd())\n"


# --------------------------------------------------------------------------
# Curve Check (Analyze): the goal curve, before any timing
# --------------------------------------------------------------------------

_CURVE_CHECK = {}


def _progress_state(parm):
    """Everything needed to put Progress back as it was."""
    keys = parm.keyframes()
    if keys:
        return ("keys", keys)
    try:
        return ("expr", parm.expression(), parm.expressionLanguage())
    except hou.OperationFailed:
        return ("value", parm.eval())


def _restore_progress(parm, state):
    parm.deleteAllKeyframes()
    if state[0] == "keys":
        parm.setKeyframes(state[1])
    elif state[0] == "expr":
        parm.setExpression(state[1], state[2])
    else:
        parm.set(state[1])


def curve_check(asset):
    """Analyze > Curve Check: walk Progress 0 -> 1 through the real solve
    (orient mode, tool, presets as they are) and measure, per sample, what
    the pose leaves for the robot -- independent of any timing:

        reachable   the closed-form IK found an in-limit pose
        headroom    m/s: fastest TCP speed, worst direction, before a joint
                    hits its velocity limit (scripts/capability.py)
        wrist       |sin q5|: 0 at the wrist singularity
        margin      degrees to the nearest joint limit

    Progress is put back exactly as it was. FR20 (closed-form IK) only."""
    asset = asset_of(asset)
    if ik_solver_index(asset) != 1:
        raise hou.NodeError("Curve Check needs the closed-form IK (FR20)")
    scripts = _root() + "/scripts"
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    import capability
    prof = profile(asset)
    model = _ur_model(asset)
    chain = model["chain"]
    fo = float(prof["rig"].get("flange_offset_m", 0.0))
    ctrl = asset.node("TCP_PATH_CTRL")
    tool = ctrl.parmTuple("tool_offset").eval() if ctrl is not None and ctrl.parmTuple("tool_offset") else (0, 0, 0)
    tcp_local = (0.0, 0.0, fo + (sum(x * x for x in tool) ** 0.5))
    vel = velocity_limits(asset)
    lim = model["limits"]
    n = max(20, int(asset.evalParm("curve_check_samples")))
    prog = asset.parm("progress")
    state = _progress_state(prog)
    aik, goal = asset.node("analytic_ik"), asset.node("GOAL_MODE")
    key_prev = (asset.path(), round(hou.frame() - 1, 4))
    rows = []
    try:
        with hou.InterruptableOperation("Curve Check", open_interrupt_dialog=True) as op:
            prog.deleteAllKeyframes()
            for i in range(n + 1):
                u = i / float(n)
                prog.set(u)
                g = aik.geometry()
                ok = bool(g.attribValue("ik_ok"))
                q = list(g.attribValue("ik_q"))
                _IK_MEMO[key_prev] = q                 # the next sample continues from this one
                P = tuple(goal.geometry().points()[0].position())
                row = {"u": u, "P": P, "reachable": ok, "headroom": 0.0, "wrist": 0.0, "margin": 0.0}
                if ok:
                    row["headroom"] = capability.headroom(chain, q, tcp_local, vel)
                    row["wrist"] = abs(math.sin(math.radians(q[4])))
                    row["margin"] = min(min(x - lo, hi - x) for x, (lo, hi) in zip(q, lim))
                rows.append(row)
                op.updateProgress(i / float(n))
    finally:
        _restore_progress(prog, state)
        clear_ik_memo(asset)
    _CURVE_CHECK[asset.path()] = rows
    bad = [r for r in rows if not r["reachable"]]
    ok_rows = [r for r in rows if r["reachable"]]
    parts = ["%d samples" % len(rows)]
    if bad:
        parts.append("UNREACHABLE at %d (u %.2f .. %.2f)" % (len(bad), bad[0]["u"], bad[-1]["u"]))
    if ok_rows:
        h = min(ok_rows, key=lambda r: r["headroom"])
        w = min(ok_rows, key=lambda r: r["wrist"])
        m = min(ok_rows, key=lambda r: r["margin"])
        parts.append("min headroom %.2f m/s at u %.2f" % (h["headroom"], h["u"]))
        parts.append("closest to the wrist singularity %.0f deg at u %.2f"
                     % (math.degrees(math.asin(min(1.0, w["wrist"]))), w["u"]))
        parts.append("joint-limit margin %.0f deg at u %.2f" % (m["margin"], m["u"]))
    asset.parm("curve_check_status").set("; ".join(parts))
    node = asset.node("curve_check")
    if node is not None:
        node.cook(force=True)
    return rows


def cook_curve_check(node):
    """curve_check (Python SOP): the last Curve Check as a coloured polyline --
    red where unreachable, else green .. red by the worst of headroom (full at
    1.2 m/s), wrist (full at 20 deg from the singularity) and joint-limit
    margin (full at 20 deg). Point attributes u, reachable, headroom, wrist,
    margin, risk (0 fine .. 1 at a limit)."""
    geo = node.geometry()
    geo.clear()
    rows = _CURVE_CHECK.get(asset_of(node).path())
    if not rows:
        return
    for name, default in (("Cd", (1.0, 1.0, 1.0)), ("u", 0.0), ("reachable", 0), ("headroom", 0.0),
                          ("wrist", 0.0), ("margin", 0.0), ("risk", 1.0)):
        geo.addAttrib(hou.attribType.Point, name, default)
    poly = geo.createPolygon(is_closed=False)
    wfull = math.sin(math.radians(20.0))
    for r in rows:
        p = geo.createPoint()
        p.setPosition(r["P"])
        good = min(1.0, r["headroom"] / 1.2, r["wrist"] / wfull, r["margin"] / 20.0) if r["reachable"] else 0.0
        c = hou.Color()
        c.setHSV((120.0 * good if r["reachable"] else 0.0, 0.9, 1.0 if r["reachable"] else 0.55))
        p.setAttribValue("Cd", c.rgb())
        for k in ("u", "headroom", "wrist", "margin"):
            p.setAttribValue(k, float(r[k]))
        p.setAttribValue("reachable", int(r["reachable"]))
        p.setAttribValue("risk", 1.0 - good)
        poly.addVertex(p)
