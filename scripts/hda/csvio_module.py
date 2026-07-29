"""Robot animation CSV export/import.

Exports joint angles read from a solved KineFX skeleton, so it works for
FK, IK, or any combination -- nothing is read from parameter channels.

Angle extraction, validated to 2e-06 deg against reference exports:
  Lr    = rest_transform * parent_rest_transform^-1     (rest local rotation)
  L     = transform      * parent_transform^-1          (posed local rotation)
  D     = Lr^-1 * L                                     (rest -> posed delta)
  axis  = -(parm_axis * Lr)   where parm_axis comes from the joint's
          fbik_jointconfig rotation_weights
  angle = atan2(sin, cos) from D, signed by dot(D_axis, axis)

atan2 rather than acos: acos is ill-conditioned near 0 and 180 deg and
costs ~1e-3 deg on small rotations, which then compounds through the
velocity calculation.
"""

import csv
import math
import os
import sys

import hou

# Joint count and limits come from profiles/<id>.json, which is also what the
# preset callback and the joint-angle SOP read. They were three separate
# literal tables until 2026-07-29 and had already drifted: this copy said
# (-241.9, 3.4) and +-123.9 while the preset callback said (-242.0, 3.5) and
# +-124.0, in a different reference frame. The preset copy was additionally
# wrong by 148 degrees on J3. One table, converted by robot_profile, or the
# same class of bug comes back.
def _toolkit_root():
    """Find the toolkit root: the directory holding profiles/ and scripts/.

    Resolved from this asset's own .hdalc rather than from $HIP, so the asset
    keeps working when instanced in someone else's scene. $HIP is only a
    fallback for the case where the definition is embedded in the hip file
    and has no library path on disk.
    """
    cands = []
    try:
        d = hou.nodeType(hou.sopNodeTypeCategory(),
                         "wenyi::robot_anim_csv_io::1.0").definition()
        if d and d.libraryFilePath():
            cands.append(os.path.dirname(d.libraryFilePath()))
    except Exception:
        pass
    cands.append(hou.expandString("$HIP"))
    for start in cands:
        cur = os.path.abspath(start)
        for _ in range(5):
            if os.path.isdir(os.path.join(cur, "profiles")) and \
                    os.path.isdir(os.path.join(cur, "scripts")):
                return cur
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    raise hou.Error("Cannot locate the toolkit root (a directory containing "
                    "both profiles/ and scripts/) from %r" % cands)


_ROOT = _toolkit_root()
if os.path.join(_ROOT, "scripts") not in sys.path:
    sys.path.insert(0, os.path.join(_ROOT, "scripts"))
import robot_profile


def _profile(node=None):
    """Resolved per call so switching the Robot Profile menu takes effect
    without reloading the asset."""
    pid = None
    if node is not None:
        p = node.parm("robot_profile")
        if p is not None:
            pid = p.eval()
    if not pid:
        ctrl = hou.node("/obj/geo1/TCP_PATH_CTRL")
        if ctrl is not None and ctrl.parm("robot_profile") is not None:
            pid = ctrl.parm("robot_profile").eval()
    return robot_profile.load(pid or "uf850", os.path.join(_ROOT, "profiles"))


PROFILE = _profile()
JOINT_LIMITS_DEG = [tuple(r) for r in PROFILE["robot"]["limits_deg"]]
NUM_JOINTS = PROFILE["robot"]["num_joints"]
BASE_HEADERS = ["frame", "time_s"] + ["j%d_deg" % n for n in range(1, NUM_JOINTS + 1)] + ["speed_pct"]


# Set True to suppress modal dialogs. hou.ui.displayMessage blocks Houdini's
# main thread until a human clicks it, which deadlocks any scripted or
# headless run of these functions.
_QUIET = False


def _notify(text, severity=None, title=None):
    if _QUIET or not hou.isUIAvailable():
        return
    hou.ui.displayMessage(text,
                          severity=severity or hou.severityType.Message,
                          title=title or "Robot Anim CSV IO")


def _m4(nine):
    v = list(nine)
    return hou.Matrix4([v[0], v[1], v[2], 0.0,
                        v[3], v[4], v[5], 0.0,
                        v[6], v[7], v[8], 0.0,
                        0.0, 0.0, 0.0, 1.0])


def _parse_weights(raw):
    """fbik_jointconfig stores rotation_weights as the string '[0, 1, 0]'."""
    if isinstance(raw, (list, tuple)):
        vals = list(raw)
    else:
        vals = [float(x) for x in str(raw).strip("[]").split(",")]
    return [abs(float(x)) for x in vals]


def axis_map_from_geo(geo):
    """{joint number: 'x'|'y'|'z'} from fbik_jointconfig, or {} if absent."""
    out = {}
    if geo is None or geo.findPointAttrib("fbik_jointconfig") is None:
        return out
    for p in geo.points():
        nm = p.attribValue("name")
        for jn in range(1, NUM_JOINTS + 1):
            if nm == "joint_%d" % jn:
                w = _parse_weights(p.attribValue("fbik_jointconfig").get(
                    "rotation_weights", "[0, 0, 1]"))
                out[jn] = "xyz"[w.index(max(w))]
    return out


def extract_angles(geo, axis_of=None):
    """Return [j1..j6] degrees from a posed skeleton, or raise ValueError.

    axis_of supplies each joint's rotation axis. It is passed in rather than
    read from `geo` because an FK Rig Pose is typically fed from the rest
    null, which sits upstream of Configure Joints and therefore carries no
    fbik_jointconfig -- only the IK branch does.
    """
    pts = list(geo.points())
    by_name = {}
    for p in pts:
        by_name[p.attribValue("name")] = p

    for req in ("transform", "rest_transform", "parent_idx", "name"):
        if geo.findPointAttrib(req) is None:
            raise ValueError(
                "Input skeleton is missing the '%s' point attribute.\n"
                "Wire a KineFX skeleton (e.g. the Full Body IK output) into input 1." % req)

    if not axis_of:
        axis_of = axis_map_from_geo(geo)

    # Never guess the rotation axis. Guessing produces correct magnitudes with
    # randomly flipped signs -- output that looks plausible and drives the arm
    # backwards. Fail loudly instead.
    if not axis_of:
        raise ValueError(
            "Could not determine the joints' rotation axes: neither the input\n"
            "skeleton nor any upstream Configure Joints SOP provided\n"
            "'fbik_jointconfig'.\n\n"
            "Without it the exported signs would be unreliable.")

    angles = []

    for jn in range(1, NUM_JOINTS + 1):
        nm = "joint_%d" % jn
        if nm not in by_name:
            raise ValueError("Joint '%s' not found in the input skeleton." % nm)
        p = by_name[nm]
        par = p.attribValue("parent_idx")

        W = _m4(p.attribValue("transform"))
        Wr = _m4(p.attribValue("rest_transform"))
        if par is not None and par >= 0:
            Wp = _m4(pts[par].attribValue("transform"))
            Wpr = _m4(pts[par].attribValue("rest_transform"))
        else:
            Wp = hou.Matrix4(1)
            Wpr = hou.Matrix4(1)

        Lr = Wr * Wpr.inverted()
        D = Lr.inverted() * (W * Wp.inverted())

        # The joint's rotation axis, expressed in the delta frame. Closed form:
        # negate the configured axis and carry it through the rest local
        # rotation. Verified dot == -1.0 against a probe on all six joints,
        # including joint_4 whose configured Y maps onto the delta frame's X.
        ax = axis_of.get(jn)
        if ax is None:
            raise ValueError("No rotation axis known for '%s'." % nm)
        ai = "xyz".index(ax)
        base = hou.Vector3(1 if ai == 0 else 0, 1 if ai == 1 else 0, 1 if ai == 2 else 0)
        t = base * Lr
        ref = hou.Vector3(t[0], t[1], t[2])
        if ref.length() < 1e-9:
            raise ValueError("Degenerate rotation axis on '%s'." % nm)
        ref = ref.normalized()

        # Signed angle about ref directly, via atan2 of a perpendicular probe
        # vector. Stable through zero and through 180 deg, and it needs no
        # sign heuristic -- comparing a skew-derived axis against ref flips
        # sign unpredictably when the rotation is small.
        u = hou.Vector3(1.0, 0.0, 0.0)
        if abs(ref[0]) > 0.9:
            u = hou.Vector3(0.0, 1.0, 0.0)
        u = (u - ref * u.dot(ref)).normalized()
        r = u * D
        r = hou.Vector3(r[0], r[1], r[2])
        angles.append(math.degrees(math.atan2(u.cross(r).dot(ref), u.dot(r))))

    return angles


def _wrap180(d):
    return d - 360.0 * round(d / 360.0)


def resolve_wrist(ang, prev):
    """Pick the wrist branch nearest the previous frame.

    A spherical wrist has two exactly equivalent solutions:
        (J4, J5, J6) == (J4 +- 180, -J5, J6 +- 180)
    Both put the tool in the identical position AND orientation, so the
    solver is free to return either -- and FBIK solves each frame
    independently, so it switches between them mid-path. That switch is a
    real 180 degree wrist spin the arm has no need to perform.

    Choosing the nearer branch removes the spin exactly. This selects
    between two valid solutions; it is not an approximation.

    Returns (angles, flipped).
    """
    if prev is None:
        return list(ang), False
    cand = list(ang)
    cand[3] = ang[3] - 180.0 if ang[3] > 0 else ang[3] + 180.0
    cand[4] = -ang[4]
    cand[5] = ang[5] - 180.0 if ang[5] > 0 else ang[5] + 180.0
    d_raw = sum(abs(_wrap180(ang[k] - prev[k])) for k in (3, 4, 5))
    d_alt = sum(abs(_wrap180(cand[k] - prev[k])) for k in (3, 4, 5))
    if d_alt < d_raw:
        return cand, True
    return list(ang), False


def bake_ik_to_fk(kwargs):
    """Bake the solved skeleton onto an FK Rig Pose, free of wrist flips."""
    node = kwargs["node"]
    net = node.parent()
    src = node.inputs()[0] if node.inputs() else None
    if src is None:
        _notify("Nothing wired into the input.", severity=hou.severityType.Error,
                title="Bake IK to FK")
        return

    rest, axis_src = _find_rest_source(node)
    if axis_src is None:
        axis_src = _find_config_joints(node)
    if rest is None or axis_src is None:
        _notify("Could not find a Configure Joints SOP to build the Rig Pose from.",
                severity=hou.severityType.Error, title="Bake IK to FK")
        return
    axis_of = axis_map_from_geo(axis_src.geometry())
    if not axis_of:
        _notify("Configure Joints provided no fbik_jointconfig, so joint axes are unknown.",
                severity=hou.severityType.Error, title="Bake IK to FK")
        return

    name = node.parm("bake_target_name").eval().strip() or "ik_baked_fk"
    tgt = net.node(name)
    if tgt is None:
        tgt = net.createNode("kinefx::rigpose", name)
        pos = node.position()
        tgt.setPosition(hou.Vector2(pos[0] - 3.0, pos[1] + 2.4))
        tgt.setColor(hou.Color((0.2, 0.7, 0.4)))
    tgt.setFirstInput(rest)
    tgt.parm("transformations").set(NUM_JOINTS)
    for jn in range(1, NUM_JOINTS + 1):
        tgt.parm("group%d" % (jn - 1)).set("@name=joint_%d" % jn)
        for a in "xyz":
            tgt.parm("r%d%s" % (jn - 1, a)).deleteAllKeyframes()
            tgt.parm("r%d%s" % (jn - 1, a)).set(0.0)

    f0 = int(node.parm("frame_rangex").eval())
    f1 = int(node.parm("frame_rangey").eval())
    use_wrist = True
    wp = node.parm("wrist_continuity")
    if wp is not None:
        use_wrist = bool(wp.eval())

    prev = None
    prev_raw = None
    turns = [0.0] * NUM_JOINTS
    flips = []
    curves = {}
    for jn in range(1, NUM_JOINTS + 1):
        curves[jn] = []

    for frame in range(f0, f1 + 1):
        ang = extract_angles(src.geometryAtFrame(frame), axis_of)
        if use_wrist:
            ang, flipped = resolve_wrist(ang, prev)
            if flipped:
                flips.append(frame)
        prev = list(ang)
        if prev_raw is not None:
            for i in range(NUM_JOINTS):
                d = ang[i] - prev_raw[i]
                if d > 180.0:
                    turns[i] -= 360.0
                elif d < -180.0:
                    turns[i] += 360.0
        prev_raw = list(ang)
        for jn in range(1, NUM_JOINTS + 1):
            curves[jn].append((frame, ang[jn - 1] + turns[jn - 1]))

    for jn in range(1, NUM_JOINTS + 1):
        parm = tgt.parm("r%d%s" % (jn - 1, axis_of[jn]))
        for frame, val in curves[jn]:
            key = hou.Keyframe()
            key.setFrame(frame)
            key.setValue(val)
            key.setExpression("linear()", hou.exprLanguage.Hscript)
            parm.setKeyframe(key)

    node.parm("status").set("Baked %d frames to %s (%d wrist flips resolved)"
                            % (f1 - f0 + 1, tgt.name(), len(flips)))
    _notify("Baked frames %d-%d onto:\n%s\n\n%d wrist flips resolved.\n\n"
            "Tool pose is identical -- only the redundant 180 degree wrist\n"
            "spins are removed. Wire this into your FK/IK switch to use it."
            % (f0, f1, tgt.path(), len(flips)),
            title="Bake IK to FK")


def export_animation(kwargs):
    node = kwargs["node"]

    src = node.inputs()[0] if node.inputs() else None
    if src is None:
        _notify("Nothing wired into the input.\n\n"
                              "Connect a solved KineFX skeleton (e.g. the Full Body IK output).",
                              severity=hou.severityType.Error, title="No Input")
        return

    path = node.parm("export_csv").eval().strip()
    if not path:
        _notify("Output CSV path is empty.",
                              severity=hou.severityType.Error, title="No Output Path")
        return

    f0 = int(node.parm("frame_rangex").eval())
    f1 = int(node.parm("frame_rangey").eval())
    if f1 < f0:
        _notify("End frame (%d) is before start frame (%d)." % (f1, f0),
                              severity=hou.severityType.Error, title="Bad Frame Range")
        return

    fps = hou.fps()
    dt = 1.0 / fps
    max_vel = float(node.parm("max_velocity").eval())
    cap = float(node.parm("speed_cap").eval())
    check_limits = int(node.parm("check_limits").eval())
    warn_frac = float(node.parm("warn_threshold").eval())

    out_dir = os.path.dirname(path)
    if out_dir and not os.path.isdir(out_dir):
        try:
            os.makedirs(out_dir)
        except OSError as e:
            _notify("Cannot create output directory:\n%s\n\n%s" % (out_dir, e),
                                  severity=hou.severityType.Error, title="Directory Error")
            return

    axis_of = axis_map_from_geo(src.geometry())
    if not axis_of:
        cfg = _find_config_joints(node)
        if cfg is not None:
            axis_of = axis_map_from_geo(cfg.geometry())

    rows = []
    limit_hits = []
    speed_hits = []
    prev = None
    signs = _joint_signs(node)
    do_unwrap = True
    up = node.parm("unwrap_angles")
    if up is not None:
        do_unwrap = bool(up.eval())
    prev_raw = None
    turns = [0.0] * NUM_JOINTS
    use_wrist = True
    _wp = node.parm("wrist_continuity")
    if _wp is not None:
        use_wrist = bool(_wp.eval())
    prev_wrist = None
    wrist_flips = []

    try:
        for frame in range(f0, f1 + 1):
            geo = src.geometryAtFrame(frame)
            ang = extract_angles(geo, axis_of)

            # Resolve the wrist branch first. FBIK solves each frame on its
            # own and may return either of the two equivalent wrist
            # solutions, which shows up as a 180 deg spin the arm need not do.
            if use_wrist:
                ang, _flipped = resolve_wrist(ang, prev_wrist)
                if _flipped:
                    wrist_flips.append(frame)
            prev_wrist = list(ang)

            # Unwrap before anything else. Extracted angles live in
            # (-180, 180], so a joint rotating smoothly through the boundary
            # emits 180.0 then -179.97 -- and a robot reading that literally
            # spins 360 degrees backwards. Accumulate whole turns so the
            # exported channel stays continuous. J1/J4/J6 have +-360 range,
            # which is the headroom this relies on.
            if do_unwrap:
                if prev_raw is not None:
                    for i in range(NUM_JOINTS):
                        d = ang[i] - prev_raw[i]
                        if d > 180.0:
                            turns[i] -= 360.0
                        elif d < -180.0:
                            turns[i] += 360.0
                prev_raw = list(ang)
                ang = [ang[i] + turns[i] for i in range(NUM_JOINTS)]

            # into the robot's convention before anything else looks at it,
            # so limit checks and velocities are evaluated on real J values
            ang = [ang[i] * signs[i] for i in range(NUM_JOINTS)]
            idx = frame - f0 + 1

            if prev is None:
                speed = 0.0
            else:
                vels = [abs(ang[i] - prev[i]) / dt for i in range(NUM_JOINTS)]
                speed = min((max(vels) / max_vel) * 100.0, cap)
                thr = max_vel * warn_frac
                for i, v in enumerate(vels):
                    if v > thr:
                        speed_hits.append("frame %d  J%d  %.1f deg/s" % (frame, i + 1, v))

            if check_limits:
                for i, a in enumerate(ang):
                    lo, hi = JOINT_LIMITS_DEG[i]
                    if a < lo or a > hi:
                        limit_hits.append("frame %d  J%d  %.3f deg (limit %.1f..%.1f)"
                                          % (frame, i + 1, a, lo, hi))

            row = [str(idx), "%.4f" % ((idx - 1) * dt)]
            row += ["%.6f" % a for a in ang]
            row.append("%.2f" % speed)
            rows.append(row)
            prev = ang
    except ValueError as e:
        _notify(str(e), severity=hou.severityType.Error, title="Extraction Failed")
        return

    try:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(BASE_HEADERS)
            w.writerows(rows)
    except IOError as e:
        _notify("Failed to write CSV:\n%s\n\n%s" % (path, e),
                              severity=hou.severityType.Error, title="File Write Error")
        return

    msg = "Exported %d frames at %g fps to:\n%s" % (len(rows), fps, path)
    sev = hou.severityType.Message
    if limit_hits:
        sev = hou.severityType.Warning
        msg += "\n\nJOINT LIMIT VIOLATIONS (%d):\n" % len(limit_hits)
        msg += "\n".join(limit_hits[:15])
        if len(limit_hits) > 15:
            msg += "\n... and %d more" % (len(limit_hits) - 15)
    if speed_hits:
        sev = hou.severityType.Warning
        msg += "\n\nSPEED WARNINGS above %d%% of %g deg/s (%d):\n" % (
            int(warn_frac * 100), max_vel, len(speed_hits))
        msg += "\n".join(speed_hits[:15])
        if len(speed_hits) > 15:
            msg += "\n... and %d more" % (len(speed_hits) - 15)
    if wrist_flips:
        msg += ("\n\nWrist branch resolved at %d frames (tool pose unchanged)."
                % len(wrist_flips))
    if not limit_hits and not speed_hits:
        msg += "\n\nNo joint-limit or speed warnings."

    node.parm("status").set(
        "Exported %d frames%s" % (len(rows),
                                  "  (%d limit, %d speed warnings)" % (len(limit_hits), len(speed_hits))
                                  if (limit_hits or speed_hits) else "  (clean)"))
    _notify(msg, severity=sev, title="Export Complete")


def _joint_signs(node):
    """Per-joint sign applied at the CSV boundary.

    The Houdini rig and the robot do not agree on the direction of every
    joint -- joint_3 runs the opposite way here. The flip is applied on
    export and un-applied on import, so a round trip still reproduces the
    source file exactly while the CSV stays in the robot's convention.
    """
    out = []
    for jn in range(1, NUM_JOINTS + 1):
        p = node.parm("invert_j%d" % jn)
        out.append(-1.0 if (p is not None and p.eval()) else 1.0)
    return out


def _find_config_joints(node):
    """Nearest Configure Joints SOP: upstream first, else anywhere alongside.

    Import does not need anything wired into this node's input, so fall back
    to scanning the parent network rather than failing.
    """
    seen = set()
    queue = [n for n in node.inputs() if n is not None]
    while queue:
        n = queue.pop(0)
        if n.path() in seen:
            continue
        seen.add(n.path())
        if n.type().name() == "kinefx::configurejoints":
            return n
        queue.extend([x for x in n.inputs() if x is not None])

    parent = node.parent()
    if parent is not None:
        for n in parent.children():
            if n.type().name() == "kinefx::configurejoints":
                return n
    return None


def _find_rest_source(node):
    """Return (wire_from, axis_from) for building a new FK Rig Pose.

    A CSV holds joint angles, so it must be keyed onto a Rig Pose fed by the
    REST skeleton -- feeding it from an already-posed one would stack the
    imported rotations on top of existing animation.

    These are two different nodes. The new Rig Pose is wired from whatever
    feeds Configure Joints (the shared rest null), so it sits parallel to the
    IK branch rather than downstream of it. But the rotation axes have to be
    read from Configure Joints itself, since that is what publishes
    fbik_jointconfig.
    """
    cfg = _find_config_joints(node)
    if cfg is None:
        return None, None
    ins = [n for n in cfg.inputs() if n is not None]
    return (ins[0] if ins else cfg), cfg


def import_animation(kwargs):
    """Read a CSV and key its joint angles onto an FK Rig Pose."""
    node = kwargs["node"]

    path = node.parm("import_csv").eval().strip()
    if not path or not os.path.isfile(path):
        _notify("Import CSV not found:\n%s" % path,
                              severity=hou.severityType.Error, title="No Input File")
        return

    with open(path, "r") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        _notify("CSV has no data rows.", severity=hou.severityType.Error,
                              title="Empty CSV")
        return

    jcols = [c for c in rows[0].keys() if c and c.endswith("_deg")]
    jcols.sort(key=lambda c: int(c.replace("_deg", "").replace("j", "")))
    if not jcols:
        _notify("No joint columns found (expected j1_deg .. j6_deg).",
                              severity=hou.severityType.Error, title="CSV Format Error")
        return

    create_new = int(node.parm("import_mode").eval()) == 0
    made_node = False

    axis_src = None
    if create_new:
        rest_path = node.parm("rest_source").eval().strip()
        rest = hou.node(rest_path) if rest_path else None
        auto_rest, axis_src = _find_rest_source(node)
        if rest is None:
            rest = auto_rest
        if rest is None:
            _notify(
                "Could not find a Configure Joints SOP upstream to build the\n"
                "Rig Pose from, and 'Rest Skeleton' is empty.\n\n"
                "Set 'Rest Skeleton' to the rest-skeleton null that feeds your\n"
                "Configure Joints node.",
                severity=hou.severityType.Error, title="No Rest Skeleton")
            return

        base = os.path.splitext(os.path.basename(path))[0]
        safe = "".join(ch if (ch.isalnum() or ch == "_") else "_" for ch in base)
        tgt = node.parent().createNode("kinefx::rigpose", "import_" + safe)
        tgt.setFirstInput(rest)
        pos = node.position()
        tgt.setPosition(hou.Vector2(pos[0] - 3.0, pos[1] + 1.2))
        tgt.setColor(hou.Color((0.2, 0.7, 0.4)))
        tgt.parm("transformations").set(len(jcols))
        for i, col in enumerate(jcols):
            jn = int(col.replace("_deg", "").replace("j", ""))
            tgt.parm("group%d" % i).set("@name=joint_%d" % jn)
            for a in "xyz":
                tgt.parm("r%d%s" % (i, a)).set(0.0)
        made_node = True
    else:
        tgt_path = node.parm("target_rigpose").eval().strip()
        tgt = hou.node(tgt_path) if tgt_path else None
        if tgt is None:
            _notify("Target Rig Pose node not found:\n%s" % tgt_path,
                                  severity=hou.severityType.Error, title="No Target")
            return
        if tgt.type().name() != "kinefx::rigpose":
            _notify("Target must be a kinefx::rigpose node.\n'%s' is a '%s'."
                                  % (tgt_path, tgt.type().name()),
                                  severity=hou.severityType.Error, title="Wrong Target Type")
            return

    # joint name -> group index on the target
    mapping = {}
    i = 0
    while tgt.parm("group" + str(i)) is not None:
        grp = tgt.parm("group" + str(i)).eval()
        for jn in range(1, NUM_JOINTS + 1):
            if "joint_%d" % jn in grp:
                mapping[jn] = i
        i += 1
    if not mapping:
        _notify("Target Rig Pose has no joint groups set up.",
                              severity=hou.severityType.Error, title="Target Not Configured")
        return

    # Rotation axis per joint. The Rig Pose is wired to the rest null, which
    # carries no fbik_jointconfig, so read the axes from Configure Joints.
    if axis_src is None:
        axis_src = _find_config_joints(node)
    axis_of = {}
    for cand in (axis_src, tgt):
        if cand is None:
            continue
        g = cand.geometry()
        if g is None or g.findPointAttrib("fbik_jointconfig") is None:
            continue
        for p in g.points():
            nm = p.attribValue("name")
            for jn in range(1, NUM_JOINTS + 1):
                if nm == "joint_%d" % jn:
                    w = _parse_weights(p.attribValue("fbik_jointconfig").get(
                        "rotation_weights", "[0, 0, 1]"))
                    axis_of[jn] = "xyz"[w.index(max(w))]
        if axis_of:
            break
    if not axis_of:
        if made_node:
            tgt.destroy()
        _notify(
            "Could not find 'fbik_jointconfig' on Configure Joints or on the\n"
            "target Rig Pose's input, so each joint's rotation axis is unknown\n"
            "and the imported signs would be unreliable.",
            severity=hou.severityType.Error, title="Unknown Rotation Axes")
        return

    signs = _joint_signs(node)
    start = int(node.parm("frame_rangex").eval())
    n_keys = 0
    for r_i, row in enumerate(rows):
        frame = start + r_i
        for col in jcols:
            jn = int(col.replace("_deg", "").replace("j", ""))
            if jn not in mapping or jn not in axis_of:
                continue
            parm = tgt.parm("r%d%s" % (mapping[jn], axis_of[jn]))
            if parm is None:
                continue
            key = hou.Keyframe()
            key.setFrame(frame)
            key.setValue(float(row[col]) * signs[jn - 1])
            parm.setKeyframe(key)
            n_keys += 1

    if made_node:
        node.parm("target_rigpose").set(tgt.path())

    node.parm("status").set("Imported %d frames -> %s" % (len(rows), tgt.name()))
    _notify(
        "Imported %d frames (%d keyframes) onto:\n%s\n\nFrames %d..%d\n\n%s"
        % (len(rows), n_keys, tgt.path(), start, start + len(rows) - 1,
           "Created a new FK Rig Pose wired to '%s'." % tgt.inputs()[0].name()
           if made_node else "Keyed onto the existing Rig Pose."),
        title="Import Complete")
