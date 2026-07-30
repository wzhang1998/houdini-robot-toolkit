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


def _find_profile_parm(node):
    """Locate the Robot Profile menu without an absolute path.

    Checked in order: this node, a sibling controller, then the enclosing
    asset. Absolute paths like /obj/geo1/TCP_PATH_CTRL break the moment the
    network is wrapped into a subnet or instanced twice.
    """
    if node is None:
        return None
    if node.parm("robot_profile") is not None:
        return node.parm("robot_profile")
    net = node.parent()
    for _ in range(4):
        if net is None:
            break
        if net.parm("robot_profile") is not None:
            return net.parm("robot_profile")
        for sib in net.children():
            if sib.parm("robot_profile") is not None:
                return sib.parm("robot_profile")
        net = net.parent()
    return None


def _profile(node=None):
    """Resolved per call so switching the Robot Profile menu takes effect
    without reloading the asset."""
    p = _find_profile_parm(node)
    pid = p.eval() if p is not None else None
    return robot_profile.load(pid or "uf850", os.path.join(_ROOT, "profiles"))


PROFILE = _profile()
JOINT_LIMITS_DEG = [tuple(r) for r in PROFILE["robot"]["limits_deg"]]
NUM_JOINTS = PROFILE["robot"]["num_joints"]
BASE_HEADERS = ["frame", "time_s"] + ["j%d_deg" % n for n in range(1, NUM_JOINTS + 1)] + ["speed_pct"]


# Set True to suppress modal dialogs. hou.ui.displayMessage blocks Houdini's
# main thread until a human clicks it, which deadlocks any scripted or
# headless run of these functions.
_QUIET = False


# Set by each entry point so _notify can find the Quiet toggle without an
# absolute path. Absolute paths break the moment the asset is instanced twice.
_HOST = None


def _quiet_parm():
    """Honour a Quiet toggle on this node or the asset wrapping it.

    hou.ui.displayMessage blocks Houdini's main thread until a human clicks
    it. A scripted or bridge-driven run then deadlocks with no way to dismiss
    the dialog from outside, which has stalled this project twice. The toggle
    lets automation suppress dialogs without touching the module global.
    """
    n = _HOST
    for _ in range(4):
        if n is None:
            break
        if n.parm("quiet_mode") is not None:
            return bool(n.parm("quiet_mode").eval())
        n = n.parent()
    return False


def _notify(text, severity=None, title=None):
    quiet = _QUIET
    if not quiet:
        try:
            quiet = _quiet_parm()
        except Exception:
            quiet = False
    if quiet or not hou.isUIAvailable():
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


def _active_ancestors(node, seen=None):
    """Upstream nodes reachable along the LIVE branch only.

    hou.Node.inputAncestors() walks every branch of a switch, which is the
    wrong question here: what matters is what the node is actually reading
    right now, not what is merely wired into it.
    """
    if seen is None:
        seen = set()
    if node is None or node in seen:
        return seen
    seen.add(node)
    ins = node.inputs()
    if not ins:
        return seen
    p = node.parm("input")
    if node.type().name() == "switch" and p is not None:
        live = [c for c in ins if c is not None]
        if live:
            _active_ancestors(live[int(p.eval()) % len(live)], seen)
        return seen
    for c in ins:
        _active_ancestors(c, seen)
    return seen


def bake_ik_to_fk(kwargs):
    """Bake the solved skeleton onto an FK Rig Pose, free of wrist flips."""
    node = kwargs["node"]
    global _HOST
    _HOST = node
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

    # Refuse to bake through the target. This asset's input is now the shared
    # POSE_SOURCE switch, so selecting "Baked IK to FK" makes src resolve to
    # tgt itself: the bake would read its own previous output and compound the
    # error a little more on every press, with nothing to show it was wrong.
    #
    # Must follow only the SELECTED branch of a switch. inputAncestors() walks
    # every branch, so with the bake target wired into the switch at all it
    # reports a conflict on every pose source and blocks legitimate bakes.
    chain = _active_ancestors(src)
    if tgt in chain:
        _notify("'%s' is upstream of this node's input, so baking would read "
                "its own output and compound on every press. Set Pose Source "
                "to IK (solved) and bake again." % tgt.name(),
                severity=hou.severityType.Error, title="Bake IK to FK")
        return

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

    _out_parm(node, "status").set("Baked %d frames to %s (%d wrist flips resolved)"
                            % (f1 - f0 + 1, tgt.name(), len(flips)))
    _notify("Baked frames %d-%d onto:\n%s\n\n%d wrist flips resolved.\n\n"
            "Tool pose is identical -- only the redundant 180 degree wrist\n"
            "spins are removed. Wire this into your FK/IK switch to use it."
            % (f0, f1, tgt.path(), len(flips)),
            title="Bake IK to FK")


def _collect(node, src):
    """Walk the frame range once and build the exact rows the CSV would hold.

    Shared by export_animation and preflight, deliberately. A pre-flight that
    recomputed the angles its own way could pass a clip the exporter then
    writes differently, and a gate that validates something other than what
    ships is worse than no gate.
    """
    f0 = int(node.parm("frame_rangex").eval())
    f1 = int(node.parm("frame_rangey").eval())
    if f1 < f0:
        raise ValueError("End frame (%d) is before start frame (%d)." % (f1, f0))

    fps = hou.fps()
    dt = 1.0 / fps
    max_vel = float(node.parm("max_velocity").eval())
    cap = float(node.parm("speed_cap").eval())
    # Resolve limits from THIS node's profile rather than the module-level one,
    # which is bound once at import. With two arms in a scene on different
    # profiles, the module-level copy would validate both against whichever
    # loaded first and pass angles the second arm cannot reach.
    limits = [tuple(r) for r in _profile(node)["robot"]["limits_deg"]]
    check_limits = int(node.parm("check_limits").eval())
    warn_frac = float(node.parm("warn_threshold").eval())

    axis_of = axis_map_from_geo(src.geometry())
    if not axis_of:
        cfg = _find_config_joints(node)
        if cfg is not None:
            axis_of = axis_map_from_geo(cfg.geometry())

    rows, limit_hits, speed_hits, wrist_flips = [], [], [], []
    angles, steps = [], []
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
    peak_vel = 0.0
    worst_vel = None

    for frame in range(f0, f1 + 1):
        geo = src.geometryAtFrame(frame)
        ang = extract_angles(geo, axis_of)

        # Resolve the wrist branch first. FBIK solves each frame on its own
        # and may return either of the two equivalent wrist solutions, which
        # shows up as a 180 deg spin the arm need not do.
        if use_wrist:
            ang, _flipped = resolve_wrist(ang, prev_wrist)
            if _flipped:
                wrist_flips.append(frame)
        prev_wrist = list(ang)

        # Unwrap before anything else. Extracted angles live in (-180, 180],
        # so a joint rotating smoothly through the boundary emits 180.0 then
        # -179.97 -- and a robot reading that literally spins 360 degrees
        # backwards. Accumulate whole turns so the exported channel stays
        # continuous. J1/J4/J6 have +-360 range, the headroom this relies on.
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

        # into the robot's convention before anything else looks at it, so
        # limit checks and velocities are evaluated on real J values
        ang = [ang[i] * signs[i] for i in range(NUM_JOINTS)]
        idx = frame - f0 + 1

        if prev is None:
            speed = 0.0
        else:
            vels = [abs(ang[i] - prev[i]) / dt for i in range(NUM_JOINTS)]
            speed = min((max(vels) / max_vel) * 100.0, cap)
            if max(vels) > peak_vel:
                peak_vel = max(vels)
                worst_vel = (frame, vels.index(max(vels)) + 1, max(vels))
            thr = max_vel * warn_frac
            for i, v in enumerate(vels):
                if v > thr:
                    speed_hits.append("frame %d  J%d  %.1f deg/s" % (frame, i + 1, v))
            for i in range(NUM_JOINTS):
                d = ang[i] - prev[i]
                if abs(d) > 180.0:
                    steps.append((frame, i + 1, d))

        if check_limits:
            for i, a in enumerate(ang):
                lo, hi = limits[i]
                if a < lo or a > hi:
                    limit_hits.append("frame %d  J%d  %.3f deg (limit %.1f..%.1f)"
                                      % (frame, i + 1, a, lo, hi))

        row = [str(idx), "%.4f" % ((idx - 1) * dt)]
        row += ["%.6f" % a for a in ang]
        row.append("%.2f" % speed)
        rows.append(row)
        angles.append(list(ang))
        prev = ang

    return {"rows": rows, "angles": angles, "limit_hits": limit_hits,
            "speed_hits": speed_hits, "wrist_flips": wrist_flips,
            "steps_over_180": steps, "fps": fps, "dt": dt, "f0": f0, "f1": f1,
            "limits": limits, "max_vel": max_vel, "warn_frac": warn_frac,
            "peak_vel": peak_vel, "worst_vel": worst_vel,
            "unwrapped": do_unwrap, "wrist_resolved": use_wrist}


def _out_parm(node, name):
    """The OUTERMOST parameter with this name, walking up from node.

    Status strings belong on the asset the user is looking at, not on the
    nested node that happens to compute them. Mirroring them upward with
    channel references failed: an expression set on one node never reaches
    other instances, and a DialogScript expression-default stores the raw
    text for string parameters instead of evaluating it. Writing directly to
    the outermost owner has neither problem.
    """
    found = node.parm(name)
    n = node.parent()
    for _ in range(4):
        if n is None:
            break
        if n.parm(name) is not None:
            found = n.parm(name)
        n = n.parent()
    return found


def _parm_upward(node, name):
    """Find a parameter on this node or any ancestor.

    Several controls live on the wrapping asset rather than on this one.
    Looking them up locally returns None, which reads as "not configured" and
    silently skips the check that depends on it.
    """
    n = node
    for _ in range(4):
        if n is None:
            return None
        if n.parm(name) is not None:
            return n.parm(name)
        n = n.parent()
    return None


def _cache_is_stale(node, data):
    """Is the solve cache showing something other than the live solve?

    The analysis chain reads the cache while pre-flight and export read live,
    so a stale cache means the numbers on screen describe a different clip
    from the one about to ship.
    """
    net = node.parent()
    if net is None:
        return None
    cache = net.node("cache_solve")
    src = node.inputs()[0] if node.inputs() else None
    if cache is None or src is None:
        return None

    def tcp(n, f):
        g = n.geometryAtFrame(f)
        if g is None:
            return None
        for p in g.points():
            if p.attribValue("name") == "joint_%d" % NUM_JOINTS:
                return tuple(round(x, 5) for x in p.position())
        return None

    f0, f1 = data["f0"], data["f1"]
    for f in (f0, (f0 + f1) // 2, f1):
        a, b = tcp(src, f), tcp(cache, f)
        if a is None or b is None:
            return None
        if any(abs(a[i] - b[i]) > 1e-4 for i in range(3)):
            return f
    return False


def _preflight_checks(node, data):
    """Structured pass/fail list. Each entry: (ok, severity, name, detail).

    severity FAIL blocks export; WARN is advisory. The split is about whether
    the arm can execute the command at all, not about how good the motion is.
    """
    out = []
    lim = data["limits"]
    ang = data["angles"]
    f0 = data["f0"]

    # 1. joint limits, checked on the UNWRAPPED values that actually ship
    worst = None
    n_over = 0
    for i, row in enumerate(ang):
        for j, a in enumerate(row):
            lo, hi = lim[j]
            if a < lo or a > hi:
                n_over += 1
                over = max(lo - a, a - hi)
                if worst is None or over > worst[3]:
                    worst = (f0 + i, j + 1, a, over)
    if n_over:
        out.append((False, "FAIL", "Joint limits",
                    "%d samples out of range; worst J%d = %.1f deg at frame %d "
                    "(%.1f deg past the limit)"
                    % (n_over, worst[1], worst[2], worst[0], worst[3])))
    else:
        out.append((True, "OK", "Joint limits", "all samples within profile limits"))

    # 2. steps over 180 deg -- the command that spins the arm backwards
    steps = data["steps_over_180"]
    if steps:
        w = max(steps, key=lambda t: abs(t[2]))
        out.append((False, "FAIL", "Angle continuity",
                    "%d steps over 180 deg; worst J%d = %.1f deg at frame %d"
                    % (len(steps), w[1], w[2], w[0])))
    else:
        out.append((True, "OK", "Angle continuity", "no step exceeds 180 deg"))

    # 3. unwrapping must be on, or check 2 is meaningless
    if not data["unwrapped"]:
        out.append((False, "FAIL", "Unwrap",
                    "Unwrap Angles is off, so the channel is discontinuous "
                    "at every +-180 crossing"))
    else:
        out.append((True, "OK", "Unwrap", "continuous angles"))

    # 4. joint velocity
    mv = data["max_vel"]
    wv = data["worst_vel"]
    if wv and wv[2] > mv:
        out.append((False, "FAIL", "Joint velocity",
                    "peak %.1f deg/s exceeds %.0f (J%d at frame %d)"
                    % (wv[2], mv, wv[1], wv[0])))
    elif wv and wv[2] > mv * data["warn_frac"]:
        out.append((True, "WARN", "Joint velocity",
                    "peak %.1f deg/s, above %.0f%% of %.0f (J%d at frame %d)"
                    % (wv[2], data["warn_frac"] * 100, mv, wv[1], wv[0])))
    else:
        out.append((True, "OK", "Joint velocity",
                    "peak %.1f deg/s of %.0f allowed" % (wv[2] if wv else 0.0, mv)))

    # 5. wrist branch
    wf = data["wrist_flips"]
    if not data["wrist_resolved"]:
        out.append((True, "WARN", "Wrist branch",
                    "Resolve Wrist Flips is off; FBIK may alternate between "
                    "the two equivalent wrist solutions"))
    else:
        out.append((True, "OK", "Wrist branch",
                    "resolved at %d frames (tool pose unchanged)" % len(wf)))

    # 6. frame range against the playbar
    fr = hou.playbar.frameRange()
    if int(fr[0]) != data["f0"] or int(fr[1]) != data["f1"]:
        out.append((True, "WARN", "Frame range",
                    "exporting %d-%d but the playbar is %d-%d"
                    % (data["f0"], data["f1"], int(fr[0]), int(fr[1]))))
    else:
        out.append((True, "OK", "Frame range",
                    "%d-%d, matches playbar" % (data["f0"], data["f1"])))

    # 7. tracking residual, IK only -- FK has no target to miss
    try:
        src = node.inputs()[0]
        g = src.geometryAtFrame(data["f0"])
        if g is not None and g.findGlobalAttrib("tcp_residual") is not None:
            worst_r, worst_f = 0.0, data["f0"]
            for f in range(data["f0"], data["f1"] + 1, max(1, (data["f1"] - data["f0"]) // 60 or 1)):
                gg = src.geometryAtFrame(f)
                r = gg.attribValue("tcp_residual")
                if r > worst_r:
                    worst_r, worst_f = r, f
            # residual_tolerance lives on the wrapping asset, not on this
            # node. Looking it up locally returned None, so tolv was 0, so the
            # check reported OK on a 634 mm miss against a 490 mm tolerance.
            tol = _parm_upward(node, "residual_tolerance")
            tolv = tol.eval() if tol is not None else 0.0
            if tolv and worst_r > tolv:
                out.append((True, "WARN", "Tracking residual",
                            "peak %.2f mm at frame %d, over the %.2f mm tolerance"
                            % (worst_r * 1000, worst_f, tolv * 1000)))
            else:
                out.append((True, "OK", "Tracking residual",
                            "peak %.2f mm" % (worst_r * 1000)))
        else:
            out.append((True, "OK", "Tracking residual",
                        "not applicable (no IK target on this pose)"))
    except Exception as e:
        out.append((True, "WARN", "Tracking residual", "could not sample: %s" % str(e)[:60]))

    # 8. solve cache against the live solve
    try:
        stale = _cache_is_stale(node, data)
        if stale is None:
            out.append((True, "OK", "Solve cache", "not in use"))
        elif stale is False:
            out.append((True, "OK", "Solve cache", "matches the live solve"))
        else:
            out.append((True, "WARN", "Solve cache",
                        "stale at frame %d -- the Analyze tab is describing a "
                        "different clip from the one about to export; "
                        "Clear and Recache" % stale))
    except Exception as e:
        out.append((True, "WARN", "Solve cache", "could not compare: %s" % str(e)[:60]))

    return out


def _preflight_failures(data):
    """Just the blocking reasons, for the export gate."""
    lim = data["limits"]
    fails = []
    n_over = sum(1 for row in data["angles"]
                 for j, a in enumerate(row) if a < lim[j][0] or a > lim[j][1])
    if n_over:
        worst = max(((f, j + 1, a, max(lim[j][0] - a, a - lim[j][1]))
                     for f, row in enumerate(data["angles"])
                     for j, a in enumerate(row)
                     if a < lim[j][0] or a > lim[j][1]),
                    key=lambda t: t[3])
        fails.append("Joint limits: %d samples out of range, worst J%d = %.1f deg "
                     "at frame %d" % (n_over, worst[1], worst[2],
                                      data["f0"] + worst[0]))
    if data["steps_over_180"]:
        w = max(data["steps_over_180"], key=lambda t: abs(t[2]))
        fails.append("Angle continuity: %d steps over 180 deg, worst J%d = %.1f deg "
                     "at frame %d" % (len(data["steps_over_180"]), w[1], w[2], w[0]))
    if not data["unwrapped"]:
        fails.append("Unwrap Angles is off; the channel is discontinuous")
    wv = data["worst_vel"]
    if wv and wv[2] > data["max_vel"]:
        fails.append("Joint velocity: peak %.1f deg/s exceeds %.0f (J%d frame %d)"
                     % (wv[2], data["max_vel"], wv[1], wv[0]))
    return fails


def preflight(kwargs):
    """Run every check and write a readable report to the asset."""
    node = kwargs["node"]
    global _HOST
    _HOST = node

    src = node.inputs()[0] if node.inputs() else None
    if src is None:
        _out_parm(node, "preflight_report").set("No input wired.")
        _notify("Nothing wired into the input.",
                severity=hou.severityType.Error, title="Pre-Flight")
        return

    try:
        data = _collect(node, src)
    except ValueError as e:
        _out_parm(node, "preflight_report").set("FAILED: %s" % e)
        _notify(str(e), severity=hou.severityType.Error, title="Pre-Flight")
        return

    checks = _preflight_checks(node, data)
    n_fail = sum(1 for c in checks if c[1] == "FAIL")
    n_warn = sum(1 for c in checks if c[1] == "WARN")

    # Publish the offending frames so the viewport can mark them. Recomputing
    # this in VEX would miss the unwrap failures entirely -- the analysis chain
    # sees wrapped angles, where J4 at -422 deg looks perfectly in range.
    bad = set()
    lim = data["limits"]
    for i, row in enumerate(data["angles"]):
        for j, a in enumerate(row):
            if a < lim[j][0] or a > lim[j][1]:
                bad.add(data["f0"] + i)
    for fr, _j, _d in data["steps_over_180"]:
        bad.add(fr)
    dt = data["dt"]
    prev = None
    for i, row in enumerate(data["angles"]):
        if prev is not None:
            if max(abs(row[k] - prev[k]) / dt for k in range(NUM_JOINTS)) > data["max_vel"]:
                bad.add(data["f0"] + i)
        prev = row
    fp = _out_parm(node, "preflight_frames")
    if fp is not None:
        fp.set(",".join(str(x) for x in sorted(bad)))
    R_bad = len(bad)

    lines = ["%-5s %-20s %s" % (c[1], c[2], c[3]) for c in checks]
    verdict = ("BLOCKED - %d failure%s, %d warning%s"
               % (n_fail, "" if n_fail == 1 else "s",
                  n_warn, "" if n_warn == 1 else "s")) if n_fail else (
              "READY - %d warning%s" % (n_warn, "" if n_warn == 1 else "s"))
    report = "%s\n\n%s" % (verdict, "\n".join(lines))
    report += "\n\n%d frame%s marked in the viewport." % (
        R_bad, "" if R_bad == 1 else "s")
    _out_parm(node, "preflight_report").set(report)
    _out_parm(node, "status").set(verdict)
    _notify(report,
            severity=hou.severityType.Error if n_fail else
            (hou.severityType.Warning if n_warn else hou.severityType.Message),
            title="Pre-Flight Check")


def export_animation(kwargs):
    node = kwargs["node"]
    global _HOST
    _HOST = node

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

    out_dir = os.path.dirname(path)
    if out_dir and not os.path.isdir(out_dir):
        try:
            os.makedirs(out_dir)
        except OSError as e:
            _notify("Cannot create output directory:\n%s\n\n%s" % (out_dir, e),
                                  severity=hou.severityType.Error, title="Directory Error")
            return

    try:
        data = _collect(node, src)
    except ValueError as e:
        _notify(str(e), severity=hou.severityType.Error, title="Extraction Failed")
        return

    rows = data["rows"]
    limit_hits = data["limit_hits"]
    speed_hits = data["speed_hits"]
    wrist_flips = data["wrist_flips"]
    fps = data["fps"]
    max_vel = data["max_vel"]
    warn_frac = data["warn_frac"]

    # Refuse to write a clip that would misbehave on hardware. Gate Export is
    # on by default: an out-of-limit or >180 deg step is not a style choice,
    # it is a command the arm cannot execute or will execute backwards.
    gate = node.parm("gate_export")
    if gate is not None and gate.eval():
        fails = _preflight_failures(data)
        if fails:
            _out_parm(node, "status").set("BLOCKED by pre-flight (%d)" % len(fails))
            _notify("Export blocked by pre-flight:\n\n" + "\n".join(fails) +
                    "\n\nRun Pre-Flight Check for detail, or switch off "
                    "Gate Export On Pre-Flight to write it anyway.",
                    severity=hou.severityType.Error, title="Export Blocked")
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

    _out_parm(node, "status").set(
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
    global _HOST
    _HOST = node

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

    _out_parm(node, "status").set("Imported %d frames -> %s" % (len(rows), tgt.name()))
    _notify(
        "Imported %d frames (%d keyframes) onto:\n%s\n\nFrames %d..%d\n\n%s"
        % (len(rows), n_keys, tgt.path(), start, start + len(rows) - 1,
           "Created a new FK Rig Pose wired to '%s'." % tgt.inputs()[0].name()
           if made_node else "Keyed onto the existing Rig Pose."),
        title="Import Complete")
