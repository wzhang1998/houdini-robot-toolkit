"""Per-frame joint angles gathered onto the TCP point.

Same extraction as the CSV exporter: local rest->posed delta, signed about
each joint's own axis via atan2. Cheap here (7 points), and the expensive
IK solve upstream is cached, so this is safe to run per frame.

Joint count, limits and sign convention come from profiles/<id>.json. They
used to be a literal table here, a second copy in the CSV asset's Python
module, and a third in the preset callback -- and the three had already
drifted (-241.9 vs -242.0, +-123.9 vs +-124.0). Reported angles are in the
ROBOT frame, i.e. after the profile's sign is applied, so they match the
exported CSV and can be checked against the profile's limits directly.
"""
import math
import sys

import hou

node = hou.pwd()
geo = node.geometry()

_root = hou.expandString("$HIP/..")
if _root + "/scripts" not in sys.path:
    sys.path.insert(0, _root + "/scripts")
import robot_profile

_pp = node.parent().node("TCP_PATH_CTRL")
_pp = _pp.parm("robot_profile") if _pp is not None else None
PROFILE = robot_profile.load(_pp.eval() if _pp is not None else "uf850",
                             _root + "/profiles")

NUM_JOINTS = PROFILE["robot"]["num_joints"]
JOINT_LIMITS_DEG = [tuple(r) for r in PROFILE["robot"]["limits_deg"]]


def m4(nine):
    v = list(nine)
    return hou.Matrix4([v[0], v[1], v[2], 0.0,
                        v[3], v[4], v[5], 0.0,
                        v[6], v[7], v[8], 0.0,
                        0.0, 0.0, 0.0, 1.0])


def weights(raw):
    if isinstance(raw, (list, tuple)):
        vals = list(raw)
    else:
        vals = [float(x) for x in str(raw).strip("[]").split(",")]
    return [abs(float(x)) for x in vals]


# Sign convention: the profile is authoritative. The CSV asset still carries
# its own invert_jN toggles; Stage 2 drives those from the profile too. Until
# then a manual change there will disagree with what this reports, so warn
# rather than silently pick one.
signs = [float(s) for s in PROFILE["rig"]["sign"]]
hda = node.parent().node("robot_csv_io")
if hda is not None:
    for i in range(NUM_JOINTS):
        p = hda.parm("invert_j%d" % (i + 1))
        if p is None:
            continue
        asset_sign = -1.0 if p.eval() else 1.0
        if asset_sign != signs[i]:
            raise hou.NodeError(
                "J%d sign disagrees: profile '%s' says %+d, robot_csv_io/"
                "invert_j%d says %+d. Reported angles and exported angles "
                "would differ by 2x on that joint."
                % (i + 1, PROFILE["id"], int(signs[i]), i + 1, int(asset_sign)))

pts = list(geo.points())
by = {}
for p in pts:
    by[p.attribValue("name")] = p

# Assert the skeleton is the one the profile describes. Every serious failure
# in this project produced confident wrong numbers rather than an error: a
# guessed rotation axis zeroed three joints, wrapped angles read as a 48x
# velocity spike. Checking costs one pass over 7 points.
_problems = robot_profile.validate_skeleton(geo, PROFILE)
if _problems:
    raise hou.NodeError(
        "Skeleton does not match profile '%s':\n  %s"
        % (PROFILE["id"], "\n  ".join(_problems)))

for nm, dflt in (("angle", 0.0), ("residual", 0.0), ("orient_error", 0.0),
                 ("limit_margin", 0.0), ("is_tcp", 0)):
    if geo.findPointAttrib(nm) is None:
        geo.addAttrib(hou.attribType.Point, nm, dflt)
for i in range(1, NUM_JOINTS + 1):
    if geo.findPointAttrib("j%d" % i) is None:
        geo.addAttrib(hou.attribType.Point, "j%d" % i, 0.0)

# Rotation axis per joint. FK sources (a Rig Pose fed from the rest null)
# carry no fbik_jointconfig -- only the IK branch does. Never fall back to
# a guessed axis: guessing z zeroes out every y-axis joint (J1/J4/J6),
# which reads as "the wrist never moves" and is off by up to 177 degrees.
axis_of = {}
src_geo = geo
if geo.findPointAttrib("fbik_jointconfig") is None:
    src_geo = None
    for n in node.parent().children():
        if n.type().name() == "kinefx::configurejoints":
            g2 = n.geometry()
            if g2 is not None and g2.findPointAttrib("fbik_jointconfig") is not None:
                src_geo = g2
                break
if src_geo is not None:
    for p in src_geo.points():
        nm = p.attribValue("name")
        for jn in range(1, NUM_JOINTS + 1):
            if nm == "joint_%d" % jn:
                w = weights(p.attribValue("fbik_jointconfig").get(
                    "rotation_weights", "[0, 0, 1]"))
                axis_of[jn] = w.index(max(w))
if len(axis_of) < NUM_JOINTS:
    raise hou.NodeError(
        "No 'fbik_jointconfig' on the input or on any Configure Joints SOP in "
        "this network, so the joints' rotation axes are unknown. Reported "
        "angles would be silently wrong on every y-axis joint.")

angles = [0.0] * NUM_JOINTS

for jn in range(1, NUM_JOINTS + 1):
    nm = "joint_%d" % jn
    p = by.get(nm)
    if p is None:
        continue
    par = p.attribValue("parent_idx")
    W = m4(p.attribValue("transform"))
    Wr = m4(p.attribValue("rest_transform"))
    if par is not None and par >= 0:
        Wp = m4(pts[par].attribValue("transform"))
        Wpr = m4(pts[par].attribValue("rest_transform"))
    else:
        Wp = hou.Matrix4(1)
        Wpr = hou.Matrix4(1)

    Lr = Wr * Wpr.inverted()
    D = Lr.inverted() * (W * Wp.inverted())

    ai = axis_of[jn]
    base = hou.Vector3(1 if ai == 0 else 0, 1 if ai == 1 else 0, 1 if ai == 2 else 0)
    t = base * Lr
    ref = hou.Vector3(t[0], t[1], t[2])
    if ref.length() < 1e-9:
        continue
    ref = ref.normalized()

    u = hou.Vector3(1.0, 0.0, 0.0)
    if abs(ref[0]) > 0.9:
        u = hou.Vector3(0.0, 1.0, 0.0)
    u = (u - ref * u.dot(ref)).normalized()
    r = u * D
    r = hou.Vector3(r[0], r[1], r[2])
    a = math.degrees(math.atan2(u.cross(r).dot(ref), u.dot(r))) * signs[jn - 1]
    angles[jn - 1] = a
    p.setAttribValue("angle", a)

# gather everything onto the TCP so a Trail can carry it along the path
tcp = by.get("joint_%d" % NUM_JOINTS)
if tcp is not None:
    margin = 1e9
    for i, a in enumerate(angles):
        lo, hi = JOINT_LIMITS_DEG[i]
        margin = min(margin, a - lo, hi - a)
        tcp.setAttribValue("j%d" % (i + 1), a)
    tcp.setAttribValue("limit_margin", margin)
    tcp.setAttribValue("is_tcp", 1)
    # Residual and orientation error only exist on the IK branch -- they
    # compare a REQUESTED pose against the achieved one. An FK pose has no
    # target, so -1 marks "not applicable" rather than 0, which would read
    # as perfect tracking.
    for s_, d_ in (("tcp_residual", "residual"), ("tcp_orient_error", "orient_error")):
        if geo.findGlobalAttrib(s_) is not None:
            tcp.setAttribValue(d_, geo.attribValue(s_))
        else:
            tcp.setAttribValue(d_, -1.0)
