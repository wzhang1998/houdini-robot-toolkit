"""Build a KineFX rest skeleton for a serial arm from its URDF.

The rest of this toolkit expects the KineFX convention: +Y runs down each
bone toward its child, and the tool axis is +Y on the last joint. A URDF uses
a different one: every joint rotates about its own local axis (usually z),
and frames are Z-up. So the URDF frames cannot be copied across. Each joint
frame is rebuilt here from two facts the URDF does state -- where the joints
are and which way each one turns -- and the result is classified per joint:

    rotation_axis 'y'  the joint turns about its own bone (a twist)
    rotation_axis 'z'  the joint turns perpendicular to its bone (a hinge)

Anything in between is a skewed joint that a single-axis KineFX rotation
cannot represent, and raises instead of producing a wrong rig.

rotation_axis and sign are MEASURED from the geometry, not assumed. They feed
straight into profiles/<id>.json, where the rest of the toolkit reads them.

The pure functions have no hou dependency and are runnable as
    python scripts/urdf_rig.py
"""

import math
import os
import struct
import xml.etree.ElementTree as ET

# URDF is Z-up, Houdini is Y-up. (x, y, z) -> (x, z, -y): a -90 deg turn
# about X, which keeps the URDF's +X as the arm's forward direction.
Z_UP_TO_Y_UP = ((1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
                (0.0, -1.0, 0.0))

PARALLEL_TOL = 1e-6


# --------------------------------------------------------------------------
# small linear algebra -- 3-vectors and 3x3 row-major matrices
# --------------------------------------------------------------------------

def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _scale(a, s):
    return tuple(x * s for x in a)


def _dot(a, b):
    if len(a) == 3:
        # unrolled: same sum, in the same order, as the general case
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _norm(a):
    return math.sqrt(_dot(a, a))


def _normalize(a):
    n = _norm(a)
    if n < 1e-12:
        raise ValueError("cannot normalise a zero-length vector")
    return _scale(a, 1.0 / n)


def _mat_mul(a, b):
    # Unrolled 3x3: the IK's Newton polish runs forward kinematics ~120 times
    # a solve, and the generator version was 90 % of Retime's cook time.
    # Same products summed in the same order, so the results are identical.
    (a0, a1, a2), (b0, b1, b2) = a, b
    return ((a0[0] * b0[0] + a0[1] * b1[0] + a0[2] * b2[0],
             a0[0] * b0[1] + a0[1] * b1[1] + a0[2] * b2[1],
             a0[0] * b0[2] + a0[1] * b1[2] + a0[2] * b2[2]),
            (a1[0] * b0[0] + a1[1] * b1[0] + a1[2] * b2[0],
             a1[0] * b0[1] + a1[1] * b1[1] + a1[2] * b2[1],
             a1[0] * b0[2] + a1[1] * b1[2] + a1[2] * b2[2]),
            (a2[0] * b0[0] + a2[1] * b1[0] + a2[2] * b2[0],
             a2[0] * b0[1] + a2[1] * b1[1] + a2[2] * b2[1],
             a2[0] * b0[2] + a2[1] * b1[2] + a2[2] * b2[2]))


def _mat_vec(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


_RPY_CACHE = {}


def rpy_to_matrix(rpy):
    """URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
    Cached: forward kinematics asks for the same few joint origins
    thousands of times a solve."""
    key = tuple(rpy)
    m = _RPY_CACHE.get(key)
    if m is None:
        r, p, y = key
        m = _RPY_CACHE[key] = _mat_mul(_rot_z(y), _mat_mul(_rot_y(p), _rot_x(r)))
    return m


def axis_angle_matrix(axis, angle):
    """Rodrigues rotation about a unit axis, column-vector convention."""
    x, y, z = _normalize(axis)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return ((t * x * x + c, t * x * y - s * z, t * x * z + s * y),
            (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            (t * x * z - s * y, t * y * z + s * x, t * z * z + c))


# --------------------------------------------------------------------------
# URDF parsing
# --------------------------------------------------------------------------

def _floats(text, default):
    return tuple(float(v) for v in text.split()) if text else default


def parse_urdf(path):
    """Return {'name', 'links', 'chain'} for a single serial chain.

    links maps link name -> mesh filename (or None).
    chain lists the revolute joints from the root outward.
    """
    root = ET.parse(path).getroot()

    links = {}
    for link in root.findall("link"):
        mesh = link.find("visual/geometry/mesh")
        vo = link.find("visual/origin")
        if vo is not None and (_floats(vo.get("xyz"), (0, 0, 0)) != (0, 0, 0)
                               or _floats(vo.get("rpy"), (0, 0, 0)) != (0, 0, 0)):
            raise ValueError("%s: link %s has a non-zero visual origin; "
                             "placing it is not implemented"
                             % (path, link.get("name")))
        links[link.get("name")] = mesh.get("filename") if mesh is not None else None

    joints = {}
    for j in root.findall("joint"):
        if j.get("type") not in ("revolute", "continuous"):
            continue
        origin = j.find("origin")
        limit = j.find("limit")
        joints[j.find("parent").get("link")] = {
            "name": j.get("name"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": _floats(origin.get("xyz") if origin is not None else None,
                           (0.0, 0.0, 0.0)),
            "rpy": _floats(origin.get("rpy") if origin is not None else None,
                           (0.0, 0.0, 0.0)),
            "axis": _floats(j.find("axis").get("xyz")
                            if j.find("axis") is not None else None,
                            (1.0, 0.0, 0.0)),
            "lower_deg": math.degrees(float(limit.get("lower"))) if limit is not None else -180.0,
            "upper_deg": math.degrees(float(limit.get("upper"))) if limit is not None else 180.0,
            "velocity_deg_s": math.degrees(float(limit.get("velocity"))) if limit is not None else None,
        }

    children = {j["child"] for j in joints.values()}
    roots = [p for p in joints if p not in children]
    if len(roots) != 1:
        raise ValueError("%s: expected one serial chain, found roots %r"
                         % (path, roots))

    chain, link = [], roots[0]
    while link in joints:
        chain.append(joints[link])
        link = joints[link]["child"]
    return {"name": root.get("name"), "links": links, "chain": chain,
            "root_link": roots[0]}


# --------------------------------------------------------------------------
# forward kinematics in the URDF frame
# --------------------------------------------------------------------------

def forward_kinematics(chain, q_deg=None):
    """Walk the chain. Returns one dict per joint:

        position  joint origin in the world (base) frame
        axis      joint rotation axis in the world frame
        link_R    orientation of the child link after this joint rotates
        link_p    origin of the child link (== position)
    """
    q_deg = q_deg or [0.0] * len(chain)
    R, p = IDENTITY, (0.0, 0.0, 0.0)
    out = []
    for joint, q in zip(chain, q_deg):
        p = _add(p, _mat_vec(R, joint["xyz"]))
        R = _mat_mul(R, rpy_to_matrix(joint["rpy"]))
        axis_world = _normalize(_mat_vec(R, joint["axis"]))
        R = _mat_mul(R, axis_angle_matrix(joint["axis"], math.radians(q)))
        out.append({"position": p, "axis": axis_world, "link_R": R,
                    "link_p": p})
    return out


def flange_point(fk, flange_offset):
    """Point flange_offset metres along the last link's local +z."""
    last = fk[-1]
    return _add(last["link_p"], _mat_vec(last["link_R"],
                                         (0.0, 0.0, flange_offset)))


# --------------------------------------------------------------------------
# STL -- only used to find where the flange face is
# --------------------------------------------------------------------------

def stl_bounds(path):
    """(min, max) of a binary or ASCII STL's vertices, in its own frame."""
    with open(path, "rb") as f:
        data = f.read()
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3

    def take(v):
        for i in range(3):
            lo[i] = min(lo[i], v[i])
            hi[i] = max(hi[i], v[i])

    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + count * 50 and count > 0:
        for t in range(count):
            base = 84 + t * 50 + 12
            for k in range(3):
                take(struct.unpack_from("<3f", data, base + k * 12))
    else:
        for line in data.decode("ascii", "ignore").splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[0] == "vertex":
                take(tuple(float(x) for x in parts[1:]))
    if lo[0] == float("inf"):
        raise ValueError("%s: no vertices read" % path)
    return tuple(lo), tuple(hi)


def resolve_mesh(filename, package_root):
    """package://<pkg>/rest  ->  <package_root>/<pkg>/rest"""
    prefix = "package://"
    if filename.startswith(prefix):
        return os.path.join(package_root, filename[len(prefix):])
    return filename


# --------------------------------------------------------------------------
# KineFX rest skeleton
# --------------------------------------------------------------------------

def _to_houdini(v):
    return _mat_vec(Z_UP_TO_Y_UP, v)


def _perpendicular(ref, y):
    """ref with its component along y removed, normalised."""
    return _normalize(_sub(ref, _scale(y, _dot(ref, y))))


def build_rest_skeleton(chain, flange_offset, joint_name_pattern="joint_%d"):
    """KineFX rest skeleton at the URDF zero pose, in Houdini Y-up.

    Each joint's +Y points at the next joint (the last one at the flange).
    Where the URDF axis is perpendicular to that bone, +Z is the axis and the
    joint is a hinge ('z'). Where it lies along the bone, the joint is a twist
    ('y') and +Z is borrowed from a neighbouring joint's axis so the frame is
    still deterministic.

    Returns a list of dicts: name, parent, P, transform (rows X, Y, Z),
    rotation_axis, sign. sign relates the two conventions:
        urdf_angle = kinefx_angle * sign
    """
    fk = forward_kinematics(chain)
    n = len(fk)
    positions = [f["position"] for f in fk] + [flange_point(fk, flange_offset)]
    axes = [f["axis"] for f in fk]

    joints = []
    for i in range(n):
        bone = _sub(positions[i + 1], positions[i])
        if _norm(bone) < 1e-9:
            raise ValueError("joint %d and its child coincide; a zero-length "
                             "bone has no direction" % (i + 1))
        y = _normalize(bone)
        d = _dot(axes[i], y)

        if abs(abs(d) - 1.0) < PARALLEL_TOL:
            rot, sign = "y", (1 if d > 0 else -1)
            ref = None
            for k in (i + 1, i - 1, i + 2, i - 2):
                if 0 <= k < n and abs(abs(_dot(axes[k], y)) - 1.0) > 1e-3:
                    ref = axes[k]
                    break
            z = _perpendicular(ref if ref is not None else (0.0, 0.0, 1.0), y)
        elif abs(d) < PARALLEL_TOL:
            rot, sign = "z", 1
            z = axes[i]
        else:
            raise ValueError(
                "joint %d axis is %.3f deg off its bone -- neither a hinge nor "
                "a twist, so one KineFX axis cannot represent it"
                % (i + 1, math.degrees(math.acos(max(-1.0, min(1.0, abs(d)))))))

        x = _cross(y, z)
        joints.append({
            "name": joint_name_pattern % (i + 1),
            "parent": (joint_name_pattern % i) if i > 0 else None,
            "P": _to_houdini(positions[i]),
            "transform": (_to_houdini(x), _to_houdini(y), _to_houdini(z)),
            "rotation_axis": rot,
            "sign": sign,
        })
    return joints


def _transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def posed_skeleton(chain, flange_offset, q_deg, joint_name_pattern="joint_%d"):
    """The rest skeleton moved to joint angles q_deg (URDF frame), for
    previewing. Each joint keeps its rest frame, rotated by how far its link
    has turned since the zero pose, so a rest/posed pair drives the link
    meshes rigidly through Transform Pieces."""
    rest = build_rest_skeleton(chain, flange_offset, joint_name_pattern)
    fk0 = forward_kinematics(chain)
    fkq = forward_kinematics(chain, q_deg)
    c, ct = Z_UP_TO_Y_UP, _transpose(Z_UP_TO_Y_UP)
    out = []
    for j, f0, fq in zip(rest, fk0, fkq):
        delta = _mat_mul(c, _mat_mul(_mat_mul(fq["link_R"],
                                              _transpose(f0["link_R"])), ct))
        posed = dict(j)
        posed["P"] = _to_houdini(fq["position"])
        posed["transform"] = tuple(_mat_vec(delta, row)
                                   for row in j["transform"])
        out.append(posed)
    return out


def link_placements(chain, parsed_links, package_root, root_link):
    """World placement (Houdini Y-up) of every link mesh at the zero pose.

    Returns a list of (link_name, mesh_path, R_houdini, p_houdini, joint_index)
    where joint_index is 0 for the static base and i for the link moved by
    joint i.
    """
    fk = forward_kinematics(chain)
    out = []
    base_mesh = parsed_links.get(root_link)
    if base_mesh:
        out.append((root_link, resolve_mesh(base_mesh, package_root),
                    Z_UP_TO_Y_UP, (0.0, 0.0, 0.0), 0))
    for i, (joint, f) in enumerate(zip(chain, fk), start=1):
        mesh = parsed_links.get(joint["child"])
        if not mesh:
            continue
        R = _mat_mul(Z_UP_TO_Y_UP, f["link_R"])
        out.append((joint["child"], resolve_mesh(mesh, package_root), R,
                    _to_houdini(f["link_p"]), i))
    return out


def flange_offset_from_mesh(chain, parsed_links, package_root):
    """Distance along the last link's +z to the far face of its mesh."""
    mesh = parsed_links.get(chain[-1]["child"])
    if not mesh:
        return 0.0
    _, hi = stl_bounds(resolve_mesh(mesh, package_root))
    return hi[2]


def profile_rig_fields(skeleton):
    """The two lists profiles/<id>.json needs, derived from geometry."""
    return ([j["rotation_axis"] for j in skeleton],
            [j["sign"] for j in skeleton])


# --------------------------------------------------------------------------
# Houdini side -- the only part that imports hou
# --------------------------------------------------------------------------

def write_skeleton(geo, skeleton):
    """Fill a SOP's geometry with a KineFX skeleton: points with name and
    transform, one two-point polyline per parent -> child bone."""
    import hou

    geo.addAttrib(hou.attribType.Point, "name", "")
    geo.addAttrib(hou.attribType.Point, "transform", (1.0, 0.0, 0.0,
                                                       0.0, 1.0, 0.0,
                                                       0.0, 0.0, 1.0))
    points = {}
    for j in skeleton:
        pt = geo.createPoint()
        pt.setPosition(j["P"])
        pt.setAttribValue("name", j["name"])
        flat = [c for row in j["transform"] for c in row]
        pt.setAttribValue("transform", flat)
        points[j["name"]] = pt
    for j in skeleton:
        if j["parent"]:
            poly = geo.createPolygon(is_closed=False)
            poly.addVertex(points[j["parent"]])
            poly.addVertex(points[j["name"]])


def write_link_meshes(geo, placements, joint_name_pattern="joint_%d"):
    """Merge every link mesh into geo at its zero-pose placement, tagged with
    a primitive 'name' of the joint that moves it ('base' for the static
    link) so Transform Pieces can drive it rigidly from a posed skeleton.

    STL faces wind counter-clockwise seen from outside; Houdini's front face
    is clockwise, and its STL reader keeps the file's order. Loaded as-is,
    every face points into the part (measured on all seven FR20 links), so
    STL meshes are reversed here."""
    import hou

    reverse = hou.sopNodeTypeCategory().nodeVerb("reverse")
    geo.addAttrib(hou.attribType.Prim, "name", "")
    for link, path, R, p, idx in placements:
        g = hou.Geometry()
        g.loadFromFile(path)
        if path.lower().endswith(".stl"):
            flipped = hou.Geometry()
            reverse.execute(flipped, [g])
            g = flipped
        m = hou.Matrix4((R[0][0], R[1][0], R[2][0], 0.0,
                         R[0][1], R[1][1], R[2][1], 0.0,
                         R[0][2], R[1][2], R[2][2], 0.0,
                         p[0], p[1], p[2], 1.0))
        g.transform(m)
        if g.findPrimAttrib("name") is None:
            g.addAttrib(hou.attribType.Prim, "name", "")
        tag = "base" if idx == 0 else joint_name_pattern % idx
        for prim in g.prims():
            prim.setAttribValue("name", tag)
        geo.merge(g)


# --------------------------------------------------------------------------
# tests -- run: python scripts/urdf_rig.py
# --------------------------------------------------------------------------

def _toolkit_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


if __name__ == "__main__":
    root = _toolkit_root()
    pkg = os.path.join(root, "assets")
    urdf = os.path.join(pkg, "fairino_description", "urdf",
                        "fairino20_v6.urdf")
    parsed = parse_urdf(urdf)
    chain = parsed["chain"]
    fails = []

    def check(label, ok, detail=""):
        if not ok:
            fails.append("%s %s" % (label, detail))

    check("six revolute joints", len(chain) == 6, "got %d" % len(chain))

    flange = flange_offset_from_mesh(chain, parsed["links"], pkg)
    check("flange offset is positive and under 0.3 m", 0.0 < flange < 0.3,
          "got %.4f" % flange)

    fk = forward_kinematics(chain)
    tip = flange_point(fk, flange)

    # Datasheet reach is 1854 mm. It is NOT the zero-pose distance to the
    # flange -- at the URDF zero the wrist points down, which gives 1.74 m.
    # It is the in-plane chain with the wrist straightened outward:
    # upper arm (J2->J3) + forearm (J3->J4) + J5->J6. The J4->J5 step is a
    # sideways offset along J4's own axis and does not add to reach.
    pos = [f["position"] for f in fk]
    in_plane = (_norm(_sub(pos[2], pos[1])) + _norm(_sub(pos[3], pos[2]))
                + _norm(_sub(pos[5], pos[4])))
    check("upper arm + forearm + J5->J6 = datasheet 1.854 m",
          abs(in_plane - 1.854) < 0.001, "got %.4f m" % in_plane)
    reach = in_plane

    # J1 is vertical, so turning it 90 deg must turn the flange 90 deg about
    # the vertical axis and leave its height alone.
    fk90 = forward_kinematics(chain, [90.0, 0, 0, 0, 0, 0])
    tip90 = flange_point(fk90, flange)
    ang = math.degrees(math.atan2(tip90[1], tip90[0]) -
                       math.atan2(tip[1], tip[0]))
    ang = (ang + 180.0) % 360.0 - 180.0
    check("J1 +90 turns the flange +90 about vertical", abs(ang - 90.0) < 1e-6,
          "got %.6f" % ang)
    check("J1 leaves flange height alone", abs(tip90[2] - tip[2]) < 1e-9)

    skel = build_rest_skeleton(chain, flange)
    for j in skel:
        X, Y, Z = j["transform"]
        ortho = max(abs(_dot(X, Y)), abs(_dot(Y, Z)), abs(_dot(X, Z)))
        unit = max(abs(_norm(v) - 1.0) for v in (X, Y, Z))
        det = _dot(_cross(X, Y), Z)
        check("%s frame orthonormal" % j["name"], ortho < 1e-9 and unit < 1e-9)
        check("%s frame right-handed" % j["name"], abs(det - 1.0) < 1e-9)

    # +Y must point at the child, which is what the toolkit's VEX assumes.
    for a, b in zip(skel, skel[1:]):
        bone = _normalize(_sub(b["P"], a["P"]))
        check("%s +Y points at %s" % (a["name"], b["name"]),
              abs(_dot(bone, a["transform"][1]) - 1.0) < 1e-9)

    # Houdini is Y-up: J1's bone goes straight up.
    check("J1 bone is vertical in Houdini",
          abs(skel[0]["transform"][1][1] - 1.0) < 1e-9)

    # Posing at zero must reproduce the rest skeleton exactly.
    zero = posed_skeleton(chain, flange, [0.0] * 6)
    for r, z in zip(skel, zero):
        dp = _norm(_sub(r["P"], z["P"]))
        dr = max(_norm(_sub(a, b)) for a, b in zip(r["transform"],
                                                     z["transform"]))
        check("%s posed at zero == rest" % r["name"], dp < 1e-12 and dr < 1e-12)

    # A posed skeleton must keep +Y down each bone: joint frames rotate with
    # their links, so the bone to the child rotates with them.
    q = [30.0, -60.0, 45.0, -20.0, 35.0, 10.0]
    posed = posed_skeleton(chain, flange, q)
    for a, b in zip(posed, posed[1:]):
        bone = _normalize(_sub(b["P"], a["P"]))
        check("posed %s +Y still points at %s" % (a["name"], b["name"]),
              abs(_dot(bone, a["transform"][1]) - 1.0) < 1e-9)

    # A hinge ('z') joint turning by q must rotate its frame by q about +Z,
    # measured by the angle between rest and posed +Y in the same pose with
    # only that joint moved. This is what makes sign +1 true, not assumed.
    for i, j in enumerate(skel):
        qq = [0.0] * 6
        qq[i] = 25.0
        moved = posed_skeleton(chain, flange, qq)[i]
        axis_row = {"x": 0, "y": 1, "z": 2}[j["rotation_axis"]]
        other = (axis_row + 1) % 3
        before, after = j["transform"][other], moved["transform"][other]
        about = j["transform"][axis_row]
        s = _dot(_cross(before, after), about)
        ang = math.degrees(math.atan2(s, _dot(before, after)))
        check("%s +25 deg reads back as %+.0f about %s"
              % (j["name"], 25.0 * j["sign"], j["rotation_axis"]),
              abs(ang - 25.0 * j["sign"]) < 1e-6, "got %.6f" % ang)

    axes, signs = profile_rig_fields(skel)

    # The profile must say what the geometry says. If the URDF or the builder
    # changes, this fails before a stale profile mis-drives the rig.
    import robot_profile
    prof = robot_profile.load("fr20", os.path.join(root, "profiles"))
    check("profile rotation_axis matches geometry",
          prof["rig"]["rotation_axis"] == axes,
          "profile %r, geometry %r" % (prof["rig"]["rotation_axis"], axes))
    check("profile sign matches geometry", prof["rig"]["sign"] == signs,
          "profile %r, geometry %r" % (prof["rig"]["sign"], signs))
    check("profile flange_offset_m matches the wrist3 mesh",
          abs(prof["rig"].get("flange_offset_m", 0.0) - flange) < 0.0005,
          "profile %r, mesh %.4f" % (prof["rig"].get("flange_offset_m"), flange))
    # The URDF stores radians rounded to 4 places (-3.0543 = -175.00007 deg),
    # so compare to a hundredth of a degree, not exactly.
    for i, (j, (lo, hi)) in enumerate(zip(chain, prof["robot"]["limits_deg"]),
                                      start=1):
        check("J%d profile limits match the URDF" % i,
              abs(j["lower_deg"] - lo) < 0.01 and abs(j["upper_deg"] - hi) < 0.01,
              "profile [%g, %g], URDF [%.5f, %.5f]"
              % (lo, hi, j["lower_deg"], j["upper_deg"]))

    print("robot:          %s" % parsed["name"])
    print("flange offset:  %.4f m" % flange)
    print("reach (chain):  %.4f m" % reach)
    print("rotation_axis:  %s" % axes)
    print("sign:           %s" % signs)
    print("limits (deg):   %s" % [(round(j["lower_deg"], 2),
                                   round(j["upper_deg"], 2)) for j in chain])
    print("urdf vel (deg/s): %s" % [round(j["velocity_deg_s"], 1)
                                    for j in chain])
    for j in skel:
        print("  %-8s P=(%.4f, %.4f, %.4f)  axis=%s sign=%+d"
              % (j["name"], j["P"][0], j["P"][1], j["P"][2],
                 j["rotation_axis"], j["sign"]))

    if fails:
        print("FAIL (%d)" % len(fails))
        for f in fails:
            print("  " + f)
        raise SystemExit(1)
    print("OK: skeleton frames are orthonormal, point down their bones, and "
          "the chain reproduces the datasheet reach")
