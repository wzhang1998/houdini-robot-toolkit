"""A tool mounted on the flange, described as URDF: read and write.

The tool is its own small URDF (assets/tools/<tool>.urdf), the standard way
an end effector is described for ROS / MoveIt / Isaac Sim: a root link that
is the flange's mounting face (z out of the face; x and y as the arm's last
link), and links hanging from it by fixed joints, each with its visual and
collision geometry. A link without geometry named like a TCP (`*tcp*`) marks
the tool's working point (the LED face of the strip).

    tool = read("assets/tools/led_strip.urdf")
        {"name", "boxes": [{"name", "xyz", "rpy", "size", "R"}], "tcp": {"xyz", "rpy", "R"} or None}
        (xyz / R: each collision box's centre and axes in the mount frame, joints and origins composed)
    write(tool, path)      one link per box (visual = collision), fixed to the mount; the TCP link
    to_stl(tool, path)     the boxes as one STL in mm, for the controller's tool model (WebApp)

    python scripts/tool_urdf.py --stl assets/tools/led_strip.urdf assets/tools/led_strip.stl        (mm, CAD)
    python scripts/tool_urdf.py --stl assets/tools/led_strip.urdf assets/tools/led_strip_m.stl m    (m: the
        Fairino WebApp's tool model -- in mm it shows 1000x too large)

The robot's profile names its tool file ("tool": {"urdf": ...}); collision.py
reads it, so every check sees the tool. robot_show's Tool page edits it in
Houdini. Only box geometry (what the checks turn into capsules).

    python scripts/tool_urdf.py         self-test
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def rpy_matrix(rpy):
    """URDF's roll-pitch-yaw (fixed axes x, y, z): R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def matrix_rpy(R):
    """rpy of a rotation matrix (the inverse of rpy_matrix)."""
    p = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(math.cos(p)) > 1e-9:
        return (math.atan2(R[2][1], R[2][2]), p, math.atan2(R[1][0], R[0][0]))
    return (math.atan2(-R[1][2], R[1][1]), p, 0.0)


def _mul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _apply(A, v):
    return tuple(sum(A[i][k] * v[k] for k in range(3)) for i in range(3))


def _origin(el):
    o = el.find("origin") if el is not None else None
    xyz = [float(x) for x in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(x) for x in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return xyz, rpy


def read(path):
    """The tool from its URDF (see the module docstring)."""
    root = ET.parse(path).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    children = {j.find("child").get("link") for j in joints}
    mounts = [n for n in links if n not in children]
    if len(mounts) != 1:
        raise ValueError("%s: one root link (the mounting face) expected, found %s" % (path, mounts))
    for j in joints:
        if j.get("type") != "fixed":
            raise ValueError("%s: joint %s is %s -- a mounted tool has fixed joints only"
                             % (path, j.get("name"), j.get("type")))
    # every link's pose in the mount frame
    pose = {mounts[0]: ((0.0, 0.0, 0.0), rpy_matrix((0.0, 0.0, 0.0)))}
    pending = list(joints)
    while pending:
        progressed = False
        for j in list(pending):
            parent, child = j.find("parent").get("link"), j.find("child").get("link")
            if parent in pose:
                xyz, rpy = _origin(j)
                pp, pR = pose[parent]
                pose[child] = (tuple(a + b for a, b in zip(pp, _apply(pR, xyz))), _mul(pR, rpy_matrix(rpy)))
                pending.remove(j)
                progressed = True
        if not progressed:
            raise ValueError("%s: joints not connected to %s" % (path, mounts[0]))
    boxes, tcp = [], None
    for name, link in links.items():
        lp, lR = pose[name]
        cols = link.findall("collision")
        if not cols and "tcp" in name.lower():
            tcp = {"name": name, "xyz": list(lp), "R": lR, "rpy": list(matrix_rpy(lR))}
        for k, col in enumerate(cols):
            box = col.find("geometry/box")
            if box is None:
                raise ValueError("%s: link %s has a collision that is not a box" % (path, name))
            xyz, rpy = _origin(col)
            R = _mul(lR, rpy_matrix(rpy))
            boxes.append({"name": name if len(cols) == 1 else "%s_%d" % (name, k),
                          "xyz": [a + b for a, b in zip(lp, _apply(lR, xyz))], "R": R, "rpy": list(matrix_rpy(R)),
                          "size": [float(x) for x in box.get("size").split()]})
    return {"name": root.get("name"), "boxes": boxes, "tcp": tcp}


def _f(v):
    return " ".join("%.9g" % x for x in v)


def write(tool, path, note=None):
    """The tool as URDF: a link per box (its visual the same box), fixed to
    the mount at the box's pose; the TCP as a link without geometry."""
    lines = ['<?xml version="1.0"?>']
    if note:
        lines.append("<!-- %s -->" % note.replace("--", "-"))
    lines.append('<robot name="%s">' % tool.get("name", "tool"))
    lines.append('  <link name="tool_mount"/>')
    for b in tool["boxes"]:
        geo = '<geometry><box size="%s"/></geometry>' % _f(b["size"])
        lines += ['  <link name="%s">' % b["name"],
                  '    <visual>', '      <origin xyz="0 0 0" rpy="0 0 0"/>', '      ' + geo, '    </visual>',
                  '    <collision>', '      <origin xyz="0 0 0" rpy="0 0 0"/>', '      ' + geo, '    </collision>',
                  '  </link>',
                  '  <joint name="tool_mount_to_%s" type="fixed">' % b["name"],
                  '    <parent link="tool_mount"/>', '    <child link="%s"/>' % b["name"],
                  '    <origin xyz="%s" rpy="%s"/>' % (_f(b["xyz"]), _f(b.get("rpy", (0, 0, 0)))),
                  '  </joint>']
    t = tool.get("tcp")
    if t:
        name = t.get("name", "tcp")
        lines += ['  <link name="%s"/>' % name,
                  '  <joint name="tool_mount_to_%s" type="fixed">' % name,
                  '    <parent link="tool_mount"/>', '    <child link="%s"/>' % name,
                  '    <origin xyz="%s" rpy="%s"/>' % (_f(t["xyz"]), _f(t.get("rpy", (0, 0, 0)))),
                  '  </joint>']
    lines.append("</robot>")
    with open(path, "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return path


def to_stl(tool, path, scale=1000.0):
    """The tool's boxes as one binary STL, in the mount frame (origin at the
    flange's mounting face, z out of it), scaled (1000: millimetres, what a
    controller's tool-model import and CAD expect). Outward normals."""
    import struct
    tris = []
    for b in tool["boxes"]:
        hx, hy, hz = (s / 2.0 for s in b["size"])
        R, c = b["R"], b["xyz"]
        corner = lambda sx, sy, sz: tuple(scale * (c[i] + R[i][0] * sx * hx + R[i][1] * sy * hy + R[i][2] * sz * hz)
                                          for i in range(3))
        # each face: its outward axis (local), and its 4 corners counter-clockwise seen from outside
        for ax, sgn in ((0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)):
            u, v = (ax + 1) % 3, (ax + 2) % 3             # u x v = +ax (right-handed)
            if sgn < 0:
                u, v = v, u
            quad = []
            for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                s = [0, 0, 0]
                s[ax], s[u], s[v] = sgn, su, sv
                quad.append(corner(*s))
            n = tuple(R[i][ax] * sgn for i in range(3))
            tris += [(n, quad[0], quad[1], quad[2]), (n, quad[0], quad[2], quad[3])]
    with open(path, "wb") as f:
        f.write(("%s tool, units %s, origin: flange mounting face, z out of it"
                 % (tool.get("name", "tool"), "mm" if scale == 1000.0 else "x%g m" % scale)).encode()[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for n, a, b_, c_ in tris:
            f.write(struct.pack("<12fH", *n, *a, *b_, *c_, 0))
    return path


def self_test():
    import tempfile
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    t = read(os.path.join(ROOT, "assets", "tools", "led_strip.urdf"))
    strip = {b["name"]: b for b in t["boxes"]}.get("strip")
    check("the LED strip reads: a bracket, a 1 m strip 6 cm out, the LED face as its TCP",
          strip and abs(strip["size"][1] - 1.0) < 1e-9 and abs(strip["xyz"][2] - 0.06) < 1e-9
          and t["tcp"] and abs(t["tcp"]["xyz"][2] - 0.07) < 1e-9 and len(t["boxes"]) == 2, t)
    rpy = (0.3, -0.4, 1.2)
    back = matrix_rpy(rpy_matrix(rpy))
    check("roll-pitch-yaw round-trips through a matrix", all(abs(a - b) < 1e-9 for a, b in zip(rpy, back)), back)
    # a box turned and offset through a chain of fixed joints
    turned = {"name": "t", "boxes": [{"name": "arm", "xyz": [0.1, 0.0, 0.2], "rpy": [0.0, 0.0, math.pi / 2],
                                      "size": [0.5, 0.02, 0.02]}], "tcp": {"name": "tip_tcp", "xyz": [0, 0, 0.3]}}
    p = os.path.join(tempfile.mkdtemp(), "t.urdf")
    write(turned, p)
    r = read(p)
    b = r["boxes"][0]
    check("a written tool reads back the same (pose, size, TCP)",
          all(abs(x - y) < 1e-6 for x, y in zip(b["xyz"], [0.1, 0.0, 0.2]))
          and abs(b["R"][1][0] - 1.0) < 1e-6 and b["size"] == [0.5, 0.02, 0.02]
          and abs(r["tcp"]["xyz"][2] - 0.3) < 1e-9, (b, r["tcp"]))
    import struct
    stl = os.path.join(tempfile.mkdtemp(), "t.stl")
    to_stl(t, stl)
    data = open(stl, "rb").read()
    n = struct.unpack_from("<I", data, 80)[0]
    vol = 0.0
    for k in range(n):
        f = struct.unpack_from("<12f", data, 84 + 50 * k)
        a, b_, c_ = f[3:6], f[6:9], f[9:12]
        vol += (a[0] * (b_[1] * c_[2] - b_[2] * c_[1]) - a[1] * (b_[0] * c_[2] - b_[2] * c_[0])
                + a[2] * (b_[0] * c_[1] - b_[1] * c_[0])) / 6.0
    want = sum(b["size"][0] * b["size"][1] * b["size"][2] for b in t["boxes"]) * 1e9
    check("the STL (mm) closes around every box, faces outward: its volume is the boxes'",
          n == 12 * len(t["boxes"]) and abs(vol - want) < 1e-3 * want, (n, round(vol), round(want)))
    print("tool_urdf self-test: %s" % ("PASS" if not fails else "FAIL: %s" % fails))
    return not fails


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "--stl":      # python scripts/tool_urdf.py --stl tool.urdf out.stl [m]
        m = len(sys.argv) > 4 and sys.argv[4] == "m"         # metres: the controller WebApp's 3D view reads them so
        print("wrote", to_stl(read(sys.argv[2]), sys.argv[3], scale=1.0 if m else 1000.0), "in", "m" if m else "mm")
        sys.exit(0)
    sys.exit(0 if self_test() else 1)
