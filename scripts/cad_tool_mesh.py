"""A tool's CAD (STEP, converted to USD by Isaac's HOOPS converter) as one plain mesh in the tool's frame, for
the display: every mesh of the converted file merged, scaled to metres, turned about z and moved as the tool
sits on the flange -- no instancing, no references, no transform left to get wrong where it is referenced.

    C:/isaacsim6/python.bat scripts/isaac/step_to_usd.py IN.stp OUT.usd     (Isaac's HOOPS: STEP -> USD, in mm)
    uv run scripts/cad_tool_mesh.py assets/tools/cad/FULLUVBAR_forWenyi.usd assets/tools/cad/uv_bar_visual.usda \\
        --scale 0.001 --rotate-z 90 --translate 0 0 0.02975

The UV bar: the CAD's x along the bar, z = 0 its middle, the plate under it; on the flange the bar runs along
the flange's y (rotate z 90) and its middle is 29.75 mm out (assets/tools/uv_bar.urdf).

    uv run scripts/cad_tool_mesh.py --self-test
"""

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)


def transform(p, scale, rot_z_deg, t):
    """A point of the CAD (its units) in the tool's frame (m): scaled, turned about z, moved."""
    c, s = math.cos(math.radians(rot_z_deg)), math.sin(math.radians(rot_z_deg))
    x, y, z = p[0] * scale, p[1] * scale, p[2] * scale
    return (c * x - s * y + t[0], s * x + c * y + t[1], z + t[2])


def merge(src, dst, scale, rot_z_deg, t):
    """Every mesh of src (instance proxies too, world transforms applied) as one mesh in dst. (points, faces)."""
    from pxr import Gf, Usd, UsdGeom
    st = Usd.Stage.Open(src)
    xc = UsdGeom.XformCache()
    pts, counts, idx = [], [], []
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        P, C, I = m.GetPointsAttr().Get(), m.GetFaceVertexCountsAttr().Get(), m.GetFaceVertexIndicesAttr().Get()
        if not P or not C or not I:
            continue
        M = xc.GetLocalToWorldTransform(p)
        base = len(pts)
        pts += [transform(M.Transform(Gf.Vec3d(q)), scale, rot_z_deg, t) for q in P]
        counts += list(C)
        idx += [base + i for i in I]
    out = Usd.Stage.CreateNew(dst)
    UsdGeom.SetStageMetersPerUnit(out, 1.0)
    UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(out, "/tool")
    out.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(out, "/tool/mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateDoubleSidedAttr(True)                     # the CAD is surfaces, not closed solids
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.78, 0.8, 0.82)])
    lo = [min(q[i] for q in pts) for i in range(3)]
    hi = [max(q[i] for q in pts) for i in range(3)]
    mesh.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    out.GetRootLayer().Save()
    return pts, counts, (lo, hi)


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    p = transform((543.75, 0.0, 17.6), 0.001, 90.0, (0.0, 0.0, 0.02975))
    check("the bar's end (CAD x, mm) on the flange's y, its LED face 47.35 mm out",
          abs(p[0]) < 1e-9 and abs(p[1] - 0.54375) < 1e-9 and abs(p[2] - 0.04735) < 1e-9, p)
    p = transform((0.0, 58.0, -29.75), 0.001, 90.0, (0.0, 0.0, 0.02975))
    check("the plate's underside on the flange's face (z 0), its edge on -x", abs(p[2]) < 1e-9
          and abs(p[0] + 0.058) < 1e-9, p)
    visual = os.path.join(ROOT, "assets", "tools", "cad", "uv_bar_visual.usda")
    if os.path.exists(visual):
        import collision as C
        import robot_profile as RP
        from pxr import Usd, UsdGeom
        st = Usd.Stage.Open(visual)
        cad = []
        for prim in st.Traverse():
            if prim.IsA(UsdGeom.Mesh):
                xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                cad += [tuple(xf.Transform(v)) for v in UsdGeom.Mesh(prim).GetPointsAttr().Get() or []]
        caps = C.tool_capsules(C.tool_def(RP.load("fr20")), 0.0)
        out = [q for q in cad if not any(C._seg_point_dist(c["a"], c["b"], q) <= c["r"] + 1e-6 for c in caps)]
        front = max(max(c["a"][2], c["b"][2]) + c["r"] for c in caps)
        check("the bar's collision model holds every CAD point (%d), its front at most 1 mm past the shade's rim "
              "(47.35 mm: a scan's gap can be what it says)" % len(cad),
              cad and not out and front <= 0.04735 + 0.001, (len(out), round(1e3 * front, 2)))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--rotate-z", type=float, default=0.0)
    ap.add_argument("--translate", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    a = ap.parse_args(argv)
    pts, faces, (lo, hi) = merge(a.src, a.dst, a.scale, a.rotate_z, a.translate)
    print("wrote %s: %d points, %d faces, x %.4f..%.4f y %.4f..%.4f z %.4f..%.4f m"
          % (a.dst, len(pts), len(faces), lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    return 0


if __name__ == "__main__":
    sys.exit(self_test() if "--self-test" in sys.argv else main())
