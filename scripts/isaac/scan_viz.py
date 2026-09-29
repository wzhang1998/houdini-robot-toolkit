"""The scan drawn over an Isaac Sim stage: the scan's area outlined on the
paper, the strip violet while its LEDs are on, the paper exposed so far
shaded violet -- shared by record_segments.py and run_show.py. Only what
the camera sees. Import after SimulationApp has started (it needs pxr).

    viz = ScanViz(stage, cfg)
    viz.update(u, leds_on, q)      # each drawn frame: u -1 before the pass, 0..1 in it, 2 after
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pxr import Gf, Sdf, UsdGeom  # noqa: E402

import show  # noqa: E402

VIOLET = (0.55, 0.2, 1.0)
HIDDEN = (0.0, 0.0, -5.0)


def curve(stage, path, rgb, width, closed=False):
    c = UsdGeom.BasisCurves.Define(stage, Sdf.Path(path))
    c.CreateTypeAttr("linear")
    c.CreateWrapAttr("periodic" if closed else "nonperiodic")
    c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    c.CreateWidthsAttr([width])
    c.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    return c


def set_points(c, pts):
    c.GetPointsAttr().Set([Gf.Vec3f(*p) for p in pts])
    c.GetCurveVertexCountsAttr().Set([len(pts)])


def area_corners(cfg, lift):
    """The scan area's corners on the paper's face, lift m in front of it:
    top left, top right, bottom right, bottom left as seen from the robot."""
    middle, w, h = show.scan_area(cfg)
    n = cfg["canvas"]["normal"]
    left = (-n[1], n[0], 0.0)
    off = cfg["canvas"].get("thickness", 0.02) / 2.0 + lift
    base = [middle[i] - n[i] * off for i in range(3)]
    return [[base[0] + left[0] * sx * w / 2, base[1] + left[1] * sx * w / 2, base[2] + sz * h / 2]
            for sx, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1))]


def exposed_quad(cfg, u):
    """The paper exposed at u (0..1 of the scan): the area's part the strip has passed."""
    tl, tr, br, bl = area_corners(cfg, 0.004)
    u = min(max(u, 0.0), 1.0)
    d = cfg["scan"].get("direction", "left_to_right")
    lerp = lambda a, b: [a[i] + (b[i] - a[i]) * u for i in range(3)]  # noqa: E731
    if d == "top_to_bottom":
        return [tl, tr, lerp(tr, br), lerp(tl, bl)]
    if d == "bottom_to_top":
        return [lerp(bl, tl), lerp(br, tr), br, bl]
    if d == "left_to_right":               # left as seen from the robot's side: tl is on its left
        return [tl, lerp(tl, tr), lerp(bl, br), bl]
    return [lerp(tr, tl), tr, br, lerp(br, bl)]


class ScanViz:
    def __init__(self, stage, cfg, root="/World/ScanViz"):
        import collision as C
        import gestures as G
        import robot_profile as RP
        self.cfg, self.rig = cfg, G.Rig()
        strip = C.strip_box(C.tool_def(RP.load("fr20")))
        self.half = (max(strip["size"]) if strip else 1.0) / 2.0
        UsdGeom.Scope.Define(stage, Sdf.Path(root))
        set_points(curve(stage, root + "/Area", (0.35, 0.35, 0.4), 0.006, closed=True), area_corners(cfg, 0.003))
        self.exposed = UsdGeom.Mesh.Define(stage, Sdf.Path(root + "/Exposed"))
        self.exposed.CreateFaceVertexCountsAttr([4])
        self.exposed.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        self.exposed.CreateDoubleSidedAttr(True)
        self.exposed.CreateDisplayColorAttr([Gf.Vec3f(*VIOLET)])
        self.exposed.CreatePointsAttr([Gf.Vec3f(*HIDDEN)] * 4)          # out of sight until a pass starts
        self.leds = curve(stage, root + "/LEDs", VIOLET, 0.03)
        set_points(self.leds, [HIDDEN, HIDDEN])

    def update(self, u, leds_on, q):
        """u: -1 before a pass (nothing exposed), 0..1 in it, above 1 after it
        (all of it); leds_on: the strip lit; q: the arm's joints (deg)."""
        self.exposed.GetPointsAttr().Set([Gf.Vec3f(*p) for p in exposed_quad(self.cfg, u)] if u > 0 else
                                         [Gf.Vec3f(*HIDDEN)] * 4)
        if leds_on:
            R, tcp, _ = self.rig.tool(q)
            ax = (R[0][1], R[1][1], R[2][1])                          # the strip along the flange's y
            set_points(self.leds, [[tcp[j] + ax[j] * k * self.half for j in range(3)] for k in (-1, 1)])
        else:
            set_points(self.leds, [HIDDEN, HIDDEN])
