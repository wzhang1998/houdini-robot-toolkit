"""TouchDesigner's canvas preview on the paper in Isaac Sim: a grid of small
quads over the scan's area, each coloured as a pixel of the image TD sends
(canvas_link.Receiver) -- the paper as the scan leaves it, live. A grid, not
a texture: it renders the same in a window, headless and offline.

    viz = CanvasViz(stage, cfg)
    viz.update(w, h, rgb)      # an image: rows top first, columns left to right as seen from the robot
    viz.hide()

    python scripts/isaac/canvas_viz.py --self-test        the grid (no Isaac needed)
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

LIFT_M = 0.005                        # just in front of the paper (the scan area's outline is at 3 mm)


def grid(corners, w, h):
    """(points, face vertex counts, face indices) of a w x h grid over the
    quad corners (top left, top right, bottom right, bottom left); face k =
    row * w + column, row 0 at the top, column 0 at the left."""
    tl, tr, br, bl = corners
    pts = []
    for r in range(h + 1):
        fv = r / float(h)
        left = [tl[i] + (bl[i] - tl[i]) * fv for i in range(3)]
        right = [tr[i] + (br[i] - tr[i]) * fv for i in range(3)]
        for c in range(w + 1):
            fu = c / float(w)
            pts.append([left[i] + (right[i] - left[i]) * fu for i in range(3)])
    idx = []
    for r in range(h):
        for c in range(w):
            a = r * (w + 1) + c
            idx += [a, a + 1, a + w + 2, a + w + 1]
    return pts, [4] * (w * h), idx


def colours(rgb):
    """RGB8 bytes as (r, g, b) floats 0..1 per pixel."""
    return [(rgb[k] / 255.0, rgb[k + 1] / 255.0, rgb[k + 2] / 255.0) for k in range(0, len(rgb), 3)]


class CanvasViz:
    def __init__(self, stage, cfg, path="/World/CanvasPreview"):
        from pxr import Sdf, UsdGeom
        from scan_viz import area_corners
        self.corners = area_corners(cfg, LIFT_M)
        self.mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        self.mesh.CreateDoubleSidedAttr(True)
        self.mesh.CreateSubdivisionSchemeAttr("none")
        self.color = self.mesh.CreateDisplayColorAttr()
        UsdGeom.Primvar(self.color).SetInterpolation(UsdGeom.Tokens.uniform)
        self.size = None
        self.hide()

    def update(self, w, h, rgb):
        from pxr import Gf, UsdGeom
        if self.size != (w, h):
            pts, counts, idx = grid(self.corners, w, h)
            self.mesh.GetPointsAttr().Set([Gf.Vec3f(*p) for p in pts])
            self.mesh.GetFaceVertexCountsAttr().Set(counts)
            self.mesh.GetFaceVertexIndicesAttr().Set(idx)
            self.size = (w, h)
            UsdGeom.Imageable(self.mesh).MakeVisible()
        import numpy as np
        from pxr import Vt
        px = np.frombuffer(rgb, dtype=np.uint8).reshape(-1, 3).astype(np.float32) / 255.0
        self.color.Set(Vt.Vec3fArray.FromNumpy(px))              # one array, not 8100 Python objects

    def hide(self):
        from pxr import UsdGeom
        UsdGeom.Imageable(self.mesh).MakeInvisible()
        self.size = None


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    corners = [[0.0, 0.0, 1.0], [3.0, 0.0, 1.0], [3.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    pts, counts, idx = grid(corners, 3, 2)
    check("a w x h grid: (w+1)(h+1) points, w*h quads", len(pts) == 12 and counts == [4] * 6 and len(idx) == 24)
    first = [pts[i] for i in idx[:4]]
    last = [pts[i] for i in idx[-4:]]
    check("face 0 is the top left pixel, the last the bottom right",
          first[0] == [0.0, 0.0, 1.0] and first[2] == [1.0, 0.0, 0.5] and last[2] == [3.0, 0.0, 0.0], (first, last))
    check("pixels as colours 0..1", colours(bytes([255, 0, 51])) == [(1.0, 0.0, 0.2)])
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    print(__doc__)
