"""The LED strip as its 60 LEDs, each lit to the level TouchDesigner sends
(artnet.Receiver): dots on the strip's face, moved with the arm by FK of the
commanded joints (as scan_viz draws the scan). A preview of TD's idle and scan
LEDs, live -- run_show.py --artnet.

    viz = LedViz(stage)
    viz.update(levels, q)       # each drawn frame: 60 levels 0..1 (None: all dark), joints (deg)

    python scripts/isaac/led_viz.py --self-test        the positions (no Isaac needed)

LED 0 is at the strip's -y end (the flange's y) until the strip is wired and
pixel_scan's `LED 0 at the Bottom` says otherwise (led0="plus").
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

LEDS = 60
LIT = (0.62, 0.3, 1.0)               # UV violet
GLOW = 3.0                           # a lit LED glows past white paper in the lit room (HDR colour)
EYE_GAMMA = 2.2                      # an LED at a tenth of its power still looks bright: levels as seen
DARK = (0.06, 0.06, 0.07)
DOT_M = 0.012
LIFT_M = 0.004                       # the dots just in front of the strip's face


def strip_geometry(profile="fr20"):
    """(offset along the tool's z to the LEDs' face, the strip's length) from
    the profile's tool (collision.strip_box: its box in the flange's frame)."""
    import collision as C
    import robot_profile as RP
    box = C.strip_box(C.tool_def(RP.load(profile)))
    if not box:
        raise SystemExit("the profile's tool has no strip")
    return box["xyz"][2] + box["size"][2] / 2.0 + LIFT_M, box["size"][1]


def led_positions(R, tcp, face, length, n=LEDS, led0="minus"):
    """World positions of n LEDs spread evenly along the flange's y, on the
    face `face` m out along its z from the tcp (the flange's face)."""
    ax = (R[0][1], R[1][1], R[2][1])
    d = (R[0][2], R[1][2], R[2][2])
    sign = 1.0 if led0 == "minus" else -1.0
    pts = []
    for i in range(n):
        s = sign * (-length / 2.0 + length * (i + 0.5) / n)
        pts.append([tcp[j] + d[j] * face + ax[j] * s for j in range(3)])
    return pts


def colour(level, gain=1.0):
    """A dot's colour at a level: dark grey off, glowing violet lit, the level
    as the eye sees it (EYE_GAMMA); gain: see a dim pattern -- the
    preview's, not the strip's."""
    k = min(1.0, max(0.0, level * gain)) ** (1.0 / EYE_GAMMA)
    return tuple(DARK[i] + (LIT[i] * GLOW - DARK[i]) * k for i in range(3))


def glow(stage, path, prim):
    """A material that glows in each point's displayColor (lit LEDs read in a lit room)."""
    from pxr import Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    reader = UsdShade.Shader.Define(stage, Sdf.Path(path + "/Color"))
    reader.CreateIdAttr("UsdPrimvarReader_float3")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
    out = reader.CreateOutput("result", Sdf.ValueTypeNames.Float3)
    surf = UsdShade.Shader.Define(stage, Sdf.Path(path + "/Surface"))
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(out)
    surf.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(out)
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


class LedViz:
    def __init__(self, stage, path="/World/LedViz", n=LEDS, led0="minus", gain=1.0):
        from pxr import Gf, Sdf, UsdGeom
        import gestures as G
        self.Gf, self.rig, self.n, self.led0, self.gain = Gf, G.Rig(), n, led0, gain
        self.face, self.length = strip_geometry()
        self.pts = UsdGeom.Points.Define(stage, Sdf.Path(path))
        self.pts.CreatePointsAttr([Gf.Vec3f(0.0, 0.0, -5.0)] * n)
        self.pts.CreateWidthsAttr([DOT_M] * n)
        self.pts.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        self.color = self.pts.CreateDisplayColorAttr([Gf.Vec3f(*DARK)] * n)
        UsdGeom.Primvar(self.color).SetInterpolation(UsdGeom.Tokens.vertex)
        glow(stage, path + "/Glow", self.pts.GetPrim())

    def update(self, levels, q):
        R, tcp, _ = self.rig.tool(q)
        Gf = self.Gf
        self.pts.GetPointsAttr().Set([Gf.Vec3f(*p) for p in led_positions(R, tcp, self.face, self.length, self.n,
                                                                           self.led0)])
        lv = levels or [0.0] * self.n
        self.color.Set([Gf.Vec3f(*colour(x, self.gain)) for x in lv[:self.n]] + [Gf.Vec3f(*DARK)] * (self.n - len(lv)))


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    near = lambda a, b: all(abs(x - y) < 1e-9 for x, y in zip(a, b))  # noqa: E731
    I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    pts = led_positions(I, (0.0, 0.0, 0.0), 0.1, 1.2, n=60)
    step = pts[1][1] - pts[0][1]
    check("60 LEDs evenly along the flange's y, the strip's length, on its face",
          len(pts) == 60 and abs(step - 0.02) < 1e-9 and abs(pts[0][1] + 0.59) < 1e-9
          and all(abs(p[2] - 0.1) < 1e-9 and p[0] == 0.0 for p in pts), (pts[0], pts[-1]))
    flip = led_positions(I, (0.0, 0.0, 0.0), 0.1, 1.2, n=60, led0="plus")
    check("... LED 0 at the other end when the strip is wired the other way", flip[0] == pts[-1] and flip[-1] == pts[0])
    face, length = strip_geometry()
    check("the strip's face and length come from the profile's tool (FR20: 1 m, ~7 cm out)",
          abs(length - 1.0) < 1e-6 and 0.05 < face < 0.1, (face, length))
    check("dark off, glowing violet at full, the gain brightens a dim level up to full",
          near(colour(0.0), DARK) and near(colour(1.0), [x * GLOW for x in LIT])
          and near(colour(0.1, 10.0), colour(1.0)) and colour(0.5)[2] < colour(1.0)[2] - 0.1)
    check("a tenth of the power looks a third as bright, not a tenth (the eye)",
          0.3 < colour(0.1)[2] / colour(1.0)[2] < 0.42,
          round(colour(0.1)[2] / colour(1.0)[2], 3))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    print(__doc__)
