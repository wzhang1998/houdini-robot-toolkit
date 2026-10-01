"""The tracked guests in Isaac: TouchDesigner's people_track (/track/people, robot frame) drawn as people -- a
head where it is, a body from the floor to the shoulders under it -- on a fixed number of prims (each id keeps
its prim while it is seen: track_osc.Slots), hidden while nobody is there. run_show draws them live and
records them (td_capture's people); replay_render draws them again.

    viz = PeopleViz(stage)
    viz.update([(id, x, y, z, ...)], now)

    python scripts/isaac/people_viz.py        self-test (the body's place; no Isaac)
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

HEAD_R, BODY_R, NECK_M = 0.11, 0.17, 0.2
SKIN, CLOTHES = (0.85, 0.7, 0.55), (0.25, 0.3, 0.45)
MAX_PEOPLE = 6


def body_of(head):
    """(centre, height) of the body column under a head (x, y, z): from the floor to NECK_M below the head."""
    h = max(0.2, head[2] - NECK_M)
    return (head[0], head[1], h / 2.0), h


class PeopleViz:
    def __init__(self, stage, path="/World/Guests", n=MAX_PEOPLE):
        from pxr import Gf, Sdf, UsdGeom
        import track_osc as TO
        self.Gf, self.UsdGeom, self.slots = Gf, UsdGeom, TO.Slots(n)
        self.prims = []
        for k in range(n):
            root = path + "/Guest%d" % (k + 1)
            head = UsdGeom.Sphere.Define(stage, Sdf.Path(root + "/Head"))
            head.CreateRadiusAttr(HEAD_R)
            head.CreateDisplayColorAttr([Gf.Vec3f(*SKIN)])
            body = UsdGeom.Cylinder.Define(stage, Sdf.Path(root + "/Body"))
            body.CreateRadiusAttr(BODY_R)
            body.CreateDisplayColorAttr([Gf.Vec3f(*CLOTHES)])
            self.prims.append((head, head.AddTranslateOp(), body, body.AddTranslateOp(), body.GetHeightAttr()))
            self._show(k, False)
        self.shown = [False] * n

    def _show(self, k, on):
        head, _, body, _, _ = self.prims[k]
        for p in (head, body):
            img = self.UsdGeom.Imageable(p.GetPrim())
            (img.MakeVisible if on else img.MakeInvisible)()

    def update(self, people, now):
        """people: [(id, x, y, z, ...)] now (heads, metres, the robot's frame)."""
        Gf = self.Gf
        where = self.slots.update([p[0] for p in people], now)
        seen = set()
        for p in people:
            k = where.get(p[0])
            if k is None:
                continue
            head, head_op, body, body_op, height = self.prims[k]
            head_op.Set(Gf.Vec3d(p[1], p[2], p[3]))
            c, h = body_of(p[1:4])
            body_op.Set(Gf.Vec3d(*c))
            height.Set(h)
            seen.add(k)
        for k in range(len(self.prims)):
            on = k in seen
            if on != self.shown[k]:
                self._show(k, on)
                self.shown[k] = on


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    c, h = body_of((-0.5, -1.9, 1.6))
    check("a body from the floor to the shoulders under the head",
          max(abs(a - b) for a, b in zip(c, (-0.5, -1.9, 0.7))) < 1e-9 and abs(h - 1.4) < 1e-9, (c, h))
    c, h = body_of((0.0, 0.0, 0.1))
    check("a head on the floor (a crouch gone wrong): a short body, not a negative one", h == 0.2 and c[2] == 0.1)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
