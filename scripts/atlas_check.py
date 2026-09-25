"""Stage 2 check: the baked atlas agrees with a direct measurement.

    hython scripts/atlas_check.py          (after build_atlas_scene.py --bake)

Loads scenes/FR20_atlas.hiplc, cooks atlas_merge (the slabs PDG wrote), and
compares 200 random voxels with capability.measure() at the voxel centre --
the same function, so they must agree exactly; a mismatch means the grid,
the slab merge or the Houdini -> URDF frame is off. Also: points beyond the
reach are 0, the viewers cook, and headroom is lowest on the J1 axis above
the base (the shoulder singularity), as it should be.
"""

import os
import random
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
sys.path.insert(0, ROOT + "/scripts")
import capability as C  # noqa: E402

fails = []


def check(label, ok, detail=""):
    print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
    if not ok:
        fails.append(label)


for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
    hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
hou.hipFile.load(ROOT + "/scenes/FR20_atlas.hiplc", suppress_save_prompt=True, ignore_load_warnings=True)
geo = hou.node("/obj/fr20_atlas")
atlas = geo.node("atlas")
merged = geo.node("atlas_merge").geometry()
vols = {p.attribValue("name"): p for p in merged.prims()}
check("merge has every field", {"reachable", "headroom", "wrist", "margin", "nsol"} <= set(vols),
      ", ".join(sorted(vols)))
v = vols["reachable"]
res = v.resolution()
print("     grid %d x %d x %d, voxel %.3f m, bake %.0f s of work over %d slabs"
      % (res[0], res[1], res[2], atlas.evalParm("voxel"),
         merged.attribValue("atlas_seconds"), merged.attribValue("atlas_slabs")))

model, chain, fo, vel = C.load_fr20(ROOT)
d_h = (atlas.evalParm("dirx"), atlas.evalParm("diry"), atlas.evalParm("dirz"))
to_urdf = lambda p: (p[0], -p[2], p[1])
random.seed(11)
bad, reach_n, n = [], 0, 200
while n > 0:
    idx = (random.randrange(res[0]), random.randrange(res[1]), random.randrange(res[2]))
    P = v.indexToPos(idx)
    m = C.measure(model, chain, to_urdf(tuple(P)), to_urdf(d_h), fo, vel, tool_len=atlas.evalParm("tool_len"))
    if not m["reachable"] and random.random() < 0.7:
        continue                                   # oversample the reachable part
    n -= 1
    got = {k: vols[k].voxel(idx) for k in ("reachable", "headroom", "wrist")}
    want = {"reachable": float(m["reachable"]), "headroom": m["headroom_mps"] if m["reachable"] else 0.0,
            "wrist": m["wrist"] if m["reachable"] else 0.0}
    # outside the radius / reach cut the bake skips the solve and writes 0
    import math
    if math.hypot(P[0], P[2]) > atlas.evalParm("radius") + 1e-9:
        want = {k: 0.0 for k in want}
    reach_n += int(want["reachable"])
    if any(abs(got[k] - want[k]) > 1e-4 for k in got):
        bad.append((idx, tuple(round(x, 3) for x in P), got, want))
check("200 random voxels equal a direct measurement at their centre", not bad,
      "%d reachable among them%s" % (reach_n, ("; first mismatch %s" % (bad[0],)) if bad else ""))

far = v.voxel(v.posToIndex(hou.Vector3(2.05, 0.2, 0.0)))
check("2.05 m out from the base axis is unreachable", far == 0.0)

h = vols["headroom"]
axis = h.voxel(h.posToIndex(hou.Vector3(0.0, 1.2, 0.0)))
side = h.voxel(h.posToIndex(hou.Vector3(0.9, 0.6, 0.0)))
check("headroom on the J1 axis above the base (shoulder singularity) is below the open workspace's",
      axis < side, "%.3f vs %.3f m/s" % (axis, side))

for name in ("to_vdb", "reach_shell", "shell_cutaway", "slice_look", "OUT"):
    node = geo.node(name)
    g = node.geometry()
    errs = node.errors()
    check("%s cooks" % name, g is not None and not errs and len(g.prims()) > 0,
          "%d prims%s" % (len(g.prims()) if g else 0, ("; " + errs[0]) if errs else ""))

print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
sys.exit(1 if fails else 0)
