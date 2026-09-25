"""Stage 3 check: a clip goes into Houdini and comes back out unchanged.

    hython scripts/roundtrip_check.py [clip.json | clip.csv]    (default: a sample dance clip)

A fresh scene, an FR20 (wenyi::robot_arm) whose FK joints read the clip
frame by frame -- the route the Dance Phrase node and the cell scene use --
exported again through the asset's own exporter (_collect, the rows the CSV
would hold). Every frame's six joints must come back within 0.01 deg.
Nothing is saved.

The asset's Import CSV button is not used: it builds nodes inside the asset,
which a locked instance (every matched instance, the FR20 scene's too)
refuses -- "Cannot create a node inside a locked asset".
"""

import os
import sys
import tempfile

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
sys.path.insert(0, ROOT + "/scripts")
import fairino_player as P  # noqa: E402
import motion_clip as M  # noqa: E402

src = sys.argv[1] if len(sys.argv) > 1 else ROOT + "/tests/clips/d17_punch-float-punch.json"
tmp = tempfile.mkdtemp().replace("\\", "/")
csv = tmp + "/roundtrip_in.csv"
if src.lower().endswith(".json"):
    M.to_csv(M.load(src), csv)
else:
    csv = src
t_in, q_in = P.load_csv(csv)

for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
    hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
hou.hipFile.clear(suppress_save_prompt=True)
hou.hipFile.setName(ROOT + "/scenes/_roundtrip.hiplc")          # $HIP/.. = the toolkit (not saved)
geo = hou.node("/obj").createNode("geo", "fr20", run_init_scripts=False)
arm = geo.createNode("wenyi::robot_arm::1.0", "robot_arm")
arm.parm("robot_profile").set("fr20")
arm.hdaModule().on_profile_changed(arm)
for n in ("quiet", "quiet_mode", "no_popups"):
    if arm.parm(n) is not None:
        arm.parm(n).set(1)
end = len(t_in)
hou.playbar.setFrameRange(1, end)
hou.playbar.setPlaybackRange(1, end)
SESSION = '''import sys
sys.path.insert(0, %r)
import fairino_player as _P
_T, _Q = _P.load_csv(%r)


def rt_joint(j):
    f = int(round(hou.frame())) - 1
    return _Q[max(0, min(len(_Q) - 1, f))][j - 1]
'''
hou.setSessionModuleSource(SESSION % (ROOT + "/scripts", csv))
arm.parm("pose_source").set(0)                                   # FK
for j in range(1, 7):
    arm.parm("fk_j%d" % j).setExpression("hou.session.rt_joint(%d)" % j, hou.exprLanguage.Python)
io = arm.node("robot_csv_io")
ns = {"hou": hou}
exec(io.type().definition().sections()["PythonModule"].contents(), ns)
ns["_QUIET"] = True
for n in ("frame_rangex", "frame_rangey"):
    p = io.parm(n)
    if p is not None and not p.isLocked():
        try:
            p.deleteAllKeyframes()
            p.set(1 if n.endswith("x") else end)
        except hou.PermissionError:
            pass
data = ns["_collect"](io, io.inputs()[0])
q_out = data["angles"]
n = min(len(q_out), len(q_in))
worst = max((abs(a - b), f + 1, j + 1) for f in range(n) for j, (a, b) in enumerate(zip(q_in[f], q_out[f])))
ok = len(q_out) == len(q_in) and worst[0] < 0.01
print("frames in %d, out %d; worst joint difference %.5f deg (frame %d, J%d)" % (len(q_in), len(q_out), *worst))
print("OK" if ok else "FAILED")
sys.exit(0 if ok else 1)
