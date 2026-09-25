"""Stage 3 check: a clip goes into Houdini and comes back out unchanged.

    hython scripts/roundtrip_check.py [clip.json | clip.csv] [--profile uf850] [--fk]

A fresh scene and a wenyi::robot_arm (locked, as every matched instance
is); the clip goes in through the asset's own Import CSV button (the file
is then read live, frame by frame) -- or, with --fk, through FK joints
driven by an expression, the route the Dance Phrase node uses -- and comes
out through the asset's exporter (_collect, the rows the CSV would hold).
Every frame's six joints must come back within 0.01 deg. Nothing is saved.
"""

import os
import sys
import tempfile

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
sys.path.insert(0, ROOT + "/scripts")
import fairino_player as P  # noqa: E402
import motion_clip as M  # noqa: E402

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
PROFILE = next((sys.argv[i + 1] for i, a in enumerate(sys.argv) if a == "--profile"), "fr20")
ARGS = [a for a in ARGS if a != PROFILE]
USE_FK = "--fk" in sys.argv
src = ARGS[0] if ARGS else ROOT + "/tests/clips/d17_punch-float-punch.json"
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
arm.parm("robot_profile").set(PROFILE)
arm.hdaModule().on_profile_changed(arm)
for n in ("quiet", "quiet_mode", "no_popups"):
    if arm.parm(n) is not None:
        arm.parm(n).set(1)
end = len(t_in)
hou.playbar.setFrameRange(1, end)
hou.playbar.setPlaybackRange(1, end)
if USE_FK:
    SESSION = '''import sys
sys.path.insert(0, %r)
import fairino_player as _P
_T, _Q = _P.load_csv(%r)


def rt_joint(j):
    f = int(round(hou.frame())) - 1
    return _Q[max(0, min(len(_Q) - 1, f))][j - 1]
'''
    hou.setSessionModuleSource(SESSION % (ROOT + "/scripts", csv))
    arm.parm("pose_source").set(0)                               # FK
    for j in range(1, 7):
        arm.parm("fk_j%d" % j).setExpression("hou.session.rt_joint(%d)" % j, hou.exprLanguage.Python)
else:
    arm.parm("import_csv").set(src if src.lower().endswith(".json") else csv)
    arm.parm("import_btn").pressButton()                         # checks, sets the range, Pose Source -> Imported CSV
    print("locked instance: %s; pose source %s; status: %s" % (
        arm.matchesCurrentDefinition(), arm.parm("pose_source").evalAsString(),
        arm.node("robot_csv_io").parm("status").eval() if arm.node("robot_csv_io").parm("status") else ""))
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
# compare by time: frame f shows the clip at (f - 1) / fps (a clip need not
# be one row per frame)
fps = hou.fps()
import bisect  # noqa: E402


def at(s):
    if s <= t_in[0]:
        return q_in[0]
    if s >= t_in[-1]:
        return q_in[-1]
    i = bisect.bisect_right(t_in, s) - 1
    f = (s - t_in[i]) / (t_in[i + 1] - t_in[i])
    return [a + f * (b - a) for a, b in zip(q_in[i], q_in[i + 1])]


n_expect = int(round((t_in[-1] - t_in[0]) * fps)) + 1
worst = max((abs(a - b), f + 1, j + 1) for f, row in enumerate(q_out)
            for j, (a, b) in enumerate(zip(at(t_in[0] + f / fps), row)))
ok = len(q_out) == n_expect and worst[0] < 0.01
print("clip %.2f s (%d rows) -> %d frames, expected %d; worst joint difference %.5f deg (frame %d, J%d)"
      % (t_in[-1] - t_in[0], len(t_in), len(q_out), n_expect, *worst))
print("OK" if ok else "FAILED")
sys.exit(0 if ok else 1)
