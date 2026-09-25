"""Build scenes/FR20_cell.hiplc -- the robot in its cell (real2sim).

    hython scripts/build_cell_scene.py [clip]      (default tests/csv/fr20_test.csv)

    /obj/CELL_CTRL   Clip (a joint CSV or a factory clip JSON), Environment
                     (envs/*.json), Tool Length; Fit Range to the clip
    /obj/cell_env    the environment, coloured by role (scripts/cell_sop.py)
    /obj/fr20        wenyi::robot_arm, FK, its joints read from the clip at
                     the current frame (hou.session.joint(n)); Pre-Flight's
                     Cell check uses the same environment
    /obj/capsules    the collision capsules at this frame, green = clear ..
                     red = at the margin; detail min_clearance_m / nearest
    /obj/ghosts      (display off) onion skin of the whole clip: capsules at
                     Count frames, blue = start .. red = end, and the TCP path

The scene is generated -- rebuild it rather than hand-edit it. Change the
clip or the environment on CELL_CTRL.
"""

import os
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
SCENE = ROOT + "/scenes/FR20_cell.hiplc"

SESSION = '''import sys, hou
_p = hou.text.expandString("$HIP/../scripts")
if _p not in sys.path:
    sys.path.insert(0, _p)
import cell_sop


def joint(j):
    return cell_sop.joint(j)
'''

SOP = '''import hou
hou.session.cell_sop.%s(hou.pwd())
'''


def build(clip=None):
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    hou.hipFile.clear(suppress_save_prompt=True)
    hou.hipFile.setName(SCENE)
    hou.setSessionModuleSource(SESSION)
    obj = hou.node("/obj")

    ctrl = obj.createNode("null", "CELL_CTRL")
    g = ctrl.parmTemplateGroup()
    g.append(hou.StringParmTemplate("clip", "Clip", 1, string_type=hou.stringParmType.FileReference,
                                    default_value=(clip or "$HIP/../tests/csv/fr20_test.csv",),
                                    help="A joint CSV (the asset's export) or a clip JSON (geo/clips, geo/dance, tests/clips)"))
    g.append(hou.StringParmTemplate("env", "Environment", 1, string_type=hou.stringParmType.FileReference,
                                    default_value=("$HIP/../envs/volvox_lab.json",)))
    g.append(hou.FloatParmTemplate("tool_len", "Tool Length (m)", 1, default_value=(0.15,), min=0.0, max=0.5,
                                   help="A capsule this long past the flange stands in for the tool"))
    g.append(hou.ButtonParmTemplate("fit_range", "Fit Range to Clip", script_callback="hou.session.cell_sop.fit_range()",
                                    script_callback_language=hou.scriptLanguage.Python))
    ctrl.setParmTemplateGroup(g)

    env = obj.createNode("geo", "cell_env", run_init_scripts=False)
    s = env.createNode("python", "env")
    s.parm("python").set(SOP % "env_geo")
    s.setDisplayFlag(True)

    rob = obj.createNode("geo", "fr20", run_init_scripts=False)
    arm = rob.createNode("wenyi::robot_arm::1.0", "robot_arm")
    arm.parm("robot_profile").set("fr20")
    arm.hdaModule().on_profile_changed(arm)
    arm.parm("pose_source").set(0)                   # FK
    for j in range(1, 7):
        arm.parm("fk_j%d" % j).setExpression("hou.session.joint(%d)" % j, hou.exprLanguage.Python)
    arm.parm("env_file").set('`chs("/obj/CELL_CTRL/env")`')
    arm.setDisplayFlag(True)

    caps = obj.createNode("geo", "capsules", run_init_scripts=False)
    s = caps.createNode("python", "capsules")
    s.parm("python").set(SOP % "capsule_geo")
    s.setDisplayFlag(True)
    gh = obj.createNode("geo", "ghosts", run_init_scripts=False)
    s = gh.createNode("python", "ghosts")
    g = s.parmTemplateGroup()
    g.append(hou.IntParmTemplate("count", "Count", 1, default_value=(8,), min=2, max=40))
    s.setParmTemplateGroup(g)
    s.parm("python").set(SOP % "ghost_geo")
    wire = gh.createNode("polywire", "tcp_wire")
    wire.setInput(0, s)
    wire.parm("group").set("@name=tcp_path")
    wire.parm("radius").set(0.008)
    m = gh.createNode("merge", "OUT")
    m.setInput(0, s)
    m.setInput(1, wire)
    m.setDisplayFlag(True)
    gh.setDisplayFlag(False)                         # onion skin: turn on to review a whole phrase
    obj.layoutChildren()
    hou.session.cell_sop.fit_range()
    hou.hipFile.save(SCENE)
    return ctrl


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else None)
    print("built", SCENE)
