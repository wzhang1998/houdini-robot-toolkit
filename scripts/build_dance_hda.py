"""Build otls/sop_wenyi.dance_phrase.1.0.hdalc and scenes/FR20_dance.hiplc.

    hython scripts/build_dance_hda.py

wenyi::dance_phrase (SOP, no inputs) wraps scripts/choreo.py -- see
scripts/hda/dance_phrase_module.py for what it does. The scene: a phrase
node, an FR20 (wenyi::robot_arm) whose FK joints it drives, the room shown,
and one phrase already generated (float -> punch -> glide).
"""

import os
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
HDA = ROOT + "/otls/sop_wenyi.dance_phrase.1.0.hdalc"
SCENE = ROOT + "/scenes/FR20_dance.hiplc"
TYPE = "wenyi::dance_phrase::1.0"
ACTIONS = ["punch", "slash", "press", "wring", "dab", "flick", "glide", "float"]
LABELS = ["Punch  (strong, sudden, direct)", "Slash  (strong, sudden, indirect)",
          "Press  (strong, sustained, direct)", "Wring  (strong, sustained, indirect)",
          "Dab  (light, sudden, direct)", "Flick  (light, sudden, indirect)",
          "Glide  (light, sustained, direct)", "Float  (light, sustained, indirect)"]


def _cb(fn):
    return "hou.phm().%s(kwargs['node'])" % fn


def parm_group():
    g = hou.ParmTemplateGroup()
    ph = hou.FolderParmTemplate("phrase_folder", "Phrase", folder_type=hou.folderType.Tabs)
    bars = hou.FolderParmTemplate("bars", "Bars", folder_type=hou.folderType.MultiparmBlock, default_value=3)
    bars.addParmTemplate(hou.MenuParmTemplate("action#", "Action", ACTIONS, LABELS, default_value=7,
                                              help="Laban effort action for this bar (4 beats)"))
    ph.addParmTemplate(bars)
    ph.addParmTemplate(hou.IntParmTemplate("bpm", "Tempo (bpm)", 1, default_value=(90,), min=50, max=150))
    ph.addParmTemplate(hou.FloatParmTemplate(
        "flow", "Flow", 1, default_value=(0.0,), min=-1.0, max=1.0,
        help="-1 bound (a held beat after every move, crisp stops) .. +1 free (moves overlap, the arm breathes)"))
    ph.addParmTemplate(hou.IntParmTemplate("seed", "Seed", 1, default_value=(0,), min=0, max=999,
                                           help="Which poses and variations the phrase picks"))
    ph.addParmTemplate(hou.FloatParmTemplate(
        "acc", "Plan Acceleration (deg/s²)", 1, default_value=(0.0,), min=0.0, max=1200.0,
        help="0: the profile's (FR20 150). Put what scripts/accel_probe.py measured on the real arm -- "
             "a higher limit lets sudden moves be bigger."))
    ph.addParmTemplate(hou.StringParmTemplate("env_file", "Cell Environment", 1,
                                              default_value=("$HIP/../envs/volvox_lab.json",),
                                              string_type=hou.stringParmType.FileReference,
                                              help="The room the phrase must clear (empty: not checked)"))
    ph.addParmTemplate(hou.ButtonParmTemplate("generate_btn", "Generate", script_callback=_cb("generate"),
                                              script_callback_language=hou.scriptLanguage.Python,
                                              help="Make the phrase (a few seconds). Kept on the node."))
    ph.addParmTemplate(hou.ButtonParmTemplate("fit_btn", "Fit Range", script_callback=_cb("fit_range"),
                                              script_callback_language=hou.scriptLanguage.Python))
    ph.addParmTemplate(hou.StringParmTemplate("status", "Status", 1, default_value=("not generated",),
                                              help="Written by Generate / Load / Export"))
    g.append(ph)

    rb = hou.FolderParmTemplate("robot_folder", "Robot", folder_type=hou.folderType.Tabs)
    rb.addParmTemplate(hou.StringParmTemplate("robot", "Robot", 1, default_value=("",),
                                              string_type=hou.stringParmType.NodeReference,
                                              help="A wenyi::robot_arm to drive"))
    rb.addParmTemplate(hou.ButtonParmTemplate("drive_btn", "Drive robot_arm", script_callback=_cb("drive_robot"),
                                              script_callback_language=hou.scriptLanguage.Python,
                                              help="Its Pose Source becomes FK, its joints read this phrase"))
    g.append(rb)

    ex = hou.FolderParmTemplate("export_folder", "Export", folder_type=hou.folderType.Tabs)
    ex.addParmTemplate(hou.StringParmTemplate("export_csv", "CSV", 1,
                                              default_value=("$HIP/../geo/dance/`$OS`.csv",),
                                              string_type=hou.stringParmType.FileReference,
                                              help="The player's CSV; the clip JSON goes beside it"))
    ex.addParmTemplate(hou.ButtonParmTemplate("export_btn", "Export", script_callback=_cb("export"),
                                              script_callback_language=hou.scriptLanguage.Python))
    g.append(ex)

    lb = hou.FolderParmTemplate("library_folder", "Library", folder_type=hou.folderType.Tabs)
    lb.addParmTemplate(hou.StringParmTemplate("clip_file", "Clip", 1, default_value=("",),
                                              string_type=hou.stringParmType.FileReference,
                                              help="A clip JSON from the dance factory (geo/dance) or tests/clips"))
    lb.addParmTemplate(hou.ButtonParmTemplate("load_btn", "Load Clip", script_callback=_cb("load_clip"),
                                              script_callback_language=hou.scriptLanguage.Python))
    g.append(lb)
    return g


def build_hda():
    obj = hou.node("/obj")
    tmp = obj.createNode("geo", "_dance_build", run_init_scripts=False)
    sub = tmp.createNode("subnet", "dance_phrase")
    py = sub.createNode("python", "cook")
    py.parm("python").set("hou.pwd().parent().hdaModule().cook(hou.pwd())\n")
    out = sub.node("output0") or sub.createNode("output", "output0")
    out.setInput(0, py)
    for c in sub.children():
        if c.type().name() == "subinput":
            c.destroy()
    hda = sub.createDigitalAsset(name=TYPE, hda_file_name=HDA, description="Dance Phrase",
                                 min_num_inputs=0, max_num_inputs=0)
    d = hda.type().definition()
    d.addSection("PythonModule", open(ROOT + "/scripts/hda/dance_phrase_module.py", encoding="utf-8").read())
    d.setExtraFileOption("PythonModule/IsPython", True)
    d.setParmTemplateGroup(parm_group())
    d.setIcon("SOP_python")
    d.save(HDA)
    tmp.destroy()
    hou.hda.installFile(HDA, force_use_assets=True)


def build_scene():
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    hou.hipFile.clear(suppress_save_prompt=True)
    hou.hipFile.setName(SCENE)
    build_hda()
    obj = hou.node("/obj")
    rob = obj.createNode("geo", "fr20", run_init_scripts=False)
    arm = rob.createNode("wenyi::robot_arm::1.0", "robot_arm")
    arm.parm("robot_profile").set("fr20")
    arm.hdaModule().on_profile_changed(arm)
    arm.setDisplayFlag(True)
    dg = obj.createNode("geo", "dance", run_init_scripts=False)
    ph = dg.createNode(TYPE, "phrase")
    ph.parm("bars").set(3)
    for i, a in enumerate(("float", "punch", "glide")):
        ph.parm("action%d" % (i + 1)).set(ACTIONS.index(a))
    ph.parm("robot").set(arm.path())
    ph.setDisplayFlag(True)
    obj.layoutChildren()
    mod = ph.hdaModule()
    mod.generate(ph)
    mod.drive_robot(ph)
    hou.hipFile.save(SCENE)
    return ph


if __name__ == "__main__":
    ph = build_scene()
    print("built", HDA, "and", SCENE)
    print("status:", ph.evalParm("status"))
