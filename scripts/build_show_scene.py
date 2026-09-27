"""Build scenes/FR20_show.hiplc -- the show designed in Houdini.

    hython scripts/build_show_scene.py [shows/party.json]

The cell scene (scripts/build_cell_scene.py: the FR20 in the measured room,
its joints from CELL_CTRL's clip) plus:

    /obj/SHOW_CTRL   the show config (shows/*.json) as parameters:
                       Hubs       name, joint angles, clips per hub; Start Hub
                       Library    clip length, bars, BPM, seed; hub stay
                       Stage      the work zone the clips stay in (override the env's)
                       Paper      the canvas (placeholder), idle margin
                       Buttons    Load / Write config, Build Show, Dry Run,
                                  preview a segment or a hub on the arm
    /obj/show_viz    the show drawn: stage (green), paper (yellow), hubs (their
                     colour; red when not clear), every clip's TCP path in its
                     hub's colour, moves white, scan yellow

Edit a hub or the stage and the viewport shows it at once (hub clear or
not); Build makes and checks the clips (about a minute) and redraws. The
scene is generated -- rebuild it rather than hand-edit it.
"""

import os
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
sys.path.insert(0, ROOT + "/scripts")
import build_cell_scene  # noqa: E402

SCENE = ROOT + "/scenes/FR20_show.hiplc"
SESSION_EXTRA = '''
import show_sop
'''


def _cb(fn):
    return {"script_callback": "hou.session.show_sop.%s()" % fn,
            "script_callback_language": hou.scriptLanguage.Python}


def build(config):
    build_cell_scene.build()
    hou.hipFile.setName(SCENE)
    hou.setSessionModuleSource(hou.sessionModuleSource() + SESSION_EXTRA)
    obj = hou.node("/obj")
    ctrl = obj.createNode("null", "SHOW_CTRL")
    g = ctrl.parmTemplateGroup()
    T = hou
    g.append(T.StringParmTemplate("config", "Show Config", 1, string_type=T.stringParmType.FileReference,
                                  default_value=(config,)))
    g.append(T.ButtonParmTemplate("load", "Load Config", **_cb("load_config")))
    g.append(T.ButtonParmTemplate("write", "Write Config", **_cb("write_config")))

    hubs = T.FolderParmTemplate("hubs_f", "Hubs", folder_type=T.folderType.Simple)
    ml = T.FolderParmTemplate("hubs", "Hubs", folder_type=T.folderType.MultiparmBlock)
    ml.addParmTemplate(T.StringParmTemplate("hub_name#", "Name", 1))
    ml.addParmTemplate(T.FloatParmTemplate("hub_q#", "Joints (deg)", 6, min=-270.0, max=270.0,
                                           help="J1..J6 of the hub pose; J1 is the direction it faces"))
    ml.addParmTemplate(T.IntParmTemplate("hub_clips#", "Clips", 1, default_value=(8,), min=0, max=40))
    hubs.addParmTemplate(ml)
    hubs.addParmTemplate(T.StringParmTemplate("start_hub", "Start Hub", 1, default_value=("center",)))
    hubs.addParmTemplate(T.IntParmTemplate("hub_pick", "Preview Hub #", 1, default_value=(1,), min=1, max=8))
    hubs.addParmTemplate(T.ButtonParmTemplate("preview_hub_b", "Preview Hub on the Arm", **_cb("preview_hub")))
    g.append(hubs)

    lib = T.FolderParmTemplate("lib_f", "Library", folder_type=T.folderType.Simple)
    lib.addParmTemplate(T.FloatParmTemplate("duration", "Clip Length (s)", 2, default_value=(4.0, 10.0), min=1.0, max=30.0))
    lib.addParmTemplate(T.IntParmTemplate("bars", "Bars", 2, default_value=(1, 2), min=1, max=4))
    lib.addParmTemplate(T.IntParmTemplate("bpm", "BPM", 2, default_value=(80, 125), min=40, max=180))
    lib.addParmTemplate(T.IntParmTemplate("seed", "Seed", 1, default_value=(11,)))
    lib.addParmTemplate(T.IntParmTemplate("hub_stay", "Clips per Hub Visit", 2, default_value=(2, 4), min=1, max=20))
    g.append(lib)

    stage = T.FolderParmTemplate("stage_f", "Stage and Paper", folder_type=T.folderType.Simple)
    stage.addParmTemplate(T.ToggleParmTemplate("stage_override", "Override the Env's Stage", default_value=False,
                                               help="Off: the stage (work zone) of envs/*.json. On: this box"))
    stage.addParmTemplate(T.FloatParmTemplate("stage_center", "Stage Centre (robot frame, m)", 3))
    stage.addParmTemplate(T.FloatParmTemplate("stage_size", "Stage Size (m)", 3, default_value=(2.0, 2.0, 2.0)))
    stage.addParmTemplate(T.FloatParmTemplate("stage_yaw", "Stage Yaw (deg)", 1))
    stage.addParmTemplate(T.FloatParmTemplate("canvas_center", "Paper Centre (m)", 3))
    stage.addParmTemplate(T.FloatParmTemplate("canvas_normal", "Paper Normal", 3, default_value=(0.0, 1.0, 0.0)))
    stage.addParmTemplate(T.FloatParmTemplate("canvas_size", "Paper Size (m)", 2, default_value=(1.0, 0.6)))
    stage.addParmTemplate(T.FloatParmTemplate("idle_canvas_m", "Idle Clips Keep Off the Paper (m)", 1,
                                              default_value=(0.15,), min=0.0, max=0.5))
    g.append(stage)

    run = T.FolderParmTemplate("run_f", "Build and Preview", folder_type=T.folderType.Simple)
    run.addParmTemplate(T.ButtonParmTemplate("build_b", "Build Show", **_cb("build_show")))
    run.addParmTemplate(T.FloatParmTemplate("dry_minutes", "Dry Run Minutes", 1, default_value=(10.0,), min=1.0, max=120.0))
    run.addParmTemplate(T.ButtonParmTemplate("dry_b", "Dry Run", **_cb("dry_run")))
    run.addParmTemplate(T.StringParmTemplate("segment", "Segment", 1, menu_type=T.menuType.Normal,
                                             item_generator_script="hou.session.show_sop.segment_menu()",
                                             item_generator_script_language=T.scriptLanguage.Python,
                                             **_cb("preview_segment")))
    run.addParmTemplate(T.StringParmTemplate("report", "Report", 1, tags={"editor": "1", "editorlines": "12-30"}))
    g.append(run)
    ctrl.setParmTemplateGroup(g)
    hou.session.show_sop.load_config(ctrl)

    viz = obj.createNode("geo", "show_viz", run_init_scripts=False)
    sop = viz.createNode("python", "show")
    sop.parm("python").set("import hou\nhou.session.show_sop.show_geo(hou.pwd())\n")
    wire = viz.createNode("polywire", "paths")                # a line is a hair: thin tubes, coloured
    wire.setInput(0, sop)
    wire.parm("radius").set(0.005)
    wire.parm("div").set(5)
    hubs = viz.createNode("blast", "hub_points")
    hubs.setInput(0, sop)
    hubs.parm("group").set("@pscale>0.05")
    hubs.parm("grouptype").set(3)                             # points
    hubs.parm("negate").set(1)
    ball = viz.createNode("sphere", "ball")
    ball.parm("type").set(2)
    ball.parmTuple("rad").set((0.05, 0.05, 0.05))
    spheres = viz.createNode("copytopoints::2.0", "hub_spheres")
    spheres.setInput(0, ball)
    spheres.setInput(1, hubs)
    out = viz.createNode("merge", "OUT")
    out.setInput(0, wire)
    out.setInput(1, spheres)
    out.setDisplayFlag(True)
    obj.layoutChildren()
    hou.hipFile.save(SCENE)
    return ctrl


if __name__ == "__main__":
    cfg = os.path.abspath(sys.argv[1]).replace("\\", "/") if len(sys.argv) > 1 else ROOT + "/shows/party.json"
    build(cfg)
    print("built", SCENE)
