"""Build scenes/FR20_show.hiplc -- the show designed in Houdini.

    hython scripts/build_show_scene.py [shows/party.json]

One node: /obj/robot_show, the digital asset wenyi::robot_show
(otls/obj_wenyi.robot_show.1.0.hdalc; rebuild it with
`hython scripts/show_rig.py --build-hda` after changing show_rig.py).
Its parameter page holds the robot profile, the environment, the show
config, zones, hubs, the operating range, the library, the authored clip,
build / dry run / preview and the display toggles; inside it are the
robot_arm asset (drawing the room), the zones, the hub ghosts, the look
rays, the built paths and the range. On load the scene turns the viewport's Remove Backfaces on (the
room's walls face in: a cutaway, as in Isaac) and registers the Edit in
Viewport handles.

The scene is generated -- rebuild it rather than hand-edit it; the rig
scene (FR20_rig.hiplc) stays the hand tool.
"""

import os
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
sys.path.insert(0, ROOT + "/scripts")
import show_rig  # noqa: E402

SCENE = ROOT + "/scenes/FR20_show.hiplc"
SESSION = '''import sys, hou
_p = hou.text.expandString("$HIP/../scripts")
if _p not in sys.path:
    sys.path.insert(0, _p)
import show_rig
show_rig.on_load()
'''


def build(config):
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    hou.hipFile.clear(suppress_save_prompt=True)
    hou.hipFile.setName(SCENE)
    hou.setSessionModuleSource(SESSION)
    node = show_rig.install(config)
    node.setDisplayFlag(True)
    hou.hipFile.save(SCENE)
    return node


if __name__ == "__main__":
    cfg = os.path.abspath(sys.argv[1]).replace("\\", "/") if len(sys.argv) > 1 else ROOT + "/shows/party.json"
    build(cfg)
    print("built", SCENE)
