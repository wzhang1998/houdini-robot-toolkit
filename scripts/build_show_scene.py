"""Build scenes/FR20_show.hiplc -- the show designed in Houdini.

    hython scripts/build_show_scene.py [shows/party.json]

The cell scene (scripts/build_cell_scene.py: the FR20 in the measured room,
its joints from CELL_CTRL's clip) plus the show objects of show_rig.py:

    /obj/SHOW          the show config (shows/*.json) as parameters: start
                       hub, operating range, library, add hub / zone, the
                       authored clip, Check / Build / Dry Run, play a built
                       segment or hold a hub's pose on the arm
    /obj/zone_<name>   the zones, as boxes you move, turn and scale
    /obj/hub_<name>    the hubs (tool tip), look_<name> what each looks at
    /obj/show_viz      hubs solved live as ghost arms, look rays, the range,
                       the built tool paths

The room is drawn once, like the Isaac scene (cell_sop / room_geom): solid
floor and objects, walls facing in (Remove Backfaces makes the near ones
vanish), zones as thin rings. The scene is generated -- rebuild it rather
than hand-edit it; the rig scene (FR20_rig.hiplc) stays the hand tool.
"""

import os
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
sys.path.insert(0, ROOT + "/scripts")
import build_cell_scene  # noqa: E402
import show_rig  # noqa: E402

SCENE = ROOT + "/scenes/FR20_show.hiplc"
SESSION_EXTRA = '''
import show_rig
show_rig.clean_view_on_load()
'''


def build(config):
    build_cell_scene.build()
    hou.hipFile.setName(SCENE)
    hou.setSessionModuleSource(hou.sessionModuleSource() + SESSION_EXTRA)
    show = show_rig.install(config)
    hou.node("/obj").layoutChildren()
    show_rig._layout()
    hou.hipFile.save(SCENE)
    return show


if __name__ == "__main__":
    cfg = os.path.abspath(sys.argv[1]).replace("\\", "/") if len(sys.argv) > 1 else ROOT + "/shows/party.json"
    build(cfg)
    print("built", SCENE)
