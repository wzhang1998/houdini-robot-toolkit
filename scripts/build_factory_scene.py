"""Build scenes/FR20_clip_factory.hiplc -- the Stage 3 clip factory in PDG.

    hython scripts/build_factory_scene.py            # build and save
    hython scripts/build_factory_scene.py --cook     # ... and run it

/obj/factory (TOP network):
    variants   Wedge: one work item per variant (0 .. Count-1)
    make       Python Script, out of process (parallel): clip_factory.make()
               for clip_factory.wedge(Count)[variant] -> geo/clips/<id>.json
    all        Wait for All
    manifest   Python Script, in process: motion_clip.write_manifest() ->
               geo/clips/manifest.json (ok clips, rejected ones and why)

geo/ is gitignored. The variants come from a fixed seed, so a run is
repeatable; edit clip_factory.wedge() to change what is made.
"""

import os
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
SCENE = ROOT + "/scenes/FR20_clip_factory.hiplc"

MAKE = '''import sys
sys.path.insert(0, r"%(root)s/scripts")
import clip_factory as F
# out of process, neither `backticks` nor @attributes expand in the script:
# read the work item's attributes
v = F.wedge(work_item.intAttribValue("wedgetotal"))[work_item.intAttribValue("variant")]
clip = F.make(v, r"%(root)s/geo/clips")
print(v["id"], "ok" if clip["safety"]["ok"] else "rejected: " + clip["safety"]["reasons"][0])
'''

MANIFEST = '''import sys
sys.path.insert(0, r"%(root)s/scripts")
import motion_clip as M
man = M.write_manifest(r"%(root)s/geo/clips")
print("manifest: %%d ok, %%d rejected" %% (man["ok"], man["rejected"]))
'''


def build(count=50):
    hou.hipFile.clear(suppress_save_prompt=True)
    hou.hipFile.setName(SCENE)
    tops = hou.node("/obj").createNode("topnet", "factory")
    w = tops.createNode("wedge", "variants")
    w.parm("wedgecount").set(count)
    w.parm("wedgeattributes").set(1)
    w.parm("name1").set("variant")
    w.parm("type1").set(2)                    # Integer
    w.parm("wedgetype1").set(0)               # Range
    w.parm("intrange1x").set(0)
    w.parm("intrange1y").setExpression('ch("wedgecount") - 1')
    w.parm("createwedgetotal").set(1)         # @wedgetotal: out of process, only @attributes expand

    make = tops.createNode("pythonscript", "make")
    make.setInput(0, w)
    make.parm("pdg_cooktype").set(2)          # out of process: one Python per work item, in parallel
    make.parm("pythonbin").set(1)             # PDG Python (no Houdini licence per item)
    make.parm("script").set(MAKE % {"root": ROOT})

    wait = tops.createNode("waitforall", "all")
    wait.setInput(0, make)
    man = tops.createNode("pythonscript", "manifest")
    man.setInput(0, wait)
    man.parm("pdg_cooktype").set(1)           # in process
    man.parm("script").set(MANIFEST % {"root": ROOT})
    man.setDisplayFlag(True)
    tops.layoutChildren()
    hou.hipFile.save(SCENE)
    return man


if __name__ == "__main__":
    node = build()
    print("built", SCENE)
    if "--cook" in sys.argv:
        import time
        import pdg
        t = time.time()
        node.cookWorkItems(block=True)
        mk = hou.node("/obj/factory/make").getPDGNode().workItems
        states = sorted(set(str(wi.state).split(".")[-1] for wi in mk))
        print("cooked %d variants in %.0f s (%s)" % (len(mk), time.time() - t, ", ".join(states)))
