"""Build scenes/FR20_atlas.hiplc -- the Stage 2 capability atlas scene.

Run with hython (no UI needed):
    hython scripts/build_atlas_scene.py            # build and save the scene
    hython scripts/build_atlas_scene.py --bake     # ... and bake it with PDG

The scene is generated, not hand-edited, so it can be rebuilt after the bake
code changes. What is in it:

    /obj/fr20_atlas
        atlas         Python SOP -> scripts/hda/atlas_sop.py cook(): one
                      volume per field (reachable, headroom, wrist, margin,
                      nsol[, capability]) for the Slab it is given. Nothing
                      displays it directly: a full bake takes minutes.
        atlas_merge   the slab files the TOP network wrote, as one volume
                      per field (scripts/hda/atlas_sop.py merge())
        to_vdb        the same fields as VDBs
        reach_shell   iso-surface of `clear` at 0.5: the space the tool can
                      reach pointing along Tool Direction WITHOUT touching the
                      room (Cell Environment); shell_cutaway
                      keeps the z < 0 half so the inside shows
        slice         a grid through the fields (z = 0: vertical, through
                      the base; move it with Center / Orientation);
                      slice_look samples the volumes, keeps the reachable part
                      (dark red where it is reachable only by touching the room) and
                      colours it by headroom: red = slow / near a
                      singularity, green = fast (Green At, m/s)
        OUT           shell + slice, displayed
    /obj/fr20_robot   an FR20 (wenyi::robot_arm) at its zero pose, for scale
    /obj/atlas_tops   Wedge (slab 0..N-1, written to atlas/slab) -> ROP
                      Geometry Output (atlas -> geo/atlas/v<voxel>mm/
                      slab_<n>.bgeo.sc). Cook the ROP node to bake.

geo/ is gitignored; the bake is rebuilt from the scene. PDG reuses slab
files that already exist (CookedCache): after changing the bake code or
parameters other than Voxel Size, right-click the bake node > Delete This
Node's Output Files (or delete geo/atlas/v<voxel>mm) and cook again.
"""

import os
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
SCENE = ROOT + "/scenes/FR20_atlas.hiplc"

ATLAS_CODE = '''import sys, importlib
p = hou.text.expandString("$HIP/../scripts/hda")
if p not in sys.path:
    sys.path.insert(0, p)
import atlas_sop
importlib.reload(atlas_sop)
atlas_sop.cook(hou.pwd())
'''
MERGE_CODE = ATLAS_CODE.replace("atlas_sop.cook(", "atlas_sop.merge(")
SLAB_DIR = '$HIP/../geo/atlas/v`round(ch("/obj/fr20_atlas/atlas/voxel")*1000)`mm'


def _spare(node, templates):
    g = node.parmTemplateGroup()
    for t in templates:
        g.append(t)
    node.setParmTemplateGroup(g)


def build(slabs=8, voxel=0.1):
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    hou.hipFile.clear(suppress_save_prompt=True)
    hou.hipFile.setName(SCENE)
    obj = hou.node("/obj")

    geo = obj.createNode("geo", "fr20_atlas", run_init_scripts=False)
    atlas = geo.createNode("python", "atlas")
    _spare(atlas, [
        hou.FloatParmTemplate("voxel", "Voxel Size (m)", 1, default_value=(voxel,), min=0.01, max=0.5),
        hou.MenuParmTemplate("mode", "Mode", ("fixed", "capability"),
                             ("Fixed Tool Direction", "Capability (26 directions, ~50x slower)")),
        hou.FloatParmTemplate("dir", "Tool Direction", 3, default_value=(0.0, -1.0, 0.0),
                              naming_scheme=hou.parmNamingScheme.XYZW),
        hou.FloatParmTemplate("tool_len", "Tool Length (m)", 1, default_value=(0.0,), min=0.0, max=0.5,
                              help="Beyond the flange face, along the tool direction"),
        hou.FloatParmTemplate("radius", "Radius (m)", 1, default_value=(2.1,)),
        hou.FloatParmTemplate("ymin", "Y Min (m)", 1, default_value=(-1.0,),
                              help="Below the base: the arm on a cart or pedestal reaches down here"),
        hou.FloatParmTemplate("ymax", "Y Max (m)", 1, default_value=(2.3,)),
        hou.IntParmTemplate("slab", "Slab", 1, default_value=(-1,),
                            help="-1: the whole grid. The TOP network sets 0..Slabs-1, one work item each"),
        hou.IntParmTemplate("slabs", "Slabs", 1, default_value=(slabs,), min=1, max=64),
        hou.StringParmTemplate("env_file", "Cell Environment", 1, default_value=("$HIP/../envs/volvox_lab.json",),
                               string_type=hou.stringParmType.FileReference,
                               help="The room: adds clear / clearance (reachable without touching it). Empty: off"),
    ])
    atlas.parm("python").set(ATLAS_CODE)

    merge = geo.createNode("python", "atlas_merge")
    _spare(merge, [hou.StringParmTemplate("pattern", "Slab Files", 1,
                                          default_value=(SLAB_DIR + "/slab_*.bgeo.sc",))])
    merge.parm("python").set(MERGE_CODE)

    vdb = geo.createNode("convertvdb", "to_vdb")
    vdb.setInput(0, merge)
    vdb.parm("conversion").set("vdb")

    shell = geo.createNode("convertvdb", "reach_shell")
    shell.setInput(0, vdb)
    # the space the tool can use: reachable AND clear of the room
    shell.parm("group").set("@name=clear")
    shell.parm("conversion").set("poly")
    shell.parm("isovalue").set(0.5)
    # cut away the +Z half, so the robot and the slice (at z = 0) show
    shell_cut = geo.createNode("clip", "shell_cutaway")
    shell_cut.setInput(0, shell)
    shell_cut.parmTuple("dir").set((0.0, 0.0, 1.0))
    shell_cut.parm("clipop").set(1)          # keep what is below the plane: z < 0
    shell_col = geo.createNode("attribwrangle", "shell_look")
    shell_col.setInput(0, shell_cut)
    shell_col.parm("class").set(2)           # points
    shell_col.parm("snippet").set("v@Cd = {0.45, 0.7, 1.0};\nf@Alpha = 0.22;")

    # a polygon grid rather than Volume Slice (whose output is a 2D volume,
    # so the unreachable part could not be cut away); move / turn it with
    # the grid's Center and Orientation
    sl = geo.createNode("grid", "slice")
    sl.parm("orient").set(0)                 # XY: vertical, through the base
    sl.parmTuple("size").set((4.4, 3.4))
    sl.parmTuple("t").set((0.0, 0.65, 0.0))
    sl.parm("rows").set(171)
    sl.parm("cols").set(221)
    sl_col = geo.createNode("attribwrangle", "slice_look")
    sl_col.setInput(0, sl)
    sl_col.setInput(1, merge)
    sl_col.parm("class").set(2)
    sl_col.parm("snippet").set(
        "// only where the tool can reach; colour = headroom (m/s):\n"
        "// red = slow, near a singularity -> green = fast\n"
        "if (volumesample(1, \"reachable\", @P) < 0.5) { removepoint(0, @ptnum); return; }\n"
        "// reachable, but only by touching the room: dark red\n"
        "if (volumesample(1, \"clear\", @P) < 0.5) { v@Cd = {0.35, 0.05, 0.05}; f@headroom = 0; return; }\n"
        "float h = volumesample(1, \"headroom\", @P);\n"
        "f@headroom = h;\n"
        "f@wrist = volumesample(1, \"wrist\", @P);\n"
        "float t = clamp(h / chf(\"full_speed\"), 0, 1);\n"
        "v@Cd = hsvtorgb(set(t * 0.33, 0.85, 1.0));\n")
    _spare(sl_col, [hou.FloatParmTemplate("full_speed", "Green At (m/s)", 1, default_value=(1.2,),
                                          help="headroom shown fully green; below it runs to red at 0")])

    out = geo.createNode("merge", "OUT")
    out.setInput(0, shell_col)
    out.setInput(1, sl_col)
    out.setDisplayFlag(True)
    out.setRenderFlag(True)
    geo.layoutChildren()

    rob = obj.createNode("geo", "fr20_robot", run_init_scripts=False)
    arm = rob.createNode("wenyi::robot_arm::1.0", "robot_arm")
    arm.parm("robot_profile").set("fr20")
    try:
        arm.hdaModule().on_profile_changed(arm)
    except Exception:
        pass
    arm.setDisplayFlag(True)

    tops = obj.createNode("topnet", "atlas_tops")
    w = tops.createNode("wedge", "slabs")
    w.parm("wedgecount").setExpression('ch("/obj/fr20_atlas/atlas/slabs")')
    w.parm("wedgeattributes").set(1)
    w.parm("name1").set("slab")
    w.parm("type1").set(2)                    # Integer
    w.parm("wedgetype1").set(0)               # Range
    w.parm("intrange1x").set(0)
    w.parm("intrange1y").setExpression('ch("/obj/fr20_atlas/atlas/slabs") - 1')
    w.parm("exportchannel1").set(1)
    w.parm("channel1").set("/obj/fr20_atlas/atlas/slab")
    rop = tops.createNode("ropgeometry", "bake")
    rop.setInput(0, w)
    rop.parm("soppath").set("/obj/fr20_atlas/atlas")
    rop.parm("sopoutput").set(SLAB_DIR + "/slab_`@slab`.bgeo.sc")
    rop.setDisplayFlag(True)
    tops.layoutChildren()
    obj.layoutChildren()
    hou.hipFile.save(SCENE)
    return rop


def bake(rop):
    import time
    t = time.time()
    rop.cookWorkItems(block=True)
    items = rop.getPDGNode().workItems
    import pdg
    states = [str(wi.state).split(".")[-1] for wi in items]
    ok = (pdg.workItemState.CookedSuccess, pdg.workItemState.CookedCache)
    bad = [wi for wi in items if wi.state not in ok]
    return time.time() - t, len(items), len(bad), sorted(set(states))


if __name__ == "__main__":
    rop = build()
    print("built", SCENE)
    if "--bake" in sys.argv:
        secs, n, bad, states = bake(rop)
        print("baked %d slabs in %.1f s, %d failed (%s)" % (n, secs, bad, ", ".join(states)))
        hou.hipFile.save(SCENE)
