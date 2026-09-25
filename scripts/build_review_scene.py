"""Build scenes/FR20_review.hiplc -- the clip review in PDG.

    hython scripts/build_review_scene.py                   # build and save the scene
    hython scripts/build_review_scene.py --cook            # ... and cook it (every clip)
    hython scripts/build_review_scene.py --cook --ids d01_punch-punch v04_line
    hython scripts/build_review_scene.py --cook --sets stage --ok-only   # the quick-test set (stage_set.py)

The scene is scenes/FR20_cell.hiplc (the measured room, the FR20 driven by
CELL_CTRL's Clip) turned into the review picture by
render_clip_review.setup_scene(), with CELL_CTRL's Clip set to @clip -- so a
work item selected in the TOP network shows its clip in the viewport. The
network, the usual PDG way (one work item per clip, OpenGL frames, then
ffmpeg / ImageMagick):

    /obj/review
        clips        Python Processor: a work item per clip of geo/dance and
                     geo/clips (Ids / Sets / OK Only on the node), with @id
                     @set @clip @dir @ok @nframes
        frames       ROP OpenGL Render: the clip's frames 1-@nframes, one work
                     item per clip (single task), the viewport's look; a clip
                     rejected before any motion renders one still (arm at
                     HOME, the path it was asked for, unreachable part red)
        tiles        Python Script: ffmpeg burns the text in (red frame when
                     rejected), writes
                     tile.mp4 and a poster frame posters/<id>.png
        all          Wait for All
        videos       Python Script: page videos per set, and every clip in
                     overview pages of 5 x 5 (ffmpeg xstack)
        sheet        ImageMagick montage of the posters (each carries its id,
                     burnt in): contact_sheet.png

Output: geo/review/ (not in git). PDG caches work items; to redo a clip,
delete its geo/review/<set>/<id>/ or dirty the node.
"""

import os
import sys

import hou

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
sys.path.insert(0, ROOT + "/scripts")
import render_clip_review as R  # noqa: E402

SCENE = ROOT + "/scenes/FR20_review.hiplc"
MAGICK = "C:/Program Files/ImageMagick-7.1.2-Q16-HDRI/magick.exe"

GENERATE = '''import sys
sys.path.insert(0, %(scripts)r)
import importlib, render_clip_review as R
importlib.reload(R)
ids = self["ids"].evaluateString().split() or None
sets = self["sets"].evaluateString().split() or ["dance", "clips"]
for i, it in enumerate(R.clip_items(sets, ids, bool(self["ok_only"].evaluateInt()))):
    item = item_holder.addWorkItem(index=i)
    for k in ("id", "set", "clip", "dir"):
        item.setStringAttrib(k, it[k])
    item.setIntAttrib("ok", it["ok"])
    item.setIntAttrib("nframes", it["nframes"])
'''

TILE = '''import os, shutil, sys
sys.path.insert(0, %(scripts)r)
import render_clip_review as R
# out of process: read the work item's attributes (no @ / backtick expansion here)
it = {k: work_item.stringAttribValue(k) for k in ("id", "set", "clip", "dir")}
tile, poster = R.encode_tile(it)
os.makedirs(R.OUT + "/posters", exist_ok=True)
labelled = R.OUT + "/posters/" + it["id"] + ".png"     # the contact sheet labels by file name
shutil.copyfile(poster, labelled)
work_item.addOutputFile(tile, "file/video")
work_item.addOutputFile(labelled, "file/image")
print(it["id"], tile)
'''

VIDEOS = '''import sys
sys.path.insert(0, %(scripts)r)
import importlib, render_clip_review as R
importlib.reload(R)
# the tiles come in as this item's input files: .../geo/review/<set>/<id>/tile.mp4
items = []
for f in work_item.inputFiles:
    if f.path.endswith("/tile.mp4"):
        d = f.path[:-len("/tile.mp4")]
        items.append({"dir": d, "id": d.split("/")[-1], "set": d.split("/")[-2]})
order = {k: i for i, k in enumerate(R.SETS)}
items.sort(key=lambda i: (order.get(i["set"], 9), i["id"]))
for p in R.build_videos(items):
    work_item.addOutputFile(p, "file/video")
'''


def _spare(node, templates):
    g = node.parmTemplateGroup()
    for t in templates:
        g.append(t)
    node.setParmTemplateGroup(g)


def build():
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    hou.hipFile.load(ROOT + "/scenes/FR20_cell.hiplc", suppress_save_prompt=True, ignore_load_warnings=True)
    for f in ("sop_wenyi.robot_anim_csv_io.1.0.hdalc", "sop_wenyi.robot_arm.1.0.hdalc"):
        hou.hda.installFile(ROOT + "/otls/" + f, force_use_assets=True)
    hou.hipFile.setName(SCENE)
    cam = R.setup_scene()
    hou.node("/obj/CELL_CTRL").parm("clip").set("`@clip`")      # backticks: expanded per work item in the render job
    sub = {"scripts": ROOT + "/scripts"}

    tops = hou.node("/obj").createNode("topnet", "review")
    sched = [c for c in tops.children() if c.type().name() == "localscheduler"][0]
    sched.parm("maxprocsmenu").set("-1")                  # custom
    sched.parm("maxprocs").set(3)                         # OpenGL renders side by side on one GPU

    gen = tops.createNode("pythonprocessor", "clips")
    _spare(gen, [hou.StringParmTemplate("ids", "Ids", 1, help="Clip ids, space separated; empty: every clip"),
                 hou.StringParmTemplate("sets", "Sets", 1, default_value=("dance clips",)),
                 hou.ToggleParmTemplate("ok_only", "OK Only", default_value=False,
                                        help="Leave out the rejected clips (else rendered, framed red)")])
    gen.parm("generate").set(GENERATE % sub)

    ren = tops.createNode("ropopengl", "frames")
    ren.setInput(0, gen)
    ren.parm("framegeneration").set("1")                  # frame range
    ren.parm("f1").set(1)
    ren.parm("f2").setExpression("max(@nframes, 1)")      # no motion (rejected early): one still
    ren.parm("f3").set(1)
    ren.parm("singletask").set(1)                         # one work item per clip, all its frames (not an item per frame)
    ren.parm("camera").set(cam.path())
    ren.parm("tres").set(1)
    ren.parm("res1").set(R.TILE[0])
    ren.parm("res2").set(R.TILE[1])
    ren.parm("picture").set("$HIP/../geo/review/`@set`/`@id`/frames/f_$F4.png")   # backticks: expanded per work item
    for n, v in (("aamode", "aa4"), ("hqlighting", 1), ("shadows", 0), ("ambocclusion", 0), ("usehdr", 1)):
        ren.parm(n).set(v)

    tiles = tops.createNode("pythonscript", "tiles")
    tiles.setInput(0, ren)
    tiles.parm("pdg_cooktype").set(2)                     # out of process
    tiles.parm("pythonbin").set(1)                        # PDG Python: ffmpeg, no Houdini licence per item
    tiles.parm("script").set(TILE % sub)

    wait = tops.createNode("waitforall", "all")
    wait.setInput(0, tiles)

    vids = tops.createNode("pythonscript", "videos")
    vids.setInput(0, wait)
    vids.parm("pdg_cooktype").set(1)                      # in process
    vids.parm("script").set(VIDEOS % sub)

    sheet = tops.createNode("imagemagick", "sheet")
    sheet.setInput(0, wait)
    sheet.parm("operation").set("0")                      # montage
    sheet.parm("inputfiletag").set("file/image")
    sheet.parm("overlaymode").set("0")                    # no labels: each poster carries its clip's id already
    sheet.parm("montagecolumnsswitch").set(1)
    sheet.parm("montagecolumns").set(10)
    sheet.parm("background").set("#202224")
    sheet.parm("imagemagickbinary").set("3")              # custom path
    sheet.parm("customimagemagickbinary").set(MAGICK)
    sheet.parm("outputfilepath").set("$HIP/../geo/review/contact_sheet.png")

    out = tops.createNode("merge", "done")
    out.setInput(0, vids)
    out.setInput(1, sheet)
    out.setDisplayFlag(True)
    tops.layoutChildren()
    hou.hipFile.save(SCENE)
    return tops, gen, out


def _values(argv, flag):
    """The arguments after flag, up to the next --flag."""
    out = []
    for x in argv[argv.index(flag) + 1:]:
        if x.startswith("--"):
            break
        out.append(x)
    return out


def cook(out):
    import time
    t = time.time()
    out.cookWorkItems(block=True)
    report = []
    for n in out.parent().children():
        pn = n.getPDGNode() if hasattr(n, "getPDGNode") else None
        if pn is None or not hasattr(pn, "workItems"):
            continue
        states = {}
        for wi in pn.workItems:
            s = str(wi.state).split(".")[-1]
            states[s] = states.get(s, 0) + 1
        report.append("%s %s" % (n.name(), states))
    print("cooked in %.0f s: %s" % (time.time() - t, "; ".join(report)))


if __name__ == "__main__":
    tops, gen, out = build()
    if "--sets" in sys.argv:
        sets = _values(sys.argv, "--sets")
        gen.parm("sets").set(" ".join(sets))
        tops.node("sheet").parm("outputfilepath").set("$HIP/../geo/review/contact_sheet_%s.png" % "_".join(sets))
        hou.hipFile.save(SCENE)
    if "--ok-only" in sys.argv:
        gen.parm("ok_only").set(1)
        hou.hipFile.save(SCENE)
    if "--ids" in sys.argv:
        gen.parm("ids").set(" ".join(_values(sys.argv, "--ids")))
        hou.hipFile.save(SCENE)
    print("built", SCENE)
    if "--cook" in sys.argv:
        cook(out)
