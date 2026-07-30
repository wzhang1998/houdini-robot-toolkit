import glob
import os
import re

node = kwargs["node"]
# This callback runs either on the internal controller null or on the wrapping
# wenyi::robot_arm asset, depending on which copy of the parameter was pressed.
# Resolve the network that actually holds the tool nodes instead of assuming
# node.parent(): on the asset the tool nodes are CHILDREN, not siblings.
geo = node if node.node("cache_solve") is not None else node.parent()
cache = geo.node("cache_solve")

if cache is None:
    if hou.isUIAvailable():
        hou.ui.displayMessage("cache_solve node not found.",
                              severity=hou.severityType.Error, title="No Cache Node")
else:
    # With filemethod 0 the real path is built from components and lives on
    # 'sopoutput'; 'file' only holds the explicit-mode value and points at a
    # different (non-existent) file family. Expanding the raw string is also
    # unsafe -- local variables like $OS resolve against the calling context
    # rather than the node.
    pname = "file" if cache.parm("filemethod").eval() == 1 else "sopoutput"
    evaluated = cache.parm(pname).evalAtFrame(1)

    # Wildcard the dot-delimited frame field. Diffing two evaluated frames
    # does not work: with zero padding, frames 1 and 2 differ only in the
    # final digit, so the glob would match 9 files out of 500.
    folder, name = os.path.split(evaluated)
    pattern_name, hits = re.subn(r"\.\d+\.", ".*.", name)
    if hits == 0:
        pattern_name = re.sub(r"\d+", "*", name, count=1)
    pattern = os.path.join(folder, pattern_name).replace("\\", "/")

    removed = 0
    for f in glob.glob(pattern):
        try:
            os.remove(f)
            removed += 1
        except OSError:
            pass

    # Nothing is written to the filecache's own parameters. A locked asset
    # instance forbids that -- reads are fine, writes raise PermissionError --
    # and it is the contract, not a quirk: anything a callback must change
    # belongs on a promoted parameter. This used to toggle loadfromdisk around
    # the write, which made Recache silently fail on every locked instance.
    #
    # Measured: Save to Disk writes the full frame range with loadfromdisk
    # left at 1, so the toggle was never doing anything. Pressing a button on
    # an internal node IS permitted while locked; only parm writes are not.
    cache.parm("execute").pressButton()

    f0 = int(cache.parm("f1").eval())
    f1 = int(cache.parm("f2").eval())
    written = len(glob.glob(pattern))
    msg = "Recached frames %d-%d (%d files written, %d cleared)." % (
        f0, f1, written, removed)
    node.parm("cache_status").set(msg)
    if hou.isUIAvailable():
        hou.ui.displayMessage(msg, title="IK Cache")
