import glob
import os
import re

node = kwargs["node"]
geo = node.parent()
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

    cache.parm("loadfromdisk").set(0)
    cache.parm("execute").pressButton()
    cache.parm("loadfromdisk").set(1)

    f0 = int(cache.parm("f1").eval())
    f1 = int(cache.parm("f2").eval())
    written = len(glob.glob(pattern))
    msg = "Recached frames %d-%d (%d files written, %d cleared)." % (
        f0, f1, written, removed)
    node.parm("cache_status").set(msg)
    if hou.isUIAvailable():
        hou.ui.displayMessage(msg, title="IK Cache")
