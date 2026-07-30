# Describe the tool state in words.
#
# The tool has two halves that are easy to confuse: the GEOMETRY on input 3 is
# display only, and the numeric Tool Offset is what the solver actually uses.
# Wiring a mesh and expecting the TCP to move is the obvious mistake, so the
# status says plainly which of the two is in effect.
node = kwargs["node"]
off = node.parmTuple("tool_offset").eval()
rot = node.parmTuple("tool_rotate").eval()
length = (off[0] ** 2 + off[1] ** 2 + off[2] ** 2) ** 0.5
rotated = any(abs(r) > 1e-9 for r in rot)

geo_pts = 0
ti = node.node("TOOL_IN")
if ti is not None:
    try:
        g = ti.geometry()
        geo_pts = len(g.points()) if g is not None else 0
    except Exception:
        geo_pts = 0

if length < 1e-9 and not rotated:
    msg = "no tool frame -- the tip is the flange"
    if geo_pts:
        msg += "; geometry on input 3 is shown but does NOT move the tip"
else:
    msg = "tip %.1f mm from the flange" % (length * 1000.0)
    if rotated:
        msg += ", rotated %g / %g / %g" % (rot[0], rot[1], rot[2])
    msg += "; geometry %s" % ("mounted" if geo_pts else "not wired")

node.parm("tool_status").set(msg)
