# Describe the tool state in words.
#
# The tool has two halves that are easy to confuse: the GEOMETRY on input 3
# and the numeric Tool Offset. Wiring a mesh and expecting the arm to reach
# with it is the obvious expectation, and when only the numeric half drove the
# solve the mesh hung past the goal with nothing on screen saying why. The
# status names the source in effect, and says outright when a tip was guessed
# from extent rather than declared.
node = kwargs["node"]

SRC = {0: "group 'tcp'", 1: "point named 'tcp'", 2: "furthest point along +Y",
       3: "furthest point along +Y"}
from_geo = node.parm("tool_frame_source").eval() == 0

geo_pts = 0
found = 0
tip_len = 0.0
probe = node.node("tool_tcp_probe")
if probe is not None:
    try:
        g = probe.geometry()
        if g is not None:
            geo_pts = len(g.points())
            found = int(g.attribValue("tcp_found"))
            tip_len = float(g.attribValue("tcp_len"))
    except Exception:
        pass

off = node.parmTuple("tool_offset").eval()
rot = node.parmTuple("tool_rotate").eval()
manual_len = (off[0] ** 2 + off[1] ** 2 + off[2] ** 2) ** 0.5
rotated = any(abs(r) > 1e-9 for r in rot)

if from_geo and geo_pts and found:
    how = {1: "group 'tcp'", 2: "point named 'tcp'",
           3: "furthest point along +Y (guessed)"}[found]
    msg = "tip %.1f mm from the flange, from %s" % (tip_len * 1000.0, how)
    if found == 3:
        msg += " -- add a 'tcp' point or group to declare it"
elif from_geo and not geo_pts:
    msg = "From Geometry selected but nothing wired to input 3 -- tip is the flange"
elif manual_len < 1e-9 and not rotated:
    msg = "no tool frame -- the tip is the flange"
    if geo_pts:
        msg += "; geometry is shown but does NOT move the tip"
else:
    msg = "tip %.1f mm from the flange (manual)" % (manual_len * 1000.0)
    if geo_pts:
        msg += "; geometry mounted"

if rotated:
    msg += ", rotated %g / %g / %g" % (rot[0], rot[1], rot[2])

node.parm("tool_status").set(msg)
