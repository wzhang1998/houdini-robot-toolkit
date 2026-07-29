"""Apply an IK configuration preset by writing joint-limit ranges.

The three flags below are the standard industrial-controller configuration
set (ABB confdata, KUKA Status/Turn, Fanuc config). Each is gated by one
joint, and FBIK enforces joint limits as hard constraints, so restricting
a range makes the unwanted branch unreachable rather than merely unlikely.

All values are in the CONFIGUREJOINTS frame, which is NOT the frame the
CSV export and path analysis report. Measured on this rig:
    J1, J2, J5, J6 : offset 0
    J3             : extracted = configurejoints + 90
So a J3 limit of 3.5 here shows up as 93.5 in the analysis.

Known limitation on this rig: J4's limits are NOT enforced by FBIK -- its
declared rotation axis (Y) does not match the axis it actually rotates
about (X), so clamping Y constrains nothing. J4 is therefore absent here,
and wrist FLIPS cannot be prevented by configuration. Use the asset's
Resolve Wrist Flips instead.

Written flat: parameter callbacks execute as a single script and nested
defs do not reliably see module-level names.
"""
node = kwargs["node"]

# menu index -> (min, max) in configurejoints frame
SHOULDER = {0: (-360.0, 360.0), 1: (-90.0, 90.0), 2: (90.0, 270.0)}
ELBOW = {0: (-242.0, 3.5), 1: (-90.0, 3.5), 2: (-242.0, -90.0)}
WRIST = {0: (-124.0, 124.0), 1: (0.0, 124.0), 2: (-124.0, 0.0)}

SH_LBL = {0: "any", 1: "front", 2: "back"}
EL_LBL = {0: "any", 1: "up", 2: "down"}
WR_LBL = {0: "any", 1: "positive", 2: "negative"}

if kwargs["parm"].name() == "cfg_reset":
    node.parm("cfg_shoulder").set(0)
    node.parm("cfg_elbow").set(0)
    node.parm("cfg_wrist").set(0)
    node.parm("cfg_j4_weight").set(1.0)

s = int(node.parm("cfg_shoulder").eval())
e = int(node.parm("cfg_elbow").eval())
w = int(node.parm("cfg_wrist").eval())

node.parm("cfg_j1_min").set(SHOULDER[s][0])
node.parm("cfg_j1_max").set(SHOULDER[s][1])
node.parm("cfg_j3_min").set(ELBOW[e][0])
node.parm("cfg_j3_max").set(ELBOW[e][1])
node.parm("cfg_j5_min").set(WRIST[w][0])
node.parm("cfg_j5_max").set(WRIST[w][1])

j4w = node.parm("cfg_j4_weight").eval()
extra = "" if abs(j4w - 1.0) < 1e-6 else "  |  J4 roll freedom %.2f" % j4w

if s == 0 and e == 0 and w == 0:
    node.parm("cfg_status").set("unrestricted (full joint ranges)" + extra)
else:
    node.parm("cfg_status").set(
        "shoulder %s / elbow %s / wrist %s"
        % (SH_LBL[s], EL_LBL[e], WR_LBL[w]) + extra)
