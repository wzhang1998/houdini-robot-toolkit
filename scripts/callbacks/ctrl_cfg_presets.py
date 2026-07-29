"""Apply an IK configuration preset by writing joint-limit ranges.

The three flags below are the standard industrial-controller configuration
set (ABB confdata, KUKA Status/Turn, Fanuc config). Each is gated by one
joint, and FBIK enforces joint limits as hard constraints, so restricting a
range makes the unwanted branch unreachable rather than merely unlikely.

Ranges come from profiles/<id>.json in the ROBOT frame and are converted to
the configurejoints frame by robot_profile.preset_configurejoints(). Do not
hand-write a converted number here.

That rule is why this file was rewritten. It previously carried its own
table with the robot-frame elbow range (-242.0, 3.5) written straight into
the configurejoints parm. J3 has both a +90 offset and a sign inversion, so
the range that actually reached FBIK was robot -93.8 .. +152.2: roughly 149
degrees of motion the arm cannot perform, while forbidding 148 degrees it
can. The clip in the scene was solving with J3 pinned within 0.25 deg of
that phantom wall. Removing it dropped mean tracking residual from 7.24 mm
to 4.65 mm and let the elbow reach -111 deg instead of stopping at -93.5.

J4 is absent because FBIK ignores its limits on this rig -- reproduced by
pinning J4 to -20 and +20 and getting -84.51 both times, a slope of exactly
zero. Rotation weights DO bind on J4, so constrain it through J4 Roll
Freedom instead. Why limits are ignored there but honoured on J1, which has
the same +-360 range, is still unexplained.

Written flat: parameter callbacks execute as a single script and nested defs
do not reliably see module-level names.
"""
import sys

import hou

node = kwargs["node"]

_root = hou.expandString("$HIP/..")
if _root + "/scripts" not in sys.path:
    sys.path.insert(0, _root + "/scripts")
import robot_profile

_pp = node.parm("robot_profile")
_pid = _pp.eval() if _pp is not None else "uf850"
prof = robot_profile.load(_pid, _root + "/profiles")

OPTIONS = {"shoulder": ("any", "front", "back"),
           "elbow": ("any", "up", "down"),
           "wrist": ("any", "positive", "negative")}

if kwargs["parm"].name() == "cfg_reset":
    node.parm("cfg_shoulder").set(0)
    node.parm("cfg_elbow").set(0)
    node.parm("cfg_wrist").set(0)
    node.parm("cfg_j4_weight").set(1.0)

chosen = {}
for group in ("shoulder", "elbow", "wrist"):
    idx = int(node.parm("cfg_" + group).eval())
    option = OPTIONS[group][idx]
    chosen[group] = option
    jn = prof["presets"][group]["joint"]
    lo, hi = robot_profile.preset_configurejoints(prof, group, option)
    node.parm("cfg_j%d_min" % jn).set(lo)
    node.parm("cfg_j%d_max" % jn).set(hi)

j4w = node.parm("cfg_j4_weight").eval()
extra = "" if abs(j4w - 1.0) < 1e-6 else "  |  J4 roll freedom %.2f" % j4w

if all(v == OPTIONS[g][0] for g, v in chosen.items()):
    node.parm("cfg_status").set(
        "%s: unrestricted (full joint ranges)" % prof["label"] + extra)
else:
    node.parm("cfg_status").set(
        "%s: shoulder %s / elbow %s / wrist %s"
        % (prof["label"], chosen["shoulder"], chosen["elbow"],
           chosen["wrist"]) + extra)
