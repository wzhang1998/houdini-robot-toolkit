# Item generator for the Orient Mode menu.
#
# The first option means different things depending on where the goal comes
# from: a curve has a tangent to follow, a single point does not -- it carries
# its own orientation instead. Labelling it "Curve Tangent" in Point Transform
# mode was misleading, so the label follows Goal Mode.
#
# Tokens stay 0/1/2 regardless, so saved values and the ORIENT_MODE switch are
# unaffected by the relabelling.
node = kwargs["node"]
p = node.parm("goal_mode")
mode = p.eval() if p is not None else 1

if mode == 2:
    first = "From Point  (point's own orient)"
elif mode == 0:
    first = "From Manual Rig Pose"
else:
    first = "Curve Tangent"

return ["0", first,
        "1", "Aim At Target",
        "2", "Fixed Direction"]
