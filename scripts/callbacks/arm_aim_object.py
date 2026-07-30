# Resolve Aim Object to an absolute path.
#
# The path is typed on the asset, where "../transform3" means a sibling of the
# asset. The expressions that consume it live on TCP_PATH_CTRL *inside* the
# asset, where the same string means a sibling of that null -- a different and
# usually non-existent node. Resolving here, on the node the user typed into,
# removes the ambiguity.
#
# A callback rather than an expression on aim_object_abs: an expression set on
# one node never reaches other instances, and a DialogScript expression-default
# stores the raw text for string parameters instead of evaluating it.
node = kwargs["node"]
raw = node.parm("aim_object").eval().strip()

resolved = ""
if raw:
    tgt = node.node(raw)          # relative to the asset, as typed
    if tgt is None:
        tgt = hou.node(raw)       # already absolute
    if tgt is not None:
        resolved = tgt.path()

node.parm("aim_object_abs").set(resolved)

if raw and not resolved:
    node.parm("aim_status").set("not found: %s" % raw)
elif resolved:
    t = hou.node(resolved)
    kind = t.type().category().name()
    if kind == "Sop":
        try:
            n = len(t.geometry().points())
        except Exception:
            n = 0
        node.parm("aim_status").set(
            "%s (SOP, %d point%s) -- aiming at point 0"
            % (resolved, n, "" if n == 1 else "s")
            if n else
            "%s is SOP geometry with no points" % resolved)
    else:
        node.parm("aim_status").set("%s (%s) -- aiming at its origin"
                                    % (resolved, kind))
else:
    node.parm("aim_status").set("")
