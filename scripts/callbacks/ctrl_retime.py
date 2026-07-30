"""Time reparameterization of `progress` from joint velocity limits.

The TCP path in space is untouched -- only WHEN the tool is where changes.

Method: for each interval of u, the fastest joint sweeps `rate * du` degrees.
At the velocity limit that interval must take `rate * du / w_max` seconds.
Accumulating those gives total duration and an exact t -> u mapping, which
is then sampled per frame.

Accumulating time (rather than stepping u by an allowed increment) matters:
stepping u uses the rate at the current sample, so a large step in an easy
stretch can jump straight over a narrow spike and silently under-report the
duration needed.

Written flat (no function defs) because parameter callbacks execute as a
single script and nested defs do not reliably see module-level names.
"""
import math

def _max_velocity(node):
    """The one joint-velocity limit, shared with export and pre-flight.

    Retime used to carry its own copy. Two editable numbers for one physical
    limit is the drift that put three different joint-limit tables in this
    project, one of them wrong by 148 degrees -- so retime reads the same
    parameter the exporter validates against.
    """
    p = node.parm("max_velocity")
    return float(p.eval()) if p is not None else 180.0


node = kwargs["node"]
# This callback runs either on the internal controller null or on the wrapping
# wenyi::robot_arm asset, depending on which copy of the parameter was pressed.
# Resolve the network that actually holds the tool nodes instead of assuming
# node.parent(): on the asset the tool nodes are CHILDREN, not siblings.
which = kwargs["parm"].name()
geo = node if node.node("cache_solve") is not None else node.parent()
prog = node.parm("progress")

if which == "reset_progress_btn":
    prog.deleteAllKeyframes()
    prog.setExpression("fit($FF, $RFSTART, $RFEND, 0, 1)")
    node.parm("retime_status").set("progress reset to linear over the frame range")
    if hou.isUIAvailable():
        hou.ui.displayMessage("progress reset to a linear sweep of the frame range.",
                              title="Retime")
else:
    src = geo.node("measure_residual")
    hda = geo.node("robot_csv_io")
    cfg = geo.node("configurejoints1")

    if src is None or hda is None or cfg is None:
        if hou.isUIAvailable():
            hou.ui.displayMessage(
                "Need measure_residual, robot_csv_io and configurejoints1 in this network.",
                severity=hou.severityType.Error, title="Retime")
    else:
        _mod = {}
        exec(hda.type().definition().sections()["PythonModule"].contents(), _mod)
        extract = _mod["extract_angles"]
        axis_of = _mod["axis_map_from_geo"](cfg.geometry())

        NS = max(8, int(node.parm("retime_samples").eval()))
        wmax = (_max_velocity(node)
                * float(node.parm("retime_safety").eval()))
        fps = hou.fps()
        du = 1.0 / NS
        # floor on how fast u may advance, so flat stretches do not take zero time
        dt_floor = du / (float(node.parm("retime_max_step").eval()) * du * fps)

        prog.deleteAllKeyframes()
        angles = []
        for i in range(NS + 1):
            prog.set(i * du)
            angles.append(extract(src.geometry(), axis_of))

        cum = [0.0]
        rates = []
        for i in range(NS):
            rate = 0.0
            for j in range(6):
                # Wrap into [-180, 180]. Extracted angles live in (-180, 180],
                # so a joint passing through the boundary reads as a 360 deg
                # jump. Left unwrapped, the retime spends its whole budget
                # slowing down for a measurement artifact.
                d = angles[i + 1][j] - angles[i][j]
                d -= 360.0 * round(d / 360.0)
                rate = max(rate, abs(d) / du)
            rates.append(rate)
            cum.append(cum[-1] + max(rate * du / wmax, dt_floor))

        total = cum[-1]
        f0 = int(hou.playbar.frameRange()[0])
        f1 = int(hou.playbar.frameRange()[1])

        if int(node.parm("retime_mode").eval()) == 0:
            # Preserve Duration: keep the existing frame count and only
            # REDISTRIBUTE time, so hard stretches get more frames and easy
            # ones fewer. Time-optimal mode instead runs everything up to the
            # limit, which shortens the clip -- useful for cycle time, but not
            # what you want when the motion is already within limits.
            nframes = max(1, f1 - f0)
            want = nframes / fps
            if total > 1e-9:
                scale = want / total
                cum = [c * scale for c in cum]
                total = cum[-1]
        else:
            nframes = max(1, int(math.ceil(total * fps)))

        prog.deleteAllKeyframes()
        k_index = 0
        for fi in range(nframes + 1):
            tt = fi / fps
            if tt >= total:
                u = 1.0
            else:
                while k_index < NS - 1 and cum[k_index + 1] <= tt:
                    k_index += 1
                span = cum[k_index + 1] - cum[k_index]
                frac = 0.0 if span <= 0 else (tt - cum[k_index]) / span
                u = min(1.0, (k_index + frac) * du)
            key = hou.Keyframe()
            key.setFrame(f0 + fi)
            key.setValue(u)
            key.setExpression("linear()", hou.exprLanguage.Hscript)
            prog.setKeyframe(key)

        fend = f0 + nframes
        if node.parm("retime_set_range").eval():
            hou.playbar.setFrameRange(f0, fend)
            hou.playbar.setPlaybackRange(f0, fend)

        node.parm("retime_status").set("Retimed to %d frames (%d-%d)"
                                       % (nframes + 1, f0, fend))
        if hou.isUIAvailable():
            hou.ui.displayMessage(
                "Retimed to %d frames (%d-%d) at %g%% of %g deg/s.\n"
                "Peak joint rate %.0f deg per unit u.\n\n"
                "The path in space is unchanged -- only the timing.\n"
                "Recache the IK solve to refresh the analysis."
                % (nframes + 1, f0, fend,
                   node.parm("retime_safety").eval() * 100.0,
                   _max_velocity(node), max(rates)),
                title="Retime")
