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
import os
import sys

def _max_velocity(node):
    """The one joint-velocity limit, shared with export and pre-flight.

    Retime used to carry its own copy. Two editable numbers for one physical
    limit is the drift that put three different joint-limit tables in this
    project, one of them wrong by 148 degrees -- so retime reads the same
    parameter the exporter validates against.
    """
    p = node.parm("max_velocity")
    return float(p.eval()) if p is not None else 180.0


def _velocity_limits(node, geo):
    """Per-joint limits, the same ones export and pre-flight check against:
    the asset module's velocity_limits() (profile per joint, capped by Max
    Joint Velocity). One shared number for every joint would let retime
    budget a base joint as if it could turn as fast as the wrist."""
    try:
        return list(geo.hdaModule().velocity_limits(geo))
    except Exception:
        return [_max_velocity(node)] * 6


def _acceleration_limits(geo):
    """The asset module's acceleration_limits(), or None (velocity-only)."""
    try:
        return geo.hdaModule().acceleration_limits(geo)
    except Exception:
        return None


def _value(node, geo, name, default):
    """A parameter from whichever copy was pressed, else the asset's."""
    for n in (node, geo):
        p = n.parm(name) if n is not None else None
        if p is not None:
            return float(p.eval())
    return default


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
        safety = float(node.parm("retime_safety").eval())
        wlim = [v * safety for v in _velocity_limits(node, geo)]
        fps = hou.fps()
        du = 1.0 / NS
        # floor on how fast u may advance, so flat stretches do not take zero time
        dt_floor = du / (float(node.parm("retime_max_step").eval()) * du * fps)

        prog.deleteAllKeyframes()
        # The closed-form IK picks each frame's branch nearest the previous
        # FRAME. Sampling u at one frame would compare every sample with the
        # same stale reference, and near a singularity could hop branches the
        # played clip never takes -- a jump the planner would then budget for.
        # So each sample is referenced to the previous SAMPLE.
        mod = geo.hdaModule() if geo.type().definition() else None
        memo = getattr(mod, "_IK_MEMO", None)
        aik = geo.node("analytic_ik")
        key_prev = (geo.path(), round(hou.frame() - 1, 4))
        angles = []
        for i in range(NS + 1):
            prog.set(i * du)
            angles.append(extract(src.geometry(), axis_of))
            if memo is not None and aik is not None and aik.geometry().findGlobalAttrib("ik_q"):
                memo[key_prev] = list(aik.geometry().attribValue("ik_q"))
        if mod is not None and hasattr(mod, "clear_ik_memo"):
            mod.clear_ik_memo(geo)

        # unwrapped path, for the acceleration-aware planner
        path = [list(angles[0])]
        for i in range(1, NS + 1):
            row = []
            for j in range(6):
                d = angles[i][j] - angles[i - 1][j]
                d -= 360.0 * round(d / 360.0)
                row.append(path[-1][j] + d)
            path.append(row)

        cum = [0.0]
        rates = []
        for i in range(NS):
            rate = 0.0
            need = 0.0      # seconds this step needs at the slowest-relative joint
            for j in range(6):
                # Wrap into [-180, 180]. Extracted angles live in (-180, 180],
                # so a joint passing through the boundary reads as a 360 deg
                # jump. Left unwrapped, the retime spends its whole budget
                # slowing down for a measurement artifact.
                d = angles[i + 1][j] - angles[i][j]
                d -= 360.0 * round(d / 360.0)
                rate = max(rate, abs(d) / du)
                need = max(need, abs(d) / wlim[j])
            rates.append(rate)
            cum.append(cum[-1] + max(need, dt_floor))

        # With an acceleration limit, plan velocity AND acceleration together
        # (retime_topp): the clip then plays at its own speed on the robot,
        # instead of the player slowing the whole of it for one stretch.
        acc = _acceleration_limits(geo)
        planner = "velocity only"
        if acc:
            _scr = hou.expandString("$HIP/..") + "/scripts"
            if _scr not in sys.path:
                sys.path.insert(0, _scr)
            import importlib
            import retime_topp
            import fairino_player
            importlib.reload(retime_topp)
            importlib.reload(fairino_player)
            # Safety is the one knob: the plan runs at robot limit x Safety,
            # which the Retime folder shows (read-only) as Max Velocity / Max
            # Acceleration. The frames are then fitted to the ROBOT's limit.
            plan_acc = [a * safety for a in acc]
            t_topp = retime_topp.plan(path, wlim, plan_acc)
            cum = [0.0]
            for i in range(NS):
                cum.append(cum[-1] + max(t_topp[i + 1] - t_topp[i], dt_floor))
            # The plan holds on the path samples; the robot plays the 24 fps
            # frames through the player's cubic, and at a corner of the goal
            # curve a joint's velocity jumps between two frames. Measure the
            # frames as the player does and slow only those moments: first on
            # frames estimated from the samples (fast), then on frames cooked
            # at their real progress, until the player would not slow them.
            need_of = (lambda ts, qs, P=fairino_player, v=_velocity_limits(node, geo), a=acc:
                       P.need_profile(ts, qs, 125.0, v, a))
            cum, passes, _ok = retime_topp.fit(
                cum, lambda c, R=retime_topp, p=path, f=fps: R.frames(c, p, f), need_of)
            # Those frames are estimates. The real ones -- keyed, eased, cooked
            # -- are measured and fitted below, after keying.
            planner = "velocity + acceleration (plan %g deg/s^2, robot %g; corners fitted in %d pass%s)" % (
                plan_acc[0], acc[0], passes, "" if passes == 1 else "es")

        total = cum[-1]
        f0 = int(hou.playbar.frameRange()[0])
        f1 = int(hou.playbar.frameRange()[1])

        # Ease in / out: start and end at rest. Without it time-optimal runs
        # at the limit from frame 1 -- a clip measured 91 deg/s between its
        # first two frames -- and a robot, which has to get there from rest,
        # can only be given that by slowing the whole clip.
        _scripts = hou.expandString("$HIP/..") + "/scripts"
        if _scripts not in sys.path:
            sys.path.insert(0, _scripts)
        import retime_ease
        ease_in = max(0.0, _value(node, geo, "retime_ease_in", 0.0))
        ease_out = max(0.0, _value(node, geo, "retime_ease_out", 0.0))
        note = ""

        # Fit on what the robot will actually play: the keyed frames, eased,
        # cooked in order (as Recache does), measured as the player measures
        # them (need_profile), and slowed locally where they still break a
        # limit. Measuring anything else misleads: the player resamples the
        # whole clip on a grid set by its length, so the same curve reads a
        # few % differently in a clip of another length -- frames fitted
        # before the ease was added passed, then failed Pre-Flight at 1.04x
        # (J4 acceleration by the wrist). A uniform slow-down does not
        # converge either: it moves the grid again (1.04x became 1.18x).
        lengthen = False
        verify = ""
        rounds = 0
        for attempt in range(8):
            if int(node.parm("retime_mode").eval()) == 0:
                # Preserve Duration: keep the existing frame count and only
                # REDISTRIBUTE time, so hard stretches get more frames and easy
                # ones fewer. Time-optimal mode instead runs everything up to the
                # limit, which shortens the clip -- useful for cycle time, but not
                # what you want when the motion is already within limits.
                nframes = max(1, f1 - f0)
                want = nframes / fps
                # the ramps take (in + out) / 2 of extra time; the plan gets the rest
                covered = (ease_in + ease_out) / 2.0
                if covered > want / 2.0:
                    k = (want / 2.0) / covered
                    ease_in, ease_out, covered = ease_in * k, ease_out * k, want / 2.0
                if total > 1e-9:
                    scale = (want - covered) / total
                    if (scale < 1.0 and acc) or lengthen:
                        # squeezing would break the limits: lengthen instead
                        nframes = max(1, int(math.ceil(retime_ease.eased_duration(total, ease_in, ease_out) * fps)))
                        note = "; lengthened from %d frames to stay within the limits" % (f1 - f0 + 1)
                    else:
                        cum = [c * scale for c in cum]
                        total = cum[-1]
            else:
                nframes = max(1, int(math.ceil(retime_ease.eased_duration(total, ease_in, ease_out) * fps)))

            prog.deleteAllKeyframes()
            k_index = 0
            for fi in range(nframes + 1):
                tt = retime_ease.source_time(fi / fps, total, ease_in, ease_out)
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

            if not acc:
                break
            if mod is not None and hasattr(mod, "clear_ik_memo"):
                mod.clear_ik_memo(geo)
            vt, vq = [], []
            for fi in range(nframes + 1):
                a = extract(src.geometryAtFrame(f0 + fi), axis_of)
                if vq:
                    a = [pv + (b - pv) - 360.0 * round((b - pv) / 360.0) for pv, b in zip(vq[-1], a)]
                vt.append(fi / fps)
                vq.append(a)
            if mod is not None and hasattr(mod, "clear_ik_memo"):
                mod.clear_ik_memo(geo)
            bad = [(retime_ease.source_time(t, total, ease_in, ease_out), n)
                   for t, n in need_of(vt, vq) if n > 1.0]
            if not bad:
                verify = "; keyed frames fitted in %d round%s" % (rounds, "" if rounds == 1 else "s")
                break
            if attempt == 7:
                _s = fairino_player.limiting(vt, vq, 125.0, _velocity_limits(node, geo), acc)["scale_needed"]
                verify = "; keyed frames NOT converged: plays %.2fx slower, see Pre-Flight" % _s
                break
            cum = retime_topp.stretch(cum, bad)
            total = cum[-1]
            rounds += 1
            lengthen = True
        fend = f0 + nframes
        if node.parm("retime_set_range").eval():
            hou.playbar.setFrameRange(f0, fend)
            hou.playbar.setPlaybackRange(f0, fend)

        node.parm("retime_status").set("Retimed to %d frames (%d-%d), %s, ease %g / %g s%s%s"
                                       % (nframes + 1, f0, fend, planner, ease_in, ease_out, note, verify))
        if hou.isUIAvailable():
            hou.ui.displayMessage(
                "Retimed to %d frames (%d-%d) at %g%% of each joint's limit (%s deg/s).\n"
                "Peak joint rate %.0f deg per unit u.\n\n"
                "The path in space is unchanged -- only the timing.\n"
                "Recache the IK solve to refresh the analysis."
                % (nframes + 1, f0, fend,
                   node.parm("retime_safety").eval() * 100.0,
                   "/".join("%g" % v for v in _velocity_limits(node, geo)), max(rates)),
                title="Retime")
