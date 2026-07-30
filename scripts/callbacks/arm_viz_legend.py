# Update the Colour By legend.
#
# A colour ramp with no stated scale is uninterpretable -- you cannot tell
# whether red means "slightly over" or "ten times the limit". This reads the
# scale the wrangle actually used back off the geometry, so the legend can
# never drift from the colouring.
node = kwargs["node"]
pv = node.node("path_viz")
if pv is None:
    node.parm("viz_legend").set("path_viz not found")
else:
    UNITS = {"residual": ("mm", 1000.0), "orient_error": ("deg", 1.0),
             "vel_max": ("deg/s", 1.0), "flip_ratio": ("ratio", 1.0),
             "limit_margin": ("deg", 1.0), "tcp_speed": ("m/s", 1.0)}
    try:
        g = pv.geometry()
        lo = g.attribValue("viz_lo")
        hi = g.attribValue("viz_hi")
        name = g.attribValue("viz_name")
        absolute = g.attribValue("viz_absolute")
        unit, scale = UNITS.get(name, ("", 1.0))
        vals = [p.attribValue("viz_value") for p in g.points()]
        worst = max(vals) if name != "limit_margin" else min(vals)
        node.parm("viz_legend").set(
            "%s   blue %.4g  ->  red %.4g %s   (%s)   actual range %.4g .. %.4g"
            % (name, lo * scale, hi * scale, unit,
               "profile limits" if absolute else "5th-95th percentile",
               min(vals) * scale, max(vals) * scale))
    except Exception as e:
        node.parm("viz_legend").set("legend unavailable: %s" % str(e)[:70])
