"""The room is an OpenUSD file: envs/<room>.usda is the source.

Houdini (Solaris), Isaac Sim, usdview and a text editor all open it, and an
AI edits it as text. collision.load_env reads it into the room the checks
use, so nothing that takes a room changes:

    room = read("envs/volvox_lab.usda")      {name, frame, margin_m, objects: [...]}
    write(room, "envs/volvox_lab.usda")      the file from such a room
    write_show(cfg, "shows/party.usda")      the show's layer over the room: its paper, its stage

    python scripts/room_usd.py                               self-test
    python scripts/room_usd.py --rewrite envs/volvox_lab.usda   re-fit wall extents, faces and zone outlines after an edit
    python scripts/room_usd.py --show shows/party.json       write shows/party.usda

The file (robot base frame: Z up, metres, the base at the origin):

    /Room                   defaultPrim. motionlab:margin_m, the clearance
                            every obstacle keeps unless it has its own; its
                            documentation is the frame, motionlab:source says
                            how the room was measured
    /Room/Structure/<name>  floor, ceiling, walls: an Xform holding a `slab`,
                            a Cube with collision. The slab's face towards the
                            robot base is the measured plane: collision.py
                            treats the slab as the halfspace behind that face,
                            so the check is exact however long the slab is.
                            A wall also holds its `face` (one single-sided quad
                            turned into the room: a cutaway, hidden from
                            outside); the ceiling its `outline`. They are for
                            display; the slab is what counts.
    /Room/Objects/<name>    obstacles: a Cube, Cylinder or Sphere with
                            collision and a UsdPreviewSurface look
    /Room/Zones/<name>      volumes the checks use (work, keep_out, slow): an
                            Xform holding a `volume` (purpose guide: drawn as
                            a helper, never rendered, no collision) and an
                            `outline` (curves, up to the walls)
    /Room/Looks             the materials (room_geom.LOOKS, shared with the
                            Houdini display)

What the toolkit needs beyond geometry sits on the shape prims as
namespaced attributes, USD's own way to extend a file:

    motionlab:role            obstacle | keep_out | slow | work; a prim with
                              collision and no role is an obstacle
    motionlab:margin_m        its own clearance, when not the room's
    motionlab:tcp_speed_mps   a slow zone's TCP speed cap
    motionlab:measured        how it was measured: true, tape, scan
    documentation             its note

Editing: move, turn or scale a Cube or an Xform; add a Cube with collision
under /Room/Objects and it is an obstacle; deactivate a prim to drop it.
Objects and zones turn about Z only (collision.py's boxes); slabs may face
any way. A mesh with collision is refused (use Cube / Cylinder / Sphere).
After moving walls, --rewrite re-fits the slabs' extents, the wall faces
and the zone outlines (it rewrites the file from what it reads: materials
not in room_geom.LOOKS and prims it does not know are dropped).

Needs pxr: Houdini's hython has it; the system Python gets it from PyPI's
usd-core (Pixar's own build).
"""

import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import room_geom as RG  # noqa: E402

ENV_SCHEMA = "motionlab.env/1"          # the in-memory room collision.py validates
WALL_T = 0.08                           # slab thicknesses, into the wall (m)
FLOOR_T = 0.05
CEIL_T = 0.05
NS = "motionlab:"
TOL = 1e-9                              # read values are rounded to this (m, deg)


# --------------------------------------------------------------------------
# read
# --------------------------------------------------------------------------

def _r(x):
    return round(float(x), 9) + 0.0      # + 0.0: no -0.0 in the room


def _axes(prim):
    """World centre and the three scaled half-axes of a gprim's unit shape."""
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    c = m.Transform(Gf.Vec3d(0, 0, 0))
    return c, [m.TransformDir(Gf.Vec3d(*e)) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]


def _upright(prim, axes, what):
    """Refuse a shape turned out of the horizontal (the checks turn about Z only)."""
    z = axes[2]
    if abs(z[2]) < (1.0 - 1e-6) * z.GetLength() or abs(axes[0][2]) > 1e-6 * axes[0].GetLength():
        raise ValueError("%s: %s may turn about Z only (collision.py); it is tilted" % (prim.GetPath(), what))


def _attr(prim, name, default=None):
    a = prim.GetAttribute(NS + name)
    return a.Get() if a and a.HasAuthoredValue() else default


def _meta(prim, o):
    """The motionlab:* attributes and the note onto the room object o."""
    for k in ("margin_m", "tcp_speed_mps"):
        v = _attr(prim, k)
        if v is not None:
            o[k] = _r(v)
    m = _attr(prim, "measured")
    if m is not None:
        o["measured"] = {"true": True, "false": False}.get(m, m)
    doc = prim.GetDocumentation()
    if doc:
        o["note"] = doc
    return o


def _slab(prim, name):
    """A structure slab as the halfspace behind its face towards the origin."""
    c, ax = _axes(prim)
    size = UsdGeom.Cube(prim).GetSizeAttr().Get()
    thin = min(range(3), key=lambda i: ax[i].GetLength())
    half = ax[thin].GetLength() * size / 2.0
    u = ax[thin].GetNormalized()
    uc = Gf.Dot(u, c)
    if uc + half < 0:                     # the origin is past the +u face: the room is on that side
        n, offset = u, uc + half
    elif uc - half > 0:
        n, offset = -u, -uc + half
    else:
        raise ValueError("%s: the robot base is inside the slab" % prim.GetPath())
    return {"name": name, "type": "halfspace", "normal": [_r(x) for x in n], "offset": _r(offset)}


def _shape(prim, name):
    """A Cube / Cylinder / Sphere as the room's box / cylinder / sphere."""
    c, ax = _axes(prim)
    if prim.IsA(UsdGeom.Cube):
        _upright(prim, ax, "a box")
        s = UsdGeom.Cube(prim).GetSizeAttr().Get()
        return {"name": name, "type": "box", "center": [_r(x) for x in c],
                "size": [_r(a.GetLength() * s) for a in ax],
                "yaw_deg": _r(math.degrees(math.atan2(ax[0][1], ax[0][0])))}
    if prim.IsA(UsdGeom.Cylinder):
        cyl = UsdGeom.Cylinder(prim)
        if cyl.GetAxisAttr().Get() != "Z":
            raise ValueError("%s: a cylinder must stand on Z (axis Z)" % prim.GetPath())
        _upright(prim, ax, "a cylinder")
        sx, sy, sz = (a.GetLength() for a in ax)
        if abs(sx - sy) > 1e-6 * max(sx, sy):
            raise ValueError("%s: a cylinder must be scaled alike in X and Y" % prim.GetPath())
        h = cyl.GetHeightAttr().Get() * sz
        return {"name": name, "type": "cylinder", "center": [_r(c[0]), _r(c[1]), _r(c[2] - h / 2.0)],
                "radius": _r(cyl.GetRadiusAttr().Get() * sx), "height": _r(h)}
    if prim.IsA(UsdGeom.Sphere):
        s = [a.GetLength() for a in ax]
        if max(s) - min(s) > 1e-6 * max(s):
            raise ValueError("%s: a sphere must be scaled alike on all axes" % prim.GetPath())
        return {"name": name, "type": "sphere", "center": [_r(x) for x in c],
                "radius": _r(UsdGeom.Sphere(prim).GetRadiusAttr().Get() * s[0])}
    raise ValueError("%s: %s with collision or a role: use a Cube, Cylinder or Sphere"
                     % (prim.GetPath(), prim.GetTypeName()))


def read_stage(stage, name="room"):
    """The room (collision.py's dict) from an open stage."""
    root = stage.GetDefaultPrim()
    if not root:
        raise ValueError("%s: no defaultPrim (expected /Room)" % stage.GetRootLayer().identifier)
    base = root.GetPath()
    structure = base.AppendChild("Structure")
    room = {"schema": ENV_SCHEMA, "name": name, "frame": root.GetDocumentation(),
            "source": _attr(root, "source", ""), "margin_m": _r(_attr(root, "margin_m", 0.05)), "objects": []}
    for prim in Usd.PrimRange(root):
        role = _attr(prim, "role")
        collides = prim.HasAPI(UsdPhysics.CollisionAPI)
        if role is None and not collides:
            continue
        rel = prim.GetPath().MakeRelativePath(base).pathString.split("/")
        if len(rel) < 2:
            raise ValueError("%s: a room object sits under /Room/<Structure|Objects|Zones>/<name>" % prim.GetPath())
        oname = rel[1]                                    # the object's name: its group under the scope
        if prim.GetPath().HasPrefix(structure):
            if not prim.IsA(UsdGeom.Cube):
                raise ValueError("%s: a room boundary is a Cube slab" % prim.GetPath())
            o = _slab(prim, oname)
        else:
            o = _shape(prim, oname)
        o["role"] = role or "obstacle"
        room["objects"].append(_meta(prim, o))
    names = [o["name"] for o in room["objects"]]
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        raise ValueError("%s: two objects named %s (one shape per group)" % (stage.GetRootLayer().identifier, dup))
    return room


def read(path):
    """The room from a .usda / .usd / .usdc file (any layer stack over it)."""
    stage = Usd.Stage.Open(path, Usd.Stage.LoadAll)
    if stage is None:
        raise ValueError("%s: USD could not open it" % path)
    return read_stage(stage, os.path.splitext(os.path.basename(path))[0])


# --------------------------------------------------------------------------
# write
# --------------------------------------------------------------------------

def _dop(xf, kind, value):
    """A double-precision xform op (metres and degrees survive the round trip)."""
    add = {"t": xf.AddTranslateOp, "rz": xf.AddRotateZOp, "s": xf.AddScaleOp}[kind]
    op = add(UsdGeom.XformOp.PrecisionDouble)
    op.Set(Gf.Vec3d(*value) if kind != "rz" else float(value))
    return op


def _place(prim, t=None, rz=None, s=None):
    xf = UsdGeom.Xformable(prim)
    if t is not None:
        _dop(xf, "t", t)
    if rz:
        _dop(xf, "rz", rz)
    if s is not None:
        _dop(xf, "s", s)


def _material(stage, key):
    path = "/Room/Looks/" + key
    if stage.GetPrimAtPath(path):
        return UsdShade.Material.Get(stage, path)
    rgb, rough, metal, opacity = RG.LOOKS[key]
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    if opacity < 1.0:
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _bind(prim, look):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(_material(prim.GetStage(), look))


def _tag(prim, o, room_margin, collide):
    """The motionlab:* attributes, the note and (for solids) collision."""
    prim.CreateAttribute(NS + "role", Sdf.ValueTypeNames.String).Set(o["role"])
    if "margin_m" in o:
        prim.CreateAttribute(NS + "margin_m", Sdf.ValueTypeNames.Double).Set(float(o["margin_m"]))
    if "tcp_speed_mps" in o:
        prim.CreateAttribute(NS + "tcp_speed_mps", Sdf.ValueTypeNames.Double).Set(float(o["tcp_speed_mps"]))
    if "measured" in o:
        m = o["measured"]
        prim.CreateAttribute(NS + "measured", Sdf.ValueTypeNames.String).Set(
            str(m).lower() if isinstance(m, bool) else str(m))
    if o.get("note"):
        prim.SetDocumentation(o["note"])
    if collide:
        UsdPhysics.CollisionAPI.Apply(prim)


def _mesh(stage, path, points, faces):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    return m


def _curves(stage, path, polylines, rgb, width=0.005):
    c = UsdGeom.BasisCurves.Define(stage, path)
    c.CreateTypeAttr(UsdGeom.Tokens.linear)
    c.CreateCurveVertexCountsAttr([len(p) for p in polylines])
    c.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*x) for p in polylines for x in p]))
    c.CreateWidthsAttr(Vt.FloatArray([width]))
    c.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    c.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set([Gf.Vec3f(*rgb)])
    return c


def _safe(name):
    return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in name)


def _local(p, t, yaw_deg):
    """p in the frame translated by t and turned yaw_deg about Z."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    dx, dy, dz = p[0] - t[0], p[1] - t[1], p[2] - t[2]
    return (c * dx + s * dy, -s * dx + c * dy, dz)


def _room_frame(room):
    """(yaw, centre xy, size xy) of the footprint in the walls' frame: the
    rectangle the floor and ceiling slabs cover."""
    walls = RG.vertical_walls(room)
    yaw = math.degrees(math.atan2(walls[0]["normal"][1], walls[0]["normal"][0])) if walls else 0.0
    fp = RG.footprint(room)
    loc = [_local((x, y, 0.0), (0.0, 0.0, 0.0), yaw) for x, y in fp]
    lo = [min(p[i] for p in loc) for i in (0, 1)]
    hi = [max(p[i] for p in loc) for i in (0, 1)]
    cl = ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0)
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return yaw, (c * cl[0] - s * cl[1], s * cl[0] + c * cl[1]), (hi[0] - lo[0], hi[1] - lo[1])


def _structure(stage, room, o, margin):
    """A boundary halfspace as an Xform holding its slab (+ face / outline)."""
    n = [float(x) for x in o["normal"]]
    L = math.sqrt(sum(x * x for x in n))
    n, d = [x / L for x in n], o["offset"] / L
    path = "/Room/Structure/" + _safe(o["name"])
    z0, z1 = RG.heights(room)
    grp = UsdGeom.Xform.Define(stage, path)
    if abs(n[2]) > 1.0 - 1e-9:                                   # floor (n up) or ceiling (n down)
        yaw, cxy, sxy = _room_frame(room)
        z = d / n[2]
        _place(grp.GetPrim(), (cxy[0], cxy[1], z), yaw)
        t = FLOOR_T if n[2] > 0 else CEIL_T
        slab = UsdGeom.Cube.Define(stage, path + "/slab")
        slab.CreateSizeAttr(1.0)
        _place(slab.GetPrim(), (0.0, 0.0, -t / 2.0 if n[2] > 0 else t / 2.0), None,
               (sxy[0] + 2 * WALL_T, sxy[1] + 2 * WALL_T, t))
        _tag(slab.GetPrim(), o, margin, True)
        if n[2] > 0:
            _bind(slab.GetPrim(), "floor")
        else:
            UsdGeom.Imageable(slab).MakeInvisible()
            ring = [_local((x, y, z), (cxy[0], cxy[1], z), yaw) for x, y in RG.footprint(room)]
            _curves(stage, path + "/outline", [ring + ring[:1]], RG.ZONE_RGB["ceiling"], 0.015)
        return
    if abs(n[2]) > 1e-9:
        raise ValueError("%s: a sloping plane (normal %s) cannot be written yet" % (o["name"], o["normal"]))
    yaw = math.degrees(math.atan2(n[1], n[0]))                   # local +X into the room
    bottom, top = z0 - FLOOR_T, z1 + CEIL_T
    piece = {w[0]: w[1] for w in RG.walls(room)}.get(o["name"])
    if piece:                                                    # the wall's edge of the floor, and a slab past it
        a, b = piece[0], piece[1]
        mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        length = math.hypot(b[0] - a[0], b[1] - a[1]) + 2 * WALL_T
    else:                                                        # a wall that does not bound the room
        mid = (n[0] * d, n[1] * d)
        length = 2 * RG.OPEN_HALF
    t0 = (mid[0], mid[1], (bottom + top) / 2.0)
    _place(grp.GetPrim(), t0, yaw)
    slab = UsdGeom.Cube.Define(stage, path + "/slab")
    slab.CreateSizeAttr(1.0)
    _place(slab.GetPrim(), (-WALL_T / 2.0, 0.0, 0.0), None, (WALL_T, length, top - bottom))
    _tag(slab.GetPrim(), o, margin, True)
    UsdGeom.Imageable(slab).MakeInvisible()                      # drawn by its face, a cutaway
    if piece:
        corners = [_local(p, t0, yaw) for p in piece]            # x = 0: on the wall's room side
        # the quad's normal into the room (+X): right-handed winding, reversed when needed
        e1 = [corners[1][i] - corners[0][i] for i in range(3)]
        e2 = [corners[2][i] - corners[0][i] for i in range(3)]
        nx = e1[1] * e2[2] - e1[2] * e2[1]
        order = [0, 1, 2, 3] if nx > 0 else [3, 2, 1, 0]
        face = _mesh(stage, path + "/face", [corners[i] for i in order], [[0, 1, 2, 3]])
        face.CreateDoubleSidedAttr(False)
        # Omniverse RTX culls by its own "singleSided" attribute (Isaac's "Single Sided" toggle)
        face.GetPrim().CreateAttribute("singleSided", Sdf.ValueTypeNames.Bool).Set(True)
        _bind(face.GetPrim(), RG.wall_look(o["name"]))


def _object(stage, room, o, margin, scope="Objects"):
    """An obstacle as a Cube / Cylinder / Sphere; a zone as an Xform holding its
    guide volume and its outline."""
    name = _safe(o["name"])
    zone = o["role"] != "obstacle"
    path = "/Room/%s/%s" % ("Zones" if zone else scope, name)
    if zone:
        grp = UsdGeom.Xform.Define(stage, path)
        at = list(o["center"])
        yaw = o.get("yaw_deg", 0.0) if o["type"] == "box" else 0.0
        _place(grp.GetPrim(), at, yaw)
        shape_path, origin = path + "/volume", (0.0, 0.0, 0.0)
    else:
        shape_path, origin, yaw = path, None, o.get("yaw_deg", 0.0)
    if o["type"] == "box":
        g = UsdGeom.Cube.Define(stage, shape_path)
        g.CreateSizeAttr(1.0)
        _place(g.GetPrim(), origin or o["center"], None if zone else yaw, o["size"])
    elif o["type"] == "cylinder":
        g = UsdGeom.Cylinder.Define(stage, shape_path)
        g.CreateAxisAttr("Z")
        g.CreateRadiusAttr(float(o["radius"]))
        g.CreateHeightAttr(float(o["height"]))
        c = origin or o["center"]
        _place(g.GetPrim(), (c[0], c[1], c[2] + o["height"] / 2.0))
    elif o["type"] == "sphere":
        g = UsdGeom.Sphere.Define(stage, shape_path)
        g.CreateRadiusAttr(float(o["radius"]))
        _place(g.GetPrim(), origin or o["center"])
    else:
        raise ValueError("%s: no USD shape for %r" % (o["name"], o["type"]))
    # the ops above are double; a Cube / Cylinder / Sphere's own extent follows its size
    _tag(g.GetPrim(), o, margin, not zone)
    if zone:
        g.CreatePurposeAttr(UsdGeom.Tokens.guide)
        lines = RG.zone_lines(o, RG.footprint(room))                 # up to the walls, not through them
        if lines:
            loc = [[_local(p, o["center"], yaw) for p in line] for line in lines]
            _curves(stage, path + "/outline", loc, RG.ZONE_RGB.get(o["role"], (1, 1, 1)))
    else:
        _bind(g.GetPrim(), RG.look_key(o["name"]))


def build_stage(room, stage=None, extra=()):
    """The room written into stage (a new in-memory one by default)."""
    stage = stage or Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Room")
    stage.SetDefaultPrim(root.GetPrim())
    if room.get("frame"):
        root.GetPrim().SetDocumentation(room["frame"])
    root.GetPrim().CreateAttribute(NS + "margin_m", Sdf.ValueTypeNames.Double).Set(float(room.get("margin_m", 0.05)))
    if room.get("source"):
        root.GetPrim().CreateAttribute(NS + "source", Sdf.ValueTypeNames.String).Set(room["source"])
    for s in ("Structure", "Objects", "Zones", "Looks"):
        UsdGeom.Scope.Define(stage, "/Room/" + s)
    margin = float(room.get("margin_m", 0.05))
    for o in list(room["objects"]) + list(extra):
        if o["type"] == "halfspace":
            _structure(stage, room, o, margin)
        else:
            _object(stage, room, o, margin)
    return stage


def write(room, path, extra=()):
    """Write the room to path (.usda: text, diffable in git)."""
    stage = build_stage(room, extra=extra)
    stage.GetRootLayer().Export(path)
    return path


# --------------------------------------------------------------------------
# a show's layer
# --------------------------------------------------------------------------

def show_extras(cfg):
    """The show's paper (an obstacle) and, when it sets one, its stage (work)."""
    out = []
    c = cfg.get("canvas")
    if c:
        n = c["normal"]
        out.append({"name": "canvas", "type": "box", "center": c["center"], "role": "obstacle",
                    "size": [c.get("thickness", 0.02), c["size"][0], c["size"][1]],
                    "yaw_deg": math.degrees(math.atan2(n[1], n[0])),
                    "note": "the show's paper (placeholder)" if c.get("placeholder") else "the show's paper"})
    st = cfg.get("stage")
    if st:
        out.append({"name": "show_stage", "type": "box", "center": st["center"], "size": st["size"],
                    "yaw_deg": st.get("yaw_deg", 0.0), "role": "work", "note": "the show's stage"})
    return out


def show_layer_path(cfg_path):
    return os.path.splitext(cfg_path)[0] + ".usda"


def write_show(cfg_path, out=None):
    """shows/<show>.usda: a layer that sublayers the room and adds the show's
    paper and stage -- the room file itself stays as measured (Isaac opens this)."""
    import json
    cfg = json.load(open(cfg_path))
    room_path = os.path.join(ROOT, cfg["env"])
    out = out or show_layer_path(cfg_path)
    room = read(room_path)
    layer = Sdf.Layer.FindOrOpen(out)
    if layer:
        layer.Clear()
    else:
        layer = Sdf.Layer.CreateNew(out)            # on disk, so the relative sublayer resolves
    try:
        sub = os.path.relpath(room_path, os.path.dirname(os.path.abspath(out)))
    except ValueError:                              # another drive: no relative path
        sub = os.path.abspath(room_path)
    layer.subLayerPaths.append(sub.replace("\\", "/"))
    layer.defaultPrim = "Room"
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    margin = float(room.get("margin_m", 0.05))
    for o in show_extras(cfg):
        _object(stage, room, o, margin)
    layer.Save()
    return out


# --------------------------------------------------------------------------
# self-test
# --------------------------------------------------------------------------

def _same(a, b, tol=1e-6):
    """Two rooms' objects alike (by name), numbers within tol: [differences]."""
    diffs = []
    ba = {o["name"]: o for o in a["objects"]}
    bb = {o["name"]: o for o in b["objects"]}
    if set(ba) != set(bb):
        return ["names differ: %s" % sorted(set(ba) ^ set(bb))]
    for k, oa in ba.items():
        ob = bb[k]
        for f in set(oa) | set(ob):
            va, vb = oa.get(f), ob.get(f)
            if f == "normal":
                la = math.sqrt(sum(x * x for x in va))
                va, vb = [x / la for x in va], vb
            if f == "offset" and "normal" in oa:
                va = va / math.sqrt(sum(x * x for x in oa["normal"]))
            if isinstance(va, (list, tuple)) and isinstance(vb, (list, tuple)):
                if len(va) != len(vb) or any(abs(x - y) > tol for x, y in zip(va, vb)):
                    diffs.append("%s.%s %s vs %s" % (k, f, va, vb))
            elif isinstance(va, float) or isinstance(vb, float):
                if f == "yaw_deg":
                    dv = (float(va or 0.0) - float(vb or 0.0) + 180.0) % 360.0 - 180.0
                    # a box turned 180 deg is the same box
                    if min(abs(dv), abs(abs(dv) - 180.0)) > tol:
                        diffs.append("%s.%s %s vs %s" % (k, f, va, vb))
                elif va is None or vb is None or abs(va - vb) > tol:
                    diffs.append("%s.%s %s vs %s" % (k, f, va, vb))
            elif va != vb:
                diffs.append("%s.%s %r vs %r" % (k, f, va, vb))
    return diffs


def self_test():
    import tempfile
    import collision as CL
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    room = {"schema": ENV_SCHEMA, "name": "test", "frame": "a test room", "source": "made up", "margin_m": 0.05,
            "objects": [
                {"name": "floor", "type": "halfspace", "normal": [0.0, 0.0, 1.0], "offset": -0.01, "role": "obstacle",
                 "measured": True, "note": "level"},
                {"name": "ceiling", "type": "halfspace", "normal": [0.0, 0.0, -1.0], "offset": -2.2,
                 "role": "obstacle", "measured": "tape"},
                {"name": "wall_a", "type": "halfspace", "normal": [0.980679, -0.195625, 0.0], "offset": -1.7,
                 "role": "obstacle"},
                {"name": "wall_b", "type": "halfspace", "normal": [0.195625, 0.980679, 0.0], "offset": -1.1,
                 "role": "obstacle"},
                {"name": "cart", "type": "box", "center": [1.5, -0.05, 0.54], "size": [0.69, 0.63, 1.09],
                 "yaw_deg": -101.04, "role": "obstacle", "margin_m": 0.02},
                {"name": "plate", "type": "box", "center": [-0.15, 0.0, -0.01], "size": [1.1, 0.8, 0.02],
                 "yaw_deg": -11.28, "role": "obstacle", "margin_m": 0.0},
                {"name": "plant", "type": "cylinder", "center": [-1.0, 0.9, 0.0], "radius": 0.3, "height": 1.9,
                 "role": "obstacle"},
                {"name": "ball", "type": "sphere", "center": [0.8, 0.8, 1.2], "radius": 0.1, "role": "obstacle"},
                {"name": "person", "type": "cylinder", "center": [1.4, -0.7, 0.0], "radius": 0.4, "height": 2.0,
                 "role": "keep_out"},
                {"name": "near", "type": "cylinder", "center": [1.4, -0.7, 0.0], "radius": 1.3, "height": 2.4,
                 "role": "slow", "tcp_speed_mps": 0.25},
                {"name": "stage", "type": "box", "center": [-0.4, 0.6, 1.0], "size": [2.2, 3.0, 2.0],
                 "yaw_deg": -11.28, "role": "work"}]}
    tmp = tempfile.mkdtemp()
    p = os.path.join(tmp, "test.usda")
    write(room, p)
    back = read(p)
    d = _same(room, back)
    check("a room written and read back is the same room (1e-6)", not d, d[:4])
    check("its name, frame, source and margin come back",
          (back["name"], back["frame"], back["source"], back["margin_m"]) == ("test", "a test room", "made up", 0.05))
    check("collision.py accepts what it reads", not CL.validate_env(back), CL.validate_env(back))
    check("an object without its own margin gets none (it uses the room's)",
          "margin_m" not in {o["name"]: o for o in back["objects"]}["plant"])

    # the clearance of the robot at a pose is unchanged
    model = CL.load_model("fr20")
    qs = [[0.0, -90.0, 90.0, -90.0, -90.0, 0.0], [-60.0, -70.0, 80.0, -100.0, -90.0, 30.0]]
    ra, rb = CL.check(model, room, [0.0, 1.0], qs), CL.check(model, back, [0.0, 1.0], qs)
    check("the same clearances for the robot", abs(ra["min_clearance_m"] - rb["min_clearance_m"]) < 1e-6,
          (ra["min_clearance_m"], rb["min_clearance_m"]))

    # the file itself: collision, looks, the cutaway, zones as guides
    st = Usd.Stage.Open(p)
    slab = st.GetPrimAtPath("/Room/Structure/wall_a/slab")
    check("a wall is a slab with collision, drawn by its face",
          slab.IsA(UsdGeom.Cube) and slab.HasAPI(UsdPhysics.CollisionAPI)
          and UsdGeom.Imageable(slab).ComputeVisibility() == UsdGeom.Tokens.invisible
          and st.GetPrimAtPath("/Room/Structure/wall_a/face").IsValid())
    vol = st.GetPrimAtPath("/Room/Zones/stage/volume")
    check("a zone is a guide volume without collision, with an outline",
          UsdGeom.Imageable(vol).GetPurposeAttr().Get() == UsdGeom.Tokens.guide
          and not vol.HasAPI(UsdPhysics.CollisionAPI) and st.GetPrimAtPath("/Room/Zones/stage/outline").IsValid())
    check("an obstacle has a material",
          bool(UsdShade.MaterialBindingAPI(st.GetPrimAtPath("/Room/Objects/cart")).ComputeBoundMaterial()[0]))

    # edited as a person would: move the cart, raise the ceiling, add a box with collision only
    cart = UsdGeom.Xformable(st.GetPrimAtPath("/Room/Objects/cart"))
    cart.GetOrderedXformOps()[0].Set(Gf.Vec3d(1.2, -0.05, 0.54))
    UsdGeom.Xformable(st.GetPrimAtPath("/Room/Structure/ceiling")).GetOrderedXformOps()[0].Set(
        Gf.Vec3d(0.0, 0.0, 2.4))
    box = UsdGeom.Cube.Define(st, "/Room/Objects/new_box")
    UsdGeom.Xformable(box).AddTranslateOp().Set(Gf.Vec3d(0.5, 0.5, 0.25))
    UsdGeom.Xformable(box).AddScaleOp().Set(Gf.Vec3f(0.25, 0.25, 0.25))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    ed = read_stage(st, "test")
    eo = {o["name"]: o for o in ed["objects"]}
    check("a moved object is read where it was moved to", abs(eo["cart"]["center"][0] - 1.2) < 1e-9)
    check("a moved ceiling slab gives the new ceiling plane", abs(eo["ceiling"]["offset"] + 2.4) < 1e-9,
          eo["ceiling"])
    nb = eo.get("new_box", {})
    check("a Cube added with collision only is an obstacle (default Cube size 2, scaled)",
          nb.get("role") == "obstacle" and nb.get("type") == "box" and
          all(abs(x - 0.5) < 1e-6 for x in nb.get("size", [0])), nb)
    UsdGeom.Xformable(box).AddRotateXOp().Set(20.0)
    try:
        read_stage(st, "test")
        check("a tilted box is refused", False)
    except ValueError as e:
        check("a tilted box is refused, with where", "new_box" in str(e), e)
    st.GetPrimAtPath("/Room/Objects/new_box").SetActive(False)
    check("a deactivated prim is not in the room", "new_box" not in {o["name"] for o in read_stage(st)["objects"]})
    m = UsdGeom.Mesh.Define(st, "/Room/Objects/scan_mesh")
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    try:
        read_stage(st)
        check("a mesh with collision is refused", False)
    except ValueError as e:
        check("a mesh with collision is refused, with what to use", "Cube" in str(e), e)

    # the real room, when it is there
    real = os.path.join(ROOT, "envs", "volvox_lab.usda")
    if os.path.exists(real):
        r = read(real)
        check("envs/volvox_lab.usda reads and validates", not CL.validate_env(r), CL.validate_env(r))
        p2 = os.path.join(tmp, "again.usda")
        write(r, p2)
        d2 = _same(r, read(p2))
        check("the real room survives a rewrite unchanged", not d2, d2[:4])
        import json
        cfgp = os.path.join(tmp, "show.json")
        json.dump({"env": os.path.relpath(real, ROOT), "canvas": {"center": [0.07, 1.41, 0.88], "normal": [0.193, 0.981, 0.0],
                   "size": [1.0, 0.6], "thickness": 0.02}}, open(cfgp, "w"))
        sr = read(write_show(cfgp))
        check("a show's layer is the room with its paper over it",
              sorted(o["name"] for o in sr["objects"]) == sorted([o["name"] for o in r["objects"]] + ["canvas"]))
    print("room_usd self-test: %s" % ("PASS" if not fails else "FAIL: %s" % fails))
    return not fails


def main(argv):
    if not argv:
        sys.exit(0 if self_test() else 1)
    if argv[0] == "--rewrite":
        for p in argv[1:]:
            write(read(p), p)
            print("rewrote", p)
    elif argv[0] == "--show":
        for p in argv[1:]:
            print("wrote", write_show(p))
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
