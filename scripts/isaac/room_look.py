"""The real room's look over the simulated one, for every Isaac Sim view of
the project (isaac_stage.load_room; the user, 2026-09-29): lights and
surfaces after the user's photo of the lab. Only
what the camera sees -- no colliders, the room's files (envs/, shows/)
untouched; the collision checks never see any of it.

    notes = room_look.apply(stage, env)      # after the room is referenced at /World/Room

From the photo: a square LED frame hung under the joists is the room's
light (cool white); the warehouse beyond the glass gives a warm fill; a
dark plank floor; the TV wall carved pale plywood; the ceiling open joists
and boards; the glass walls in wooden studs. ESTIMATED from the photo, not
measured: the frame's place (+-0.5 m) and height, its size, the joists'
spacing. Import after SimulationApp has started (it needs pxr).
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
sys.path.insert(0, SCRIPTS)

from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade  # noqa: E402

import room_geom as RG  # noqa: E402

ROOT_PATH = "/World/Look"
YAW_DEG = -11.28                     # the room's walls in the robot frame (envs/volvox_lab.usda)
LED_FRAME = {"center": (-0.4, 0.5), "side_m": 1.3, "below_ceiling_m": 0.15, "bar_m": 0.04}
JOIST = {"spacing_m": 0.41, "width_m": 0.045, "depth_m": 0.24}
BEAM = {"spacing_m": 1.6, "width_m": 0.14, "depth_m": 0.3}
STUD = {"spacing_m": 1.22, "width_m": 0.09}
COOL = (0.88, 0.94, 1.0)             # the LED frame, ~6000 K
WARM = (1.0, 0.8, 0.6)               # the warehouse's light through the glass
# the room's own materials (room_usd.py's Looks), recoloured
SURFACES = {"floor": ((0.17, 0.1, 0.06), 0.5, 1.0),        # dark worn planks
            "wall": ((0.84, 0.69, 0.49), 0.8, 1.0),        # the TV wall: carved pale plywood
            "base_plate": ((0.85, 0.74, 0.55), 0.8, 1.0),  # plywood
            "glass_face": ((0.8, 0.88, 0.92), 0.05, 0.12),  # glass: see-through
            "shelves": ((0.62, 0.62, 0.64), 0.35, 1.0)}     # chrome wire shelving
WOOD = (0.74, 0.52, 0.32)


def _material(stage, name, rgb, roughness=0.7, emissive=None):
    path = Sdf.Path("%s/Materials/%s" % (ROOT_PATH, name))
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path.AppendChild("Surface"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    if emissive:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _box(stage, path, center, size, yaw_deg, mat):
    """A visual box (no collider): center, size (x, y, z), turned yaw_deg about Z."""
    c = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.AddRotateZOp().Set(yaw_deg)
    c.AddScaleOp().Set(Gf.Vec3d(*size))
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
    return c


def _room_frame(fp):
    """(u, v, extents) of the footprint in the room's own axes: u along the
    TV wall's normal turned, v across; extents (u0, u1, v0, v1)."""
    a = math.radians(YAW_DEG)
    u, v = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
    us = [p[0] * u[0] + p[1] * u[1] for p in fp]
    vs = [p[0] * v[0] + p[1] * v[1] for p in fp]
    return u, v, (min(us), max(us), min(vs), max(vs))


def apply(stage, env, room="/World/Room", guides=True):
    """Lights, surfaces and the ceiling's wood as in the photo. guides: the
    safety guides (the zones' and walls' outlines) stay drawn -- the user,
    2026-09-29; False hides them. Returns the notes (what was estimated)."""
    fp = RG.footprint(env)
    z0, z1 = RG.heights(env)
    UsdGeom.Scope.Define(stage, Sdf.Path(ROOT_PATH))
    UsdGeom.Scope.Define(stage, Sdf.Path(ROOT_PATH + "/Materials"))
    wood = _material(stage, "wood", WOOD, 0.75)
    board = _material(stage, "board", (0.8, 0.6, 0.4), 0.8)
    alu = _material(stage, "led", (1.0, 1.0, 1.0), 0.3, emissive=(1.0, 1.0, 1.0))

    # the room's surfaces recoloured
    for name, (rgb, rough, opacity) in SURFACES.items():
        sh = UsdShade.Shader(stage.GetPrimAtPath("%s/Looks/%s/Surface" % (room, name)))
        if not sh:
            continue
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    for p in ([] if guides else stage.Traverse()):
        s = p.GetPath().pathString
        if s.startswith(room + "/Zones") and p.GetParent().GetPath().pathString == room + "/Zones" \
                or (s.startswith(room + "/Structure") and p.GetName() == "outline"):
            UsdGeom.Imageable(p).MakeInvisible()

    u, v, (u0, u1, v0, v1) = _room_frame(fp)

    def at(uu, vv, z):
        return (u[0] * uu + v[0] * vv, u[1] * uu + v[1] * vv, z)

    # the ceiling: boards on joists on beams (the tape's 2.16 m is the joists' underside)
    top = z1 + JOIST["depth_m"]
    _box(stage, ROOT_PATH + "/Ceiling/boards", at((u0 + u1) / 2, (v0 + v1) / 2, top + 0.01),
         (u1 - u0 + 0.4, v1 - v0 + 0.4, 0.02), YAW_DEG, board)
    k = 0
    vv = v0 + 0.2
    while vv < v1:
        _box(stage, ROOT_PATH + "/Ceiling/joist_%02d" % k, at((u0 + u1) / 2, vv, z1 + JOIST["depth_m"] / 2),
             (u1 - u0 + 0.4, JOIST["width_m"], JOIST["depth_m"]), YAW_DEG, wood)
        vv += JOIST["spacing_m"]
        k += 1
    uu, k = u0 + 0.5, 0
    while uu < u1:
        _box(stage, ROOT_PATH + "/Ceiling/beam_%02d" % k, at(uu, (v0 + v1) / 2, z1 - BEAM["depth_m"] / 2 + 0.24),
             (BEAM["width_m"], v1 - v0 + 0.4, BEAM["depth_m"]), YAW_DEG, wood)
        uu += BEAM["spacing_m"]
        k += 1

    # the glass walls' wooden studs and rails
    glass = [w for w in RG.walls(env) if _bound(stage, room, w[0]) == "glass_face"]
    for name, corners, _out in glass:
        a, b = corners[0], corners[1]
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        yaw = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        n = max(1, int(round(L / STUD["spacing_m"])))
        for i in range(n + 1):
            t = i / float(n)
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            _box(stage, "%s/Studs/%s_%02d" % (ROOT_PATH, name, i), (p[0], p[1], (z0 + z1) / 2),
                 (STUD["width_m"], STUD["width_m"], z1 - z0), yaw, wood)
        for j, z in enumerate((z0 + 0.05, z1 - 0.05)):
            _box(stage, "%s/Studs/%s_rail_%d" % (ROOT_PATH, name, j), ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, z),
                 (L, STUD["width_m"], 0.09), yaw, wood)

    # the LED frame: four bars, each a rect light facing down and a glowing bar to see
    f = LED_FRAME
    zf = z1 - f["below_ceiling_m"]
    cu = f["center"][0] * u[0] + f["center"][1] * u[1]
    cv = f["center"][0] * v[0] + f["center"][1] * v[1]
    h = f["side_m"] / 2.0
    bars = {"a": (cu - h, cv, 0.0), "b": (cu + h, cv, 0.0), "c": (cu, cv - h, 90.0), "d": (cu, cv + h, 90.0)}
    for key, (bu, bv, turn) in bars.items():
        c = at(bu, bv, zf)
        size = (f["bar_m"], f["side_m"], f["bar_m"]) if turn == 0.0 else (f["side_m"], f["bar_m"], f["bar_m"])
        _box(stage, "%s/LEDFrame/bar_%s" % (ROOT_PATH, key), c, size, YAW_DEG, alu)
        lt = UsdLux.RectLight.Define(stage, Sdf.Path("%s/LEDFrame/light_%s" % (ROOT_PATH, key)))
        lt.CreateWidthAttr(size[0])
        lt.CreateHeightAttr(size[1])
        lt.CreateColorAttr(Gf.Vec3f(*COOL))
        lt.CreateIntensityAttr(90000.0)
        x = UsdGeom.Xformable(lt)
        x.AddTranslateOp().Set(Gf.Vec3d(c[0], c[1], c[2] - f["bar_m"] / 2 - 0.005))
        x.AddRotateZOp().Set(YAW_DEG)                   # a rect light shines down its -Z: already down
    # the warehouse beyond the glass: a warm fill from everywhere
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path(ROOT_PATH + "/Warehouse"))
    dome.CreateIntensityAttr(260.0)
    dome.CreateColorAttr(Gf.Vec3f(*WARM))
    return ["the LED frame at %s, %.1f m square, %.2f m under the ceiling: estimated from the photo"
            % (f["center"], f["side_m"], f["below_ceiling_m"]),
            "joists every %.2f m, beams every %.1f m: estimated" % (JOIST["spacing_m"], BEAM["spacing_m"]),
            "glass walls with studs: %s" % ", ".join(w[0] for w in glass)]


def _bound(stage, room, wall):
    """The material a wall's face is bound to (its name), or None."""
    face = stage.GetPrimAtPath("%s/Structure/%s/face" % (room, wall))
    if not face:
        return None
    mat, _ = UsdShade.MaterialBindingAPI(face).ComputeBoundMaterial()
    return mat.GetPrim().GetName() if mat else None
