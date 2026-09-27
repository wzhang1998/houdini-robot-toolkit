"""The room (envs/*.json, collision.py's cell) as a USD stage, for Isaac Sim
and anything else that reads OpenUSD.

    hython scripts/env_to_usd.py envs/volvox_lab.json envs/volvox_lab.usda [--show shows/party.json]

Run it with hython (Houdini's pxr). Isaac Sim's python.bat has pxr only
inside a SimulationApp; the system Python usually has none.

Robot base frame, Z up, metres (Isaac's convention too). The room is built
closed and finite (room_geom.py), as a set would be, not as the infinite
planes the collision check uses:

    /Room/Structure   floor slab over the room's footprint (wood); per
                      measured wall a single-sided face turned into the room
                      (a cutaway: hidden from outside; plaster, or pale
                      blue-grey for glass) and an invisible collision slab;
                      the ceiling grid's height as an outline
    /Room/Objects     obstacles as solids with plausible materials (the red
                      control cart, the TV, shelves, the plywood base plate,
                      the paper) and collision
    /Room/Zones       work / keep-out / slow zones as outlines only
                      (BasisCurves), coloured by role -- they are
                      volumes the checks use, not things in the room

Colours and the wall / zone drawing rules are room_geom's, shared with the
Houdini cell display (cell_sop.py), so both look alike.

Every prim carries motionlab:role and motionlab:margin_m. --show adds the
show's paper (canvas) and its stage. The JSON stays the source; this is
derived from it.
"""

import json
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import room_geom as RG  # noqa: E402

WALL_T = 0.08
SOLID_WALLS, LOOKS, ZONE_RGB = RG.SOLID_WALLS, RG.LOOKS, RG.ZONE_RGB   # shared with Houdini (cell_sop)
FLOOR_T = 0.05


def _material(stage, key):
    path = "/Room/Looks/" + key
    if stage.GetPrimAtPath(path):
        return UsdShade.Material.Get(stage, path)
    rgb, rough, metal, opacity = LOOKS[key]
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


def _tag(prim, role, margin, doc=None, look=None, collide=False):
    p = prim.GetPrim()
    p.CreateAttribute("motionlab:role", Sdf.ValueTypeNames.String).Set(role)
    p.CreateAttribute("motionlab:margin_m", Sdf.ValueTypeNames.Float).Set(float(margin))
    if doc:
        p.SetDocumentation(doc)
    if look:
        UsdShade.MaterialBindingAPI.Apply(p).Bind(_material(prim.GetPrim().GetStage(), look))
    if collide:
        UsdPhysics.CollisionAPI.Apply(p)


def _mesh(stage, path, points, faces):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    return m


def _prism(stage, path, ring, dz0, dz1):
    """A slab: the polygon ring (x, y) from z = dz0 to dz1 (closed mesh)."""
    n = len(ring)
    pts = [(x, y, dz0) for x, y in ring] + [(x, y, dz1) for x, y in ring]
    faces = [list(range(n))[::-1], list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    return _mesh(stage, path, pts, faces)


def _panel(stage, path, corners, outward, t):
    """A wall: the 4 corners on the room side, extruded t outwards."""
    o = [c for c in corners]
    b = [(c[0] + outward[0] * t, c[1] + outward[1] * t, c[2]) for c in corners]
    pts = o + b
    faces = [[0, 1, 2, 3], [7, 6, 5, 4], [0, 4, 5, 1], [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]
    return _mesh(stage, path, pts, faces)


def _inward_face(stage, path, corners, outward):
    """One single-sided quad on the wall's room side, its normal into the room
    (right-handed winding, reversed when needed); culled from outside."""
    a, b, c = corners[0], corners[1], corners[2]
    e1 = [b[i] - a[i] for i in range(3)]
    e2 = [c[i] - a[i] for i in range(3)]
    n = (e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0])
    order = [0, 1, 2, 3] if sum(n[i] * -outward[i] for i in range(3)) > 0 else [3, 2, 1, 0]
    m = _mesh(stage, path, [corners[i] for i in order], [[0, 1, 2, 3]])
    m.CreateDoubleSidedAttr(False)
    # Omniverse RTX culls a face by its own "singleSided" attribute (the
    # "Single Sided" toggle in Isaac's property panel), not by doubleSided
    m.GetPrim().CreateAttribute("singleSided", Sdf.ValueTypeNames.Bool).Set(True)
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


def export(env, out, extra=()):
    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Room")
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetDocumentation("%s -- %s" % (env.get("name", ""), env.get("frame", "")))
    for s in ("Structure", "Objects", "Zones", "Looks"):
        UsdGeom.Scope.Define(stage, "/Room/" + s)
    margin = float(env.get("margin_m", 0.05))
    by_name = {o["name"]: o for o in env["objects"]}

    # structure: floor, walls, ceiling outline
    fp = RG.footprint(env)
    z0, z1 = RG.heights(env)
    fl = by_name.get("floor", {"name": "floor"})
    floor = _prism(stage, "/Room/Structure/floor", fp, z0 - FLOOR_T, z0)
    _tag(floor, "obstacle", fl.get("margin_m", margin), fl.get("note"), "floor", True)
    for name, corners, outward in RG.walls(env):
        o = by_name[name]
        # a cutaway, as level editors and cell viewers show a room: the wall is
        # drawn as one single-sided face turned into the room, so it hides
        # itself when the camera is outside and reads as a wall from inside.
        # Collision is a separate slab behind it, never drawn.
        w = _panel(stage, "/Room/Structure/" + _safe(name), corners, outward, WALL_T)
        _tag(w, "obstacle", o.get("margin_m", margin), o.get("note"), None, True)
        UsdGeom.Imageable(w).MakeInvisible()
        face = _inward_face(stage, "/Room/Structure/" + _safe(name) + "_face", corners, outward)
        _tag(face, "obstacle", o.get("margin_m", margin), None, RG.wall_look(name))
    ring = [(x, y, z1) for x, y in fp]
    ce = _curves(stage, "/Room/Structure/ceiling_grid", [ring + ring[:1]], ZONE_RGB["ceiling"], 0.015)
    _tag(ce, "obstacle", by_name.get("ceiling", {}).get("margin_m", margin),
         "the ceiling grid's lowest point (%.2f m): drawn as its outline so the room can be seen from above" % z1)

    # objects and zones
    for o in list(env["objects"]) + list(extra):
        if o["type"] == "halfspace":
            continue
        name, role = _safe(o["name"]), o["role"]
        if role != "obstacle":                                  # a volume the checks use: outline only
            lines = RG.zone_lines(o, fp)                              # up to the walls, not through them
            if not lines:
                continue
            c = _curves(stage, "/Room/Zones/" + name, lines, ZONE_RGB.get(role, (1, 1, 1)))
            _tag(c, role, o.get("margin_m", margin), o.get("note"))
            continue
        path = "/Room/Objects/" + name
        if o["type"] == "box":
            lo, hi = RG.box_corners(o["center"], o["size"], o.get("yaw_deg", 0.0))
            prim = _mesh(stage, path, lo + hi, [[3, 2, 1, 0], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5],
                                                [2, 3, 7, 6], [3, 0, 4, 7]])
        elif o["type"] == "cylinder":
            prim = UsdGeom.Cylinder.Define(stage, path)
            prim.CreateAxisAttr("Z")
            prim.CreateRadiusAttr(o["radius"])
            prim.CreateHeightAttr(o["height"])
            c = o["center"]
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(c[0], c[1], c[2] + o["height"] / 2.0))
        elif o["type"] == "sphere":
            prim = UsdGeom.Sphere.Define(stage, path)
            prim.CreateRadiusAttr(o["radius"])
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*o["center"]))
        else:
            raise ValueError("unknown shape %r" % o["type"])
        _tag(prim, role, o.get("margin_m", margin), o.get("note"), RG.look_key(o["name"]), True)
    stage.GetRootLayer().Save()
    return stage


def show_extras(cfg):
    """The show's paper (an obstacle) and, when it overrides the env's, its stage."""
    out = []
    c = cfg.get("canvas")
    if c:
        n = c["normal"]
        out.append({"name": "canvas", "type": "box", "center": c["center"], "role": "obstacle",
                    "size": [c.get("thickness", 0.02), c["size"][0], c["size"][1]],
                    "yaw_deg": math.degrees(math.atan2(n[1], n[0])), "note": "the show's paper (placeholder)"})
    st = cfg.get("stage")
    if st:
        out.append({"name": "show_stage", "type": "box", "center": st["center"], "size": st["size"],
                    "yaw_deg": st.get("yaw_deg", 0.0), "role": "work", "note": "the show's stage"})
    return out


if __name__ == "__main__":
    args = sys.argv[1:]
    env = json.load(open(args[0]))
    extra = show_extras(json.load(open(args[args.index("--show") + 1]))) if "--show" in args else []
    export(env, args[1], extra)
    print("wrote", args[1], "room %.1f m2, %d walls, %d objects" % (
        0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(RG.footprint(env), RG.footprint(env)[1:] + RG.footprint(env)[:1]))),
        len(RG.walls(env)), len(env["objects"]) + len(extra)))
