"""The room (envs/*.json, collision.py's cell) as a USD stage, for Isaac Sim
and anything else that reads OpenUSD.

    <python with pxr> scripts/env_to_usd.py envs/volvox_lab.json envs/volvox_lab.usda [--show shows/party.json]

Run it with hython (Houdini's pxr). Isaac Sim's python.bat has pxr only
inside a SimulationApp; the system Python usually has none.

Robot base frame, Z up, metres (Isaac's convention too: metersPerUnit 1,
upAxis Z). One prim per object under /Room, named as in the env:

    obstacle     a Cube / Cylinder with UsdPhysics.CollisionAPI, grey
    keep_out, slow, work   visual only (no collision), see-through, coloured
    halfspace    a slab 0.1 m thick behind the plane, 8 m across

Every prim carries motionlab:role and motionlab:margin_m, so the stage says
what collision.py checks. --show adds the show's canvas (the paper) as an
obstacle. The JSON stays the source for now; this is derived from it.
"""

import json
import math
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROLE_RGB = {"obstacle": (0.55, 0.56, 0.58), "keep_out": (0.9, 0.2, 0.15), "slow": (1.0, 0.65, 0.1),
            "work": (0.2, 0.85, 0.35)}
SLAB = 0.1
SPAN = 8.0


def _material(stage, role):
    path = "/Room/Looks/" + role
    if stage.GetPrimAtPath(path):
        return UsdShade.Material.Get(stage, path)
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*ROLE_RGB[role]))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0 if role == "obstacle" else 0.15)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _xform(prim, t, rot=None, scale=None):
    x = UsdGeom.Xformable(prim)
    x.AddTranslateOp().Set(Gf.Vec3d(*t))
    if rot is not None:
        x.AddOrientOp().Set(Gf.Quatf(rot.GetQuat()))
    if scale is not None:
        x.AddScaleOp().Set(Gf.Vec3f(*scale))


def add_object(stage, o, default_margin):
    name = o["name"].replace(" ", "_").replace("-", "_")
    path = "/Room/" + name
    role = o["role"]
    t = o["type"]
    if t in ("box", "halfspace"):
        prim = UsdGeom.Cube.Define(stage, path)
        prim.CreateSizeAttr(1.0)
        if t == "box":
            rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), o.get("yaw_deg", 0.0))
            _xform(prim, o["center"], rot, o["size"])
        else:
            n = Gf.Vec3d(*o["normal"]).GetNormalized()
            centre = n * (o["offset"] - SLAB / 2.0)
            rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), n)
            _xform(prim, centre, rot, (SLAB, SPAN, SPAN))
    elif t == "cylinder":
        prim = UsdGeom.Cylinder.Define(stage, path)
        prim.CreateAxisAttr("Z")
        prim.CreateRadiusAttr(o["radius"])
        prim.CreateHeightAttr(o["height"])
        c = o["center"]
        _xform(prim, (c[0], c[1], c[2] + o["height"] / 2.0))
    elif t == "sphere":
        prim = UsdGeom.Sphere.Define(stage, path)
        prim.CreateRadiusAttr(o["radius"])
        _xform(prim, o["center"])
    else:
        raise ValueError("unknown shape %r" % t)
    p = prim.GetPrim()
    p.CreateAttribute("motionlab:role", Sdf.ValueTypeNames.String).Set(role)
    p.CreateAttribute("motionlab:margin_m", Sdf.ValueTypeNames.Float).Set(float(o.get("margin_m", default_margin)))
    if o.get("note"):
        p.SetDocumentation(o["note"])
    UsdShade.MaterialBindingAPI.Apply(p).Bind(_material(stage, role))
    if role == "obstacle":
        UsdPhysics.CollisionAPI.Apply(p)
    return p


def export(env, out, extra=()):
    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Room")
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetDocumentation("%s -- %s" % (env.get("name", ""), env.get("frame", "")))
    margin = float(env.get("margin_m", 0.05))
    for o in list(env["objects"]) + list(extra):
        add_object(stage, o, margin)
    stage.GetRootLayer().Save()
    return stage


if __name__ == "__main__":
    args = sys.argv[1:]
    env = json.load(open(args[0]))
    extra = []
    if "--show" in args:
        import os
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        cfg = json.load(open(args[args.index("--show") + 1]))
        c = cfg["canvas"]
        n = c["normal"]
        extra.append({"name": "canvas", "type": "box", "center": c["center"], "role": "obstacle",
                      "size": [c.get("thickness", 0.02), c["size"][0], c["size"][1]],
                      "yaw_deg": math.degrees(math.atan2(n[1], n[0])), "note": "the show's paper (placeholder)"})
    export(env, args[1], extra)
    print("wrote", args[1], len(env["objects"]) + len(extra), "objects")
