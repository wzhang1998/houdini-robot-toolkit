"""The simulated cell as one USD file, the arm posed: to share (usdview,
Houdini Solaris, Omniverse, Isaac Sim) without the repo or the simulator.

    C:/isaacsim6/python.bat scripts/isaac/export_pose_usd.py --pose scan_start
    C:/isaacsim6/python.bat scripts/isaac/export_pose_usd.py --pose rest --out geo/share/fr20_rest.usda
    C:/isaacsim6/python.bat scripts/isaac/export_pose_usd.py --joints "0 -90 90 -90 -90 0"

The stage is built as the simulation builds it (isaac_stage.py): the show's
layer (shows/<show>.usda: the room, the stage, the paper and the frame's
rails), the FR20 from its URDF, the mounted tool (the LED strip). The arm is
posed by the toolkit's own kinematics (urdf_rig, the one the collision
checks use): each link's transform written to its prim, so the file shows
the pose with no simulation running. --pose is a hub of the compiled show
(scan_start: the scan's first frame, the strip at the paper's left edge;
scan_end, rest, greet, low, high). Everything is flattened into the one
file (the room, the robot's meshes); a camera, /World/ShareCam, looks at
the arm from inside the room.
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=os.path.join(ROOT, "shows", "party.json"))
ap.add_argument("--pose", default="scan_start", help="a hub of the compiled show")
ap.add_argument("--joints", default="", help="six joint angles (deg) instead of --pose")
ap.add_argument("--out", default="")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux  # noqa: E402

import collision as CL  # noqa: E402
import robot_profile as RP  # noqa: E402
import show  # noqa: E402
import urdf_rig as U  # noqa: E402
from isaac_stage import attach_tool, import_robot, show_camera  # noqa: E402


def matrix(R, p):
    """Gf.Matrix4d (row vectors, translation in the last row) of rotation R, position p."""
    return Gf.Matrix4d(R[0][0], R[1][0], R[2][0], 0.0,
                       R[0][1], R[1][1], R[2][1], 0.0,
                       R[0][2], R[1][2], R[2][2], 0.0,
                       p[0], p[1], p[2], 1.0)


def main():
    cfg_path = os.path.abspath(args.config)
    cfg = json.load(open(cfg_path))
    if args.joints:
        q, label = [float(x) for x in args.joints.split()], "joints"
    else:
        graph = show.Graph.load(show.compiled_path(cfg_path))
        if args.pose not in graph.hubs:
            raise SystemExit("no hub %r in the compiled show: %s" % (args.pose, ", ".join(graph.hubs)))
        q, label = graph.hubs[args.pose], args.pose
    out = os.path.abspath(args.out or os.path.join(ROOT, "geo", "share", "fr20_%s_%s.usda" % (cfg.get("name", "show"), label)))
    os.makedirs(os.path.dirname(out), exist_ok=True)

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    add_reference_to_stage(os.path.splitext(cfg_path)[0] + ".usda", "/World/Room")
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(600)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Key"))
    key.CreateIntensityAttr(2500)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 30.0))
    root = import_robot()
    attach_tool(stage)

    # the pose: every link's world transform from our kinematics, written as
    # its local transform under whatever parent the importer gave it
    prof = RP.load("fr20")
    chain = U.parse_urdf(os.path.join(ROOT, prof["rig"]["urdf"]))["chain"]
    fk = U.forward_kinematics(chain, q)
    world_of = {chain[0]["parent"]: matrix(U.IDENTITY, (0.0, 0.0, 0.0))}
    for j, f in zip(chain, fk):
        world_of[j["child"]] = matrix(f["link_R"], f["link_p"])
    robot_prims = [p for p in stage.Traverse() if p.GetPath().pathString.startswith("/World/fr20")]
    # the links are nested along the chain (Geometry/base_link/shoulder_link/...), each with a child of
    # its own name holding its visual mesh (and a *_1 one its collision mesh, purpose guide): the link
    # itself is the prim on the chain's path -- the first of that name found was the visual child,
    # which left the links, and so the collision meshes, at zero (a second arm on the floor, 2026-09-29)
    top = next(p for p in robot_prims if p.GetName() == chain[0]["parent"])
    link_path = {chain[0]["parent"]: top.GetPath()}
    for j in chain:
        link_path[j["child"]] = link_path[j["parent"]].AppendChild(j["child"])
    posed = []
    for name, M in world_of.items():
        prim = stage.GetPrimAtPath(link_path[name])
        if not prim.IsValid():
            continue
        parent = UsdGeom.Xformable(prim.GetParent())
        pw = parent.ComputeLocalToWorldTransform(Usd.TimeCode.Default()) if prim.GetParent().IsA(UsdGeom.Xformable) \
            else Gf.Matrix4d(1.0)
        x = UsdGeom.Xformable(prim)
        x.ClearXformOpOrder()
        x.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(M * pw.GetInverse())
        posed.append(name)
    missing = [n for n in world_of if n not in posed]
    if missing:
        raise SystemExit("links not found in the robot's USD: %s" % ", ".join(missing))

    # check: the tool's tip where our kinematics puts it
    tool = CL.tool_def(prof)
    last = next(p for p in robot_prims if p.GetName() == chain[-1]["child"])
    lw = UsdGeom.Xformable(last).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    diff = max(abs(lw[3][i] - fk[-1]["link_p"][i]) for i in range(3))

    cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/ShareCam"))
    cam.CreateFocalLengthAttr(13.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    eye, target = show_camera(CL.load_env(os.path.join(ROOT, cfg["env"])), cfg)
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(view)
    world.GetPrim().SetCustomDataByKey("robot_show", {
        "config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"), "pose": label,
        "joints_deg": [round(x, 3) for x in q], "tool": tool["name"] if tool else "none",
        "note": "exported by houdini-robot-toolkit scripts/isaac/export_pose_usd.py"})

    # no instancing in the file: flattened, an instanced mesh becomes an "over" prototype at the root
    # (/Flattened_Prototype_N), which USD does not draw but some viewers do -- the FR20's meshes are
    # modelled in the zero pose, so they drew a second arm on the floor (2026-09-29)
    for _ in range(10):                                       # instances nest (a material inside a mesh's)
        found = [p for p in stage.Traverse() if p.IsInstanceable()]
        if not found:
            break
        for p in found:
            p.SetInstanceable(False)
    flat = stage.Flatten()                                    # an Sdf.Layer
    edit = Sdf.BatchNamespaceEdit()
    for spec in list(flat.rootPrims):                         # only /World: not the app's cameras, render settings
        if spec.name != "World":
            edit.Add(spec.path, Sdf.Path.emptyPath)
    if not flat.Apply(edit):
        raise SystemExit("could not remove the extra root prims")
    flat.Export(out)
    size = os.path.getsize(out) / 1e6
    print("[export] %s  (%s: %s)  %.1f MB, links posed %d, last link %.1e m from our FK"
          % (out, label, " ".join("%.1f" % x for x in q), size, len(posed), diff))


main()
app.close()
