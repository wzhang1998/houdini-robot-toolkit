"""The FR20 and its tool in an Isaac Sim stage, shared by run_show.py and
run_tracking.py. Import after SimulationApp has started (it needs omni)."""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402,F401

PHYSICS_DT = 1.0 / 120.0


def show_camera(env, cfg):
    """(eye, target): the room's corner farthest from the operator (keep-out
    zones), 0.35 m in from the walls and just under the ceiling, looking at
    the stage's centre -- a view of the whole working area from inside."""
    import room_geom as RG
    fp = RG.footprint(env)
    z0, z1 = RG.heights(env)
    cx, cy = sum(p[0] for p in fp) / len(fp), sum(p[1] for p in fp) / len(fp)
    people = [o["center"] for o in env["objects"] if o["role"] == "keep_out"] or [(cx, cy)]
    corner = max(fp, key=lambda p: min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in people))
    d = math.hypot(cx - corner[0], cy - corner[1])
    k = min(1.0, 0.9 / d)                                     # ~0.6 m off both walls (clear of a plant in the corner)
    eye = [corner[0] + (cx - corner[0]) * k, corner[1] + (cy - corner[1]) * k, z1 - 0.2]
    st = cfg.get("stage") or next((o for o in env["objects"] if o["name"] == "stage"), None)
    target = [st["center"][0] / 2, st["center"][1] / 2, 0.7] if st else [cx, cy, 0.8]   # between the base and the stage
    return eye, target
URDF = os.path.join(ROOT, "assets", "fairino_description", "urdf", "fairino20_v6.urdf")
PKG = os.path.join(ROOT, "assets", "fairino_description")
STIFFNESS = 1.0e5      # Nm/rad: stiff position drives, so tracking shows the dynamics, not a soft spring
DAMPING = 1.0e3        # Nm*s/rad
ROBOT_USD_DIR = os.path.join(ROOT, "geo", "isaac", "fr20_usd")


def robot_usd():
    """The FR20 as USD, converted from the vendor URDF by Isaac Sim 6's
    importer (URDF stays the source); converted again when the URDF is newer."""
    import omni.kit.app
    em = omni.kit.app.get_app().get_extension_manager()
    for ext in ("omni.scene.optimizer.core", "isaacsim.robot.schema"):
        em.set_extension_enabled_immediate(ext, True)
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    done = os.path.join(ROBOT_USD_DIR, "converted.txt")
    if os.path.exists(done) and os.path.getmtime(done) > os.path.getmtime(URDF):
        return open(done).read().strip()
    os.makedirs(ROBOT_USD_DIR, exist_ok=True)
    cfg = URDFImporterConfig()
    cfg.urdf_path = URDF
    cfg.usd_path = ROBOT_USD_DIR
    cfg.ros_package_paths = [{"name": "fairino_description", "path": PKG.replace("\\", "/")}]
    cfg.fix_base = True
    cfg.merge_fixed_joints = False
    cfg.joint_target_type = "position"
    cfg.override_joint_stiffness = STIFFNESS
    cfg.override_joint_damping = DAMPING
    usd = URDFImporter(cfg).import_urdf()
    open(done, "w").write(usd)
    print("[show] converted the URDF to %s" % usd)
    return usd


def import_robot():
    """Reference the robot USD into the stage; the articulation root's path."""
    usd = robot_usd()
    add_reference_to_stage(usd, "/World/fr20")
    stage = omni.usd.get_context().get_stage()
    root = None
    for prim in stage.Traverse():
        if not prim.GetPath().pathString.startswith("/World/fr20"):
            continue
        if root is None and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            root = prim.GetPath().pathString
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
    if root is None:
        raise SystemExit("no articulation root under /World/fr20 in %s" % usd)
    return root


def attach_tool(stage, robot_root="/World/fr20", profile="fr20"):
    """The profile's mounted tool (its URDF, read by tool_urdf as the
    collision checks read it) as colliders under the last link: in USD Physics
    a collider under a rigid body is part of that body, so the tool moves,
    weighs and touches with the flange. None when the profile has no tool."""
    import collision as CL
    import robot_profile as RP
    prof = RP.load(profile)
    tool = CL.tool_def(prof)
    if not tool:
        return None
    last = CL.U.parse_urdf(os.path.join(ROOT, prof["rig"]["urdf"]))["chain"][-1]["child"]
    link = next((p for p in stage.Traverse()
                 if p.GetName() == last and p.GetPath().pathString.startswith(robot_root)), None)
    if link is None:
        raise SystemExit("no %s under %s: cannot mount the tool" % (last, robot_root))
    fo = float(prof["rig"].get("flange_offset_m", 0.0))           # the mount is the flange's face
    base = link.GetPath().AppendChild(tool["name"])
    UsdGeom.Xform.Define(stage, base)
    for b in tool["boxes"]:
        cube = UsdGeom.Cube.Define(stage, base.AppendChild(b["name"]))
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.95, 0.9)])
        R = b["R"]
        rot = Gf.Matrix3d(*[R[j][i] for i in range(3) for j in range(3)]).ExtractRotation()   # Gf is row-vector
        cube.AddTranslateOp().Set(Gf.Vec3d(b["xyz"][0], b["xyz"][1], b["xyz"][2] + fo))
        cube.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(rot.GetQuat()))
        cube.AddScaleOp().Set(Gf.Vec3d(*b["size"]))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        PhysxSchema.PhysxContactReportAPI.Apply(cube.GetPrim()).CreateThresholdAttr().Set(0.0)
    print("[show] tool %s on %s: %s" % (tool["name"], link.GetPath(), ", ".join(b["name"] for b in tool["boxes"])))
    return base.pathString
