"""The room, the FR20 and its tool in an Isaac Sim stage, the cameras and the
render settings: shared by every Isaac script (run_show, run_tracking,
record_library, replay_render, export_pose_usd), so they all show the one room, lit as
the lab (load_room). Import after SimulationApp has started (it needs omni)."""

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


def load_room(stage, cfg_path, look="room", guides=False, log=print):
    """The show's room at /World/Room (shows/<show>.usda: the room with the
    show's paper and stage), lit and coloured as the lab (room_look.py) --
    the one look of every Isaac view of the project (the user, 2026-09-29);
    look "plain": the old flat grey room under a dome and a key. The room's
    collision cell (collision.load_env) returned."""
    import json
    import collision as CL
    room_usd = os.path.splitext(cfg_path)[0] + ".usda"
    if not os.path.exists(room_usd):
        raise SystemExit("no %s: write it first (python scripts/room_usd.py --show %s)"
                         % (room_usd, os.path.relpath(cfg_path, ROOT)))
    add_reference_to_stage(room_usd, "/World/Room")
    env = CL.load_env(os.path.join(ROOT, json.load(open(cfg_path))["env"]))
    if look == "room":
        import room_look
        for note in room_look.apply(stage, env, guides=guides):
            log("[stage] look: %s" % note)
    else:
        UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(600)
        key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Key"))
        key.CreateIntensityAttr(2500)
        key.CreateAngleAttr(8.0)
        UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 30.0))
    return env


def contact_paths(h):
    """The two actors of a PhysX contact report header as prim paths. Isaac
    Sim 6 hands them over as encoded ints (PhysicsSchemaTools): str() of one
    is a number, so a test like "/Room" in str(h.actor0) never matched and
    every contact count read 0 until 2026-09-29."""
    from pxr import PhysicsSchemaTools
    return str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))


def render_settings():
    """Real-time ray tracing; the walls' faces culled from outside (they are
    single faces turned into the room: a camera outside sees through); the
    glass's opacity below 1 drawn see-through."""
    import carb.settings
    st = carb.settings.get_settings()
    st.set("/rtx/rendermode", "RaytracedLighting")
    st.set("/rtx/hydra/faceCulling/enabled", True)
    st.set("/rtx/raytracing/fractionalCutoutOpacity", True)


def cameras(cfg, env):
    """{name: (eye, target, focal mm)}: interact -- the interactive mode, the guest on the left and the whole
    room: 2.0 m behind the audience zone, 0.9 m to its right, at eye height (the user, 2026-09-30); room -- the whole room from the corner
    behind the robot (the audience's wall and the back wall: where the
    user's photo was taken, 2026-09-29); audience -- the guests' side, from
    1.5 m behind the audience zone through the glass; side -- along the
    paper, the gap between the strip and it."""
    import room_geom as RG
    import show
    a = cfg["zones"]["audience"]
    n = cfg["canvas"]["normal"]
    middle = show.scan_area(cfg)[0] if cfg.get("scan") else list(cfg["canvas"]["center"])
    right = (n[1], -n[0], 0.0)
    fp = RG.footprint(env)
    cx, cy = sum(p[0] for p in fp) / len(fp), sum(p[1] for p in fp) / len(fp)
    corner = max(fp, key=lambda p: -n[0] * p[0] - n[1] * p[1] + right[0] * p[0] + right[1] * p[1])
    k = 0.25 / math.hypot(cx - corner[0], cy - corner[1])
    eye_room = [corner[0] + (cx - corner[0]) * k, corner[1] + (cy - corner[1]) * k, 2.0]
    eye_aud = [a["center"][0] - n[0] * 1.5, a["center"][1] - n[1] * 1.5, 1.65]
    eye_side = [middle[0] + right[0] * 1.5 - n[0] * 1.4, middle[1] + right[1] * 1.5 - n[1] * 1.4, 1.8]
    yaw = math.radians(a.get("yaw_deg", 0.0))
    across, along = (-math.sin(yaw), math.cos(yaw)), (math.cos(yaw), math.sin(yaw))
    g = cfg["hubs"]["greet"]["tcp"]
    eye_int = [a["center"][0] - across[0] * 2.0 + along[0] * 0.9, a["center"][1] - across[1] * 2.0 + along[1] * 0.9,
               1.45]
    return {"interact": (eye_int, [g[0] - 0.13, g[1] + 0.21, 1.15], 12.0),
            "room": (eye_room, [cx, cy, 0.7], 8.0),
            "audience": (eye_aud, [(cx + middle[0]) / 2, (cy + middle[1]) / 2, 1.0], 13.0),
            "side": (eye_side, [middle[0] - n[0] * 0.4, middle[1] - n[1] * 0.4, middle[2]], 14.0)}


def camera_spec(spec, cfg, env):
    """(eye, target, focal mm) of a --camera: a name of cameras(), or
    'ex ey ez tx ty tz [focal]' in the robot frame."""
    views = cameras(cfg, env)
    if spec in views:
        return views[spec]
    v = [float(x) for x in spec.split()]
    if len(v) not in (6, 7):
        raise SystemExit("--camera %r: one of %s, or 'ex ey ez tx ty tz [focal]'" % (spec, ", ".join(views)))
    return v[:3], v[3:6], v[6] if len(v) == 7 else 13.0


def use_camera(stage, path, eye, target, focal):
    """A camera at eye looking at target, made the viewport's (when there is one)."""
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport
    cam = UsdGeom.Camera.Define(stage, Sdf.Path(path))
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    set_camera_view(eye=eye, target=target, camera_prim_path=path)
    vp = get_active_viewport()
    if vp is not None:
        vp.camera_path = path
    return vp


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
        # contact reports are asked of a rigid body (the links); their colliders sit in instances,
        # which Traverse does not enter -- on those alone no link ever reported (found 2026-09-29)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.CollisionAPI):
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


def encode_video(app, frames, corner, shots, mp4, fps=30, size=(1600, 900), tag="show"):
    """frames/f_%05d.png as mp4 with the corner's text burnt in (overlay.py:
    corner [(t, text)]); its path relative to the repo, None if ffmpeg fails
    (the frames are kept then; else removed)."""
    import shutil
    import subprocess
    import overlay
    for _ in range(60):
        app.update()                                        # the last captures written
    with open(os.path.join(frames, "overlay.ass"), "w", encoding="utf-8") as f:
        f.write(overlay.ass(overlay.compress(corner, 0.5), size[0], size[1], shots / float(fps)))
    r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", "f_%05d.png",
                        "-vf", "ass=overlay.ass", "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "22",
                        os.path.abspath(mp4)], cwd=frames, capture_output=True, text=True)
    if r.returncode:
        print("[%s] ffmpeg: %s (frames kept in %s)" % (tag, r.stderr[-400:], frames))
        return None
    shutil.rmtree(frames, ignore_errors=True)
    print("[%s] video %s" % (tag, mp4))
    return os.path.relpath(mp4, ROOT).replace("\\", "/")
