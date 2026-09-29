"""The LED strip's cable in Isaac Sim: does it tangle, does it pull on its
two connectors (the strip's, the laptop's on the red cart)?

    uv run scripts/cable_route.py                                  the clip layouts (geometry)
    C:/isaacsim6/python.bat scripts/isaac/cable_sim.py --headless --video
    C:/isaacsim6/python.bat scripts/isaac/cable_sim.py --clips 3 --segments greet_05_trace,high_17_salute
    C:/isaacsim6/python.bat scripts/isaac/cable_sim.py --all --headless          every motion of the show
    C:/isaacsim6/python.bat scripts/isaac/cable_sim.py --headless --extra 0.2,0,0,0,0.5 --stills rest
                                                    the clips to take to the arm: the settled cable, numbered

The cable is a chain of capsules (PhysX rigid bodies, SEG_M long, along
their X) joined by D6 joints: stretch locked, bending on soft drives (the
cable's stiffness) inside a cone, twist about X limited -- PhysX's own rope (omni.physx.demos RigidBodyRopeDemo), the
joints kept out of the arm's articulation (excludeFromArticulation, else
PhysX solves the arm and the cable as one). It collides with the arm, the room and itself. Clipped to
the arm by fixed joints where cable_route.json's layout puts the clips,
each span as long as the survey says (the longest chord x 1.1 + 5 cm).
Its two ends are connectors: springs (CONNECTOR_N_PER_M) from the strip's
bracket and from the cart's top, so the pull on each is its stretch times
that stiffness, in newtons.

Played: the survey's worst motions (the ones at each span's extremes), the
big wipes and the scan, each reached by the show's own moves; --all: every
idle clip. Per motion: the largest pull on each connector, how long the
cable rubs each link, touches itself (a knot, a wrap), or two links at once
(a pinch). Writes geo/cable/cable_sim.json and, with --video, an mp4 (the
audience's side, the readings in the corner).
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=os.path.join(ROOT, "shows", "party.json"))
ap.add_argument("--shows", default="shows/party.json,shows/party_bigwipe.json", help="where the motions come from")
ap.add_argument("--route", default=os.path.join(ROOT, "geo", "cable", "cable_route.json"))
ap.add_argument("--clips", type=int, default=4, help="the survey's layout with this many clips")
ap.add_argument("--segments", default="", help="comma separated motions (default: the survey's worst)")
ap.add_argument("--all", action="store_true", help="every idle clip, the big wipes and the scan")
ap.add_argument("--diameter", type=float, default=0.006)
ap.add_argument("--extra", default="", help="cable added to each span, m, comma separated from the strip "
                                             "(a service loop: '0.2' gives the strip's span 20 cm more)")
ap.add_argument("--kg-per-m", type=float, default=0.08)
ap.add_argument("--camera", default="1.25 -1.0 1.75 0.0 0.35 0.85 12",
                help="a name of isaac_stage.cameras or 'ex ey ez tx ty tz [focal]'; default: close on the arm")
ap.add_argument("--video", action="store_true")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--debug", action="store_true", help="print the connectors while the cable settles")
ap.add_argument("--stills", default="", help="a hub: no motion -- the cable settled with the arm at that hub, "
                                             "pictures of it with its clips numbered (geo/cable/clips*.png)")
ap.add_argument("--out", default=os.path.join(ROOT, "geo", "cable"))
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720, "renderer": "RaytracedLighting"})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

import overlay  # noqa: E402
import show  # noqa: E402
from isaac_stage import (attach_tool, camera_spec, contact_paths, import_robot, load_room,  # noqa: E402
                         render_settings, use_camera)

DT = 1.0 / 240.0                  # a thin chain wants a finer step than the show's 1/120
SEG_M = 0.03
CONNECTOR_N_PER_M = 2000.0        # the end springs: 1 mm stretch = 2 N
PULL_N = 5.0                      # a connector pulled harder than this (over its pull at rest) is reported
BEND_R_MIN_M = 0.03               # a bend tighter than this (5 x a 6 mm cable's diameter) is reported
BEND_NM_PER_RAD = 0.02            # a 6 mm PVC cable's bending (EI ~ 6e-4 N m^2) over one segment
LEAD_M = 0.04                     # the cable straight out of the strip's plug
SETTLE_S = 3.0
HOLD_S = 0.5
FPS_VIDEO = 30
CABLE = "/World/Cable"


# ---------------------------------------------------------------- the cable's first shape
def sag_path(a, b, length, away, n):
    """n+1 points from a to b, the curve length long: an arc bulging along
    away (a unit vector) and down, the bulge found by bisection."""
    def pts(h):
        out = []
        for i in range(n + 1):
            t = i / n
            bump = 4 * t * (1 - t) * h
            out.append(tuple(a[k] + (b[k] - a[k]) * t + away[k] * bump for k in range(3)))
        return out

    def arc(p):
        return sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))
    lo, hi = 0.0, max(0.05, length)
    if arc(pts(0.0)) >= length:
        return pts(0.0)
    for _ in range(50):
        mid = (lo + hi) / 2
        if arc(pts(mid)) < length:
            lo = mid
        else:
            hi = mid
    return pts(hi)


def resample(p, step):
    """Points every step along a polyline."""
    out, carry = [p[0]], 0.0
    for a, b in zip(p, p[1:]):
        L = math.dist(a, b)
        s = step - carry
        while s <= L:
            out.append(tuple(a[k] + (b[k] - a[k]) * s / L for k in range(3)))
            s += step
        carry = L - (s - step)
    return out


def quat_x_to(d):
    """Gf.Quatf turning +X (a segment's axis: PhysX's D6 twists about X) onto unit d."""
    z = Gf.Vec3d(1, 0, 0)
    v = Gf.Vec3d(*d).GetNormalized()
    r = Gf.Rotation(z, v)
    q = r.GetQuat()
    return Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary()))


# ---------------------------------------------------------------- building it
def world_of(prim):
    """A prim's local-to-world matrix as authored in USD (Gf, row vectors):
    for the cable's segments while it is built. Not for anything the
    physics moves: Isaac keeps that in Fabric, USD goes stale."""
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def matrix(R, p):
    """Gf.Matrix4d (row vectors) of rotation R, position p -- a link's frame from our FK
    (the robot's USD link frames are the URDF's: export_pose_usd.py)."""
    return Gf.Matrix4d(R[0][0], R[1][0], R[2][0], 0.0, R[0][1], R[1][1], R[2][1], 0.0,
                       R[0][2], R[1][2], R[2][2], 0.0, p[0], p[1], p[2], 1.0)


def link_prims(stage):
    """{link name: the robot's rigid body prim of that name}."""
    out = {}
    for p in stage.Traverse():
        if p.GetPath().pathString.startswith("/World/fr20") and p.HasAPI(UsdPhysics.RigidBodyAPI):
            out.setdefault(p.GetName(), p)
    return out


def build_cable(stage, points, clip_at, start_body, link_mat, radius, kg_per_m):
    """Rigid capsules between consecutive points; D6 joints between them;
    fixed joints at the clips (clip_at: {point index: link prim}); the ends
    as spring connectors to start_body (the bracket's link) and the world.
    link_mat(prim): the link's world matrix now (from FK). Returns
    ([segment prims], {end: (anchor body or None, anchor local, segment index, segment end local)})."""
    UsdGeom.Scope.Define(stage, Sdf.Path(CABLE))
    look = UsdShade.Material.Define(stage, Sdf.Path(CABLE + "/Look"))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(CABLE + "/Look/Surface"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.45, 0.08))          # orange: seen against the white arm
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    look.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    segs = []
    for i, (a, b) in enumerate(zip(points, points[1:])):
        L = math.dist(a, b)
        path = Sdf.Path("%s/seg_%03d" % (CABLE, i))
        cap = UsdGeom.Capsule.Define(stage, path)
        cap.CreateAxisAttr("X")
        cap.CreateRadiusAttr(radius)
        cap.CreateHeightAttr(max(1e-3, L - 2 * radius))
        mid = [(a[k] + b[k]) / 2 for k in range(3)]
        x = UsdGeom.Xformable(cap)
        x.AddTranslateOp().Set(Gf.Vec3d(*mid))
        x.AddOrientOp().Set(quat_x_to([b[k] - a[k] for k in range(3)]))
        prim = cap.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(kg_per_m * L)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateEnableCCDAttr(True)
        rb.CreateSolverPositionIterationCountAttr(24)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(0.2)
        rb.CreateAngularDampingAttr(0.5)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(look)
        segs.append((prim, L))
    for i in range(len(segs) - 1):                     # bending joints
        (pa, La), (pb, Lb) = segs[i], segs[i + 1]
        j = UsdPhysics.Joint.Define(stage, Sdf.Path("%s/bend_%03d" % (CABLE, i)))
        j.CreateBody0Rel().SetTargets([pa.GetPath()])
        j.CreateBody1Rel().SetTargets([pb.GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(La / 2, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-Lb / 2, 0, 0))
        # the frames: body1's rest turn relative to body0 (the first shape is the rest shape of the twist)
        q0 = UsdGeom.Xformable(pa).GetOrderedXformOps()[1].Get()
        q1 = UsdGeom.Xformable(pb).GetOrderedXformOps()[1].Get()
        rel = (q0.GetInverse() * q1).GetNormalized()
        j.CreateLocalRot0Attr(Gf.Quatf(rel))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)       # the cable is no part of the arm's articulation
        p = j.GetPrim()
        for ax in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(p, ax)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)                   # low > high: locked
        tw = UsdPhysics.LimitAPI.Apply(p, "rotX")          # twist, about the segment's axis
        tw.CreateLowAttr(-15.0)
        tw.CreateHighAttr(15.0)
        for ax in ("rotY", "rotZ"):                         # bending: a cone (PhysX's rope demo)
            cone = UsdPhysics.LimitAPI.Apply(p, ax)
            cone.CreateLowAttr(-110.0)
            cone.CreateHighAttr(110.0)
            d = UsdPhysics.DriveAPI.Apply(p, ax)
            d.CreateTypeAttr("force")
            d.CreateTargetPositionAttr(0.0)
            d.CreateStiffnessAttr(BEND_NM_PER_RAD * math.pi / 180.0)     # USD angular drives: per degree
            d.CreateDampingAttr(0.002 * math.pi / 180.0)
    for k, (idx, link) in enumerate(sorted(clip_at.items())):     # the clips
        s = min(idx, len(segs) - 1)
        seg, L = segs[s]
        end = (-L / 2, 0, 0) if idx <= s else (L / 2, 0, 0)
        at = world_of(seg).Transform(Gf.Vec3d(*end))
        local = link_mat(link).GetInverse().Transform(at)
        j = UsdPhysics.FixedJoint.Define(stage, Sdf.Path("%s/clip_%d" % (CABLE, k)))
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateBody0Rel().SetTargets([link.GetPath()])
        j.CreateBody1Rel().SetTargets([seg.GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(*local))
        j.CreateLocalRot0Attr(Gf.Quatf(link_mat(link).ExtractRotationQuat().GetInverse()
                                       * world_of(seg).ExtractRotationQuat()))
        j.CreateLocalPos1Attr(Gf.Vec3f(*end))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    ends = {}
    for name, (s, end_local, body) in (("strip", (0, (-segs[0][1] / 2, 0, 0), start_body)),
                                       ("laptop", (len(segs) - 1, (segs[-1][1] / 2, 0, 0), None))):
        seg, L = segs[s]
        at = world_of(seg).Transform(Gf.Vec3d(*end_local))
        j = UsdPhysics.Joint.Define(stage, Sdf.Path("%s/connector_%s" % (CABLE, name)))
        j.CreateExcludeFromArticulationAttr(True)
        if body is not None:
            j.CreateBody0Rel().SetTargets([body.GetPath()])
            local = link_mat(body).GetInverse().Transform(at)
        else:
            local = at
        j.CreateBody1Rel().SetTargets([seg.GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(*local))
        j.CreateLocalPos1Attr(Gf.Vec3f(*end_local))
        p = j.GetPrim()
        for ax in ("transX", "transY", "transZ"):
            d = UsdPhysics.DriveAPI.Apply(p, ax)
            d.CreateTypeAttr("force")
            d.CreateTargetPositionAttr(0.0)
            d.CreateStiffnessAttr(CONNECTOR_N_PER_M)
            d.CreateDampingAttr(5.0)
        ends[name] = (body, tuple(local), s, end_local)
    return [s for s, _ in segs], ends


# ---------------------------------------------------------------- the motions
def playlist(graph, names, start_hub):
    """Segments to play: each named one reached by the show's moves from wherever the last ended."""
    by = {s.name: s for s in graph.segments}
    out, hub = [], start_hub
    for n in names:
        s = by.get(n)
        if s is None:
            continue
        if s.kind == "scan":
            chain = [by["to_scan"], s, by["from_scan"]]
        else:
            chain = [s]
        route = graph.route(hub, chain[0].start) or []
        out += route + chain
        hub = chain[-1].end
    return out


# ---------------------------------------------------------------- the clips as pictures
MARK_RGB = [(1.0, 0.85, 0.1), (0.1, 0.8, 1.0), (0.3, 1.0, 0.3), (1.0, 0.4, 0.9), (1.0, 0.5, 0.1), (0.6, 0.6, 1.0),
            (1.0, 1.0, 1.0)]
APERTURE_MM = 20.955                  # USD's default horizontal aperture
STILL_W, STILL_H = 1280, 720
JOINT_WORDS = {"J1": "Base (J1)", "J2": "Shoulder (J2)", "J3": "Elbow (J3)", "J4": "Wrist 1 (J4)",
               "J5": "Wrist 2 (J5)", "J6": "Wrist 3 (J6)"}


def project(p, eye, target, focal):
    """Pixel (x, y) of world point p in a camera at eye looking at target (Z up)."""
    f = [target[i] - eye[i] for i in range(3)]
    n = math.sqrt(sum(x * x for x in f))
    f = [x / n for x in f]
    r = [f[1], -f[0], 0.0]                                    # forward x up
    n = math.sqrt(sum(x * x for x in r))
    r = [x / n for x in r]
    u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]]
    d = [p[i] - eye[i] for i in range(3)]
    zc = sum(d[i] * f[i] for i in range(3))
    k = focal / APERTURE_MM * STILL_W
    return (STILL_W / 2 + sum(d[i] * r[i] for i in range(3)) / zc * k,
            STILL_H / 2 - sum(d[i] * u[i] for i in range(3)) / zc * k)


def clip_words(pid):
    if pid == "cart":
        return "Laptop on the red cart"
    if pid.startswith("tool/"):
        return "Strip plug (bracket side, out 4 cm)"
    link, where = pid.split("/")
    if "_axis" in where:
        j, side = where.split("_axis")
        return "%s joint cap, %s side" % (JOINT_WORDS[j], side)
    return {"shoulder_link": "Shoulder housing, side", "upperarm_link": "Upper arm",
            "forearm_link": "Forearm"}.get(link, link)


def stills(stage, robot, idx, q, world, layout, pts):
    """The settled cable and its clips from two sides, numbered, a legend below: geo/cable/clips.png."""
    from omni.kit.viewport.utility import capture_viewport_to_file
    from PIL import Image, ImageDraw, ImageFont
    for i, p in enumerate(pts):
        m = UsdGeom.Sphere.Define(stage, Sdf.Path("/World/ClipMarks/c%d" % i))
        m.CreateRadiusAttr(0.022)
        m.CreateDisplayColorAttr([Gf.Vec3f(*MARK_RGB[i % len(MARK_RGB)])])
        m.AddTranslateOp().Set(Gf.Vec3d(*p))
    # the whole arm in view: aimed between the base and the clips, from two corners, wide
    xs = [p[0] for p in pts[:-1]] + [0.0]
    ys = [p[1] for p in pts[:-1]] + [0.0]
    zs = [p[2] for p in pts[:-1]] + [0.0]
    mid = [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2]
    views = [([1.25, -1.05, 1.95], mid, 8.5), ([-1.5, -0.85, 1.95], mid, 8.5)]
    font = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 22)
    small = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 18)
    shots = []
    for v, (eye, target, focal) in enumerate(views):
        vp = use_camera(stage, "/World/ClipCam_%d" % v, eye, target, focal)
        for _ in range(int(1.0 / DT)):                       # the cable keeps hanging; the view's first frames
            robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=idx))
            world.step(render=True)
        png = os.path.join(args.out, "clips_%d.png" % v)
        capture_viewport_to_file(vp, png)
        for _ in range(30):
            app.update()
        img = Image.open(png).convert("RGB")
        d = ImageDraw.Draw(img)
        for i, p in enumerate(pts):
            x, y = project(p, eye, target, focal)
            if 0 <= x < STILL_W and 0 <= y < STILL_H:
                col = tuple(int(255 * c) for c in MARK_RGB[i % len(MARK_RGB)])
                d.ellipse([x - 15, y - 15, x + 15, y + 15], outline=col, width=3)
                d.rectangle([x + 15, y - 30, x + 42, y - 4], fill=(0, 0, 0))
                d.text((x + 22, y - 31), str(i + 1), fill=col, font=font)
        shots.append(img)
    more = [float(x) for x in args.extra.split(",") if x.strip()] if args.extra else []
    lens = [sp["cable_m"] + (more[i] if i < len(more) else 0.0) for i, sp in enumerate(layout["spans"])]
    both = Image.new("RGB", (STILL_W * 2, STILL_H + 220), (20, 20, 22))
    for i, img in enumerate(shots):
        both.paste(img, (STILL_W * i, 0))
    d = ImageDraw.Draw(both)
    y = STILL_H + 12
    d.text((20, y), "LED strip cable: clips and route (arm at the '%s' hub; two views; the cable as simulated)"
           % args.stills, fill=(255, 255, 255), font=font)
    y += 36
    for i, p in enumerate(layout["points"]):
        col = tuple(int(255 * c) for c in MARK_RGB[i % len(MARK_RGB)])
        d.text((20 + (i % 3) * 850, y + (i // 3) * 28), "%d  %s" % (i + 1, clip_words(p["id"])), fill=col,
               font=small)
    y += 66
    d.text((20, y), "Cable per span:   " + "     ".join(
        "%d-%d  %.2f m%s" % (i + 1, i + 2, L, (" (incl. %.0f cm service loop)" % (100 * more[i]))
                             if i < len(more) and more[i] else "") for i, L in enumerate(lens)),
        fill=(230, 230, 230), font=small)
    d.text((20, y + 28), "Total about %.1f m. The 1-2 loop lets J6 turn the strip without wrapping the cable tight "
                         "round the wrist; the last loop takes J1's big swing (the big wipes)." % sum(lens),
           fill=(230, 230, 230), font=small)
    d.text((20, y + 56), "Sleeve the J6 gap (wrist 2 / wrist 3) and the J1 gap (base / shoulder): the cable rests "
                         "across both. Tidy the spare loop on the base plate.", fill=(230, 230, 230), font=small)
    out = os.path.join(args.out, "clips.png")
    both.save(out)
    print("[cable] clips %s" % out, flush=True)


def main():
    cfg_path = os.path.abspath(args.config)
    cfg = json.load(open(cfg_path))
    route = json.load(open(args.route))
    layout = next((L for L in route["layouts"] if L["clips"] == args.clips), None)
    if layout is None:
        raise SystemExit("no %d-clip layout in %s (there: %s)" % (args.clips, args.route,
                                                                 [L["clips"] for L in route["layouts"]]))
    graphs = [show.Graph.load(show.compiled_path(os.path.join(ROOT, s))) for s in args.shows.split(",")]
    if args.segments:
        wanted = args.segments.split(",")
    elif args.all:
        wanted = [s.name for g in graphs for s in g.segments if s.kind in ("idle", "scan")]
    else:
        wanted = []
        for sp in layout["spans"]:
            wanted += [sp["longest_in"], sp["shortest_in"]] + [n for n, _ in sp.get("through_arm_in", [])[:2]]
        wanted += ["low_wipe_rows", "greet_wipe_cols", "scan"]
    seen, names = set(), []
    for n in wanted:
        if n not in seen:
            seen.add(n)
            names.append(n)
    plays = []
    for n in names:                                         # the show that has it: party first
        g = next((g for g in graphs if any(s.name == n for s in g.segments)), None)
        if g is not None:
            plays.append((n, g))
    hub = graphs[0].info.get("start_hub", "rest")
    seq = []
    for n, g in plays:
        got = playlist(g, [n], hub)
        if got:
            seq += [(x, n) for x in got]
            hub = got[-1].end
    if not seq:
        raise SystemExit("nothing to play: %s" % names)

    world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    env = load_room(stage, cfg_path, "room", guides=False)
    prim_path = import_robot()
    attach_tool(stage)
    robot = world.scene.add(SingleArticulation(prim_path, name="fr20"))
    scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene")) \
        if stage.GetPrimAtPath("/physicsScene") else None
    if scene:
        scene.CreateSolverTypeAttr("TGS")
    world.reset()
    dof = list(robot.dof_names)
    idx = np.array([dof.index("j%d" % i) for i in range(1, 7)])
    q0 = seq[0][0].q[0]
    if args.stills:                                          # the arm still at a hub
        q0 = graphs[0].hubs[args.stills]
    robot.set_joint_positions(np.radians(q0), joint_indices=idx)
    for _ in range(10):
        robot.apply_action(ArticulationAction(joint_positions=np.radians(q0), joint_indices=idx))
        world.step(render=False)

    # the clips' world points now (the robot at q0), from the robot's own prims
    import cable_route as CR
    import collision as C
    model = C.load_model("fr20")
    frames, _ = CR._link_frames(model, q0)
    links = link_prims(stage)
    k_of = {"tool": 5, "wrist2_link": 4, "wrist1_link": 3, "forearm_link": 2, "upperarm_link": 1, "shoulder_link": 0,
            "base_link": -1}
    prim_of = {"tool": links.get("wrist3_link")}
    prim_of.update({n: links.get(n) for n in k_of if n != "tool"})
    pts = []
    for p in layout["points"]:
        if p["id"] == "cart":
            pts.append(tuple(p["p_world_m"]))
        else:
            pts.append(CR._to_world(frames[k_of[p["link"]]], p["p_link_m"]))
    # the first shape: each span its cable long, bulging away from the arm and down
    caps = CR._world_caps(model, q0)
    # the connector leaves the bracket square to its side, LEAD_M, as a real plug does (curving straight
    # back, the first segments lay on the flange's housing, the spring held against it: 7 N at rest)
    side = layout["points"][0]["id"].rsplit("_", 1)[1]           # x+, x-, y+, y-
    R0 = frames[5][0]
    col = {"x": 0, "y": 1}[side[0]]
    out = [R0[r][col] * (1.0 if side[1] == "+" else -1.0) for r in range(3)]
    lead = tuple(pts[0][k] + out[k] * LEAD_M for k in range(3))
    poly, clip_idx = [pts[0], lead], {}
    starts = [lead] + pts[1:]
    for i, sp in enumerate(layout["spans"]):
        a, b = starts[i], pts[i + 1]
        mid = [(a[k] + b[k]) / 2 for k in range(3)]
        near = min(caps, key=lambda c: CR._seg_dist(mid, c["a"], c["b"]) - c["r"])
        ab = [near["b"][k] - near["a"][k] for k in range(3)]
        L2 = sum(x * x for x in ab) or 1.0
        t = max(0.0, min(1.0, sum((mid[k] - near["a"][k]) * ab[k] for k in range(3)) / L2))
        foot = [near["a"][k] + ab[k] * t for k in range(3)]
        away = [mid[k] - foot[k] for k in range(3)]
        away[2] -= 0.3 * math.sqrt(sum(x * x for x in away))                 # and a little down
        n = math.sqrt(sum(x * x for x in away)) or 1.0
        away = [x / n for x in away]
        more = [float(x) for x in args.extra.split(",") if x.strip()] if args.extra else []
        extra_m = more[i] if i < len(more) else 0.0
        part = sag_path(a, b, sp["cable_m"] + extra_m - (LEAD_M if i == 0 else 0.0), away, 60)
        poly += part[1:]
        clip_idx[i + 1] = len(poly) - 1
    points = resample(poly, SEG_M)
    points[-1] = poly[-1]
    # map each clip's polyline index to the nearest resampled point
    cum = [0.0]
    for a, b in zip(poly, poly[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    clip_at = {}
    for i, pi in clip_idx.items():
        if i == len(pts) - 1:
            continue                                        # the cart: the laptop's connector
        clip_at[int(round(cum[pi] / SEG_M))] = prim_of[layout["points"][i]["link"]]
    k_of_prim = {prim_of[n].GetPath(): k for n, k in k_of.items() if prim_of.get(n) is not None}

    def link_mat_at(q):
        fr, _ = CR._link_frames(model, q)
        return lambda prim: matrix(*fr[k_of_prim[prim.GetPath()]])
    segs, ends = build_cable(stage, points, clip_at, prim_of["tool"], link_mat_at(q0), args.diameter / 2,
                             args.kg_per_m)
    from isaacsim.core.prims import RigidPrim
    cable_view = RigidPrim(CABLE + "/seg_.*", name="cable")
    world.scene.add(cable_view)
    from omni.physx import get_physx_interface
    bracket_path = prim_of["tool"].GetPath().pathString

    def bracket_pose():
        """The bracket's link as the physics has it: (position, quat w x y z)."""
        d = get_physx_interface().get_rigidbody_transformation(bracket_path)
        pos, r = d["position"], d["rotation"]                # rotation x y z w
        return (pos[0], pos[1], pos[2]), (r[3], r[0], r[1], r[2])
    near_clip = set()                                        # segments at a clip or a connector: their bends
    for i in list(clip_at) + [0, len(points) - 2]:           # and contacts are the clip's, not the motion's
        near_clip.update(range(max(0, i - 2), i + 3))
    world.reset()                                           # the cable into the physics; the arm back to q0
    robot.set_joint_positions(np.radians(q0), joint_indices=idx)
    print("[cable] %d segments, %.2f m, clips on %s" % (len(segs), SEG_M * len(segs),
                                                        [layout["points"][i]["id"] for i in range(1, len(pts) - 1)]))

    series = []
    contacts = {"now": set()}
    rest_self, rest_pinched = set(), set()
    try:
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType

        def on_contact(headers, data):
            for h in headers:
                a, b = contact_paths(h)
                if CABLE not in a and CABLE not in b:
                    continue
                key = tuple(sorted((a, b)))
                if h.type in (ContactEventType.CONTACT_FOUND, ContactEventType.CONTACT_PERSIST):
                    contacts["now"].add(key)
                elif h.type == ContactEventType.CONTACT_LOST:
                    contacts["now"].discard(key)
        sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)  # noqa: F841
    except Exception as e:
        print("[cable] contact reports unavailable: %s" % e)

    def pose_mat(pos, quat):
        m = Gf.Matrix4d(1.0)
        m.SetRotateOnly(Gf.Quatd(float(quat[0]), Gf.Vec3d(*[float(x) for x in quat[1:]])))
        m.SetTranslateOnly(Gf.Vec3d(*[float(x) for x in pos]))
        return m

    def pull(name, poses, q_sim):
        """The connector's pull (N): its spring's stretch -- the segment and the bracket's link as the
        physics has them."""
        body, local, s, end_local = ends[name]
        if body is not None:
            anchor = pose_mat(*bracket_pose()).Transform(Gf.Vec3d(*local))
        else:
            anchor = Gf.Vec3d(*local)
        pos, quat = poses[0][s], poses[1][s]                # quat w x y z
        r = Gf.Rotation(Gf.Quatd(float(quat[0]), Gf.Vec3d(*[float(x) for x in quat[1:]])))
        seg_end = Gf.Vec3d(*[float(x) for x in pos]) + r.TransformDir(Gf.Vec3d(*end_local))
        return (anchor - seg_end).GetLength() * CONNECTOR_N_PER_M

    def what(path):
        """A contact's other side in words: a link, the strip, the room, or None."""
        if path.startswith("/World/fr20"):
            if "/led_strip/" in path:
                return "led_strip"
            return next((n for n in reversed(path.split("/")) if n.endswith("_link")), path)
        if path.startswith("/World/Room"):
            return "room:" + path.split("/")[4] if len(path.split("/")) > 4 else "room"
        return None

    def classify():
        """(things touched, self contact, pinched segments): a pinch is one
        segment touching two things at once (two links, a link and the room)."""
        touched, self_touch, per_seg = set(), False, {}
        for a, b in contacts["now"]:
            ca, cb = CABLE in a, CABLE in b
            if ca and cb:
                ia, ib = int(a.rsplit("_", 1)[1]), int(b.rsplit("_", 1)[1])
                if abs(ia - ib) > 3 and (min(ia, ib), max(ia, ib)) not in rest_self:
                    self_touch = True
                continue
            seg_path, other = (a, b) if ca else (b, a)
            name = what(other)
            if name is None or "_" not in seg_path.rsplit("/", 1)[-1]:
                continue
            touched.add(name)
            per_seg.setdefault(seg_path, set()).add(name)
        pinched = [sp for sp, names in per_seg.items() if len(names) >= 2
                   and int(sp.rsplit("_", 1)[1]) not in near_clip and sp not in rest_pinched]
        return touched, self_touch, pinched

    def bend_radius(poses):
        """The tightest bend along the cable now (m): a segment's length over the angle to the next."""
        quats = poses[1]
        axes = []
        for q in quats:
            r = Gf.Rotation(Gf.Quatd(float(q[0]), Gf.Vec3d(*[float(x) for x in q[1:]])))
            axes.append(r.TransformDir(Gf.Vec3d(1, 0, 0)))
        worst = math.inf
        for i, (a, b) in enumerate(zip(axes, axes[1:])):
            if i in near_clip or i + 1 in near_clip:
                continue
            ang = math.acos(max(-1.0, min(1.0, a * b)))
            if ang > 1e-4:
                worst = min(worst, SEG_M / ang)
        return worst

    render_settings()
    vp = use_camera(stage, "/World/CableCam", *camera_spec(args.camera, cfg, env))
    frames_dir = os.path.join(args.out, "_frames_cable")
    shot, corner = 0, []
    if args.video:
        from omni.kit.viewport.utility import capture_viewport_to_file
        shutil.rmtree(frames_dir, ignore_errors=True)
        os.makedirs(frames_dir)
    t = 0.0

    def step(q, label, record):
        nonlocal t, shot
        robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=idx))
        shoot = args.video and t * FPS_VIDEO >= shot
        world.step(render=shoot or not args.headless)
        t += DT
        if shoot:
            capture_viewport_to_file(vp, os.path.join(frames_dir, "f_%05d.png" % shot))
            corner.append((shot / float(FPS_VIDEO), label()))
            shot += 1
        return record()

    # settle, the robot still; the connectors' pull at rest (the cable's weight) is the baseline
    for k in range(int(SETTLE_S / DT)):
        step(q0, lambda: "settling", lambda: None)
        if args.debug and k % int(0.25 / DT) == 0:
            poses = cable_view.get_world_poses()
            print("[cable] settle %.2f s: strip %.2f N, laptop %.2f N; seg0 %s"
                  % (k * DT, pull("strip", poses, None), pull("laptop", poses, None),
                     [round(float(x), 4) for x in poses[0][0]]), flush=True)
    poses = cable_view.get_world_poses()
    q_sim = [float(x) for x in np.degrees(robot.get_joint_positions(joint_indices=idx))]
    if args.debug:
        raw = get_physx_interface().get_rigidbody_transformation(bracket_path)
        fkm = link_mat_at(q_sim)(prim_of["tool"])
        print("[cable] raw rotation %s; FK quat %s" % (list(raw["rotation"]), fkm.ExtractRotationQuat()), flush=True)
    fk = link_mat_at(q_sim)(prim_of["tool"]).ExtractTranslation()
    print("[cable] the bracket's link: physics %s, our FK %s" % ([round(x, 4) for x in bracket_pose()[0]],
                                                                 [round(x, 4) for x in fk]), flush=True)
    rest = {"strip": pull("strip", poses, q_sim), "laptop": pull("laptop", poses, q_sim)}
    for a, b in contacts["now"]:                            # what touches at rest is how it hangs
        if CABLE in a and CABLE in b:
            ia, ib = int(a.rsplit("_", 1)[1]), int(b.rsplit("_", 1)[1])
            rest_self.add((min(ia, ib), max(ia, ib)))
    rest_pinched.update(classify()[2])
    if args.debug:
        seen = {}
        for a_, b_ in contacts["now"]:
            if CABLE in a_ and CABLE not in b_:
                seen.setdefault(int(a_.rsplit("_", 1)[1]), set()).add(what(b_))
            elif CABLE in b_ and CABLE not in a_:
                seen.setdefault(int(b_.rsplit("_", 1)[1]), set()).add(what(a_))
        print("[cable] at rest, touching: %s" % sorted((k, sorted(v)) for k, v in seen.items()), flush=True)
        print("[cable] clips at segments %s" % sorted(clip_at), flush=True)
    print("[cable] at rest: strip %.2f N, laptop %.2f N, tightest bend %.3f m"
          % (rest["strip"], rest["laptop"], bend_radius(poses)), flush=True)
    if args.stills:
        stills(stage, robot, idx, q0, world, layout, pts)
        return
    per = {}
    order = []
    for seg, motion in seq:
        m = per.setdefault(motion, {"pull_strip_n": 0.0, "pull_laptop_n": 0.0, "rub_s": {}, "self_touch_s": 0.0,
                                    "pinch_s": 0.0, "pinch_at": {}, "bend_min_m": math.inf, "played": [],
                                    "over_s": 0.0})
        if motion not in order:
            order.append(motion)
        m["played"].append(seg.name)
        n = max(1, int(seg.duration / DT))
        live = {"ps": 0.0, "pl": 0.0}

        def rec(m=m, live=live):
            poses = cable_view.get_world_poses()
            q_sim = [float(x) for x in np.degrees(robot.get_joint_positions(joint_indices=idx))]
            ps, pl = pull("strip", poses, q_sim), pull("laptop", poses, q_sim)
            live["ps"], live["pl"] = ps, pl
            m["pull_strip_n"] = max(m["pull_strip_n"], ps)
            m["pull_laptop_n"] = max(m["pull_laptop_n"], pl)
            m["bend_min_m"] = min(m["bend_min_m"], bend_radius(poses))
            extra = max(ps - rest["strip"], pl - rest["laptop"])
            m["over_s"] += DT if extra > PULL_N else 0.0
            if t >= len(series) / float(FPS_VIDEO):             # the readings over time, at the video's rate
                series.append([round(t, 3), motion, round(ps, 2), round(pl, 2)])
            touched, self_touch, pinched = classify()
            for L in touched:
                m["rub_s"][L] = m["rub_s"].get(L, 0.0) + DT
            m["self_touch_s"] += DT if self_touch else 0.0
            m["pinch_s"] += DT if pinched else 0.0
            for sp in pinched:
                k = sp.rsplit("/", 1)[-1]
                m["pinch_at"][k] = m["pinch_at"].get(k, 0.0) + DT

        for i in range(n + 1):
            q = seg.at(seg.duration * i / n)
            step(q, lambda seg=seg, motion=motion, live=live: "%s   %s   pull strip %.1f N  laptop %.1f N"
                 % (motion, seg.name, live["ps"], live["pl"]), rec)
        for _ in range(int(HOLD_S / DT)):
            step(seg.q[-1], lambda motion=motion, live=live: "%s   hold   pull strip %.1f N  laptop %.1f N"
                 % (motion, live["ps"], live["pl"]), rec)

    rows = []
    for motion in order:
        m = per[motion]
        extra_s, extra_l = m["pull_strip_n"] - rest["strip"], m["pull_laptop_n"] - rest["laptop"]
        flags = []
        if m["over_s"] > 0.1:                               # a spike shorter than that: the solver, not the cable
            flags.append("pulls a connector (+%.1f N, %.1f s over %g N)" % (max(extra_s, extra_l), m["over_s"],
                                                                          PULL_N))
        if m["self_touch_s"] > 0.2:
            flags.append("touches itself %.1f s (a wrap or knot)" % m["self_touch_s"])
        if m["pinch_s"] > 0.1:
            flags.append("pinched %.1f s" % m["pinch_s"])
        if m["rub_s"].get("led_strip", 0.0) > 0.1:
            flags.append("on the strip %.1f s" % m["rub_s"]["led_strip"])
        if m["bend_min_m"] < BEND_R_MIN_M:
            flags.append("bent to r %.0f mm" % (1000 * m["bend_min_m"]))
        top_pinch = sorted(m["pinch_at"].items(), key=lambda x: -x[1])[:3]
        rows.append(dict(motion=motion, pull_strip_n=round(m["pull_strip_n"], 2),
                         pull_laptop_n=round(m["pull_laptop_n"], 2), extra_strip_n=round(extra_s, 2),
                         extra_laptop_n=round(extra_l, 2), over_s=round(m["over_s"], 2),
                         bend_min_m=round(m["bend_min_m"], 4),
                         contact_s={k: round(v, 2) for k, v in sorted(m["rub_s"].items())},
                         self_touch_s=round(m["self_touch_s"], 2), pinch_s=round(m["pinch_s"], 2),
                         pinch_at=[[k, round(v, 2)] for k, v in top_pinch], flags=flags, played=m["played"]))
        print("[cable] %-24s strip %5.1f N (+%4.1f)  laptop %5.1f N (+%4.1f)  over %4.1f s  bend %3.0f mm  %s"
              % (motion, m["pull_strip_n"], extra_s, m["pull_laptop_n"], extra_l, m["over_s"], 1000 * m["bend_min_m"],
                 "; ".join(flags) or "ok"), flush=True)
    more = [float(x) for x in args.extra.split(",") if x.strip()] if args.extra else []
    doc = {"layout": [p["id"] for p in layout["points"]],
           "spans_m": [round(s["cable_m"] + (more[i] if i < len(more) else 0.0), 3)
                       for i, s in enumerate(layout["spans"])],
           "cable_m": round(SEG_M * len(segs), 2), "diameter_m": args.diameter, "kg_per_m": args.kg_per_m,
           "connector_n_per_m": CONNECTOR_N_PER_M, "pull_limit_n": PULL_N, "bend_r_min_m": BEND_R_MIN_M,
           "at_rest_n": {k: round(v, 2) for k, v in rest.items()}, "motions": rows,
           "series": {"columns": ["t_s", "motion", "strip_n", "laptop_n"], "rows": series}}
    if args.video and shot:
        for _ in range(60):
            app.update()
        with open(os.path.join(frames_dir, "overlay.ass"), "w", encoding="utf-8") as f:
            f.write(overlay.ass(overlay.compress(corner, 0.25), 1280, 720, shot / float(FPS_VIDEO)))
        mp4 = os.path.join(args.out, "cable_%dclips%s.mp4" % (args.clips, ("_extra" + args.extra.replace(",", "_"))
                                                              if args.extra else ""))
        r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS_VIDEO), "-i", "f_%05d.png",
                            "-vf", "ass=overlay.ass", "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "22",
                            os.path.abspath(mp4)], cwd=frames_dir, capture_output=True, text=True)
        if r.returncode == 0:
            shutil.rmtree(frames_dir, ignore_errors=True)
            doc["video"] = os.path.relpath(mp4, ROOT).replace("\\", "/")
            print("[cable] video %s" % mp4)
        else:
            print("[cable] ffmpeg: %s" % r.stderr[-400:])
    tag = "%dclips%s" % (args.clips, ("_extra" + args.extra.replace(",", "_")) if args.extra else "")
    out = os.path.join(args.out, "cable_sim_%s.json" % tag)
    json.dump(doc, open(out, "w"), indent=1)
    print("[cable] wrote %s" % out)


main()
app.close()
