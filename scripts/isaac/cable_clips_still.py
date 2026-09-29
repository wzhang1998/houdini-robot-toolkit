"""Where the cable's clips go, as pictures to take to the arm: the FR20 still
at a hub in the lab-look room, a numbered marker at the strip's plug, each
clip and the cart's end, from two sides (an axis cap faces one way), the
numbers and a legend drawn on (the cable per span, with its service loops).

    C:/isaacsim6/python.bat scripts/isaac/cable_clips_still.py --clips 4 --extra 0.2,0,0,0,0.5

Reads geo/cable/cable_route.json (cable_route.py); writes
geo/cable/clips_<view>.png and geo/cable/clips.png (both side by side).
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=os.path.join(ROOT, "shows", "party.json"))
ap.add_argument("--route", default=os.path.join(ROOT, "geo", "cable", "cable_route.json"))
ap.add_argument("--clips", type=int, default=4)
ap.add_argument("--extra", default="0.2,0,0,0,0.5", help="the service loops cable_sim found (m per span)")
ap.add_argument("--hub", default="rest")
ap.add_argument("--out", default=os.path.join(ROOT, "geo", "cable"))
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

W, H = 1280, 720
app = SimulationApp({"headless": True, "width": W, "height": H, "renderer": "RaytracedLighting"})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file  # noqa: E402
from pxr import Gf, Sdf, UsdGeom  # noqa: E402

import cable_route as CR  # noqa: E402
import collision as C  # noqa: E402
from isaac_stage import PHYSICS_DT, attach_tool, import_robot, load_room, render_settings, use_camera  # noqa: E402

COLOURS = [(1.0, 0.85, 0.1), (0.1, 0.8, 1.0), (0.3, 1.0, 0.3), (1.0, 0.4, 0.9), (1.0, 0.5, 0.1), (0.6, 0.6, 1.0),
           (1.0, 1.0, 1.0)]
APERTURE_MM = 20.955                  # USD's default horizontal aperture
K_OF = {"tool": 5, "wrist2_link": 4, "wrist1_link": 3, "forearm_link": 2, "upperarm_link": 1, "shoulder_link": 0,
        "base_link": -1}
WORDS = {"tool": "灯条插头（支架侧面）", "cart": "红车上的电脑"}


def project(p, eye, target, focal):
    """Pixel (x, y) of world point p in a camera at eye looking at target (Z up)."""
    f = [target[i] - eye[i] for i in range(3)]
    n = math.sqrt(sum(x * x for x in f))
    f = [x / n for x in f]
    r = [f[1] * 1.0 - f[2] * 0.0, f[2] * 0.0 - f[0] * 1.0, 0.0]            # f x up
    n = math.sqrt(sum(x * x for x in r))
    r = [x / n for x in r]
    u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]]
    d = [p[i] - eye[i] for i in range(3)]
    zc = sum(d[i] * f[i] for i in range(3))
    s = focal / APERTURE_MM * W
    return W / 2 + sum(d[i] * r[i] for i in range(3)) / zc * s, H / 2 - sum(d[i] * u[i] for i in range(3)) / zc * s


def label_of(pid):
    if pid == "cart":
        return WORDS["cart"]
    if pid.startswith("tool/"):
        return WORDS["tool"]
    link, where = pid.split("/")
    names = {"J1": "底座转动 J1", "J2": "肩 J2", "J3": "肘 J3", "J4": "腕1 J4", "J5": "腕2 J5", "J6": "腕3 J6"}
    if "_axis" in where:
        j, side = where.split("_axis")
        return "%s 关节盖（%s 侧）" % (names[j], side)
    parts = {"shoulder_link": "肩座外壳侧面", "upperarm_link": "大臂", "forearm_link": "小臂"}
    return parts.get(link, link)


def main():
    cfg = json.load(open(args.config))
    layout = next(L for L in json.load(open(args.route))["layouts"] if L["clips"] == args.clips)
    more = [float(x) for x in args.extra.split(",") if x.strip()] if args.extra else []
    q = cfg["hubs"][args.hub]["q"] if "q" in cfg["hubs"][args.hub] else None
    if q is None:
        import show
        q = show.Graph.load(show.compiled_path(args.config)).hubs[args.hub]
    model = C.load_model("fr20")
    frames, _ = CR._link_frames(model, q)
    pts = [tuple(p["p_world_m"]) if p["id"] == "cart" else CR._to_world(frames[K_OF[p["link"]]], p["p_link_m"])
           for p in layout["points"]]

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    load_room(stage, args.config, "room", guides=False)
    prim_path = import_robot()
    attach_tool(stage)
    robot = world.scene.add(SingleArticulation(prim_path, name="fr20"))
    for i, p in enumerate(pts):
        s = UsdGeom.Sphere.Define(stage, Sdf.Path("/World/Clips/c%d" % i))
        s.CreateRadiusAttr(0.03)
        s.CreateDisplayColorAttr([Gf.Vec3f(*COLOURS[i % len(COLOURS)])])
        s.AddTranslateOp().Set(Gf.Vec3d(*p))
    world.reset()
    dof = list(robot.dof_names)
    idx = np.array([dof.index("j%d" % i) for i in range(1, 7)])
    robot.set_joint_positions(np.radians(q), joint_indices=idx)
    render_settings()
    mid = [sum(p[k] for p in pts[:-1]) / (len(pts) - 1) for k in range(3)]
    views = {"a": ([0.95, -0.95, 1.75], mid, 11.0), "b": ([-1.35, -0.75, 1.65], mid, 11.0)}
    from PIL import Image, ImageDraw, ImageFont
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 22)
    small = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 18)
    shots = []
    for name, (eye, target, focal) in views.items():
        vp = use_camera(stage, "/World/ClipCam_" + name, eye, target, focal)
        for _ in range(60):
            robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=idx))
            world.step(render=True)
        png = os.path.join(args.out, "clips_%s.png" % name)
        capture_viewport_to_file(vp, png)
        for _ in range(30):
            app.update()
        img = Image.open(png).convert("RGB")
        d = ImageDraw.Draw(img)
        for i, p in enumerate(pts):
            x, y = project(p, eye, target, focal)
            if not (0 <= x < W and 0 <= y < H):
                continue
            col = tuple(int(255 * c) for c in COLOURS[i % len(COLOURS)])
            d.ellipse([x - 16, y - 16, x + 16, y + 16], outline=col, width=3)
            d.rectangle([x + 16, y - 30, x + 44, y - 4], fill=(0, 0, 0))
            d.text((x + 22, y - 31), str(i + 1), fill=col, font=font)
        shots.append(img)
    both = Image.new("RGB", (W * 2, H + 230), (20, 20, 22))
    for i, img in enumerate(shots):
        both.paste(img, (W * i, 0))
    d = ImageDraw.Draw(both)
    y = H + 12
    d.text((20, y), "线缆固定点（机械臂在 %s 姿态；左右两张是两个方向）" % args.hub, fill=(255, 255, 255), font=font)
    y += 36
    for i, p in enumerate(layout["points"]):
        col = tuple(int(255 * c) for c in COLOURS[i % len(COLOURS)])
        d.text((20 + (i % 3) * 850, y + (i // 3) * 30), "%d  %s" % (i + 1, label_of(p["id"])), fill=col, font=small)
    y += 70
    spans = ["%d→%d: %.2f m%s" % (i + 1, i + 2, s["cable_m"] + (more[i] if i < len(more) else 0.0),
                                  "（含 %.0f cm 服务圈）" % (100 * more[i]) if i < len(more) and more[i] else "")
             for i, s in enumerate(layout["spans"])]
    total = sum(s["cable_m"] + (more[i] if i < len(more) else 0.0) for i, s in enumerate(layout["spans"]))
    d.text((20, y), "每段线长：" + "    ".join(spans), fill=(230, 230, 230), font=small)
    d.text((20, y + 30), "总长约 %.1f m。1→2 的服务圈给 J6 转动绕手腕用；最后一段的服务圈给 J1 大角度转动（big wipe）用。"
                         "J6 缝（腕2/腕3）和 J1 缝（底座/肩）建议加护套。" % total, fill=(230, 230, 230), font=small)
    out = os.path.join(args.out, "clips.png")
    both.save(out)
    print("[clips] %s" % out)


main()
app.close()
