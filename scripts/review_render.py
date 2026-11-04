"""Review sheets for the clip library, without Houdini: one PNG per clip.

The Houdini route (render_clip_review.py, scenes/FR20_review.hiplc) needs
hython, ffmpeg and a GPU. This one needs only Python with numpy and
matplotlib (Agg, off screen), so a clip can be reviewed for looks on any
machine -- the review-clips skill (.claude/skills/review-clips) runs it and
judges the sheets against docs/review_rubric.md.

    uv run --with numpy --with matplotlib scripts/review_render.py geo/rebuild/party
    uv run --with numpy --with matplotlib scripts/review_render.py tests/clips/*.json tests/csv/dance_*.csv
    uv run --with numpy --with matplotlib scripts/review_render.py shows/party.compiled.json --ids rest_00_float-float
    uv run --with numpy --with matplotlib scripts/review_render.py --self-test

Inputs (any mix): a motion clip (motionlab.clip/1 .json), a directory or
manifest.json of them (show.py's write_preview, clip_factory, choreo), a
compiled show (*.compiled.json: its idle segments; --kinds for the others)
or a joint CSV (fairino_player's columns).

Per clip, under --out (default geo/review_png, not in git):

    <id>.png          three orthographic views -- FRONT (the robot's front,
                      from -X), AUDIENCE SIDE (from the audience zone
                      towards the base, level), TOP -- each with N ghost
                      poses (the FR20's URDF STLs decimated by vertex
                      clustering, --mesh-cell; --capsules draws
                      collision.py's capsule model instead), numbered at the
                      tool, the LED strip as a line with its LEDs' facing,
                      the TCP path coloured bar by bar by the INTENDED Laban
                      action (grey outside the bars), the room's obstacles
                      as wireframes (keep-out red, slow orange, work zones
                      dashed, the show's paper gold, the audience zone
                      dotted); a timeline (TCP and strip-tip speed over the
                      bars, each joint's speed and acceleration as % of its
                      limit, the ghosts' times) and the numbers: id,
                      duration, intent vs measured action per bar, peak
                      speed and acceleration % per joint, room clearance
    <id>.labels.json  the same numbers, for a reviewer that reads text

Ghosts: the clip's key poses -- where the watched points (TCP, strip tips)
come nearly to rest -- topped up evenly in time, --ghosts of them (8-12).

Numbers: joint peaks are motion_clip.measure's (the player's PCHIP at
125 Hz, the clip's own timing) against the profile's limits; the room
clearance collision.check against the show's room plus its paper
(show.show_env, margins.idle_canvas_m) -- the distance, not the distance
past the margin. A gesture or a path (gestures.py, paths.py) has no Laban
intent: its bars are its own 4-beat bars, coloured by the MEASURED action
(motion_labels.label_span), and say so.

Pure Python plus numpy / matplotlib; reads the generators' modules, never
changes them.
"""

import argparse
import glob
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import collision as C  # noqa: E402
import fairino_player as P  # noqa: E402
import motion_clip as M  # noqa: E402
import motion_labels as L  # noqa: E402
import robot_profile as RP  # noqa: E402
import urdf_rig as U  # noqa: E402

OUT = os.path.join(ROOT, "geo", "review_png")
SHOW = os.path.join(ROOT, "shows", "party.json")
# render_clip_review.ACTION_RGB: the same colours as the Houdini review
ACTION_RGB = {"punch": (0.95, 0.25, 0.2), "slash": (1.0, 0.6, 0.15), "press": (0.65, 0.2, 0.15),
              "wring": (0.7, 0.35, 0.9), "dab": (1.0, 0.9, 0.2), "flick": (0.25, 0.9, 0.95),
              "glide": (0.3, 0.85, 0.35), "float": (0.55, 0.7, 1.0)}
OFF_BAR_RGB = (0.6, 0.6, 0.6)
# the views' extent, robot base frame (m): the same for every clip, so sheets compare
BOX = {"x": (-2.0, 1.3), "y": (-1.9, 1.9), "z": (-0.05, 2.25)}
IMG_W_PX = 900                   # a view's ghost image, pixels across
VIEW_PAD_M = 0.3                 # room shown around what the clip sweeps
MIN_VIEW_M = 1.6
GHOSTS = 10
MESH_CELL_M = 0.015              # vertex clustering cell: ~6.6k triangles for the arm
RATE_HZ = 125.0
TWITCH_DEG = 2.0                 # a reversal of a joint after less than this travel ...
TWITCH_S = 0.3                   # ... and within this long counts as a twitch
TWITCH_V_DEG_S = 3.0             # velocity sign changes below this speed are noise


# --------------------------------------------------------------------------
# inputs
# --------------------------------------------------------------------------

def _clip_from_segment(seg, src):
    t, q = seg["t"], seg["q"]
    t0 = t[0]
    return {"schema": M.SCHEMA, "id": seg["name"], "robot": "fr20",
            "joint_names": ["j%d" % i for i in range(1, 7)],
            "points": [{"t": a - t0, "q": b} for a, b in zip(t, q)],
            "tcp": None, "meta": {"duration_s": t[-1] - t0,
                                  "show": {"kind": seg["kind"], "start": seg["start"], "end": seg["end"]}},
            "labels": seg.get("labels") or {}, "_source": src, "_show_labels": seg.get("labels") or {},
            "_hub": seg["start"]}


def load_inputs(paths, ids=None, kinds=("idle",)):
    """[(clip, source)] from clip files, directories / manifests, compiled shows and CSVs."""
    out = []

    def want(cid):
        return not ids or cid in ids

    for p in paths:
        if os.path.isdir(p):
            p = os.path.join(p, "manifest.json")
        base = os.path.basename(p)
        if base == "manifest.json":
            man = json.load(open(p))
            rows = {r["name"]: r for r in man.get("segments", [])}
            for e in man.get("clips", []):
                if not want(e["id"]):
                    continue
                row = rows.get(e["id"], {})
                if row and row.get("kind") not in kinds:
                    continue
                c = json.load(open(os.path.join(os.path.dirname(p), e["file"])))
                c["_source"] = os.path.relpath(os.path.join(os.path.dirname(p), e["file"]), ROOT)
                c["_show_labels"] = row.get("labels") or {}
                c["_hub"] = row.get("start")
                out.append(c)
        elif base.endswith(".compiled.json"):
            g = json.load(open(p))
            for s in g["segments"]:
                if s["kind"] in kinds and want(s["name"]):
                    out.append(_clip_from_segment(s, os.path.relpath(p, ROOT)))
        elif p.lower().endswith(".csv"):
            c = M.from_csv(p, "fr20")
            if want(c["id"]):
                c["_source"] = os.path.relpath(p, ROOT)
                out.append(c)
        else:
            c = json.load(open(p))
            if want(c.get("id")):
                c["_source"] = os.path.relpath(p, ROOT)
                out.append(c)
    return out


# --------------------------------------------------------------------------
# the robot's geometry
# --------------------------------------------------------------------------

_MESH = {}


def read_stl(path):
    """Triangles (n, 3, 3) of a binary or ASCII STL."""
    import numpy as np
    data = open(path, "rb").read()
    n = int(np.frombuffer(data, "<u4", 1, 80)[0]) if len(data) >= 84 else 0
    if n and len(data) == 84 + 50 * n:
        rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        return np.frombuffer(data, rec, n, 84)["v"].astype(float)
    pts = [tuple(float(x) for x in ln.split()[1:4]) for ln in data.decode("ascii", "ignore").splitlines()
           if ln.strip().startswith("vertex")]
    return np.array(pts, float).reshape(-1, 3, 3)


def decimate(tri, cell):
    """Vertex clustering: every vertex snapped to the mean of its cell's, the
    triangles that collapse dropped. (vertices (m, 3), faces (k, 3))."""
    import numpy as np
    v = tri.reshape(-1, 3)
    key = np.floor(v / cell).astype(np.int64)
    uk, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    rep = np.zeros((len(uk), 3))
    np.add.at(rep, inv, v)
    rep /= np.bincount(inv)[:, None]
    f = inv.reshape(-1, 3)
    ok = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = np.unique(np.sort(f[ok], axis=1), axis=0)
    return rep, f


def arm_meshes(cell=MESH_CELL_M, robot="fr20"):
    """[(joint index: -1 the base, vertices, faces)] in each link's frame."""
    key = (robot, cell)
    if key not in _MESH:
        prof = RP.load(robot)
        parsed = U.parse_urdf(os.path.join(ROOT, prof["rig"]["urdf"]))
        links = [(parsed["root_link"], -1)] + [(j["child"], i) for i, j in enumerate(parsed["chain"])]
        out = []
        for name, idx in links:
            mesh = parsed["links"].get(name)
            if mesh:
                v, f = decimate(read_stl(U.resolve_mesh(mesh, os.path.join(ROOT, "assets"))), cell)
                out.append((idx, v, f))
        _MESH[key] = out
    return _MESH[key]


def posed_triangles(model, q, cell=MESH_CELL_M):
    """Triangles (n, 3, 3) of the arm's links at pose q, the robot's frame."""
    import numpy as np
    fk = U.forward_kinematics(model["chain"], q)
    parts = []
    for idx, v, f in arm_meshes(cell):
        if idx < 0:
            R, p = np.eye(3), np.zeros(3)
        else:
            R, p = np.array(fk[idx]["link_R"]), np.array(fk[idx]["link_p"])
        parts.append((v @ R.T + p)[f])
    return np.concatenate(parts)


def posed_capsules(model, q, tool=None):
    """[(name, a, b, r)] of the capsule model at q: the links' (tool False),
    the tool's (tool True), or all (None)."""
    caps, _ = C.capsules(model, q)
    if tool is None:
        return caps
    return [c for c in caps if c[0].startswith("tool") == bool(tool)]


# --------------------------------------------------------------------------
# views
# --------------------------------------------------------------------------

def _unit(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return tuple(x / n for x in v)


def views(cfg):
    """[(key, title, right, up, forward)]: orthographic views, forward from
    the viewer into the scene."""
    a = ((cfg or {}).get("zones") or {}).get("audience")
    to_base = _unit((-a["center"][0], -a["center"][1], 0.0)) if a else (0.0, 1.0, 0.0)
    up = (0.0, 0.0, 1.0)
    cross = lambda u, w: (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])  # noqa
    out = []
    for key, title, fwd in (("front", "FRONT (robot's front, from -X)", (1.0, 0.0, 0.0)),
                            ("side", "AUDIENCE SIDE (from the audience zone, level)", to_base)):
        out.append((key, title, _unit(cross(fwd, up)), up, fwd))
    out.append(("top", "TOP (from above; audience at the bottom)", (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                (0.0, 0.0, -1.0)))
    return out


def view_limits(view, pts=None, pad=VIEW_PAD_M):
    """((h0, h1), (v0, v1)) of a view: around pts (the clip's swept points,
    the robot's frame) padded by pad, at least MIN_VIEW_M across, the floor
    always in a side view; without pts, BOX's corners."""
    _, _, right, up, _ = view
    pts = pts if pts is not None else [(x, y, z) for x in BOX["x"] for y in BOX["y"] for z in BOX["z"]]
    hs = [sum(a * b for a, b in zip(c, right)) for c in pts]
    vs = [sum(a * b for a, b in zip(c, up)) for c in pts]

    def span(lo, hi):
        lo, hi = lo - pad, hi + pad
        if hi - lo < MIN_VIEW_M:
            mid = (lo + hi) / 2.0
            lo, hi = mid - MIN_VIEW_M / 2.0, mid + MIN_VIEW_M / 2.0
        return lo, hi
    if view[0] != "top":
        return span(min(hs), max(hs)), (BOX["z"][0], max(vs) + pad)
    return span(min(hs), max(hs)), span(min(vs), max(vs))


def swept_points(model, qs, tcp):
    """Points bounding the arm over poses qs (capsule ends +- radius) and the TCP path."""
    pts = [tuple(p) for p in tcp] + [(0.0, 0.0, 0.0)]
    for q in qs:
        for _, a, b, r in posed_capsules(model, q):
            for c in (a, b):
                pts += [(c[0] - r, c[1] - r, c[2] - r), (c[0] + r, c[1] + r, c[2] + r)]
    return pts


def px_per_m(lims):
    w = lims[0][1] - lims[0][0]
    return max(180.0, min(420.0, IMG_W_PX / w))


def project(view, pts):
    import numpy as np
    _, _, right, up, _ = view
    pts = np.asarray(pts, float)
    return np.stack([pts @ np.array(right), pts @ np.array(up)], axis=-1)


def stadium(a2, b2, r, n=8):
    """The outline of a capsule seen side on: a 2D segment a2-b2 thickened by r."""
    dx, dy = b2[0] - a2[0], b2[1] - a2[1]
    ang = math.atan2(dy, dx) if (dx or dy) else 0.0
    pts = []
    for c, start in ((b2, ang - math.pi / 2), (a2, ang + math.pi / 2)):
        for k in range(n + 1):
            t = start + math.pi * k / n
            pts.append((c[0] + r * math.cos(t), c[1] + r * math.sin(t)))
    return pts


def render_pose(view, lims, model, q, rgb, capsules=False, cell=MESH_CELL_M):
    """An RGBA image (rows from the top) of the arm at q seen in view, shaded,
    transparent around it."""
    import numpy as np
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.collections import PolyCollection
    from matplotlib.figure import Figure
    (h0, h1), (v0, v1) = lims
    ppm = px_per_m(lims)
    w, h = max(8, int((h1 - h0) * ppm)), max(8, int((v1 - v0) * ppm))
    fig = Figure(figsize=(w / 100.0, h / 100.0), dpi=100)
    fig.patch.set_alpha(0.0)
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(h0, h1)
    ax.set_ylim(v0, v1)
    ax.axis("off")
    fwd = np.array(view[4])
    base = np.array(rgb)
    polys, cols = [], []
    if capsules:
        items = posed_capsules(model, q, tool=False)
    else:
        tri = posed_triangles(model, q, cell)
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
        shade = 0.45 + 0.55 * np.abs(n @ fwd)
        depth = (tri.mean(axis=1) @ fwd)
        order = np.argsort(-depth)                         # far first
        p2 = project(view, tri)
        polys = list(p2[order])
        cols = list(np.clip(base[None, :] * shade[order, None], 0, 1))
        items = []
    if items:                                              # capsules: far first, by their middle
        items = sorted(items, key=lambda c: -float(np.dot((np.array(c[1]) + np.array(c[2])) / 2.0, fwd)))
        for name, a, b, r in items:
            a2, b2 = project(view, [a, b])
            polys.append(stadium(a2, b2, r))
            ax3 = abs(float(np.dot(_unit(tuple(np.array(b) - np.array(a))), fwd)))
            cols.append(np.clip(base * (0.6 + 0.4 * (1.0 - ax3)), 0, 1))
    tool = posed_capsules(model, q, tool=True)
    for name, a, b, r in sorted(tool, key=lambda c: -float(np.dot((np.array(c[1]) + np.array(c[2])) / 2.0, fwd))):
        a2, b2 = project(view, [a, b])
        polys.append(stadium(a2, b2, max(r, 0.008)))
        cols.append(np.array((0.25, 0.25, 0.3)))
    pc = PolyCollection(polys, facecolors=cols, edgecolors=cols, linewidths=0.3, antialiased=True)
    ax.add_collection(pc)
    canvas.draw()
    img = np.asarray(canvas.buffer_rgba()).copy()
    return img


def over(dst, src, alpha):
    """src (uint8 RGBA) composited over dst (float RGBA 0..1) at alpha."""
    import numpy as np
    s = src.astype(float) / 255.0
    a = s[..., 3:4] * alpha
    out = np.empty_like(dst)
    out[..., 3:4] = a + dst[..., 3:4] * (1.0 - a)
    out[..., :3] = (s[..., :3] * a + dst[..., :3] * dst[..., 3:4] * (1.0 - a)) / np.maximum(out[..., 3:4], 1e-6)
    return out


# --------------------------------------------------------------------------
# the room
# --------------------------------------------------------------------------

def box_edges(o):
    """12 edges of a (yawed) box, the robot's frame."""
    cx, cy, cz = o["center"]
    sx, sy, sz = (s / 2.0 for s in o["size"])
    yaw = math.radians(o.get("yaw_deg", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    pts = {}
    for i, dx in enumerate((-sx, sx)):
        for j, dy in enumerate((-sy, sy)):
            for k, dz in enumerate((-sz, sz)):
                pts[(i, j, k)] = (cx + c * dx - s * dy, cy + s * dx + c * dy, cz + dz)
    edges = []
    for a in pts:
        for d in range(3):
            if a[d] == 0:
                b = tuple(1 if e == d else a[e] for e in range(3))
                edges.append((pts[a], pts[b]))
    return edges


def cylinder_edges(o, n=24):
    cx, cy, z0 = o["center"]
    r, hgt = o["radius"], o["height"]
    ring = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]
    edges = []
    for z in (z0, z0 + hgt):
        edges += [((a[0], a[1], z), (b[0], b[1], z)) for a, b in zip(ring, ring[1:])]
    for k in range(0, n, n // 4):
        edges.append(((ring[k][0], ring[k][1], z0), (ring[k][0], ring[k][1], z0 + hgt)))
    return edges


def plane_line(view, o, lims):
    """The 2D line where a halfspace's plane crosses a view through the base
    (the view's depth 0), clipped to its limits; None when the plane faces
    the viewer (more than ~70 degrees from edge on)."""
    import numpy as np
    _, _, right, up, fwd = view
    n = np.array(o["normal"], float)
    if abs(float(n @ np.array(fwd))) > 0.35:
        return None
    a, b = float(n @ np.array(right)), float(n @ np.array(up))
    (h0, h1), (v0, v1) = lims
    pts = []
    if abs(b) > 1e-9:
        for h in (h0, h1):
            v = (o["offset"] - a * h) / b
            if v0 - 1e-9 <= v <= v1 + 1e-9:
                pts.append((h, v))
    if abs(a) > 1e-9:
        for v in (v0, v1):
            h = (o["offset"] - b * v) / a
            if h0 - 1e-9 <= h <= h1 + 1e-9:
                pts.append((h, v))
    pts = sorted(set((round(h, 6), round(v, 6)) for h, v in pts))
    return (pts[0], pts[-1]) if len(pts) >= 2 else None


def room_style(o):
    """(colour, linestyle, linewidth, alpha) of a room object."""
    if o.get("_canvas"):
        return (0.85, 0.65, 0.1), "-", 1.4, 0.95
    if o.get("_zone"):
        return (0.3, 0.45, 0.9), ":", 1.0, 0.8
    role = o["role"]
    if role == "keep_out":
        return (0.9, 0.15, 0.15), "-", 1.1, 0.9
    if role == "slow":
        return (0.95, 0.55, 0.1), "--", 0.8, 0.7
    if role == "work":
        return (0.35, 0.35, 0.35), "--", 0.7, 0.5
    return (0.25, 0.25, 0.25), "-", 0.8, 0.75


def draw_room(ax, view, lims, objs):
    for o in objs:
        col, ls, lw, al = room_style(o)
        if o["type"] == "halfspace":
            seg = plane_line(view, o, lims)
            if seg:
                ax.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], color=col, ls=ls, lw=lw * 1.4, alpha=al,
                        zorder=1)
                mid = ((seg[0][0] + seg[1][0]) / 2, (seg[0][1] + seg[1][1]) / 2)
                ax.text(mid[0], mid[1], o["name"], fontsize=5.5, color=col, alpha=0.8, zorder=1,
                        ha="center", va="bottom", clip_on=True)
            continue
        if o["type"] == "box":
            edges = box_edges(o)
        elif o["type"] == "cylinder":
            edges = cylinder_edges(o)
        elif o["type"] == "sphere":
            c, r = o["center"], o["radius"]
            edges = [((c[0] + r * math.cos(a), c[1] + r * math.sin(a), c[2]),
                      (c[0] + r * math.cos(a + 0.26), c[1] + r * math.sin(a + 0.26), c[2]))
                     for a in [k * 0.26 for k in range(25)]]
        else:
            continue
        for a, b in edges:
            p = project(view, [a, b])
            ax.plot(p[:, 0], p[:, 1], color=col, ls=ls, lw=lw, alpha=al, zorder=1)
        lab = project(view, [edges[0][0]])[0]
        ax.text(lab[0], lab[1], o["name"], fontsize=5.5, color=col, alpha=0.8, zorder=1, clip_on=True)


def room_objects(cfg, env, hub=None):
    """The room's objects plus the show's paper (as boxes, show.canvas_boxes)
    and its audience zone and the clip's hub zone (dotted, for orientation)."""
    import show
    paper = {b["name"] for b in show.canvas_boxes(cfg["canvas"])} if (cfg or {}).get("canvas") else set()
    objs = [dict(o, _canvas=o["name"] in paper) for o in env.get("objects", [])]
    if paper and not any(o["_canvas"] for o in objs):        # a room without the show's paper in it yet
        objs += [dict(b, _canvas=True) for b in show.canvas_boxes(cfg["canvas"])]
    zones = (cfg or {}).get("zones") or {}
    zone_of = ((cfg or {}).get("hubs") or {}).get(hub or "", {}).get("zone")
    for k in ("audience", zone_of):
        if k and k in zones:
            z = zones[k]
            objs.append({"name": "zone:" + k, "type": "box", "role": "zone", "_zone": True,
                         "center": z["center"], "size": z["size"], "yaw_deg": z.get("yaw_deg", 0.0)})
    return objs


# --------------------------------------------------------------------------
# what the sheet measures
# --------------------------------------------------------------------------

def resample(t, q, rate=RATE_HZ):
    """(ts, qs, dt): the player's path (PCHIP) at rate."""
    path = P.Path(t, q)
    n = max(2, int(math.ceil(path.duration * rate)))
    dt = path.duration / n
    return [k * dt for k in range(n + 1)], [path.at(k * dt) for k in range(n + 1)], dt


def derivatives(qs, dt):
    import numpy as np
    a = np.array(qs, float)
    v = np.gradient(a, dt, axis=0)
    acc = np.gradient(v, dt, axis=0)
    return v, acc


def gesture_bars(clip, labels):
    """[(t0, t1, intent, measured)] for a clip without Laban bars: its own
    4-beat bars at its tempo (slowed as it was), measured
    (label_span); the intent the family's name."""
    pts = clip["points"]
    dur = pts[-1]["t"]
    bpm = float(labels.get("bpm") or (clip.get("style") or {}).get("bpm") or 0.0)
    slowed = float((labels.get("params") or {}).get("slowed") or 1.0)
    fam = labels.get("family") or (clip.get("style") or {}).get("family") or "-"
    if bpm <= 0:
        return [(0.0, dur, fam, (labels.get("measured") or {}).get("action") or labels.get("action"))]
    bar = 4.0 * 60.0 / bpm * slowed
    edges = [0.0]
    while edges[-1] + bar < dur - 0.5 * bar:
        edges.append(edges[-1] + bar)
    edges.append(dur)
    out = []
    for t0, t1 in zip(edges, edges[1:]):
        out.append((t0, t1, fam, L.label_span(clip, t0, t1).get("action")))
    return out


def clip_bars(clip):
    """(bars [(t0, t1, intent, measured)], kind): 'laban' (choreo's bars),
    'family' (a gesture / path: measured per 4-beat bar), 'approx' (a
    compiled segment: the intent list spread evenly), or 'none'."""
    lab = clip.get("labels") or {}
    if lab.get("bars"):
        return [(b["t0"], b["t1"], b.get("intent"), b.get("action")) for b in lab["bars"]], "laban"
    show_lab = clip.get("_show_labels") or {}
    fam = lab.get("family") or show_lab.get("family") or (clip.get("style") or {}).get("family")
    if fam or (clip.get("style") or {}).get("generator") in ("gestures.py", "paths.py"):
        merged = dict(show_lab, **{k: v for k, v in lab.items() if v is not None})
        merged.setdefault("family", fam)
        return gesture_bars(clip, merged), "family"
    intent = show_lab.get("intent") or lab.get("intent")
    if isinstance(intent, dict):
        intent = intent.get("actions")
    if intent:
        dur = clip["points"][-1]["t"]
        n = len(intent)
        edges = [dur * k / n for k in range(n + 1)]
        return [(a, b, i, L.label_span(clip, a, b).get("action")) for a, b, i in zip(edges, edges[1:], intent)], \
            "approx"
    return [], "none"


def pick_ghosts(ts, speed, n):
    """n times: the clip's key poses (local minima of the watched speed under
    a third of its 90th percentile), spread out, topped up evenly; always
    the start."""
    dur = ts[-1]
    srt = sorted(speed)
    p90 = srt[int(0.9 * (len(srt) - 1))] or 1e-9
    gap = dur / (n * 1.6)
    keys = [ts[k] for k in range(1, len(speed) - 1)
            if speed[k] <= speed[k - 1] and speed[k] <= speed[k + 1] and speed[k] < p90 / 3.0]
    chosen = [0.0]
    for t in sorted(keys, key=lambda t: speed[min(range(len(ts)), key=lambda k: abs(ts[k] - t))]):
        if len(chosen) >= n:
            break
        if all(abs(t - c) >= gap for c in chosen) and t < dur - gap:
            chosen.append(t)
    k = 1
    while len(chosen) < n and k < 4 * n:
        for j in range(1, n + 1):
            if len(chosen) >= n:
                break
            t = dur * j / (n + 1)
            if all(abs(t - c) >= gap / k for c in chosen):
                chosen.append(t)
        k += 1
    return sorted(chosen)[:n]


def sign_crossings(xs, at=0.0):
    s = [x - at for x in xs if abs(x - at) > 1e-6]
    return sum(1 for a, b in zip(s, s[1:]) if (a > 0) != (b > 0))


def twitches(qs, v, dt):
    """Per joint: velocity reversals after less than TWITCH_DEG of travel and
    within TWITCH_S -- a joint going back on itself in a flick of a move."""
    out = []
    for j in range(6):
        last_t, last_q, last_s, n = None, None, 0, 0
        for k in range(len(qs)):
            vj = v[k][j]
            if abs(vj) < TWITCH_V_DEG_S:
                continue
            s = 1 if vj > 0 else -1
            if last_s and s != last_s:
                if last_t is not None and k * dt - last_t < TWITCH_S and abs(qs[k][j] - last_q) < TWITCH_DEG:
                    n += 1
                last_t, last_q = k * dt, qs[k][j]
            last_s = s
        out.append(n)
    return out


# --------------------------------------------------------------------------
# expressiveness: what makes a motion read as alive rather than mechanical
# --------------------------------------------------------------------------

MOVE_FLOOR = 0.05                # a frame moves when the TCP is faster than this share of its peak (a slow wind-up too)
BIG_MOVE = 0.35                  # a move is big at this share of the longest (and BIG_MOVE_M at least)
BIG_MOVE_M = 0.03
COUNTER_SHARE = (0.03, 0.4)      # a counter-move (wind-up, overshoot) is this share of its big move's length
COUNTER_GAP_S = 0.45             # ... and this near to it in time (between the speed floor's crossings)
HOLD_S = 0.4                     # a stillness inside the clip this long counts as a hold


def moves(t, path, speed):
    """[(i0, i1, length m, displacement)]: spans where the point moves
    (speed over MOVE_FLOOR of its peak), split where it turns back (two
    steps more than ~100 degrees apart: a reversal 24 fps samples can step
    over without a dip in speed)."""
    peak = max(speed) or 1e-9
    mov = [v > max(L.STILL_TCP_M_S, MOVE_FLOOR * peak) for v in speed]
    step = [None] + [[path[i][j] - path[i - 1][j] for j in range(3)] for i in range(1, len(path))]
    out, k = [], 0
    while k < len(mov):
        if not mov[k]:
            k += 1
            continue
        i0 = max(0, k - 1)
        k += 1
        while k < len(mov) and mov[k] and not (step[k] and step[k - 1] and _cos(step[k], step[k - 1]) < -0.2):
            k += 1
        i1 = min(len(path) - 1, k if k >= len(mov) or not mov[k] else k - 1)
        ln = sum(math.dist(path[i], path[i - 1]) for i in range(i0 + 1, i1 + 1))
        out.append((i0, i1, ln, [path[i1][j] - path[i0][j] for j in range(3)]))
    return out


def _cos(a, b):
    na, nb = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in b))
    return sum(x * y for x, y in zip(a, b)) / (na * nb) if na > 1e-9 and nb > 1e-9 else 0.0


def expression(clip, t, tcp, v_tcp, tips, bars):
    """The measures behind the rubric's expressiveness (docs/review_rubric.md, 4):

    lag_s            the wrist trailing the arm (motion_labels.successive_lag): follow-through
    accents_per_min  speed dips between surges (motion_labels): phrasing
    still_share      the share of the clip held still; holds: stillnesses of HOLD_S inside it
    dynamic_range    the TCP's peak over its mean speed while moving: light and shade
    big_moves        moves of BIG_MOVE of the longest; for them:
      anticipated    a short counter-move just before, against its direction (a wind-up)
      overshot       a short move just after, back against it (past the mark and settle)
      onset_cv       how much the times between their starts vary: ~0 metronomic
    bar_contrast     the largest spread of one effort (weight, time, space, flow) over the bars
    strip_ratio      the strip tips' path over the TCP's: how much the prop carries the motion
    joints_used      joints with over 10 % of the travel

    Heuristics on the TCP's path: a counter-move is found only when the speed
    dips between it and its move, as at a reversal."""
    q = [p["q"] for p in clip["points"]]
    lab = L.label(dict(clip, tcp=tcp))
    d = lab["descriptors"]
    mv = moves(t, tcp, v_tcp)
    longest = max((m[2] for m in mv), default=0.0)
    big = [k for k, m in enumerate(mv) if m[2] >= max(BIG_MOVE * longest, BIG_MOVE_M)]
    ant, ovs, marks = 0, 0, []
    for k in big:
        i0, i1, ln, disp = mv[k]
        if k > 0 and k - 1 not in big:
            p0, p1, pl, pd = mv[k - 1]
            if (t[i0] - t[p1] <= COUNTER_GAP_S and COUNTER_SHARE[0] * ln <= pl <= COUNTER_SHARE[1] * ln
                    and _cos(pd, disp) < -0.3):
                ant += 1
                marks.append(("A", t[i0]))
        if k + 1 < len(mv) and k + 1 not in big:
            n0, n1, nl, nd = mv[k + 1]
            if (t[n0] - t[i1] <= COUNTER_GAP_S and COUNTER_SHARE[0] * ln <= nl <= COUNTER_SHARE[1] * ln
                    and _cos(nd, disp) < -0.3):
                ovs += 1
                marks.append(("O", t[i1]))
    starts = [t[mv[k][0]] for k in big]
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    cv = (math.sqrt(sum((g - sum(gaps) / len(gaps)) ** 2 for g in gaps) / len(gaps)) / (sum(gaps) / len(gaps))
          if len(gaps) >= 2 and sum(gaps) > 0 else None)
    moving = [v for v in v_tcp if v > max(L.STILL_TCP_M_S, MOVE_FLOOR * (max(v_tcp) or 1e-9))]
    first, last = (mv[0][0], mv[-1][1]) if mv else (0, 0)
    holds = sum(1 for a, b in zip(mv, mv[1:]) if t[b[0]] - t[a[1]] >= HOLD_S and a[1] >= first and b[0] <= last)
    efforts = []
    for b in bars:
        if b[1] - b[0] > 0.3:
            e = L.label_span(dict(clip, tcp=tcp), b[0], b[1])
            if e.get("action"):
                efforts.append(e)
    contrast = (round(max(max(e[k] for e in efforts) - min(e[k] for e in efforts)
                          for k in ("weight", "time", "space", "flow")), 2) if len(efforts) >= 2 else None)
    tcp_path = sum(math.dist(tcp[i], tcp[i - 1]) for i in range(1, len(tcp))) or 1e-9
    tip_path = max(sum(math.dist(p[i], p[i - 1]) for i in range(1, len(p))) for p in tips)
    share = d.get("joint_share") or []
    return {
        "lag_s": d.get("successive_lag_s"), "accents_per_min": d.get("accents_per_min"),
        "still_share": (lab["measured"].get("raw") or {}).get("still_fraction"), "holds": holds,
        "dynamic_range": round(max(moving) / (sum(moving) / len(moving)), 2) if moving else None,
        "big_moves": len(big), "anticipated": ant, "overshot": ovs,
        "onset_cv": round(cv, 2) if cv is not None else None,
        "bar_contrast": contrast, "strip_ratio": round(tip_path / tcp_path, 2),
        "joints_used": sum(1 for x in share if x > 0.1),
        "_marks": marks,
    }


def measure_clip(clip, cfg, env, model):
    """Everything the sheet shows, as a dict (the labels.json)."""
    import numpy as np
    import show
    t = [p["t"] for p in clip["points"]]
    q = [p["q"] for p in clip["points"]]
    prof = RP.load("fr20")
    vel, acc = RP.velocity_limits(prof), RP.acceleration_limits(prof)
    jerk = RP.jerk_limits(prof)
    work = dict(clip)
    saf = M.measure(work)
    ts, qs, dt = resample(t, q)
    v, a = derivatives(qs, dt)
    tcp = clip.get("tcp") or M._tcp_path("fr20", q)
    clip["tcp"] = tcp
    strip = [show.strip_ends(x) for x in q]
    tips = [[s[0] for s in strip], [s[1] for s in strip]]
    def speeds(path):
        sp = [0.0] + [math.dist(path[i], path[i - 1]) / (t[i] - t[i - 1]) if t[i] > t[i - 1] else 0.0
                      for i in range(1, len(path))]
        return sp
    v_tcp = speeds(tcp)
    v_tips = [speeds(p) for p in tips]
    watched = [max(x) for x in zip(v_tcp, *v_tips)]
    rep = C.check(model, env, t, q)
    eyes = show.audience_eyes(cfg or {})
    face = [show.facing(x, eyes) for x in q] if eyes else []
    bars, bar_kind = clip_bars(clip)
    lab = clip.get("labels") or {}
    show_lab = clip.get("_show_labels") or {}
    meas = lab.get("measured") or {}
    if not meas:
        meas = L.label(dict(clip, tcp=tcp)).get("measured") or {}
    style = clip.get("style") or {}
    j5 = [x[4] for x in q]
    pv = [round(100.0 * float(np.max(np.abs(v[:, j]))) / vel[j], 1) for j in range(6)]
    pa = [round(100.0 * float(np.max(np.abs(a[:, j]))) / acc[j], 1) for j in range(6)]
    peak_vel = saf.get("peak_vel") or [float(np.max(np.abs(v[:, j]))) for j in range(6)]
    peak_acc = saf.get("peak_acc") or [float(np.max(np.abs(a[:, j]))) for j in range(6)]
    hub = clip.get("_hub") or ((clip.get("meta") or {}).get("show") or {}).get("start")
    if not hub and "_" in clip["id"] and clip["id"].split("_")[0] in ((cfg or {}).get("hubs") or {}):
        hub = clip["id"].split("_")[0]
    xs = list(zip(*tcp))
    out = {
        "id": clip["id"], "source": clip.get("_source"), "hub": hub,
        "generator": style.get("generator") or ("gestures.py" if lab.get("family") or show_lab.get("family")
                                                 else None),
        "family": lab.get("family") or show_lab.get("family"),
        "duration_s": round(t[-1], 3), "bpm": lab.get("bpm") or show_lab.get("bpm") or style.get("bpm")
        or (style.get("spec") or {}).get("bpm"),
        "intensity": show_lab.get("intensity") or style.get("intensity"),
        "params": lab.get("params") or show_lab.get("params"),
        "bars_kind": bar_kind,
        "bars": [{"t0": round(b[0], 3), "t1": round(b[1], 3), "intent": b[2], "measured": b[3]} for b in bars],
        "bars_agree": [sum(1 for b in bars if b[2] == b[3]), len(bars)] if bar_kind in ("laban", "approx") else None,
        "measured": {k: meas.get(k) for k in ("action", "weight", "time", "space", "flow")},
        "peak_vel_deg_s": [round(x, 1) for x in peak_vel], "peak_acc_deg_s2": [round(x, 1) for x in peak_acc],
        "vel_limit": vel, "acc_limit": acc,
        "peak_vel_pct": [round(100.0 * x / l, 1) for x, l in zip(peak_vel, vel)],
        "peak_acc_pct": [round(100.0 * x / l, 1) for x, l in zip(peak_acc, acc)],
        "peak_jerk_pct": ([round(100.0 * x / l, 1) for x, l in zip(saf["peak_jerk"], jerk)]
                          if jerk and saf.get("peak_jerk") else None),
        "playback_scale": saf.get("playback_scale"), "player_ok": saf.get("ok"), "player_reasons": saf.get("reasons"),
        "room": {"min_clearance_m": rep["min_env_clearance_m"], "closest": rep["closest_env"],
                 "min_self_clearance_m": rep["min_self_clearance_m"], "closest_self": rep["closest_self"],
                 "ok": rep["ok"], "violations": rep["violations"][:5], "margin_m": rep["margin_m"]},
        "flags": {
            "j5_zero_crossings": sign_crossings(j5),
            "j5_min_abs_deg": round(min(abs(x) for x in j5), 1),
            "j3_zero_crossings": sign_crossings([x[2] for x in q]),
            "j4_range_deg": round(max(x[3] for x in q) - min(x[3] for x in q), 1),
            "j6_range_deg": round(max(x[5] for x in q) - min(x[5] for x in q), 1),
            "twitches_per_joint": twitches(qs, v.tolist(), dt),
            "wrist_share": round(sum(sum(abs(b[j] - a_[j]) for a_, b in zip(q, q[1:])) for j in (3, 4, 5))
                                 / max(1e-9, sum(sum(abs(b[j] - a_[j]) for a_, b in zip(q, q[1:]))
                                                 for j in range(6))), 2),
            "facing_audience_mean": round(sum(face) / len(face), 2) if face else None,
            "facing_audience_share_over_half": round(sum(1 for f in face if f > 0.5) / len(face), 2) if face else None,
        },
        "tcp": {"z_min": round(min(xs[2]), 3), "z_max": round(max(xs[2]), 3),
                "extent_m": [round(max(c) - min(c), 3) for c in xs], "peak_mps": round(max(v_tcp), 3),
                "strip_tip_peak_mps": round(max(max(x) for x in v_tips), 3)},
        "show_labels": {k: show_lab.get(k) for k in ("energy", "clearance_m", "wrist_share", "action")
                        if show_lab.get(k) is not None},
    }
    expr = expression(clip, t, tcp, v_tcp, tips, bars)
    out["expression"] = {k: v for k, v in expr.items() if not k.startswith("_")}
    series = {"t": t, "v_tcp": v_tcp, "v_tips": v_tips, "watched": watched, "ts": ts, "marks": expr["_marks"],
              "vel_pct": (np.abs(v) / np.array(vel) * 100.0), "acc_pct": (np.abs(a) / np.array(acc) * 100.0),
              "tcp": tcp, "strip": strip, "face": face}
    return out, series


# --------------------------------------------------------------------------
# the sheet
# --------------------------------------------------------------------------

def _pct_colour(p):
    return "#c62828" if p >= 90 else "#ef6c00" if p >= 70 else "#222222"


def bar_colour(intent, measured, kind):
    key = measured if kind == "family" else intent
    return ACTION_RGB.get(key, OFF_BAR_RGB)


def render_sheet(clip, info, series, cfg, env, model, path, n_ghosts=GHOSTS, capsules=False, cell=MESH_CELL_M,
                 quantize=True):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.collections import LineCollection
    from matplotlib.lines import Line2D
    t = series["t"]
    q = [p["q"] for p in clip["points"]]
    gt = pick_ghosts(t, series["watched"], n_ghosts)
    gk = [min(range(len(t)), key=lambda k: abs(t[k] - x)) for x in gt]
    info["ghost_times_s"] = [round(t[k], 2) for k in gk]
    bars = [(b["t0"], b["t1"], b["intent"], b["measured"]) for b in info["bars"]]
    kind = info["bars_kind"]
    tcp = np.array(series["tcp"])
    cols = []
    for tt in t:
        c = next((bar_colour(i, m, kind) for t0, t1, i, m in bars if t0 <= tt <= t1), OFF_BAR_RGB)
        cols.append(c)
    objs = room_objects(cfg, env, info.get("hub"))
    vs = views(cfg)
    swept = swept_points(model, [q[k] for k in gk], series["tcp"])
    fig = plt.figure(figsize=(19.2, 11.0), dpi=100)
    gs = fig.add_gridspec(2, 3, height_ratios=[1.5, 1.0], hspace=0.14, wspace=0.1,
                          left=0.03, right=0.99, top=0.9, bottom=0.04)
    tone = lambda k: np.array((0.70, 0.80, 0.90)) * (1 - k) + np.array((0.22, 0.24, 0.36)) * k  # noqa: E731
    for i, view in enumerate(vs):
        ax = fig.add_subplot(gs[0, i])
        lims = view_limits(view, swept)
        draw_room(ax, view, lims, objs)
        img = None
        for n, k in enumerate(gk):
            src = render_pose(view, lims, model, q[k], tone(n / max(1, len(gk) - 1)), capsules=capsules, cell=cell)
            if img is None:
                img = np.zeros(src.shape[:2] + (4,))
            img = over(img, src, 0.34 if n else 0.5)
        ax.imshow(img, extent=(lims[0][0], lims[0][1], lims[1][0], lims[1][1]), origin="upper",
                  interpolation="bilinear", zorder=2)
        p2 = project(view, tcp)
        segs = np.stack([p2[:-1], p2[1:]], axis=1)
        ax.add_collection(LineCollection(segs, colors=cols[1:], linewidths=2.2, zorder=4))
        for n, k in enumerate(gk):                     # the strip and its LEDs' facing, the ghost's number
            a_, b_ = series["strip"][k]
            s2 = project(view, [a_, b_])
            ax.plot(s2[:, 0], s2[:, 1], color=(0.85, 0.1, 0.55), lw=1.6, alpha=0.75, zorder=5,
                    solid_capstyle="round")
            import show
            _, tip, d = show.tool_pose(q[k])
            mid = [(x + y) / 2.0 for x, y in zip(a_, b_)]
            m2, e2 = project(view, [mid, [m + 0.18 * dd for m, dd in zip(mid, d)]])
            ax.annotate("", xy=e2, xytext=m2, zorder=5,
                        arrowprops=dict(arrowstyle="-|>", color=(0.85, 0.1, 0.55), lw=0.9, alpha=0.8))
            t2 = project(view, [tcp[k]])[0]
            ax.text(t2[0], t2[1], " %d" % (n + 1), fontsize=7.5, color="black", zorder=6, weight="bold",
                    ha="left", va="bottom")
        ax.set_xlim(*lims[0])
        ax.set_ylim(*lims[1])
        ax.set_aspect("equal")
        ax.set_title(view[1], fontsize=9, loc="left")
        ax.tick_params(labelsize=6)
        ax.grid(alpha=0.15, lw=0.5)
    # timeline: speeds over the bars, joints' speed / acceleration % of their limits
    sub = gs[1, 0:2].subgridspec(3, 1, hspace=0.08)
    ax1 = fig.add_subplot(sub[0])
    for t0, t1, i, m in bars:
        ax1.axvspan(t0, t1, color=bar_colour(i, m, kind), alpha=0.22, lw=0)
        txt = ("%s" % (m or "?")) if kind == "family" else "%s -> %s" % (i, m or "?")
        ax1.text((t0 + t1) / 2, 0.97, txt, transform=ax1.get_xaxis_transform(), fontsize=7.5, ha="center",
                 va="top", color="#c62828" if (kind != "family" and i != m) else "#111111")
    ax1.plot(t, series["v_tcp"], color="black", lw=1.1, label="TCP")
    ax1.plot(t, series["v_tips"][0], color=(0.85, 0.1, 0.55), lw=0.8, alpha=0.8, label="strip tip a")
    ax1.plot(t, series["v_tips"][1], color=(0.55, 0.1, 0.85), lw=0.8, alpha=0.8, label="strip tip b")
    for n, k in enumerate(gk):
        ax1.axvline(t[k], color="#555555", lw=0.5, ls=":")
        ax1.text(t[k], 0.02, "%d" % (n + 1), transform=ax1.get_xaxis_transform(), fontsize=6.5, ha="center")
    for letter, tt in series.get("marks", []):           # a wind-up (A) before a move, an overshoot (O) after it
        ax1.text(tt, 0.80, letter, transform=ax1.get_xaxis_transform(), fontsize=8, weight="bold",
                 color="#1565c0", ha="center")
    ax1.set_ylabel("m/s", fontsize=7)
    ax1.legend(handles=[Line2D([0], [0], color=c, lw=2, label=l) for c, l in
                        (("black", "TCP"), ((0.85, 0.1, 0.55), "strip tip a"), ((0.55, 0.1, 0.85), "strip tip b"))],
               fontsize=6.5, loc="upper right", ncol=3, framealpha=0.7, bbox_to_anchor=(1.0, 0.85))
    jc = plt.get_cmap("tab10")
    for row, (key, lab) in enumerate((("vel_pct", "speed %"), ("acc_pct", "accel %"))):
        ax = fig.add_subplot(sub[row + 1], sharex=ax1)
        arr = series[key]
        for j in range(6):
            ax.plot(series["ts"], arr[:, j], color=jc(j), lw=0.8, label="J%d" % (j + 1))
        ax.axhline(100, color="#c62828", lw=0.7, ls="--")
        ax.set_ylabel(lab + " of limit", fontsize=7)
        ax.set_ylim(0, max(105, float(arr.max()) * 1.05))
        ax.tick_params(labelsize=6)
        if row == 0:
            ax.legend(handles=[Line2D([0], [0], color=jc(j), lw=2, label="J%d" % (j + 1)) for j in range(6)],
                      fontsize=6.5, ncol=6, loc="upper right", framealpha=0.7)
    for ax in fig.axes[3:]:
        ax.tick_params(labelsize=6)
        ax.set_xlim(0, t[-1])
    fig.axes[-1].set_xlabel("s", fontsize=7)
    # the numbers
    axt = fig.add_subplot(gs[1, 2])
    axt.axis("off")
    lines = text_lines(info)
    y = 1.0
    for txt, col, size in lines:
        axt.text(0.0, y, txt, fontsize=size, color=col, family="monospace", va="top", transform=axt.transAxes)
        y -= 0.052 * size / 8.0
    legend = [Line2D([0], [0], color=ACTION_RGB[k], lw=3, label=k) for k in ACTION_RGB]
    legend.append(Line2D([0], [0], color=OFF_BAR_RGB, lw=3, label="outside a bar"))
    legend.append(Line2D([0], [0], color=(0.85, 0.1, 0.55), lw=1.6, label="LED strip, -> LEDs face"))
    fig.legend(handles=legend, loc="upper left", ncol=11, fontsize=7.5, frameon=False,
               title="TCP path coloured by the %s" % ("MEASURED action per 4-beat bar (a gesture has no Laban intent)"
                                                     if kind == "family" else "INTENDED action per bar"),
               title_fontsize=7.5, bbox_to_anchor=(0.03, 0.965), alignment="left")
    fig.text(0.03, 0.98, "%s   %.1f s   %s" % (info["id"], info["duration_s"], info.get("source") or ""),
             fontsize=13, weight="bold", va="center")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=100)
    plt.close(fig)
    if quantize:
        quantize_png(path)
    return path


def quantize_png(path):
    """The sheet as a 256-colour PNG (Pillow's octree, no dither: the action
    colours stay within a few levels, ~1/4 of the size); left as it is
    without Pillow."""
    try:
        from PIL import Image
    except ImportError:
        return False
    im = Image.open(path).convert("RGB")
    im.quantize(colors=256, method=Image.Quantize.FASTOCTREE, dither=Image.Dither.NONE).save(path, optimize=True)
    return True


def text_lines(info):
    """[(text, colour, size)] for the sheet's number panel."""
    out = []
    gen = info.get("generator") or "?"
    out.append(("%s  hub %s  %s%s" % (gen, info.get("hub") or "-", ("family " + info["family"] + "  ")
                                       if info.get("family") else "", "bpm %s" % info.get("bpm")), "#111111", 8.5))
    if info.get("params"):
        p = info["params"]
        out.append(("params " + ", ".join("%s=%s" % (k, p[k]) for k in list(p)[:6]), "#444444", 7))
    if info["bars"]:
        if info["bars_kind"] == "family":
            out.append(("intent   %s (a gesture: no Laban intent)" % info["family"], "#111111", 8))
            out.append(("measured " + "-".join(b["measured"] or "?" for b in info["bars"]), "#111111", 8))
        else:
            ag = info["bars_agree"]
            out.append(("intent   " + "-".join(b["intent"] or "?" for b in info["bars"]) +
                        ("   (bars spread evenly: approx)" if info["bars_kind"] == "approx" else ""), "#111111", 8))
            out.append(("measured " + "-".join(b["measured"] or "?" for b in info["bars"]) + "   %d/%d agree" % tuple(ag),
                        "#2e7d32" if ag[0] == ag[1] else "#c62828", 8))
    m = info["measured"]
    if m.get("action"):
        out.append(("whole clip: %s  (W %+.2f T %+.2f S %+.2f F %+.2f)" % (
            m["action"], m.get("weight") or 0, m.get("time") or 0, m.get("space") or 0, m.get("flow") or 0),
            "#111111", 8))
    out.append(("", "#111111", 4))
    out.append(("            J1    J2    J3    J4    J5    J6", "#111111", 8))
    for key, lab in (("peak_vel_pct", "speed %"), ("peak_acc_pct", "accel %"), ("peak_jerk_pct", "jerk %")):
        vals = info.get(key)
        if vals:
            worst = max(vals)
            out.append(("%-9s" % lab + "".join("%6.0f" % x for x in vals), _pct_colour(worst), 8))
    if info.get("playback_scale") and info["playback_scale"] > 1.0:
        out.append(("player slows it x%.2f" % info["playback_scale"], "#c62828", 8))
    r = info["room"]
    cl = r["closest"] or {}
    out.append(("", "#111111", 4))
    out.append(("room min %s mm (%s to %s, t %s s)%s" % (
        "%.0f" % (1000 * r["min_clearance_m"]) if r["min_clearance_m"] is not None else "-",
        cl.get("link"), cl.get("with"), cl.get("t"), "" if r["ok"] else "  VIOLATES"),
        "#c62828" if not r["ok"] else "#111111", 8))
    if r.get("min_self_clearance_m") is not None:
        out.append(("self min %.0f mm (%s / %s)" % (1000 * r["min_self_clearance_m"], (r["closest_self"] or {}).get("link"),
                                                     (r["closest_self"] or {}).get("with")), "#444444", 7))
    f = info["flags"]
    out.append(("J5 crosses 0: %d  (min |J5| %.0f deg)   J3 crosses 0: %d" % (
        f["j5_zero_crossings"], f["j5_min_abs_deg"], f["j3_zero_crossings"]),
        "#c62828" if f["j5_zero_crossings"] or f["j3_zero_crossings"] else "#111111", 8))
    out.append(("J4 range %.0f  J6 range %.0f deg   wrist share %.0f %%" % (
        f["j4_range_deg"], f["j6_range_deg"], 100 * f["wrist_share"]), "#111111", 8))
    tw = f["twitches_per_joint"]
    out.append(("twitches J1-6 " + " ".join(str(x) for x in tw), "#ef6c00" if sum(tw) else "#111111", 8))
    if f.get("facing_audience_mean") is not None:
        out.append(("LEDs face the audience: mean %.2f, %.0f %% of the time > 0.5" % (
            f["facing_audience_mean"], 100 * f["facing_audience_share_over_half"]), "#111111", 8))
    tc = info["tcp"]
    out.append(("TCP z %.2f-%.2f m  extent %s m  peak %.2f m/s  strip tip %.2f m/s" % (
        tc["z_min"], tc["z_max"], "x".join("%.2f" % e for e in tc["extent_m"]), tc["peak_mps"],
        tc["strip_tip_peak_mps"]), "#111111", 7))
    x = info.get("expression")
    if x:
        out.append(("expression: lag %.2f s  accents %s/min  still %s  holds %d  dyn x%s" % (
            x["lag_s"] or 0.0, x["accents_per_min"], "%.0f%%" % (100 * x["still_share"]) if x["still_share"] is not None
            else "-", x["holds"], x["dynamic_range"]), "#1565c0", 7.5))
        out.append(("  %d big moves: A %d  O %d  onset cv %s  bar contrast %s  strip x%s  joints %d" % (
            x["big_moves"], x["anticipated"], x["overshot"], x["onset_cv"], x["bar_contrast"], x["strip_ratio"],
            x["joints_used"]), "#1565c0", 7.5))
    if info.get("show_labels", {}).get("energy") is not None:
        out.append(("show energy %.2f" % info["show_labels"]["energy"], "#444444", 7))
    out.append(("ghosts at " + " ".join("%.1f" % x for x in info.get("ghost_times_s", [])) + " s", "#444444", 7))
    return out


# --------------------------------------------------------------------------
# running
# --------------------------------------------------------------------------

def setup(show_path=SHOW):
    """(cfg, env with the paper, collision model) for a show config."""
    import show
    cfg = json.load(open(show_path)) if show_path else {}
    env = C.load_env(os.path.join(ROOT, cfg.get("env", "envs/volvox_lab.usda")))
    if cfg.get("canvas"):
        env = show.show_env(env, cfg, cfg["margins"]["idle_canvas_m"])
    return cfg, env, C.load_model("fr20")


def review(clips, out_dir=OUT, show_path=SHOW, n_ghosts=GHOSTS, capsules=False, cell=MESH_CELL_M, log=print,
           quantize=True):
    cfg, env, model = setup(show_path)
    done = []
    for clip in clips:
        info, series = measure_clip(clip, cfg, env, model)
        png = os.path.join(out_dir, clip["id"] + ".png")
        info["image"] = os.path.relpath(png, ROOT).replace("\\", "/")
        info["show"] = os.path.relpath(show_path, ROOT).replace("\\", "/") if show_path else None
        info["model"] = "capsules (collision.py)" if capsules else "URDF STL, vertex clustering %.0f mm" % (1000 * cell)
        render_sheet(clip, info, series, cfg, env, model, png, n_ghosts, capsules, cell, quantize)
        with open(os.path.join(out_dir, clip["id"] + ".labels.json"), "w") as f:
            json.dump(info, f, indent=1)
        log("%s: %s" % (clip["id"], info["image"]))
        done.append(info)
    return done


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="clip .json, directory / manifest.json, *.compiled.json, .csv")
    ap.add_argument("--ids", nargs="*", help="only these clip ids")
    ap.add_argument("--kinds", default="idle", help="segment kinds of a compiled show / manifest (comma separated)")
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--show", default=SHOW, help="the show config: its room, paper and audience")
    ap.add_argument("--ghosts", type=int, default=GHOSTS, help="ghost poses per view (8-12)")
    ap.add_argument("--capsules", action="store_true", help="draw the collision capsules, not the STL meshes")
    ap.add_argument("--mesh-cell", type=float, default=MESH_CELL_M, help="decimation cell (m)")
    ap.add_argument("--no-quantize", action="store_true", help="keep the full-colour PNG (~4x the size)")
    a = ap.parse_args(argv)
    if not 8 <= a.ghosts <= 12:
        ap.error("--ghosts: 8-12")
    paths = []
    for p in a.inputs:
        paths += sorted(glob.glob(p)) or [p]
    clips = load_inputs(paths, set(a.ids) if a.ids else None, tuple(a.kinds.split(",")))
    if not clips:
        ap.error("no clips in %s" % a.inputs)
    review(clips, a.out, a.show, a.ghosts, a.capsules, a.mesh_cell, quantize=not a.no_quantize)
    return 0


def self_test():
    import numpy as np
    ok = True

    def check(cond, what):
        nonlocal ok
        print(("ok   " if cond else "FAIL ") + what)
        ok = ok and bool(cond)

    # decimation keeps the shape: bounds within a cell, far fewer triangles
    tri = read_stl(os.path.join(ROOT, "assets/fairino_description/meshes/fairino20_v6/upperarm_link.STL"))
    v, f = decimate(tri, MESH_CELL_M)
    lo, hi = tri.reshape(-1, 3).min(0), tri.reshape(-1, 3).max(0)
    check(len(f) < len(tri) / 5 and np.all(v.min(0) >= lo - 1e-9) and np.all(v.max(0) <= hi + 1e-9)
          and np.all(np.abs(v.min(0) - lo) < MESH_CELL_M) and np.all(np.abs(v.max(0) - hi) < MESH_CELL_M),
          "decimate: %d -> %d triangles, bounds within a cell" % (len(tri), len(f)))
    # the meshes sit on the FK where the capsules do: the meshes' centroid per link near its capsules'
    model = C.load_model("fr20")
    qh = [-60.0, -90.0, 90.0, -90.0, -90.0, -15.0]
    trs = posed_triangles(model, qh)
    caps = posed_capsules(model, qh, tool=False)
    cpts = np.array([c[1] for c in caps] + [c[2] for c in caps])
    check(np.all(trs.reshape(-1, 3).min(0) > cpts.min(0) - 0.25) and np.all(trs.reshape(-1, 3).max(0) < cpts.max(0) + 0.25),
          "meshes and capsules agree on where the arm is")
    # stadium: a horizontal capsule's outline spans its ends +- r
    st = np.array(stadium((0.0, 0.0), (1.0, 0.0), 0.1))
    check(abs(st[:, 0].min() + 0.1) < 1e-6 and abs(st[:, 0].max() - 1.1) < 1e-6 and abs(st[:, 1].max() - 0.1) < 1e-6,
          "stadium outline")
    # compositing: half over transparent keeps colour, alpha halves
    dst = np.zeros((2, 2, 4))
    src = np.zeros((2, 2, 4), np.uint8)
    src[..., 0], src[..., 3] = 255, 255
    o = over(dst, src, 0.5)
    check(abs(o[0, 0, 3] - 0.5) < 1e-9 and abs(o[0, 0, 0] - 1.0) < 1e-9, "alpha over")
    # a known motion: J1 a sine of amplitude A, frequency f -> peak speed A 2 pi f
    A, fr, dur = 30.0, 0.25, 8.0
    pts = []
    for k in range(int(dur * 24) + 1):
        tt = k / 24.0
        qq = list(qh)
        qq[0] += A / (2 * math.pi * fr) * (1.0 - math.cos(2 * math.pi * fr * tt))   # speed A sin: at rest at both ends
        qq[4] += 8.0 * math.sin(2 * math.pi * 0.5 * tt)
        pts.append({"t": tt, "q": qq})
    clip = {"schema": M.SCHEMA, "id": "selftest_sine", "robot": "fr20", "joint_names": ["j%d" % i for i in range(1, 7)],
            "points": pts, "meta": {"duration_s": dur}, "labels": {}, "style": {}}
    cfg, env, model = setup(SHOW)
    info, series = measure_clip(clip, cfg, env, model)
    want = 100.0 * A / RP.velocity_limits(RP.load("fr20"))[0]
    check(abs(info["peak_vel_pct"][0] - want) < 0.05 * want, "J1 peak speed %.1f %% (expected %.1f)"
          % (info["peak_vel_pct"][0], want))
    check(info["flags"]["j5_zero_crossings"] == 0 and sum(info["flags"]["twitches_per_joint"]) == 0,
          "a smooth sine: no J5 crossing, no twitch")
    check(info["room"]["min_clearance_m"] is not None and info["room"]["min_clearance_m"] > 0,
          "room clearance measured (%s m, %s)" % (info["room"]["min_clearance_m"], info["room"]["closest"]))
    # a twitch is found: a joint jittering back and forth by a degree
    qs = [[math.sin(k / 3.0) * 0.6, 0, 0, 0, 0, 0] for k in range(100)]
    vv = np.gradient(np.array(qs), 0.008, axis=0).tolist()
    check(twitches(qs, vv, 0.008)[0] >= 5, "a jitter counts as twitches")
    gt = pick_ghosts(series["t"], series["watched"], 10)
    check(len(gt) == 10 and gt[0] == 0.0 and gt == sorted(gt), "10 ghosts, the start first")
    # expression: a move with a wind-up before it and an overshoot after it is found as such
    tt = [k / 24.0 for k in range(24 * 6)]

    def ease(u):
        u = max(0.0, min(1.0, u))
        return u * u * u * (10 - 15 * u + 6 * u * u)
    xs = []
    for x_ in tt:
        x = -0.04 * ease((x_ - 0.6) / 0.4)                          # back 4 cm
        x += 0.54 * ease((x_ - 1.1) / 0.8)                          # out 54 cm
        x -= 0.06 * ease((x_ - 1.95) / 0.4)                         # settle back 6 cm
        x += -0.44 * ease((x_ - 3.5) / 1.0)                         # home, plainly
        xs.append(x)
    path = [(x, 0.0, 1.0) for x in xs]
    sp = [0.0] + [abs(xs[i] - xs[i - 1]) * 24.0 for i in range(1, len(xs))]
    pts_ = [{"t": a, "q": list(qh)} for a in tt]
    ex = expression({"points": pts_, "tcp": path}, tt, path, sp, [path, path], [])
    check(ex["big_moves"] == 2 and ex["anticipated"] == 1 and ex["overshot"] == 1 and ex["holds"] == 1,
          "expression: a wind-up and an overshoot found (%s)" % {k: ex[k] for k in ("big_moves", "anticipated",
                                                                                     "overshot", "holds")})
    even = [(0.1 * (1.0 - math.cos(2 * math.pi * x_)), 0.0, 1.0) for x_ in tt]   # the same swing, the same beat
    sp2 = [0.0] + [math.dist(even[i], even[i - 1]) * 24.0 for i in range(1, len(even))]
    ex2 = expression({"points": pts_, "tcp": even}, tt, even, sp2, [even, even], [])
    check(ex2["onset_cv"] is not None and ex2["onset_cv"] < 0.05 and ex2["anticipated"] == 0,
          "expression: an even swing is metronomic (onset cv %s)" % ex2["onset_cv"])
    # family bars: 4 beats at the bpm
    gb = gesture_bars(clip, {"bpm": 120, "family": "wave"})
    check(len(gb) == 4 and all(abs((b[1] - b[0]) - 2.0) < 1e-6 for b in gb), "a gesture's 4-beat bars")
    # a halfspace edge on is a line, face on none
    fv = views(cfg)[0]
    lims = view_limits(fv)
    check(plane_line(fv, {"normal": [0, 0, 1], "offset": 0.0}, lims) is not None
          and plane_line(fv, {"normal": [1, 0, 0], "offset": -1.0}, lims) is None, "walls drawn edge on only")
    # both renderings write a sheet and its labels
    with tempfile.TemporaryDirectory() as d:
        for caps_ in (False, True):
            sub = os.path.join(d, "c" if caps_ else "m")
            got = review([dict(clip)], sub, SHOW, 8, caps_, log=lambda *x: None)
            png = os.path.join(sub, "selftest_sine.png")
            lab = json.load(open(os.path.join(sub, "selftest_sine.labels.json")))
            check(os.path.getsize(png) > 50000 and lab["id"] == "selftest_sine" and len(lab["ghost_times_s"]) == 8
                  and got, "sheet + labels (%s)" % ("capsules" if caps_ else "meshes"))
    # quantizing keeps the action colours (swatches on a gradient, as on a sheet)
    try:
        from PIL import Image
        arr = np.zeros((120, 400, 3), np.uint8)
        arr[...] = np.linspace(0, 255, 400, dtype=np.uint8)[None, :, None]
        for k, rgb in enumerate(ACTION_RGB.values()):
            arr[10:40, 10 + 45 * k:40 + 45 * k] = [int(255 * c) for c in rgb]
        with tempfile.TemporaryDirectory() as d:
            pth = os.path.join(d, "q.png")
            Image.fromarray(arr).save(pth)
            quantize_png(pth)
            back = np.asarray(Image.open(pth).convert("RGB")).astype(int)
            err = max(int(np.abs(back[25, 25 + 45 * k] - arr[25, 25 + 45 * k]).max()) for k in range(len(ACTION_RGB)))
        check(err <= 10, "quantized PNG keeps the action colours (worst %d levels)" % err)
    except ImportError:
        print("skip quantize (no Pillow)")
    # the fixtures load: a clip, a CSV
    fx = load_inputs([os.path.join(ROOT, "tests/clips/d01_punch-punch.json"),
                      os.path.join(ROOT, "tests/csv/dance_d01_punch-punch.csv")])
    check(len(fx) == 2 and clip_bars(fx[0])[1] == "laban" and clip_bars(fx[1])[1] == "none", "fixtures load")
    print("self-test %s" % ("passed" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
