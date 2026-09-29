"""The show's clip library reviewed in Isaac Sim: every idle clip (and the
big wipes, the scan) played one after the other in the lab's room, physics
on, each with its metadata burnt into the corner (overlay.clip_card: name,
length, hub, family / intent, action, bpm, energy, motion stats, the room's
clearance, the simulation's tracking and contacts) -- the Houdini review
(render_clip_review.py) of the show's library, in Isaac's look.

    C:/isaacsim6/python.bat scripts/isaac/record_library.py --headless
    C:/isaacsim6/python.bat scripts/isaac/record_library.py --headless --clips greet_00_look,scan --cameras room
    python scripts/isaac/record_library.py --self-test

A new scan before the library is built again (show.py build --scan-only
writes geo/show/<show>_scan/compiled.json): any segments of any compiled
show, in order, the cameras side by side (<show>_segments.mp4), or one
picture a camera (--still, to tune the look):

    uv run scripts/show.py build shows/party.json --scan-only
    C:/isaacsim6/python.bat scripts/isaac/record_library.py --headless --graph geo/show/party_scan/compiled.json \
        --segments to_scan,scan,from_scan --no-pages
    C:/isaacsim6/python.bat scripts/isaac/record_library.py --headless --segments scan --still 2.0

Each clip: a cut to its first pose, HOLD_IN_S still (the card to read), the
clip, HOLD_OUT_S still. One pass per camera (--cameras, isaac_stage.cameras:
room, audience, side); the room as every Isaac view (load_room, the lab's
look, the guides hidden unless --guides). Out (--out,
geo/isaac/review/): <show>_library_<camera>.mp4, <show>_library_<camera>_sheet.jpg
(a frame 40 % into each clip, labelled), <show>_library.json (chapters: where
each clip starts in the video, its card, tracking and contacts per camera).
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
sys.path.insert(0, HERE)

FPS_VIDEO = 30
HOLD_IN_S = 0.6                  # the first pose, still: the card to read, the picture settled after the cut
HOLD_OUT_S = 0.3
SETTLE_STEPS = 30                # physics steps at the new pose after a cut, not drawn
POSTER_AT = 0.4                  # the contact sheet's frame, this far into the clip (as the Houdini review)
FONT = "C:/Windows/Fonts/consola.ttf"


# --------------------------------------------------------------------------
# what is played, when (pure)
# --------------------------------------------------------------------------

def playlist(graph, extra=None, extra_names=(), scan=True, only=None):
    """[(segment, source)]: the graph's idle clips in its order, then the
    extra graph's named clips (e.g. party_bigwipe's big wipes), then the scan;
    only: names to keep (a quick look)."""
    out = [(s, "main") for s in graph.idle()]
    if extra is not None:
        by = {s.name: s for s in extra.segments}
        missing = [n for n in extra_names if n not in by]
        if missing:
            raise SystemExit("not in the extra show: %s" % ", ".join(missing))
        out += [(by[n], "extra") for n in extra_names]
    if scan and any(s.kind == "scan" for s in graph.segments):
        out.append((graph.one("scan"), "main"))
    if only:
        out = [x for x in out if x[0].name in only]
        if not out:
            raise SystemExit("none of %s in the library" % ", ".join(sorted(only)))
    return out


def pick_segments(graph, names):
    """[(segment, "main")] of the named segments of a compiled show, in the
    order given (any kind: moves, to_scan, the scan ...)."""
    by = {s.name: s for s in graph.segments}
    missing = [n for n in names if n not in by]
    if missing:
        raise SystemExit("not in the show: %s (there: %s)" % (", ".join(missing), ", ".join(sorted(by))))
    return [(by[n], "main") for n in names]


def spans(durations, dt, fps=FPS_VIDEO, hold_in=HOLD_IN_S, hold_out=HOLD_OUT_S):
    """Per clip: physics steps (hold, clip, hold), the frames shot (every
    1/fps: steps 0, every, 2 every, ...), the video frame it starts at, and the
    poster frame (POSTER_AT into the clip itself)."""
    every = int(round(1.0 / (fps * dt)))
    out, first = [], 0
    for d in durations:
        steps = int(math.ceil((hold_in + d + hold_out) / dt))
        frames = (steps + every - 1) // every
        poster = first + min(frames - 1, int(round((hold_in + POSTER_AT * d) * fps)))
        out.append({"steps": steps, "frames": frames, "first": first, "poster": poster, "every": every})
        first += frames
    return out


def sheet_label(card):
    """The contact sheet's caption of a clip: its name and length, then the
    card's second line (hub, family, action); ImageMagick's % and \\ escaped."""
    text = card[0].rsplit("   ", 1)[0] if card[0].count("   ") > 1 else card[0]
    text += "\n" + card[1]
    return text.replace("\\", "\\\\").replace("%", "%%")


VERSION = "v9"                   # the library: v8's clips, the big wipes, the scan top to bottom on the floor canvas
PAGE = (4, 2)                    # the Houdini review's pages and overview (render_clip_review.build_videos)
OVERVIEW = (5, 5)
TILE_WH = (640, 360)             # the frame whole (16:9): a wide strip is never cut
OVERVIEW_TILE = (448, 252)           # even sides (yuv420p)


def tile_span(first, frames, duration, fps=FPS_VIDEO, hold_in=HOLD_IN_S):
    """(first frame, frame count) of the clip itself in its pass: after the
    held first pose, its length (+1 frame), inside what was shot."""
    start = first + int(round(hold_in * fps))
    return start, max(1, min(first + frames - start, int(round(duration * fps)) + 1))


def sim_line(version, cam, sim):
    """A tile's foot: the library's version, the camera, the run in Isaac."""
    return "%s  isaac %s   track %.2f deg   contacts %d" % (version, cam, sim["track_deg"], sim["contacts"])


def preview_path(name, src, show_dirs):
    """The show's preview of a clip (show.write_preview, what the Houdini
    review reads): geo/show/<show>/<name>.json, the extra show's for its clips."""
    return os.path.join(show_dirs[src], name + ".json")


def review_pages(frames, sp, plist, sims, cam, out_dir, show_dirs, version=VERSION):
    """The Houdini review's grid from Isaac's frames: a tile per clip (a
    square from the middle of the frame; render_clip_review's header -- id,
    length, verdict, action, acc % per joint, vel, room, wrist, bpm -- and
    timecode; the version, camera and Isaac's tracking at its foot), pages of
    PAGE and every clip in overview pages of OVERVIEW. [video paths]."""
    sys.path.insert(0, SCRIPTS)
    import render_clip_review as RCR
    w, h = TILE_WH
    tdir = os.path.join(out_dir, "tiles_" + cam)
    shutil.rmtree(tdir, ignore_errors=True)
    os.makedirs(tdir)
    tiles = []
    for k, ((seg, src), x, sim) in enumerate(zip(plist, sp, sims)):
        clip = json.load(open(preview_path(seg.name, src, show_dirs)))
        start, n = tile_span(x["first"], x["frames"], seg.duration)
        text = os.path.join(tdir, "text_%02d" % (k + 1))
        vf = RCR.tile_filter(clip, text, w, h)
        foot = os.path.join(text, "foot.txt")
        with open(foot, "w", encoding="utf-8") as f:
            f.write(sim_line(version, cam, sim))
        fs = max(10, int(w / 36))
        vf += (",drawtext=fontfile='%s':textfile='%s':expansion=none:x=6:y=h-%d:fontsize=%d:fontcolor=0xd0d0d0"
               ":box=1:boxcolor=black@0.55:boxborderw=3"
               % (RCR.FONT, RCR._esc(foot), int(fs * 3.2), fs))             # above the dance's bar line
        tile = os.path.join(tdir, "%02d_%s.mp4" % (k + 1, seg.name))
        r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS_VIDEO), "-start_number", str(start),
                            "-i", os.path.join(frames, "f_%05d.png"), "-frames:v", str(n), "-vf", vf,
                            "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", tile], capture_output=True, text=True)
        if r.returncode:
            print("[library] tile %s: %s" % (seg.name, r.stderr[-300:]))
            continue
        tiles.append(tile)
    out = []
    for k, grp in enumerate(RCR.pages(tiles, PAGE[0] * PAGE[1])):
        out.append(RCR.stack(grp, PAGE[0], PAGE[1], w, h, os.path.join(out_dir, "page_%s_%d.mp4" % (cam, k + 1))))
    for k, grp in enumerate(RCR.pages(tiles, OVERVIEW[0] * OVERVIEW[1])):
        out.append(RCR.stack(grp, OVERVIEW[0], OVERVIEW[1], OVERVIEW_TILE[0], OVERVIEW_TILE[1],
                             os.path.join(out_dir, "overview_%s_%d.mp4" % (cam, k + 1)), crf=24))
    return [os.path.relpath(x, ROOT).replace("\\", "/") for x in out]


def self_test():
    import show
    fails = []

    def check(name, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", name, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(name)

    dt = 1.0 / 120.0
    sp = spans([2.0, 3.5], dt)
    check("a clip's steps: the holds and the clip", sp[0]["steps"] == int(math.ceil((HOLD_IN_S + 2.0 + HOLD_OUT_S) / dt)), sp)
    check("frames every 4th step at 30 fps / 120 Hz, the next clip starts where the last ended",
          sp[0]["every"] == 4 and sp[0]["frames"] == (sp[0]["steps"] + 3) // 4 and sp[1]["first"] == sp[0]["frames"], sp)
    check("the poster inside its clip, 40 % in",
          sp[1]["first"] < sp[1]["poster"] < sp[1]["first"] + sp[1]["frames"]
          and sp[1]["poster"] - sp[1]["first"] == round((HOLD_IN_S + 0.4 * 3.5) * 30), sp)
    g = show.Graph.load(os.path.join(ROOT, "shows", "party.compiled.json"))
    x = show.Graph.load(os.path.join(ROOT, "shows", "party_bigwipe.compiled.json"))
    pl = playlist(g, x, ("low_wipe_rows", "greet_wipe_cols"))
    names = [s.name for s, _ in pl]
    check("the library: every idle clip of party, the two big wipes, the scan last",
          len(names) == len(g.idle()) + 3 and names[-1] == "scan" and names[-3:-1] == ["low_wipe_rows", "greet_wipe_cols"]
          and all(s.kind == "idle" for s, _ in pl[:-1]), (len(names), names[-4:]))
    check("a quick look keeps the order", [s.name for s, _ in playlist(g, x, ("low_wipe_rows",), only={"scan", "rest_02_glide"})]
          == ["rest_02_glide", "scan"])
    lab = sheet_label(["greet_16_wipe   11.92 s   17/73", "hub greet   family wipe   action slash"])
    check("the sheet's caption: name, length, hub and family, no library index",
          lab == "greet_16_wipe   11.92 s\nhub greet   family wipe   action slash", lab)
    check("... ImageMagick's escapes doubled", sheet_label(["a 5%", "b"]) == "a 5%%\nb")
    check("a tile is the clip itself: after the held first pose, its length + 1 frame",
          tile_span(100, 90, 2.0) == (118, 61) and tile_span(0, 20, 5.0) == (18, 2), (tile_span(100, 90, 2.0),))
    check("a tile's foot: version, camera, Isaac's tracking and contacts",
          sim_line("v9", "room", {"track_deg": 0.167, "contacts": 0}) == "v9  isaac room   track 0.17 deg   contacts 0")
    picked = [x.name for x, _ in pick_segments(g, ["to_scan", "scan", "from_scan"])]
    check("any segments of a show, in the order given (a scan's preview)", picked == ["to_scan", "scan", "from_scan"],
          picked)
    try:
        pick_segments(g, ["scan", "no_such"])
        refused = False
    except SystemExit as e:
        refused = "no_such" in str(e)
    check("... one it does not have is refused, by name", refused)
    dirs = {"main": os.path.join(ROOT, "geo", "show", "party"), "extra": os.path.join(ROOT, "geo", "show", "party_bigwipe")}
    miss = [s.name for s, src in pl if not os.path.exists(preview_path(s.name, src, dirs))]
    check("every clip of the library has its preview (the review's header comes from it)", not miss, miss[:5])
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__" and "--self-test" in sys.argv:
    sys.exit(self_test())

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=os.path.join(ROOT, "shows", "party.json"))
ap.add_argument("--extra", default=os.path.join(ROOT, "shows", "party_bigwipe.json"),
                help="another show whose --extra-clips join the review ('' for none)")
ap.add_argument("--extra-clips", default="low_wipe_rows,greet_wipe_cols")
ap.add_argument("--no-scan", action="store_true", help="leave the scan out")
ap.add_argument("--clips", default="", help="only these (comma separated), a quick look")
ap.add_argument("--cameras", default="room,audience", help="room, audience, side")
ap.add_argument("--guides", action="store_true", help="draw the safety guides (zones' outlines); hidden by default")
ap.add_argument("--look", default="room", choices=("room", "plain"))
ap.add_argument("--out", default=os.path.join(ROOT, "geo", "isaac", "review"))
ap.add_argument("--headless", action="store_true")
ap.add_argument("--no-sequence", action="store_true", help="only the review pages, not the clips one after another")
ap.add_argument("--version", default=VERSION, help="the library's version, on every tile and the output folder")
ap.add_argument("--graph", default="", help="take the segments from this compiled show (e.g. a --scan-only build's "
                                             "geo/show/<show>_scan/compiled.json) instead of the show's own")
ap.add_argument("--segments", default="", help="these segments (comma separated, any kind, in order) instead of the "
                                                "library; the cameras side by side in <show>_segments.mp4")
ap.add_argument("--still", type=float, default=-1.0, help="only a PNG a camera, this far (s) into the first clip")
ap.add_argument("--no-pages", action="store_true", help="no review pages (a quick look)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720, "renderer": "RaytracedLighting"})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

import overlay  # noqa: E402
import show  # noqa: E402
from isaac_stage import (PHYSICS_DT, attach_tool, cameras, contact_paths, encode_video, import_robot,  # noqa: E402
                         load_room, render_settings, use_camera)


def contact_sheet(posters, labels, path):
    """ImageMagick montage: one frame per clip, captioned."""
    cols = int(math.ceil(math.sqrt(len(posters) * 16.0 / 9.0 / 1.2)))
    cmd = ["magick", "montage"]
    for p, lab in zip(posters, labels):
        cmd += ["-label", lab, p]
    cmd += ["-tile", "%dx" % cols, "-geometry", "320x180+4+4", "-background", "#202224", "-fill", "white",
            "-font", FONT, "-pointsize", "12", "-quality", "88", path]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        print("[library] montage: %s" % r.stderr[-400:])
        return None
    return os.path.relpath(path, ROOT).replace("\\", "/")


def main():
    cfg_path = os.path.abspath(args.config)
    cfg = json.load(open(cfg_path))
    name = os.path.splitext(os.path.basename(cfg_path))[0]
    gpath = os.path.abspath(args.graph) if args.graph else show.compiled_path(cfg_path)
    graph = show.Graph.load(gpath)
    extra = show.Graph.load(show.compiled_path(os.path.abspath(args.extra))) if args.extra else None
    names = [n for n in args.extra_clips.split(",") if n] if extra is not None else []
    only = set(n for n in args.clips.split(",") if n) or None
    plist = (pick_segments(graph, [n for n in args.segments.split(",") if n]) if args.segments
             else playlist(graph, extra, names, not args.no_scan, only))
    show_dirs = {"main": os.path.dirname(gpath) if args.graph else os.path.join(ROOT, "geo", "show", name),
                 "extra": os.path.join(ROOT, "geo", "show", os.path.splitext(os.path.basename(args.extra))[0])
                 if args.extra else None}
    segs = [s for s, _ in plist]
    sp = spans([s.duration for s in segs], PHYSICS_DT)
    print("[library] %d clips, %.0f s of video a camera" % (len(segs), sum(x["frames"] for x in sp) / float(FPS_VIDEO)),
          flush=True)

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    env = load_room(stage, cfg_path, args.look, guides=args.guides)
    prim_path = import_robot()
    attach_tool(stage)
    robot = world.scene.add(SingleArticulation(prim_path, name="fr20"))
    viz = None
    if cfg.get("scan") and any(s.kind == "scan" for s in segs):
        from scan_viz import ScanViz
        viz = ScanViz(stage, cfg)
    world.reset()
    dof = list(robot.dof_names)
    idx = np.array([dof.index("j%d" % i) for i in range(1, 7)])

    current, contacts = [None], []
    try:
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType

        def on_contact(headers, data):
            if current[0] is None:
                return                                              # a cut, not a clip
            for h in headers:
                if h.type == ContactEventType.CONTACT_FOUND:
                    a, b = contact_paths(h)
                    if ("/Room" in a) != ("/Room" in b):
                        contacts.append((current[0], a, b))
        sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)  # noqa: F841
    except Exception as e:
        print("[library] contact reports unavailable: %s" % e)

    from omni.kit.viewport.utility import capture_viewport_to_file
    render_settings()
    cams = cameras(cfg, env)
    os.makedirs(args.out, exist_ok=True)
    total = len(segs)
    summary = {"config": os.path.relpath(cfg_path, ROOT).replace("\\", "/"),
               "extra": os.path.relpath(os.path.abspath(args.extra), ROOT).replace("\\", "/") if extra else None,
               "fps": FPS_VIDEO, "hold_in_s": HOLD_IN_S, "hold_out_s": HOLD_OUT_S,
               "clips": [{"name": s.name, "kind": s.kind, "hub": s.start, "source": src, "duration_s": round(s.duration, 3),
                          "video_start_s": round(x["first"] / float(FPS_VIDEO), 3), "card":
                          overlay.clip_card(s.name, s.kind, s.start, s.duration, s.labels, k + 1, total), "sim": {}}
                         for k, ((s, src), x) in enumerate(zip(plist, sp))],
               "passes": {}}
    tag = "segments" if args.segments else "library"         # a scan's preview does not overwrite the library
    videos = []
    for cam_name in args.cameras.split(","):
        eye, look, focal = cams[cam_name]
        vp = use_camera(stage, "/World/Cam_" + cam_name, eye, look, focal)
        frames = os.path.join(args.out, "_frames_%s_%s" % (name, cam_name))
        shutil.rmtree(frames, ignore_errors=True)
        os.makedirs(frames)
        shot, corner, cards, sims, still = 0, [], [], [], None
        for k, (seg, x) in enumerate(zip(segs, sp)):
            q0 = seg.q[0]
            current[0] = None
            robot.set_joint_positions(np.radians(q0), joint_indices=idx)
            robot.set_joint_velocities(np.zeros(6), joint_indices=idx)
            if viz is not None:
                viz.update(-1.0, False, q0)
            for i in range(SETTLE_STEPS + (60 if k == 0 else 10)):     # still, then the picture drawn afresh
                robot.apply_action(ArticulationAction(joint_positions=np.radians(q0), joint_indices=idx))
                world.step(render=i >= SETTLE_STEPS)
            current[0] = k
            del contacts[:]
            worst = 0.0
            if seg.kind == "scan":
                on0, on1 = seg.labels["led_on_s"]
                u_of = seg.labels["u"]
            for i in range(x["steps"]):
                s = min(max(i * PHYSICS_DT - HOLD_IN_S, 0.0), seg.duration)
                q = seg.at(s)
                robot.apply_action(ArticulationAction(joint_positions=np.radians(q), joint_indices=idx))
                shoot = i % x["every"] == 0
                if viz is not None and seg.kind == "scan" and shoot:
                    playing = 0.0 < i * PHYSICS_DT - HOLD_IN_S < seg.duration
                    u = (u_of[min(len(u_of) - 1, int(s / seg.duration * (len(u_of) - 1)))] if playing
                         else (-1.0 if s <= 0.0 else 2.0))
                    viz.update(u, playing and on0 <= s <= on1, q)
                world.step(render=shoot or not args.headless)
                if shoot:
                    capture_viewport_to_file(vp, os.path.join(frames, "f_%05d.png" % shot))
                    shot += 1
                sim = np.degrees(robot.get_joint_positions(joint_indices=idx))
                worst = max(worst, max(abs(float(sim[j]) - q[j]) for j in range(6)))
                if args.still >= 0 and i * PHYSICS_DT >= HOLD_IN_S + args.still:    # one converged picture
                    for _ in range(40):
                        world.step(render=True)
                    still = os.path.join(args.out, "%s_%s_still.png" % (name, cam_name))
                    capture_viewport_to_file(vp, still)
                    for _ in range(20):
                        app.update()
                    break
            if still:
                print("[library] still %s" % still, flush=True)
                break
            sims.append({"track_deg": round(worst, 3), "contacts": len(contacts),
                         "first_contacts": [c[1:] for c in contacts[:3]]})
            card = overlay.clip_card(seg.name, seg.kind, seg.start, seg.duration, seg.labels, k + 1, total, sims[-1])
            cards.append(card)
            corner.append((x["first"] / float(FPS_VIDEO), overlay.block(card)))
            print("[library] %s %d/%d %s: tracking %.2f deg, contacts %d"
                  % (cam_name, k + 1, total, seg.name, worst, len(contacts)), flush=True)
        current[0] = None
        if still:
            shutil.rmtree(frames, ignore_errors=True)
            continue
        for _ in range(60):
            app.update()                                            # the last captures written
        posters_dir = os.path.join(args.out, "_posters_%s_%s" % (name, cam_name))
        shutil.rmtree(posters_dir, ignore_errors=True)
        os.makedirs(posters_dir)
        posters = []
        for k, (seg, x) in enumerate(zip(segs, sp)):
            p = os.path.join(posters_dir, "%02d_%s.png" % (k + 1, seg.name))
            shutil.copy(os.path.join(frames, "f_%05d.png" % x["poster"]), p)
            posters.append(p)
        sheet = contact_sheet(posters, [sheet_label(c) for c in cards],
                              os.path.join(args.out, "%s_%s_%s_sheet.jpg" % (name, tag, cam_name)))
        if sheet:
            shutil.rmtree(posters_dir, ignore_errors=True)
        vdir = os.path.join(args.out, args.version)
        os.makedirs(vdir, exist_ok=True)
        grid = [] if args.no_pages else review_pages(frames, sp, plist, sims, cam_name, vdir, show_dirs, args.version)
        if grid:
            print("[library] %s pages: %s" % (cam_name, ", ".join(grid)), flush=True)
        video = None
        if args.no_sequence:
            shutil.rmtree(frames, ignore_errors=True)
        else:
            mp4 = os.path.join(args.out, "%s_%s_%s.mp4" % (name, tag, cam_name))
            video = encode_video(app, frames, corner, shot, mp4, FPS_VIDEO, (1280, 720), "library")
            if video:
                videos.append(mp4)
        for c, sim in zip(summary["clips"], sims):
            c["sim"][cam_name] = sim
        summary["passes"][cam_name] = {"video": video, "pages": grid, "sheet": sheet, "frames": shot,
                                       "tracking_max_deg": max(s["track_deg"] for s in sims),
                                       "clips_with_contacts": [c["name"] for c, s in zip(summary["clips"], sims)
                                                               if s["contacts"]]}
        print("[library] %s: %s, sheet %s" % (cam_name, video, sheet), flush=True)
    if args.segments and len(videos) > 1:                    # the cameras side by side
        both = os.path.join(args.out, "%s_segments.mp4" % name)
        r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + sum((["-i", v] for v in videos), []) +
                           ["-filter_complex", "hstack=inputs=%d" % len(videos), "-pix_fmt", "yuv420p",
                            "-c:v", "libx264", "-crf", "22", both], capture_output=True, text=True)
        summary["side_by_side"] = os.path.relpath(both, ROOT).replace("\\", "/") if not r.returncode else r.stderr[-300:]
        print("[library] side by side: %s" % summary["side_by_side"], flush=True)
    out_json = os.path.join(args.out, "%s_%s.json" % (name, tag))
    json.dump(summary, open(out_json, "w"), indent=1)
    print("[library] %s" % os.path.relpath(out_json, ROOT), flush=True)


main()
app.close()
