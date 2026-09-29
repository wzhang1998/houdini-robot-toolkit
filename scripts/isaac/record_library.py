"""The show's clip library reviewed in Isaac Sim: every idle clip (and the
big wipes, the scan) played one after the other in the lab's room, physics
on, each with its metadata burnt into the corner (overlay.clip_card: name,
length, hub, family / intent, action, bpm, energy, motion stats, the room's
clearance, the simulation's tracking and contacts) -- the Houdini review
(render_clip_review.py) of the show's library, in Isaac's look.

    C:/isaacsim6/python.bat scripts/isaac/record_library.py --headless
    C:/isaacsim6/python.bat scripts/isaac/record_library.py --headless --clips greet_00_look,scan --cameras room
    python scripts/isaac/record_library.py --self-test

Each clip: a cut to its first pose, HOLD_IN_S still (the card to read), the
clip, HOLD_OUT_S still. One pass per camera (--cameras, isaac_stage.cameras:
room, audience, side); the room as record_segments draws it (load_room, the
lab's look, the guides hidden unless --guides). Out (--out,
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
    graph = show.Graph.load(show.compiled_path(cfg_path))
    extra = show.Graph.load(show.compiled_path(os.path.abspath(args.extra))) if args.extra else None
    names = [n for n in args.extra_clips.split(",") if n] if extra is not None else []
    only = set(n for n in args.clips.split(",") if n) or None
    plist = playlist(graph, extra, names, not args.no_scan, only)
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
    for cam_name in args.cameras.split(","):
        eye, look, focal = cams[cam_name]
        vp = use_camera(stage, "/World/Cam_" + cam_name, eye, look, focal)
        frames = os.path.join(args.out, "_frames_%s_%s" % (name, cam_name))
        shutil.rmtree(frames, ignore_errors=True)
        os.makedirs(frames)
        shot, corner, cards, sims = 0, [], [], []
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
            sims.append({"track_deg": round(worst, 3), "contacts": len(contacts),
                         "first_contacts": [c[1:] for c in contacts[:3]]})
            card = overlay.clip_card(seg.name, seg.kind, seg.start, seg.duration, seg.labels, k + 1, total, sims[-1])
            cards.append(card)
            corner.append((x["first"] / float(FPS_VIDEO), overlay.block(card)))
            print("[library] %s %d/%d %s: tracking %.2f deg, contacts %d"
                  % (cam_name, k + 1, total, seg.name, worst, len(contacts)), flush=True)
        current[0] = None
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
                              os.path.join(args.out, "%s_library_%s_sheet.jpg" % (name, cam_name)))
        if sheet:
            shutil.rmtree(posters_dir, ignore_errors=True)
        mp4 = os.path.join(args.out, "%s_library_%s.mp4" % (name, cam_name))
        video = encode_video(app, frames, corner, shot, mp4, FPS_VIDEO, (1280, 720), "library")
        for c, sim in zip(summary["clips"], sims):
            c["sim"][cam_name] = sim
        summary["passes"][cam_name] = {"video": video, "sheet": sheet, "frames": shot,
                                       "tracking_max_deg": max(s["track_deg"] for s in sims),
                                       "clips_with_contacts": [c["name"] for c, s in zip(summary["clips"], sims)
                                                               if s["contacts"]]}
        print("[library] %s: %s, sheet %s" % (cam_name, video, sheet), flush=True)
    out_json = os.path.join(args.out, "%s_library.json" % name)
    json.dump(summary, open(out_json, "w"), indent=1)
    print("[library] %s" % os.path.relpath(out_json, ROOT), flush=True)


main()
app.close()
