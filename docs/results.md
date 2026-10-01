# Results log

Measured results, kept for good (IMPLEMENTATION_PLAN.md is deleted when its
stages are done -- its Status numbers are copied here first).

Every entry: date, the number, before -> after, **where** it was measured, and
the commit. "Where" is one of:

- **unit** -- a test script, no robot
- **Houdini** -- the scene / PDG, no robot
- **SimMachine** -- Fairino's controller simulator (FR20-V1-001 V6.0 image), no arm moving
- **real FR20** -- the physical arm in the lab

Only **real FR20** rows describe something the arm has actually done.

## Real FR20

| Date | Result | Before -> after | Commit |
|---|---|---|---|
| 2026-10-01 | The new wall measured from the arm at the scan's start (bar off, a tape from the flange): 17.2 cm to the paper where the build put 10.74 (the arm exactly at the build's joints, its tool square to the canvas): the wall 6.46 cm further than the placeholder canvas. Canvas and scan area moved back 6.46 cm; the bar's collision boxes split to its section (its capsules 4.3 mm past the shade's rim, was 7.8; all 17,459 CAD points inside); both shows rebuilt clear; scans at 3.5 / 4.5 cm built to try (3.1 / 4.1 cm clear). The first scans had been 12.9 cm from the LED chips to the paper: blurred (a bare LED's spot ~1.3 x the distance) | -- |
| 2026-09-25 | First clip streamed on the real arm: `fr20_test_new.csv` (323 frames, 13.4 s), ServoJ at 125 Hz, 3 runs, 0 controller errors | -- | bdfa47c (retime), 6c4b335 (play_ui) |
| 2026-09-25 | At speed 1.0 the clip played at its designed length, 13.4 s, not slowed (peak J5 acceleration 148.8 of 150 deg/s^2) | the same curve's earlier export needed x8.54 | bdfa47c |
| 2026-09-25 | Tracking after removing the controller's ~114 ms lag: max 0.80 / 0.76 deg, RMS 0.29 / 0.26 deg (speed 1.0); two runs agree within 0.23 deg, end pose 0.035 deg | raw, un-aligned error ~7 deg (all lag) | -- |
| 2026-09-25 | Speed 0.3 run: 26.5 s (x1.97, since speed scales the acceleration limit, not the time), max 0.43 deg, RMS 0.11 deg | -- | -- |
| 2026-09-25 16:22 | `fr20_test_inside_wall_fast.csv` (test curve inside the left wall, retimed at the measured 300/600 limits; 336 frames, 14.0 s) on the real arm: no controller error. Recorded vs commanded, lag-aligned (102 ms of clip time): max 0.85 deg (J4), RMS 0.23 deg; end pose within 0.11 deg. Video: IMG_0346.mov | -- | c7cd6ff (clip) |
| 2026-09-25 | Room probed with the arm (`probe_ui.py`): our URDF TCP vs the controller's TCP ~1.1 mm on every point | SimMachine: 0.004 mm | c92509d, 9d73d98 |
| 2026-09-25 | Measured room: floor, two walls (arm mounted turned 11.28 deg to the room), ceiling 2.16 m by tape; the rest from a RoomPlan scan aligned to the probed walls (agree to 0.47 deg) | an estimated room | 9d73d98, 1f9c43c |
| 2026-09-25 | Acceleration probe (`accel_ui.py`), every joint to 900 deg/s^2 with no controller error; tracking 150 -> 900: J1 0.20 -> 0.38, J2 0.24 -> 0.49, J3 0.26 -> 0.63, J4 0.30 -> 0.69, J5 0.29 -> 0.67, J6 0.38 -> 1.34 deg. J1-J3 visibly shake (arm + plywood base) | planned at 150 on every joint | 074bdd0, 0087660 |
| 2026-09-25 | Planning limits set from it: J1-J3 300, J4-J6 600 deg/s^2 (J1-J3 kept low until the base is bolted to spec) | 150 all joints | 7435161 |
| 2026-09-25 | Controller interference zone (cube, "work area") stops the arm when the TCP leaves it -- tested with PTP in auto mode (jog ignores it) | -- | (WebApp setting, no code) |
| 2026-09-28 | **The show (v5 library, 4 hubs) streamed on the real arm** from `show_ui` (Real FR20): 10 min at speed 0.6 and 10 min at speed 1.0, both ended at a hub, no controller error; largest step 78 / 84 % of the velocity limit | SimMachine only before | 4125c7d, 8b1f974 |
| 2026-09-28 | Skipped ticks: 42 (0.6) and 78 (1.0) single ticks in 10 min, each after one slow ServoJ call (16-31 ms; median send 2.9 ms) -- the controller's XML-RPC, not this PC waking late | SimMachine: 0 | 701d19a (the reasons logged) |
| 2026-09-28 | Latency grows: the arm's lag behind the commands rises steadily, 104 -> 352 ms over 10 min at 0.6 (~+0.4 ms per s: the controller consumes ServoJ ~400 ppm slower than the PC sends); at 1.0 it rises the same way and drops back where skips cluster (each skipped tick shortens the controller's queue) | SimMachine: ~15 ppm | -- |
| 2026-09-28 | Stream paced by the motion queue (GetMotionQueueLength): 10 min at 1.0, 0 skips, queue steady at ~7, no error -- but the lag still grew 104 -> 656 ms (~0.95 ms per s): a buffer behind the queue fills. The controller's clock (GetSystemClock, `controller_clock.py`) is only -126 ppm against the PC; it plays ServoJ ~950 ppm slower than cmdT in all (the PC-paced runs agree once their skipped ticks are counted). Now: points paced at a calibrated playback rate per controller | 6ac1c88 -> see git log |
| 2026-09-28 | Path shape, each 20 s window aligned by its own lag: max error 0.43-0.94 deg per joint, RMS <= 0.10 deg (0.6); max 1.3-3.7 deg, RMS <= 0.28 deg (1.0). One constant lag for the whole run showed up to 8 deg -- the drifting lag, not the arm | -- | -- |
| 2026-09-28 | **Paced at the calibrated playback rate (-950 ppm), the lag holds**: the show (strip library) 10 min at 0.5 and 6 min at 1.0 on the real arm, 0 skips, no error, queue 0-3; per 30 s window the arm is 104-128 ms behind all run, path RMS 0.03-0.22 deg. The reports said the lag grew to 500 ms and suggested -1461 / -1628 ppm: their analysis put point k at k x dt, but a paced point leaves the PC at k x dt / rate -- the calibration read back as drift. Fixed (the log's times and the analysis on the PC time each point was sent at); -950 stays | the lag grew ~1 ms per s | see git log |
| 2026-09-30 | **The interactive mode (track_mode) on the real arm**, guests tracked live by the Femto Mega through TD (logs `stream_20260930-*.json`, target hardware): 4 runs -- 121 s at speed 0.3 (interactive share 0.2; 2 engagements; 15,028 sends, 50 skipped), 194 s at 0.3 (0.2; no guest engaged; 24,201 sends, 63 skipped), 430 s at 0.5 (0.35; 4 engagements; 53,528 sends, 222 skipped), 290 s at 0.8 (0.35; 3 engagements; 35,455 sends, 801 skipped, worst step 77 % of the velocity limit). Every run: 0 unsafe ticks, 0 refused, no controller error, ended at a hub. The skips are the robot laptop's (it misses ticks, most at 0.8); the engage code then was before the wider follow, crouch, lead and the goodbye fix (all since in sim only) | first real run of the mode | logs from the robot laptop |

Not yet run on the real arm: any dance clip, `--goto-home`, a clip refused by
the cell check. Dance clips so far: dry-run and SimMachine only. The UV strip
lit on the arm, and a scan on real paper, are each to get their own row when
they first happen (a real scan attempt on 2026-09-30 stopped at MoveJ error 14,
a controller EtherCAT fault, before it began).

## SimMachine

| Date | Result | Commit |
|---|---|---|
| 2026-09-24 | FR20 kinematics: our FK vs `GetForwardKin` 0.016 mm / 0.0006 deg over 23 poses; the controller's IK is among our branches 18/18 | 4c64d36 |
| 2026-09-24 | ServoJ player: 240-frame curve, 2666 sends at 125.04 Hz, 0 skips, max lateness 0.55 ms, send p50/p95/max 1.2/3.4/8.3 ms; aligned tracking max 0.072 deg (RMS 0.018), lag ~40 ms | 4c64d36 |
| 2026-09-24 | Probe tip vs controller TCP 0.004 mm | -- |
| 2026-09-25 | d01 dance clip played at speed 1.0, no controller error | -- |
| 2026-09-27 | Show streamed from the state machine (`show_stream.py`, party show): 3 min, 22,634 ServoJ sends at 125 Hz, 0 skips, max lateness 3.6 ms, no controller error, ended at a hub at rest; largest step 67 % of the velocity limit; tracking after a 40 ms lag max 0.54 deg (RMS 0.06). OSC: `greet` and `scan` triggers played at the end of the running clip; `/robot/stop` stopped at once, no controller error | af7697f |
| 2026-09-27 | Show v5 library (52 idle clips at 4 hubs on three levels; dances with levels / sizes / tempo curves, 15 gesture and 6 path families; energy arc), streamed 3 min at speed 1.0: 22,754 sends, 0 skips, max lateness 5.4 ms, no controller error, all four hubs visited, ended at a hub; largest step 73 % of the velocity limit; tracking after a 40 ms lag max 0.57 deg (RMS 0.10) | see git log |
| 2026-09-27 | Show v4 library (34 idle clips: 12 dances with animation principles, 22 gestures and spatial paths over 17 families; Ruckig moves), streamed 3 min at speed 1.0: 22,764 sends, 0 skips, max lateness 1.1 ms, no controller error, ended at a hub; largest step 80 % of the velocity limit; tracking after a 40 ms lag max 0.28 deg (RMS 0.027) | see git log |
| 2026-09-27 | Show stream, 30 min with TouchDesigner-style OSC (39 triggers greet / scan / calm, a pause and resume, energy changes), after the GC fix: 225,501 sends, 0 skips, max lateness 5.4 ms, no controller error, ended at a hub; 350 segments. The arm's lag grew 40 -> 68 ms over the run (clock drift, see Failures); tracking after the best single lag 2.2 deg max, RMS 0.29 | 32ef7a8 |
| 2026-10-01 | **5 h soak, the unattended evening's length** (party_bigwipe, speed 1.0, TouchDesigner live: Auto Scan every 3 min, the scan images in turn): 2,249,850 sends in 18,001 s, ended at a hub, no fault, no controller error; 2,865 segments (all 87 of the show), 68 scans triggered by TD on its own; 288 ticks skipped (0.013 %; the worst a 227 ms ServoJ call); the lag stayed 40-64 ms over the 5 h (no drift); tracking after the best lag 2.2 deg max, RMS 0.27. The stream's memory grew evenly, 96 -> 477 MB (its logs), and its report's analysis peaked at 1.8 GB at the end | 94aeb53 |

## Isaac Sim

| Date | Result | Commit |
|---|---|---|
| 2026-09-28 | Show stream paced by the controller's motion queue (GetMotionQueueLength, target 6 points): 5 min at 1.0, 37,832 points, 0 skips, queue 0-5 (median 4), never ran empty, lag 36 ms in both windows, tracking after lag max 0.35 deg | see git log |
| 2026-09-28 | Show v5 in Isaac Sim 6.0, headless 3 min on the USD room, a scan trigger every ~40 s: 35 clips at all four hubs, 4 scans (trigger to scan 10.5 s mean, 12.1 max -- longer than v4: the scan starts from rest, and the hubs are further apart), 0 arm-room contacts, tracking max 1.03 deg (J4), RMS <= 0.22 | see git log |
| 2026-09-27 | Show v4 in Isaac Sim 6.0, headless 2 min, the room read from OpenUSD (`shows/party.usda` over `envs/volvox_lab.usda`), a scan trigger every ~30 s: 31 clips over both hubs, 4 scans (trigger to scan 5.4 s mean, 6.7 max), 0 arm-room contacts (PhysX), tracking under gravity and the drives max 0.67 deg (J5), RMS <= 0.23 | see git log |

## Houdini / unit

| Date | Result | Before -> after | Commit |
|---|---|---|---|
| 2026-09-24 | Closed-form IK: 10,000 random in-limit poses, every branch reproduces its target (worst 4e-12 m, 8e-11 deg); drawn curve 240/240 frames <= 0.005 mm, 4 branch changes, all forced by a limit | FBIK (iterative) | 4c64d36 |
| 2026-09-24 | Retime with acceleration (TOPP): 433 frames, dry-runs at time scale 1.0, 18.0 s | x8.54 slower than designed | a6a5ae4 |
| 2026-09-25 | Retime fits the keyed, cooked frames: the user's curve passes Pre-Flight at 323 frames; 216 at the measured limits | 1.04 -> 1.18 with a uniform stretch (the failed first fix) | bdfa47c, 7435161 |
| 2026-09-24 | Capability atlas PDG bake (10 cm, with the room): 136 s of work in 38 s over 8 items; 45 % of the reachable space clear of the lab, tool down | -- | f921932, 0617eb2 |
| 2026-09-25 | Primitive factory (50 variants) against the measured room at the measured limits: 24/50 ok (14 unreachable, 12 cell) | 20/50 (14 unreachable, 16 cell) | 7435161 |
| 2026-09-25 | Dance factory: 48/48 ok; d32 37.6 -> 25.8 s at the new limits | -- | 7435161 |
| 2026-09-25 | Export round trip through the asset exact (0.00000 deg), with J4 shifted by a whole turn into its limits | J4 233 deg "over its limit" | 47e3286, b9b730b |
| 2026-09-25 | Clip review in PDG: 98 clips rendered, tiled, 266 s | -- | 9a73869, 54ef62d |

## Failures and what they taught

- **The real LED bar would have hit the arm in half the show** (2026-10-01,
  before it went on the arm). The show had been built for the tool as
  specified before it existed: a 1.0 m strip, 25 x 20 mm, 70 mm out on a
  40 mm bracket. The fab team's CAD (FULLUVBAR, STEP) is 1087.5 mm long with
  its end caps, 37 x 35 mm in section, on a 116 mm plate, 47 mm out.
  - *Found*: every motion of the compiled show checked again with the bar
    as built: 43 of 87 segments failed -- the forearm into the bar (capsules
    4.8 cm into each other, 12 clips) and into the plate (2.3 cm, 5), the
    bar's end 5 mm under the floor's margin plane in 10 low clips, and 20
    greet clips 26 cm from the ceiling (the sprinkler's 30 cm margin).
  - *Fix*: the bar's URDF from the CAD (assets/tools/uv_bar.urdf), the show
    rebuilt with it: 86 segments, every one clear (nearest: the plate 4.3 cm
    from the paper in the scan); one big wipe (greet_wipe_cols) refused. Isaac
    shows the bar as built (the CAD as one mesh, cad_tool_mesh.py).
  - *Lesson*: check the show again whenever the tool is measured, before the
    arm moves with it.

- **A 30 min stream stuttered, and the log hid a clock drift** (2026-09-27,
  SimMachine). The first 30 min run skipped 159 ticks in 31 bursts, and
  feedback stalled for up to 0.22 s at t = 72, 401, 810, 1032, 1310, 1662 s.
  - *Cause*: Python's full garbage collections. They walk every live
    object, and the run's own log (lists per tick) reached about 800k
    objects. A test showed a 45 ms pause at 450k objects.
  - *A measurement error*: joint reads were stamped before the call. A
    stalled read looked like a 3 deg tracking error (J5 at 16.8 deg/s x
    0.2 s), which the arm never had.
  - *Fix*: the logs are flat arrays, `gc.freeze()` runs after start-up,
    reads are stamped mid round trip, and slow reads are dropped. The rerun
    had 0 skips.
  - *What the rerun then showed*: the arm's lag grew from 40 to 68 ms,
    about 15 ppm. This PC sends ServoJ slightly faster than the
    controller's 8 ms cycle consumes it, so its queue fills. The skips in
    the first run had been draining it.
  - *Now*: the report measures the lag per 3 min window
    (`lag_ms_by_window`). The FR20's own drift is measured at the studio.
    For a multi-hour show the plan is to drop or pad ticks while the arm
    rests at a hub.

- **Labels drift at new limits** (2026-09-25, Houdini). The measured Laban
  labels were calibrated at 150 deg/s^2. At 300/600 the same phrases measure
  differently: d32 generated float-slash-press-float, measured
  flick-wring-punch-slash; in the review 76/127 bars measure as intended. Found by the
  factory's own label-vs-intent check and in the review videos. Open:
  recalibrate. Picture: `geo/review/page_dance_*.mp4`.
- **26 of 50 primitives refused** (2026-09-25, Houdini): 14 unreachable, 12
  hit the room (mostly the upper arm near the floor, one into the left
  partition). Shown framed red in the review, the unreachable part of the
  path drawn red. The system refusing clips is the point: the check runs
  before anything reaches the robot.
- **IK success criterion changed** (2026-09-24, unit). "No joint step above
  the velocity limit" failed on the test curve -- but the curve itself needs
  8-14 deg/frame on J4/J6 near the wrist singularity. Rewritten as "every
  branch change is forced by a joint limit": a property of the solver, not of
  the curve.
- **Retime still 1.04x slow** (2026-09-25, Houdini). The fit measured
  un-eased frames on a different grid than the player; a uniform stretch made
  it worse (1.18x). Fixed by fitting on the keyed, eased, cooked frames.
- **Floor fitted 6 deg tilted** (2026-09-25, real FR20 probe). Three nearly
  colinear points gave a plane +17 cm off. Fixed with a spread warning and
  "wall" / "level" fits that use what is known (walls vertical, floor level).
- **MoveJ error 154 + FK mismatch of 11.28 deg** (2026-09-25, real FR20). A
  workpiece frame was left active on the controller after setting the safe
  zone; `GetForwardKin` answers in that frame. The player now refuses to move
  and says which frame to reset. ccbf75f
- **Runs labelled "sim" were real** (2026-09-25). play_ui's Target was
  SimMachine while the IP pointed at the real arm, so no confirmation was
  asked and files were named `_sim_`. Noticed from the controller model in the
  report.

## Media index

Name real-robot video files
`YYYYMMDD_<clip>_speed<s>.mp4` (e.g. `20260925_d17_speed0.6.mp4`) with a
matching Houdini viewport capture from the same angle, and list them here.

**To film at the show recording** (asked by the portfolio session, 2026-09-30):

1. A fixed, wide tripod shot of one full cycle in the dark room -- idle, the scan, the image appearing, its fade:
   90 s or more, 4K, exposure locked.
2. A close-up of the strip passing over the paper, and a still of the finished violet image.
3. The interactive mode: someone steps onto the spot, the arm turns, follows, crouches, nods goodbye; two people if
   possible (attention choosing between them). **Record the Femto at the same time** (TD tracking_test > Record:
   `tracking/femto_rec_*.csv`) and note the take: the Isaac replay of that same take, from the same camera, is the
   real/sim pair (a replay from another recording does not pair).
4. Isaac replays from the same camera poses as 1 and 3 (run_tracking / replay_render `--camera 'ex ey ez tx ty tz
   focal'`; Isaac prints the viewport's as `[camera] --camera ...`), to cut real and sim side by side. Note the pose.
5. A screen recording of TD (pixel_scan, idle_leds, the monitor) during the take.

| File | Where | What |
|---|---|---|
| IMG_0349.MOV (Downloads; 1280x720, 16 s) -- **main**; stabilised on the website as fr20-real.mp4 | real FR20, 2026-09-25 16:24 | `fr20_test_inside_wall_fast.csv` playing. A later run than the recorded one (16:22): the numbers above are from the recorded run |
| IMG_0346.mov (Downloads; 1280x720, 23 s, handheld) -- spare | real FR20, 2026-09-25 16:14 | the same clip, an earlier run |
| geo/review/side_by_side/inside_wall_fast_houdini.mp4 (local, geo/ is gitignored; `scripts/render_real_compare.py` remakes it) | Houdini | the same clip from the CSV, camera lined up by eye with IMG_0349; 1280x720, 30 fps, 16.0 s: CSV frame 1 held 1 s, then played by time_s (video t = 1 s + time_s), last frame held 1 s. Room outlines and zones as the review draws them; the goal curve (cyan) from scenes/FR20_rig.hiplc as saved 2026-09-26 12:10 -- the clip's TCP path stays within 3.2 mm of it |
| geo/review/side_by_side/inside_wall_fast_cat_still.png, inside_wall_fast_cat.mp4 (local; `render_real_compare.py --view curve`) | Houdini | the same clip square on to the goal curve's plane (the cat drawing, 0.88 x 0.52 m, flat to 4 mm), from the robot's side: still = CSV frame 336 (the TCP on the curve's end), 1600x900; clip = CSV frames 200-336, 5.7 s, 30 fps, H.264 High@4.0. Curve as a cyan tube; room as floor and solid furniture only |
| IMG_0345.jpg (Downloads) | real FR20 lab | the lab: arm, TV wall, control cart |
| tests/csv/fr20_test_inside_wall_fast_actual_hardware_20260925-162248.csv (local, gitignored) | real FR20 | the recorded joints of that run |
| docs/images/portfolio/review_contact_sheet_1600.jpg | Houdini (OpenGL, clean) | the review contact sheet, 1600 px: 98 clips, rejected framed red |
| geo/review/contact_sheet.png | Houdini | every clip, one frame, rejected framed red |
| geo/review/page_*.mp4, overview.mp4 | Houdini | clip review videos |
| geo/isaac/td_capture_20260930-191550/replay_front_1920x1080.mp4 (local) | Isaac | the 5 min show with TD live (strip, canvas, ceiling look with the accent flash), the front camera; `--camera front` |
| geo/isaac/td_capture_20260930-184751/replay_interact_1920x1080.mp4 (local) | Isaac | the same show, the interact camera, the ceiling look before the flash |
| geo/isaac/td_capture_20260930-175134/replay_interact_3840x2160.mp4 (local) | Isaac | 4K, the first ceiling look (superseded) |
| geo/isaac/portfolio/twin_room_like_IMG_0443_1600x1200.png (local) | Isaac | matches IMG_0443.jpg (Downloads; the room through the doorway, the arm at rest): `replay_render.py geo/isaac/td_capture_20260930-191550 --camera "-1.8 -1.6 1.55 -0.2 2.0 0.8 9" --size 1600x1200 --start 0 --ceiling 1.0 --no-label --still ...`; the pose matched by eye. The twin has the show's planned canvas wall, not built in the photo |
| geo/isaac/portfolio/twin_tracking_like_tracking_MOV_1280x720.png (local) | Isaac | matches tracking.MOV (Downloads; real FR20, 2026-09-30 ~20:07, the interactive mode: the guest behind the glass on the left, the arm following): `--camera "2.0 -1.1 1.6 -1.6 0.45 1.0 10" --size 1280x720 --start 175.6` (the arm at greet), matched by eye -- too far from the video's camera and pose to pair (not used on the site). No Femto recording of that session on this PC (the robot laptop may have one), so no replay clip yet |
| tracking.MOV (Downloads; 1280x720, 70 s) | **real FR20**, 2026-09-30 ~20:07 | the interactive mode with a guest: the guest raises a hand behind the glass, the arm turns and follows |

Still to make (1600 px, clean -- the Karma previews are noisy): atlas, cell,
dance sheet.

## Credits

- **Wenyi Zhang** -- built everything (Houdini assets, IK, player, cell,
  factories, review pipeline) and operated the robot.
- **Volvox Labs** -- owns the FR20 and the lab space.
