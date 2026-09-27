# Implementation plan — motion lab

Architecture and reasoning: `docs/motion-lab-architecture.md`.
Delete this file when every stage is Complete -- first copy the measured
numbers from each Status into `docs/results.md` (kept for good).

## Stage 1: FR20 closed-form IK inside Houdini
**Goal**: A UR-type closed-form IK (`scripts/ur_ik.py`, no hou dependency)
returning all valid branches, and a solver backend in `robot_arm` selected by
the profile: FR20 uses it through a Python SOP in place of FBIK; UF850 keeps
FBIK.
**Success Criteria**:
- Every returned branch reproduces its target through URDF FK to < 1 µm and
  < 0.001° over 10,000 random in-limit poses.
- Solutions respect `limits_deg`; unreachable targets report "no solution"
  instead of a wrong pose.
- Along the drawn test curve, the solver adds no discontinuity of its own:
  every branch change is forced by a joint limit. (The original wording, "no
  joint step above the velocity limit", turned out to be a property of the
  curve, not the solver: near the wrist singularity the curve itself needs
  8–14 deg/frame on J4/J6.)
- UF850 regression unchanged (baseline harness).
- When SimMachine is available: 20 poses agree with `GetForwardKin` /
  `GetInverseKin`.
**Tests**: FK round-trip per branch; limit filtering; singular configurations
(wrist, shoulder, elbow) return finite results or a clear failure; continuity
on the test curve; UF850 baseline.
**Status**: Complete — `ur_ik.py` tests pass over 10,000 random in-limit
poses: every branch reproduces its target (worst 4e-12 m, 8e-11°); away from
singularities the pose's own q is always among the branches; the 190 poses in
a wrist / elbow / shoulder singular neighbourhood are checked for exactness
only, since there branches merge or a family of solutions reaches the same
pose. Also float32 goal noise, unreachable → `closest()`; asset: manual goals 0.0001–0.0008 mm, drawn curve 240/240
frames ≤ 0.005 mm, 4 branch changes all limit-forced; SimMachine FR20: FK
0.016 mm / 0.0006° over 23 poses, controller IK among our branches 18/18;
UF850 baseline unchanged.

## Stage 1b: Playback on FR20 — A: ServoJ stream
**Goal**: `scripts/fairino_player.py` plays a joint CSV exported from the
asset on a Fairino arm by ServoJ streaming, following the td-robot-twin
playback findings: controller-planned move to the first pose; the whole path
conditioned before motion (shape-preserving interpolation, rest at both ends,
equal-interval resampling at the control rate, uniform time scaling to the
velocity / acceleration envelope); absolute-deadline sends that coalesce stale
samples; feedback on a separate connection; a measured report. SimMachine
first. Later: segment-by-segment sending as in td-robot-twin.
**Success Criteria**: Conditioning self-tests pass (passes through samples, no
overshoot, zero end velocity, envelope honoured after scaling, bad CSVs
rejected). On SimMachine FR20: the exported clip plays start to end without a
controller error; the report gives effective send rate, skips, lateness,
duration scale, and tracking error of actual vs commanded joints after lag
alignment.
**Tests**: `python scripts/fairino_player.py --self-test`; a SimMachine run.
**Status**: Complete on SimMachine (FR20-V1-001 V6.0) — self-tests pass; the
scene's 240-frame curve clip (Fixed Direction) exported from the asset,
conditioned to 21.3 s (time scale 2.14, acceleration-bound at 300 deg/s^2),
streamed 2666 ServoJ at 125.04 Hz with 0 skips, max lateness 0.55 ms, send
p50/p95/max 1.2/3.4/8.3 ms; actual joints lag the command by ~40 ms and,
aligned, track within 0.072 deg (RMS 0.018 deg). Hardware not yet run: the
real controller's network latency and dynamics will differ.

## Stage 1c: Acceleration in Houdini -- what you see is what the robot plays
**Goal**: A clip that passes Pre-Flight plays on the robot at its designed
speed. One acceleration limit (profile `robot.max_acceleration_deg_s2`,
capped by the asset's Max Joint Acceleration) used by Houdini and the player;
Pre-Flight runs the player's own conditioning and fails when it would slow
the clip; Retime plans with velocity AND acceleration limits (forward /
backward integration along the path, at rest at both ends).
**Success Criteria**: `scripts/retime_topp.py` tests pass (single-joint move
gives the analytic trapezoid time; every sample within the velocity and
acceleration limits; rest at both ends). On the FR20 scene: after Retime,
Pre-Flight reports a playback time scale <= 1.0 and the player's dry run
agrees; the clip is shorter than the velocity-only retime + uniform
stretch it replaces.
**Tests**: `python scripts/retime_topp.py`; Houdini: Retime -> Pre-Flight ->
export -> `fairino_player.py --dry-run`.
**Status**: Complete on the FR20 scene — Resample Length 5 mm, Safety
0.97: Retime -> 433 frames, Pre-Flight "plays at its designed speed" (worst
J6 acceleration 97 %), exported `tests/csv/fr20_test.csv` dry-runs at time
scale 1.0, 18.0 s (the earlier export of the same curve needed x8.54).
Findings: at the old 5 cm Resample Length the target had a corner every
5 cm, amplified by the wrist near its singularity (x1.57 after TOPP alone);
Retime now fits the corners locally (estimated, then cooked frames) and
finishes with a small uniform stretch (x1.068 here) verified on cooked
frames. Retime's Max Velocity / Max Acceleration are read-only displays of
limit x Safety. Not yet run on hardware.

## Stage 2: Capability atlas
**Goal**: FR20 reachability, singularity distance, joint-limit margin and
speed headroom baked as volumes over the workspace, via PDG, readable by
other SOPs (path tools, simulations) as constraints.
**Success Criteria**: Field values at 200 random voxels match a direct
measurement at their centres; beyond the reach reads unreachable; a
singular region reads 0 headroom; documented bake time and resolution; a
way to look at it in Houdini.
**Tests**: `python scripts/capability.py` (FK-made targets reachable, headroom
= brute-force worst direction, ~0 at wrist / elbow singularities, limit
margin); `hython scripts/atlas_check.py` after `build_atlas_scene.py --bake`.
**Status**: Complete for the fixed-direction atlas -- core, PDG bake (10 cm,
with the room: 136 s of work in 38 s over 8 work items), merge, viewers,
`clear` / `clearance` against the cell (45 % of the reachable space is
clear of the lab, tool down), and robot_arm's Curve Check (the goal curve
measured through the real solve, coloured by risk; FR20 scene: the final
hook passes 3 deg from the wrist singularity). Both test scripts pass.
Not done: Capability-mode bake timing; finer voxels; the asset's tool
length in the bake (a parameter today).

## Stage 2b: The cell -- real2sim, collision, safety zones
**Goal**: The room the robot works in, in the robot base frame, checked on
every clip: obstacles with a margin, keep-out / slow / work zones; the arm
as capsules from its URDF meshes; measured with the arm itself.
**Success Criteria**: Every mesh vertex inside its capsule; capsule-box
clearance matches brute force; floor, self-collision, obstacle, slow and
work-zone cases caught; Pre-Flight fails a clip that hits the cell;
touch-off points agree with the controller's TCP; the cell in Houdini.
**Tests**: `python scripts/collision.py`, `python scripts/env_from_points.py
--self-test`; hython: Pre-Flight on FR20 (clear), with a crate on the path
(FAIL at frame 101), UF850 (not applicable).
**Status**: Complete in software -- all of the above pass; probe vs
controller TCP 0.004 mm on SimMachine; `scenes/FR20_cell.hiplc`. The room
is MEASURED (2026-09-25): floor and the two near walls probed on the real
FR20 (`probe_ui.py`; URDF vs controller TCP ~1.1 mm on every point), the
ceiling grid by tape (2.16 m -- the raised arm reaches it), the rest from a
RoomPlan scan aligned to the probed walls (`scan_to_env.py`, the walls
agree to 0.47 deg); the arm is mounted turned ~11 deg on a plate square to
the room. Still estimated: the plant's size, the base plate's size; the
operator's place as marked by the user (+-0.2 m). The factory's clips were
checked against the old estimate: re-cook against the measured room.

Later (noted 2026-09-25, not started): robot_arm's input 5 "Collision
(reserved, unused)" as the room's geometry, replacing Setup > Cell
Environment when connected. Pieces by prim `name`; `role` (obstacle /
keep_out / slow / work, default obstacle); optional `margin`.
1. Simple shapes -> oriented box / cylinder fits, so Pre-Flight and Curve
   Check use collision.py's exact distances (same results as the JSON).
2. Any mesh (OAK-D / phone scan, furniture) -> VDB SDF, sampled along the
   arm's capsule axes -- no hand simplification of scans.
3. A Write Env JSON button, so PDG factory / atlas / clip library keep one
   room file.
Start with 1 + 3; 2 when a scan exists. Before building: check the
standard tools first -- Houdini's VDB from Polygons / SDF sampling for 2,
FCL / MoveIt planning scene (Stage 4) for the robot side -- and reuse
them rather than growing collision.py.

## Stage 3: Motion clip contract + PDG factory
**Goal**: A clip format (JointTrajectory-shaped JSON: times, joint positions,
TCP path, style parameters, metadata -- spatial bounds, duration, tags,
safety), clips made in PDG from primitives and from Laban-driven dance
phrases, labelled by measurement, checked against the robot and the cell;
people's motion brought in.
**Success Criteria**: A 50-variant wedge produces valid clips; every clip
stays within the velocity / acceleration (/ jerk where known) limits and
clears the cell; the manifest lists rejected variants with the reason;
measured labels agree with the generator's intent on single-action
phrases; a keypoint take becomes a playable clip.
**Tests**: `python scripts/motion_clip.py`, `clip_factory.py`, `choreo.py`,
`retarget.py`; `hython scripts/build_factory_scene.py --cook`.
**Status**: In Progress -- clip contract, primitive factory (20/50 ok: 14
unreachable, 16 cell), dance factory (48/48 ok, 42 s in PDG), measured
labels (6/6 effort orderings, 7/8 actions per clip, 66/127 per bar), human
retargeting (direct and effort) all pass their tests; every dance and
factory clip dry-runs at time scale 1.0. Also done: the Dance Phrase HDA (make / preview / export / load phrases in
Houdini), the clip library (search by measured labels, chain into a show
with transitions, checked like any clip), the Houdini round trip (exact,
through FK). The real acceleration limits are measured (2026-09-25,
`accel_ui.py`: clean to 900 deg/s^2 on every joint; planned at J1-J3 300,
J4-J6 600) and the factories re-cooked with them against the measured
room: dance 48/48 ok, primitives 24/50 (was 20); d32 37.6 -> 25.8 s. Open:
the labels' calibration (motion_labels CAL) was made at 150 -- at the new
limits the measured sequences drift (d32 generated float-slash-press-float,
measured flick-wring-punch-slash): recalibrate; Ruckig (a new dependency,
ask first); FR20 jerk limit unknown; J1-J3 higher once the base is fixed
to spec; an OAK-D capture script. Fixed: the asset's Import CSV (reads the file live; works
on locked instances; exact round trip on FR20 and UF850).

Clip review (2026-09-25): scenes/FR20_review.hiplc (scripts/build_review_scene.py), the usual PDG way --
a work item per clip -> ROP OpenGL (the user's viewport camera, headlight and grey background; the arm
white; the TCP path coloured by intended action) -> ffmpeg burns in intent vs measured per bar, acc %,
room clearance, player scale (rejected: framed red; unreachable: a card) -> page videos per set, one
overview video, and an ImageMagick contact sheet. Checked on 4 clips; the full set on the user's OK.

## Stage 3b: a richer library (2026-09-25, from docs/motion_library_survey.md)
**Goal**: Clips that use the whole stage and a wider vocabulary than lines,
circles and HOME-centred phrases, placed by the atlas, not rejected by it.
**Order** (each its own commit; new dependencies -- Ruckig, pyribs -- need
the user's OK):
1. Atlas placement: one score field (capability x headroom x wrist distance
   x clearance, joint-margin threshold), eroded by a path's extent; path
   centres and tool directions sampled from it; hub stations at several
   levels. Success: primitive acceptance well above 24/50, coverage of the
   stage measured before / after.
2. Hub poses + Ruckig transitions; phrases no longer only HOME-to-HOME.
3. Path families (Lissajous, rose, spiral, helix, waypoint splines, field
   flow) with typed parameters (extent, pivot, entry/exit, plane,
   orientation fixed / tangent / look-at), Choreographer-style.
4. MAP-Elites (pyribs) over family / phrase parameters, descriptors = the
   measured labels.
5. Dances: motif / variation operators, Laban Shape, BPM grid; the label
   recalibration at 300/600.
6. Retargeting: AIST++ tests, a PCA mode; later OAK-D + motion matching.
**Status**: Not Started (survey done). Ruckig and pyribs approved by the
user (2026-09-27).

## Stage 6: Interactive show on the real FR20 (2026-09-27, PROPOSED -- to discuss)
**Goal**: A demo for the studio: on the real FR20, the arm plays through
the clip library by itself; TouchDesigner can trigger named sequences and
a timetable schedules them; every motion is checked in advance and joins
the next without a jump. LED strip and paper are decided later (Mon/Tue):
the scan stays a placeholder until then.
**Already there** (commit 804c2dc): `scripts/show.py` -- hub motion graph,
Runner state machine (IDLE / TO_SCAN / SCAN / FROM_SCAN / PAUSED / FAULT),
Selector (no repeat, mood, energy), OSC bridge, dry run (20 min: 178 clips,
no joint jump); `envs/volvox_lab.usda`; `scripts/isaac/run_show.py`
(written, not run: the installed Isaac 5.0 RC does not start on driver
610.47).

### 6.1 Library v2: hubs and short clips (~1 day)
- 2-3 hub poses inside the controller's work area, facing the audience. A
  proposal to discuss: `rest` (low, centred, calm), `show` (high, towards
  the audience), `scan_ready` (next to the paper, once it is placed).
- Short idle clips (6-10 s) generated for this stage, hub to hub (choreo.py
  phrases that start and end at a given hub instead of HOME). Enough for
  variety: ~15 per hub, a few moves between hubs.
- Label recalibration at the 300/600 limits (motion_labels CAL), so mood
  selection means what it says.
- **Success**: every clip passes the room + work-area + canvas checks.
  Longest idle clip <= 10 s, so a trigger waits <= 10 s. A 30 min dry run
  uses every hub, has no joint jump, and repeats no clip within 8.
- **Tests**: `choreo.py` (start/end at a hub, at rest), `show.py
  --self-test` (multi-hub graph, route to a sequence's first hub), dry run.

### 6.2 Show control: sequences, timetable, status (~0.5 day)
- **Named sequences** in the show config, e.g. `greet` (show hub, 2 lively
  clips), `showcase` (a fixed playlist), later `scan`. The Runner routes to
  the sequence's hub at the end of the running clip, plays it, and returns
  to idling.
- **Timetable** (local time): e.g. `showcase` every hour on the hour; mood /
  energy by time of night. A trigger queued during a sequence waits
  (queue length 1 per name).
- **Status out** (OSC): state, clip, next clip, sequence, seconds to the
  next scheduled item, scan progress, a heartbeat.
- **Success**: in a dry run with a scripted timetable and triggers, every
  sequence starts within one clip of its trigger / its time; the event log
  matches the script.
- **Tests**: self-tests for queueing, the timetable, routing between hubs.

### 6.3 Streaming backend: the Runner on SimMachine and the FR20 (~1 day)
- One continuous ServoJ stream at 125 Hz driven by `Runner.step()`, instead
  of one CSV per run. It reuses fairino_player: absolute deadlines, a
  separate feedback connection, and conditioning. Every segment is checked
  at build time and meets the next at rest.
- Start: MoveJ to the first hub through `safe_move`, confirmed on hardware.
- Guards:
  - poll controller errors, and on an error stop the stream and FAULT;
  - check the joint step before sending;
  - a software STOP (StopMotion + ServoMoveEnd);
  - a speed scale (0.3 first) applied to the whole show;
  - an operator key or OSC `/robot/stop`.
- Log: commanded vs actual per tick, events, into the results format.
- **Success**: 30 min on SimMachine with triggers from TD, 0 controller
  errors, 0 skips, tracking like the single-clip runs. Then on the FR20:
  10 min at 0.3, then 0.6, stop / resume / fault drill.
- **Tests**: player self-tests extended (segment joins in the stream, stop
  mid-clip, fault handling); SimMachine run.

### 6.4 TouchDesigner control surface (~0.5 day)
- The OSC spec (docs/show_pipeline.md) as the contract.
- A TD network built by a script run in TD's textport (TD files are binary):
  - OSC Out: trigger buttons per sequence, pause / resume / stop, mood menu,
    energy slider;
  - OSC In: state, clip, next, countdown, scan progress on a panel.
  - Step-by-step instructions as a fallback.
- **Success**: every button changes the Runner as expected in the dry run
  and on SimMachine; the TD panel shows state within 0.1 s.

### 6.5 Real-robot demo (studio day)
- Checklist (docs/hardware_test_plan.md, new section):
  - work area clear, E-stop in hand;
  - pull; wiggle; the first hub through a checked move;
  - show at 0.3, then 0.6; TD triggers; a timetable item;
  - stop / resume / fault drill.
- Film it: landscape, fixed camera, TD panel in shot if possible. Log it to
  docs/results.md.
- **Success**: 15+ min show on the real arm, triggered and scheduled from
  TD, no controller error.

### 6.6 In parallel: Isaac Sim 6.0 twin (~0.5 day after the user installs it)
- Run `scripts/isaac/run_show.py` (headless test, then the window). Add the
  multi-hub graph and the TD OSC link, so the same TD panel drives the twin.
- **Success**: a 10 min headless run with a report (tracking, contacts,
  waits), and a GUI session driven from TD.

### 6.7 After (as decided)
- LED strip as a tool (URDF link, controller tool load, collision
  capsule) and the real constant-speed scan with `/robot/scan`.
- OAK-D people presence in TD -> trigger `greet` / set energy / pause when
  someone is near (an experience feature; safety stays with the controller
  and the barrier).
- Houdini-native library browser; Lua / JointTrajectory / USD exports;
  Ruckig early exit from a clip.
- **Done 2026-09-27: hub and zone editor in the rig scene**
  (`scripts/show_rig.py`, installed live into FR20_rig.hiplc through the
  Houdini Agent bridge):
  - zones are draggable boxes; hubs are tool + look nulls, solved live and
    drawn as ghost arms (red when unusable);
  - operating range and library parameters on the panel; the config is
    written back with a check;
  - authored clips go from the robot_arm export to the library;
  - `python scripts/show_rig.py --self-test` passes (19 checks), and
    hython install, write and drag tests pass.

**Order**: 6.1 -> 6.2 -> 6.3 -> 6.4 -> 6.5; 6.6 whenever Isaac 6.0 is in.
**Open decisions**: hub places (6.1); which sequences the demo shows (6.2);
the demo's speed cap; the studio day.
**Status**: Proposed, waiting for the user's go.

## Follow-ups: standard tools evaluation (2026-09-25)
**Goal**: Replace or validate hand-rolled parts with industry-standard
tools, per `docs/standard_tools_eval.md` (read-only research; Houdini's
Python 3.13 has numpy but no scipy and no compiler, so only prebuilt
wheels count).
**Verdicts**: player path/home check -> REPLACE with the controller's
safety settings; collision.py -> keep, validate against python-fcl;
retime_topp -> keep, validate against toppra offline (no Windows wheels);
room probe, IK, Laban labels / retarget -> keep.
**Items, in priority order** (all Not Started; new dependencies need the
user's OK):
1. Controller safety as the runtime guard: soft limits, collision level /
   strategy, TCP speed cap, cuboid interference zones, set once in the
   WebApp. The player reads back `GetSafetyParamsCheckSum` and refuses to
   stream on a mismatch with the profile. Prove on SimMachine that they
   trip during ServoJ (the docs do not say). Retire `home_path_check`.
   Seen on the real FR20 (WebApp V3.9.3.1, FR20 V6.0), 2026-09-25: Safe
   Stop not enabled (policy: Default trigger, suspend); Safe Speed not
   enabled (manual 250 mm/s, stop alarm); Protective Stop category 2;
   Safety Plane 1 enabled (left wall, taught with the WebApp's 3 + 1
   reference points, safe distance 10 mm; TCP only); no interference
   zones. The SDK has no call that reads a safety plane back
   (GetSafetyParamsCheckSum only), so its points reach the env file by
   re-touching them with `probe_env.py`, or from a WebApp Data backup if
   that package turns out to carry them.
2. Collision check you can trust: sample between frames (at 24 fps an arm
   point moves ~0.13 m, more than the 0.05 m margin -- a thin obstacle can
   be jumped); a python-fcl mesh-vs-capsule oracle test (tests only); VDB
   SDF for scans (see Stage 2b's input 5 note).
3. Ruckig (MIT, cp313 wheel) for state-to-state moves: go to start / HOME,
   clip-to-clip transitions, a smooth abort ramp. Not a path parameteriser:
   retime_topp stays for following paths.
Also: cross-check URDF FK/IK against the controller's `GetForwardKin` /
`GetInverseKin` on the real FR20 (the URDF vs controller link-length
mismatch is the bigger risk); measure the probe tip with the controller's
tool calibration instead of `--tool-len`; OAK-D via depthai + OpenCV
ChArUco hand-eye (check `cv2.calibrateHandEye` exists in OpenCV 5, else pin
4.x); align Laban definitions with Larboulette & Gibet 2015 (our flow =
stillness fraction, theirs = jerk); AIST++ as the retarget test set.

## Stage 4: ROS 2 validation service
**Goal**: A container (ROS 2 Jazzy + MoveIt 2) with an FR20 MoveIt config
(URDF from `assets/fairino_description`, SRDF, collision scene) and an HTTP
service that checks a clip — collision, Pilz LIN/PTP/CIRC re-timing — and
plans transitions between clips. PDG calls it; results come back into
Houdini.
**Success Criteria**: The Stage 3 library validates end to end from PDG; a
deliberately colliding clip is rejected; a planned transition joins two
clips without collision.
**Tests**: Service contract tests; the colliding-clip case; transition
continuity at both ends.
**Status**: Not Started

## Stage 5: AI critic
**Goal**: Render a preview per clip and have a vision-language model label
it (legibility, perceived intent), written into the manifest.
**Success Criteria**: Labels on the full library; agreement with a small set
of hand labels reported, not assumed.
**Tests**: Deterministic manifest update; a hand-labelled check set.
**Status**: Not Started

Runtime (audience-driven clip selection with OAK-D) follows once the library
exists and is planned separately.
