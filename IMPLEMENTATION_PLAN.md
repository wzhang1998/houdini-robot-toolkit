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
