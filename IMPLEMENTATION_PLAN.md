# Implementation plan — motion lab

Architecture and reasoning: `docs/motion-lab-architecture.md`.
Delete this file when every stage is Complete.

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
**Status**: In Progress — core, PDG bake (10 cm: 43x34x43, 89 s of work in
23 s over 8 work items), merge, viewers (half shell + headroom slice) and
both test scripts pass. Next: sample the atlas on robot_arm's goal curve
(colour the curve by headroom before solving); Capability mode bake timing;
finer voxels; tool length from the asset's tool.

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
(`envs/volvox_lab.json`) is still an ESTIMATE from one photo, and its
orientation (the arm's front towards the TV wall) is assumed: measure it
with `probe_env.py` on the real arm. OAK-D point clouds as an obstacle
source: not started.

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
factory clip dry-runs at time scale 1.0. Open: Ruckig (a new dependency,
ask first); FR20 jerk limit unknown; the real acceleration limit
(`accel_probe.py` on hardware) -- it decides how dynamic phrases can be;
per-bar label accuracy; an OAK-D capture script; the Houdini import
round-trip of a clip.

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
