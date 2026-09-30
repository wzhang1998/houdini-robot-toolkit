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
**Status (2026-09-27)**: 2 and 3 done, in the show; 1, 4-6 not started.
Ruckig and pyribs approved by the user.
- 2: `transitions.py` -- hub moves jerk-limited (Ruckig), corners blended
  within 2 deg of the planned legs; `show.timed_move` uses it. 125 Hz jerk
  on the hub moves 10364 -> 767 deg/s^3, ~8% longer. `exit_from` /
  `ramp_stop` written for an early exit, not wired yet.
- 3: `paths.py` -- lissajous, figure8, spiral, helix, rose, spline; tool
  look / tangent / fixed; two-thirds power law timing; 36/36 family x mode
  x hub combinations made, ~86% of random draws kept.
- Also: gestures fixed (220/220 draws, were dropping wave / nod / trace /
  short reach) with five new families (peek, shy, stretch, bounce, search);
  animation principles in gestures and dances; labels refitted at 300/600
  (single-action dances 91/96 held out, was 65/96).

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
- **Status (2026-09-27)**: Mostly done. Hubs `rest` (joints, dance
  phrases) and `greet` (tool + look, gestures that look at the audience).
  Zones, the operating range and authored clips are in the config. A
  10 min dry run has no jump and uses both hubs.
  - v4 (2026-09-27 night): rest 12 dances, greet 22 clips over 17
    families; 30 min dry run uses all 34, worst step 1.16 of 1.44 deg;
    3 min SimMachine stream 0 skips, tracking 0.28 deg.
  - Open: a `scan_ready` hub (waits for the paper);
    the cat clip needs a hub in its elbow configuration; the motion
    dynamics refinement (to discuss).

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
- **Status (2026-09-27)**: Partly done.
  - Done: named sequences (`greet`, `calm`), the trigger queue, routing
    between hubs, OSC status (state, clip, hub, progress, scan, joints).
  - Not started: the timetable, "next clip" / countdown in the status,
    the heartbeat.

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
- **Status (2026-09-27)**: In Progress.
  - Built as `scripts/show_stream.py`. `Segment.at` now samples with
    PCHIP (no velocity step at the 24 fps samples).
  - Self-test (fake controller): 11 checks pass.
  - SimMachine:
    - 3 min: 22,634 sends, 0 skips, no controller error, ended at a hub;
      tracking 0.54 deg max after a 40 ms lag;
    - OSC `greet` and `scan` triggers and `/robot/stop` checked.
  - 30 min SimMachine with OSC triggers, after fixing GC stalls: 225,501
    sends, 0 skips, no controller error.
  - It found a clock drift: the lag grew 40 -> 68 ms. The report now
    measures it per window.
  - Still to do: FR20 runs (`docs/hardware_test_plan.md` section 6c),
    which also measure the FR20's drift. Then drift compensation for
    multi-hour shows: drop or pad ticks while the arm rests at a hub,
    where it cannot be seen.

### 6.4 TouchDesigner control surface (~0.5 day)
- The OSC spec (docs/show_pipeline.md) as the contract.
- A TD network built by a script run in TD's textport (TD files are binary):
  - OSC Out: trigger buttons per sequence, pause / resume / stop, mood menu,
    energy slider;
  - OSC In: state, clip, next, countdown, scan progress on a panel.
  - Step-by-step instructions as a fallback.
- **Success**: every button changes the Runner as expected in the dry run
  and on SimMachine; the TD panel shows state within 0.1 s.
- **Status**: Not started in TD.
  - `scripts/show_ui.py` is a stand-in panel over the same OSC. A 1 min
    SimMachine run: greet, pause and resume all taken; ended at a hub,
    0 skips.
  - The architecture for TD is proposed in td-robot-twin
    `docs/superpowers/specs/2026-09-27-robot-agnostic-core-design.md`:
    show_stream as a worker speaking the Go worker's protocol, with a
    contact lease.

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
- **Status**: Headless 2 min run clean (tracking <= 0.96 deg, 0 contacts). The
  room is a cutaway. The panel and the OSC link are written, not yet driven
  from TD.

### 6.8 The show as one Houdini tool (2026-09-27, user's direction)
**Goal**: `/obj/robot_show` is one Geometry. Everything is on its parameter
page: robot profile, environment JSON, show config, zones and hubs
(multiparms), operating range, library, authored clip, build / dry run /
preview, and display toggles. Inside it:
- the robot_arm asset, which draws the room from the environment;
- the zones, as Box SOPs copied to data points;
- the hub ghosts, the robot's own meshes posed by Transform Pieces;
- the range, as Circle SOPs;
- the rays and paths, through PolyWire.

It replaces the scattered SHOW / zone / hub / look / CELL_CTRL /
cell_env / show_viz / hub_ghosts objects. Not an HDA (the user's choice):
a scripted Geometry node with spare parameters.
- **Stage A**: the node, its parameters, Load / Write / Check / Build on
  the multiparms, the arm previewing clips and hubs. Test: hython
  round-trip, config written unchanged.
- **Stage B**: the native display chain inside it, each branch switchable.
  Test: no errors or warnings in the network.
- **Stage C**: viewport handles in a Python viewer state ("Edit in
  Viewport"). Pick a zone or hub; an xform handle moves, turns and
  scales a zone; translate handles move a hub's tool tip and look target.
  Test: dragging changes the parameters, and Write gives the moved values.
**Status**: A and B done (2026-09-27).
- A fresh hython build has one node, and no node inside it has an error
  or a warning.
- The config round-trips unchanged.
- Both hubs check clear.
- Hold at Hub poses the arm.

C: handles need a node state (SideFX: "Handles cannot be bound to
nodeless states"), so the tool became the asset `wenyi::robot_show` with
the state as its default state. In a fresh scene it has no node errors
and the config round-trips unchanged. The handle <-> parameter mapping is tested live. A zone moves 0.1 m,
turns 10 deg and scales 1.5x; a look target moves 0.2 m.
Seeing and dragging the handles needs a person at the viewport: the
state's onEnter runs only once the viewer is used.

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

## Stage 7: The environment in OpenUSD (2026-09-27, user's go the same night)
**Goal**: The room is one OpenUSD file, envs/<room>.usda, with UsdPhysics
collision. It is edited in Houdini (Solaris), by hand in text, or by an
AI. Isaac Sim uses it as is; MuJoCo and Newton read USD. The toolkit's
own JSON room format (motionlab.env) is retired. No new format: the
only project-specific part is the safety meaning (work / keep-out / slow,
margins), stored as namespaced attributes on the prims (`motionlab:role`,
`motionlab:margin_m`), the extension mechanism USD provides.

**Why not the others**: URDF describes one robot, not a room. SDF
(Gazebo) is not edited natively in Houdini or Isaac. PyBullet does not
read USD (URDF / SDF / MJCF only) and is not a target.

**The file** (what each object becomes):
- a solid obstacle (cart, shelves, TV, base plate, walls, floor, ceiling):
  a UsdGeom Cube / Cylinder / Sphere, scaled and placed, with
  UsdPhysics CollisionAPI and a UsdPreviewSurface look;
- walls, floor and ceiling: finite slabs (Cubes) instead of infinite
  planes, so every tool can read them, each with the inward-facing
  single-sided face that makes the cutaway;
- a zone (work / keep-out / slow): a Cube or Cylinder with purpose
  "guide" (drawn as a helper, never rendered, no collision) and the
  `motionlab:*` attributes;
- metres, Z up, the robot base at the origin (as now).

**Stages**
- **7.1 Read and write USD** (~0.5 day): `room_usd.py` with `read(path)`,
  which returns the same in-memory room collision.py uses today, and
  `write(room, path)` (from env_to_usd.py). `collision.load_env` reads
  .usda. None of the ~30 scripts that take a room change. Test:
  JSON -> USD -> read gives the same objects (1e-6 m) and the same
  clearances on the stage clips and the show.
- **7.2 Switch over** (~0.5 day): envs/volvox_lab.usda becomes the source
  and the JSON is dropped. The defaults are changed in play.py,
  playback.toml, the show config and the robot_arm Environment. The
  measuring tools (probe, scan_to_env, env_from_points) write USD. Test:
  every self-test; the show build gives the same graph; the player's
  checked moves give the same routes.
- **7.3 Edit in Houdini** (~0.5 day): a documented Solaris workflow.
  Sublayer the room, then edit it (Transform / Edit / Cube, and
  Configure Primitive for the collision and the role) and save the
  layer. The robot_show tool and the robot_arm Environment take the
  .usda. Test: a box added in Solaris is drawn in Houdini, refused by
  safe_move and shown in Isaac.
- **7.4 Versions**: git on the text .usda. Per-show changes (the paper, a
  stage override) are a layer over the measured room, not a copy.

**Status (2026-09-27)**: 7.1-7.3 done, 7.4 started.
- `scripts/room_usd.py` reads and writes the room; `collision.load_env`
  reads `.usda` (usd-core from PyPI in the system Python, installed with
  the user's OK; hython has its own pxr). Walls, floor and ceiling are
  finite Cube slabs (the user's pick); the checks treat each as the
  halfspace behind its face towards the base, so they are unchanged:
  JSON -> USD -> read gives the same objects, and the 23 segments of the
  compiled show have the same clearances (difference 0).
- `envs/volvox_lab.json` and `env_to_usd.py` are gone. Every default, the
  show config, probe_ui / env_from_points / scan_to_env (write USD through
  `collision.save_env`), the robot_arm / dance_phrase / robot_show assets
  (Environment default, file filter, PythonModule) take the `.usda`. An
  old `.json` path in a saved scene opens the `.usda` beside it, with a
  note.
- Solaris edit tried in hython: Sublayer -> Cube -> Configure Primitive
  (PhysicsCollisionAPI) -> Transform on the ceiling -> USD ROP. The new
  box reads as an obstacle, the ceiling moves, nothing else changes.
- 7.4: the show's paper is a layer over the room, `shows/party.usda`
  (`room_usd.py --show`), which Isaac opens.
- Not yet: seeing the edit in Isaac (needs Isaac running); a
  collision-checked edit loop inside the robot_show tool.

## Stage 8: The LED strip on the real arm -- scan, cable, headroom, frame (2026-09-28)
**Goal**: the user's notes after the first strip runs: one scan pass, a cable
that never winds, room under the sprinkler, a scan speed to tune, a graceful
way out of the scan, bigger strip gestures, and the real frame in every view.
**Decisions (the user's)**: scan left to right as seen from the robot's side
facing the canvas, out from the right end; canvas centre 1.30 m; J6 inside
+-150 deg of the mounting pose; the frame's rails 5.25 in wide on the face,
overlapping the canvas, which sits behind them.
**Parts**
1. Cable: `tool.cable_j6_deg` in the profile; `robot_profile.motion_limits`
   read by IK (capability.load_fr20), gestures, choreo, safe_move, the player
   and the stream guard; the build refuses a segment outside them. -- Complete
2. Frame: `canvas.frame` {outer, face_width, depth}; `show.canvas_parts` (four
   wooden rails in front, the canvas behind) is the collision obstacle, the
   show's USD layer (wood and canvas materials: Isaac), the Houdini show asset
   and the review render. Canvas centre 1.30 m.
3. Headroom: a ceiling margin for every motion (the sprinkler at the ceiling's
   centre): nothing within `margins.ceiling_m` of it.
4. Scan: one pass (`scan.direction` left_to_right), in at the start's approach,
   out along the paper's normal at the end, then a checked route home; the
   scan's speed set in the config (m/s) and a run-time scan speed (<= 1: only
   slower) in show_ui / show_stream, for tuning the exposure.
5. The way out: a route that stays upright (no lying on the floor): an authored
   via pose if the planner's detour is still low.
6. Strip gestures (an agent): window-wipe arcs, a floor mop, a spinning sweep --
   all inside the cable's range; twirl as a back-and-forth.
7. Rebuild; report, dry run, SimMachine, Isaac, the review reel.
**Tests**: motion_limits narrows J6 only; limit_breaches names a J6 past the
cable; canvas_parts' rails overlap the canvas edges and stand proud of it;
the scan's first point is at the left end; no segment above the ceiling
margin; the run-time scan speed slows only the scan.
**Status**: Complete (2026-09-28: library v7, 70 idle clips; mop waits for a
gesture hub over the floor)

## Stage 9: Cleanup after the strip day (2026-09-28, the user's go: cleanup first)
**Goal**: remove what the day's patches left behind and make the build's
inputs single-sourced, before new features (tracking, LED sync). From the
review of 4125c7d..27b1ca0. Each step small, its self-tests green, committed
alone; steps marked (verify) rebuild + dry run + SimMachine after.
1. Dead code and stale text (mechanical): `show.find_approach` (no caller
   since scan_way), `fairino_player.Controller.system_clock` (no caller),
   `safe_move.ROOT_LINK` (unused); the stream's motion-queue poll slowed from
   every 20 ms to a 1 s diagnostic (its report stat stays; one RPC less on
   the feedback thread 50 times a second); gestures' volvox_lab.json fallback;
   docs describing queue pacing / the placeholder scan; `canvas.placeholder`.
2. Pacing names (verify): `clock_ppm` is a playback rate -- `playback_ppm`
   in the CLI, report and playback.toml (the old key still read);
   controller_clock.py reports the clock only (its --write put the clock's
   -126 where the playback's -950 belongs).
3. A compiled show knows its inputs (verify): `info.inputs` = hashes of the
   config, profile, tool URDF and room; show_stream, the dry run and Isaac
   refuse a compiled file whose inputs changed ("rebuild first").
4. One canvas module, one TCP helper (mechanical, compare outputs): the
   canvas parts / boxes / fixture points / USD extras from one place (kind
   carried through, colours shared); one `tool_point(q, local)`; the strip
   picked one way.
5. One source for margins and bands (verify): the ceiling margin and the
   strip's top from the show config (safe_move's constants the fallback);
   gestures' TCP_Z / STRIP_TOP_M from the config's range.
6. Failing self-tests fixed or re-baselined (choreo wring vs the TV wall with
   the strip, gestures' synthetic high hub, clip_library's fixtures) --
   before any module split.
7. Module splits (import-only, later): show.py -> graph/runner, build, scan,
   stats; show_rig.py; gestures.py.
**Tests**: each module's --self-test; after 2, 3, 5: rebuild, report, dry run
30 min, SimMachine 3 min; after 4: the review render and shows/party.usda
unchanged but for the order of prims.
**Status**: 1-4 done (2026-09-28, in v8). 5-6 open: gestures' self-test fails 7
checks, all at its synthetic high hub (1.5 m, where the upright strip meets
the 0.3 m ceiling margin) or on variety targets set before the strip and
margin (median extent 0.269 < 0.28 m, ...); bounce at synth_high starts
0.05-0.09 deg off rest -- the built library (v8) has every clip within
0.0005 deg. To decide: move synth_high under the margin and re-baseline the
targets, or keep them and widen the room. 7 after Wednesday.

## Stage 10: cuRobo in the pipeline, showpieces (2026-09-28, the user's go)
**Goal**: cuRobo plans the moves when its service runs, checked by ours;
big wipes along the front wall as clips (a second show version to compare).
**Success Criteria**: a Build with and without Docker; every cuRobo path
passes safe_move; the wipes built, clear, under the slow zones' limit.
**Tests**: curobo_bridge / safe_move / showpiece / show self-tests; the hub
moves with and without cuRobo (4 detours 114-116 vs 160-193 deg); the
rows wipe from low (1.45 x 0.66 m, 0.24 m/s) and columns from greet
(1.65 x 0.60 m); build shows/party_bigwipe.json, dry run, SimMachine.
**Status**: Complete (2026-09-29): party_bigwipe built (87 segments), dry run
30 min, each wipe on SimMachine (0 skipped, 0.12 deg), review reel. Build
speed (profiled): 33 min -> 141 s, party.json rebuilt identical to v8.
What is left of the build's time is the gestures' IK and FK in pure
Python (87 %): numpy for them, or the hubs in parallel (each its own
seed, so a new library), when it matters.

## Stage 11: Tracking, gaze first -- tested on simulated data, then Isaac (2026-09-29, the user's go)
**Goal**: the arm follows a person safely on top of the greet clips: a
tracking layer that never makes a sudden move, whatever the data does.
Defaults agreed: the head first; the track box ~40 x 30 x 15 cm on the
wall's side, never towards the audience; Ruckig at 30 % of the joints'
limits; lost: hold 0.7 s, then back to the clip in ~1.5 s.
1. Simulated tracking data (track_sim.py): `/track/target x y z conf t id`
   at ~30 Hz in robot-base metres, `/track/lost`; scenarios with a ground
   truth and OAK-D-like errors (latency 60-110 ms, depth noise, frame
   jitter, drops) -- walk across, stand still, hand wave, a jump to another
   person, a one-frame outlier, lost 2 s, lost for good, in and out of the
   zone, out of reach.
2. The tracking layer (tracking.py), mode A (gaze): gate (speed, confidence,
   timeout), One Euro filter, the target clamped to the box, J1 / J5
   offsets bounded, Ruckig on the offsets, hold then back when lost, every
   tick's pose checked against the room (else no further). An offline
   harness (track_eval.py): each scenario on a greet clip at 125 Hz ->
   joint velocity / acceleration / jerk against the limits, clearance,
   gaze error, time to engage and to let go; pass / fail.
3. Isaac (scripts/isaac/run_tracking.py): the same, physics on, scenarios
   headless with a report and video; a window with a draggable target.
4. Modes B / C (follow left-right, the hand up-down): an early exit from the
   clip (transitions.exit_from / ramp_stop), a checked move to the track
   pose; then SimMachine, then the real arm.
5. OSC in from TD / the OAK-D process, the same format.
**Success Criteria**: every scenario passes: joint velocity, acceleration
and jerk within their share of the limits, no step at engage / lost / back,
clear of the room by the margins; the gaze error small while tracked.
**Tests**: track_sim / tracking / track_eval self-tests; the scenario report.
**Status**: In Progress -- 1 and 2 done (2026-09-29): all 11 scenarios pass
offline (no unsafe tick, clear 59 mm, offsets within the share, the arm
within 0.83 of its limits, lost -> back to the clip). What it took: aim
from the hub's pose, not the moving clip's (chasing the clip made the gaze
worse); look ahead 0.6 s along the clip (greet_04_tilt closes the
forearm-strip gap to 11 mm and the offsets were too late to leave it);
a slow-zone governor. The gaze gain is modest while the gestures swing
the tool (walk 34 -> 32 deg, still 13 -> 17 deg): a behaviour matter
(look clips while tracked), not safety. Cost 1.1 ms a tick mean, 8.7 ms
at most on a check tick: for show_stream's 125 Hz, the checks go to a
worker thread (stage 5).
3 done (2026-09-29): every scenario played in Isaac with physics and the
tool: no contact in 11, the simulated arm within 0.93 deg of the commands
(the greet clip's own fastest moment); videos from the audience's side in
geo/tracking/. Live (2026-09-29, Ruckig in Isaac's Python): run_tracking --live, a
ball to drag as the head, TargetInput + Gaze every physics tick in real time;
headless check (the ball walking, gone 3 s): 72 s, 0 unsafe ticks, 0 contacts,
tracking within 1.0 deg, faster than real time.
Crowds (2026-09-29): tracking.Attention picks whom to look at from /track/people
(in view 1 s, not walking by, one at a time 5-12 s, turns, groups as one);
track_sim's five crowd scenarios; all 16 scenarios pass offline (the crowd showed:
speed over 0.6 s, look-ahead to 1.4 s, the offsets held still near a slow zone);
Isaac: crowd replays and --live --people 3 (the one looked at green), 0 unsafe,
0 contacts. Next: 4 (modes B / C); the OAK-D process sending /track/people.
4 (2026-09-29, the user: the spot straight in front of greet, 0.6 m across; the clip
stops and the arm attends; C on simulated hands first; 30 s at most; the audience must
see that the mode began): scripts/engage.py.
  4a geometry: the spot, a box around the greet hub's tool point, 25 x 15 x 6 cm
     (along -0.15..+0.10, up -0.12..+0.03, towards 0..0.06: what the room leaves the
     upright 1 m strip), the aim within 30 deg of straight and -20..+25 deg of level;
     432 poses reachable, 20 mm more than the margins, one branch. Done.
  4b the state machine: enter (a checked, planned move from the clip; perk up, then
     face them), track (B: along with the person, C: up with a raised hand), leave /
     lost / 30 s: a nod, back to the hub, the clips go on; hysteresis at the spot's
     edge, a cooldown; /robot/state TRACK for TD's own light.
     Done: engage.Engage + ClipPlayer; its self-test (enter after 1 s, faces them,
     B follows a sway, C rises to a hand, goodbye 0.7 s after they step off, 30 s cap,
     a passer-by and the edge do not trigger, 24 % of the joints' speed, every pose
     clear). The first run chose another IK branch for the perk-up and the planned
     move's check refused it (through the ceiling): IK from the pose sent, 90 deg refused.
  4d Isaac live: run_tracking --live --engage (the spot as a ring that lights up, the
     head and a pink hand to drag); headless 72 s: two engagements, 0 refused, 0 unsafe,
     0 contacts, faster than real time. Done.
  On hold (the user, 2026-09-29): the interactive mode stays a test on its own
  (engage.py, run_tracking --live --engage), apart from the party show; the user
  looks at it in Isaac, then decides on a real-arm test of it alone. Not started
  until then: TD's light for TRACK, show_stream's Runner, the OAK-D process.
  Real people (2026-09-29): CMU mocap scenes (mocap_cmu.py, mocap_scenes.py,
  the `mc_` scenarios) with the Femto Mega's noise and latency. A person
  waiting reads 0.7 m/s over 0.6 s (sway), so the walking speed is taken
  over 1.2 s (0.6 s while someone is new); the glass band keeps whoever is
  in it until 0.15 m beyond (depth noise at the edge). All 22 pass.
  TD -> Isaac / SimMachine (2026-09-29): TD's people_track (TD-ROBOT-UVSCAN)
  plays the Femto sim CSVs as /track/hands + /track/people to 9010 (Isaac,
  run_tracking --live --osc-in) and 9011 (SimMachine, track_test.py: the
  show_stream loop and guard). Both run track_runner.Runner, the one live
  loop (clips, gaze, interactive mode); its self-test runs the mocap scenes
  through it. It found what neither test alone had: the slow zone's hold
  undid the gaze's retreat 3 ticks in 4, so a clip after an engagement
  took the strip 47 mm from the glass (margin 50) -- the retreat now wins
  (a `trouble` flag; the speed still governed). Isaac's summary counts
  every gaze (it is renewed after each engagement).

## Stage 11b: Sim and the real Femto through one door (2026-09-29, the user: "能切换模拟和真机数据")
**Goal**: the simulation is the Femto's own output, so the real camera
replaces it with one switch: TD's Kinect Azure CHOP (Orbbec, USB) and the
sim give the same channels -- `frame`, `timestamp`, `pN/id`,
`pN/<32 k4abt joints>:tx/ty/tz/confidence` (read from TD itself; an empty
slot has id 0) -- and everything after reads only those.
**Stages**:
1. `femto_format.py` (this repo): the channel layout, a recording as CSV
   (a row a frame, a column a channel), mocap -> all 32 joints with the
   Femto's noise and confidence, a recording -> /track/ events (offline
   tests on real recordings too). mocap_scenes writes the sims in it.
2. TD people_track: Source Sim (a recording played by a Script CHOP) /
   Femto (Kinect Azure TOP + CHOP) -> Switch -> `bodies` -> the logic ->
   OSC Out DATs (Isaac 9010, SimMachine 9011) and the view; Record writes
   `bodies` in the same CSV. The extrinsic from CHOP space to the robot,
   solved on site from 3+ known floor marks (Kabsch): the CHOP's axis
   convention, which TD does not document, does not matter.
3. The checks: TD screenshot; TD -> Isaac and TD -> SimMachine again.
**Status**: Complete (2026-09-29). femto_format.py (the layout read from
TD's own Kinect Azure CHOP; a pure-Python Kabsch that tries both
handednesses and keeps the one with heads above feet); mocap_scenes writes
all 32 joints with the tracker's confidence (a hidden hand only
predicted), and its offline scenarios go through femto_format too;
`rec:<csv>` plays any recording offline (a sim recording gives exactly its
scene's events). TD people_track in /project1/tracking_test, Active off by
default: File In DAT -> DAT to CHOP -> Trim (a Speed CHOP clock) | Kinect
Azure TOP (Orbbec, DirectML) + CHOP -> Switch -> `bodies` -> Select
`joints` -> a CHOP Execute on frame/timestamp (only on new data: ~30 Hz)
-> OSC Out DATs, tables written whole and only when changed -> DAT to
CHOP -> instanced Geometry COMPs, the layout by DAT to SOP, an ortho
camera, a Render TOP. Calibrate on the simulated calibration recording
came back within 2.6 cm of the sim's placement. TD -> Isaac (engage,
1 min): 1798 frames, 4 engagements, 0 unsafe, 0 contacts.

## Stage 12: The scan top to bottom, a floor-standing canvas (2026-09-29, the user's go)
**Goal**: The scan runs down the paper, the 1 m strip laid level, over a
canvas the fab team makes much larger than the scan, standing on the
floor on the old scan's plane; a preview to approve before the library is
made again.
**Success Criteria**: show.py scans any of four directions over
`scan.area` (a part of the paper, centred on it); a scan-only build
(`show.py build --scan-only`, geo/show/<show>_scan, refused as a show)
plays in Isaac with no contact; the user approves the video; then
party.json takes the candidate's canvas and scan, the library is built
again and the TD pixel map reads rows (u down the image, the LEDs across).
**Tests**: show.py self-test: top to bottom over scan.area, the strip
level (roll 90), an unknown direction refused, a scan-only build refused
as a show; the showpiece trigger (runner, show_ui).
**Status**: In Progress
1 done: shows/party_vscan.json, the candidate (party.json untouched, the
real PC's show still plays). The paper plane 1.097 m from J1's axis; the
area 1.0 x 1.0 m, 0.5-1.5 m high, centred on J1's axis's foot on it. Its
top is the controller's work zone (Z 1600 mm, a WebUI safety setting)
less the ramp and lead; the level strip reaches down to 0.2 m. The canvas
2.4 x 1.8 m on the floor: a PLACEHOLDER until the fab team's size.
2 done: scan 6.1 s at 0.2 m/s, to_scan 5.3 s, from_scan 5.8 s; Isaac
(isaac/record_segments.py): no contact, 0.56 deg; the video
geo/isaac/party_vscan_segments.mp4. From the audience's middle the
forearm stands in front of the area's middle while it scans.
The recording lit and coloured after the user's photo of the lab
(isaac/room_look.py: the LED frame under the joists, a warm fill through
the glass, plank floor, plywood TV wall, joists, studded glass walls --
visual only, the frame's place estimated); cameras: the whole room from
the corner behind the robot, and the guests' side from behind the
audience zone.
Also: a showpiece is a trigger by its name (show_ui's buttons per show,
e.g. party_bigwipe's low_wipe_rows / greet_wipe_cols), for a quick look.
3 (2026-09-29, the user approved the preview): party.json took the
candidate's canvas and scan (the candidate files removed); the library
built again in 148 s: 82 of 85 segments identical, only to_scan, scan,
from_scan new -- the floor-standing canvas cost no idle clip; dry run 10
min, 12 scans, worst step 1.19 of 1.44 deg a tick. TD rows next.
Open: the canvas's real size (fab team); party_bigwipe.json still has the
old framed canvas (rebuild it when it is next wanted).

## Stage 13: The LED strip's cable along the arm (2026-09-29, the user's go)
**Goal**: Where to clip the strip's cable to the arm, how much cable each
span needs, and which motions would strain, fling or pinch it; the cable
runs from the strip to the laptop on the red cart (the user).
**Success Criteria**: 1. a geometric survey (scripts/cable_route.py) over
every motion of the built shows: per span between clips the chord's range
(the slack loop that swings) and its longest (the cable it needs); the
best layouts with 2-5 clips; the motions that set the extremes; a video of
the chosen layout's spans in Isaac. 2. the cable as a chain of capsules in
Isaac (PhysX) on the chosen clips, the worst motions and the library
played: no pinch, no contact with the strip, the room or the canvas;
videos.
**Tests**: cable_route self-test: a clip on a joint's axis keeps its span
constant while only that joint turns, one off it does not; the layout
search takes the axis clips; a chord through a link is caught.
**Status**: In Progress

## Stage 14: TouchDesigner's idle LEDs, previewed live in Isaac (2026-09-29, the user's go)
**Goal**: TD lights the strip in greet (docs/td_idle_leds_and_isaac_link.md):
layers driven by the show (breath over the clip, the arm's speed, the
beat, facing the guests), a per-family table, capped, gated to greet +
IDLE, the scan always first. Designed in TD, live: Isaac plays the show and
sends TD its status; TD's LEDs come back over Art-Net and are drawn on the
strip in real time (the user: no renders while tuning).
**Steps**: 1. the show's OSC carries what the LEDs play from (/robot/family,
/action, /clip_t, /clip_len, /beat, /bpm_now, /facing). 2. run_show
--osc-out, --artnet (artnet.py, isaac/led_viz.py: 60 glowing LEDs), real
time, TD's STOP holds. 3. TD: `idle_leds` (inputs, layers, table, cap,
gate) and a second DMX Out to Isaac. 4. TD's canvas preview on the paper, live (canvas_link.py, isaac/canvas_viz.py,
run_show --canvas). 5. offline: the canvas model and pixel_scan's image in Python for
headless videos. 6. TD in a video: run_show --record keeps the arm, the status, TD's LEDs
and canvas as they played live (td_capture.py); replay_render.py renders it at leisure, any camera.
**Success Criteria**: TD leaves OFFLINE with Isaac as the player; the LEDs
TD sends show on Isaac's strip within a frame; dark outside greet + IDLE.
**Tests**: show self-test (beat at the played tempo, wall-time clip_t,
facing 1 / 0, greet faces the guests more than rest, the OSC list);
artnet self-test (ArtDmx parse, levels, the receiver's newest packet);
led_viz self-test (60 along the strip, LED 0's end); an Isaac snapshot
with a fake Art-Net sender (geo/isaac/leds_live_test.png).
**Status**: In Progress (steps 1-6 done; the idle LEDs play wherever /robot/paper < 0.001; tuning with the user)

## Stage 15: The stream's lag held over a day (2026-09-29, from 11 real-arm runs)
**Goal**: The lag between command and arm (~105-143 ms with the -950 ppm
calibration) stays put over a whole day. 1. play_ui.py's save no longer
drops what it does not own from playback.toml ([controller_playback_ppm]:
without it the PC's clock paces, +55 ms/min). 2. show_stream.py measures
the lag as it runs (a delay-locked loop on the feedback, target: the first
minute after a warm-up) and, at a rest only (the same pose as the 25
points before it, never in the scan), leaves one rest point out (lag >10 ms
too long) or sends it twice (too short), at most one every 5 s. On for
--hardware, --no-lag-correction turns it off; no feedback, no correction.
**Success Criteria**: a play_ui save keeps [controller_playback_ppm] word
for word; a long run's lag stays within its target +- 15 ms where it grew
before; the motion itself is unchanged; the report counts the corrections
(lag_correction: drops, repeats, corrected_ms, the ticks).
**Tests**: play_ui self-test (foreign tables and keys kept, show_stream
reads -950 after a save, a second save changes nothing); show_stream
self-test (the meter on made-up feedback: target, lag now, drop, repeat,
one per gap; a fake controller 1000 ppm slow over 30 s: the lag grows
without, held with; the poses tick for tick the same, only rest ticks
missing; every correction at a rest; no feedback: off).
**Status**: Complete (on the robot PC: check playback.toml has the section;
a 1-2 h run to see the lag held)

## Stage 16: the library reviewed in Isaac (2026-09-29, plan A)
**Goal**: The show's clip library as the Houdini review showed it
(render_clip_review.py: one clip after another, its metadata burnt in, a
contact sheet), in Isaac's lab room with physics on:
scripts/isaac/record_library.py plays every idle clip of party, the two
big wipes of party_bigwipe (low_wipe_rows, greet_wipe_cols) and the scan,
a cut to each clip's first pose, the card held 0.6 s.
**Success Criteria**: per camera (room, audience) one mp4 with each
clip's card in the corner (name, length, n/N; hub, family / intent,
measured action; bpm made and played, energy, intensity; peak TCP speed,
heights, the room's clearance, the wrist's share; the simulation's
tracking and arm-room contacts) and a labelled contact sheet; a JSON of
chapters (where each clip starts) with the tracking and contacts per
clip; the lab look, the guides hidden, lights and cameras as they are.
Out: geo/isaac/review/ (not in git).
**Tests**: overlay self-test (clip_card for a gesture, a dance, a
showpiece, the scan; lines within 52 characters, no ASS braces; block pads
the lines to one column); record_library self-test (steps, frames and the
poster frame per clip; the playlist: every idle clip, the big wipes, the
scan last; the sheet's caption).
**Status**: Complete (73 clips, 698 s of video a camera, both cameras in
1064 s: geo/isaac/review/party_library_{room,audience}.mp4, _sheet.jpg,
party_library.json; tracking worst 0.96 deg (low_14_pop), no arm-room
contact in any clip)

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
