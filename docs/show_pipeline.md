# Show pipeline: a library, a state machine, a digital twin

2026-09-27. How the arm plays through a clip library under outside control
(TouchDesigner), what lives in Python and what in Houdini, and which
standard formats hold each piece. First show: the Volvox party. An LED strip
held by the arm sweeps UV paper on a trigger, and the arm idles through
generated motion otherwise.

## The shape: a motion graph with hubs

Every motion starts and ends **at rest at a hub pose**. Motion graphs in
games and Boston Dynamics' Choreographer do the same. Any two motions then
join without a jump, and all checking happens before the show.

```
home --idle clip--> home                (the generated library)
home --to_scan--> scan_start --scan--> scan_end --from_scan--> home
```

- **Idle clips** come from the library (`tests/csv/stage`), turned to face the
  stage. Each is checked against the room and the controller's work area, and
  kept `idle_canvas_m` away from the paper, so idling can never touch it.
- **The scan** is the one fixed motion. It is allowed near the paper
  (`scan_canvas_m`). For now it is a placeholder straight sweep, until the
  paper and the strip are decided.
- **Joining moves** (`to_scan`, `from_scan`) are planned once by
  `safe_move.route`, which detours round the ceiling and keeps the TCP in the
  work area. They are timed by `retime_topp` and checked like any clip.
- **The scan is a design input to the library.** The paper is an obstacle
  with a margin for every idle clip, and the hubs sit where the move to the
  scan is short and clear. When the paper moves, rebuild:
  `python scripts/show.py build shows/party.json`.

## Deciding what plays: the state machine

`scripts/show.py` `Runner`. The same code runs on a dry-run clock, in Isaac
Sim, and (next) on SimMachine and the real FR20.

```
IDLE --(trigger, at the end of the running clip)--> TO_SCAN -> SCAN -> FROM_SCAN -> IDLE
IDLE --(pause)--> PAUSED at home;  any --(fault)--> FAULT, holds until reset
```

- **Choosing the next idle clip.** Weighted random, with no repeat within the
  last 8. Weights lean towards a **mood** (a Laban action) and an **energy**
  (0 calm .. 1 lively), which TouchDesigner can send.
- **Latency.** A trigger waits for the running clip to end; the longest idle
  clip sets the worst case (18 s now). Later, Ruckig will make an early
  smooth exit to the nearest hub possible.
- **TouchDesigner link (OSC).**
  - In: `/robot/trigger`, `/pause`, `/resume`, `/reset`, `/mood`, `/energy`.
  - Out: `/robot/state`, `/clip`, `/progress`, `/joints`, and `/robot/scan`
    (0..1 while scanning). The last one is the LED strip's column clock: TD
    lights the column for the arm's position, so the image does not depend
    on the arm's timing.

Dry run, 20 minutes (2026-09-27):
- 178 clips played, including 31 scans; all 24 idle clips used.
- Largest joint step per 8 ms tick: 0.96 deg (the limit is 1.44).
- Trigger to scan: 7.5 s mean, 16.3 s max.

## Playing it on the arm: one stream

`scripts/show_stream.py` plays the show on the FR20, or on SimMachine, as
**one continuous ServoJ stream at 125 Hz**. It sends the Runner's joints
tick by tick. There is no CSV per run and no stop between clips.

- **Timing.** The stream runs on an absolute clock, as the single-clip
  player does. When a tick is late, the Runner advances by the ticks
  missed and only the newest pose is sent. Feedback comes on a separate
  connection.
- **Checks on every tick.** Before a pose is sent, its joint step must be
  within the velocity limits times the speed, and every joint within its
  limits. The feedback thread polls the controller's error code. Either
  failure is a FAULT: StopMotion, then ServoMoveEnd.
- **Speed.** `--speed` scales the show's clock. Hardware defaults to 0.3.
- **Start.** A MoveJ to the start hub through the checked route.
- **End.** At the end of `--minutes` the running clip finishes at a hub,
  at rest.
- **Stop.** Ctrl+C or OSC `/robot/stop` is a software stop.
- **TouchDesigner.** `--osc` puts the same OSC contract on the stream.
  Triggers are queued and applied between ticks.
- **Sampling.** Segments are sampled with PCHIP, the player's
  shape-preserving cubic. A 24 fps clip streamed at 125 Hz then has no
  velocity step at each sample.

Compared with the other ways to drive the arm:
- **CSV playback** (`fairino_player`) is one clip at a time and stops
  between clips.
- **Controller programs** (WebUI Lua) cannot react to TouchDesigner in
  time.
- **The stream** is the same Runner that the dry run and Isaac use, so
  what was simulated is what plays.

## Hubs, zones, stage and range

Four spatial ideas, each with one job:

| Term | What it is | What it does |
|---|---|---|
| **Stage** | The work zone box (the env's `stage`, or the show's override) | A hard limit. Every clip and move keeps the TCP inside it (inset 3 cm). |
| **Zone** | A named box: `audience`, `greet`, `idle`, ... | Soft design space. A gesture hub's clips perform inside its zone, and look targets are picked in `audience`. A zone does not reject motion. |
| **Hub** | A rest pose that clips start and end at | The graph's node. It is either **joints** (`q`), or **tool + look**: where the tool tip is and what it looks at, with the pose solved near a seed. |
| **Range** | J1 sector, TCP height band, speed fraction | A show-wide limit. A clip outside the J1 sector or the height band is dropped. Clips are made at `speed` times the robot's limits. |

So a hub lives **inside** the stage (checked), usually **inside** its zone
(designed), and the range narrows both for the whole show.

## Editing the show in Houdini

Two scenes, two jobs:
- `scenes/FR20_rig.hiplc` is the **hand tool**. Draw a curve, solve it,
  and export a joint CSV.
- `scenes/FR20_show.hiplc` is the **show**. It holds one node,
  `/obj/robot_show` (`scripts/show_rig.py`), and every control is on that
  node's parameter page.

Rebuild the show scene with `hython scripts/build_show_scene.py`, or put
the tool into any open scene with `import show_rig; show_rig.install()`.

| Tab | What is there |
|---|---|
| Setup | Robot Profile (profiles/*.json), Environment (envs/*.json), Show Config (shows/*.json); Load / Write / Check; **Edit in Viewport**: pick a zone or hub, press Handles On |
| Zones | Zones as a multiparm: name, centre, size, yaw (robot frame, metres). "Use Zone 'stage' as the Stage" replaces the environment's work zone |
| Hubs | Start hub; hubs as a multiparm: name, mode (Tool Tip + Look At, or Joint Angles), tool tip, look target, seed pose, clips, generator, zone, gesture families; Keep the Solved Pose as Seed |
| Operating Range | J1 sector, tool tip height band, speed |
| Library | clip length, bars, BPM, intensity, seed, clips per hub visit, no repeat; the authored clip (a joint CSV at a hub) |
| Build and Preview | Build Show, Dry Run; play a built segment on the arm, or hold it at a hub; the report |
| Display | robot, room, zones, hub ghosts, look rays, built paths, range |

- **Edit in Viewport**: the handles work on the item picked in Setup.
  - A zone gets a transform handle: move it, turn it about the vertical,
    scale it.
  - A hub gets two translate handles: its tool tip and its look target.
  - The handles write the parameters. The pose is solved again at once,
    and a hub that cannot be used turns red.
- **Write refuses** a config that names something missing, such as a
  sequence at a deleted hub, and says why. Keys the node does not show
  are kept, for example OSC, sequences, canvas and scan.

Inside the node, Python makes data only. The shapes come from Houdini's
own nodes:
- **the robot**: the `robot_arm` asset, which also draws the room;
- **zones**: a Box, drawn with Convert Line, copied onto one point per
  zone;
- **hub ghosts**: the arm's own link meshes, posed per hub by For-Each
  and Transform Pieces;
- **the range**: Circle SOPs that read the node's parameters;
- **rays and paths**: PolyWire.

**The room** is drawn the same way as in the Isaac scene (`room_geom`
looks, shared with `env_to_usd`):
- the floor and the objects are solid, facing outward;
- each wall is one face turned into the room, so with the viewport's
  Remove Backfaces on (set on load), the near walls vanish;
- zones are outlines.

## Python or Houdini

The rule: **what must run without Houdini, or be unit-tested, is Python.
What is spatial, visual or designed by hand is Houdini.** Houdini calls the
same Python, so nothing is written twice.

| Piece | Where | Why |
|---|---|---|
| IK, timing (TOPP), collision / safety checks, safe moves | Python (`ur_ik`, `retime_topp`, `collision`, `safe_move`) | Runs on the lab PC, in Isaac, in tests; one truth for "is this safe" |
| Player, controller I/O, show state machine, OSC | Python (`fairino_player`, `show`) | The runtime: real time, no DCC |
| Measuring the room (probe, scan alignment) | Python tools (`probe_ui`, `scan_to_env`) | At the robot, no Houdini there |
| **Room assembly and editing** | **Houdini Solaris / USD** | A layout people see and change: scan + measured planes + paper + zones |
| **Capability atlas, placement** | **Houdini VDB** (reshape / erode, Scatter by density) | Fields are volumes; placement is scattering into them; you see and paint them |
| **Path families** (lines, Lissajous, spirals, drawn curves like the cat) | **Houdini curves / VEX** | Shapes are designed and art-directed |
| **Hubs and the motion graph** | **Houdini points and edges** (hubs as points, moves as edges, coloured by clearance) | Hubs are placed by hand, near the scan, and read at a glance |
| Library curation (keep / drop, tags, mood) | **Houdini** attributes + the PDG review | A person decides what is in the show |
| Clip sequencing preview, music timing | **Houdini** KineFX MotionClip / CHOPs | Timing felt, not computed |
| Batch generation, review renders | **Houdini PDG** | Already the case |

## Standard formats

- **Robot description: URDF is the source.** It is the ROS / MoveIt
  standard: kinematics, limits, inertia, meshes. Isaac Sim imports it to a
  USD articulation. Never hand-edit a derived USD robot.
- **Robot data beside it:** `profiles/fr20.json`. Measured limits (velocity,
  accelerations of 300/600 deg/s^2), HOME, the flange offset.
- **Tool / payload** (the LED strip): its mass and centre of mass go in the
  controller's tool load (WebApp), and its shape goes into the URDF as a
  tool link, so collision checks and the sim see it. Not modelled yet: the
  next safety item once the strip is chosen.
- **Environment: USD is the target.** Isaac, Omniverse, Houdini Solaris and
  RoomPlan (USDZ) all speak it.
  - Today the JSON room (`envs/volvox_lab.json`) is the source and
    `scripts/env_to_usd.py` writes `envs/volvox_lab.usda`.
  - Every prim carries its role and margin (`motionlab:role`,
    `motionlab:margin_m`); obstacles carry UsdPhysics collision.
  - Later: author the room in Solaris and derive the JSON for the checker.
- **Materials:**
  - visual: UsdPreviewSurface (MDL in Isaac if needed);
  - physical: link masses and inertias from the URDF.
- **Motion:**
  - clip JSON (`motionlab.clip/1`, JointTrajectory-shaped) is the source;
  - CSV feeds the controller;
  - the compiled show (`shows/*.compiled.json`) is what a runtime loads.

## Simulation, in layers

Each layer answers a different question. Every clip and show goes through
them in order.

1. **Kinematic checks** (Python, milliseconds, every clip). Reach, joint
   limits, TOPP timing, capsule collision against the room and the work
   area.
2. **Show dry run** (Python). The state machine on a clock: continuity,
   latency, variety.
3. **Physics twin** (Isaac Sim, `scripts/isaac/run_show.py`). The same
   Runner drives the articulation's drives under gravity. It reports:
   tracking, arm-room contacts (PhysX), and the full show loop with triggers
   from the panel, keys, OSC (TouchDesigner) or an auto-trigger. Later it
   adds simulated people and an OAK-D camera for the interactive part.
4. **Controller twin** (SimMachine). The real controller software: timing,
   errors, safety settings.
5. **Real robot.** Speed 0.3 first, filmed, logged to `docs/results.md`.

Status: layers 1–2 run. Layer 3 is written but blocked: the installed Isaac
Sim 5.0 release candidate crashes at start on this machine's driver (610.47,
CUDA 13.3), in Warp's CUDA init, before any project code. It needs a newer
Isaac Sim. Layers 4–5 need a streaming backend for the Runner (the player
plays one CSV today).
