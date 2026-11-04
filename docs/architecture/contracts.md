# Contracts: Robot, Perception, Motion, Director

Four boundaries, each written down so that one side can be replaced without
reading the other: a different arm (UF850, xArm), a different camera
(OAK-D), a different library, a different way of choosing what plays next.
The coupling they cut is measured in [coupling.md](coupling.md); the decisions
behind them are in [docs/adr/](../adr/). The order in which they could reach the running show is
[migration.md](migration.md). The interfaces are in
`scripts/contracts/` as `typing.Protocol`s with a reference implementation
each and a conformance suite (`python scripts/contracts/conformance.py`).

**Nothing in this page changes how the show runs today.** The contracts
describe what the code already does where it does it (the FR20 backend, the
Femto's messages, the clip schema, `show.Selector`), name it, and say what a
replacement must also do. Where the code does not yet meet its contract, it
says so under *Gaps*.

Each contract states:

- **State owner** — who holds the state on each side, so nobody else writes it.
- **Failure modes** — for each: how it is **Detected**, how far it is
  **Contained** (what keeps moving, what stops), how it is **Recovered**, and
  how it is **Reported** (to the operator, to the log).
- **Versioning** — what may change without a version bump, and what may not.

```
          ┌──────────── Perception ────────────┐
camera ──▶│ SceneState (robot base frame)      │──▶ tracking / engage (gaze offsets, attention)
          └──────────────┬─────────────────────┘
                         │ summarize()
                         ▼
          ┌──────────── Director ──────────────┐
          │ DirectorInput ──▶ Intent (enum)    │──▶ GuardedDirector ──▶ show.Runner (owns execution)
          └────────────────────────────────────┘                          │
          ┌──────────── Motion ────────────────┐                          │
show cfg ▶│ arm-neutral intent ─build→ per-arm │──▶ compiled show ────────┤
          └────────────────────────────────────┘                          ▼
          ┌──────────── Robot ─────────────────┐                   show_stream.stream
profile ─▶│ profile + RobotBackend             │◀───────── servo_j, 125 Hz, Guard
          └────────────────────────────────────┘
```

---

## 1. Robot = profile + RobotBackend

**What the arm is** lives in `profiles/<id>.json`; **how to move it** lives in a
backend. Neither knows about shows.

### 1.1 The profile (schema_version 1, as it stands)

| Key | Required | Read by | Notes |
|---|---|---|---|
| `id`, `label`, `schema_version` | yes | everything | `id` must equal the file name |
| `robot.num_joints` | yes | `_check_profile` | every per-joint list is checked against it |
| `robot.limits_deg` | yes | HDA, player, stream `Guard`, generators | **robot frame**, the datasheet's |
| `robot.max_velocity_deg_s` | yes | everything that times motion | one number or one per joint |
| `robot.max_acceleration_deg_s2` | no (player falls back to 300) | Retime, Pre-Flight, player, generators | say in a `_comment` whether it was **measured** on the arm (`accel_ui.py`) |
| `robot.max_jerk_deg_s3` | no | `motion_clip.measure`, Ruckig | |
| `robot.home_deg` | no for the HDA, **yes for the show** | choreo, cell, `--goto-home` | |
| `rig.rotation_axis`, `sign`, `offset_deg`, `fbik_enforces_limits` | yes | HDA, `robot_profile` frame conversion | the three frames: configurejoints / extracted / robot |
| `rig.fbx` or `rig.urdf` | one of | HDA | **the Python side needs `urdf`**: collision capsules, TCP, IK, Isaac |
| `rig.flange_offset_m` | yes | TCP everywhere | |
| `rig.ik_solver` | no (`fbik`) | HDA; the generators use `ur_ik` regardless (gap) | `ur_closed_form` only for UR-type arms (`ur_ik.analyse`) |
| `tool` | no | `collision.load_model`, `motion_limits` (cable J6), strip geometry | see Gap R3 |
| `output.writer`, `speed_cap_pct`, … | yes | HDA CSV export | |
| `presets` | yes for the HDA | Solve tab | |

`python scripts/contracts/robot.py` prints what each profile has against what
the full show needs (today: FR20 complete; UF850 missing `home_deg`, `rig.urdf`,
`tool`, and with a non-measured, non-per-joint envelope).

**Proposed for schema_version 2** (not applied; each item removes a hard-coded
FR20 fact listed in coupling.md §A4/§B1):

```jsonc
"kinematics": {
  "singular": [{"joint": 5, "at_deg": 0.0, "near_deg": 5.0}],   // show.wrist_singular_report, show_stream.recover
  "reach_m": 1.6, "shoulder_m": [0.0, 0.0, 0.215]                // gestures.REACH_M, gestures.SHOULDER
},
"backend": {
  "kind": "fairino_xmlrpc",                // which RobotBackend implementation
  "servo_rate_hz": 125,
  "simulator_hosts": ["192.168.116.0/24"], // fairino_player.SIM_NET
  "faults": {"singular_pose_main": 10}     // show_stream.SINGULAR_POSE
}
// and "tool" moves out of the arm profile into the cell / show (Gap R3)
```

### 1.2 RobotBackend (`scripts/contracts/robot.py`)

The ten methods `show_stream.stream`, `Link`, `recover`, `goto` and
`move_checked` already call on `fairino_player.Controller` — named, not
changed ([ADR 002](../adr/002-robotbackend-mirrors-the-existing-controller.md)):

| Method | Semantics |
|---|---|
| `model() -> str` | contains the profile id (case-insensitive); `fairino_player.main` refuses otherwise |
| `joints() -> [deg]` | actual, robot frame |
| `error_code() -> [ret, main, sub]` | `[0, 0, 0]` = no fault |
| `reset_errors()` | clear a recoverable fault |
| `prepare()` | whatever precedes a planned move (FR20: auto mode, enable) |
| `move_to(q, vel_pct, tool=0, wobj=0)` | controller-planned joint move; returns once taken (callers wait on `joints()`); raises `RuntimeError` saying why when refused |
| `servo_start()` / `servo_j(q, cmd_t, cmd_id) -> 0` / `servo_end()` | the stream; `cmd_id` rises; non-zero return is a fault |
| `stop()` | stop now; must work **from the feedback connection** while the sender is blocked |
| optional `queue_length()`, `frames()`, `diagnose()` | used when present |

A backend is opened as a **BackendFactory**: `command()` (the sender: blocking,
no socket timeout) and `feedback()` (reads and the watchdog's stop: calls time
out). `BackendInfo` says the kind, the profile, the servo rate, whether it is a
simulator, and its `FaultPolicy` (which main code is the recoverable
singular-pose stop).

Reference implementations: `fairino_player.Controller` (the real one; passes
the structural check), `scripts/contracts/sim_backend.py` (`SimBackend`, pure
kinematics, drives `show_stream.stream` in the conformance suite).

### 1.3 State owner

| State | Owner | Others |
|---|---|---|
| The arm's actual joints, its fault, its mode | **the controller** | the backend reads them; nobody caches them as truth |
| What is commanded (segment, time in it) | **`show.Runner`** | the stream asks it each tick |
| Limits, geometry, home, envelope | **the profile file** (whoever measured) | read-only at runtime; a build records its digest (`info.inputs.profile`) |
| Per-controller clock (`playback_ppm`) | **`playback.toml`**, measured per controller | |
| The connection | the backend instance | one thread each |

### 1.4 Failure modes

| Failure | Detect | Contain | Recover | Report |
|---|---|---|---|---|
| Controller fault mid-stream (collision, protective stop) | `Link` polls `error_code()` every 0.1 s; `servo_j` returns non-zero | `stop()`, `servo_end()`, Runner → FAULT (holds the pose) | operator reset only — except the stop `FaultPolicy` names (FR20 singular pose): `recover()` resets, MoveJ on to the hub, `Runner.restart_at`, ≤ 3 a run, 30 min apart | stream report `fault`, events, `/robot/fault` |
| A command that would jump or leave the limits | `Guard.check` before every send | refused, never sent; FAULT | operator | report names the joint and step |
| Sender hung (a send that never returns) | feedback thread sees no heartbeat for 0.5 s | `stop()` from the feedback connection | operator | `["sender", …]` |
| Feedback lost | no answer for 1 s | FAULT | operator | `["feedback", …]` |
| Wrong arm behind the address | `model()` lacks the profile id | refuse before any motion | fix the profile / IP | message |
| Malformed profile | `robot_profile._check_profile` at load | nothing starts | fix the file | the failing key |
| Profile *values* wrong (envelope not measured, URDF zero ≠ controller zero) | not by code: `robot.py` readiness marks `partial`; `fairino_player --check` compares FK with the URDF | hardware defaults to `--speed 0.3`, MoveJ at 10 % | measure (`accel_ui.py`), compare FK on the vendor simulator | readiness table |

### 1.5 Versioning

- Profile: `schema_version` integer. Adding an optional key with a safe
  default is not a bump. Renaming a key, changing its unit or frame, or making
  an optional key required **is** (v2 above). `_check_profile` should refuse an
  unknown major version (Gap R4).
- RobotBackend: `robot/1` = the ten methods. Optional methods may be added
  (callers use `hasattr`). A new required method is `robot/2` and needs an
  adapter for every existing backend in the same change.

### 1.6 Gaps

- **R1** `"fr20"` hard-coded in 111 code sites (coupling.md §A1); a second
  profile cannot be built or checked by the Python side until they read it from the caller.
- **R2** The Python generators solve with `ur_ik` only (coupling.md §A3).
- **R3** The tool (UV bar, cable range) is part of the FR20 profile.
- **R4** No schema-version gate in `robot_profile.load`.
- **R5** FR20 fault codes and simulator subnet live in `show_stream` / `fairino_player`, not in `BackendInfo`.

---

## 2. Perception = SceneState in the robot base frame

### 2.1 The schema (`motionlab.scene/1`, `scripts/contracts/perception.py`)

A frame is the people (each one's **head**) and their **hands** (each body's
higher wrist), every point in the **robot base frame** — the URDF's
`base_link`, Z up, metres — with a confidence and the time it was measured:

```jsonc
{
  "schema": "motionlab.scene/1",
  "frame": "robot_base",
  "t": 12.533,                       // measured, s, the source's timeline
  "received": 12.62,                 // arrival on the consumer's clock (null: unknown)
  "source": "femto_recording:femto_rec_20260930-150731.csv",
  "extrinsic": "femto_extrinsic.json@e7a2e8f6344f",
  "people": [{"id": 3, "x": 0.12, "y": -2.0, "z": 1.61, "conf": 0.9}],
  "hands":  [{"id": 3, "x": 0.31, "y": -2.0, "z": 1.88, "conf": 0.9}]
}
```

Rules (`perception.validate`): ids are ints > 0, unique per list per frame,
stable across frames for the same person; every hand belongs to a person in
`people`; `conf` in [0, 1] (`< 0.5` = predicted, not seen — `tracking.MIN_CONF`);
positions finite and in metres. **On the wire it is the existing OSC**:
`/track/hands t n (id x y z conf)*n`, then `/track/people` likewise
(`SceneState.to_osc()` equals `track_replay.osc_frame`, checked in its self-test).

### 2.2 Extrinsic

A producer owns the camera → base transform `q = R c + p` and applies it
before it emits anything ([ADR 003](../adr/003-scenestate-in-the-robot-base-frame.md)).
Today it is `{"R": 3x3, "p": 3}` beside a recording (`femto_extrinsic.json`),
solved on site by `femto_format.solve_extrinsic`: Kabsch on 3+ floor marks,
both handednesses tried (the CHOP's is undocumented), the one with heads above
feet kept, the RMS returned. A SceneState names the extrinsic it used, so a log
says which calibration produced it. Proposed (not applied):
`motionlab.extrinsic/1 = {R, p, rms_m, solved_at, camera, marks}`, so a stale
or poor calibration is visible from the file alone.

### 2.3 Reference implementation

`scripts/femto_format.py` is the reference producer: the Kinect Azure CHOP's
channel layout in, `messages()` → (people, hands) in the base frame out, plus
`solve_extrinsic`. `scripts/contracts/recorded_perception.py` wraps it (via
`track_replay.frames`) as a `PerceptionSource`; `perception.OscPerception`
wraps the live door (`track_osc.TrackIn`). A new camera writes one producer —
for an OAK-D, a DepthAI process emitting the same OSC — and passes
`perception_suite`.

`PerceptionSource`: `info()` (frame, kind, rate, extrinsic id),
`poll(now) -> SceneState | None` (the newest frame arrived by `now`, **never
blocking**), `close()`.

### 2.4 State owner

| State | Owner |
|---|---|
| Body ids, the skeleton, the extrinsic | **the producer** (TD people_track, an OAK-D process) |
| Filtering, outlier rejection, who is "the target" | **the consumer**: `tracking.TargetInput` (speed gate, One Euro, switch frames), `tracking.Attention` |
| The Director's view | `perception.summarize()` — derived, no state |

### 2.5 Failure modes

| Failure | Detect | Contain | Recover | Report |
|---|---|---|---|---|
| No frames (camera, TD or network down) | `TargetInput`: nothing for `LOST_AFTER_S` (0.35 s); `SceneSummary.stale` after 1 s | gaze offsets held `HOLD_S`, then back to the clip (Ruckig, bounded); a Director sees a stale, empty room | automatic when frames return | status, decision log `input.scene.stale` |
| One-frame glitch / depth outlier | faster than `MAX_SPEED_MPS` | not taken | taken if `SWITCH_FRAMES` agree (another person) | — |
| Wrong or stale extrinsic (camera bumped) | not by the stream: heads below the floor, people inside the wall; the solve's RMS | tracking moves only bounded offsets (`MAX_OFFSET_DEG`), each collision-checked (`Gaze._clear`) | recalibrate (people_track Calibrate) | the extrinsic id in every SceneState and decision |
| Latency, clock skew | `received - t` | `TargetInput` predicts nothing; Ruckig limits the response | — | `SourceInfo.latency_s`, scene `age_s` |
| Malformed message | `validate()` (contract tests); `track_osc` drops what does not parse | the frame is dropped | next frame | — |

### 2.6 Versioning

`scene/1`: fields may be **added** (consumers ignore what they do not read);
`frame` never changes (a camera-frame point is a contract violation, not a
variant). The OSC argument layout of `/track/people` and `/track/hands` is
frozen; richer data (a skeleton, a gaze vector) arrives as a **new** address.
`/track/target` + `/track/lost` (the 09-28 OAK-D proposal) is a second shape
for the same information; retire it or define it as `people` of length ≤ 1
(Gap P1).

### 2.7 Gaps

- **P1** Two message families (`/track/people` vs `/track/target`).
- **P2** `track_osc.people_of` drops the frame's `t`; `OscPerception` stamps arrival time instead.
- **P3** Extrinsic files are found by the name `femto_extrinsic.json`.
- **P4** "Hands" need a body tracker; an OAK-D pipeline without one must say `hands = []` (and `hands_up` will read 0).

---

## 3. Motion = what is the show's, what is one arm's solution

### 3.1 The split

The clip (`motionlab.clip/1`, `scripts/motion_clip.py`) and the compiled show
(`motionlab.show.compiled/2`, `scripts/show.py`) mix two kinds of data. The
contract separates them by what happens on an arm change:

| Field | Kind | On a new arm |
|---|---|---|
| show config: `hubs.*.tcp`, `hubs.*.look`, `zones`, `stage`, `canvas`, `scan` (speed, passes, accel, direction), `select`, `sequences`, `families`, hub `clips` / `generator` | **arm-neutral** — the show's intent | kept |
| generator parameters: choreo specs (Laban action, bars, bpm), gesture / path family parameters, showpiece specs | **arm-neutral** | kept |
| clip `labels`: `action`, `effort`, `intent`, `family`, `bpm`, `direction` | arm-neutral (what it is meant to be) | kept |
| clip `labels.energy`, `labels.stats`, `clearance_m` | **measured on one arm's motion** | re-measured |
| clip `points[].q`, `t` (as retimed), `safety.*`, compiled `segments[].q/t`, `hubs` (joint poses) | **one arm's solution** | **re-solved**, re-checked |
| show config `hubs.*.q` (rest), `hubs.*.near` (IK seeds), `range.j1_deg` | **one arm's** (today in the show config) | re-chosen (Gap M2) |
| clip `tcp` | arm-neutral *as a path*, but computed from one arm's FK + flange | recomputed |
| `robot`, `joint_names`, `units` | identifies which arm's solution it is | |

Rule ([ADR 005](../adr/005-clips-are-solved-per-arm.md)): **solutions are build
products.** A clip for the UF850 is made by running the same generators with
the same arm-neutral parameters against the UF850 profile, then checking it in
the room — never by mapping FR20 joint angles onto UF850 joints.

### 3.2 State owner

| State | Owner |
|---|---|
| `shows/<name>.json` | the show designer (by hand or the Houdini show scene) |
| `shows/<name>.compiled.json` | **`show.py build` only**; never hand-edited; records the digests of its inputs (`info.inputs`: config, profile, tool, room, robot URDF+meshes) |
| The clip library / manifest | `clip_factory` / `motion_clip.write_manifest` |

### 3.3 Failure modes

| Failure | Detect | Contain | Recover | Report |
|---|---|---|---|---|
| A generated motion this arm cannot do (limits, envelope, collision, range) | at build: `motion_clip.measure`, `collision.check`, `show.out_of_range` | dropped from the library (`info.dropped`) | change parameters, rebuild | build log, `info.warnings` |
| A route missing between hubs | `Graph.gaps()` at build; `Runner._plan_next` | build refuses; the Runner refuses a trigger with no route | rebuild | log line |
| Compiled show stale against its inputs | `require_fresh` (digests) | the stream refuses to start | `show.py build` | message naming what changed |
| **Compiled show for a different arm** than the one connected | not detected today (Gap M1) | — | — | — |
| A clip that passed the build but faults on the arm (singularity) | controller | Robot contract §1.4 | `recover` where allowed | stream report |

### 3.4 Versioning

Schema strings (`motionlab.clip/1`, `motionlab.show.compiled/2`, `motionlab.show/3`)
bump on a breaking change. Labels may be added freely. Units are fixed
(deg, s, m). A compiled show is never migrated: it is rebuilt.

### 3.5 Gaps

- **M1** The compiled show records the profile's digest, not its id; the stream should refuse a compiled show whose `robot` ≠ the backend's profile.
- **M2** Joint-space values in the show config (`rest.q`, `near`, `range.j1_deg`) — move them to a per-arm overlay (`shows/<name>.<profile>.json`) or derive them.
- **M3** `motion_clip` writes `"robot": "fr20"` from several generators regardless of the profile used.

---

## 4. Director = enum-limited intent; the Runner executes

### 4.1 Interface (`scripts/contracts/director.py`)

`Director.decide(DirectorInput) -> Intent`, called each time the Runner needs
the next idle clip (`Selector.pick`).

**Input** (`motionlab.director_input/1`):

| Part | Contents |
|---|---|
| `scene` | `SceneSummary`: `n_people`, `n_predicted`, `nearest {id, dist_m, bearing_deg, height_m}`, `hands_up`, `ids`, `age_s`, `stale` — **no skeletons, no images** |
| `library` | `LibraryIndex`: every idle clip `{id, hub, family, action, energy, duration_s, tags}`, the triggers the show takes, a digest |
| `show` | `ShowState`: `clock`, `hub`, `state`, `sequence`, `mood` (asked), `operator_mood`, `operator_energy`, `energy_target` (the arc's or the operator's), `recent`, `recent_picks`, `pending` |
| `candidates` | the idle clip ids at the current hub — the only ones `PLAY_CLIP` may name |

**Output** (`motionlab.intent/1`) — one of four kinds, nothing else parses
(unknown kinds or fields, and fields of the wrong type — `clip` / `action` /
`family` / `trigger` / `reason` strings, `energy` a number, not a bool; a
number sent as a string is taken):

| Kind | Fields | Carried out as |
|---|---|---|
| `play_clip` | `clip` | that clip, if it is in `candidates` |
| `play_like` | `action` / `family` / `energy` (0..1) | the Selector's weighted pick restricted to matching clips at this hub |
| `trigger` | `trigger` ∈ `library.triggers` (scan, a sequence, a showpiece) | `Runner.trigger(name)` — routed and refused exactly as an operator's `/robot/trigger`, after the `TriggerPolicy` (§4.6); this pick itself is the Selector's |
| `defer` | — | the Selector picks |

Every kind may carry `reason` (≤ 200 chars) for the log. No joint angles,
poses, speeds, timings or routes: whatever a Director names was built and
checked before the show began ([ADR 001](../adr/001-director-outputs-intent-not-joints.md)).

### 4.2 The Runner keeps the final say

`GuardedDirector` runs the Director on a worker thread with a budget
(default 50 ms) and falls back to the existing `show.Selector` when the
outcome's `fallback` is set — callers check that, not the intent (an answer
refused as invalid is never returned; it is kept in `outcome["rejected"]` for
the log):

| Fallback | When |
|---|---|
| `timeout` | no answer within the budget (the call is left to finish; until it does, the next calls are `busy` — calls never pile up) |
| `error` | the Director raised |
| `invalid` | the answer does not parse as an Intent, names a field its kind does not take, a clip not at this hub, a trigger the show does not have, an energy outside [0, 1], or a `play_like` nothing matches; or the `TriggerPolicy` refuses it (a clip within `no_repeat`, a second Director scan, a trigger inside its cooldown) |
| `busy` | the previous call is still running |
| `defer` | the Director said so |

`DirectorSelector` is a `show.Selector` subclass that asks the guard first, so
it drops into `show.Runner` unchanged (`director.runner_with(graph, director)`).
`BaselineDirector` wraps `show.Selector` itself; through `DirectorSelector` it
reproduces `show.Selector`'s picks exactly (conformance: 3 seeds × 15 min with
triggers and operator mood / energy changes, identical segment histories)
([ADR 004](../adr/004-runner-keeps-authority-director-falls-back.md)).

**Live use is not wired yet, on purpose.** `Selector.pick` runs inside
`Runner.step`, i.e. on `show_stream`'s 125 Hz tick thread. A synchronous call
with a 50 ms budget there would cost six ticks. For the dry clock, replay and
Isaac it is fine; for the stream the Director must be asked **ahead of time**
(during the running clip — the Runner knows `time_left`) and `pick()` must only
read a ready answer or fall back in microseconds. That is [migration step 6](migration.md).

### 4.3 State owner

| State | Owner |
|---|---|
| Queue, hub, pending triggers, sequence, state | **`show.Runner`** — a Director never writes them; it can only ask (`trigger`) |
| Recent picks, mood, energy, the random stream | the `Selector` (`DirectorSelector` keeps them for the fallback) |
| Whatever a Director remembers | the Director; it must tolerate being skipped (fallbacks are not announced to it — it sees `recent` next time) |
| The decision log | append-only; written by `DirectorSelector`, never read by the show |

### 4.4 Failure modes

| Failure | Detect | Contain | Recover | Report |
|---|---|---|---|---|
| Slow (an LLM, a network) | budget | fallback `timeout`, then `busy` | next pick after it returns | `latency_ms`, `fallback.reason` |
| Crash / exception | worker future | fallback `error` | next pick | `validation.errors` |
| Hallucinated or out-of-contract answer | `Intent.from_dict`, `validate_intent` | fallback `invalid` | next pick | the raw answer (≤ 500 chars) |
| Trigger spam, repeats | `TriggerPolicy` in `DirectorSelector` | refused as `invalid`: one Director scan a run, 120 s per trigger name, `play_clip` within `no_repeat` (conformance: a Director asking for a trigger / the last clip at every pick, 30 min: the paper exposed once, no repeat played) | next pick | `validation.errors` |
| Valid but poor taste (dull, off-energy) | **not by the guard**: `decision_log.metrics` (`same_family_twice`, `energy_abs_error_mean`, variety) | every clip is a built, checked clip | replace the Director | the log and its metrics |
| Wrong scene (stale perception) | `scene.stale` in the input | the Director's problem; the baseline ignores the scene | — | logged with every decision |

What the Director **cannot** cause, whatever it returns: a motion that was not
built and checked, a route the graph does not have, a speed other than the
built one, a pose outside the limits (the stream's `Guard` is untouched).

### 4.5 Versioning

`intent/1`, `director_input/1`, `decision/1`. Adding a field to the input is
minor (Directors ignore what they do not know). Adding an `IntentKind` is
**major**: an older guard refuses an unknown kind as `invalid`, which is the
safe direction, but the kind does nothing until the guard is updated. Removing
or reinterpreting a kind is major.

### 4.6 Policy defaults (`TriggerPolicy`)

Settled in review as the safe defaults, applied by `DirectorSelector` to the
Director's answers only (the operator's `/robot/trigger` is unchanged):

- **`scan_per_run = 1`** — a Director may start at most one scan a run.
  `Runner.trigger` only refuses a scan while one runs or waits, so without
  this a Director asking at every pick would re-expose the paper after every
  scan (show.py: "the paper is exposed once").
- **`cooldown_s = 120`** — show seconds between two Director triggers of the
  same name (sequences, showpieces).
- **`enforce_no_repeat = True`** — a `play_clip` of a clip within the last
  `no_repeat` is refused, as the Selector's own pool excludes it, unless every
  clip at the hub is that recent.

Changing a default is a decision for the show, recorded here.

Still open:

- `GO_HUB` (change hub now) is deliberately absent: `Runner` has no public
  method for it; it would need one, and it is a larger motion than any clip.
